// 采集、推理、结果和原图四条工作循环，以及它们的启动和停止。
// 参数、ROS 话题和最终发布在 detect_node.cpp。

#include "node/detect_node.hpp"

#include <interface_pkg/app/core.hpp>
#include <algorithm>
#include <atomic>
#include <cmath>
#include <exception>
#include <filesystem>
#include <limits>
#include <stdexcept>
#include <thread>
#include <utility>
#include <vector>

#include "yolo/yolo.hpp"

// 连续读取失败次数是既有重连规则；打开失败另走可被停止通知打断的重试等待。
constexpr int kMaxReadFailures = 30;

// ---- Runtime：线程启动、停止与唤醒 ----

void DetectNode::Runtime::clearPendingFrames()
{
  inference_slot_.clear();
  raw_slot_.clear();
  result_slot_.clear();
}

void DetectNode::Runtime::start()
{
  // 启动失败也先唤醒并回收已启动线程，再关闭相机并向上抛出；会话不可重启。
  std::vector<WorkerGroup::Task> tasks{
    [this] {captureLoop();}, [this] {inferenceLoop();}, [this] {resultLoop();}};
  if (owner_.raw_pub_) {tasks.emplace_back([this] {rawLoop();});}
  try {
    workers_.start(std::move(tasks), [this] {wakeWaiters();});
  } catch (...) {
    stop();
    throw;
  }
}

void DetectNode::Runtime::stop()
{
  workers_.stop([this] {wakeWaiters();});
  // 摄像头由采集线程独占；先回收该线程，再关闭相机。
  // 已进入的同步 open/read 仍须由后端返回，唤醒邮箱和重试等待不等于能强行中断驱动调用。
  if (camera_) {camera_->close();}
}

void DetectNode::Runtime::wakeWaiters()
{
  // 停止通知与最后一组发布串行；返回后不再有已通过检查的发布越过停止边界。
  // 这里只取消和唤醒，离开发布锁后 WorkerGroup 才 join，随后所有者关闭相机。
  auto publication = stage_.lockPublication();
  if (camera_) {camera_->requestStop();}
  inference_slot_.stop();
  raw_slot_.stop();
  result_slot_.stop();
}

// ---- Runtime：采集、推理与发布循环 ----

void DetectNode::Runtime::captureLoop()
{
  int consecutive_failures = 0;

  while (workers_.running()) {
    if (!camera_->isOpen()) {
      const bool opened = camera_->open();
      // open() 是同步后端调用；它返回后若已停止，不再发起探测后的下一次读取。
      if (!workers_.running()) {break;}
      if (!opened) {
        const std::string detail = camera_->lastError();
        RCLCPP_ERROR_THROTTLE(owner_.get_logger(), *owner_.get_clock(), 5000,
                              "cannot open %s%s%s; retrying",
                              owner_.camera_config_.device.c_str(),
                              detail.empty() ? "" : ": ",
                              detail.c_str());
        workers_.waitFor(std::chrono::seconds(2));
        continue;
      }
      RCLCPP_INFO(owner_.get_logger(), "camera open: %s", camera_->description().c_str());
      logLetterboxScale();
      consecutive_failures = 0;
    }

    CameraFrame camera_frame;
    if (!workers_.running()) {break;}
    const bool received = camera_->read(camera_frame);
    if (!workers_.running()) {break;}
    if (!received) {
      if (!camera_->lastError().empty()) {
        RCLCPP_ERROR_THROTTLE(owner_.get_logger(), *owner_.get_clock(), 5000, "%s",
                              camera_->lastError().c_str());
      }
      if (++consecutive_failures >= kMaxReadFailures || camera_->reconnectRequired()) {
        RCLCPP_WARN(owner_.get_logger(), "%s stopped delivering frames; reopening",
                    owner_.camera_config_.device.c_str());
        // 与最终发布共用闸门：重连之后，旧会话中已取走的帧和结果也不能再发布。
        stage_.resetCamera([this] {
            clearPendingFrames();
          });
        camera_->close();
        camera_reopens_.fetch_add(1);
        consecutive_failures = 0;
        workers_.waitFor(std::chrono::seconds(1));
      } else {
        workers_.waitFor(std::chrono::milliseconds(20));
      }
      continue;
    }
    consecutive_failures = 0;

    auto frame = std::make_shared<Frame>();
    frame->image = std::move(camera_frame.image);
    frame->nvmm = std::move(camera_frame.nvmm);
    // 用 camera_frame.at 的 steady_clock 间隔回推 ROS 源时间；该时刻由各后端定义，不是曝光时间。
    // 只扣除该时刻之后的耗时，GStreamer 管线内解码不计入；无法回推时沿用当前 ROS 时间。
    {
      const rclcpp::Time stamp_now = owner_.now();
      const auto decode_delay = std::chrono::steady_clock::now() - camera_frame.at;
      const auto delay_ns =
        std::chrono::duration_cast<std::chrono::nanoseconds>(decode_delay).count();
      frame->stamp = (camera_frame.at.time_since_epoch().count() != 0 && delay_ns > 0 &&
        delay_ns < stamp_now.nanoseconds()) ?
        stamp_now - rclcpp::Duration::from_nanoseconds(delay_ns) : stamp_now;
    }
    frame->sequence = sequence_.fetch_add(1);
    frame->camera_session = stage_.cameraSession();
    captured_frames_.fetch_add(1);

    // 两个最新值邮箱共享只读帧；各自覆盖未消费帧，图像、源时间与相机会话始终随帧交接。
    // 阶段检查、赋代次与入槽同持采集锁，防止切阶段清槽后旧帧重新入槽。
    stage_.capture(frame->stamp.nanoseconds(), [&](std::uint64_t generation) {
        frame->generation = generation;
        if (owner_.raw_pub_ && owner_.raw_pub_->get_subscription_count() > 0) {
          raw_slot_.set(frame);
        }
        inference_slot_.set(frame);
      });
  }
}

void DetectNode::Runtime::inferenceLoop()
{
  FrameRateLimiter limits[2];
  std::uint64_t seen_generation = 0;
  std::uint64_t seen_camera_session = 0;
  cv::Size field_frame_size;
  while (workers_.running()) {
    auto frame = inference_slot_.take();
    if (!frame) {break;}
    if (frame->generation != stage_.generation()) {continue;}

    // 场地状态只由推理线程复位，采集/服务线程仅使代次失效，避免并发改动定位器。
    if (seen_generation != frame->generation || seen_camera_session != frame->camera_session) {
      limits[0].reset();
      limits[1].reset();
      seen_generation = frame->generation;
      seen_camera_session = frame->camera_session;
      detector_->resetField();
      field_frame_size = {};
    }

    const int active = stage_.active();
    if (active != 1 && active != 2) {continue;}
    const int task = active == 1 ? 0 : 1;
    const double limit = task == 0 ? seg_max_fps_.load() : field_max_fps_.load();
    if (!limits[task].allow(std::chrono::steady_clock::now(), limit)) {
      (task == 0 ? seg_skipped_ : field_skipped_).fetch_add(1);
      continue;
    }
    if (!owner_.frameIsCurrent(*frame, active)) {continue;}

    try {
      if (task == 0) {
        const bool want_mask = owner_.seg_mask_pub_ &&
          owner_.seg_mask_pub_->get_subscription_count() > 0;
        // 后端执行可能跨越切阶段；算完再次检查，最终 ROS 发布还会在发布锁内复核。
        auto result = detector_->segment(frame->image, want_mask, frame->nvmm.get());
        frame->nvmm.reset();
        if (!owner_.frameIsCurrent(*frame, active)) {continue;}
        result_slot_.set(std::make_shared<CompletedFrame>(
            CompletedFrame{frame, std::move(result)}));
      } else {
        frame->nvmm.reset();
        if (field_frame_size != frame->image.size()) {
          detector_->resetField();
          field_frame_size = frame->image.size();
        }
        auto result = detector_->field(frame->image);
        if (!owner_.frameIsCurrent(*frame, active)) {continue;}
        result_slot_.set(std::make_shared<CompletedFrame>(
            CompletedFrame{frame, std::move(result)}));
      }
    } catch (const vision::BackendError &) {
      throw;  // 后端失败由 WorkerGroup 关闭会话，禁止在损坏的 GPU 状态上继续交付。
    } catch (const std::exception & error) {
      RCLCPP_ERROR_THROTTLE(owner_.get_logger(), *owner_.get_clock(), 5000,
        "%s frame %lu failed: %s", task == 0 ? "seg" : "field",
        static_cast<unsigned long>(frame->sequence), error.what());
      // 这一帧改发空结果，下游不必一直拿着上一条有效目标。
      if (frame && owner_.frameIsCurrent(*frame, active)) {
        if (task == 0) {
          result_slot_.set(std::make_shared<CompletedFrame>(CompletedFrame{frame, SegRun{}}));
        } else {
          result_slot_.set(std::make_shared<CompletedFrame>(CompletedFrame{frame, FieldResult{}}));
        }
      }
    }
  }
}

void DetectNode::Runtime::resultLoop()
{
  while (workers_.running()) {
    auto completed = result_slot_.take();
    if (!completed) {break;}
    const Frame & frame = *completed->frame;
    // 提前过滤避免复制无效大图，publishSeg/publishField 仍保留最终加锁复核。
    if (frame.generation != stage_.generation()) {continue;}
    try {
      if (const auto * seg = std::get_if<SegRun>(&completed->result)) {
        owner_.publishSeg(frame, *seg);
      } else {
        owner_.publishField(frame, std::get<FieldResult>(completed->result));
      }
    } catch (const std::exception & error) {
      RCLCPP_ERROR_THROTTLE(owner_.get_logger(), *owner_.get_clock(), 5000,
        "result publication failed: %s", error.what());
    }
  }
}

void DetectNode::Runtime::rawLoop()
{
  std::uint64_t generation = 0, session = 0;
  while (workers_.running()) {
    auto frame = raw_slot_.take();
    if (!frame) {break;}
    if (generation != frame->generation || session != frame->camera_session) {
      raw_limiter_.reset();
      generation = frame->generation;
      session = frame->camera_session;
    }
    try {
      owner_.publishRaw(*frame);
    } catch (const std::exception & error) {
      RCLCPP_ERROR_THROTTLE(owner_.get_logger(), *owner_.get_clock(), 5000,
                           "raw image publication failed: %s", error.what());
    }
  }
}

void DetectNode::Runtime::logLetterboxScale()
{
  const cv::Size frame = camera_->size();
  if (frame.width <= 0 || frame.height <= 0) {return;}

  auto describe = [&](const char * what, const cv::Size & input) {
      if (input.width <= 0) {return;}
      const float scale = std::min(static_cast<float>(input.height) / frame.height,
                                static_cast<float>(input.width) / frame.width);
      RCLCPP_INFO(owner_.get_logger(), "letterbox %s: %dx%d -> %dx%d, scale %.4f%s", what,
                frame.width, frame.height, input.width, input.height, scale,
                std::abs(scale - 1.0f) < 1e-4 ? " (1:1, no resampling)" : "");
    };
  describe("seg", detector_->segInput());

}
