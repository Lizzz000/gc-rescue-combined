// Copyright 2026 jetson_yolo integration
// SPDX-License-Identifier: Apache-2.0

#include "runtime/detect_runtime.hpp"
#include "yolo/yolo.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <exception>
#include <utility>
#include <vector>

namespace
{
constexpr int kMaxReadFailures = 30;
}

DetectRuntime::DetectRuntime(Ports ports)
: ports_(std::move(ports))
{
}

DetectRuntime::~DetectRuntime()
{
  stop();
}

void DetectRuntime::setCamera(const CameraConfig & config, double raw_max_fps)
{
  camera_ = std::make_unique<UsbCamera>(config);
  raw_max_fps_ = raw_max_fps;
}

void DetectRuntime::setDetector(std::unique_ptr<Detector> detector)
{
  detector_ = std::move(detector);
}

void DetectRuntime::setRates(double seg, double field)
{
  seg_max_fps_.store(seg);
  field_max_fps_.store(field);
}

std::pair<double, double> DetectRuntime::rates() const
{
  return {seg_max_fps_.load(), field_max_fps_.load()};
}

bool DetectRuntime::hasSeg() const
{
  return detector_ && detector_->hasSeg();
}

bool DetectRuntime::hasField() const
{
  return detector_ && detector_->hasField();
}

bool DetectRuntime::segActive() const
{
  return (stage_.active() & 1) != 0 && hasSeg();
}

bool DetectRuntime::fieldActive() const
{
  return (stage_.active() & 2) != 0 && hasField();
}

bool DetectRuntime::supportsTask(int mask) const
{
  return !detector_ || (((mask & 1) == 0 || hasSeg()) && ((mask & 2) == 0 || hasField()));
}

const FieldConfig & DetectRuntime::fieldConfig() const
{
  return detector_->config().field;
}

void DetectRuntime::setTask(int mask)
{
  stage_.transition(mask, [this] {return ports_.source_now();}, [this] {
      clearPendingFrames();
    }, [this, mask](std::int64_t stamp_ns, std::uint64_t epoch) {
      ports_.publish_task(mask, stamp_ns, epoch);
    });
}

void DetectRuntime::refreshTask()
{
  stage_.refresh([this] {return ports_.source_now();}, [this] {
      clearPendingFrames();
    }, [this](int mask, std::int64_t stamp_ns, std::uint64_t epoch) {
      ports_.publish_task(mask, stamp_ns, epoch);
    });
}

bool DetectRuntime::frameIsCurrent(const DetectFrame & frame, int task_bit) const
{
  if (!workers_.running() || !stage_.current(frame.generation, frame.camera_session)) {
    return false;
  }
  if (task_bit == 1) {return segActive();}
  if (task_bit == 2) {return fieldActive();}
  return frame.stamp_ns >= stage_.cutoff();
}

DetectRuntime::Status DetectRuntime::sampleStatus()
{
  return {
    capture_window_.rate(captured_frames_.load()), seg_window_.rate(seg_meter_.count()),
    field_window_.rate(field_meter_.count()), raw_window_.rate(raw_published_.load()),
    seg_meter_.latencyMs(), field_meter_.latencyMs(),
    seg_gpu_ms_.load(), seg_post_ms_.load(), field_post_ms_.load(),
    inference_slot_.overwritten(), raw_slot_.overwritten(), result_slot_.overwritten(),
    camera_reopens_.load(), seg_skipped_.load(), field_skipped_.load(),
    segActive(), fieldActive()};
}

// ---- 工作线程与采集 ----

void DetectRuntime::clearPendingFrames()
{
  inference_slot_.clear();
  raw_slot_.clear();
  result_slot_.clear();
}

void DetectRuntime::start(bool raw_enabled)
{
  // 启动失败也先唤醒并回收已启动线程，再关闭相机并向上抛出；会话不可重启。
  std::vector<WorkerGroup::Task> tasks{
    [this] {captureLoop();}, [this] {inferenceLoop();}, [this] {resultLoop();}};
  if (raw_enabled) {tasks.emplace_back([this] {rawLoop();});}
  try {
    workers_.start(std::move(tasks), [this] {wakeWaiters();});
  } catch (...) {
    stop();
    throw;
  }
}

void DetectRuntime::stop()
{
  workers_.stop([this] {wakeWaiters();});
  // 摄像头由采集线程独占；先回收该线程，再关闭相机。
  // 已进入的同步 open/read 仍须由后端返回，唤醒邮箱和重试等待不等于能强行中断驱动调用。
  if (camera_) {camera_->close();}
}

void DetectRuntime::wakeWaiters()
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

void DetectRuntime::captureLoop()
{
  int consecutive_failures = 0;

  while (workers_.running()) {
    if (!camera_->isOpen()) {
      const bool opened = camera_->open();
      // open() 是同步后端调用；它返回后若已停止，不再发起探测后的下一次读取。
      if (!workers_.running()) {break;}
      if (!opened) {
        const std::string detail = camera_->lastError();
        ports_.log(Log::OpenFailed, "cannot open " + camera_->config().device +
          (detail.empty() ? "" : ": " + detail) + "; retrying");
        workers_.waitFor(std::chrono::seconds(2));
        continue;
      }
      ports_.log(Log::Info, "camera open: " + camera_->description());
      logLetterboxScale();
      consecutive_failures = 0;
    }

    CameraFrame camera_frame;
    if (!workers_.running()) {break;}
    const bool received = camera_->read(camera_frame);
    if (!workers_.running()) {break;}
    if (!received) {
      if (!camera_->lastError().empty()) {
        ports_.log(Log::ReadFailed, camera_->lastError());
      }
      if (++consecutive_failures >= kMaxReadFailures || camera_->reconnectRequired()) {
        ports_.log(Log::Reopening, camera_->config().device + " stopped delivering frames; reopening");
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

    auto frame = std::make_shared<DetectFrame>();
    frame->image = std::move(camera_frame.image);
    frame->nvmm = std::move(camera_frame.nvmm);
    // 用 camera_frame.at 的 steady_clock 间隔回推 ROS 源时间；该时刻由各后端定义，不是曝光时间。
    // 只扣除该时刻之后的耗时，GStreamer 管线内解码不计入；无法回推时沿用当前 ROS 时间。
    {
      const auto stamp_now = ports_.source_now();
      const auto decode_delay = std::chrono::steady_clock::now() - camera_frame.at;
      const auto delay_ns =
        std::chrono::duration_cast<std::chrono::nanoseconds>(decode_delay).count();
      frame->stamp_ns = (camera_frame.at.time_since_epoch().count() != 0 && delay_ns > 0 &&
        delay_ns < stamp_now) ?
        stamp_now - delay_ns : stamp_now;
    }
    frame->sequence = sequence_++;
    frame->camera_session = stage_.cameraSession();
    captured_frames_.fetch_add(1);

    // 两个最新值邮箱共享只读帧；各自覆盖未消费帧，图像、源时间与相机会话始终随帧交接。
    // 阶段检查、赋代次与入槽同持采集锁，防止切阶段清槽后旧帧重新入槽。
    stage_.capture(frame->stamp_ns, [&](std::uint64_t generation) {
        frame->generation = generation;
        if (ports_.wants_raw()) {
          raw_slot_.set(frame);
        }
        inference_slot_.set(frame);
      });
  }
}

void DetectRuntime::inferenceLoop()
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
    if (!frameIsCurrent(*frame, active)) {continue;}

    try {
      if (task == 0) {
        const bool want_mask = ports_.wants_mask();
        // 后端执行可能跨越切阶段；算完再次检查，最终 ROS 发布还会在发布锁内复核。
        auto result = detector_->segment(frame->image, want_mask, frame->nvmm.get());
        frame->nvmm.reset();
        if (!frameIsCurrent(*frame, active)) {continue;}
        result_slot_.set(std::make_shared<CompletedFrame>(
            CompletedFrame{frame, std::move(result)}));
      } else {
        frame->nvmm.reset();
        if (field_frame_size != frame->image.size()) {
          detector_->resetField();
          field_frame_size = frame->image.size();
        }
        auto result = detector_->field(frame->image);
        if (!frameIsCurrent(*frame, active)) {continue;}
        result_slot_.set(std::make_shared<CompletedFrame>(
            CompletedFrame{frame, std::move(result)}));
      }
    } catch (const vision::BackendError &) {
      throw;  // 后端失败由 WorkerGroup 关闭会话，禁止在损坏的 GPU 状态上继续交付。
    } catch (const std::exception & error) {
      ports_.log(Log::InferenceFailed, std::string(task == 0 ? "seg" : "field") +
        " frame " + std::to_string(frame->sequence) + " failed: " + error.what());
      // 这一帧改发空结果，下游不必一直拿着上一条有效目标。
      if (frame && frameIsCurrent(*frame, active)) {
        if (task == 0) {
          result_slot_.set(std::make_shared<CompletedFrame>(CompletedFrame{frame, SegRun{}}));
        } else {
          result_slot_.set(std::make_shared<CompletedFrame>(CompletedFrame{frame, FieldResult{}}));
        }
      }
    }
  }
}

void DetectRuntime::resultLoop()
{
  while (workers_.running()) {
    auto completed = result_slot_.take();
    if (!completed) {break;}
    const auto & frame = *completed->frame;
    if (frame.generation != stage_.generation()) {continue;}
    try {
      if (const auto * seg = std::get_if<SegRun>(&completed->result)) {
        if (frameIsCurrent(frame, 1) && ports_.publish_seg(frame, *seg)) {
          seg_meter_.record(seg->inference_ms);
          seg_gpu_ms_.store(seg->gpu_ms);
          seg_post_ms_.store(seg->post_ms);
        }
      } else {
        const auto & field = std::get<FieldResult>(completed->result);
        if (frameIsCurrent(frame, 2) && ports_.publish_field(frame, field)) {
          field_meter_.record(field.ms_total);
          field_post_ms_.store(field.ms_warp + field.ms_split);
        }
      }
    } catch (const std::exception & error) {
      ports_.log(Log::ResultPublishFailed, std::string("result publication failed: ") + error.what());
    }
  }
}

void DetectRuntime::rawLoop()
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
      if (ports_.wants_raw() && frameIsCurrent(*frame, 0) &&
        raw_limiter_.allow(std::chrono::steady_clock::now(), raw_max_fps_))
      {
        ports_.publish_raw(*frame);
      }
    } catch (const std::exception & error) {
      ports_.log(Log::RawPublishFailed, std::string("raw image publication failed: ") + error.what());
    }
  }
}

void DetectRuntime::logLetterboxScale()
{
  const cv::Size frame = camera_->size();
  if (frame.width <= 0 || frame.height <= 0) {return;}
  const cv::Size input = detector_->segInput();
  if (input.width <= 0) {return;}
  const float scale = std::min(static_cast<float>(input.height) / frame.height,
      static_cast<float>(input.width) / frame.width);
  char message[128];
  std::snprintf(message, sizeof(message), "letterbox seg: %dx%d -> %dx%d, scale %.4f%s",
    frame.width, frame.height, input.width, input.height, scale,
    std::abs(scale - 1.0f) < 1e-4 ? " (1:1, no resampling)" : "");
  ports_.log(Log::Info, message);
}
