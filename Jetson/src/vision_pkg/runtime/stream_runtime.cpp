// Copyright 2026 jetson_yolo integration
// SPDX-License-Identifier: Apache-2.0

#include "runtime/stream_runtime.hpp"
#include "node/ros_convert.hpp"
#include "stream/mjpeg/mjpeg.hpp"
#include "stream/opencv/overlay.hpp"

#include <algorithm>
#include <cstdio>
#include <utility>

namespace {
using ms_d = std::chrono::duration<double, std::milli>;
}  // namespace

StreamRuntime::StreamRuntime(StreamRuntimeConfig config, Ports ports)
: config_(std::move(config)), ports_(std::move(ports))
{
  // 关闭两路 JPEG 时不申请硬件，校验在 ROS 接口和线程启动前完成。
  if (config_.stream_enabled || config_.publish_compressed) {
    jpeg_encoder_.setBackend(config_.jpeg_backend);
  }
  // 标签依附于检测框；只等待配置启用的图层。
  synchronizer_.setRequirements({config_.draw_boxes, config_.draw_masks, config_.draw_field});
}

StreamRuntime::~StreamRuntime()
{
  stop();
}

void StreamRuntime::start()
{
  try {
    startServer();
    workers_.start({[this] {composeLoop();}}, [this] {wakeWaiters();});
  } catch (...) {
    stop();
    throw;
  }
}

void StreamRuntime::startServer()
{
  if (!config_.stream_enabled) {return;}

  MjpegServer::Options options;
  options.address = config_.stream_address;
  options.port = static_cast<int>(config_.stream_port);
  options.quality = static_cast<int>(config_.stream_quality);
  options.jpeg_backend = config_.jpeg_backend;
  options.max_fps = config_.stream_max_fps;
  options.scale = config_.stream_scale;
  options.frame_timeout_s = config_.input_timeout_s;

  // 绑定失败仅关闭 HTTP，ROS 图像输出仍可运行。
  streamer_ = std::make_unique<MjpegServer>(options);
  std::string error;
  const bool started = streamer_->start(error);
  ports_.http_started(started, error);
}

void StreamRuntime::rethrowIfFailed() const
{
  workers_.rethrowIfFailed();
}

std::exception_ptr StreamRuntime::takeHttpFailure()
{
  return streamer_ ? streamer_->takeFailure() : std::exception_ptr{};
}

void StreamRuntime::stop()
{
  // 先唤醒并回收合成线程，再停止服务，避免线程继续向已关闭的服务提交画面。
  workers_.stop([this] {wakeWaiters();});
  if (streamer_) {streamer_->stop();}
}

void StreamRuntime::wakeWaiters()
{
  // 与合成等待的条件锁配对，避免错过停机通知。
  std::lock_guard<std::mutex> lock(overlay_mutex_);
  frame_slot_.stop();
  // 停止后会话不可重启；撤销当前 HTTP 代次，让已越过入口检查的编码也无法最终提交。
  // reset 不等待编码锁，仍在 join 后才停止服务并释放资源。
  if (streamer_) {streamer_->reset(synchronizer_.generation() + 1);}
  overlay_cv_.notify_all();
}

// ---- 输入入队与阶段切换 ----

void StreamRuntime::acceptFrame(
  vision_ros::ImageViewConstPtr view, std::int64_t stamp, std::int64_t current,
  Clock::time_point received_at)
{
  // 接收前的源帧年龄也消耗本地有效期，后续排队和等待沿用同一个截止时刻。
  const auto valid_until = received_at + std::chrono::duration_cast<Clock::duration>(
    std::chrono::duration<double>(config_.input_timeout_s -
    static_cast<double>(current - stamp) / 1e9));
  auto frame = std::make_shared<Frame>(Frame{std::move(view), stamp, received_at, valid_until});

  std::lock_guard<std::mutex> lock(overlay_mutex_);
  const auto generation = synchronizer_.acceptFrame({stamp, frame->view->header.frame_id});
  if (!generation) {return;}
  image_arrivals_.fetch_add(1);
  // 赋代次和入槽不能分开解锁，否则任务切换清槽后会收到旧代次帧。
  frame->generation = *generation;
  frame_slot_.set(std::move(frame));
  overlay_cv_.notify_all();
}

void StreamRuntime::acceptDetections(
  const OverlayFrameKey & key, std::int64_t current,
  vision_msgs::msg::Detection2DArray::ConstSharedPtr msg)
{
  insertOverlay(key, OverlayLayer::Detections, current,
    [&](Overlay & overlay) {overlay.seg = std::move(msg);});
}

void StreamRuntime::acceptMask(
  const OverlayFrameKey & key, std::int64_t current, vision_ros::ImageViewConstPtr view)
{
  insertOverlay(key, OverlayLayer::Mask, current,
    [&](Overlay & overlay) {overlay.mask = std::move(view);});
}

void StreamRuntime::acceptField(
  const OverlayFrameKey & key, std::int64_t current,
  interface_pkg::msg::FieldObservation::ConstSharedPtr msg)
{
  insertOverlay(key, OverlayLayer::Field, current,
    [&](Overlay & overlay) {overlay.field = std::move(msg);});
}

bool StreamRuntime::matchesFrame(const Frame & frame, const std_msgs::msg::Header & header)
{
  const auto & source = frame.view->header;
  return source.stamp == header.stamp && source.frame_id == header.frame_id;
}

// 源时钟用于判旧，本地单调截止包含所有排队与等待耗时。
bool StreamRuntime::freshFrame(const Frame & frame) const
{
  const auto current = ports_.source_now();
  const auto local_now = Clock::now();
  return local_now <= frame.valid_until &&
         frame.stamp > 0 && frame.stamp <= current &&
         static_cast<double>(current - frame.stamp) <= config_.input_timeout_s * 1e9 &&
         std::chrono::duration<double>(local_now - frame.received_at).count() <=
         config_.input_timeout_s;
}

// 窗口速率游标只由状态定时器推进。
StreamRuntime::Status StreamRuntime::status()
{
  const double image_fps = image_window_.rate(image_arrivals_.load());
  const double compose_fps = compose_window_.rate(compose_meter_.count());
  const double compose_ms = compose_meter_.latencyMs();
  const auto viewers = streamer_ ? streamer_->clientCount() : 0;
  const double stream_fps = streamer_ ? streamer_->submitFps() : 0.0;
  const auto replaced = frame_slot_.overwritten();
  return {image_fps, compose_fps, compose_ms, stream_fps, viewers,
    composed_instances_.load(), replaced};
}

// 画面状态为各指标的当前快照，不参与业务有效性判断；实例数也不等同于最终选择目标数。
std::string StreamRuntime::statsText(bool drew_anything) const
{
  std::size_t viewers = 0;
  double stream_fps = 0.0;
  if (streamer_) {
    viewers = streamer_->clientCount();
    stream_fps = streamer_->submitFps();
  }
  const auto instances = composed_instances_.load();

  char buffer[256];
  std::snprintf(buffer, sizeof(buffer),
                "tx %4.1f fps q%ld  compose %4.1f ms  %zu instances  %zu viewer%s%s",
                stream_fps, static_cast<long>(config_.stream_quality),
                compose_meter_.latencyMs(), instances, viewers,
                viewers == 1 ? "" : "s", drew_anything ? "" : "  [no detections]");
  return std::string(buffer);
}

void StreamRuntime::setTask(
  const std::string & task, std::uint64_t epoch, std::int64_t cutoff)
{
  std::lock_guard<std::mutex> lock(overlay_mutex_);
  if (!synchronizer_.setTask(task, epoch, cutoff)) {return;}
  frame_slot_.clear();
  composed_instances_.store(0);
  // 同步清掉服务端旧 JPEG；仅清叠加缓存仍会让浏览器继续读到上一阶段的图像。
  if (streamer_) {streamer_->reset(synchronizer_.generation());}
  overlay_cv_.notify_all();
}

// ---- 合成主循环与有界图层等待 ----

void StreamRuntime::composeLoop()
{
  while (workers_.running()) {
    std::shared_ptr<Frame> frame = frame_slot_.take();
    if (!frame) {break;}

    if (!freshFrame(*frame)) {continue;}
    const auto demand = ports_.demand();
    if (!streamer_ && !demand.annotated && !demand.compressed) {continue;}
    // 复用服务端的同一限速状态，避免两套计时器叠加导致额外降帧。
    // 有 ROS 图像消费者时保留完整处理速率，HTTP 仍由 submit() 单独限速。
    if (!demand.annotated && !demand.compressed &&
      !streamer_->readyForFrame(frame->generation, frame->valid_until)) {continue;}

    Overlay snapshot;
    const auto wait_result = waitForOverlay(*frame, snapshot);
    if (wait_result == OverlayWait::Stopped) {break;}
    if (wait_result != OverlayWait::Ready) {continue;}
    // 等待也消耗帧的有效期；过期后不再复制或编码。
    if (!freshFrame(*frame)) {continue;}
    // compose_ms 覆盖画布/消息准备、ROS 发布及 HTTP 编码/提交；不含此前图层等待，
    // 也不代表网络完整帧接收或端到端延迟。只记录通过最终发布准入且未抛异常的帧。
    const auto t0 = std::chrono::steady_clock::now();

    try {
      auto output = prepareOutput(*frame, snapshot, demand);
      if (output.canvas.empty()) {continue;}
      if (!publishCurrentFrame(*frame, output)) {continue;}
      const auto t1 = std::chrono::steady_clock::now();
      compose_meter_.record(ms_d(t1 - t0).count());
    } catch (const std::exception & error) {
      // 捕获当前帧的标准异常后继续下一帧；已发消息不回滚，已取走的叠加快照也不回填。
      ports_.compose_error(error.what());
    }
  }
}

StreamRuntime::OverlayWait StreamRuntime::waitForOverlay(
  const Frame & frame, Overlay & snapshot)
{
  // config_.overlay_wait_ms 从原图到达时计时，排队耗时也占等待预算，不能在出槽后重新起算。
  // 超时就用已到达的同帧结果绘制；等待前后各核对一次代次。
  auto generation = frame.generation;
  OverlayFrameKey key{frame.stamp, frame.view->header.frame_id};
  const auto deadline = std::min(
    frame.valid_until,
    synchronizer_.waitDeadline(frame.received_at, config_.overlay_wait_ms));
  {
    std::unique_lock<std::mutex> lock(overlay_mutex_);
    if (generation != synchronizer_.generation()) {return OverlayWait::Skip;}
    // 等这一帧自己的识别图层。后到的识别帧留在队列里，不能用来取消当前帧。
    overlay_cv_.wait_until(
      lock, deadline, [&] {
        return !workers_.running() || generation != synchronizer_.generation() ||
        synchronizer_.ready(key);
      });
    if (!workers_.running()) {return OverlayWait::Stopped;}
    if (generation != synchronizer_.generation()) {return OverlayWait::Skip;}
    const auto result = synchronizer_.snapshot(key, generation);
    if (!result) {return OverlayWait::Skip;}
    // 只在锁内取得共享只读载荷；绘图、图像复制和编码放到锁外，避免堵住订阅回调。
    snapshot = *result;
    const auto seg_instances = snapshot.seg ? snapshot.seg->detections.size() : 0;
    const auto field_instances = snapshot.field && snapshot.field->valid ? 1u : 0u;
    composed_instances_.store(seg_instances + field_instances);
  }
  return OverlayWait::Ready;
}

// ---- 锁外绘制、编码与消息准备 ----

cv::Mat StreamRuntime::buildCanvas(
  const Frame & frame, const Overlay & overlay,
  bool & drew_anything)
{
  drew_anything = false;
  if (frame.view->image.empty()) {return cv::Mat();}

  // 快照里的消息是共享只读，绘制与转换都在合成线程、overlay_mutex_ 之外。
  const bool draw_mask = config_.draw_masks && overlay.mask &&
    matchesFrame(frame, overlay.mask->header);
  const bool draw_boxes = config_.draw_boxes && overlay.seg &&
    !overlay.seg->detections.empty() && matchesFrame(frame, overlay.seg->header);
  const bool draw_field = config_.draw_field && overlay.field &&
    matchesFrame(frame, overlay.field->header);
  const bool modify_pixels = config_.overlay_stats || draw_mask || draw_boxes || draw_field;
  cv::Mat canvas = modify_pixels ? frame.view->image.clone() : frame.view->image;
  // 图层须通过同源同帧检查；遮挡顺序为掩膜、检测框、场地区域，统计文字最后追加。
  if (draw_mask) {
    overlayLabelMask(canvas, overlay.mask->image);
    drew_anything = true;
  }
  if (draw_boxes) {
    drawDetectionArray(canvas, *overlay.seg, config_.draw_labels);
    drew_anything = !overlay.seg->detections.empty() || drew_anything;
  }
  if (draw_field) {
    drawFieldObservation(canvas, *overlay.field);
    drew_anything = overlay.field->valid || drew_anything;
  }
  return canvas;
}

StreamRuntime::PreparedOutput StreamRuntime::prepareOutput(
  const Frame & frame, const Overlay & snapshot, Demand demand)
{
  PreparedOutput output;
  bool drew_anything = false;
  output.canvas = buildCanvas(frame, snapshot, drew_anything);
  if (output.canvas.empty()) {return output;}

  // 统计底板最后追加，画布由 buildCanvas() 产出；有统计时该画布已与只读源图分离。
  if (config_.overlay_stats) {
    drawStats(output.canvas, statsText(drew_anything));
  }

  // 消息准备仍在锁外；真正发布由 publishCurrentFrame 做最终准入。
  // 沿用原图源时间，合成完成时间不能让已经过期的画面重新变新。
  auto header = frame.view->header;
  // 参数为空时沿用输入图像的 frame_id，而不是发布空字符串。
  if (!config_.frame_id.empty()) {header.frame_id = config_.frame_id;}

  if (demand.annotated) {
    output.annotated = vision_ros::makeBgrImageMessage(output.canvas, header);
  }

  // ROS 压缩图保留完整尺寸；HTTP 仅在比例恰为 1 且质量相同时共享该 JPEG。
  // 两路不满足复用条件时分别编码；缓存匹配帧键及归一化后的缩放、质量。
  output.share_jpeg = demand.compressed && streamer_ &&
    config_.stream_scale == 1.0 &&
    static_cast<int>(config_.stream_quality) == static_cast<int>(config_.compressed_quality);

  if (demand.compressed) {
    // 帧键每帧自增：同一帧重复提交才可能命中缓存，阶段切换后不会复用上一帧的字节。
    output.encode_started = std::chrono::steady_clock::now();
    output.jpeg = jpeg_encoder_.encode(
      output.canvas, ++encode_frame_key_, 1.0, static_cast<int>(config_.compressed_quality));
    output.encode_ms = ms_d(std::chrono::steady_clock::now() - output.encode_started).count();
    if (output.jpeg && !output.jpeg->empty()) {
      output.compressed = std::make_unique<sensor_msgs::msg::CompressedImage>();
      output.compressed->header = header;
      output.compressed->format = "jpeg";
      output.compressed->data = *output.jpeg;
    }
  }
  return output;
}

// ---- ROS 最终发布与 HTTP 二次准入 ----

bool StreamRuntime::publishCurrentFrame(
  const Frame & frame, PreparedOutput & output)
{
  const bool have_compressed = static_cast<bool>(output.compressed);
  {
    // 绘图与编码时可能已经换阶段；发布前重新持锁验代次，防止旧画面越过切换。
    // 不把这段 publish 移到锁外：阶段回调也用同一把锁完成清缓存与代次切换。
    std::lock_guard<std::mutex> lock(overlay_mutex_);
    if (!workers_.running() || frame.generation != synchronizer_.generation() ||
      !freshFrame(frame)) {return false;}
    if (output.annotated || output.compressed) {
      ports_.publish(std::move(output.annotated), std::move(output.compressed));
    }
  }
  if (streamer_ && workers_.running()) {
    // 图传在锁外提交，由 MjpegServer 按 frame.generation 和 valid_until 再做最终准入。
    // 停止可能发生在这次检查之后；wakeWaiters() 撤销 HTTP 代次，最终提交仍会拒绝旧帧。
    if (output.share_jpeg && have_compressed) {
      // 复用同一次编码：ROS的data已复制JPEG字节，HTTP共享output.jpeg，不再编码。
      streamer_->submitJpeg(
        output.jpeg, frame.generation, frame.valid_until,
        output.encode_started, output.encode_ms, frame.stamp);
    } else {
      streamer_->submit(output.canvas, frame.generation, frame.valid_until, frame.stamp);
    }
  }

  // true 只表示通过 ROS 最终发布准入；HTTP 的二次准入仍可能因换代、过期或限速拒绝该帧。
  return true;
}
