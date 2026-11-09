// 阅读：图传节点的订阅、叠层和 HTTP 会话。
// 待发队列在 stream_sync.hpp。JPEG 在 stream/jpeg，MJPEG 在 stream/mjpeg。
// 不跑检测，也不选择目标。
#include "node/stream_node.hpp"
#include "stream/opencv/overlay.hpp"

#include <rclcpp_components/register_node_macro.hpp>

#include <sensor_msgs/image_encodings.hpp>
#include <interface_pkg/app/core.hpp>
#include <utility>
#include <cstdio>
#include <interface_pkg/app/core.hpp>
#include <arpa/inet.h>
#include <cmath>
#include <stdexcept>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>


namespace {
// 与 detector.cpp 相同的毫秒时长别名，用于把 steady_clock 的差值换算成毫秒。
using ms_d = std::chrono::duration<double, std::milli>;

bool isIpv4Literal(const std::string & value)
{
  in_addr parsed{};
  return ::inet_pton(AF_INET, value.c_str(), &parsed) == 1;
}
}  // namespace

// ---- 生命周期与只读配置 ----

// 启动只读参数校验完成后建立 ROS 输入，最后启动合成和 HTTP 会话。
StreamNode::StreamNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("stream_node", options), runtime_(std::make_unique<Runtime>(*this))
{
  declareParameters();
  loadParameters();

  // 完整参数校验后、ROS接口和工作线程创建前检查所选编码能力；关闭两路JPEG时不申请硬件。
  if (stream_enabled_ || publish_compressed_) {
    runtime_->jpeg_encoder_.setBackend(
      jpeg_backend_ == "nvjpeg" ? JpegBackend::NvJpeg : JpegBackend::OpenCv);
  }

  createRosInterfaces();
  if (!rclcpp::ok(get_node_base_interface()->get_context())) {return;}
  runtime_->start();
  if (!rclcpp::ok(get_node_base_interface()->get_context())) {
    runtime_->stop();
    return;
  }

  RCLCPP_INFO(get_logger(),
              "stream_node started: image from %s, boxes=%d masks=%d field=%d",
              image_topic_.c_str(), static_cast<int>(draw_boxes_),
              static_cast<int>(draw_masks_), static_cast<int>(draw_field_));
}

// 合成线程借用节点的发布者与配置，必须先回收线程再释放节点成员。
StreamNode::~StreamNode()
{
  runtime_->stop();
}

// 所有参数声明为只读，防止回调与合成线程读取到未同步的配置变化。
void StreamNode::declareParameters()
{
  rcl_interfaces::msg::ParameterDescriptor descriptor;
  descriptor.read_only = true;
  rescue::declareRequired<std::string>(*this, "image_topic", descriptor);
  rescue::declareRequired<std::string>(*this, "seg_detections_topic", descriptor);
  rescue::declareRequired<std::string>(*this, "seg_masks_topic", descriptor);
  rescue::declareRequired<std::string>(*this, "field_results_topic", descriptor);

  rescue::declareRequired<bool>(*this, "draw_boxes", descriptor);
  rescue::declareRequired<bool>(*this, "draw_labels", descriptor);
  rescue::declareRequired<bool>(*this, "draw_masks", descriptor);
  rescue::declareRequired<bool>(*this, "draw_field", descriptor);
  rescue::declareRequired<double>(*this, "overlay_wait_ms", descriptor);
  rescue::declareRequired<double>(*this, "input_timeout_s", descriptor);
  rescue::declareRequired<std::string>(*this, "active_task_topic", descriptor);

  rescue::declareRequired<bool>(*this, "stream_enabled", descriptor);
  rescue::declareRequired<std::string>(*this, "stream_address", descriptor);
  rescue::declareRequired<int>(*this, "stream_port", descriptor);
  rescue::declareRequired<std::string>(*this, "jpeg_backend", descriptor);
  rescue::declareRequired<int>(*this, "stream_quality", descriptor);
  rescue::declareRequired<double>(*this, "stream_max_fps", descriptor);
  rescue::declareRequired<double>(*this, "stream_scale", descriptor);

  rescue::declareRequired<bool>(*this, "publish_annotated", descriptor);
  rescue::declareRequired<bool>(*this, "publish_compressed", descriptor);
  rescue::declareRequired<int>(*this, "compressed_quality", descriptor);
  rescue::declareRequired<bool>(*this, "overlay_stats", descriptor);
  rescue::declareRequired<bool>(*this, "publish_status", descriptor);
  rescue::declareRequired<std::string>(*this, "frame_id", descriptor);
}

// 先完整校验地址、时间预算和图像输出设置，再建立同帧图层要求。
void StreamNode::loadParameters()
{
  image_topic_ = get_parameter("image_topic").as_string();
  seg_detections_topic_ = get_parameter("seg_detections_topic").as_string();
  seg_masks_topic_ = get_parameter("seg_masks_topic").as_string();
  field_results_topic_ = get_parameter("field_results_topic").as_string();

  draw_boxes_ = get_parameter("draw_boxes").as_bool();
  draw_labels_ = get_parameter("draw_labels").as_bool();
  draw_masks_ = get_parameter("draw_masks").as_bool();
  draw_field_ = get_parameter("draw_field").as_bool();
  overlay_wait_ms_ = get_parameter("overlay_wait_ms").as_double();
  input_timeout_s_ = get_parameter("input_timeout_s").as_double();
  active_task_topic_ = get_parameter("active_task_topic").as_string();

  stream_enabled_ = get_parameter("stream_enabled").as_bool();
  stream_address_ = get_parameter("stream_address").as_string();
  stream_port_ = get_parameter("stream_port").as_int();
  jpeg_backend_ = get_parameter("jpeg_backend").as_string();
  stream_quality_ = get_parameter("stream_quality").as_int();
  stream_max_fps_ = get_parameter("stream_max_fps").as_double();
  stream_scale_ = get_parameter("stream_scale").as_double();

  publish_annotated_ = get_parameter("publish_annotated").as_bool();
  publish_compressed_ = get_parameter("publish_compressed").as_bool();
  compressed_quality_ = get_parameter("compressed_quality").as_int();
  overlay_stats_ = get_parameter("overlay_stats").as_bool();
  publish_status_ = get_parameter("publish_status").as_bool();
  frame_id_ = get_parameter("frame_id").as_string();
  if (jpeg_backend_ != "opencv" && jpeg_backend_ != "nvjpeg") {
    throw rescue::ConfigurationError("invalid jpeg_backend: expected opencv or nvjpeg");
  }
  if (!isIpv4Literal(stream_address_)) {
    throw rescue::ConfigurationError(
      "invalid stream_address '" + stream_address_ +
      "': expected an IPv4 address, for example 0.0.0.0 or 127.0.0.1");
  }
  if (!std::isfinite(input_timeout_s_) || input_timeout_s_ <= 0.0 ||
    input_timeout_s_ > 3600.0 || !std::isfinite(overlay_wait_ms_) || overlay_wait_ms_ < 0.0 ||
    overlay_wait_ms_ > 1000.0 || stream_port_ < 1 ||
    stream_port_ > 65535 || stream_quality_ < 1 || stream_quality_ > 100 ||
    compressed_quality_ < 1 || compressed_quality_ > 100 ||
    !std::isfinite(stream_scale_) || stream_scale_ <= 0.0 || stream_scale_ > 1.0 ||
    !std::isfinite(stream_max_fps_) || stream_max_fps_ < 0.0)
  {
    throw rescue::ConfigurationError(
      "invalid stream parameters: check quality, port, scale and overlay settings");
  }
  // labels 依附于 boxes，不单独等待；关闭某图层后不等待其消息，仍严格按当前阶段匹配同帧。
  runtime_->synchronizer_.setRequirements({draw_boxes_, draw_masks_, draw_field_});
}

// ---- ROS 接口装配 ----

// 仅订阅启用的图层；实时图像采用最新值 QoS，阶段通知单独保留给晚加入者。
void StreamNode::createRosInterfaces()
{
  const auto qos = rclcpp::SensorDataQoS().keep_last(1);
  rclcpp::SubscriptionOptions mode_options;
  mode_options.use_intra_process_comm = rclcpp::IntraProcessSetting::Disable;
  active_task_sub_ = create_subscription<interface_pkg::msg::TaskState>(
    active_task_topic_, rclcpp::QoS(1).reliable().transient_local(),
    [this](interface_pkg::msg::TaskState::SharedPtr msg) {activeTaskCallback(msg);}, mode_options);

  image_sub_ = create_subscription<sensor_msgs::msg::Image>(
    image_topic_, qos,
    [this](sensor_msgs::msg::Image::ConstSharedPtr msg) {imageCallback(msg);});

  if (draw_boxes_) {
    seg_det_sub_ = create_subscription<vision_msgs::msg::Detection2DArray>(
      seg_detections_topic_, qos,
      [this](vision_msgs::msg::Detection2DArray::SharedPtr msg) {segDetectionCallback(msg);});
  }
  if (draw_masks_) {
    seg_mask_sub_ = create_subscription<sensor_msgs::msg::Image>(
      seg_masks_topic_, qos,
      [this](sensor_msgs::msg::Image::ConstSharedPtr msg) {segMaskCallback(msg);});
  }
  if (draw_field_) {
    field_sub_ = create_subscription<interface_pkg::msg::FieldObservation>(
      field_results_topic_, qos,
      [this](interface_pkg::msg::FieldObservation::SharedPtr msg) {fieldCallback(msg);});
  }

  if (publish_annotated_) {
    annotated_pub_ = create_publisher<sensor_msgs::msg::Image>("~/image_annotated", qos);
  }
  if (publish_compressed_) {
    compressed_pub_ =
      create_publisher<sensor_msgs::msg::CompressedImage>("~/image_compressed", qos);
  }
  if (publish_status_) {
    status_pub_ = create_publisher<interface_pkg::msg::StreamStatus>("~/status", qos);
  }
  // 后台失败必须传给入口，不能由诊断发布开关关闭异常出口。
  status_timer_ = create_wall_timer(std::chrono::seconds(1), [this] {
    runtime_->workers_.rethrowIfFailed();
    runtime_->reportHttpFailure();
    publishStatus();
  });

}

// ---- 阶段更新与同帧准入 ----

// 阶段通知沿用源时钟；实际清槽、唤醒与 JPEG 失效统一由运行会话完成。
void StreamNode::activeTaskCallback(const interface_pkg::msg::TaskState::SharedPtr msg)
{
  std::int64_t cutoff = 0;
  if (!RecognitionValidation::sourceTime(msg->header.stamp, now().nanoseconds(), cutoff)) {
    return;
  }
  runtime_->setTask(msg->task, msg->epoch, cutoff);
}

bool StreamNode::matchesFrame(const Frame & frame, const std_msgs::msg::Header & header) const
{
  const auto stamp = rclcpp::Time(header.stamp).nanoseconds();
  if (stamp <= 0 || header.frame_id != frame.frame_id) {return false;}
  return stamp == frame.stamp.nanoseconds();
}

// 同时检查源时钟年龄和本地单调截止时间，等待/排队不能把旧图重新变新。
bool StreamNode::freshFrame(const Frame & frame) const
{
  const auto current = now().nanoseconds();
  const auto stamp = frame.stamp.nanoseconds();
  const auto local_now = std::chrono::steady_clock::now();
  return local_now <= frame.valid_until &&
         stamp > 0 && stamp <= current &&
         static_cast<double>(current - stamp) <= input_timeout_s_ * 1e9 &&
         std::chrono::duration<double>(
    local_now - frame.received_at).count() <= input_timeout_s_;
}

// ---- 源图与图层交接 ----

// 保留 cv_bridge 视图所有者；源帧在接收前消耗的年龄也扣入本地有效期。
void StreamNode::imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr & msg)
{
  const auto received_at = std::chrono::steady_clock::now();
  const auto current = now().nanoseconds();
  std::int64_t stamp = 0;
  if (!RecognitionValidation::sourceTime(msg->header.stamp, current, stamp) ||
    static_cast<double>(current - stamp) > input_timeout_s_ * 1e9) {return;}
  cv_bridge::CvImageConstPtr view;
  try {
    if (msg->encoding == sensor_msgs::image_encodings::BGR8 && msg->is_bigendian == 0) {
      view = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8);
    } else {
      view = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
  } catch (const cv_bridge::Exception & e) {
    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000, "cv_bridge: %s", e.what());
    return;
  }
  if (view->image.empty()) {return;}

  auto frame = std::make_shared<Frame>();
  frame->image = view->image;
  frame->holder = view;  // 保住零拷贝视图所属的消息
  frame->stamp = msg->header.stamp;
  frame->frame_id = msg->header.frame_id;
  frame->received_at = received_at;
  frame->valid_until = frame->received_at +
    std::chrono::duration_cast<std::chrono::steady_clock::duration>(
    std::chrono::duration<double>(input_timeout_s_ -
      static_cast<double>(current - stamp) / 1e9));
  runtime_->acceptFrame(std::move(frame));
}

void StreamNode::segDetectionCallback(
  const vision_msgs::msg::Detection2DArray::SharedPtr msg)
{
  std::int64_t stamp = 0;
  const auto current = now().nanoseconds();
  if (!RecognitionValidation::sourceTime(msg->header.stamp, current, stamp)) {return;}
  runtime_->insertOverlay({stamp, msg->header.frame_id}, OverlayLayer::Detections,
    current, input_timeout_s_, [&](Overlay & overlay) {overlay.seg = msg;});
}

// 通过 cv_bridge 取得 MONO8 视图；若输入编码可转换，桥接层可能分配转换结果。
// 缓存持有该视图的所有者，绘制时仍须与源图匹配 header。
void StreamNode::segMaskCallback(const sensor_msgs::msg::Image::ConstSharedPtr & msg)
{
  std::int64_t stamp = 0;
  const auto current = now().nanoseconds();
  if (!RecognitionValidation::sourceTime(msg->header.stamp, current, stamp) ||
    static_cast<double>(current - stamp) > input_timeout_s_ * 1e9) {return;}
  cv_bridge::CvImageConstPtr view;
  try {
    view = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::MONO8);
  } catch (const cv_bridge::Exception & e) {
    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000, "mask cv_bridge: %s", e.what());
    return;
  }
  if (view->image.empty() || view->image.type() != CV_8UC1) {return;}
  runtime_->insertOverlay({stamp, msg->header.frame_id}, OverlayLayer::Mask,
    current, input_timeout_s_, [&](Overlay & overlay) {overlay.mask = view;});
}

void StreamNode::fieldCallback(
  const interface_pkg::msg::FieldObservation::SharedPtr msg)
{
  std::int64_t stamp = 0;
  const auto current = now().nanoseconds();
  if (!RecognitionValidation::sourceTime(msg->header.stamp, current, stamp)) {return;}
  runtime_->insertOverlay({stamp, msg->header.frame_id}, OverlayLayer::Field,
    current, input_timeout_s_, [&](Overlay & overlay) {overlay.field = msg;});
}

// ---- 状态采样与叠层文字 ----

// 画面状态为各指标的当前快照，不参与业务有效性判断；实例数也不等同于最终选择目标数。
std::string StreamNode::statsText(bool drew_anything) const
{
  std::size_t viewers = 0;
  std::size_t instances = 0;
  double stream_fps = 0.0;
  if (runtime_->streamer_) {
    viewers = runtime_->streamer_->clientCount();
    stream_fps = runtime_->streamer_->submitFps();
  }
  instances = runtime_->composed_instances_.load();

  char buffer[256];
  std::snprintf(buffer, sizeof(buffer),
                "tx %4.1f fps q%ld  compose %4.1f ms  %zu instances  %zu viewer%s%s",
                stream_fps, static_cast<long>(stream_quality_),
                runtime_->compose_meter_.latencyMs(), instances, viewers,
                viewers == 1 ? "" : "s", drew_anything ? "" : "  [no detections]");
  return std::string(buffer);
}

// 状态定时器读取原子计数与服务快照；窗口速率游标仅由此回调更新。
void StreamNode::publishStatus()
{
  if (!rclcpp::ok() || !status_pub_) {return;}

  const double image_fps = runtime_->image_window_.rate(runtime_->image_arrivals_.load());
  const double compose_fps = runtime_->compose_window_.rate(runtime_->compose_meter_.count());
  const double compose_ms = runtime_->compose_meter_.latencyMs();
  const std::size_t viewers = runtime_->streamer_ ? runtime_->streamer_->clientCount() : 0;
  const double stream_fps = runtime_->streamer_ ? runtime_->streamer_->submitFps() : 0.0;
  const std::uint64_t replaced = runtime_->frame_slot_.overwritten();

  std::size_t instances = 0;
  instances = runtime_->composed_instances_.load();

  interface_pkg::msg::StreamStatus status;
  status.header.stamp = now();
  status.image_fps = image_fps;
  status.compose_fps = compose_fps;
  status.compose_ms = compose_ms;
  status.stream_fps = stream_fps;
  status.viewers = viewers;
  status.instances = instances;
  status.replaced_frames = replaced;
  status_pub_->publish(status);

  RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
                       "image %.1f fps | compose %.1f fps %.1f ms | tx %.1f fps | "
                       "viewers %ld | instances %zu | replaced %lu",
                       image_fps, compose_fps, compose_ms, stream_fps,
                       static_cast<long>(viewers), instances,
                       static_cast<unsigned long>(replaced));
}

// ---- 会话启动、HTTP 装配与停止 ----

void StreamNode::Runtime::start()
{
  try {
    startServer();
    workers_.start({[this] {composeLoop();}}, [this] {wakeWaiters();});
  } catch (...) {
    stop();
    throw;
  }
}

void StreamNode::Runtime::startServer()
{
  if (owner_.stream_enabled_) {
    // Options 使用 StreamNode::loadParameters() 已校验的配置。
    // 服务绑定失败只记录错误并关闭 HTTP 能力，节点仍可处理已启用的 ROS 图像输出。
    MjpegServer::Options stream_options;
    stream_options.address = owner_.stream_address_;
    stream_options.port = static_cast<int>(owner_.stream_port_);
    stream_options.quality = static_cast<int>(owner_.stream_quality_);
    stream_options.jpeg_backend =
      owner_.jpeg_backend_ == "nvjpeg" ? JpegBackend::NvJpeg : JpegBackend::OpenCv;
    stream_options.max_fps = owner_.stream_max_fps_;
    stream_options.scale = owner_.stream_scale_;
    stream_options.frame_timeout_s = owner_.input_timeout_s_;

    streamer_ = std::make_unique<MjpegServer>(stream_options);
    std::string error;
    if (!streamer_->start(error)) {
      RCLCPP_ERROR(
        owner_.get_logger(), "cannot start the stream on %s:%ld: %s",
        owner_.stream_address_.c_str(), static_cast<long>(owner_.stream_port_),
        error.c_str());
    } else {
      RCLCPP_INFO(
        owner_.get_logger(),
        "stream bound to %s:%ld -- http://<this-host>:%ld/ for MJPEG, "
        "/snapshot.jpg for a still, /healthz for counters",
        owner_.stream_address_.c_str(), static_cast<long>(owner_.stream_port_),
        static_cast<long>(owner_.stream_port_));
    }
  }

}

void StreamNode::Runtime::reportHttpFailure()
{
  if (!streamer_) {return;}
  const auto failure = streamer_->takeFailure();
  if (!failure) {return;}
  // takeFailure 只回收已结束线程；仍在接入/读请求头时留给下轮，不阻塞 ROS 回调。
  // 对象保留到 Runtime 收尾，避免锁外编码、状态采样或换代仍在访问它时销毁。
  try {
    std::rethrow_exception(failure);
  } catch (const std::exception & error) {
    RCLCPP_ERROR(owner_.get_logger(), "HTTP workers stopped: %s; ROS image output continues", error.what());
  } catch (...) {
    RCLCPP_ERROR(owner_.get_logger(), "HTTP workers stopped: unknown exception; ROS image output continues");
  }
}

void StreamNode::Runtime::stop()
{
  // 先唤醒并回收合成线程，再停止服务，避免线程继续向已关闭的服务提交画面。
  workers_.stop([this] {wakeWaiters();});
  if (streamer_) {streamer_->stop();}
}

void StreamNode::Runtime::wakeWaiters()
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

void StreamNode::Runtime::acceptFrame(std::shared_ptr<Frame> frame)
{
  std::lock_guard<std::mutex> lock(overlay_mutex_);
  const auto generation = synchronizer_.acceptFrame(
    {frame->stamp.nanoseconds(), frame->frame_id});
  if (!generation) {return;}
  image_arrivals_.fetch_add(1);
  // 赋代次和入槽不能分开解锁；否则 setTask() 可能清槽后又收到带旧代次的帧。
  frame->generation = *generation;
  frame_slot_.set(std::move(frame));
  overlay_cv_.notify_all();
}

void StreamNode::Runtime::setTask(
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

void StreamNode::Runtime::composeLoop()
{
  while (workers_.running()) {
    std::shared_ptr<Frame> frame = frame_slot_.take();
    if (!frame) {break;}

    if (!owner_.freshFrame(*frame)) {continue;}
    const bool want_annotated = owner_.annotated_pub_ &&
      owner_.annotated_pub_->get_subscription_count() > 0;
    const bool want_compressed = owner_.compressed_pub_ &&
      owner_.compressed_pub_->get_subscription_count() > 0;
    if (!streamer_ && !want_annotated && !want_compressed) {continue;}
    // 复用服务端的同一限速状态，避免两套计时器叠加导致额外降帧。
    // 有 ROS 图像消费者时保留完整处理速率，HTTP 仍由 submit() 单独限速。
    if (!want_annotated && !want_compressed &&
      !streamer_->readyForFrame(frame->generation, frame->valid_until)) {continue;}

    Overlay snapshot;
    const auto wait_result = waitForOverlay(*frame, snapshot);
    if (wait_result == OverlayWait::Stopped) {break;}
    if (wait_result != OverlayWait::Ready) {continue;}
    // 等待也消耗帧的有效期；过期后不再复制或编码。
    if (!owner_.freshFrame(*frame)) {continue;}
    // compose_ms 覆盖画布/消息准备、ROS 发布及 HTTP 编码/提交；不含此前图层等待，
    // 也不代表网络完整帧接收或端到端延迟。只记录通过最终发布准入且未抛异常的帧。
    const auto t0 = std::chrono::steady_clock::now();

    try {
      auto output = prepareOutput(*frame, snapshot, want_annotated, want_compressed);
      if (output.canvas.empty()) {continue;}
      if (!publishCurrentFrame(*frame, output)) {continue;}
      const auto t1 = std::chrono::steady_clock::now();
      compose_meter_.record(ms_d(t1 - t0).count());
    } catch (const std::exception & error) {
      // 捕获当前帧的标准异常后继续下一帧；已发消息不回滚，已取走的叠加快照也不回填。
      RCLCPP_ERROR_THROTTLE(
        owner_.get_logger(), *owner_.get_clock(), 5000,
        "cannot compose this frame: %s", error.what());
    }
  }
}

StreamNode::Runtime::OverlayWait StreamNode::Runtime::waitForOverlay(
  const Frame & frame, Overlay & snapshot)
{
  // owner_.overlay_wait_ms_ 从原图到达时计时，排队耗时也占等待预算，不能在出槽后重新起算。
  // 超时就用已到达的同帧结果绘制；等待前后各核对一次代次。
  auto generation = frame.generation;
  OverlayFrameKey key{frame.stamp.nanoseconds(), frame.frame_id};
  const auto deadline = std::min(
    frame.valid_until,
    synchronizer_.waitDeadline(frame.received_at, owner_.overlay_wait_ms_));
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

cv::Mat StreamNode::Runtime::buildCanvas(
  const Frame & frame, const Overlay & overlay,
  bool & drew_anything)
{
  drew_anything = false;
  if (frame.image.empty()) {return cv::Mat();}

  // 快照里的消息是共享只读，绘制与转换都在合成线程、overlay_mutex_ 之外。
  const bool draw_mask = owner_.draw_masks_ && overlay.mask &&
    owner_.matchesFrame(frame, overlay.mask->header);
  const bool draw_boxes = owner_.draw_boxes_ && overlay.seg &&
    !overlay.seg->detections.empty() && owner_.matchesFrame(frame, overlay.seg->header);
  const bool draw_field = owner_.draw_field_ && overlay.field &&
    owner_.matchesFrame(frame, overlay.field->header);
  const bool modify_pixels = owner_.overlay_stats_ || draw_mask || draw_boxes || draw_field;
  cv::Mat canvas = modify_pixels ? frame.image.clone() : frame.image;
  // 图层须通过同源同帧检查；遮挡顺序为掩膜、检测框、场地区域，统计文字最后追加。
  if (draw_mask) {
    overlayLabelMask(canvas, overlay.mask->image);
    drew_anything = true;
  }
  if (draw_boxes) {
    drawDetectionArray(canvas, *overlay.seg, owner_.draw_labels_);
    drew_anything = !overlay.seg->detections.empty() || drew_anything;
  }
  if (draw_field) {
    drawFieldObservation(canvas, *overlay.field);
    drew_anything = overlay.field->valid || drew_anything;
  }
  return canvas;
}

StreamNode::Runtime::PreparedOutput StreamNode::Runtime::prepareOutput(
  const Frame & frame, const Overlay & snapshot, bool want_annotated, bool want_compressed)
{
  PreparedOutput output;
  bool drew_anything = false;
  output.canvas = buildCanvas(frame, snapshot, drew_anything);
  if (output.canvas.empty()) {return output;}

  // 统计底板最后追加，画布由 buildCanvas() 产出；有统计时该画布已与只读源图分离。
  if (owner_.overlay_stats_) {
    drawStats(output.canvas, owner_.statsText(drew_anything));
  }

  // 消息准备仍在锁外；真正发布由 publishCurrentFrame 做最终准入。
  // 沿用原图源时间，合成完成时间不能让已经过期的画面重新变新。
  std_msgs::msg::Header header;
  header.stamp = frame.stamp;
  // 参数为空时沿用输入图像的 frame_id，而不是发布空字符串。
  header.frame_id = owner_.frame_id_.empty() ? frame.frame_id : owner_.frame_id_;

  if (want_annotated) {
    output.annotated = vision_ros::makeBgrImageMessage(output.canvas, header);
  }

  // ROS 压缩图保留完整尺寸；HTTP 仅在比例恰为 1 且质量相同时共享该 JPEG。
  // 两路不满足复用条件时分别编码；缓存匹配帧键及归一化后的缩放、质量。
  output.share_jpeg = want_compressed && streamer_ &&
    owner_.stream_scale_ == 1.0 &&
    static_cast<int>(owner_.stream_quality_) == static_cast<int>(owner_.compressed_quality_);

  if (want_compressed) {
    // 帧键每帧自增：同一帧重复提交才可能命中缓存，阶段切换后不会复用上一帧的字节。
    output.encode_started = std::chrono::steady_clock::now();
    output.jpeg = jpeg_encoder_.encode(
      output.canvas, ++encode_frame_key_, 1.0, static_cast<int>(owner_.compressed_quality_));
    output.encode_ms = ms_d(std::chrono::steady_clock::now() - output.encode_started).count();
    if (output.jpeg && !output.jpeg->empty()) {
      output.compressed = std::make_unique<sensor_msgs::msg::CompressedImage>();
      output.compressed->header = header;
      output.compressed->format = "jpeg";
      output.compressed->data = *output.jpeg;
      output.have_compressed = true;
    }
  }
  return output;
}

// ---- ROS 最终发布与 HTTP 二次准入 ----

bool StreamNode::Runtime::publishCurrentFrame(
  const Frame & frame, PreparedOutput & output)
{
  {
    // 绘图与编码时可能已经换阶段；发布前重新持锁验代次，防止旧画面越过切换。
    // 不把这段 publish 移到锁外：阶段回调也用同一把锁完成清缓存与代次切换。
    std::lock_guard<std::mutex> lock(overlay_mutex_);
    if (!workers_.running() || frame.generation != synchronizer_.generation() ||
      !owner_.freshFrame(frame)) {return false;}
    if (output.annotated) {owner_.annotated_pub_->publish(std::move(output.annotated));}
    if (output.have_compressed) {owner_.compressed_pub_->publish(std::move(output.compressed));}
  }
  if (streamer_ && workers_.running()) {
    // 图传在锁外提交，由 MjpegServer 按 frame.generation 和 valid_until 再做最终准入。
    // 停止可能发生在这次检查之后；wakeWaiters() 撤销 HTTP 代次，最终提交仍会拒绝旧帧。
    if (output.share_jpeg && output.have_compressed) {
      // 复用同一次编码：ROS的data已复制JPEG字节，HTTP共享output.jpeg，不再编码。
      streamer_->submitJpeg(
        output.jpeg, frame.generation, frame.valid_until,
        output.encode_started, output.encode_ms, frame.stamp.nanoseconds());
    } else {
      streamer_->submit(output.canvas, frame.generation, frame.valid_until, frame.stamp.nanoseconds());
    }
  }

  // true 只表示通过 ROS 最终发布准入；HTTP 的二次准入仍可能因换代、过期或限速拒绝该帧。
  return true;
}

RCLCPP_COMPONENTS_REGISTER_NODE(StreamNode)
