// Copyright 2026 jetson_yolo integration
// SPDX-License-Identifier: Apache-2.0

#include "node/stream_node.hpp"
#include "runtime/stream_runtime.hpp"

#include <arpa/inet.h>
#include <cmath>
#include <exception>
#include <utility>

#include <interface_pkg/app/ros/ros.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <rclcpp_components/register_node_macro.hpp>

namespace {
bool isIpv4Literal(const std::string & value)
{
  in_addr parsed{};
  return ::inet_pton(AF_INET, value.c_str(), &parsed) == 1;
}
}  // namespace

// 参数和编码能力先校验，完成 ROS 接口装配后再启动工作线程。
StreamNode::StreamNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("stream_node", options)
{
  declareParameters();
  const auto config = loadParameters();
  StreamRuntime::Ports ports{
    [this] {return now().nanoseconds();},
    [this] {
      return StreamRuntime::Demand{
        annotated_pub_ && annotated_pub_->get_subscription_count() > 0,
        compressed_pub_ && compressed_pub_->get_subscription_count() > 0};
    },
    [this](std::unique_ptr<sensor_msgs::msg::Image> annotated,
      std::unique_ptr<sensor_msgs::msg::CompressedImage> compressed) {
      if (annotated) {annotated_pub_->publish(std::move(annotated));}
      if (compressed) {compressed_pub_->publish(std::move(compressed));}
    },
    [this, address = config.stream_address, port = config.stream_port](
      bool started, const std::string & error) {
      if (!started) {
        RCLCPP_ERROR(get_logger(), "cannot start the stream on %s:%ld: %s",
          address.c_str(), static_cast<long>(port), error.c_str());
        return;
      }
      RCLCPP_INFO(
        get_logger(),
        "stream bound to %s:%ld -- http://<this-host>:%ld/ for MJPEG, "
        "/snapshot.jpg for a still, /healthz for counters",
        address.c_str(), static_cast<long>(port), static_cast<long>(port));
    },
    [this](const char * error) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000,
        "cannot compose this frame: %s", error);
    }};
  runtime_ = std::make_unique<StreamRuntime>(config, std::move(ports));

  createRosInterfaces(config);
  if (!rclcpp::ok(get_node_base_interface()->get_context())) {return;}
  runtime_->start();
  if (!rclcpp::ok(get_node_base_interface()->get_context())) {
    runtime_->stop();
    return;
  }

  RCLCPP_INFO(get_logger(),
              "stream_node started: image from %s, boxes=%d masks=%d field=%d",
              image_topic_.c_str(), static_cast<int>(config.draw_boxes),
              static_cast<int>(config.draw_masks), static_cast<int>(config.draw_field));
}

// 会话销毁时先回收线程；节点的发布者和日志仍有效。
StreamNode::~StreamNode() = default;

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
StreamRuntimeConfig StreamNode::loadParameters()
{
  StreamRuntimeConfig config;
  image_topic_ = get_parameter("image_topic").as_string();
  seg_detections_topic_ = get_parameter("seg_detections_topic").as_string();
  seg_masks_topic_ = get_parameter("seg_masks_topic").as_string();
  field_results_topic_ = get_parameter("field_results_topic").as_string();

  config.draw_boxes = get_parameter("draw_boxes").as_bool();
  config.draw_labels = get_parameter("draw_labels").as_bool();
  config.draw_masks = get_parameter("draw_masks").as_bool();
  config.draw_field = get_parameter("draw_field").as_bool();
  config.overlay_wait_ms = get_parameter("overlay_wait_ms").as_double();
  config.input_timeout_s = get_parameter("input_timeout_s").as_double();
  active_task_topic_ = get_parameter("active_task_topic").as_string();

  config.stream_enabled = get_parameter("stream_enabled").as_bool();
  config.stream_address = get_parameter("stream_address").as_string();
  config.stream_port = get_parameter("stream_port").as_int();
  const auto jpeg_backend = get_parameter("jpeg_backend").as_string();
  config.stream_quality = get_parameter("stream_quality").as_int();
  config.stream_max_fps = get_parameter("stream_max_fps").as_double();
  config.stream_scale = get_parameter("stream_scale").as_double();

  publish_annotated_ = get_parameter("publish_annotated").as_bool();
  config.publish_compressed = get_parameter("publish_compressed").as_bool();
  config.compressed_quality = get_parameter("compressed_quality").as_int();
  config.overlay_stats = get_parameter("overlay_stats").as_bool();
  publish_status_ = get_parameter("publish_status").as_bool();
  config.frame_id = get_parameter("frame_id").as_string();
  if (jpeg_backend != "opencv" && jpeg_backend != "nvjpeg") {
    throw rescue::ConfigurationError("invalid jpeg_backend: expected opencv or nvjpeg");
  }
  if (!isIpv4Literal(config.stream_address)) {
    throw rescue::ConfigurationError(
      "invalid stream_address '" + config.stream_address +
      "': expected an IPv4 address, for example 0.0.0.0 or 127.0.0.1");
  }
  if (!std::isfinite(config.input_timeout_s) || config.input_timeout_s <= 0.0 ||
    config.input_timeout_s > 3600.0 || !std::isfinite(config.overlay_wait_ms) || config.overlay_wait_ms < 0.0 ||
    config.overlay_wait_ms > 1000.0 || config.stream_port < 1 ||
    config.stream_port > 65535 || config.stream_quality < 1 || config.stream_quality > 100 ||
    config.compressed_quality < 1 || config.compressed_quality > 100 ||
    !std::isfinite(config.stream_scale) || config.stream_scale <= 0.0 || config.stream_scale > 1.0 ||
    !std::isfinite(config.stream_max_fps) || config.stream_max_fps < 0.0)
  {
    throw rescue::ConfigurationError(
      "invalid stream parameters: check quality, port, scale and overlay settings");
  }
  config.jpeg_backend = jpeg_backend == "nvjpeg" ? JpegBackend::NvJpeg : JpegBackend::OpenCv;
  return config;
}

// ---- ROS 接口装配 ----

// 仅订阅启用的图层；实时图像采用最新值 QoS，阶段通知单独保留给晚加入者。
void StreamNode::createRosInterfaces(const StreamRuntimeConfig & config)
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

  if (config.draw_boxes) {
    seg_det_sub_ = create_subscription<vision_msgs::msg::Detection2DArray>(
      seg_detections_topic_, qos,
      [this](vision_msgs::msg::Detection2DArray::SharedPtr msg) {segDetectionCallback(msg);});
  }
  if (config.draw_masks) {
    seg_mask_sub_ = create_subscription<sensor_msgs::msg::Image>(
      seg_masks_topic_, qos,
      [this](sensor_msgs::msg::Image::ConstSharedPtr msg) {segMaskCallback(msg);});
  }
  if (config.draw_field) {
    field_sub_ = create_subscription<interface_pkg::msg::FieldObservation>(
      field_results_topic_, qos,
      [this](interface_pkg::msg::FieldObservation::SharedPtr msg) {fieldCallback(msg);});
  }

  if (publish_annotated_) {
    annotated_pub_ = create_publisher<sensor_msgs::msg::Image>("~/image_annotated", qos);
  }
  if (config.publish_compressed) {
    compressed_pub_ =
      create_publisher<sensor_msgs::msg::CompressedImage>("~/image_compressed", qos);
  }
  if (publish_status_) {
    status_pub_ = create_publisher<interface_pkg::msg::StreamStatus>("~/status", qos);
  }
  // 后台失败必须传给入口，不能由诊断发布开关关闭异常出口。
  status_timer_ = create_wall_timer(std::chrono::seconds(1), [this] {
    runtime_->rethrowIfFailed();
    reportHttpFailure();
    publishStatus();
  });
}

// 阶段通知沿用源时钟；实际清槽、唤醒与 JPEG 失效统一由运行会话完成。
void StreamNode::activeTaskCallback(const interface_pkg::msg::TaskState::SharedPtr msg)
{
  std::int64_t cutoff = 0;
  if (!RecognitionValidation::sourceTime(msg->header.stamp, now().nanoseconds(), cutoff)) {
    return;
  }
  runtime_->setTask(msg->task, msg->epoch, cutoff);
}

// 校验源时钟及像素格式，保留消息所有者并交给运行会话。
void StreamNode::imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr & msg)
{
  const auto received_at = std::chrono::steady_clock::now();
  const auto current = now().nanoseconds();
  std::int64_t stamp = 0;
  if (!RecognitionValidation::sourceTime(msg->header.stamp, current, stamp) ||
    static_cast<double>(current - stamp) > runtime_->inputTimeout() * 1e9) {return;}
  vision_ros::ImageViewConstPtr view;
  try {
    view = vision_ros::bgrImageView(msg);
  } catch (const std::exception & e) {
    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000, "image input: %s", e.what());
    return;
  }
  if (view->image.empty()) {return;}

  runtime_->acceptFrame(std::move(view), stamp, current, received_at);
}

void StreamNode::segDetectionCallback(
  const vision_msgs::msg::Detection2DArray::SharedPtr msg)
{
  std::int64_t stamp = 0;
  const auto current = now().nanoseconds();
  if (!RecognitionValidation::sourceTime(msg->header.stamp, current, stamp)) {return;}
  runtime_->acceptDetections({stamp, msg->header.frame_id}, current, msg);
}

// MONO8/8UC1 原始标签共享视图，不转换或缩放类别字节。
// 缓存持有该视图的所有者，绘制时仍须与源图匹配 header。
void StreamNode::segMaskCallback(const sensor_msgs::msg::Image::ConstSharedPtr & msg)
{
  std::int64_t stamp = 0;
  const auto current = now().nanoseconds();
  if (!RecognitionValidation::sourceTime(msg->header.stamp, current, stamp) ||
    static_cast<double>(current - stamp) > runtime_->inputTimeout() * 1e9) {return;}
  vision_ros::ImageViewConstPtr view;
  try {
    view = vision_ros::labelMaskView(msg);
  } catch (const std::exception & e) {
    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000, "mask input: %s", e.what());
    return;
  }
  if (view->image.empty() || view->image.type() != CV_8UC1) {return;}
  runtime_->acceptMask({stamp, msg->header.frame_id}, current, std::move(view));
}

void StreamNode::fieldCallback(
  const interface_pkg::msg::FieldObservation::SharedPtr msg)
{
  std::int64_t stamp = 0;
  const auto current = now().nanoseconds();
  if (!RecognitionValidation::sourceTime(msg->header.stamp, current, stamp)) {return;}
  runtime_->acceptField({stamp, msg->header.frame_id}, current, msg);
}

void StreamNode::reportHttpFailure()
{
  const auto failure = runtime_->takeHttpFailure();
  if (!failure) {return;}
  // takeFailure 只回收已结束线程；仍在接入/读请求头时留给下轮，不阻塞 ROS 回调。
  // HTTP 失败后 ROS 图像输出仍由合成线程处理。
  try {
    std::rethrow_exception(failure);
  } catch (const std::exception & error) {
    RCLCPP_ERROR(get_logger(), "HTTP workers stopped: %s; ROS image output continues", error.what());
  } catch (...) {
    RCLCPP_ERROR(get_logger(), "HTTP workers stopped: unknown exception; ROS image output continues");
  }
}

// 状态定时器读取原子计数与服务快照；窗口速率游标仅由此回调更新。
void StreamNode::publishStatus()
{
  if (!rclcpp::ok() || !status_pub_) {return;}

  const auto sample = runtime_->status();

  interface_pkg::msg::StreamStatus status;
  status.header.stamp = now();
  status.image_fps = sample.image_fps;
  status.compose_fps = sample.compose_fps;
  status.compose_ms = sample.compose_ms;
  status.stream_fps = sample.stream_fps;
  status.viewers = sample.viewers;
  status.instances = sample.instances;
  status.replaced_frames = sample.replaced;
  status_pub_->publish(status);

  RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
                       "image %.1f fps | compose %.1f fps %.1f ms | tx %.1f fps | "
                       "viewers %ld | instances %zu | replaced %lu",
                       sample.image_fps, sample.compose_fps, sample.compose_ms, sample.stream_fps,
                       static_cast<long>(sample.viewers), sample.instances,
                       static_cast<unsigned long>(sample.replaced));
}

RCLCPP_COMPONENTS_REGISTER_NODE(StreamNode)
