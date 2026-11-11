#include "node/detect_node.hpp"

// ---- 视觉节点实现 ----
// 职责：检测角色、运行循环；进程入口在 app/vision_runtime.cpp。
// Copyright 2026 jetson_yolo integration
// SPDX-License-Identifier: Apache-2.0
// DetectNode 在构造时完成一次配置检查，随后启动采集、推理和发布。
// 一帧依次经过 captureLoop → inferenceLoop → resultLoop → publishSeg/publishField。
// 原图由 rawLoop 独立投递；所有最终发布仍在原阶段/相机会话闸门内复核。
#include <interface_pkg/app/core.hpp>
#include <rclcpp_components/register_node_macro.hpp>
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


// 先检查范围，再把 ROS 的 int64 收成后端用的 int。
static int checkedInt(const rclcpp::Node & node, const char * name, int minimum, int maximum)
{
  const auto value = node.get_parameter(name).as_int();
  if (value < minimum || value > maximum) {
    throw rescue::ConfigurationError(std::string(name) + " must be in [" +
      std::to_string(minimum) + ", " + std::to_string(maximum) + "]");
  }
  return static_cast<int>(value);
}

// 浮点配置除范围外还检查 NaN/无穷，避免它们绕过后续比较。
static double checkedDouble(
  const rclcpp::Node & node, const char * name, double minimum, double maximum)
{
  const double value = node.get_parameter(name).as_double();
  if (!std::isfinite(value) || value < minimum || value > maximum) {
    throw rescue::ConfigurationError(std::string(name) + " must be finite and in [" +
      std::to_string(minimum) + ", " + std::to_string(maximum) + "]");
  }
  return value;
}

// 路径由统一入口解析成绝对路径；节点不再按包 share 或工作目录猜测来源。
static std::string configuredPath(const rclcpp::Node & node, const char * name)
{
  const auto value = node.get_parameter(name).as_string();
  if (!value.empty() && !std::filesystem::path(value).is_absolute()) {
    throw rescue::ConfigurationError(std::string(name) + " must be resolved to an absolute path");
  }
  return value;
}

// ---- DetectNode：生命周期与启动配置 ----

// 参数校验→后端装配→ROS 接口/初始阶段→工作线程；相机实际打开仍归采集线程。
DetectNode::DetectNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("detect_node", options), runtime_(std::make_unique<Runtime>(*this))
{
  declareParameters();
  loadParameters();
  if (!rclcpp::ok(get_node_base_interface()->get_context())) {return;}

  // task 决定加载哪些后端，active_task 决定当前执行哪一个；阶段切换不会重新加载或构建引擎。
  const auto detector_config = initializeDetector();
  // 协作取消后节点仍完整构造，入口看到关闭的 context 后跳过 spin；此时未启动相机工作线程。
  if (!detector_config || !rclcpp::ok(get_node_base_interface()->get_context())) {return;}
  runtime_->camera_ = std::make_unique<UsbCamera>(camera_config_);
  createRosOutputs();
  applyModeParameters();

  createStageService();
  runtime_->start();
  if (!rclcpp::ok(get_node_base_interface()->get_context())) {
    runtime_->stop();
    return;
  }

  RCLCPP_INFO(get_logger(), "detect_node started: loaded=%s, active_task=%s, raw feed=%s, "
              "processed feed=%s",
              toString(detector_config->task),
              get_parameter("active_task").as_string().c_str(),
              raw_pub_ ? "on (published only while subscribed)" : "off",
              processed_pub_ ? "on (published only while subscribed)" : "off");
}

// 在线程可能访问的 ROS 成员析构前先回收运行会话。
DetectNode::~DetectNode()
{
  runtime_->stop();
}

// 启动必需项不带默认值；统一入口完成解析，缺项在建立相机或推理资源之前报错。
void DetectNode::declareParameters()
{
  rescue::declareRequired<std::string>(*this, "task");

  rescue::declareRequired<std::string>(*this, "seg_model_path");

  rescue::declareRequired<std::string>(*this, "seg_labels_path");

  rescue::declareRequired<int>(*this, "dla_core");

  // 置信度/NMS 是无量纲标量；场地二元区间数组由统一装配拆成各 field.* 端点参数。
  rescue::declareRequired<double>(*this, "conf_threshold_seg");
  rescue::declareRequired<double>(*this, "nms_threshold");
  rescue::declareRequired<int>(*this, "max_detections");

  rescue::declareRequired<int>(*this, "mask_scale");
  rescue::declareRequired<int>(*this, "mask_label_offset");
  rescue::declareRequired<int>(*this, "max_masks");
  rescue::declareRequired<bool>(*this, "publish_masks");
  rescue::declareRequired<bool>(*this, "publish_field_results");

  rescue::declareRequired<bool>(*this, "publish_raw");
  // 处理帧开关来自统一配置；由结果发布线程发布结果消息实际使用的源帧。
  rescue::declareRequired<bool>(*this, "publish_processed");
  rescue::declareRequired<double>(*this, "raw_max_fps");

  rescue::declareRequired<std::string>(*this, "camera_device");
  rescue::declareRequired<int>(*this, "camera_width");
  rescue::declareRequired<int>(*this, "camera_height");
  rescue::declareRequired<int>(*this, "camera_fps");
  rescue::declareRequired<int>(*this, "camera_brightness");
  rescue::declareRequired<std::string>(*this, "camera_fourcc");
  rescue::declareRequired<int>(*this, "camera_buffer_size");
  rescue::declareRequired<std::string>(*this, "camera_backend");
  rescue::declareRequired<std::string>(*this, "camera_pipeline");

  rescue::declareRequired<bool>(*this, "publish_timing");
  // 0 表示不限速。非 0 时按这个频率推理，多出来的帧丢掉。
  rescue::declareRequired<double>(*this, "seg_max_fps");
  rescue::declareRequired<double>(*this, "field_max_fps");
  // task 决定加载一套或两套后端；active_task 只选择其中一个已加载阶段。
  rescue::declareRequired<std::string>(*this, "active_task");
  rescue::declareRequired<bool>(*this, "publish_status");
  rescue::declareRequired<std::string>(*this, "frame_id");
  declareFieldParameters(*this);
}

// 在加载后端前校验预构建引擎、推理选项与已解析的场地参数。
DetectorConfig DetectNode::loadDetectorConfig() const
{
  DetectorConfig config;
  const auto task = get_parameter("task").as_string();
  if (!parseTask(task, config.task)) {
    throw rescue::ConfigurationError("task must be seg, field or both; got '" + task + "'");
  }
  config.seg_model = configuredPath(*this, "seg_model_path");
  config.seg_labels = configuredPath(*this, "seg_labels_path");
#if VISION_ENABLE_TENSORRT
  // 生产入口只复用既有引擎；ONNX 导出/构建属于独立离线开发工具。
  if (config.task != DetectTask::Field) {
    std::error_code ec;
    if (config.seg_model.empty()) {
      throw rescue::ConfigurationError("seg_model_path is empty but task=" +
        std::string(toString(config.task)) + " needs a prebuilt .trt/.engine file");
    }
    if (!std::filesystem::is_regular_file(config.seg_model, ec)) {
      throw rescue::ConfigurationError("segmentation model not found: " + config.seg_model +
        " (seg_model_path='" + get_parameter("seg_model_path").as_string() + "').");
    }
  }
#endif
  if (!config.seg_labels.empty()) {
    std::error_code ec;
    if (!std::filesystem::is_regular_file(config.seg_labels, ec)) {
      RCLCPP_WARN(get_logger(), "seg_labels_path %s not found; labels will show class ids only",
                  config.seg_labels.c_str());
      config.seg_labels.clear();
    }
  }
  config.dla_core = checkedInt(*this, "dla_core", -1, std::numeric_limits<int>::max());
  config.conf_seg = static_cast<float>(checkedDouble(*this, "conf_threshold_seg", 0, 1));
  config.iou = static_cast<float>(checkedDouble(*this, "nms_threshold", 0, 1));
  config.max_detections = checkedInt(*this, "max_detections", 1, 10000);
  config.mask_scale = checkedInt(*this, "mask_scale", 1, 64);
  config.mask_label_offset = checkedInt(*this, "mask_label_offset", 1, 255);
  // 掩膜预算与检测数量独立；预算为 0 不会删除检测框，请求标签图时仍可产生同帧背景图。
  config.max_masks = checkedInt(*this, "max_masks", 0, 10000);
  config.field = readFieldParameters(*this);
  return config;
}

// 后台线程启动前装载并验证相机/发布配置，最后注册有限的运行参数更新入口。
void DetectNode::loadParameters()
{
  publish_masks_ = get_parameter("publish_masks").as_bool();
  publish_field_results_ = get_parameter("publish_field_results").as_bool();

  publish_raw_ = get_parameter("publish_raw").as_bool();
  publish_processed_ = get_parameter("publish_processed").as_bool();
  // 仅限制 image_raw；每次识别结果对应的 image_processed 不受这道上限限制。
  raw_max_fps_ = checkedDouble(*this, "raw_max_fps", 0, std::numeric_limits<double>::max());

  // 像素尺寸、帧/秒及驱动缓冲张数分别校验；0 的含义由 CameraConfig/后端约定，不能混成同一单位。
  camera_config_.device = get_parameter("camera_device").as_string();
  camera_config_.width = checkedInt(*this, "camera_width", 0, 16384);
  camera_config_.height = checkedInt(*this, "camera_height", 0, 16384);
  camera_config_.fps = checkedInt(*this, "camera_fps", 0, 1000);
  camera_config_.brightness = checkedInt(*this, "camera_brightness", -1, 255);
  camera_config_.fourcc = get_parameter("camera_fourcc").as_string();
  camera_config_.buffer_size =
    checkedInt(*this, "camera_buffer_size", 0, 64);
  camera_config_.backend = get_parameter("camera_backend").as_string();
  camera_config_.pipeline = get_parameter("camera_pipeline").as_string();
  // file 后端把图片／视频循环当作相机，可用于无摄像头的板端验证；不验证真实相机协商。
  if (camera_config_.backend != "v4l2" && camera_config_.backend != "gstreamer" &&
    camera_config_.backend != "file")
  {
    throw rescue::ConfigurationError("camera_backend must be v4l2, gstreamer or file; got '" +
      camera_config_.backend + "'");
  }
  if (camera_config_.brightness >= 0 &&
    (camera_config_.backend != "gstreamer" || !camera_config_.pipeline.empty()))
  {
    RCLCPP_WARN(get_logger(), "camera_brightness only applies to the automatic GStreamer pipeline");
  }
  // 统一入口之外的独立role/组件同样检查；仅校验实际会消费该格式的后端。
  // 自动GStreamer的caps格式名可为RGB/GRAY8，不能误套V4L2的4字节约束。
  if (camera_config_.backend == "gstreamer" && camera_config_.pipeline.empty() &&
    camera_config_.fourcc.empty())
  {
    throw rescue::ConfigurationError("camera_fourcc automatic GStreamer format must be nonempty");
  }
  const bool needs_fourcc = camera_config_.backend == "v4l2" && !camera_config_.fourcc.empty();
  if (needs_fourcc && (camera_config_.fourcc.size() != 4 ||
    !std::all_of(camera_config_.fourcc.begin(), camera_config_.fourcc.end(),
      [](unsigned char character) {return character >= 0x20 && character <= 0x7e;})))
  {
    throw rescue::ConfigurationError("camera_fourcc must contain four printable ASCII bytes");
  }

  publish_timing_ = get_parameter("publish_timing").as_bool();
  const double seg_rate = get_parameter("seg_max_fps").as_double();
  const double field_rate = get_parameter("field_max_fps").as_double();
  if (!std::isfinite(seg_rate) || seg_rate < 0.0 ||
    !std::isfinite(field_rate) || field_rate < 0.0)
  {
    throw rescue::ConfigurationError("seg_max_fps and field_max_fps must be finite and >= 0");
  }
  runtime_->seg_max_fps_.store(seg_rate);
  runtime_->field_max_fps_.store(field_rate);

  mode_param_cb_ = add_on_set_parameters_callback(
    [this](const std::vector<rclcpp::Parameter> & parameters) {
      return onParameters(parameters);
    });
  publish_status_ = get_parameter("publish_status").as_bool();
  frame_id_ = get_parameter("frame_id").as_string();
}

// 配置要求的后端必须真实可用；不将 both 静默降级为 field。
std::optional<DetectorConfig> DetectNode::initializeDetector()
{
  DetectorConfig detector_config = loadDetectorConfig();
#if VISION_ENABLE_TENSORRT
  // 分割只读自动管线留在 NVMM 上的 I420 或 BGRx。file、V4L2 和自定义管线没有这份契约。
  if (detector_config.task != DetectTask::Field &&
    !(camera_config_.backend == "gstreamer" && camera_config_.pipeline.empty()))
  {
    throw rescue::ConfigurationError(
      "segmentation reads NVMM from the automatic GStreamer pipeline (MJPEG I420 or raw BGRx); "
      "camera_backend must be gstreamer and camera_pipeline must be empty");
  }
#endif
  const auto context = get_node_base_interface()->get_context();
  detector_config.cancel_requested = [context] {return !rclcpp::ok(context);};

  std::string error;
  std::vector<std::string> notes;
  bool cancelled = false;
  runtime_->detector_ = Detector::create(detector_config, error, notes, &cancelled);
  if (cancelled) {
    RCLCPP_INFO(get_logger(), "detector initialization cancelled before camera workers started");
    return std::nullopt;
  }
  if (!runtime_->detector_) {
    RCLCPP_ERROR(get_logger(), "%s", error.c_str());
    throw std::runtime_error("detect_node: requested detection backend failed to initialize");
  }
  for (const auto & note : notes) {
    if (note.find("did not succeed") != std::string::npos ||
      note.find("is empty") != std::string::npos)
    {
      RCLCPP_WARN(get_logger(), "%s", note.c_str());
    } else {
      RCLCPP_INFO(get_logger(), "%s", note.c_str());
    }
  }

  return detector_config;
}

// ---- DetectNode：ROS 接口装配 ----

// 输出按已加载后端和发布开关创建；图像/业务结果队列深度为 1。
// 只有阶段通知采用可靠且持久保留的 QoS；诊断状态/耗时仍采用 SensorDataQoS。
void DetectNode::createRosOutputs()
{
  if (runtime_->detector_->hasSeg()) {
    seg_det_pub_ = create_publisher<vision_msgs::msg::Detection2DArray>(
      "~/seg/detections", rclcpp::SensorDataQoS().keep_last(1));
    if (publish_masks_) {
      seg_mask_pub_ =
        create_publisher<sensor_msgs::msg::Image>("~/seg/masks", rclcpp::SensorDataQoS().keep_last(1));
    }
    if (publish_timing_) {
      seg_timing_pub_ = create_publisher<interface_pkg::msg::InferenceTiming>(
        "~/seg/timing", rclcpp::SensorDataQoS());
    }
  }
  if (runtime_->detector_->hasField()) {
    if (publish_field_results_) {
      field_results_pub_ = create_publisher<interface_pkg::msg::FieldObservation>(
        "~/field/results", rclcpp::SensorDataQoS().keep_last(1));
    }
    if (publish_timing_) {
      field_timing_pub_ = create_publisher<interface_pkg::msg::InferenceTiming>(
        "~/field/timing", rclcpp::SensorDataQoS());
    }
  }

  if (publish_raw_) {
    raw_pub_ = create_publisher<sensor_msgs::msg::Image>("~/image_raw",
        rclcpp::SensorDataQoS().keep_last(1));
  }
  if (publish_processed_) {
    processed_pub_ = create_publisher<sensor_msgs::msg::Image>("~/image_processed",
        rclcpp::SensorDataQoS().keep_last(1));
  }

  if (publish_status_) {
    status_pub_ = create_publisher<interface_pkg::msg::DetectorStatus>(
      "~/status", rclcpp::SensorDataQoS());
  }

  rclcpp::PublisherOptions mode_options;
  // 阶段使用 transient_local 供晚加入者读取；关闭其进程内投递以兼容 Humble。
  mode_options.use_intra_process_comm = rclcpp::IntraProcessSetting::Disable;
  mode_pub_ = create_publisher<interface_pkg::msg::TaskState>(
    "~/active_task", rclcpp::QoS(1).reliable().transient_local(), mode_options);
  // 阶段重发不受 publish_status 控制；墙钟继续运行，ROS 时钟尚为零或回退时也能等待恢复。
  status_timer_ = create_wall_timer(std::chrono::seconds(1), [this] {
      runtime_->workers_.rethrowIfFailed();
      refreshActiveTask();
      publishStatus();
    });
}

// 将布尔服务映射到 active_task 参数更新，使服务和参数入口共用校验。
void DetectNode::createStageService()
{
  // 服务只确认阶段已受理；新阶段是否产出有效结果由后续消息确认。
  // 服务与参数接口共用同一校验路径。true = field, false = seg.
  set_field_service_ = create_service<std_srvs::srv::SetBool>(
    "~/set_field",
    [this](const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
      const std::string task = request->data ? "field" : "seg";
      const auto result = set_parameters_atomically({rclcpp::Parameter("active_task", task)});
      response->success = result.successful;
      response->message = result.successful ? "active_task=" + task : result.reason;
    });
}

// ---- DetectNode：阶段更新与帧准入 ----

// 先校验整批参数，再切换阶段和更新限速；不接受其他启动配置的热更新。
rcl_interfaces::msg::SetParametersResult DetectNode::onParameters(
  const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = false;
  const rclcpp::Parameter * requested_task = nullptr;
  double seg_rate = runtime_->seg_max_fps_.load();
  double field_rate = runtime_->field_max_fps_.load();

  // 整批参数先检查完，再改工作线程的状态。
  for (const auto & parameter : parameters) {
    const std::string & name = parameter.get_name();
    if (name == "active_task") {
      if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_STRING) {
        result.reason = "active_task must be a string";
        return result;
      }
      requested_task = &parameter;
      continue;
    }
    if (name != "seg_max_fps" && name != "field_max_fps") {
      result.reason = name + " is startup-only; restart the node to change it";
      return result;
    }
    if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE) {
      result.reason = name + " must be a double";
      return result;
    }
    const double value = parameter.as_double();
    if (!std::isfinite(value) || value < 0.0) {
      result.reason = name + " must be finite and >= 0 (0 means no limit)";
      return result;
    }
    if (name == "seg_max_fps") {
      seg_rate = value;
    } else {
      field_rate = value;
    }
  }

  // 启动和运行中共用同一套阶段解析。切换被拒绝时，两个帧率限制都保持原值。
  if (requested_task && !applyActiveTask(requested_task->as_string(), result.reason)) {
    return result;
  }
  runtime_->seg_max_fps_.store(seg_rate);
  runtime_->field_max_fps_.store(field_rate);
  result.successful = true;
  return result;
}

bool DetectNode::segActive() const
{
  return (runtime_->stage_.active() & 1) != 0 && runtime_->detector_ &&
         runtime_->detector_->hasSeg();
}

bool DetectNode::fieldActive() const
{
  return (runtime_->stage_.active() & 2) != 0 && runtime_->detector_ &&
         runtime_->detector_->hasField();
}

// 在发布/采集闸门内切换阶段并清槽，之后公布稳定的源时间截止与代次。
// 同阶段重复请求仍通知当前状态，但 StageGate 不因此推进已有截止或清空工作。
bool DetectNode::applyActiveTask(const std::string & value, std::string & reason)
{
  int mask = 0;
  if (value == "seg") {
    mask = 1;
  } else if (value == "field") {
    mask = 2;
  } else {
    reason = "active_task must be seg or field; got '" + value + "'";
    return false;
  }
  if (runtime_->detector_ && (((mask & 1) && !runtime_->detector_->hasSeg()) ||
    ((mask & 2) && !runtime_->detector_->hasField())))
  {
    reason = "active_task requests an engine that is not loaded";
    return false;
  }
  // StageGate 管理采集与发布顺序；节点负责 ROS 消息适配。
  runtime_->stage_.transition(mask, [this] {return now().nanoseconds();}, [&] {
      runtime_->clearPendingFrames();
    }, [&](std::int64_t stamp_ns, std::uint64_t epoch) {
      publishTaskState(mask, stamp_ns, epoch);
    });
  return true;
}

void DetectNode::applyModeParameters()
{
  const std::string requested = get_parameter("active_task").as_string();
  std::string reason;
  if (!applyActiveTask(requested, reason)) {
    throw rescue::ConfigurationError(reason);
  }
}

// 时钟回退期间的阶段通知留待墙钟刷新；固定源时间避免重发被下游视为新会话。
void DetectNode::publishTaskState(int mask, std::int64_t stamp_ns, std::uint64_t epoch)
{
  if (!mode_pub_ || stamp_ns <= 0 || stamp_ns > now().nanoseconds()) {return;}
  interface_pkg::msg::TaskState message;
  message.header.stamp = rclcpp::Time(stamp_ns);
  message.task = mask == 1 ? "seg" : "field";
  // epoch 标识阶段代次；重发沿用原 stamp/epoch，供下游区分通知重复与新会话。
  message.epoch = epoch;
  mode_pub_->publish(message);
}

// 与状态采样开关无关，保证模拟时钟从零启动、回退后追上时仍能恢复阶段通知。
void DetectNode::refreshActiveTask()
{
  if (!rclcpp::ok() || !mode_pub_) {return;}
  runtime_->stage_.refresh([this] {return now().nanoseconds();}, [&] {
      runtime_->clearPendingFrames();
    }, [this](int mask, std::int64_t stamp_ns, std::uint64_t epoch) {
      publishTaskState(mask, stamp_ns, epoch);
    });
}

// 提前检查减少无效工作；发布路径仍在闸门锁内再次调用，以阻断切换/重连竞争。
bool DetectNode::frameIsCurrent(const Frame & frame, int task_bit) const
{
  if (!runtime_->workers_.running() ||
    !runtime_->stage_.current(frame.generation, frame.camera_session))
  {
    return false;
  }
  if (task_bit == 1) {return segActive();}
  if (task_bit == 2) {return fieldActive();}
  // 原图路径已核对阶段/相机会话，还要求源时间不早于当前阶段起点。
  return frame.stamp.nanoseconds() >= runtime_->stage_.cutoff();
}

// ---- DetectNode：同帧消息准备与最终发布 ----

std::unique_ptr<sensor_msgs::msg::Image> DetectNode::prepareProcessed(
  const Frame & frame, const std_msgs::msg::Header & header)
{
  // 有订阅者时为本次取出的结果准备实际输入帧；最终发布仍须通过阶段/相机会话复核。
  // raw_max_fps 只限制原图，不对这一路处理帧另行限速。
  if (!processed_pub_ || processed_pub_->get_subscription_count() == 0) {return nullptr;}
  return vision_ros::makeBgrImageMessage(frame.image, header);
}

// 锁外转换完整同帧消息，锁内复核阶段/相机会话并移交所有权。
void DetectNode::publishSeg(const Frame & frame, const SegRun & run)
{
  if (!frameIsCurrent(frame, 1)) {return;}
  const auto t0 = std::chrono::steady_clock::now();
  // 分割消息在锁外准备；闸门内按处理帧 → 检测 → 掩膜 → 耗时发布。
  std_msgs::msg::Header header;
  header.stamp = frame.stamp;
  header.frame_id = frame_id_;

  // 锁外：处理帧（可选）、检测框与标签掩膜全部先组好。
  // 处理帧与结果使用同一 header；跨话题到达顺序不保证，消费方仍须缓存并按帧配对。
  auto processed = prepareProcessed(frame, header);
  std::unique_ptr<vision_msgs::msg::Detection2DArray> detections;
  if (seg_det_pub_ && seg_det_pub_->get_subscription_count() > 0) {
    detections = std::make_unique<vision_msgs::msg::Detection2DArray>(
      toDetection2DArray(run.detections, header));
  }
  std::unique_ptr<sensor_msgs::msg::Image> mask;
  if (seg_mask_pub_ && seg_mask_pub_->get_subscription_count() > 0 &&
    !run.label_mask.empty())
  {
    mask = std::make_unique<sensor_msgs::msg::Image>(toLabelMaskImage(run.label_mask, header));
  }

  {
    // 发布闸门：推理期间可能已切换阶段，旧代次结果在这里整体丢弃（含处理帧与耗时消息）。
    auto gate = runtime_->stage_.lockPublication();
    if (!frameIsCurrent(frame, 1)) {return;}
    if (processed) {processed_pub_->publish(std::move(processed));}
    if (detections) {seg_det_pub_->publish(std::move(detections));}
    if (mask) {seg_mask_pub_->publish(std::move(mask));}
    const double publish_ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - t0).count();

    if (seg_timing_pub_ && seg_timing_pub_->get_subscription_count() > 0) {
      interface_pkg::msg::InferenceTiming timing;
      timing.header = header;
      timing.gpu_ms = run.gpu_ms;
      timing.post_ms = run.post_ms;
      timing.inference_ms = run.inference_ms;
      timing.publish_ms = publish_ms;
      // 仅推理加本次转换/发布耗时；不含驱动排队、邮箱等待和订阅端接收，不能作为端到端延迟。
      timing.total_ms = run.inference_ms + publish_ms;
      timing.instances = run.detections.size();
      seg_timing_pub_->publish(timing);
    }
  }
  runtime_->seg_meter_.record(run.inference_ms);
  runtime_->seg_gpu_ms_.store(run.gpu_ms);
  runtime_->seg_post_ms_.store(run.post_ms);
}

// 场地观测与处理帧共用源图 header；仅 ok 且 committed 的可用几何能成为有效观测。
// 状态名或历史角点本身不能代替提交标志，候选和无效帧仍保留各自诊断字段。
void DetectNode::publishField(const Frame & frame, const FieldResult & run)
{
  if (!frameIsCurrent(frame, 2)) {return;}
  const auto t0 = std::chrono::steady_clock::now();
  // 纯算法结果先经 ROS 转换；闸门内按处理帧 → 观测 → 耗时发布。
  std_msgs::msg::Header header;
  header.stamp = frame.stamp;
  header.frame_id = frame_id_;
  // 与 seg 阶段相同：结果消息用的那一帧与结果一起发出，两者 header 逐字段一致。
  auto processed = prepareProcessed(frame, header);
  std::unique_ptr<interface_pkg::msg::FieldObservation> observation;
  if (field_results_pub_ && field_results_pub_->get_subscription_count() > 0) {
    observation = std::make_unique<interface_pkg::msg::FieldObservation>(
      toFieldObservation(run, runtime_->detector_->config().field, frame.image.size(), header));
  }
  {
    auto gate = runtime_->stage_.lockPublication();
    if (!frameIsCurrent(frame, 2)) {return;}
    if (processed) {processed_pub_->publish(std::move(processed));}
    if (observation) {field_results_pub_->publish(std::move(observation));}
    const double publish_ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - t0).count();
    if (field_timing_pub_ && field_timing_pub_->get_subscription_count() > 0) {
      interface_pkg::msg::InferenceTiming timing;
      timing.header = header;
      timing.gpu_ms = 0.0;
      timing.post_ms = run.ms_warp + run.ms_split;
      timing.inference_ms = run.ms_total;
      timing.publish_ms = publish_ms;
      // 与分割相同，这个和不含源帧到观察者之间的全部排队与传输时间。
      timing.total_ms = run.ms_total + publish_ms;
      timing.instances = run.ok && run.committed ? 1 : 0;
      field_timing_pub_->publish(timing);
    }
  }
  runtime_->field_meter_.record(run.ms_total);
  runtime_->field_post_ms_.store(run.ms_warp + run.ms_split);
}

void DetectNode::publishRaw(const Frame & frame)
{
  // 没有订阅者就不做图像转换，然后再套帧率限制。
  if (!raw_pub_ || raw_pub_->get_subscription_count() == 0) {return;}

  if (!frameIsCurrent(frame, 0) ||
    !runtime_->raw_limiter_.allow(std::chrono::steady_clock::now(), raw_max_fps_)) {return;}

  std_msgs::msg::Header header;
  header.stamp = frame.stamp;
  header.frame_id = frame_id_;
  // 锁外完成唯一一次整帧拷贝；闸门内只核对代次并移交所有权。
  auto message = vision_ros::makeBgrImageMessage(frame.image, header);
  // 原图同样按代次过滤，切换后已排队的旧帧不再发布，避免图传画面回跳。
  auto gate = runtime_->stage_.lockPublication();
  if (!frameIsCurrent(frame, 0)) {return;}
  raw_pub_->publish(std::move(message));
  runtime_->raw_published_.fetch_add(1);
}

// ---- 状态与部署诊断 ----

std::size_t DetectNode::rawIntraProcessSubscribers() const
{
  return raw_pub_ ? raw_pub_->get_intra_process_subscription_count() : 0;
}

std::size_t DetectNode::processedIntraProcessSubscribers() const
{
  return processed_pub_ ? processed_pub_->get_intra_process_subscription_count() : 0;
}

void DetectNode::publishStatus()
{
  if (!rclcpp::ok() || !status_pub_) {return;}

  const double capture_fps = runtime_->capture_window_.rate(runtime_->captured_frames_.load());
  const double seg_fps = runtime_->seg_window_.rate(runtime_->seg_meter_.count());
  const double field_fps = runtime_->field_window_.rate(runtime_->field_meter_.count());
  const double raw_fps = runtime_->raw_window_.rate(runtime_->raw_published_.load());

  interface_pkg::msg::DetectorStatus status;
  status.header.stamp = now();
  status.header.frame_id = frame_id_;
  status.capture_fps = capture_fps;
  status.seg_fps = seg_fps;
  status.field_fps = field_fps;
  status.seg_inference_ms = runtime_->seg_meter_.latencyMs();
  status.field_inference_ms = runtime_->field_meter_.latencyMs();
  status.raw_fps = raw_fps;
  status.raw_subscribers = raw_pub_ ? raw_pub_->get_subscription_count() : 0;
  status.inference_replaced = runtime_->inference_slot_.overwritten();
  status.raw_replaced = runtime_->raw_slot_.overwritten();
  status.camera_reopens = runtime_->camera_reopens_.load();
  status.seg_gpu_ms = runtime_->seg_gpu_ms_.load();
  status.seg_post_ms = runtime_->seg_post_ms_.load();
  status.field_gpu_ms = 0.0;
  status.field_post_ms = runtime_->field_post_ms_.load();
  status.seg_skipped = runtime_->seg_skipped_.load();
  status.field_skipped = runtime_->field_skipped_.load();
  status.seg_active = segActive();
  status.field_active = fieldActive();
  status.serial_inference = true;
  status_pub_->publish(status);

  RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
                       "capture %.1f fps | seg %.1f fps %.1f ms (gpu %.1f post %.1f) | "
                       "field %.1f fps %.1f ms (gpu %.1f post %.1f) | "
                       "raw %.1f fps to %lu | replaced %lu/%lu | result replaced %lu",
                       capture_fps, seg_fps, runtime_->seg_meter_.latencyMs(),
                       runtime_->seg_gpu_ms_.load(), runtime_->seg_post_ms_.load(),
                       field_fps, runtime_->field_meter_.latencyMs(), 0.0,
                       runtime_->field_post_ms_.load(), raw_fps,
                       static_cast<unsigned long>(
                         raw_pub_ ? raw_pub_->get_subscription_count() : 0),
                       static_cast<unsigned long>(runtime_->inference_slot_.overwritten()),
                       static_cast<unsigned long>(runtime_->raw_slot_.overwritten()),
                       static_cast<unsigned long>(runtime_->result_slot_.overwritten()));
}


// 采集、推理、原图和结果线程在 detect_workers.cpp。连续读失败次数也在那里。

RCLCPP_COMPONENTS_REGISTER_NODE(DetectNode)
