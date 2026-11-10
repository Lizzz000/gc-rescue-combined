#include "node/bridge_node.hpp"
#include "transport/socketcan/socketcan.hpp"
#include "transport/damiao/damiao.hpp"

// ---- 桥接节点实现 ----
// 职责：装配 ROS 话题和定时器。周期收发在 bridge_io.cpp。
// Copyright 2026 li
// SPDX-License-Identifier: Apache-2.0
// BridgeNode 装配 ROS 边界。BridgeRuntime 持有 CAN 会话，二者共用默认互斥回调组。
// 发送队列在 bridge_session.cpp。帧格式在 protocol/can_protocol.hpp。

#include <interface_pkg/app/core.hpp>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <exception>
#include <linux/can/error.h>

struct BridgeNode::StartupSettings
{
  // 端口名称在构造期固定；本对象只用于一次装配，不保存运行期业务状态。
  std::string target_topic, safety_topic, velocity_topic, task_topic, mission_topic;
  std::string roi_service, command_service;
  // 配置与ROS端口分别移交给Runtime，避免两层重复读取参数。
  std::string transport, serial_device;
  std::int64_t can_bitrate;
  BridgeRuntime::Config runtime;
  BridgeRuntime::Ports ports;
};

// ---- BridgeNode：构造与生命周期 ----

// 参数 → ROS 端口 → 运行上下文 → 定时器；socket 在首次 io() 才尝试打开。

BridgeNode::BridgeNode(const rclcpp::NodeOptions & options)
: BridgeNode(options, nullptr)
{
}

/**
 * @brief 装配节点、传输与状态对象；配置由各接收入口检查一次。
 * @param[in] options ROS 选项和完整显式参数；不提供隐式业务默认值。
 * @param[in] transport 可为空；为空时按 transport 参数创建生产传输，否则接管传入对象。
 * @throws rescue::ConfigurationError 参数非法，构造中止，不注册轮询定时器或打开设备。
 */
BridgeNode::BridgeNode(
  const rclcpp::NodeOptions & options,
  std::unique_ptr<CanTransport> transport)
try : Node("bridge_node", options)
{
  auto settings = declareAndValidateParameters();
  if (!transport) {
    if (settings.transport == "socketcan") {transport = std::make_unique<CanSocket>();}
    else {
      transport = std::make_unique<DamiaoCan>(settings.can_bitrate);
      RCLCPP_WARN(get_logger(), "Damiao uses existing adapter CAN bitrate; expected %lld bit/s is NOT configured or verified",
        static_cast<long long>(settings.can_bitrate));
    }
  }
  createRosInterfaces(settings);
  runtime_ = std::make_unique<BridgeRuntime>(
    *this, std::move(settings.runtime), std::move(settings.ports), std::move(transport));
  // 所有参数与端口均准备好后才注册定时器；构造本身不提前打开 CAN socket。
  createCanTimers();
} catch (const std::invalid_argument & error) {
  // 核心和传输的首次配置检查统一映射到既有配置错误退出语义。
  throw rescue::ConfigurationError(error.what());
}

BridgeNode::~BridgeNode() = default;

// ---- BridgeNode：启动参数 ----

// 先排除空白与空名称；完整 ROS 名称语法由 rclcpp 创建端口时检查。
static bool validTopicName(const std::string & value)
{
  return !value.empty() && value.find_first_of(" \t\n\r\f\v") == std::string::npos;
}

/**
 * @brief 读取显式启动参数，并校验本节点负责的传输、端口和等待期限。
 * 状态尺寸、类别和输入期限由 BridgeState 构造入口检查，避免重复判定同一配置。
 * @return 一次装配使用的设置；缺项或本层配置非法时抛出 ConfigurationError，不创建设备。
 */
BridgeNode::StartupSettings BridgeNode::declareAndValidateParameters()
{
  StartupSettings settings;
  BridgeStateConfig state_config;
  state_config.target_classes.clear();
  rcl_interfaces::msg::ParameterDescriptor descriptor;
  descriptor.read_only = true;
  settings.transport = rescue::declareRequired<std::string>(*this, "transport", descriptor);
  settings.serial_device = rescue::declareRequired<std::string>(*this, "serial_device", descriptor);
  settings.can_bitrate = rescue::declareRequired<std::int64_t>(*this, "can_bitrate", descriptor);
  const auto interface_name = rescue::declareRequired<std::string>(*this, "can_interface", descriptor);
  if (settings.transport == "socketcan") {
    if (!CanSocket::validInterfaceName(interface_name) || !settings.serial_device.empty() ||
      settings.can_bitrate != 0)
    {
      throw rescue::ConfigurationError("SocketCAN requires a valid interface, empty serial_device and can_bitrate=0 (system managed)");
    }
    settings.runtime.can_interface = interface_name;
  } else if (settings.transport == "damiao_serial_fixed") {
    if (!damiao::validDevice(settings.serial_device) || !damiao::validBitrate(settings.can_bitrate)) {
      throw rescue::ConfigurationError("Damiao requires an explicit /dev path and supported CAN bitrate");
    }
    settings.runtime.can_interface = settings.serial_device;
  } else {throw rescue::ConfigurationError("unsupported CAN transport");}
  state_config.image_width = rescue::declareRequired<std::int64_t>(
    *this, "image_width", descriptor);
  state_config.image_height = rescue::declareRequired<std::int64_t>(
    *this, "image_height", descriptor);
  state_config.input_timeout_s = rescue::declareRequired<double>(
    *this, "input_timeout_s", descriptor);
  state_config.velocity_timeout_s = rescue::declareRequired<double>(
    *this, "velocity_timeout_s", descriptor);
  const auto queue_timeout = rescue::declareRequired<double>(
    *this, "tx_queue_timeout_s", descriptor);
  const auto request_timeout = rescue::declareRequired<double>(
    *this, "request_timeout_s", descriptor);
  if (!std::isfinite(queue_timeout) || queue_timeout <= 0 || queue_timeout > 3600 ||
    !std::isfinite(request_timeout) || request_timeout <= 0 || request_timeout > 3600)
  {
    throw rescue::ConfigurationError("invalid CAN interface, queue or request timeout");
  }
  settings.runtime.tx_queue_timeout = std::chrono::duration_cast<Clock::duration>(
    std::chrono::duration<double>(queue_timeout));
  settings.runtime.request_timeout = std::chrono::duration_cast<Clock::duration>(
    std::chrono::duration<double>(request_timeout));
  if (settings.runtime.tx_queue_timeout <= Clock::duration::zero() ||
    settings.runtime.request_timeout <= Clock::duration::zero())
  {
    throw rescue::ConfigurationError("CAN queue or request timeout is below clock resolution");
  }
  settings.target_topic = rescue::declareRequired<std::string>(
    *this, "target_topic", descriptor);
  settings.safety_topic = rescue::declareRequired<std::string>(
    *this, "safety_topic", descriptor);
  settings.velocity_topic = rescue::declareRequired<std::string>(
    *this, "velocity_topic", descriptor);
  settings.task_topic = rescue::declareRequired<std::string>(
    *this, "task_topic", descriptor);
  settings.mission_topic = rescue::declareRequired<std::string>(
    *this, "mission_status_topic", descriptor);
  settings.roi_service = rescue::declareRequired<std::string>(
    *this, "roi_service", descriptor);
  settings.command_service = rescue::declareRequired<std::string>(
    *this, "command_service", descriptor);
  const auto target_class_ids = rescue::declareRequired<std::vector<std::int64_t>>(
    *this, "target_class_ids", descriptor);
  if (!validTopicName(settings.target_topic) || !validTopicName(settings.safety_topic) ||
    !validTopicName(settings.velocity_topic) || !validTopicName(settings.task_topic) ||
    !validTopicName(settings.mission_topic) || !validTopicName(settings.roi_service) ||
    !validTopicName(settings.command_service))
  {
    RCLCPP_ERROR(
      get_logger(),
      "Topic and service parameters must be non-empty names without whitespace: target='%s', "
      "safety='%s', velocity='%s', task='%s', mission_status='%s', roi='%s', command='%s'",
      settings.target_topic.c_str(), settings.safety_topic.c_str(), settings.velocity_topic.c_str(),
      settings.task_topic.c_str(), settings.mission_topic.c_str(), settings.roi_service.c_str(),
      settings.command_service.c_str());
    throw rescue::ConfigurationError("invalid topic parameter");
  }
  for (auto class_id : target_class_ids) {
    state_config.target_classes.push_back(std::to_string(class_id));
  }
  const auto first_class = rescue::declareRequired<std::int64_t>(
    *this, "first_target_class_id", descriptor);
  state_config.first_class = std::to_string(first_class);
  // 状态配置交给 BridgeState 构造入口验证，本层不重复检查相同值。
  settings.runtime.state = std::move(state_config);
  return settings;
}

// ---- BridgeNode：ROS 接口与定时器 ----

// 可靠持久的控制状态与最新值感知数据分开使用原有 QoS；服务回调也归默认互斥组。
void BridgeNode::createRosInterfaces(StartupSettings & settings)
{
  auto & ports = settings.ports;
  ports.status_pub = create_publisher<interface_pkg::msg::BridgeStatus>("~/status", rclcpp::QoS(1));
  target_sub_ = create_subscription<vision_msgs::msg::Detection2DArray>(
    settings.target_topic, rclcpp::SensorDataQoS().keep_last(1),
    [this](vision_msgs::msg::Detection2DArray::ConstSharedPtr msg) {
      runtime_->targetCallback(msg);
    });
  safety_sub_ = create_subscription<interface_pkg::msg::SafetyTarget>(
    settings.safety_topic, rclcpp::SensorDataQoS().keep_last(1),
    [this](interface_pkg::msg::SafetyTarget::ConstSharedPtr msg) {runtime_->safetyCallback(msg);});
  velocity_sub_ = create_subscription<geometry_msgs::msg::Twist>(
    settings.velocity_topic, rclcpp::QoS(1).reliable().durability_volatile(),
    [this](geometry_msgs::msg::Twist::ConstSharedPtr msg) {runtime_->velocityCallback(msg);});
  const auto control_qos = rclcpp::QoS(1).reliable().transient_local();
  task_sub_ = create_subscription<interface_pkg::msg::TaskState>(
    settings.task_topic, control_qos,
    [this](interface_pkg::msg::TaskState::ConstSharedPtr msg) {runtime_->taskCallback(msg);});
  mission_sub_ = create_subscription<interface_pkg::msg::MissionStatus>(
    settings.mission_topic, control_qos,
    [this](interface_pkg::msg::MissionStatus::ConstSharedPtr msg) {
      runtime_->missionCallback(msg);
    });
  ports.roi_client = create_client<interface_pkg::srv::CheckRoi>(settings.roi_service);
  ports.command_client = create_client<interface_pkg::srv::MissionCommand>(settings.command_service);
}

void BridgeNode::createCanTimers()
{
  // 每约 3.33 ms 先处理事件，再轮发一个周期帧；三类帧各约 100 Hz。
  io_timer_ = create_wall_timer(CanPeriodicSchedule::interval, [this] {runtime_->io();});
  status_timer_ = create_wall_timer(std::chrono::seconds(1), [this] {runtime_->publishStatus();});
}


// CAN 会话、到达握手和发送队列在 bridge_session.cpp。

#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE(BridgeNode)
