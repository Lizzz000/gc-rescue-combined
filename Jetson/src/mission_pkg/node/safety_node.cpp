#include "node/safety_node.hpp"
#include "field/field_geometry.hpp"


// ---- 任务节点实现 ----
// 阅读：只在 Mission 已提交目的区、且实际阶段是 field 时计算中心。
// 左还是右只看 MissionStatus，本文件不改口。
// 四边形是否可用在 field/field_geometry.hpp。
// Copyright 2026 li
// SPDX-License-Identifier: Apache-2.0

#include <chrono>
#include <exception>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <interface_pkg/app/core.hpp>
#include <utility>
#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <memory>
#include <rclcpp_components/register_node_macro.hpp>

// ---- SafetyNode：构造与参数 ----

// 接收场地观测，按 Mission 已提交的目的区生成安全中心；四角等权且不裁剪。

SafetyNode::SafetyNode(const rclcpp::NodeOptions & options)
: Node("safety_node", options), settings_(loadParameters())
{
  try {
    frame_gate_.setTimeout(settings_.reset_timeout_s);
  } catch (const std::invalid_argument & error) {
    throw rescue::ConfigurationError(error.what());
  }
  createRosInterfaces();
}

// 参数必须由装配层显式提供；本节点复核尺寸与秒制超时，防止非法值进入帧门控。
SafetyNode::Settings SafetyNode::loadParameters()
{
  rcl_interfaces::msg::ParameterDescriptor descriptor;
  descriptor.read_only = true;
  const auto input_topic = rescue::declareRequired<std::string>(
    *this, "input_topic", descriptor);
  const auto control_topic = rescue::declareRequired<std::string>(
    *this, "control_topic", descriptor);
  const auto reset_timeout_s = rescue::declareRequired<double>(
    *this, "reset_timeout_s", descriptor);
  const auto image_width = rescue::declareRequired<std::int64_t>(
    *this, "image_width", descriptor);
  const auto image_height = rescue::declareRequired<std::int64_t>(
    *this, "image_height", descriptor);
  const auto status_topic = rescue::declareRequired<std::string>(
    *this, "mission_status_topic", descriptor);
  if (image_width <= 0 || image_height <= 0 ||
    image_width > std::numeric_limits<int>::max() ||
    image_height > std::numeric_limits<int>::max())
  {
    throw rescue::ConfigurationError("invalid safety image dimensions");
  }
  return {input_topic, control_topic, status_topic, reset_timeout_s, image_width, image_height};
}

// ---- SafetyNode：ROS 接口 ----

// 阶段/目的区采用可靠且保留最近状态的 QoS，场地帧与输出只保留最新一条。
// 未指定其他回调组，订阅与超时定时器均在默认互斥组中串行访问状态。
void SafetyNode::createRosInterfaces()
{
  output_pub_ = create_publisher<interface_pkg::msg::FieldObservation>(
    "~/observation", rclcpp::SensorDataQoS().keep_last(1));
  target_pub_ = create_publisher<interface_pkg::msg::SafetyTarget>(
    "~/target", rclcpp::SensorDataQoS().keep_last(1));
  destination_sub_ = create_subscription<interface_pkg::msg::MissionStatus>(
    settings_.mission_status_topic, rclcpp::QoS(1).reliable().transient_local(),
    [this](interface_pkg::msg::MissionStatus::ConstSharedPtr msg) {destinationCallback(msg);});
  mode_sub_ = create_subscription<interface_pkg::msg::TaskState>(
    settings_.control_topic, rclcpp::QoS(1).reliable().transient_local(),
    [this](interface_pkg::msg::TaskState::ConstSharedPtr msg) {modeCallback(msg);});
  input_sub_ = create_subscription<interface_pkg::msg::FieldObservation>(
    settings_.input_topic, rclcpp::SensorDataQoS().keep_last(1),
    [this](interface_pkg::msg::FieldObservation::ConstSharedPtr msg) {observationCallback(msg);});
  // 每 100 毫秒检查一次；实际断流期限仍由 FrameGate 的单调时间决定。
  timeout_timer_ = create_wall_timer(
    std::chrono::milliseconds(100),
    [this] {checkTimeout();});
}

// ---- SafetyNode：会话、目的区与观测事件 ----

// 实际 field、导航就绪与目的区源时间截止共同约束目标准入。

void SafetyNode::modeCallback(const interface_pkg::msg::TaskState::ConstSharedPtr & msg)
{
  std::int64_t stamp;
  if (!RecognitionValidation::sourceTime(msg->header.stamp, now().nanoseconds(), stamp)) {
    return;
  }
  // 先校验 ROS 时间再判断会话；未知任务名称仍保存，但不启用场地输出。
  recognition::TaskSession next{msg->task, msg->epoch, stamp};
  if (!recognition::isNewSession(session_, next)) {return;}
  session_ = std::move(next);
  enabled_ = session_->task == interface_pkg::msg::TaskState::TASK_FIELD;
  // 保留较新的超时或任务截止时间。
  frame_gate_.clear(stamp);
  clearOutput(msg->header.stamp);
}

void SafetyNode::destinationCallback(
  const interface_pkg::msg::MissionStatus::ConstSharedPtr & msg)
{
  // 所有 MissionStatus 枚举先校验再更新水位；非法运输阶段也不能挡住后续合法状态。
  std::int64_t stamp;
  if (!RecognitionValidation::missionStatusEnums(*msg) ||
    !RecognitionValidation::sourceTime(msg->header.stamp, now().nanoseconds(), stamp)) {return;}
  // 同一个 ROS 时刻可以先 PENDING 后 SUCCESS；只拒绝更旧状态，
  // 不能按源帧去重方式过滤相等时间，否则会丢掉完成确认。
  if (stamp < status_stamp_ns_) {return;}
  status_stamp_ns_ = stamp;
  // 导航安全区、空闲、成功且目的区非空才放行中心；首趟/后续不参与此处路由选择。
  const bool ready =
    msg->cycle_phase == interface_pkg::msg::MissionStatus::CYCLE_NAVIGATING_SAFETY &&
    msg->operation == interface_pkg::msg::MissionStatus::OP_IDLE &&
    msg->outcome == interface_pkg::msg::MissionStatus::RESULT_SUCCESS &&
    msg->destination != interface_pkg::msg::MissionStatus::DEST_NONE;
  // 已就绪的相同目的区/请求号重发不改变有效目标或路由截止。
  // 无目的区始终撤销目标；请求号或就绪性改变，即使左右不变也重新设截止。
  if (msg->destination != destination_ ||
    msg->destination == interface_pkg::msg::MissionStatus::DEST_NONE ||
    msg->roi_request_id != destination_request_id_ || ready != destination_ready_ ||
    msg->cycle_generation != cycle_generation_ || msg->cycle_phase != cycle_phase_)
  {
    cycle_generation_ = msg->cycle_generation;
    cycle_phase_ = msg->cycle_phase;
    destination_ = msg->destination;
    destination_request_id_ = msg->roi_request_id;
    destination_ready_ = ready;
    destination_cutoff_ns_ = stamp;
    clearTarget(msg->header.stamp);
  }
}

void SafetyNode::observationCallback(
  const interface_pkg::msg::FieldObservation::ConstSharedPtr & msg)
{
  if (!enabled_) {return;}
  const auto current = now().nanoseconds();
  std::int64_t stamp;
  if (!RecognitionValidation::sourceTime(msg->header.stamp, current, stamp)) {return;}
  // 阶段/断流截止、源帧年龄及严格递增检查都在复制和发布之前完成。
  if (!frame_gate_.accepts(stamp, current)) {return;}
  auto message = std::make_unique<interface_pkg::msg::FieldObservation>(*msg);
  sanitizeObservation(*message);
  const auto target = makeSafetyTarget(*message, stamp);
  // 顺序是观测 → 目标 → 历史；两次发布均返回后才记录此帧。
  // 发布异常向外传播，不能提前更新历史而把未完整发布的帧当作已完成。
  output_pub_->publish(std::move(message));
  target_pub_->publish(target);
  last_header_ = msg->header;
  frame_gate_.record(stamp, msg->header.frame_id, FrameGate::Clock::now());
}

// ---- SafetyNode：几何与目标输出 ----

// 候选只作诊断；无效正式几何仍须转发清空消息以撤销旧结果。
void SafetyNode::sanitizeObservation(interface_pkg::msg::FieldObservation & output)
{
  const bool source_valid = output.valid;
  const bool valid = FieldGeometry::validObservation(output);
  if (!valid) {
    // 撤销正式几何及提交标志，保留其余诊断；上游本来无效时沿用原失败原因。
    output.valid = false;
    output.committed = false;
    output.switched = false;
    output.corners.clear();
    output.left_region.clear();
    output.right_region.clear();
    output.split_line.clear();
    if (source_valid) {output.reason = "invalid mission observation";}
  }
}

interface_pkg::msg::SafetyTarget SafetyNode::makeSafetyTarget(
  const interface_pkg::msg::FieldObservation & output, std::int64_t stamp) const
{
  interface_pkg::msg::SafetyTarget target;
  target.header = output.header;
  target.side = destination_;
  // 模式切换服务成功不代表几何已就绪。四项必须同时成立才计算中心：
  // 本帧正式几何有效、任务处于前往安全区且空闲成功、目的区不是未定、源图时间不早于该次状态。
  // 比较的两端都是 ROS 时间；帧的单调到达时间只用于断流，不参与路由截止。
  // 运输阶段不参与这里。左/右只使用 Mission 已经提交的 destination，本节点不改口。
  if (output.valid && destination_ready_ &&
    destination_ != interface_pkg::msg::MissionStatus::DEST_NONE &&
    stamp >= destination_cutoff_ns_)
  {
    const auto & points = destination_ == interface_pkg::msg::MissionStatus::DEST_LEFT ?
      output.left_region : output.right_region;
    FieldGeometry::detail::normalizedCenter(
      points, settings_.image_width, settings_.image_height, target.x, target.y);
    target.valid = true;
  }
  // 无效结果仍携带源 header 和当前 side，但坐标归零，接收端必须检查 valid。
  if (!target.valid) {target.x = 0; target.y = 0;}
  return target;
}

// ---- SafetyNode：输出撤销与超时清理 ----

// 路由变化立即撤销旧中心；新的有效中心必须来自之后收到的正式场地帧。
void SafetyNode::clearTarget(const builtin_interfaces::msg::Time & stamp)
{
  interface_pkg::msg::SafetyTarget empty;
  empty.header = last_header_;
  empty.header.stamp = stamp;
  empty.side = destination_;
  target_pub_->publish(empty);
}

// 正式观测和区域目标一起清空，保留源时间截止以拒绝后续排队到达的旧帧。
void SafetyNode::clearOutput(const builtin_interfaces::msg::Time & stamp)
{
  interface_pkg::msg::FieldObservation empty;
  empty.header = last_header_;
  empty.header.stamp = stamp;
  empty.reason = "mission disabled, transitioned or input timed out";
  output_pub_->publish(empty);
  clearTarget(stamp);
  // 保留已有截止，只撤销已接收帧历史，避免超时定时器反复发送清空消息。
  frame_gate_.clear();
}

void SafetyNode::checkTimeout()
{
  // steady_clock 判定断流；撤销消息和新源帧截止则使用此刻 ROS 时间。
  // 截止只前移，恢复后排队到达的旧帧不能重新激活已经撤销的中心。
  if (frame_gate_.expired(FrameGate::Clock::now())) {
    const auto cutoff = now();
    clearOutput(static_cast<builtin_interfaces::msg::Time>(cutoff));
    frame_gate_.clear(cutoff.nanoseconds());
  }
}

RCLCPP_COMPONENTS_REGISTER_NODE(SafetyNode)
