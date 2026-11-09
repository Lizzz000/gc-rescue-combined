#include "node/target_node.hpp"
#include "roi/ros/roi_ros.hpp"


// ---- 任务节点实现 ----
// 阅读：targetCallback 只转发恰好一个完整的选中目标。
// requestCheck 等到下一张原始检测再做 ROI，不用已经缓存的帧。
// 颜色件数在 roi/roi.cpp。左右目的区不在本文件决定。
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

// ---- TargetNode：构造与参数 ----

// 选中目标用于导航转发，完整原始检测用于 ROI；两路独立维护源帧历史。

TargetNode::TargetNode(const rclcpp::NodeOptions & options)
: Node("target_node", options), settings_(loadParameters())
{
  // 参数先完整读取，ROI 核心构造校验成功后才创建订阅、服务和定时器。
  const auto & roi = settings_.roi;
  try {
    frame_gate_.setTimeout(settings_.reset_timeout_s);
    validator_ = std::make_unique<RoiValidator>(
      settings_.image_width, settings_.image_height, roi.x, roi.y, roi.width, roi.height,
      settings_.class_ids, roiPolicy());
  } catch (const std::invalid_argument & error) {
    // 只转译纯 ROI 构造对启动参数的拒绝，ROS 装配和运行期异常沿原边界传播。
    throw rescue::ConfigurationError(error.what());
  }
  createRosInterfaces();
}

// 秒制超时在此校验；尺寸、矩形与类别交 RoiValidator 构造校验，全部通过后才装配接口。
TargetNode::Settings TargetNode::loadParameters()
{
  rcl_interfaces::msg::ParameterDescriptor descriptor;
  // 生产值由统一配置装配层显式注入；本节点不补默认值，启动后也不允许热改。
  descriptor.read_only = true;
  const auto input_topic = rescue::declareRequired<std::string>(
    *this, "input_topic", descriptor);
  const auto control_topic = rescue::declareRequired<std::string>(
    *this, "control_topic", descriptor);
  const auto reset_timeout_s = rescue::declareRequired<double>(
    *this, "reset_timeout_s", descriptor);
  const auto roi_check_timeout_s = rescue::declareRequired<double>(
    *this, "roi_check_timeout_s", descriptor);
  if (!std::isfinite(roi_check_timeout_s) || roi_check_timeout_s <= 0 ||
    roi_check_timeout_s > 3600) {throw rescue::ConfigurationError("invalid ROI check timeout");}
  const auto image_width = rescue::declareRequired<std::int64_t>(
    *this, "image_width", descriptor);
  const auto image_height = rescue::declareRequired<std::int64_t>(
    *this, "image_height", descriptor);
  const auto x = rescue::declareRequired<double>(
    *this, "roi_x", descriptor);
  const auto y = rescue::declareRequired<double>(
    *this, "roi_y", descriptor);
  const auto width = rescue::declareRequired<double>(
    *this, "roi_width", descriptor);
  const auto height = rescue::declareRequired<double>(
    *this, "roi_height", descriptor);
  // 身份是模型类别编号；颜色参数必须与检测模型实际的类别顺序一致。
  const auto green_class_id = rescue::declareRequired<std::int64_t>(
    *this, "green_class_id", descriptor);
  const auto blue_class_id = rescue::declareRequired<std::int64_t>(
    *this, "blue_class_id", descriptor);
  const auto black_class_id = rescue::declareRequired<std::int64_t>(
    *this, "black_class_id", descriptor);
  const auto orange_class_id = rescue::declareRequired<std::int64_t>(
    *this, "orange_class_id", descriptor);
  const RoiClassIds class_ids{green_class_id, blue_class_id, black_class_id, orange_class_id};
  const auto detections_topic = rescue::declareRequired<std::string>(
    *this, "detections_topic", descriptor);
  return {input_topic, control_topic, detections_topic, reset_timeout_s,
    roi_check_timeout_s, image_width, image_height, {x, y, width, height}, class_ids};
}

RoiPolicy TargetNode::roiPolicy()
{
  // 与 Mission、Bridge 共用生产组合规则；首趟和容量仍在运输层判断。
  // RoiValidator 的纯值调用仍可显式传入独立开关，生产不在节点里另设一份。
  return cargo::kProductionCombination;
}

// ---- TargetNode：ROS 接口 ----

void TargetNode::createRosInterfaces()
{
  // 图像派生检测沿用传感器 QoS；会话可靠且持久化，使后启动节点先获得当前阶段。
  // 默认互斥回调组串行访问 pending、原始快照及 FrameGate，无需再引入业务锁。
  roi_pub_ = create_publisher<interface_pkg::msg::RoiCheck>("~/roi_result", rclcpp::QoS(10));
  detections_sub_ = create_subscription<vision_msgs::msg::Detection2DArray>(
    settings_.detections_topic, rclcpp::SensorDataQoS().keep_last(1),
    [this](vision_msgs::msg::Detection2DArray::ConstSharedPtr msg) {detectionsCallback(msg);});
  check_service_ = create_service<Check>(
    "~/check_roi",
    [this](RequestHeader header, std::shared_ptr<Check::Request>) {
      requestCheck(header);
    });
  output_pub_ = create_publisher<vision_msgs::msg::Detection2DArray>(
    "~/target", rclcpp::SensorDataQoS().keep_last(1));
  mode_sub_ = create_subscription<interface_pkg::msg::TaskState>(
    settings_.control_topic, rclcpp::QoS(1).reliable().transient_local(),
    [this](interface_pkg::msg::TaskState::ConstSharedPtr msg) {modeCallback(msg);});
  input_sub_ = create_subscription<vision_msgs::msg::Detection2DArray>(
    settings_.input_topic, rclcpp::SensorDataQoS().keep_last(1),
    [this](vision_msgs::msg::Detection2DArray::ConstSharedPtr msg) {targetCallback(msg);});
  // 100 ms 轮询用于无新帧时收尾；有帧到达时还会即时核对精确单调截止。
  timeout_timer_ = create_wall_timer(
    std::chrono::milliseconds(100),
    [this] {checkTimeout();});
}

// ---- TargetNode：会话、目标与原始帧事件 ----

void TargetNode::modeCallback(const interface_pkg::msg::TaskState::ConstSharedPtr & msg)
{
  std::int64_t stamp;
  if (!RecognitionValidation::sourceTime(msg->header.stamp, now().nanoseconds(), stamp)) {
    return;
  }
  recognition::TaskSession next{msg->task, msg->epoch, stamp};
  // 丢弃旧源时间和完全重复会话；同时间换 task/epoch、同模式新时间仍需执行切换。
  if (!recognition::isNewSession(session_, next)) {return;}
  // 必须先用旧原始坐标系生成失败结果并完成等待，随后才能清旧快照、保存新会话。
  if (pending_roi_) {finishCheck(failedCheck("recognition phase changed"));}
  // 原始检测的坐标系快照不能跨识别会话复用；ROI 等待与选中目标转发独立清理。
  have_raw_ = false;
  raw_stamp_ns_ = 0;
  raw_frame_id_.clear();
  session_ = std::move(next);
  enabled_ = session_->task == interface_pkg::msg::TaskState::TASK_SEG;
  // 保留较新的超时或任务截止时间。
  frame_gate_.clear(stamp);
  clearOutput(msg->header.stamp);
}

// 选中目标仅按实际 seg 会话和本路帧门控转发；恰好一个完整合法目标才保留。
// 空、多目标或残缺目标输出空数组，仍推进帧历史；运输阶段与 CAN 停发由下游处理。
void TargetNode::targetCallback(const vision_msgs::msg::Detection2DArray::ConstSharedPtr & msg)
{
  if (!enabled_) {return;}
  const auto current = now().nanoseconds();
  std::int64_t stamp;
  if (!RecognitionValidation::sourceTime(msg->header.stamp, current, stamp)) {return;}
  if (!frame_gate_.accepts(stamp, current)) {return;}
  vision_msgs::msg::Detection2DArray output;
  output.header = msg->header;
  if (msg->detections.size() == 1) {
    const auto & detection = msg->detections.front();
    const auto & box = detection.bbox;
    bool valid = !detection.id.empty() && !detection.results.empty() &&
      RecognitionValueValidation::axisAlignedBox(
      {box.center.position.x, box.center.position.y, box.size_x, box.size_y, box.center.theta});
    // 转发的是完整 ROS 检测，因此逐条校验其全部假设；ROI 纯值转换只消费第一条假设。
    for (const auto & result : detection.results) {
      valid = valid && RecognitionValueValidation::hypothesis(
        result.hypothesis.class_id, result.hypothesis.score);
    }
    if (valid) {output.detections.push_back(detection);}
  }
  output_pub_->publish(output);
  last_header_ = msg->header;
  // 空目标也是一帧有效输入，须推进重复帧门控与断流计时。
  frame_gate_.record(stamp, msg->header.frame_id, FrameGate::Clock::now());
}

// 受理只建立等待；首张满足会话、新鲜度、坐标系和请求源时间的原始帧完成检查。
// 缓存检测不直接作答；忙或非 seg 的拒绝不能覆盖原等待。
void TargetNode::requestCheck(const RequestHeader & header)
{
  if (pending_roi_ || !enabled_) {
    auto result = failedCheck(
      pending_roi_ ? "another ROI check is pending" :
      "YOLO recognition is inactive");
    result.request_id = ++request_id_;
    respondCheck(header, result);
    return;
  }
  PendingRoiCheck pending;
  pending.id = ++request_id_;
  // 不使用已缓存的上一帧直接作答；请求源时间与单调截止分别记录。
  pending.earliest_frame_ns = now().nanoseconds();
  // 尚无原始帧时不猜测坐标系；已有快照则把它固定为本次等待的坐标系预期。
  pending.expected_frame_id = have_raw_ ? raw_frame_id_ : "";
  pending.response_header = header;
  pending.deadline = RoiClock::now() +
    std::chrono::duration_cast<RoiClock::duration>(
    std::chrono::duration<double>(settings_.roi_check_timeout_s));
  pending_roi_ = std::move(pending);
}

void TargetNode::detectionsCallback(
  const vision_msgs::msg::Detection2DArray::ConstSharedPtr & msg)
{
  if (!enabled_) {return;}
  const auto current = now().nanoseconds();
  std::int64_t stamp;
  if (!RecognitionValidation::sourceTime(msg->header.stamp, current, stamp)) {return;}
  // 原始通路单独拒绝旧会话、过期和重复源帧；不能拿选中目标的 FrameGate 历史代替。
  if (!session_ || stamp < session_->stamp_ns ||
    current - stamp > settings_.reset_timeout_s * 1e9 ||
    (have_raw_ && stamp <= raw_stamp_ns_)) {return;}
  // 先记录当前合格原始帧，再处理等待。坐标系变化优先于超时和“是否晚于请求”判断，
  // 即使此帧早于请求源时间，也必须结束不再适用的旧坐标系等待。
  const bool changed_frame = have_raw_ && msg->header.frame_id != raw_frame_id_;
  raw_frame_id_ = msg->header.frame_id;
  raw_stamp_ns_ = stamp;
  have_raw_ = true;
  if (!pending_roi_) {return;}
  if (changed_frame || (!pending_roi_->expected_frame_id.empty() &&
    pending_roi_->expected_frame_id != msg->header.frame_id))
  {
    auto result = failedCheck("detection coordinate frame changed");
    // 此失败由一张真实新坐标系帧触发，沿用该帧 header；阶段取消则保留旧 frame_id。
    result.header = msg->header;
    finishCheck(result);
    return;
  }
  consumeRoiFrame(*msg, stamp, RoiClock::now());
}

void TargetNode::consumeRoiFrame(
  const vision_msgs::msg::Detection2DArray & msg, std::int64_t stamp,
  RoiClock::time_point observed_at)
{
  // 截止相等即失败；只有尚未超时，才允许用源时间等于或晚于受理时间的帧作答。
  if (expireRoiCheck(observed_at)) {return;}
  if (stamp >= pending_roi_->earliest_frame_ns) {
    // 必须交入完整检测集合；空集合也完成本次检查，并由核心判为“空 ROI”。
    finishCheck(RoiRosConversion::evaluate(*validator_, msg));
  }
}

// ---- TargetNode：诊断与服务应答 ----

// 无源帧可作答时使用当前 ROS 时间和最近原始坐标系，仅表达失败，不伪称有对应源图。
// 失败仍携带配置 ROI 范围，valid/passed 及计数保持默认，不能继承上次检查结论。
interface_pkg::msg::RoiCheck TargetNode::failedCheck(const std::string & reason) const
{
  std_msgs::msg::Header header;
  header.stamp = now();
  header.frame_id = raw_frame_id_;
  RoiRuleResult result;
  result.roi = validator_->region();
  result.reason = reason;
  return RoiRosConversion::message(result, header);
}

void TargetNode::finishCheck(interface_pkg::msg::RoiCheck result)
{
  // 先清本地等待再发布/发送；异常路径不留下悬挂请求，也不会重复发送同一响应。
  auto pending = std::exchange(pending_roi_, std::nullopt);
  result.request_id = pending->id;
  respondCheck(pending->response_header, result);
}

void TargetNode::respondCheck(
  const RequestHeader & header,
  const interface_pkg::msg::RoiCheck & result)
{
  Check::Response response;
  response.result = result;
  // Target 的顺序是诊断先、应答后；与 Mission 的运输提交事务顺序不同。
  // 发布或发送异常向外传播，等待已消费，由上游 Mission 截止收尾。
  publishRoiResult(result);
  sendCheckResponse(header, response);
}

void TargetNode::publishRoiResult(const interface_pkg::msg::RoiCheck & result)
{
  roi_pub_->publish(result);
}

void TargetNode::sendCheckResponse(const RequestHeader & header, Check::Response & response)
{
  check_service_->send_response(*header, response);
}

// ---- TargetNode：输出撤销与超时清理 ----

// 切阶段和断流均发布空目标，清接收历史时保留 FrameGate 已建立的源时间截止。
void TargetNode::clearOutput(const builtin_interfaces::msg::Time & stamp)
{
  vision_msgs::msg::Detection2DArray empty;
  empty.header = last_header_;
  empty.header.stamp = stamp;
  output_pub_->publish(empty);
  frame_gate_.clear();
}

bool TargetNode::expireRoiCheck(RoiClock::time_point observed_at)
{
  if (pending_roi_ && observed_at >= pending_roi_->deadline) {
    finishCheck(failedCheck("ROI detection timed out"));
    return true;
  }
  return false;
}

void TargetNode::checkTimeout()
{
  if (pending_roi_) {expireRoiCheck(RoiClock::now());}
  if (frame_gate_.expired(FrameGate::Clock::now())) {
    // 超时后将目标准入截止抬到当前 ROS 时间，迟到的旧帧不能恢复已清空的输出。
    const auto cutoff = now();
    clearOutput(static_cast<builtin_interfaces::msg::Time>(cutoff));
    frame_gate_.clear(cutoff.nanoseconds());
  }
}

RCLCPP_COMPONENTS_REGISTER_NODE(TargetNode)
