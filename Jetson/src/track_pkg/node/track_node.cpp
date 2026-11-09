// 跟踪运行时合在一个实现文件里，节顺序与头文件一致。

#include "node/track_node.hpp"

// ---- 跟踪节点实现 ----
// 职责：ROS 装配、运行循环和进程入口。
// Copyright 2026 li
// SPDX-License-Identifier: Apache-2.0

#include <exception>
#include <interface_pkg/app/core.hpp>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <utility>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <interface_pkg/app/core.hpp>
#include <limits>

// ---- TrackNode：构造与参数 ----

// 源帧准入由 FrameGate 管理，关联交给 SegTracker；默认互斥回调组串行访问二者。

/**
 * @brief 读取配置并在所属参数入口校验后创建跟踪订阅、输出和超时计时器。
 * @param[in] options ROS节点选项，须包含统一配置提供的完整参数。
 * @throws rescue::ConfigurationError 参数错误时拒绝创建业务接口。
 */
TrackNode::TrackNode(const rclcpp::NodeOptions & options)
: Node("track_node", options), settings_(loadParameters())
{
  try {
    frame_gate_.setTimeout(settings_.reset_timeout_s);
  } catch (const std::invalid_argument & error) {
    throw rescue::ConfigurationError(error.what());
  }
  tracker_ = std::make_unique<SegTracker>(settings_.tracker);
  createRosInterfaces();
  RCLCPP_INFO(
    get_logger(), "OC-SORT seg tracking: %s -> ~/seg/tracks", settings_.detections_topic.c_str());
}

/**
 * @brief 读取只读ROS参数，将整数原值交给唯一跟踪参数入口校验。
 * @return 参数集合；缺失、类型或跟踪取值错误抛ConfigurationError。
 * @note 帧超时由构造中的FrameGate入口检查，不在这里重复检查。
 */
TrackNode::Settings TrackNode::loadParameters()
{
  rcl_interfaces::msg::ParameterDescriptor descriptor;
  descriptor.read_only = true;
  const auto detections_topic = rescue::declareRequired<std::string>(
    *this, "detections_topic", descriptor);
  const auto active_task_topic = rescue::declareRequired<std::string>(
    *this, "active_task_topic", descriptor);
  const double threshold = rescue::declareRequired<double>(
    *this, "det_threshold", descriptor);
  const double iou = rescue::declareRequired<double>(
    *this, "iou_threshold", descriptor);
  const double inertia = rescue::declareRequired<double>(
    *this, "inertia", descriptor);
  // 保留ROS整数宽度，唯一的TrackerParameters入口完成范围检查后再窄化。
  const auto max_age = rescue::declareRequired<std::int64_t>(*this, "max_age", descriptor);
  const auto min_hits = rescue::declareRequired<std::int64_t>(*this, "min_hits", descriptor);
  const auto delta_t = rescue::declareRequired<std::int64_t>(*this, "delta_t", descriptor);
  const bool use_byte = rescue::declareRequired<bool>(
    *this, "use_byte", descriptor);
  const auto reset_timeout_s = rescue::declareRequired<double>(
    *this, "reset_timeout_s", descriptor);
  try {
    return {detections_topic, active_task_topic, reset_timeout_s,
      TrackerParameters::validated(threshold, max_age, min_hits, iou, delta_t, inertia, use_byte)};
  } catch (const std::invalid_argument & error) {
    // 只转译从 ROS 参数构造跟踪配置时的拒绝；纯值调用仍保留 invalid_argument。
    throw rescue::ConfigurationError(error.what());
  }
}

// ---- TrackNode：ROS 接口 ----

// 默认互斥组同时串行订阅和看门狗，保证 FrameGate 与类别跟踪器没有并发读写。
void TrackNode::createRosInterfaces()
{
  tracks_pub_ = create_publisher<vision_msgs::msg::Detection2DArray>(
    "~/seg/tracks", rclcpp::SensorDataQoS().keep_last(1));
  mode_sub_ = create_subscription<interface_pkg::msg::TaskState>(
    settings_.active_task_topic, rclcpp::QoS(1).reliable().transient_local(),
    [this](interface_pkg::msg::TaskState::ConstSharedPtr msg) {modeCallback(msg);});
  detections_sub_ = create_subscription<vision_msgs::msg::Detection2DArray>(
    settings_.detections_topic, rclcpp::SensorDataQoS().keep_last(1),
    [this](vision_msgs::msg::Detection2DArray::ConstSharedPtr msg) {detectionsCallback(msg);});
  timeout_timer_ = create_wall_timer(
    std::chrono::milliseconds(100),
    [this] {checkTimeout();});
}

// ---- TrackNode：会话与检测事件 ----

// 发布成功才记录源帧；空检测仍推进帧龄和单调断流计时。

// 只信任实际 TaskState；同模式新代次也重置，迟到或完全重复的状态不重置历史。
void TrackNode::modeCallback(const interface_pkg::msg::TaskState::ConstSharedPtr & msg)
{
  std::int64_t source_stamp;
  if (!RecognitionValidation::sourceTime(msg->header.stamp, now().nanoseconds(), source_stamp)) {
    return;
  }
  // 源时间先验合法性仍在 ROS 边界检查；会话判定允许 epoch 重置，也保留未知模式原文。
  recognition::TaskSession next{msg->task, msg->epoch, source_stamp};
  if (!recognition::isNewSession(session_, next)) {return;}
  session_ = std::move(next);
  enabled_ = session_->task == interface_pkg::msg::TaskState::TASK_SEG;
  // 迟到的阶段通知不能放行已被断流看门狗丢弃的帧；clear 内部只抬高截止水位。
  frame_gate_.clear(source_stamp);
  clearTracks(msg->header.stamp);
}

void TrackNode::detectionsCallback(
  const vision_msgs::msg::Detection2DArray::ConstSharedPtr & msg)
{
  if (!enabled_) {return;}
  const auto current = now().nanoseconds();
  std::int64_t stamp;
  if (!RecognitionValidation::sourceTime(msg->header.stamp, current, stamp)) {return;}
  if (!frame_gate_.accepts(stamp, current)) {return;}
  const auto arrival = FrameGate::Clock::now();
  // 订阅可能先于看门狗执行；源时钟慢速时也必须按单调到达间隔重置历史，再接收新帧。
  if (frame_gate_.historyChanged(stamp, msg->header.frame_id) || frame_gate_.expired(arrival)) {
    tracker_->reset();
  }
  auto output = std::make_unique<vision_msgs::msg::Detection2DArray>(
    TrackRosConversion::tracked(
      tracker_->update(RecognitionRosConversion::detections(*msg)), msg->header));
  tracks_pub_->publish(std::move(output));
  // 只有发布成功才写入来源和帧历史；不能在发布前把失败的一帧记为已处理。
  last_header_ = msg->header;
  // 无检测也记录帧和到达时间，仍推进 OC-SORT 的帧龄，不能被误判为没有收到图像。
  frame_gate_.record(stamp, msg->header.frame_id, arrival);
}

// ---- TrackNode：输出撤销与超时清理 ----

// 空数组让下游立即撤掉旧目标；保留最近来源 frame_id，截止水位由 FrameGate 前移。
void TrackNode::clearTracks(const builtin_interfaces::msg::Time & stamp)
{
  tracker_->reset();
  vision_msgs::msg::Detection2DArray empty;
  empty.header = last_header_;
  empty.header.stamp = stamp;
  tracks_pub_->publish(empty);
  frame_gate_.clear();
}

// ROS 源时间停住时仍按单调到达龄断流清空；清空后的截止防止排队旧帧恢复输出。
void TrackNode::checkTimeout()
{
  if (frame_gate_.expired(FrameGate::Clock::now())) {
    const auto cutoff = now();
    clearTracks(static_cast<builtin_interfaces::msg::Time>(cutoff));
    frame_gate_.clear(cutoff.nanoseconds());
  }
}

// SelectNode 的实现在 select_node.cpp。

/**
 * @brief 按原始位置复制完整ROS检测，空结果仍携带源帧头。
 * @param[in] input 原始检测数组，不修改。
 * @param[in] index 原数组有效位置或空值；越界抛out_of_range。
 * @return 零个或一个完整检测，不表示发布已完成。
 */
vision_msgs::msg::Detection2DArray TrackRosConversion::selected(
  const vision_msgs::msg::Detection2DArray & input, std::optional<std::size_t> index)
{
  vision_msgs::msg::Detection2DArray output;
  output.header = input.header;
  // 没有选中项时仍保留这一源帧的数组头，用空数组明确表达本帧没有安全目标。
  if (index) {
    // 索引指向原始消息；完整复制包括全部假设、pose、ID 与检测自己的 header。
    // 使用 at 保留索引越界的显式失败；不能静默改成另一条轨迹或吞掉错误。
    output.detections.push_back(input.detections.at(*index));
  }
  return output;
}


/**
 * @brief 将核心生成的轨迹映射成ROS像素框。
 * @param[in] tracks 核心轨迹与类别，输入不变。
 * @param[in] header 源图像帧头，数组和单框共同沿用。
 * @return 新构建的ROS检测数组，不表示发布已完成。
 */
vision_msgs::msg::Detection2DArray TrackRosConversion::tracked(
  const RecognitionDetections & tracks, const std_msgs::msg::Header & header)
{
  vision_msgs::msg::Detection2DArray output;
  output.header = header;
  // 跟踪是重新生成框，沿用源帧时间；不复制原检测的额外假设或推测其 pose。
  output.detections.reserve(tracks.size());
  for (const auto & track : tracks) {
    vision_msgs::msg::Detection2D detection;
    detection.header = header;
    detection.id = track.id;
    detection.bbox.center.position.x = track.box.center_x;
    detection.bbox.center.position.y = track.box.center_y;
    detection.bbox.size_x = track.box.width;
    detection.bbox.size_y = track.box.height;
    if (track.hypothesis) {
      vision_msgs::msg::ObjectHypothesisWithPose result;
      result.hypothesis.class_id = track.hypothesis->class_id;
      result.hypothesis.score = track.hypothesis->score;
      detection.results.push_back(result);
    }
    output.detections.push_back(std::move(detection));
  }
  return output;
}

#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE(TrackNode)
