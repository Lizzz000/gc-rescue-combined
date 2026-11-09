#pragma once

#include "tracking/tracking.hpp"

// ---- 跟踪节点 ----
// 职责：帧准入、轨迹关联和选择的 ROS 装配。
// Copyright 2026 li
// SPDX-License-Identifier: Apache-2.0


#include <interface_pkg/app/core.hpp>
#include <chrono>
#include <cstdint>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <interface_pkg/msg/task_state.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>
#include <interface_pkg/msg/mission_status.hpp>

// ---- TrackNode：帧准入与轨迹关联 ----
// 默认互斥回调组串行访问 FrameGate 和 SegTracker，关联核心不创建线程。
class TrackNode : public rclcpp::Node
{
public:
  /**
   * @brief 在参数入口校验完整配置后创建跟踪节点。
   * @param[in] options 包含完整统一参数的ROS选项。
   * @throws rescue::ConfigurationError 非法配置时构造失败。
   */
  explicit TrackNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  friend struct TrackingNodeTestAccess;
  struct Settings
  {
    std::string detections_topic, active_task_topic;
    // 秒：同时约束源帧年龄和本机单调到达间隔，与 OC-SORT 的帧龄参数分别生效。
    double reset_timeout_s;
    TrackerParameters tracker;
  };
  /**
   * @brief 读取完整只读参数；各值交给所属构造入口检查一次。
   * @return 参数集合；缺失、类型或本节点资格错误抛ConfigurationError。
   */
  Settings loadParameters();
  void createRosInterfaces();

  // 先通过帧闸门，再关联和发布；空检测帧也记录，以区别“看见空画面”与断流。
  void detectionsCallback(const vision_msgs::msg::Detection2DArray::ConstSharedPtr & msg);
  void modeCallback(const interface_pkg::msg::TaskState::ConstSharedPtr & msg);
  void checkTimeout();
  // 发布空数组并清历史；传入阶段或看门狗的时间，不能用接收时间替换源阶段边界。
  void clearTracks(const builtin_interfaces::msg::Time & stamp);

  // 参数、关联历史与源帧身份。
  Settings settings_;
  std::unique_ptr<SegTracker> tracker_;
  // 会话保留原 task 文本、epoch 和源纳秒；enabled 只由已接收的实际 seg 阶段置位。
  std::optional<recognition::TaskSession> session_;
  bool enabled_{false};
  std_msgs::msg::Header last_header_;
  FrameGate frame_gate_;

  rclcpp::Subscription<vision_msgs::msg::Detection2DArray>::SharedPtr detections_sub_;
  rclcpp::Subscription<interface_pkg::msg::TaskState>::SharedPtr mode_sub_;
  rclcpp::Publisher<vision_msgs::msg::Detection2DArray>::SharedPtr tracks_pub_;
  rclcpp::TimerBase::SharedPtr timeout_timer_;
};

#include "node/select_node.hpp"

/** @brief 跟踪与选择的ROS输出边界；复用节点实现，不另设转换文件。 */
struct TrackRosConversion
{
  /**
   * @brief 按原输入索引复制完整检测，空索引仍保留数组源帧头。
   * @param[in] input 原始ROS检测数组，额外假设和位姿均保留。
   * @param[in] index 选择器返回的原始位置或空值；非空时须在数组范围内。
   * @return 零个或一个检测；越界抛out_of_range，原数组不变。
   */
  static vision_msgs::msg::Detection2DArray selected(
    const vision_msgs::msg::Detection2DArray & input, std::optional<std::size_t> index);
  /**
   * @brief 将关联核心生成的框映射成ROS消息，保留源图帧头。
   * @param[in] tracks 核心生成的只读像素框和轨迹身份。
   * @param[in] header 原始图像帧头，不使用本机接收时间替代。
   * @return 新ROS数组；原输入不变，不代表消息已发布。
   */
  static vision_msgs::msg::Detection2DArray tracked(
    const RecognitionDetections & tracks, const std_msgs::msg::Header & header);
};
