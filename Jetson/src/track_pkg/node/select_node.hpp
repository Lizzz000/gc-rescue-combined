#pragma once
// 运输资格和单目标选择。轨迹关联在 track_node.hpp。

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

// ---- SelectNode：运输资格与单目标选择 ----
// 实际阶段、运输资格和帧历史在默认互斥回调组中更新；选择核心只处理值类型。
class SelectNode : public rclcpp::Node
{
public:
  /**
   * @brief 在选择器与帧闸门入口校验后创建选择节点。
   * @param[in] options 包含完整统一参数的ROS选项。
   * @throws rescue::ConfigurationError 非法配置时构造失败。
   */
  explicit SelectNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  friend struct TrackingNodeTestAccess;
  struct Settings
  {
    std::string tracks_topic, active_task_topic, mission_status_topic;
    // 像素宽高用于归一化，避蓝阈值是各轴归一化框间隙；类别值对应模型编号。
    std::int64_t image_width, image_height;
    double blue_distance_ratio;
    std::vector<std::int64_t> candidate_classes;
    std::int64_t blue_class, first_class;
    double reset_timeout_s;
  };
  /**
   * @brief 读取完整只读参数；各值交给所属构造入口检查一次。
   * @return 参数集合；缺失、类型或本节点资格错误抛ConfigurationError。
   */
  Settings loadParameters();
  void createRosInterfaces();
  // 返回零或一个原始轨迹消息；不把轨迹 ID、类别或过滤后位置当作原输入索引。
  /**
   * @brief 检查新帧身份并一次筛选候选，发布后才登记历史。
   * @param[in] msg 非空原图像素轨迹数组，携带同一ROS时钟的源帧时间。
   */
  void tracksCallback(const vision_msgs::msg::Detection2DArray::ConstSharedPtr & msg);
  void modeCallback(const interface_pkg::msg::TaskState::ConstSharedPtr & msg);
  void missionCallback(const interface_pkg::msg::MissionStatus::ConstSharedPtr & msg);
  void clearTarget(const builtin_interfaces::msg::Time & stamp);
  void checkTimeout();

  // 参数、运输资格与源帧身份。
  Settings settings_;
  std::unique_ptr<TargetSelector> selector_;
  bool enabled_{false};
  bool later_transports_{false};
  std::optional<recognition::TaskSession> session_;
  // Mission 的状态发布时间与 TaskState 的阶段时间各有水位，不能用一条话题替代另一条。
  std::int64_t mission_stamp_ns_{0};
  std_msgs::msg::Header last_header_;
  FrameGate frame_gate_;
  rclcpp::Subscription<vision_msgs::msg::Detection2DArray>::SharedPtr tracks_sub_;
  rclcpp::Subscription<interface_pkg::msg::TaskState>::SharedPtr mode_sub_;
  rclcpp::Subscription<interface_pkg::msg::MissionStatus>::SharedPtr mission_sub_;
  rclcpp::Publisher<vision_msgs::msg::Detection2DArray>::SharedPtr target_pub_;
  rclcpp::TimerBase::SharedPtr timeout_timer_;
};
