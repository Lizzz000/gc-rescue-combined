#pragma once

// ---- 任务节点 ----
// 职责：任务、目标和安全三个角色的 ROS 装配。
// Copyright 2026 li
// SPDX-License-Identifier: Apache-2.0


#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <interface_pkg/msg/mission_status.hpp>
#include <interface_pkg/msg/task_state.hpp>
#include <interface_pkg/app/core.hpp>
#include <chrono>
#include <cstdint>
#include <optional>
#include <interface_pkg/msg/field_observation.hpp>
#include <interface_pkg/msg/safety_target.hpp>

// ---- SafetyNode：场地观测与目的区中心 ----
// 目的区沿用 MissionStatus；实际 field、导航就绪和源时间截止共同约束有效目标。
// 中心由 FieldGeometry 计算对角线交点，不裁剪到 [0,1]；无效结果的坐标归零。
// 默认互斥回调组中先发观测、再发目标，两次发布成功后才记录源帧历史。
class SafetyNode : public rclcpp::Node
{
public:
  explicit SafetyNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  friend class SafetyNodeTestAccess;
  // 统一装配层传入的只读参数；节点不再维护另一套生产默认值。
  struct Settings
  {
    // 分别接收原始场地观测、实际任务阶段及 Mission 已提交的目的区状态。
    std::string input_topic, control_topic, mission_status_topic;
    // 秒：同时限制源帧年龄和单调时钟上的断流等待，二者使用各自的时钟。
    double reset_timeout_s;
    // 原图像素尺寸，只用于中心归一化及尺寸错配提示，不用于裁剪投影几何。
    std::int64_t image_width, image_height;
  };
  Settings loadParameters();
  void createRosInterfaces();
  // 无效几何转发为空观测；有效目标还须满足安全区导航状态与目的区的新帧截止。
  void observationCallback(const interface_pkg::msg::FieldObservation::ConstSharedPtr & msg);
  // 清空无效正式几何；候选数组仅在含非法点时清空，合法候选仍作诊断，源 header 不变。
  void sanitizeObservation(interface_pkg::msg::FieldObservation & output);
  interface_pkg::msg::SafetyTarget makeSafetyTarget(
    const interface_pkg::msg::FieldObservation & output, std::int64_t stamp) const;
  // 仅实际 field 阶段放行；新会话清空旧输出，源时间截止只前移。
  // 同时间换 task/epoch 仍是新会话，完全重复或更旧的阶段不触发清空。
  void modeCallback(const interface_pkg::msg::TaskState::ConstSharedPtr & msg);
  void clearOutput(const builtin_interfaces::msg::Time & stamp);
  void checkTimeout();
  // 只有 NavigatingSafety + IDLE/SUCCESS 且有目的区才准备好。
  void destinationCallback(const interface_pkg::msg::MissionStatus::ConstSharedPtr & msg);
  void clearTarget(const builtin_interfaces::msg::Time & stamp);

  // ---- 目的区身份、源时间与 ROS 资源 ----
  Settings settings_;
  bool enabled_{false};
  // task、epoch 与阶段源时间共同标识会话；不要求 epoch 自身单调递增。
  std::optional<recognition::TaskSession> session_;
  // ROS 纳秒：目的区、请求号或就绪性变化时按状态时间更新，等于截止的源帧仍可用。
  std::int64_t destination_cutoff_ns_{0};
  // 使用 MissionStatus::DEST_*，不是模型类别编号；本节点不自行选择左右。
  std::uint8_t destination_{0};
  // 即使左右未变，新 ROI 请求号也要求源帧不早于该次状态时间。
  std::uint64_t destination_request_id_{0};
  // 同模式重找也会增加代次；深度 1 状态队列即使合并了中间态也必须撤销旧坐标。
  std::uint64_t cycle_generation_{0};
  std::uint8_t cycle_phase_{interface_pkg::msg::MissionStatus::CYCLE_FAULT};
  // MissionStatus 是状态发布时刻；观测沿用源图时间，两者通过截止约束隔离旧路由。
  std::int64_t status_stamp_ns_{0};
  // 等待转向/搬运、重找、故障期间均保持无效中心，场地观测仍可供显示。
  bool destination_ready_{false};
  // 最近一次完成两次发布的源 header；主动撤销时保留其坐标系。
  std_msgs::msg::Header last_header_;
  // 分别管理源时间截止、严格递增帧身份与 steady_clock 断流历史。
  FrameGate frame_gate_;
  rclcpp::Subscription<interface_pkg::msg::FieldObservation>::SharedPtr input_sub_;
  rclcpp::Subscription<interface_pkg::msg::TaskState>::SharedPtr mode_sub_;
  rclcpp::Publisher<interface_pkg::msg::FieldObservation>::SharedPtr output_pub_;
  rclcpp::Subscription<interface_pkg::msg::MissionStatus>::SharedPtr destination_sub_;
  rclcpp::Publisher<interface_pkg::msg::SafetyTarget>::SharedPtr target_pub_;
  rclcpp::TimerBase::SharedPtr timeout_timer_;
};
