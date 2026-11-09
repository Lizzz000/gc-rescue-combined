#pragma once

// MissionNode 的 ROS 装配。目标和安全区分别在 target_node.hpp、safety_node.hpp。
// Copyright 2026 li
// SPDX-License-Identifier: Apache-2.0

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <std_srvs/srv/set_bool.hpp>
#include <interface_pkg/msg/mission_status.hpp>
#include <interface_pkg/msg/roi_check.hpp>
#include <interface_pkg/msg/task_state.hpp>
#include <interface_pkg/srv/check_roi.hpp>
#include <interface_pkg/srv/mission_command.hpp>
#include <interface_pkg/app/core.hpp>
#include <chrono>
#include <cstdint>
#include "control/controller.hpp"

// arrived 延迟返回 ROI 结果；command 即时返回受理结果，实际模式还须 TaskState 确认。
// request_timeout_s 是两类服务等待的秒制上限 (0, 3600]，不约束物理转向或搬运。
class MissionNode : public rclcpp::Node, private MissionOut
{
public:
  explicit MissionNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~MissionNode() override;

protected:
  using Switch = std_srvs::srv::SetBool;
  using CheckRoi = interface_pkg::srv::CheckRoi;
  using Command = interface_pkg::srv::MissionCommand;
  using RequestHeader = std::shared_ptr<rmw_request_id_t>;
  // ROS 调用边界；合同测试可单独注入应答或发布异常，业务顺序仍由节点实现负责。
  virtual void sendRoiResponse(const RequestHeader & header, CheckRoi::Response & response);
  virtual void publishRoiResult(const interface_pkg::msg::RoiCheck & result);
  void publishStatus(const MissionState & state) override;

private:
  friend struct MissionNodeTestAccess;
  struct Settings
  {
    std::string detector_service, detector_task_topic, target_roi_service;
    // 单位秒：下游服务等待上限，使用 steady_clock；不表示图像新鲜度。
    double request_timeout_s{0};
  };
  Settings loadParameters();
  void createRosInterfaces(const Settings & settings);
  static TransportPolicy transportPolicy();
  void onArrived(const RequestHeader & header);
  // 发布或应答抛出后：还没发出去的到达先补一次失败结果，再按控制器规则退回或补发状态。
  void settleArrivalFailure(const RequestHeader & header, const std::string & message);
  void onCommand(const Command::Request & request, Command::Response & response);
  void onTaskChanged(interface_pkg::msg::TaskState::ConstSharedPtr state);
  void onSwitchReply(
    PendingOperation::Kind kind, std::uint64_t generation,
    rclcpp::Client<Switch>::SharedFuture result);
  void onRoiReply(std::uint64_t generation, rclcpp::Client<CheckRoi>::SharedFuture result);
  void checkTimeouts();

  void publishRoi(
    const RoiRuleResult & result, std::uint64_t request_id, bool use_source_stamp) override;
  void forwardTask() override;
  bool roiServiceReady() override;
  void sendRoiRequest(std::uint64_t generation) override;
  bool detectorReady() override;
  void sendSwitchRequest(bool field, std::uint64_t generation) override;
  void cancel(PendingOperation::Kind kind, std::int64_t request_id) override;

  Settings settings_;
  MissionController controller_;
  rclcpp::Client<Switch>::SharedPtr detector_client_;
  rclcpp::Client<CheckRoi>::SharedPtr roi_client_;
  rclcpp::Service<CheckRoi>::SharedPtr arrived_service_;
  rclcpp::Service<Command>::SharedPtr command_service_;
  rclcpp::Publisher<interface_pkg::msg::TaskState>::SharedPtr control_pub_;
  rclcpp::Publisher<interface_pkg::msg::RoiCheck>::SharedPtr roi_pub_;
  rclcpp::Publisher<interface_pkg::msg::MissionStatus>::SharedPtr status_pub_;
  // 只有 arrived 等待下游 ROI；即时指令不保存外部请求头。
  RequestHeader roi_header_;
  // 仅 onArrived 这一次调用有效。异步完成用 roi_header_，不能借这次栈上的头。
  RequestHeader roi_call_header_;
  const interface_pkg::msg::RoiCheck * roi_source_{nullptr};
  const interface_pkg::msg::TaskState * task_message_{nullptr};
  // 本次到达是否已成功应答。异步完成也置位，下次到达重置。
  bool arrival_answered_{false};
  rclcpp::Subscription<interface_pkg::msg::TaskState>::SharedPtr state_sub_;
  rclcpp::TimerBase::SharedPtr timeout_timer_;
};
