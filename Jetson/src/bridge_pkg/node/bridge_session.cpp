// 阅读：前半是构造和 ROS 输入，后半是发送队列、状态和重连。
// USB 轮询在 bridge_io.cpp，事务身份及回执判定在 bridge_transactions.cpp。
// 帧格式在 protocol/usb_protocol.hpp。本文件不声明参数。

#include "node/bridge_node.hpp"
#include "gripper/s288_servo.hpp"

// Copyright 2026 li
// SPDX-License-Identifier: Apache-2.0
// BridgeRuntime 与 BridgeNode 共用默认互斥回调组。源帧用 ROS 时间，队列期限用单调时间。

#include <interface_pkg/app/ros/ros.hpp>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>
#include <exception>
#include <linux/can/error.h>

// ---- BridgeRuntime：构造与会话状态 ----

// CAN 会话与异步请求共用节点默认互斥回调组；源帧用 ROS 时间，队列和服务期限用单调时间。

// 运行模块拥有完整链路会话；注入的节点提供 ROS 源时间和日志。
BridgeRuntime::BridgeRuntime(
  rclcpp::Node & node, Config config, Ports ports, std::unique_ptr<CanTransport> transport)
: node_(node), can_interface_(std::move(config.can_interface)), can_(std::move(transport)),
  tx_queue_timeout_(config.tx_queue_timeout),
  state_(std::move(config.state)), next_open_(Clock::now()),
  mission_watchdog_(std::chrono::duration_cast<std::chrono::milliseconds>(
    config.mission_watchdog_timeout)),
  gripper_config_(std::move(config.gripper)), ports_(std::move(ports)),
  transactions_(state_, arrival_point_, *this, config.request_timeout)
{
  // 启动张开只执行一次；尚未收到 kind 0 查询时也保留成功或失败，不能靠定时猜完成。
  gripper_ = std::make_unique<s288::Session>(s288::SessionConfig{
    gripper_config_.device, static_cast<int>(gripper_config_.id),
    gripper_config_.kp, gripper_config_.kd,
    gripper_config_.closed_deg, gripper_config_.open_deg, gripper_config_.ramp_fraction,
    gripper_config_.open_hold_max_nm, gripper_config_.output_tolerance_deg,
    gripper_config_.open_trim_deg, gripper_config_.closed_trim_deg});
  if (!gripper_->available() || !gripper_->start_open(gripper_config_.move_s)) {
    arrival_point_.complete(false);
  }
}

BridgeRuntime::~BridgeRuntime()
{
  if (gripper_) {gripper_->close();}
}

// 纯值枚举与消息定义的字节值必须一致。接口升级不能只改一侧。
static_assert(
  static_cast<std::uint8_t>(BridgeDestination::None) ==
  interface_pkg::msg::MissionStatus::DEST_NONE);
static_assert(
  static_cast<std::uint8_t>(BridgeDestination::Left) ==
  interface_pkg::msg::MissionStatus::DEST_LEFT);
static_assert(
  static_cast<std::uint8_t>(BridgeDestination::Right) ==
  interface_pkg::msg::MissionStatus::DEST_RIGHT);
static_assert(
  RecognitionValueValidation::cargoSide(0) ==
  interface_pkg::msg::MissionStatus::DEST_LEFT);
static_assert(
  RecognitionValueValidation::cargoSide(1) ==
  interface_pkg::msg::MissionStatus::DEST_RIGHT);
static_assert(
  static_cast<std::uint8_t>(BridgeOperation::Idle) ==
  interface_pkg::msg::MissionStatus::OP_IDLE);
static_assert(
  static_cast<std::uint8_t>(BridgeOperation::Switching) ==
  interface_pkg::msg::MissionStatus::OP_SWITCHING);
static_assert(
  static_cast<std::uint8_t>(BridgeOutcome::Success) ==
  interface_pkg::msg::MissionStatus::RESULT_SUCCESS);
static_assert(
  static_cast<std::uint8_t>(BridgeOutcome::RoiInvalid) ==
  interface_pkg::msg::MissionStatus::RESULT_ROI_INVALID);
static_assert(
  static_cast<std::uint8_t>(BridgeTransportPhase::First) ==
  interface_pkg::msg::MissionStatus::TRANSPORT_FIRST);
static_assert(
  static_cast<std::uint8_t>(BridgeTransportPhase::Later) ==
  interface_pkg::msg::MissionStatus::TRANSPORT_LATER);
static_assert(static_cast<std::uint8_t>(BridgeCyclePhase::WaitingTurn) ==
  interface_pkg::msg::MissionStatus::CYCLE_WAITING_TURN);
static_assert(static_cast<std::uint8_t>(BridgeCyclePhase::Fault) ==
  interface_pkg::msg::MissionStatus::CYCLE_FAULT);
static_assert(static_cast<std::uint8_t>(BridgeCyclePhase::LocalizingStart) ==
  interface_pkg::msg::MissionStatus::CYCLE_LOCALIZING_START);
static_assert(static_cast<std::uint8_t>(BridgeCyclePhase::InitialLocalizing) ==
  interface_pkg::msg::MissionStatus::CYCLE_INITIAL_LOCALIZING);
static_assert(CanProtocol::TURN_COMPLETED ==
  interface_pkg::srv::MissionCommand::Request::TURN_COMPLETED);
static_assert(CanProtocol::SAFETY_ARRIVED ==
  interface_pkg::srv::MissionCommand::Request::SAFETY_ARRIVED);
static_assert(CanProtocol::START_SEARCH ==
  interface_pkg::srv::MissionCommand::Request::START_SEARCH);
static_assert(CanProtocol::START_ALIGN ==
  interface_pkg::srv::MissionCommand::Request::START_ALIGN);
static_assert(CanProtocol::DELIVERY_COMPLETED ==
  interface_pkg::srv::MissionCommand::Request::DELIVERY_COMPLETED);

// ---- BridgeRuntime：ROS 输入事件 ----

// TaskState 先过源时间与重复校验；周期帧由定时器错峰重发，这里不再入队。
void BridgeRuntime::taskCallback(interface_pkg::msg::TaskState::ConstSharedPtr msg)
{
  const BridgeSourceTime current{now().nanoseconds()};
  std::int64_t stamp;
  if (!RecognitionValidation::sourceTime(msg->header.stamp, current.ns, stamp)) {return;}
  const auto update = state_.task({{stamp}, msg->task, msg->epoch}, current);
  if (!update.accepted) {return;}
  transactions_.onStateUpdate();
  transactions_.resolvePendingModeReply(Clock::now());
}

// Mission 是任务准入状态，不能仅凭安全区消息推测成功目的侧。
void BridgeRuntime::missionCallback(interface_pkg::msg::MissionStatus::ConstSharedPtr msg)
{
  const BridgeSourceTime current{now().nanoseconds()};
  std::int64_t stamp;
  if (!RecognitionValidation::sourceTime(msg->header.stamp, current.ns, stamp)) {return;}
  const BridgeMissionState input{
    {stamp}, static_cast<BridgeDestination>(msg->destination),
    static_cast<BridgeOperation>(msg->operation), static_cast<BridgeOutcome>(msg->outcome),
    static_cast<BridgeTransportPhase>(msg->transport_phase), msg->roi_request_id,
    static_cast<BridgeCyclePhase>(msg->cycle_phase), msg->cycle_generation};
  const auto update = state_.mission(input, current);
  // 周期帧不再排队，定时器轮流按当时状态重发。
  if (update.accepted) {
    if (update.mission_restarted) {missionRestarted();}
    mission_watchdog_.received({Clock::now()});
    transactions_.onStateUpdate();
    transactions_.refreshTurnRequest(Clock::now());
    transactions_.resolvePendingModeReply(Clock::now());
  }
}

// 选择结果应为零或一个目标；多目标转成无效新帧，避免延用之前的有效坐标。
void BridgeRuntime::targetCallback(vision_msgs::msg::Detection2DArray::ConstSharedPtr msg)
{
  const BridgeSourceTime current{now().nanoseconds()};
  std::int64_t stamp;
  if (!RecognitionValidation::sourceTime(msg->header.stamp, current.ns, stamp)) {return;}
  BridgeTargetInput input{{stamp}, msg->header.frame_id, std::nullopt};
  // 空集或多目标也要传给状态核心，使被接受的新帧覆盖旧中心；不能直接 return 而延用上一帧位置。
  if (msg->detections.size() == 1) {
    input.detection = RecognitionRosConversion::detection(msg->detections.front());
  }
  // 空帧、多目标和有效目标都更新状态；周期帧由定时器错峰重发。
  state_.target(input, current, {Clock::now()}, can_->socketOpen());
}

// 安全中心已归一化，保留 side 由状态核心在提交时与 Mission 目的侧核对。
void BridgeRuntime::safetyCallback(interface_pkg::msg::SafetyTarget::ConstSharedPtr msg)
{
  const BridgeSourceTime current{now().nanoseconds()};
  std::int64_t stamp;
  if (!RecognitionValidation::sourceTime(msg->header.stamp, current.ns, stamp)) {return;}
  const BridgeSafetyInput input{
    {stamp}, msg->header.frame_id, msg->valid, msg->side, msg->x, msg->y};
  state_.safety(input, current, {Clock::now()}, can_->socketOpen());
}

// ---- BridgeRuntime：排队、输出与诊断 ----

// 队列只保存事件回执的发送机会；周期帧在 io() 里按拍重建，不占队列。

can_frame BridgeRuntime::currentFrame(canid_t id) const
{
  // sendPeriodicFrame 在 Mission 超时后停发周期流；这里仍防止过期坐标进入待发快照。
  const bool mission_alive = mission_watchdog_.alive({Clock::now()});
  if (!mission_alive && (id == CanProtocol::TARGET_ID || id == CanProtocol::SAFETY_ID)) {
    return CanProtocol::center(id, false, 0, 0);
  }
  // 周期帧提交前重新检查新鲜度；一次回复使用事务模块保存的快照。
  if (CanProtocol::replyId(id)) {return transactions_.replyFrame(id);}
  const BridgeSourceTime current{now().nanoseconds()};
  const BridgeMonotonicTime monotonic{Clock::now()};
  const auto center = id == CanProtocol::TARGET_ID ?
    state_.targetOutput(current, monotonic) : state_.safetyOutput(current, monotonic);
  return CanProtocol::center(id, center.valid, center.x, center.y, center.source_frame);
}

void BridgeRuntime::flushTxQueue()
{
  if (!mission_watchdog_.alive({Clock::now()})) {
    dropped_frames_ += queue_.size();
    queue_.reset();
    return;
  }
  // 提交待发帧：每周期最多尝试 8 次（含 EINTR）；front() 丢弃过期项，currentFrame() 用当前输入重建。
  for (int attempt = 0; attempt < 8; ++attempt) {
    const auto id = queue_.front(Clock::now(), tx_queue_timeout_, dropped_frames_);
    if (!id) {break;}
    // 已排队的转向载荷不能跨运输提交或跨阶段发送，背压后也重新核对。
    if (!transactions_.replyCurrent(*id)) {
      dropped_frames_ += queue_.drop(*id);
      continue;
    }
    const auto frame = currentFrame(*id);
    std::string error;
    const auto result = can_->write(frame, error);
    if (result == CanTransport::WriteResult::SENT) {
      // SENT只表示本机内核接收整帧（串口为完整封装包）；一次回复在此结束，不代表 STM32 收到或执行完成。
      queue_.sent();
      ++tx_frames_;
    } else if (result == CanTransport::WriteResult::BACKPRESSURE) {
      // 串口暂时不可写不等于七槽队列已满；保留队首及原 TTL，下一拍重新核对资格。
      last_error_ = "USB transport backpressure";
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "USB transport on %s is backpressured; queued frames retry within their existing TTL",
        can_interface_.c_str());
      break;
    } else if (result == CanTransport::WriteResult::INTERRUPTED) {
      continue;
    } else {
      // 普通传输错误关闭并等 1 秒重连；半包会抛 FatalError，由 io() 关闭后退出。
      RCLCPP_ERROR(
        get_logger(), "USB write to %s failed: %s", can_interface_.c_str(),
        error.c_str());
      closeSocket(error);
      return;
    }
  }
}

void BridgeRuntime::publishStatus()
{
  interface_pkg::msg::BridgeStatus status;
  status.header.stamp = now();
  // 计数反映本机提交和接收；USB 故障写 last_error。
  status.can_interface = can_interface_;
  status.socket_open = can_->socketOpen();
  status.interface_up = status.socket_open;
  status.tx_frames = tx_frames_;
  status.rx_frames = rx_frames_;
  status.bus_errors = 0;
  status.dropped_frames = dropped_frames_;
  status.last_error = last_error_;
  ports_.status_pub->publish(status);
}

// ---- BridgeRuntime：重连、超时与异常清理 ----

// 打开失败后按单调时间隔 1 秒重试；成功后清旧输入与队列，有效周期输出须等待新输入。
void BridgeRuntime::openSocket()
{
  next_open_ = Clock::now() + std::chrono::seconds(1);
  if (!can_->open(can_interface_, last_error_)) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "cannot open USB transport %s: %s", can_interface_.c_str(), last_error_.c_str());
    return;
  }
  last_error_.clear();
  // 撤销旧中心和发送机会，并前移源时间截止；有效周期输出必须收到新输入。
  // 任务、运输阶段和到达保持保留，重连本身不解除握手约束。
  // Bridge 快速重启时板端可能还保有上一进程的运动租约，先给其断流检测留时间。
  motion_lease_suspended_until_ = Clock::now() + motion_lease_break_time_;
  queue_.reset();
  state_.resetInputs({now().nanoseconds()});
  RCLCPP_INFO(
    get_logger(), "USB transport opened on %s", can_interface_.c_str());
  publishStatus();
}

// 断线取消本机服务等待、撤销输入和队列，再安排重连；任务状态与到达保持留在核心。
void BridgeRuntime::closeSocket(const std::string & reason)
{
  can_->close();
  transactions_.disconnect();
  dropped_frames_ += queue_.size();
  queue_.reset();
  state_.resetInputs({now().nanoseconds()});
  last_error_ = reason;
  next_open_ = Clock::now() + std::chrono::seconds(1);
  publishStatus();
}

void BridgeRuntime::missionRestarted()
{
  // STM32 将无效中心也视为在线。重启撤销旧路线时暂停周期流，超过板端 250 ms 断流期限。
  motion_lease_suspended_until_ = Clock::now() + motion_lease_break_time_;
  transactions_.missionRestarted();
  dropped_frames_ += queue_.size();
  queue_.reset();
}

void BridgeRuntime::handleIoFailure(const std::exception & error)
{
  // io() 捕获异常后撤掉发送机会；socket、在途服务和已发生的业务状态保留。
  // 周期帧等待新输入，普通应答等待显式查询；类型 8 在双授权仍成立时按固定节拍重授。
  // STM32 按本趟身份去重，重授不重启转向，也不延长本地动作截止。
  last_error_ = error.what();
  dropped_frames_ += queue_.size();
  queue_.reset();
  RCLCPP_ERROR(
    get_logger(), "CAN output cycle failed, dropping queued frames: %s",
    error.what());
  publishStatus();
}

void BridgeRuntime::handleArrivalPoint(std::uint8_t kind, std::uint32_t request_id)
{
  const bool motion_lease_open = !motion_lease_suspended_until_ ||
    Clock::now() >= *motion_lease_suspended_until_;
  const auto decision = transactions_.handleArrivalPoint(kind, request_id, motion_lease_open);
  if (decision == ArrivalPointDecision::Reject) {return;}
  if (decision == ArrivalPointDecision::Resend) {
    if (const auto result = arrival_point_.result(kind, request_id)) {transactions_.armGripperResult(*result);}
    return;
  }
  // kind 0 永远走查询分支；kind 3 已由会话验证本轮拒绝授权。
  bool started = false;
  try {
    started = kind == 1 ? gripper_->start_close(gripper_config_.move_s) :
      gripper_->start_open(gripper_config_.move_s);
  } catch (const std::exception & error) {
    last_error_ = error.what();
  }
  if (!started) {
    const auto result = arrival_point_.complete(false);
    gripper_->acknowledge();
    if (result) {transactions_.armGripperResult(*result);}
  }
}

void BridgeRuntime::pollGripper()
{
  const auto status = gripper_->status();
  if (status != s288::MotionStatus::Succeeded && status != s288::MotionStatus::Failed) {return;}
  if (status == s288::MotionStatus::Failed) {
    last_error_ = gripper_->message();
    RCLCPP_ERROR(get_logger(), "S288: %s", last_error_.c_str());
    publishStatus();
  }
  if (arrival_point_.busy()) {
    if (status == s288::MotionStatus::Succeeded) {RCLCPP_INFO(get_logger(), "S288: %s", gripper_->message().c_str());}
    const auto result = arrival_point_.complete(status == s288::MotionStatus::Succeeded);
    if (result) {transactions_.armGripperResult(*result);}
  }
  // 已回执的张开仍可出现保位故障，只消费本次实际读到的状态。
  gripper_->acknowledge(status);
}
