// 阅读：前半是构造和 ROS 输入，后半是发送队列、状态和重连。
// 每 10 毫秒的收发、到达握手和服务身份在 bridge_io.cpp。
// 帧格式在 protocol/can_protocol.hpp。本文件不声明参数。

#include "node/bridge_node.hpp"
#include "gripper/s288_servo.hpp"
#include "transport/socketcan/socketcan.hpp"
#include "transport/damiao/damiao.hpp"

// Copyright 2026 li
// SPDX-License-Identifier: Apache-2.0
// BridgeRuntime 与 BridgeNode 共用默认互斥回调组。源帧用 ROS 时间，队列期限用单调时间。

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

// ---- BridgeRuntime：构造与会话状态 ----

// CAN 会话与异步请求共用节点默认互斥回调组；源帧用 ROS 时间，队列和服务期限用单调时间。

// 运行模块拥有完整链路会话；注入的节点提供 ROS 源时间和日志。
BridgeRuntime::BridgeRuntime(
  rclcpp::Node & node, Config config, Ports ports, std::unique_ptr<CanTransport> transport)
: node_(node), can_interface_(std::move(config.can_interface)), can_(std::move(transport)),
  tx_queue_timeout_(config.tx_queue_timeout), request_timeout_(config.request_timeout),
  state_(std::move(config.state)), next_open_(Clock::now()),
  mission_watchdog_(std::chrono::duration_cast<std::chrono::milliseconds>(
    config.mission_watchdog_timeout)),
  gripper_config_(std::move(config.gripper)), ports_(std::move(ports))
{
  // 入口已保证 device 与 move_s；打开或启动张开失败则停心跳，不得以 pause=0 继续。
  pause_ = 1;
  gripper_ = std::make_unique<s288::Session>(s288::SessionConfig{
    gripper_config_.device, static_cast<int>(gripper_config_.id),
    gripper_config_.kp, gripper_config_.kd,
    gripper_config_.closed_deg, gripper_config_.open_deg});
  if (!gripper_->available() || !gripper_->start_open(gripper_config_.move_s)) {
    heartbeat_enabled_ = false;
    return;
  }
  gripper_work_ = GripperWork::ArmingOpen;
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
static_assert(CanProtocol::TURN_COMPLETED ==
  interface_pkg::srv::MissionCommand::Request::TURN_COMPLETED);
static_assert(CanProtocol::SAFETY_ARRIVED ==
  interface_pkg::srv::MissionCommand::Request::SAFETY_ARRIVED);
static_assert(CanProtocol::START_SEARCH ==
  interface_pkg::srv::MissionCommand::Request::START_SEARCH);
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
  if (!state_.locallyHeld() && state_.cyclePhase() == BridgeCyclePhase::SearchingTarget) {
    route_aborted_ = false;
  }
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
    mission_watchdog_.received({Clock::now()});
    if (!state_.locallyHeld() && state_.cyclePhase() == BridgeCyclePhase::SearchingTarget) {
      route_aborted_ = false;
    }
    refreshTurnRequest(Clock::now());
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

// 周期收发和到达握手在 bridge_io.cpp。

// ---- BridgeRuntime：排队、输出与诊断 ----

// 队列只保存事件回执的发送机会；周期帧在 io() 里按拍重建，不占队列。

void BridgeRuntime::refreshTurnRequest(Clock::time_point current, bool force)
{
  if (!turnAuthorized()) {
    dropped_frames_ += queue_.drop(CanProtocol::TURN_REQUEST_ID);
    turn_.reset();
    return;
  }
  if (!turnRequestCurrent()) {
    dropped_frames_ += queue_.drop(CanProtocol::TURN_REQUEST_ID);
    const auto & mission = *state_.missionState();
    turn_ = TurnIdentity{mission.roi_request_id, mission.destination, mission.cycle_generation};
    next_turn_ = current;
  }
  if (!can_->socketOpen() || (!force && current < next_turn_)) {return;}
  // 固定协议节拍，不是算法参数。发送机会仍守原 TTL，不能通过重试续队首寿命。
  armReply(CanProtocol::TURN_REQUEST_ID,
    CanProtocol::turnRequest(static_cast<std::uint8_t>(turn_->destination),
    static_cast<std::uint32_t>(turn_->roi_request_id)));
  // 重授间隔与 docs/can-protocol.md 的 250 ms 链路超时为同一来源。
  next_turn_ = current + CanProtocol::LINK_TIMEOUT;
}

can_frame BridgeRuntime::currentFrame(canid_t id) const
{
  // Mission 超时不能继续发送可保活的心跳或有效坐标。
  const bool mission_alive = mission_watchdog_.alive({Clock::now()});
  if (!mission_alive && id == CanProtocol::VELOCITY_ID) {
    can_frame frame{};
    frame.can_id = id;
    return frame;
  }
  if (!mission_alive && (id == CanProtocol::TARGET_ID || id == CanProtocol::SAFETY_ID)) {
    return CanProtocol::center(id, false, 0, 0);
  }
  // 周期帧提交前重新检查新鲜度；一次回复使用最近 armReply() 保存的快照。
  if (CanProtocol::replyId(id)) {return replies_.frame(id);}
  if (id == CanProtocol::VELOCITY_ID) {
    // 任务在线：写出前当场编码 DLC 8 心跳，valid=0，速度字节全零，pause 取当前值。
    return CanProtocol::velocity(false, 0, 0, 0, pause_);
  }
  const BridgeSourceTime current{now().nanoseconds()};
  const BridgeMonotonicTime monotonic{Clock::now()};
  const auto center = id == CanProtocol::TARGET_ID ?
    state_.targetOutput(current, monotonic) : state_.safetyOutput(current, monotonic);
  return CanProtocol::center(id, center.valid, center.x, center.y, center.source_frame);
}

// ROI 终态同时更新业务结果、授权身份与编码快照；断线时只保存，重连后由查询重发。
// 零编号可保留协议结果位，但不能建立转向身份；转向还须匹配 Mission 已提交状态。
void BridgeRuntime::finishRoi(BridgeRoiResult result, std::uint64_t request_id)
{
  roi_passed_ = result.passed;
  roi_confirmation_.reset();
  if (!route_aborted_ && result.passed && request_id != 0) {
    roi_confirmation_ = TurnIdentity{request_id, result.destination, roi_cycle_generation_};
  }
  const auto frame = CanProtocol::roiResult(
    result.valid, result.passed, static_cast<std::uint8_t>(result.destination));
  if (result.passed) {
    armReply(CanProtocol::ROI_RESULT_ID, frame);
  } else {
    // 失败回执前先提交 pause=1 心跳，再开爪；开爪失败则停心跳，不得在闭合时发出 pause=0。
    queuePauseThenReply(CanProtocol::ROI_RESULT_ID, frame, GripperWork::RejectOpen);
    submitPendingPauseReply();
  }
}

// 回复槽持有载荷，队列持有发送机会；同 ID 的新回复会覆盖快照，因此这里不是逐条可靠交付队列。
void BridgeRuntime::armReply(canid_t id, const can_frame & frame)
{
  if (id == CanProtocol::COMMAND_RESULT_ID) {
    // 发送身份随192载荷一起覆盖，不能借用后来另一条成功缓存的资格。
    const auto command = frame.data[1];
    command_reply_identity_ = frame.data[0] && command != CanProtocol::START_SEARCH &&
      command != CanProtocol::ARRIVAL_POINT &&
      command >= CanProtocol::TURN_COMPLETED && command <= CanProtocol::DELIVERY_COMPLETED ?
      accepted_commands_[command - 1] : std::nullopt;
  }
  // 替换回复快照但不刷新同 ID 的最早入队时间，仍受原队列 TTL 限制。
  replies_.store(id, frame);
  if (can_->socketOpen()) {queue_.queue(id, Clock::now());}
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
    if (*id == CanProtocol::TURN_REQUEST_ID && !turnRequestCurrent()) {
      dropped_frames_ += queue_.drop(*id);
      continue;
    }
    if (*id == CanProtocol::COMMAND_RESULT_ID && command_reply_identity_ &&
      !commandIdentityCurrent(*command_reply_identity_))
    {
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
      // 背压包含串口排队预算及暂时不可写，并不等于七槽队列已满；保留队首及原TTL。
      last_error_ = "CAN transport backpressure";
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "CAN transport on %s is backpressured; queued frames retry within their existing TTL",
        can_interface_.c_str());
      break;
    } else if (result == CanTransport::WriteResult::INTERRUPTED) {
      continue;
    } else {
      // 其他 write 失败或写入长度不等于 CAN_MTU：视为链路不可用，关 socket 等 1 秒重连。
      RCLCPP_ERROR(
        get_logger(), "CAN write to %s failed: %s", can_interface_.c_str(),
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
  // tx只统计主机提交；rx/bus_errors沿用SocketCAN错误帧计数。达妙故障写last_error，不伪造内核错误帧。
  status.can_interface = can_interface_;
  status.socket_open = can_->socketOpen();
  status.interface_up = can_->interfaceUp();
  status.tx_frames = tx_frames_;
  status.rx_frames = rx_frames_;
  status.bus_errors = bus_errors_;
  status.dropped_frames = dropped_frames_;
  status.last_error = last_error_;
  ports_.status_pub->publish(status);
}

// ---- BridgeRuntime：重连、超时与异常清理 ----

// 打开失败后按单调时间隔 1 秒重试；成功后清旧输入与队列，有效周期输出须等待新输入。
void BridgeRuntime::openSocket()
{
  next_open_ = Clock::now() + std::chrono::seconds(1);
  if (!can_->open(can_interface_, open_index_, last_error_)) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "cannot open CAN transport %s: %s", can_interface_.c_str(), last_error_.c_str());
    return;
  }
  last_error_.clear();
  // 撤销旧中心、速度和发送机会，并前移源时间截止；有效周期输出必须收到新输入。
  // 任务、运输阶段和到达保持保留，重连本身不解除握手约束。
  queue_.reset();
  state_.resetInputs({now().nanoseconds()});
  RCLCPP_INFO(
    get_logger(), "CAN transport opened on %s (connection identity %d)",
    can_interface_.c_str(), open_index_);
  publishStatus();
}

// 断线取消本机服务等待、撤销输入和队列，再安排重连；任务状态与到达保持留在核心。
void BridgeRuntime::closeSocket(const std::string & reason)
{
  can_->close();
  clearRequests();
  open_index_ = 0;
  dropped_frames_ += queue_.size();
  queue_.reset();
  state_.resetInputs({now().nanoseconds()});
  last_error_ = reason;
  next_open_ = Clock::now() + std::chrono::seconds(1);
  publishStatus();
}

// expire() 先使等待失效，再移除 rclcpp 请求并保存失败快照。
// 与响应回调使用同一截止边界，迟到回包不能再次完成该请求。
void BridgeRuntime::checkRequestTimeouts(Clock::time_point current)
{
  if (const auto timeout = roi_request_.expire({current})) {
    ports_.roi_client->remove_pending_request(timeout->request_id);
    last_error_ = "ROI service request timed out";
    finishRoi();
    RCLCPP_WARN(get_logger(), "%s", last_error_.c_str());
  }
  if (const auto timeout = command_request_.expire({current})) {
    pending_command_identity_.reset();
    ports_.command_client->remove_pending_request(timeout->request_id);
    last_error_ = "mission command service request timed out";
    armReply(CanProtocol::COMMAND_RESULT_ID,
      CanProtocol::commandResult(false, timeout->command, pending_command_token_));
    RCLCPP_WARN(get_logger(), "%s", last_error_.c_str());
  }
}

// CAN 断线后旧代次不能完成新等待；任务状态和到达保持仍由 BridgeState 保留。
void BridgeRuntime::clearRequests()
{
  if (const auto cancelled = roi_request_.cancel()) {
    ports_.roi_client->remove_pending_request(cancelled->request_id);
    // 断线只取消本机等待，Mission仍可能迟到提交。保存失败供重连后的200查询，
    // 不给已关闭socket入队，也不能把后来的WaitingTurn当作本机ROI通过。
    roi_attempted_ = true;
    finishRoi();
  }
  if (const auto cancelled = command_request_.cancel()) {
    pending_command_identity_.reset();
    ports_.command_client->remove_pending_request(cancelled->request_id);
  }
}

void BridgeRuntime::handleIoFailure(const std::exception & error)
{
  // io() 捕获异常后撤掉发送机会；socket、在途服务和已发生的业务状态保留。
  // 周期帧等待新输入，普通应答等待显式查询；0x193 在双授权仍成立时按固定节拍重授。
  // STM32 按本趟身份去重，重授不重启转向，也不延长本地动作截止。
  last_error_ = error.what();
  dropped_frames_ += queue_.size();
  queue_.reset();
  RCLCPP_ERROR(
    get_logger(), "CAN output cycle failed, dropping queued frames: %s",
    error.what());
  publishStatus();
}

void BridgeRuntime::handleArrivalPoint(std::uint8_t kind)
{
  const auto decision = arrival_point_.onRequest(kind);
  if (decision == ArrivalPointDecision::Discard) {
    ++dropped_frames_;
    return;
  }
  const auto reply = CanProtocol::commandResult(true, CanProtocol::ARRIVAL_POINT);
  if (decision == ArrivalPointDecision::Resend) {
    // 重发只再交命令 5，不再做开合，也不再次把 pause 置 1。
    armReply(CanProtocol::COMMAND_RESULT_ID, reply);
    return;
  }
  queuePauseThenReply(CanProtocol::COMMAND_RESULT_ID, reply, GripperWork::ArrivalMotion, kind);
}

void BridgeRuntime::queuePauseThenReply(
  canid_t id, const can_frame & frame, GripperWork after, std::uint8_t kind)
{
  pending_pause_reply_ = PendingPauseReply{id, frame, after, kind};
}

void BridgeRuntime::submitPendingPauseReply()
{
  if (!pending_pause_reply_ || !can_->socketOpen()) {return;}
  std::string error;
  const auto result = writePauseHeartbeat(
    [this, &error](const can_frame & frame) {return can_->write(frame, error);}, pause_);
  if (result == CanTransport::WriteResult::SENT) {
    ++tx_frames_;
    const auto pending = *pending_pause_reply_;
    pending_pause_reply_.reset();
    armReply(pending.id, pending.frame);
    afterPauseReply(pending.after, pending.kind);
    return;
  }
  if (result == CanTransport::WriteResult::BACKPRESSURE) {
    last_error_ = "CAN transport backpressure";
    return;
  }
  if (result == CanTransport::WriteResult::INTERRUPTED) {return;}
  closeSocket(error);
}

void BridgeRuntime::afterPauseReply(GripperWork after, std::uint8_t kind)
{
  if (after == GripperWork::ArrivalMotion) {
    arrival_point_.markReplied();
    if (!startArrivalGripper(kind)) {
      arrival_point_.markComplete();
      stopHeartbeat();
    }
    return;
  }
  if (after == GripperWork::RejectOpen && !startGripperOpen()) {
    stopHeartbeat();
  }
}

void BridgeRuntime::pollGripper()
{
  if (gripper_work_ == GripperWork::None || !gripper_) {return;}
  const auto status = gripper_->status();
  if (status == s288::MotionStatus::Busy) {return;}
  const bool ok = status == s288::MotionStatus::Succeeded;
  gripper_->acknowledge();
  if (gripper_work_ == GripperWork::ArrivalMotion) {arrival_point_.markComplete();}
  gripper_work_ = GripperWork::None;
  if (ok) {
    pause_ = 0;
  } else {
    stopHeartbeat();
  }
}

void BridgeRuntime::stopHeartbeat()
{
  heartbeat_enabled_ = false;
}

bool BridgeRuntime::startGripperOpen()
{
  if (!gripper_->available() || !gripper_->start_open(gripper_config_.move_s)) {
    return false;
  }
  gripper_work_ = GripperWork::RejectOpen;
  return true;
}

bool BridgeRuntime::startArrivalGripper(std::uint8_t kind)
{
  const bool opening = kind == 2;
  const bool started = opening ?
    gripper_->start_open(gripper_config_.move_s) :
    gripper_->start_close(gripper_config_.move_s);
  if (!started) {return false;}
  gripper_work_ = GripperWork::ArrivalMotion;
  return true;
}
