// 阅读：每 10 毫秒的 io() 先处理超时和到达，再把待发帧交给 flushTxQueue。
// 发送队列、状态和重连在 bridge_session.cpp。ROS 订阅也在那份文件的前半。
// 帧里每个字段的含义在 protocol/usb_protocol.hpp。

#include "node/bridge_node.hpp"
#include "transport/cdc/cdc.hpp"

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


// ---- BridgeRuntime：CAN 事件与到达握手 ----

// 单轮顺序：服务超时 → socket 准入 → 有界读取 → 刷新转向授权 → 事件帧 → 一帧周期数据。
// EINTR 计入收发预算；异常仍由本轮 catch 收尾。
void BridgeRuntime::io()
{
  try {
    checkRequestTimeouts();
    if (!ensureSocketReady()) {return;}
    if (!readAvailableFrames()) {return;}
    pollGripper();
    refreshTurnRequest(Clock::now());
    submitPendingPauseReply();
    if (!can_->socketOpen()) {return;}
    flushTxQueue();
    if (can_->socketOpen()) {sendPeriodicFrame();}
  } catch (const CanTransport::FatalError & error) {
    // 独占失败或半包无法安全恢复时退出；不能等待后自动抢占刚被teleop释放的设备。
    closeSocket(error.what());
    throw;
  } catch (const std::exception & error) {
    handleIoFailure(error);
  }
}

// 每约 3.33 ms 轮发一类周期帧，避免同拍突发塞满串口；每个 ID 目标为 100 Hz。
// 背压或中断时丢弃本次机会，下一拍轮到另一类；载荷始终在提交前重建。
void BridgeRuntime::sendPeriodicFrame()
{
  const canid_t id = periodic_schedule_.next();
  // 夹爪失败停发心跳只走 250 ms 链路超时；任务失联仍由 currentFrame 发 alive=0。
  if (id == CanProtocol::VELOCITY_ID && !heartbeat_enabled_ &&
    mission_watchdog_.alive({Clock::now()}))
  {
    return;
  }
  const can_frame frame = currentFrame(id);
  std::string error;
  const auto result = can_->write(frame, error);
  if (result == CanTransport::WriteResult::SENT) {
    ++tx_frames_;
  } else if (result == CanTransport::WriteResult::BACKPRESSURE) {
    last_error_ = "USB transport backpressure";
  } else if (result != CanTransport::WriteResult::INTERRUPTED) {
    closeSocket(error);
  }
}

bool BridgeRuntime::ensureSocketReady()
{
  // socket 未打开时只按 next_open_ 触发重连（失败后固定 1 秒一次）。
  if (!can_->socketOpen()) {
    if (Clock::now() < next_open_) {return false;}
    openSocket();
    if (!can_->socketOpen()) {return false;}
  }
  int index = 0;
  // 接口 DOWN／类型变化／ioctl 失败：不再发送，直接关 socket 走 1 秒重连。
  if (!can_->check(index, last_error_)) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "USB device %s is not usable: %s", can_interface_.c_str(), last_error_.c_str());
    closeSocket(last_error_);
    return false;
  }
  // index 变化＝同名接口被删除重建／替换：按“接口被替换”处理并重连。
  if (index != open_index_) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "USB endpoint %s was replaced (identity %d -> %d); reopening it",
      can_interface_.c_str(), open_index_, index);
    closeSocket("USB device was replaced");
    return false;
  }
  // 串口后端在check中逐步初始化；尚未就绪时不读业务、不发排队帧。
  return can_->interfaceUp();
}

bool BridgeRuntime::readAvailableFrames()
{
  // 为突发流量留余量，
  // 同时限制非阻塞尝试次数（含 EINTR），确保持续收包时本拍仍会推进发送。
  constexpr int max_reads_per_cycle = 32;
  for (int attempt = 0; attempt < max_reads_per_cycle; ++attempt) {
    can_frame frame{};
    std::string error;
    const auto result = can_->read(frame, error);
    if (result == CanTransport::ReadResult::EMPTY) {break;}
    if (result == CanTransport::ReadResult::INTERRUPTED ||
      result == CanTransport::ReadResult::IGNORED) {continue;}
    if (result == CanTransport::ReadResult::FAILED) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "USB read from %s failed: %s", can_interface_.c_str(), error.c_str());
      closeSocket(error);
      return false;
    }
    if (result == CanTransport::ReadResult::SHORT || frame.len > CAN_MAX_DLEN) {
      ++dropped_frames_;
      continue;
    }
    ++rx_frames_;
    if (!handleCommand(frame)) {++dropped_frames_;}
  }
  return true;
}

// 协议校验后按阶段与请求身份分流；已受理的重复命令只重发回执，在途重复不刷新截止。
// 安全区到达先核对阶段再停坐标；转向完成必须已获授权，之后才调用 Mission。
bool BridgeRuntime::handleCommand(const can_frame & frame)
{
  if (!mission_watchdog_.alive({Clock::now()})) {return false;}
  const auto command = CanProtocol::parseCommand(frame);
  if (command.kind == CanProtocol::CommandKind::NONE) {return false;}
  if (command.kind == CanProtocol::CommandKind::MALFORMED) {
    ++dropped_frames_;
    return true;
  }
  if (command.kind == CanProtocol::CommandKind::ARRIVAL) {
    handleArrival();
    return true;
  }
  if (command.kind == CanProtocol::CommandKind::ARRIVAL_POINT) {
    handleArrivalPoint(command.command);
    return true;
  }
  // 命令应答只有受理与拒绝两种载荷；拒绝同样回显收到的原始 token，不能替换成当前路由编号。
  const auto reply = [this, &command](bool accepted) {
      armCommandReply(accepted, command.command, command.token);
    };
  // 模式确认尚未回执时，同一命令的重发先丢掉。另起一条命令会打断这次确认，直接拒绝。
  if (pending_mode_reply_) {
    if (command.command == pending_mode_reply_->command) {
      ++dropped_frames_;
    } else {
      reply(false);
    }
    return true;
  }
  // 205 在 ROI 或别的命令还在进行时只拒绝。路由和坐标保持到 Mission 真正接受重找。
  if (command.command == CanProtocol::START_SEARCH) {
    if (command_request_.waiting() && command_request_.command() == CanProtocol::START_SEARCH) {
      ++dropped_frames_;
      return true;
    }
    if (roi_request_.waiting() || command_request_.waiting()) {
      reply(false);
      return true;
    }
    requestCommand(command.command);
    return true;
  }
  const auto & mission = state_.missionState();
  const bool current_route = !route_aborted_ && mission && mission->roi_request_id != 0 &&
    mission->destination != BridgeDestination::None &&
    command.token == static_cast<std::uint32_t>(mission->roi_request_id);
  const bool current_delivery = current_route &&
    mission->cycle_phase == BridgeCyclePhase::WaitingTransfer &&
    mission->operation == BridgeOperation::Idle;
  const bool delivery_query = command.command == CanProtocol::DELIVERY_COMPLETED &&
    !current_delivery && deliveryQueryIdentity(command.token);
  if (!delivery_query && (!current_route ||
    (command.command == CanProtocol::DELIVERY_COMPLETED && !current_delivery)))
  {
    // 1/2/4先核对完整路由的低位；拒绝也回显收到的token，不拿当前token替换旧请求。
    reply(false);
    return true;
  }
  if (commandAlreadyAccepted(command.command, command.token, delivery_query)) {
    reply(true);
    command_reply_identity_->delivery_query = delivery_query;
    return true;
  }
  // 原服务还在途时重复命令不消费新槽，也不能把未完成请求伪装成失败/成功。
  if (command_request_.waiting() && command_request_.command() == command.command) {
    ++dropped_frames_;
    return true;
  }
  // 成功ROS回包丢失时，不从MissionStatus直接伪造成功：同趟后续阶段可再询问Mission。
  // 查询不重新保持坐标或建立转向授权；Fault/205/换趟仍由前面的路由门和回复身份撤销。
  const auto phase = state_.cyclePhase();
  if (command.command == CanProtocol::SAFETY_ARRIVED) {
    if (phase != BridgeCyclePhase::NavigatingSafety &&
      phase != BridgeCyclePhase::WaitingTransfer) {
      reply(false);
      return true;
    }
    if (phase == BridgeCyclePhase::NavigatingSafety) {holdCoordinates();}
  }
  if (command.command == CanProtocol::TURN_COMPLETED && !turnAuthorized() &&
    phase != BridgeCyclePhase::SwitchingField && phase != BridgeCyclePhase::NavigatingSafety)
  {
    reply(false);
    return true;
  }
  requestCommand(command.command, command.token, delivery_query);
  return true;
}

// STM32 停稳后发类型 16 请求 ROI。夹爪开合由类型 21 到点触发，不在类型 16 之前由 STM32 完成。
// 首次到达先停坐标并确认，再请求 ROI；重复到达只重发当前轮结果。
// 待转向时核对授权；仅本进程未尝试过 ROI 才能按已提交状态恢复，失败不能被查询改成成功。
// 非寻找、检查、待转向或 ROI 失败阶段拒绝到达。
void BridgeRuntime::handleArrival()
{
  if (route_aborted_) {
    queuePauseThenReply(CanProtocol::ARRIVAL_ACK_ID, CanProtocol::arrivalAck(false),
      GripperWork::RejectOpen);
    return;
  }
  const auto phase = state_.cyclePhase();
  if (phase == BridgeCyclePhase::SearchingTarget) {
    const bool duplicate = state_.locallyHeld();
    holdCoordinates();
    armReply(CanProtocol::ARRIVAL_ACK_ID, CanProtocol::arrivalAck());
    if (!duplicate) {
      roi_passed_.reset();
      roi_confirmation_.reset();
      roi_attempted_ = true;
      roi_cycle_generation_ = state_.missionState()->cycle_generation;
      requestRoi();
    } else if (roi_passed_.has_value()) {
      // 失败帧已在槽里，但仍要等 pause 心跳先上线。查询不能把上一轮通过帧提前发出。
      if (pending_pause_reply_ && pending_pause_reply_->id == CanProtocol::ROI_RESULT_ID) {
        return;
      }
      armReply(CanProtocol::ROI_RESULT_ID, replies_.frame(CanProtocol::ROI_RESULT_ID));
    }
    return;
  }
  if (phase == BridgeCyclePhase::CheckingRoi || phase == BridgeCyclePhase::WaitingTurn ||
    phase == BridgeCyclePhase::RoiFailed)
  {
    // 重发是当前轮的确认机会，不重新发起 ROI，也不重放运输提交。
    armReply(CanProtocol::ARRIVAL_ACK_ID, CanProtocol::arrivalAck());
    if (phase == BridgeCyclePhase::WaitingTurn) {
      // 尚未尝试ROI时显式查询可核对已提交身份；本机失败或放弃不能借重复200改成功。
      if (!roi_attempted_ && state_.waitingTurn()) {
        const auto & mission = *state_.missionState();
        roi_confirmation_ = TurnIdentity{
          mission.roi_request_id, mission.destination, mission.cycle_generation};
      }
      refreshTurnRequest(Clock::now(), true);
      if (roi_attempted_ && roi_passed_.has_value() &&
        (turnAuthorized() || roi_passed_ == false))
      {
        // 成功191丢失也须可查询；失败仍只重放失败，不能被迟到提交改成通过。
        armReply(CanProtocol::ROI_RESULT_ID, replies_.frame(CanProtocol::ROI_RESULT_ID));
      }
    }
    // 本地失败可能先于Mission终态，甚至遇到状态发布失败；CheckingRoi也能补失败结果。
    // 只重发已保存的191，不再请求ROI；成功结果仍沿WaitingTurn的身份核对路径查询。
    if ((phase == BridgeCyclePhase::RoiFailed && roi_passed_.has_value()) ||
      (phase == BridgeCyclePhase::CheckingRoi && roi_passed_ == false))
    {
      armReply(CanProtocol::ROI_RESULT_ID, replies_.frame(CanProtocol::ROI_RESULT_ID));
    }
    return;
  }
  queuePauseThenReply(CanProtocol::ARRIVAL_ACK_ID, CanProtocol::arrivalAck(false),
    GripperWork::RejectOpen);
}

void BridgeRuntime::holdCoordinates(bool until_search)
{
  state_.holdCoordinates({now().nanoseconds()}, until_search);
  dropped_frames_ += queue_.drop(CanProtocol::TARGET_ID);
  dropped_frames_ += queue_.drop(CanProtocol::SAFETY_ID);
}

// ---- BridgeRuntime：服务请求与身份确认 ----

// ROI 与命令各一等待槽；服务回包和 Mission 状态可乱序，双身份匹配才发193。
// 收齐 191 通过与 193 后只启动一次去程转向，191 本身不授权转向。

// 同类请求忙时忽略重复指令；服务缺席或超时均给失败快照，不延长旧等待。
void BridgeRuntime::requestRoi()
{
  using CheckRoi = interface_pkg::srv::CheckRoi;
  checkRequestTimeouts();
  if (roi_request_.waiting()) {
    ++dropped_frames_;
    return;
  }
  if (!ports_.roi_client->service_is_ready()) {
    finishRoi();
    RCLCPP_WARN(get_logger(), "arrival ROI service is not ready");
    return;
  }
  // 忙时不刷新截止；成功 begin 后捕获本代次，迟到服务响应不能完成重试中的新请求。
  const auto generation = *roi_request_.begin({Clock::now()}, request_timeout_);
  try {
    const auto pending =
      ports_.roi_client->async_send_request(
      std::make_shared<CheckRoi::Request>(),
      [this, generation](rclcpp::Client<CheckRoi>::SharedFuture result) {
        onRoiReply(generation, std::move(result));
      });
    roi_request_.bind(generation, pending.request_id);
  } catch (const std::exception & error) {
    roi_request_.cancel();
    last_error_ = error.what();
    finishRoi();
  }
}

void BridgeRuntime::onRoiReply(
  std::uint64_t generation, rclcpp::Client<interface_pkg::srv::CheckRoi>::SharedFuture result)
{
  const auto current = Clock::now();
  // 使用同一次单调采样先过期再完成，恰好到截止也执行超时；通过代次检查后才读取结果。
  checkRequestTimeouts(current);
  if (!roi_request_.complete(generation, {current})) {return;}
  try {
    const auto & checked = result.get()->result;
    BridgeRoiResult outcome;
    outcome.valid = checked.valid;
    outcome.passed = checked.valid && checked.passed;
    if (outcome.passed) {
      outcome.destination = static_cast<BridgeDestination>(
        RecognitionValueValidation::cargoSide(checked.orange_count));
    }
    finishRoi(outcome, checked.request_id);
    refreshTurnRequest(current);
  } catch (const std::exception & error) {
    last_error_ = error.what();
    finishRoi();
  }
}

// 服务应答回显本次命令；1/2/3成功只表示受理，4成功表示已记录送达，均不触发机械动作。
void BridgeRuntime::requestCommand(std::uint8_t command, std::uint32_t token, bool delivery_query)
{
  using Command = interface_pkg::srv::MissionCommand;
  checkRequestTimeouts();
  if (command_request_.waiting()) {
    ++dropped_frames_;
    return;
  }
  if (!ports_.command_client->service_is_ready()) {
    armReply(CanProtocol::COMMAND_RESULT_ID, CanProtocol::commandResult(false, command, token));
    RCLCPP_WARN(get_logger(), "mission command service is not ready");
    return;
  }
  const auto generation = *command_request_.begin({Clock::now()}, request_timeout_, command);
  pending_command_token_ = token;
  pending_command_identity_.reset();
  if (delivery_query) {
    pending_command_identity_ = *deliveryQueryIdentity(token);
    pending_command_identity_->delivery_query = true;
  } else if (const auto & mission = state_.missionState(); mission && command != CanProtocol::START_SEARCH) {
    // 只有首次203从WaitingTurn开始切field才递增代次；后续阶段查询沿用当前代次。
    pending_command_identity_ = CommandIdentity{
      command, mission->cycle_generation +
      (command == CanProtocol::TURN_COMPLETED &&
      mission->cycle_phase == BridgeCyclePhase::WaitingTurn ? 1 : 0),
      mission->roi_request_id, mission->destination, mission->cycle_phase};
  }
  try {
    auto request = std::make_shared<Command::Request>();
    request->command = command;
    if (pending_command_identity_) {
      request->roi_request_id = pending_command_identity_->roi_request_id;
    }
    const auto pending = ports_.command_client->async_send_request(
      request,
      [this, command, generation](rclcpp::Client<Command>::SharedFuture result) {
        onCommandReply(generation, command, std::move(result));
      });
    command_request_.bind(generation, pending.request_id);
    if (command == CanProtocol::DELIVERY_COMPLETED && !delivery_query) {
      last_delivery_request_ = pending_command_identity_;
    }
  } catch (const std::exception & error) {
    command_request_.cancel();
    pending_command_identity_.reset();
    last_error_ = error.what();
    armReply(CanProtocol::COMMAND_RESULT_ID, CanProtocol::commandResult(false, command, token));
  }
}

void BridgeRuntime::onCommandReply(
  std::uint64_t generation, std::uint8_t command,
  rclcpp::Client<interface_pkg::srv::MissionCommand>::SharedFuture result)
{
  const auto current = Clock::now();
  checkRequestTimeouts(current);
  // 与 ROI 相同：先核对截止和代次再 get，失效回包不能读取 future 或消费新等待。
  if (!command_request_.complete(generation, {current})) {return;}
  const auto identity = std::exchange(pending_command_identity_, std::nullopt);
  // 205只撤销旧回复资格，不取消原等待或回滚Mission已经记录的投送。
  // 新授权也不能接收旧服务的结论；先核对身份，再读取future。
  bool accepted = false;
  try {
    accepted = result.get()->accepted;
  } catch (const std::exception & error) {
    last_error_ = error.what();
  }
  const auto token = pending_command_token_;
  // 命令 1/3 的服务成功只表示已经去切换。类型 7 要等 field/seg 真正确认，失败也必须回拒绝。
  if (command == CanProtocol::TURN_COMPLETED || command == CanProtocol::START_SEARCH) {
    if (!accepted || switchFailed()) {
      armCommandReply(false, command, token);
      return;
    }
    if (command == CanProtocol::START_SEARCH) {commitSearchAbort();}
    if (switchConfirmed(command)) {
      if (const auto & mission = state_.missionState()) {
        accepted_commands_[command - 1] = CommandIdentity{
          command, mission->cycle_generation, mission->roi_request_id,
          mission->destination, mission->cycle_phase, false};
      }
      armCommandReply(true, command, token);
      return;
    }
    pending_mode_reply_ = PendingModeReply{command, token, current + request_timeout_};
    return;
  }
  // 这一代已经结束。身份对不上说明路由在调用期间变了，回拒绝，不能把回执吞掉。
  if (!accepted || !identity || !commandIdentityCurrent(*identity)) {
    armCommandReply(false, command, token);
    return;
  }
  accepted_commands_[command - 1] = identity;
  armCommandReply(true, command, token);
}

bool BridgeRuntime::turnAuthorized() const
{
  const auto & mission = state_.missionState();
  return !route_aborted_ && roi_confirmation_ && state_.waitingTurn() && mission &&
         roi_confirmation_->roi_request_id == mission->roi_request_id &&
         roi_confirmation_->destination == mission->destination &&
         roi_confirmation_->cycle_generation == mission->cycle_generation;
}

bool BridgeRuntime::commandAlreadyAccepted(
  std::uint8_t command, std::uint32_t token, bool delivery_query) const
{
  const auto & accepted = accepted_commands_[command - 1];
  if (!accepted || static_cast<std::uint32_t>(accepted->roi_request_id) != token) {return false;}
  auto identity = *accepted;
  // 当前投送必须匹配完整ROI；历史查询成功不能绕过下一趟的完整身份检查。
  identity.delivery_query = delivery_query;
  return commandIdentityCurrent(identity);
}

const BridgeRuntime::CommandIdentity * BridgeRuntime::deliveryQueryIdentity(std::uint32_t token) const
{
  // 成功事实和最近尝试分开，可能属于不同趟；未确认的新尝试不能挤掉已确认收据。
  // 缓存未命中时查询仍占用唯一命令等待，不能用Later代替这个完整编号的应答。
  for (const auto * identity : {&accepted_commands_[CanProtocol::DELIVERY_COMPLETED - 1],
      &last_delivery_request_})
  {
    if (*identity && static_cast<std::uint32_t>((*identity)->roi_request_id) == token) {
      return &**identity;
    }
  }
  return nullptr;
}

bool BridgeRuntime::commandIdentityCurrent(const CommandIdentity & identity) const
{
  if (identity.delivery_query) {
    const auto * known = deliveryQueryIdentity(static_cast<std::uint32_t>(identity.roi_request_id));
    return known && known->roi_request_id == identity.roi_request_id;
  }
  const auto & mission = state_.missionState();
  if (route_aborted_ || !mission || identity.roi_request_id != mission->roi_request_id ||
    identity.destination != mission->destination) {return false;}
  // 服务先于状态话题完成时也可重发已确认的回执；既不重做请求，也不解坐标保持。
  if (identity.origin_phase == mission->cycle_phase &&
    identity.expected_generation == mission->cycle_generation +
    (identity.command == CanProtocol::TURN_COMPLETED &&
    identity.origin_phase == BridgeCyclePhase::WaitingTurn ? 1 : 0)) {return true;}
  if (identity.expected_generation != mission->cycle_generation) {return false;}
  if (identity.command == CanProtocol::TURN_COMPLETED) {
    return mission->cycle_phase == BridgeCyclePhase::SwitchingField ||
           mission->cycle_phase == BridgeCyclePhase::NavigatingSafety;
  }
  // 盲推与返程共用WaitingTransfer：204/206可补旧收据，203不能作返程确认。
  // 补回执不授权再次推送或转向；STM32只在对应本地等待步骤消费一次。
  return mission->cycle_phase == BridgeCyclePhase::WaitingTransfer;
}

bool BridgeRuntime::turnRequestCurrent() const
{
  const auto & mission = state_.missionState();
  return turn_ && turnAuthorized() && mission &&
         turn_->roi_request_id == mission->roi_request_id &&
         turn_->destination == mission->destination &&
         turn_->cycle_generation == mission->cycle_generation;
}

void BridgeRuntime::armCommandReply(bool accepted, std::uint8_t command, std::uint32_t token)
{
  armReply(CanProtocol::COMMAND_RESULT_ID, CanProtocol::commandResult(accepted, command, token));
}

bool BridgeRuntime::switchFailed() const
{
  const auto & mission = state_.missionState();
  if (!mission) {return false;}
  return mission->cycle_phase == BridgeCyclePhase::Fault ||
    mission->outcome == BridgeOutcome::Rejected ||
    mission->outcome == BridgeOutcome::Timeout;
}

bool BridgeRuntime::switchConfirmed(std::uint8_t command) const
{
  const auto & mission = state_.missionState();
  if (!mission || switchFailed()) {return false;}
  if (command == CanProtocol::TURN_COMPLETED) {
    return !route_aborted_ &&
      mission->cycle_phase == BridgeCyclePhase::NavigatingSafety &&
      mission->destination != BridgeDestination::None;
  }
  if (command == CanProtocol::START_SEARCH) {
    return mission->cycle_phase == BridgeCyclePhase::SearchingTarget;
  }
  return false;
}

void BridgeRuntime::commitSearchAbort()
{
  holdCoordinates(true);
  route_aborted_ = true;
  arrival_point_.reset();
  pending_pause_reply_.reset();
  roi_confirmation_.reset();
  accepted_commands_[CanProtocol::TURN_COMPLETED - 1].reset();
  accepted_commands_[CanProtocol::SAFETY_ARRIVED - 1].reset();
  command_reply_identity_.reset();
  dropped_frames_ += queue_.drop(CanProtocol::COMMAND_RESULT_ID);
  turn_.reset();
  dropped_frames_ += queue_.drop(CanProtocol::TURN_REQUEST_ID);
}

void BridgeRuntime::resolvePendingModeReply(Clock::time_point current)
{
  if (!pending_mode_reply_) {return;}
  const auto pending = *pending_mode_reply_;
  if (switchFailed() || current >= pending.deadline) {
    pending_mode_reply_.reset();
    last_error_ = "recognition switch was not confirmed";
    armCommandReply(false, pending.command, pending.token);
    return;
  }
  if (!switchConfirmed(pending.command)) {return;}
  pending_mode_reply_.reset();
  if (const auto & mission = state_.missionState()) {
    accepted_commands_[pending.command - 1] = CommandIdentity{
      pending.command, mission->cycle_generation, mission->roi_request_id,
      mission->destination, mission->cycle_phase, false};
  }
  armCommandReply(true, pending.command, pending.token);
}

