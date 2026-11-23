// Copyright 2026 li
// SPDX-License-Identifier: Apache-2.0

#include "state/bridge_transactions.hpp"

#include <exception>
#include <utility>

BridgeTransactions::BridgeTransactions(
  BridgeState & state, ArrivalPointSession & arrival_point, Ports & ports,
  Clock::duration request_timeout)
: state_(state), arrival_point_(arrival_point), ports_(ports), request_timeout_(request_timeout)
{
}

void BridgeTransactions::onStateUpdate()
{
  if (!state_.locallyHeld() && state_.cyclePhase() == BridgeCyclePhase::SearchingTarget) {
    route_aborted_ = false;
  }
}

void BridgeTransactions::handleMissionCommand(std::uint8_t command, std::uint32_t token)
{
  // 命令应答只有受理与拒绝两种载荷；拒绝同样回显收到的原始 token，不能替换成当前路由编号。
  const auto reply = [this, command, token](bool accepted) {
      armCommandReply(accepted, command, token);
    };
  // 搜索和首次对准各自保存编号水位。在途重传不延长等待，已结束事务不再调用 Mission。
  if (command == CanProtocol::START_SEARCH || command == CanProtocol::START_ALIGN) {
    const bool initial = command == CanProtocol::START_ALIGN;
    auto & session = initial ? initial_align_request_ : search_request_;
    const auto decision = session.onRequest(token);
    if (decision == SearchRequestDecision::Reject) {
      reply(false);
      return;
    }
    if (decision == SearchRequestDecision::Pending) {
      ports_.dropRequest();
      return;
    }
    if (decision == SearchRequestDecision::Replay) {
      reply(*session.result() && (!initial || switchConfirmed(command)));
      return;
    }
    // 新事务忙时的拒绝也是终态，不能让同 ID 在稍后变成一次新的模式切换。
    if (pending_mode_reply_ || roi_request_.waiting() || command_request_.waiting()) {
      reply(false);
      return;
    }
    if (initial) {
      const auto & mission = state_.missionState();
      if (!mission || mission->cycle_generation != 0 || mission->transport_phase != BridgeTransportPhase::First ||
        mission->roi_request_id != 0 || mission->destination != BridgeDestination::None ||
        mission->operation != BridgeOperation::Idle || route_aborted_ || state_.locallyHeld() ||
        arrival_point_.busy() || (mission->cycle_phase != BridgeCyclePhase::SearchingTarget &&
        mission->cycle_phase != BridgeCyclePhase::Fault))
      {
        reply(false);
        return;
      }
      initial_align_deadline_ = ports_.monotonicNow() + request_timeout_;
    }
    requestCommand(command, token);
    return;
  }
  // 模式确认尚未回执时，同一命令的重发先丢掉。另起一条命令直接拒绝。
  if (pending_mode_reply_) {
    if (command == pending_mode_reply_->command) {
      ports_.dropRequest();
    } else {
      reply(false);
    }
    return;
  }
  const auto & mission = state_.missionState();
  const bool current_route = !route_aborted_ && mission && mission->roi_request_id != 0 &&
    mission->destination != BridgeDestination::None &&
    token == static_cast<std::uint32_t>(mission->roi_request_id);
  const bool transfer_route = current_route &&
    (mission->cycle_phase == BridgeCyclePhase::WaitingTransfer ||
    mission->cycle_phase == BridgeCyclePhase::LocalizingStart);
  const bool current_delivery = transfer_route &&
    mission->operation == BridgeOperation::Idle;
  const bool delivery_query = command == CanProtocol::DELIVERY_COMPLETED &&
    !current_delivery && deliveryQueryIdentity(token);
  if (!delivery_query && (!current_route ||
    (command == CanProtocol::DELIVERY_COMPLETED && !current_delivery)))
  {
    // 1/2/4先核对完整路由的低位；拒绝也回显收到的token，不拿当前token替换旧请求。
    reply(false);
    return;
  }
  if (commandAlreadyAccepted(command, token, delivery_query)) {
    reply(true);
    command_reply_identity_->delivery_query = delivery_query;
    return;
  }
  // 原服务还在途时重复命令不消费新槽，也不能把未完成请求伪装成失败/成功。
  if (command_request_.waiting() && command_request_.command() == command) {
    ports_.dropRequest();
    return;
  }
  // 成功ROS回包丢失时，不从MissionStatus直接伪造成功：同趟后续阶段可再询问Mission。
  // 查询不重新保持坐标或建立转向授权；Fault/205/换趟仍由前面的路由门和回复身份撤销。
  const auto phase = state_.cyclePhase();
  if (command == CanProtocol::SAFETY_ARRIVED) {
    if (phase != BridgeCyclePhase::NavigatingSafety && !transfer_route) {
      reply(false);
      return;
    }
    if (phase == BridgeCyclePhase::NavigatingSafety) {state_.holdCoordinates(ports_.sourceNow());}
  }
  if (command == CanProtocol::TURN_COMPLETED && !turnAuthorized() &&
    phase != BridgeCyclePhase::SwitchingField && phase != BridgeCyclePhase::NavigatingSafety)
  {
    reply(false);
    return;
  }
  requestCommand(command, token, delivery_query);
}

// 正式搬运在夹取并完成后退定距后发类型 16 请求 ROI；夹爪动作由类型 21 单独触发。
// 首次到达先停坐标并确认，再请求 ROI；重复到达只重发当前轮结果。
// 待转向时核对授权；仅本进程未尝试过 ROI 才能按已提交状态恢复，失败不能被查询改成成功。
// 非寻找、检查、待转向或 ROI 失败阶段拒绝到达。
void BridgeTransactions::handleArrival(Service service)
{
  const auto & mission = state_.missionState();
  if (initial_align_request_.busy() || (mission && roi_attempted_ &&
    roi_cycle_generation_ == mission->cycle_generation && route_service_ != service)) {
    armReply(CanProtocol::ARRIVAL_ACK_ID, CanProtocol::arrivalAck(false));
    return;
  }
  if (route_aborted_) {
    if (service == Service::Roi) {arrival_point_.authorizeRejectOpen();}
    armReply(CanProtocol::ARRIVAL_ACK_ID, CanProtocol::arrivalAck(false));
    return;
  }
  const auto phase = state_.cyclePhase();
  if (service == Service::PickupRoute) {
    // 闭爪期间目标可能移出画面，只使用这次抓取受理时锁定的类别及同编号终态。
    const auto grasp = pickup_ ? arrival_point_.result(1, pickup_->grasp_token) : std::nullopt;
    if (!grasp || !grasp->success || !mission || pickup_->cycle_generation != mission->cycle_generation ||
      (phase != BridgeCyclePhase::SearchingTarget && phase != BridgeCyclePhase::WaitingTurn) ||
      (phase == BridgeCyclePhase::WaitingTurn && mission->roi_request_id != grasp->request_id)) {
      armReply(CanProtocol::ARRIVAL_ACK_ID, CanProtocol::arrivalAck(false));
      return;
    }
  }
  if (phase == BridgeCyclePhase::SearchingTarget) {
    const bool duplicate = state_.locallyHeld();
    state_.holdCoordinates(ports_.sourceNow());
    armReply(CanProtocol::ARRIVAL_ACK_ID, CanProtocol::arrivalAck());
    if (!duplicate) {
      roi_passed_.reset();
      roi_confirmation_.reset();
      roi_attempted_ = true;
      route_service_ = service;
      roi_cycle_generation_ = state_.missionState()->cycle_generation;
      requestRoute();
    } else if (roi_passed_.has_value()) {
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
      refreshTurnRequest(ports_.monotonicNow(), true);
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
  if (service == Service::Roi) {arrival_point_.authorizeRejectOpen();}
  armReply(CanProtocol::ARRIVAL_ACK_ID, CanProtocol::arrivalAck(false));
}

ArrivalPointDecision BridgeTransactions::handleArrivalPoint(
  std::uint8_t kind, std::uint32_t request_id, bool motion_lease_open, std::uint32_t source_frame)
{
  const auto phase = state_.cyclePhase();
  const auto class_id = kind == 1 ? state_.graspClass(source_frame, ports_.sourceNow(), {ports_.monotonicNow()}) : "";
  const bool can_grasp = kind == 1 && !route_aborted_ &&
    phase == BridgeCyclePhase::SearchingTarget && !state_.locallyHeld() && !class_id.empty();
  const bool can_release = kind == 2 && !route_aborted_ && phase == BridgeCyclePhase::WaitingTransfer;
  const bool authorized = kind == 0 ||
    (!initial_align_request_.busy() && motion_lease_open && (can_grasp || can_release || kind == 3));
  const auto decision = arrival_point_.onRequest(kind, request_id, authorized, source_frame);
  if (decision == ArrivalPointDecision::Accept && kind == 1) {
    pickup_ = Pickup{class_id, request_id, state_.missionState()->cycle_generation};
  }
  if (decision == ArrivalPointDecision::Accept && kind == 3) {pickup_.reset();}
  armCommandReply(decision != ArrivalPointDecision::Reject, CanProtocol::ARRIVAL_POINT, request_id);
  return decision;
}

// 同类请求忙时忽略重复指令；服务缺席或超时均给失败快照，不延长旧等待。
void BridgeTransactions::requestRoute()
{
  checkRequestTimeouts(ports_.monotonicNow());
  if (roi_request_.waiting()) {
    ports_.dropRequest();
    return;
  }
  if (!ports_.serviceReady(route_service_)) {
    failRoute();
    ports_.warnRequest("arrival route service is not ready");
    return;
  }
  // 忙时不刷新截止；成功 begin 后捕获本代次，迟到服务响应不能完成重试中的新请求。
  const auto generation = *roi_request_.begin({ports_.monotonicNow()}, request_timeout_);
  try {
    const auto request_id = route_service_ == Service::PickupRoute ?
      ports_.sendPickupRoute(generation, pickup_->class_id, pickup_->grasp_token) :
      ports_.sendRoi(generation);
    roi_request_.bind(generation, request_id);
  } catch (const std::exception & error) {
    roi_request_.cancel();
    ports_.setRequestError(error.what());
    failRoute();
  }
}

bool BridgeTransactions::acceptRoiReply(std::uint64_t generation, Clock::time_point current)
{
  checkRequestTimeouts(current);
  return roi_request_.complete(generation, {current});
}

// 服务应答回显本次命令；1/3 仍须等模式确认，4 成功表示已记录送达。
void BridgeTransactions::requestCommand(std::uint8_t command, std::uint32_t token, bool delivery_query)
{
  checkRequestTimeouts(ports_.monotonicNow());
  if (command_request_.waiting()) {
    ports_.dropRequest();
    return;
  }
  if (!ports_.serviceReady(Service::Command)) {
    armCommandReply(false, command, token);
    ports_.warnRequest("mission command service is not ready");
    return;
  }
  const auto generation = *command_request_.begin({ports_.monotonicNow()}, request_timeout_, command);
  // handleCommand 已要求合法 Mission 心跳；必须在发送前保存，响应时状态可能已推进。
  const auto requested_cycle_generation = state_.missionState()->cycle_generation;
  pending_command_token_ = token;
  pending_command_identity_.reset();
  if (delivery_query) {
    pending_command_identity_ = *deliveryQueryIdentity(token);
    pending_command_identity_->delivery_query = true;
  } else if (const auto & mission = state_.missionState();
    mission && command != CanProtocol::START_SEARCH && command != CanProtocol::START_ALIGN) {
    // 只有首次203从WaitingTurn开始切field才递增代次；后续阶段查询沿用当前代次。
    pending_command_identity_ = CommandIdentity{
      command, mission->cycle_generation +
      (command == CanProtocol::TURN_COMPLETED &&
      mission->cycle_phase == BridgeCyclePhase::WaitingTurn ? 1 : 0),
      mission->roi_request_id, mission->destination, mission->cycle_phase};
  }
  try {
    const auto request_id = ports_.sendCommand({
      command, pending_command_identity_ ? pending_command_identity_->roi_request_id : 0,
      generation, requested_cycle_generation});
    command_request_.bind(generation, request_id);
    if (command == CanProtocol::DELIVERY_COMPLETED && !delivery_query) {
      last_delivery_request_ = pending_command_identity_;
    }
  } catch (const std::exception & error) {
    command_request_.cancel();
    pending_command_identity_.reset();
    ports_.setRequestError(error.what());
    armCommandReply(false, command, token);
  }
}

bool BridgeTransactions::acceptCommandReply(std::uint64_t generation, Clock::time_point current)
{
  checkRequestTimeouts(current);
  return command_request_.complete(generation, {current});
}

void BridgeTransactions::commandReply(
  std::uint8_t command, std::uint64_t requested_cycle_generation, bool accepted,
  Clock::time_point current)
{
  const auto identity = std::exchange(pending_command_identity_, std::nullopt);
  const auto token = pending_command_token_;
  // 命令 1/3 的服务成功只表示已经去切换。类型 7 要等 field/seg 真正确认，失败也必须回拒绝。
  if (command == CanProtocol::TURN_COMPLETED || command == CanProtocol::START_SEARCH ||
    command == CanProtocol::START_ALIGN) {
    if (!accepted || modeSwitchFailed(command, requested_cycle_generation)) {
      armCommandReply(false, command, token);
      return;
    }
    if (command == CanProtocol::START_SEARCH) {commitSearchAbort(requested_cycle_generation);}
    if (switchConfirmed(command)) {
      if (const auto & mission = state_.missionState(); mission && command == CanProtocol::TURN_COMPLETED) {
        accepted_commands_[command - 1] = CommandIdentity{
          command, mission->cycle_generation, mission->roi_request_id,
          mission->destination, mission->cycle_phase, false};
      }
      armCommandReply(true, command, token);
      return;
    }
    pending_mode_reply_ = PendingModeReply{
      command, token, requested_cycle_generation,
      command == CanProtocol::START_ALIGN ? initial_align_deadline_ : current + request_timeout_};
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

bool BridgeTransactions::turnAuthorized() const
{
  const auto & mission = state_.missionState();
  return !route_aborted_ && roi_confirmation_ && state_.waitingTurn() && mission &&
         roi_confirmation_->roi_request_id == mission->roi_request_id &&
         roi_confirmation_->destination == mission->destination &&
         roi_confirmation_->cycle_generation == mission->cycle_generation;
}

bool BridgeTransactions::commandAlreadyAccepted(
  std::uint8_t command, std::uint32_t token, bool delivery_query) const
{
  const auto & accepted = accepted_commands_[command - 1];
  if (!accepted || static_cast<std::uint32_t>(accepted->roi_request_id) != token) {return false;}
  auto identity = *accepted;
  // 当前投送必须匹配完整ROI；历史查询成功不能绕过下一趟的完整身份检查。
  identity.delivery_query = delivery_query;
  return commandIdentityCurrent(identity);
}

const BridgeTransactions::CommandIdentity * BridgeTransactions::deliveryQueryIdentity(std::uint32_t token) const
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

bool BridgeTransactions::commandIdentityCurrent(const CommandIdentity & identity) const
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
  // 到达、送达回执可能跨入起点定位阶段，仍须通过上面的完整路由核对。
  return mission->cycle_phase == BridgeCyclePhase::WaitingTransfer ||
         mission->cycle_phase == BridgeCyclePhase::LocalizingStart;
}

bool BridgeTransactions::turnRequestCurrent() const
{
  const auto & mission = state_.missionState();
  return turn_ && turnAuthorized() && mission &&
         turn_->roi_request_id == mission->roi_request_id &&
         turn_->destination == mission->destination &&
         turn_->cycle_generation == mission->cycle_generation;
}

void BridgeTransactions::armCommandReply(bool accepted, std::uint8_t command, std::uint32_t token)
{
  if (command == CanProtocol::START_SEARCH) {search_request_.complete(token, accepted);}
  if (command == CanProtocol::START_ALIGN) {initial_align_request_.complete(token, accepted);}
  armReply(CanProtocol::COMMAND_RESULT_ID, CanProtocol::commandResult(accepted, command, token));
}

bool BridgeTransactions::switchFailed() const
{
  const auto & mission = state_.missionState();
  if (!mission) {return false;}
  return mission->cycle_phase == BridgeCyclePhase::Fault ||
    mission->outcome == BridgeOutcome::Rejected ||
    mission->outcome == BridgeOutcome::Timeout;
}

bool BridgeTransactions::modeSwitchFailed(
  std::uint8_t command, std::uint64_t requested_generation) const
{
  // 新搜索可从 Fault 恢复；服务响应先到时，旧代 Fault 不是这次切换的终态。
  const auto & mission = state_.missionState();
  if (command == CanProtocol::START_ALIGN) {
    if (!mission || mission->transport_phase != BridgeTransportPhase::First ||
      mission->roi_request_id != 0 || mission->destination != BridgeDestination::None ||
      mission->cycle_generation > requested_generation + 1) {return true;}
    // 服务先到时可能仍是请求前无路由的 Fault，等新代状态判断本次切换的结果。
    if (mission->cycle_generation == requested_generation) {
      return mission->cycle_phase != BridgeCyclePhase::SearchingTarget &&
             mission->cycle_phase != BridgeCyclePhase::Fault;
    }
    if (mission->cycle_phase != BridgeCyclePhase::SwitchingField &&
      mission->cycle_phase != BridgeCyclePhase::InitialLocalizing) {return true;}
  }
  if (command == CanProtocol::START_SEARCH && mission &&
    mission->cycle_generation <= requested_generation) {return false;}
  return switchFailed();
}

bool BridgeTransactions::switchConfirmed(std::uint8_t command) const
{
  const auto & mission = state_.missionState();
  if (!mission || switchFailed()) {return false;}
  if (command == CanProtocol::START_ALIGN) {
    return mission->cycle_generation == 1 && state_.initialLocalizationConfirmed();
  }
  if (command == CanProtocol::TURN_COMPLETED) {
    return !route_aborted_ &&
      mission->cycle_phase == BridgeCyclePhase::NavigatingSafety &&
      mission->destination != BridgeDestination::None;
  }
  if (command == CanProtocol::START_SEARCH) {
    return !state_.locallyHeld() && mission->cycle_phase == BridgeCyclePhase::SearchingTarget;
  }
  return false;
}

void BridgeTransactions::commitSearchAbort(std::uint64_t requested_cycle_generation)
{
  pickup_.reset();
  state_.holdForSearch(ports_.sourceNow(), requested_cycle_generation);
  // 若新搜索终态已先到，核心已经解除闭锁，不再等待下一次心跳清除此标志。
  route_aborted_ = state_.locallyHeld();
  arrival_point_.beginSearch();
  ports_.dropReply(CanProtocol::GRIPPER_RESULT_ID);
  roi_confirmation_.reset();
  accepted_commands_[CanProtocol::TURN_COMPLETED - 1].reset();
  accepted_commands_[CanProtocol::SAFETY_ARRIVED - 1].reset();
  command_reply_identity_.reset();
  ports_.dropReply(CanProtocol::COMMAND_RESULT_ID);
  turn_.reset();
  ports_.dropReply(CanProtocol::TURN_REQUEST_ID);
}

void BridgeTransactions::resolvePendingModeReply(Clock::time_point current)
{
  if (!pending_mode_reply_) {return;}
  const auto pending = *pending_mode_reply_;
  if (modeSwitchFailed(pending.command, pending.requested_generation) || current >= pending.deadline) {
    pending_mode_reply_.reset();
    ports_.setRequestError("recognition switch was not confirmed");
    armCommandReply(false, pending.command, pending.token);
    return;
  }
  if (!switchConfirmed(pending.command)) {return;}
  pending_mode_reply_.reset();
  if (const auto & mission = state_.missionState(); mission && pending.command == CanProtocol::TURN_COMPLETED) {
    accepted_commands_[pending.command - 1] = CommandIdentity{
      pending.command, mission->cycle_generation, mission->roi_request_id,
      mission->destination, mission->cycle_phase, false};
  }
  armCommandReply(true, pending.command, pending.token);
}

void BridgeTransactions::refreshTurnRequest(Clock::time_point current, bool force)
{
  if (!turnAuthorized()) {
    ports_.dropReply(CanProtocol::TURN_REQUEST_ID);
    turn_.reset();
    return;
  }
  if (!turnRequestCurrent()) {
    ports_.dropReply(CanProtocol::TURN_REQUEST_ID);
    const auto & mission = *state_.missionState();
    turn_ = TurnIdentity{mission.roi_request_id, mission.destination, mission.cycle_generation};
    next_turn_ = current;
  }
  if (!ports_.socketOpen() || (!force && current < next_turn_)) {return;}
  // 固定协议节拍，不是算法参数。发送机会仍守原 TTL，不能通过重试续队首寿命。
  armReply(CanProtocol::TURN_REQUEST_ID,
    CanProtocol::turnRequest(static_cast<std::uint8_t>(turn_->destination),
    static_cast<std::uint32_t>(turn_->roi_request_id)));
  next_turn_ = current + CanProtocol::TURN_RETRY_INTERVAL;
}

// ROI 终态同时更新业务结果、授权身份与编码快照；断线时只保存，重连后由查询重发。
// 零编号可保留协议结果位，但不能建立转向身份；转向还须匹配 Mission 已提交状态。
void BridgeTransactions::finishRoi(BridgeRoiResult result, std::uint64_t request_id)
{
  finishRoute(result.valid, result.passed, result.destination, request_id);
}

void BridgeTransactions::finishPickupRoute(bool accepted, BridgeDestination destination, std::uint64_t route_id)
{
  const auto & mission = state_.missionState();
  const bool identity_matches = pickup_ && pickup_->grasp_token == route_id &&
    mission && mission->cycle_generation == pickup_->cycle_generation &&
    (destination == BridgeDestination::Left || destination == BridgeDestination::Right) &&
    ((mission->cycle_phase == BridgeCyclePhase::SearchingTarget && mission->roi_request_id == 0) ||
     (mission->cycle_phase == BridgeCyclePhase::WaitingTurn && mission->roi_request_id == route_id &&
      mission->destination == destination));
  finishRoute(!accepted || identity_matches, accepted && identity_matches, destination, route_id);
}

// 两条登记路径共用原类型 6 结果帧和类型 8 转向授权，分别保留服务语义。
void BridgeTransactions::finishRoute(bool valid, bool accepted, BridgeDestination destination, std::uint64_t route_id)
{
  roi_passed_ = accepted;
  roi_confirmation_.reset();
  if (!route_aborted_ && accepted && route_id != 0) {
    roi_confirmation_ = TurnIdentity{route_id, destination, roi_cycle_generation_};
  }
  const auto frame = CanProtocol::roiResult(
    valid, accepted, static_cast<std::uint8_t>(destination));
  if (!accepted && route_service_ == Service::Roi) {arrival_point_.authorizeRejectOpen();}
  armReply(CanProtocol::ROI_RESULT_ID, frame);
}

// 回复槽持有载荷，队列持有发送机会；同 ID 的新回复会覆盖快照，因此这里不是逐条可靠交付队列。
void BridgeTransactions::armReply(canid_t id, const can_frame & frame)
{
  if (id == CanProtocol::COMMAND_RESULT_ID) {
    // 发送身份随192载荷一起覆盖，不能借用后来另一条成功缓存的资格。
    const auto command = frame.data[1];
    command_reply_identity_ = frame.data[0] && command != CanProtocol::START_SEARCH &&
      command >= CanProtocol::TURN_COMPLETED && command <= CanProtocol::DELIVERY_COMPLETED ?
      accepted_commands_[command - 1] : std::nullopt;
  }
  // 替换回复快照但不刷新同 ID 的最早入队时间，仍受原队列 TTL 限制。
  replies_.store(id, frame);
  ports_.queueReply(id);
}

bool BridgeTransactions::replyCurrent(canid_t id) const
{
  if (id == CanProtocol::TURN_REQUEST_ID) {return turnRequestCurrent();}
  if (id == CanProtocol::COMMAND_RESULT_ID) {
    const auto & reply = replies_.frame(id);
    if (reply.data[0] && reply.data[1] == CanProtocol::START_ALIGN) {
      return switchConfirmed(CanProtocol::START_ALIGN);
    }
  }
  if (id == CanProtocol::COMMAND_RESULT_ID && command_reply_identity_) {
    return commandIdentityCurrent(*command_reply_identity_);
  }
  return true;
}

// expire() 先使等待失效，再移除 rclcpp 请求并保存失败快照。
// 与响应回调使用同一截止边界，迟到回包不能再次完成该请求。
void BridgeTransactions::checkRequestTimeouts(Clock::time_point current)
{
  if (const auto timeout = roi_request_.expire({current})) {
    ports_.cancelService(route_service_, timeout->request_id);
    const char * error = "arrival route service request timed out";
    ports_.setRequestError(error);
    failRoute();
    ports_.warnRequest(error);
  }
  if (const auto timeout = command_request_.expire({current})) {
    pending_command_identity_.reset();
    ports_.cancelService(Service::Command, timeout->request_id);
    const char * error = "mission command service request timed out";
    ports_.setRequestError(error);
    armCommandReply(false, timeout->command, pending_command_token_);
    ports_.warnRequest(error);
  }
  resolvePendingModeReply(current);
}

// CAN 断线后旧代次不能完成新等待；任务状态和到达保持仍由 BridgeState 保留。
void BridgeTransactions::disconnect()
{
  pickup_.reset();
  if (const auto cancelled = roi_request_.cancel()) {
    ports_.cancelService(route_service_, cancelled->request_id);
    // 断线只取消本机等待，Mission仍可能迟到提交。保存失败供重连后的200查询，
    // 不给已关闭socket入队，也不能把后来的WaitingTurn当作本机ROI通过。
    roi_attempted_ = true;
    failRoute();
  }
  if (const auto cancelled = command_request_.cancel()) {
    pending_command_identity_.reset();
    ports_.cancelService(Service::Command, cancelled->request_id);
  }
  pending_mode_reply_.reset();
  if (search_request_.busy()) {search_request_.complete(search_request_.requestId(), false);}
  if (initial_align_request_.busy()) {initial_align_request_.complete(initial_align_request_.requestId(), false);}
}

void BridgeTransactions::missionRestarted()
{
  pickup_.reset();
  // 不调用 finishRoi：旧请求失效不能在新 Mission 会话生成拒绝开爪授权。
  if (const auto cancelled = roi_request_.cancel()) {
    ports_.cancelService(route_service_, cancelled->request_id);
  }
  if (const auto cancelled = command_request_.cancel()) {
    ports_.cancelService(Service::Command, cancelled->request_id);
  }
  pending_mode_reply_.reset();
  pending_command_identity_.reset();
  pending_command_token_ = 0;
  search_request_.invalidate();
  initial_align_request_.invalidate();
  accepted_commands_.fill(std::nullopt);
  last_delivery_request_.reset();
  command_reply_identity_.reset();
  roi_confirmation_.reset();
  roi_passed_.reset();
  roi_attempted_ = false;
  roi_cycle_generation_ = 0;
  turn_.reset();
  route_aborted_ = true;
  arrival_point_.beginSearch();
}

void BridgeTransactions::armGripperResult(const GripperResult & result)
{
  armReply(CanProtocol::GRIPPER_RESULT_ID,
    CanProtocol::gripperResult(result.kind, result.success, result.request_id));
}
