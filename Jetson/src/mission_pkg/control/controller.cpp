#include "control/controller.hpp"

// 阶段迁移。ROI 应答与诊断成功才建路由；206 才记录送达；205 双确认才恢复寻找。
// Copyright 2026 li
// SPDX-License-Identifier: Apache-2.0
#include <interface_pkg/app/core.hpp>
#include <cmath>
#include <stdexcept>
#include <utility>

using CyclePhase = MissionState::CyclePhase;

namespace
{
void finishCallback(
  MissionController & controller, PendingOperation::Kind kind, std::uint64_t generation) noexcept
{
  if (kind == PendingOperation::Kind::RoiCheck) controller.roiCallbackFinished(generation);
  else if (kind == PendingOperation::Kind::AutomaticSwitch) {
    controller.switchCallbackFinished(kind, generation);
  }
}

// 这一批发布要么整段成功，要么按代次补救一次。补救再抛出时仍清令牌，避免旧回调改写新等待。
template<class Work>
void guardPublish(
  MissionController & controller, PendingOperation::Kind kind,
  std::uint64_t generation, Work work)
{
  try {
    work();
    finishCallback(controller, kind, generation);
  } catch (const std::exception & error) {
    try {
      controller.requestFailed(kind, generation, error.what());
    } catch (...) {
      finishCallback(controller, kind, generation);
      throw;
    }
    finishCallback(controller, kind, generation);
    throw;
  } catch (...) {
    finishCallback(controller, kind, generation);
    throw;
  }
}
}  // namespace

MissionController::MissionController(MissionOut & out, double request_timeout_s, TransportPolicy policy)
: out_(out), policy_(policy), timeout_s_(request_timeout_s)
{
  if (!std::isfinite(timeout_s_) || timeout_s_ <= 0 || timeout_s_ > 3600) {
    throw std::invalid_argument("request_timeout_s must be in (0, 3600]");
  }
}

PendingOperation & MissionController::pending(Kind kind)
{
  return kind == Kind::RoiCheck ? roi_check_ : automatic_switch_;
}

const PendingOperation & MissionController::pending(Kind kind) const
{
  return kind == Kind::RoiCheck ? roi_check_ : automatic_switch_;
}

bool MissionController::busy() const
{
  return roi_check_.active() || automatic_switch_.active();
}

bool MissionController::accepts(Kind kind, std::uint64_t generation) const
{
  return pending(kind).accepts(kind, generation);
}

void MissionController::registerRequest(
  Kind kind, std::uint64_t generation, std::int64_t request_id)
{
  if (accepts(kind, generation)) pending(kind).request_id = request_id;
}

bool MissionController::beginSwitchWait(Kind kind, std::uint64_t generation, TimePoint now)
{
  if (busy() ||
    state_.operation != MissionState::Operation::Switching ||
    generation != pending(kind).generation + 1) {return false;}
  pending(kind).begin(kind, now, timeout_s_);
  return true;
}

bool MissionController::beginRoiWait(std::uint64_t generation, TimePoint now)
{
  if (busy() || state_.operation != MissionState::Operation::Checking ||
    generation != roi_check_.generation + 1) {return false;}
  roi_check_.begin(Kind::RoiCheck, now, timeout_s_);
  return true;
}

MissionCommandResult MissionController::requestCommand(
  MissionCommand command, std::uint64_t roi_request_id)
{
  MissionCommandResult result;
  // 先回答已经记录的事实：206 回执丢失时，Fault 或 205 等待不能永久挡住确认重试。
  // 不发布、不恢复路由、不消费等待，也不把旧回执当成本趟的新送达。
  if (command == MissionCommand::DeliveryCompleted && roi_request_id != 0 &&
    roi_request_id == delivered_roi_request_id_)
  {
    result.accepted = true;
    result.message = "delivery already recorded for this ROI";
    return result;
  }
  const bool matches_route = roi_request_id != 0 && roi_request_id == state_.roi_request_id;
  // 203/204 的 ROS 回执可能丢失，而本趟已推进；仅在对应后续阶段只读确认。
  const bool recorded_turn = command == MissionCommand::TurnCompleted &&
    (state_.cycle_phase == CyclePhase::SwitchingField ||
    state_.cycle_phase == CyclePhase::NavigatingSafety);
  const bool recorded_arrival = command == MissionCommand::SafetyArrived &&
    state_.cycle_phase == CyclePhase::WaitingTransfer;
  if (matches_route && state_.destination != MissionState::Destination::None &&
    (recorded_turn || recorded_arrival))
  {
    result.accepted = true;
    result.message = "command already accepted for current ROI and phase";
    return result;
  }
  // 活动等待、待发布切换、待提交 ROI 均占用流程；拒绝新命令不刷新原截止。
  // 尚未 begin 且没有候选的孤立 CheckingRoi 不占用，仍可由 205 恢复。
  if (busy() || automatic_.status_publication_pending || roi_.pending_commit) {
    result.message = "another mission operation is pending";
    return result;
  }
  // 三种物理完成事件均核对完整业务编号，不能让上一趟迟到的 ROS 请求推进同阶段的新趟。
  // StartSearch 独立于路由身份，仍可从无有效路由的 Fault 恢复。
  switch (command) {
    case MissionCommand::TurnCompleted:
      if (!matches_route || state_.cycle_phase != CyclePhase::WaitingTurn) break;
      result.acted = true;
      startAutomatic(true);
      result.accepted = true;
      result.message = "turn completed; requesting field recognition";
      return result;
    case MissionCommand::SafetyArrived:
      if (!matches_route || state_.cycle_phase != CyclePhase::NavigatingSafety) break;
      state_.cycle_phase = CyclePhase::WaitingTransfer;
      state_.operation = MissionState::Operation::Idle;
      state_.outcome = MissionState::Outcome::Success;
      state_.reason = "safety zone reached; waiting for confirmed delivery or explicit abort";
      result.acted = true;
      out_.publishStatus(state_);
      result.accepted = true;
      result.message = state_.reason;
      return result;
    case MissionCommand::DeliveryCompleted:
      if (!matches_route || state_.cycle_phase != CyclePhase::WaitingTransfer ||
        state_.destination == MissionState::Destination::None) break;
      // 唯一首趟放行点：货物确认留在正确分区之后。ROI 和到达都不替代这一步。
      // 先记账再发布；发布失败不回滚已记下的送达。
      delivered_roi_request_id_ = roi_request_id;
      state_.transport_phase = MissionState::Transport::Later;
      state_.reason = "delivery confirmed for current ROI; waiting for start-search command";
      result.acted = true;
      out_.publishStatus(state_);
      result.accepted = true;
      result.message = state_.reason;
      return result;
    case MissionCommand::StartSearch:
      // 调用端须已完成返程，或停车并安全释放；Mission 不核实机械动作。
      // 205 无趟次身份。只弃路由，不确认或回滚送达。
      if (state_.cycle_phase != CyclePhase::WaitingTransfer &&
        state_.cycle_phase != CyclePhase::NavigatingSafety &&
        state_.cycle_phase != CyclePhase::WaitingTurn &&
        state_.cycle_phase != CyclePhase::CheckingRoi &&
        state_.cycle_phase != CyclePhase::RoiFailed &&
        state_.cycle_phase != CyclePhase::SearchingTarget &&
        state_.cycle_phase != CyclePhase::Fault) break;
      result.acted = true;
      startAutomatic(false);
      result.accepted = true;
      result.message = "start-search accepted; requesting seg recognition";
      return result;
  }
  result.message = "command is not allowed for the current transport phase or ROI identity";
  return result;
}

void MissionController::requestRoi()
{
  const bool roi_ready = out_.roiServiceReady();
  const bool seg_active = task_ && task_->task == "seg";
  const bool eligible = state_.cycle_phase == CyclePhase::SearchingTarget;
  const bool waiting = busy();
  // 拒绝理由按“已有等待 → 阶段不符 → 非实际 seg → 服务未就绪”取第一项。
  // 仅空闲且仍在寻找时，服务未就绪会额外进入 RoiFailed。
  if (waiting || !eligible || !seg_active || !roi_ready) {
    RoiRuleResult result;
    result.reason = waiting ? "another mission operation is pending" :
      (!eligible ? "target arrival is not allowed in the current transport cycle phase" :
      (!seg_active ? "YOLO recognition phase is not active" : "target ROI service unavailable"));
    out_.publishRoi(result, 0, false);
    if (!waiting && eligible && !roi_ready) {
      state_.cycle_phase = CyclePhase::RoiFailed;
      state_.destination = MissionState::Destination::None;
      state_.operation = MissionState::Operation::Idle;
      state_.outcome = MissionState::Outcome::RoiInvalid;
      state_.reason = result.reason;
      out_.publishStatus(state_);
    }
    return;
  }
  roi_.callback_generation.reset();
  // 先撤旧路由再发布 Checking。发布失败时等待还没 begin，到达收尾可以退回寻找。
  state_.destination = MissionState::Destination::None;
  state_.cycle_phase = CyclePhase::CheckingRoi;
  state_.operation = MissionState::Operation::Checking;
  state_.outcome = MissionState::Outcome::Pending;
  state_.roi_request_id = 0;
  state_.reason = "arrival ROI check requested; target coordinates held";
  out_.publishStatus(state_);
  out_.sendRoiRequest(roi_check_.generation + 1);
}

void MissionController::armRoi(
  const RoiRuleResult & result, std::uint64_t request_id,
  std::optional<MissionState::Outcome> failure)
{
  const auto generation = roi_check_.generation;
  roi_check_.clear();
  const auto decision = applyTransportRules(result, state_.transport_phase, policy_);
  roi_.pending_commit = RoiCommit{decision, request_id, generation, failure};
}

void MissionController::publishArmedRoi(bool use_source_stamp)
{
  const auto pending = *roi_.pending_commit;
  publish_commits_roi_ = true;
  try {
    out_.publishRoi(pending.decision.result, pending.request_id, use_source_stamp);
  } catch (...) {
    publish_commits_roi_ = false;
    throw;
  }
  publish_commits_roi_ = false;
}

void MissionController::commitRoiAfterReply()
{
  if (!roi_.pending_commit) return;
  const auto completion = std::move(*roi_.pending_commit);
  roi_.pending_commit.reset();
  roi_.callback_generation = completion.generation;
  applyRoiDecision(completion);
  if (completion.failure_outcome) {
    // 保留两份状态：第一份是规则结论，第二份把 ROI_INVALID 改成 TIMEOUT 等失败结果。
    state_.outcome = *completion.failure_outcome;
    out_.publishStatus(state_);
  }
  resumeDeferredTask();
}

void MissionController::applyRoiDecision(const RoiCommit & completion)
{
  const auto & decision = completion.decision;
  // 路由准入以 accepted 为准：输入计数矛盾时，诊断中的 passed 仍可能保留为真。
  state_.roi_request_id = completion.request_id;
  state_.operation = MissionState::Operation::Idle;
  if (!decision.accepted) {
    state_.cycle_phase = CyclePhase::RoiFailed;
    state_.destination = MissionState::Destination::None;
    state_.outcome = !decision.result.valid ?
      MissionState::Outcome::RoiInvalid : MissionState::Outcome::RoiRejected;
    state_.reason = decision.result.reason;
  } else {
    state_.destination = decision.destination;
    state_.cycle_phase = CyclePhase::WaitingTurn;
    state_.outcome = MissionState::Outcome::Success;
    state_.reason = "ROI accepted; waiting for STM32 turn completion";
  }
  out_.publishStatus(state_);
}

void MissionController::resumeDeferredTask()
{
  if (!deferred_task_) return;
  const auto deferred = std::move(*deferred_task_);
  deferred_task_.reset();
  in_deferred_task_ = true;
  try {
    taskChanged(deferred.task, deferred.observed_at);
  } catch (...) {
    in_deferred_task_ = false;
    throw;
  }
  in_deferred_task_ = false;
}

void MissionController::roiResponseFailed(std::uint64_t generation)
{
  if (!roi_.pending_commit || roi_.pending_commit->generation != generation) return;
  roi_.pending_commit.reset();
  roi_.callback_generation.reset();
  deferred_task_.reset();
  // 请求头已经取走，不能重复应答；保持坐标关闭，必须由显式重找恢复。
  state_.cycle_phase = CyclePhase::RoiFailed;
  state_.operation = MissionState::Operation::Idle;
  state_.outcome = MissionState::Outcome::RoiInvalid;
  out_.publishStatus(state_);
}

void MissionController::roiCallbackFinished(std::uint64_t generation) noexcept
{
  if (roi_.callback_generation == generation) roi_.callback_generation.reset();
}

void MissionController::failRoi(const std::string & message, MissionState::Outcome outcome)
{
  RoiRuleResult result;
  result.reason = message;
  armRoi(result, 0, outcome);
  publishArmedRoi(false);
}

void MissionController::startAutomatic(bool field)
{
  ++state_.cycle_generation;
  automatic_.callback_generation.reset();
  automatic_.service_accepted = false;
  automatic_.task_observed = false;
  automatic_.expected_field = field;
  state_.cycle_phase = field ? CyclePhase::SwitchingField : CyclePhase::SwitchingSeg;
  state_.operation = MissionState::Operation::Switching;
  state_.outcome = MissionState::Outcome::Pending;
  state_.reason = field ? "turn completed; switching to field recognition" :
    "explicit start-search; switching to seg recognition";
  // 发布失败时这个标志还在，commandFailed 会清掉，不会留下已经发出的切换请求。
  automatic_.status_publication_pending = true;
  out_.publishStatus(state_);
  continueSwitch();
}

void MissionController::continueSwitch()
{
  if (!automatic_.status_publication_pending) return;
  automatic_.status_publication_pending = false;
  if (!out_.detectorReady()) {
    finishAutomatic(
      MissionState::Outcome::Rejected,
      "detector service unavailable for recognition switch");
    return;
  }
  out_.sendSwitchRequest(automatic_.expected_field, automatic_switch_.generation + 1);
}

void MissionController::finishAutomatic(
  MissionState::Outcome outcome, const std::string & message)
{
  if (automatic_switch_.active()) {
    automatic_.callback_generation = automatic_switch_.generation;
  }
  automatic_.service_accepted = false;
  automatic_.task_observed = false;
  automatic_.status_publication_pending = false;
  automatic_switch_.clear();
  // 失败撤路由，field 成功保留路由，seg 成功清路由；三者都保留已确认的送达。
  if (outcome != MissionState::Outcome::Success) {
    state_.destination = MissionState::Destination::None;
    state_.cycle_phase = CyclePhase::Fault;
  } else if (automatic_.expected_field) {
    state_.cycle_phase = CyclePhase::NavigatingSafety;
  } else {
    state_.cycle_phase = CyclePhase::SearchingTarget;
    state_.destination = MissionState::Destination::None;
    state_.roi_request_id = 0;
  }
  state_.operation = MissionState::Operation::Idle;
  state_.outcome = outcome;
  state_.reason = message;
  out_.publishStatus(state_);
}

MissionController::TaskEvent MissionController::classifyTaskEvent(
  const std::optional<MissionTaskState> & current, const MissionTaskState & next)
{
  if (current && next.source_time.nanoseconds < current->source_time.nanoseconds) {
    return TaskEvent::Older;
  }
  if (!current) return TaskEvent::NewSession;
  if (next.task == current->task && next.epoch == current->epoch &&
    next.source_time.nanoseconds == current->source_time.nanoseconds)
  {
    return TaskEvent::Duplicate;
  }
  return TaskEvent::NewSession;
}

void MissionController::taskChanged(const MissionTaskState & task, TimePoint now)
{
  if (in_deferred_task_) {
    applyTaskChange(task, now);
    return;
  }
  const auto event = classifyTaskEvent(task_, task);
  if (event == TaskEvent::Older) return;
  const bool changed = event != TaskEvent::Duplicate;
  // 收尾标记要在发布前就定下来。取消未来抛出时，后面的应答代次也要能补救。
  if (roi_check_.active() && changed) {
    const auto generation = roi_check_.generation;
    guardPublish(*this, Kind::RoiCheck, generation, [&] {applyTaskChange(task, now);});
    return;
  }
  const bool switching = automatic_switch_.active();
  const bool expected = switching &&
    task.task == (automatic_.expected_field ? "field" : "seg");
  if (expected && (automatic_switch_.expired(now) || automatic_.service_accepted)) {
    const auto generation = automatic_switch_.generation;
    guardPublish(*this, Kind::AutomaticSwitch, generation, [&] {applyTaskChange(task, now);});
    return;
  }
  applyTaskChange(task, now);
}

void MissionController::applyTaskChange(const MissionTaskState & task, TimePoint now)
{
  const auto event = classifyTaskEvent(task_, task);
  if (event == TaskEvent::Older) return;
  const bool first_task = !task_;
  const bool changed = event != TaskEvent::Duplicate;
  if (roi_check_.active() && changed) {
    cancelRoiForTaskChange(task, now);
    return;
  }
  const bool switching = automatic_switch_.active();
  const bool expected = switching &&
    task.task == (automatic_.expected_field ? "field" : "seg");
  task_ = task;
  // 完全重复仍转发，但不取消等待或追加状态。
  out_.forwardTask();
  bool observation_published = false;
  if (expected) {
    // 实际模式先到时只记确认，状态仍由下面这一次发布。完成时 finishAutomatic 自己会发布。
    observation_published = automatic_switch_.expired(now) || automatic_.service_accepted;
    observeExpectedTask(now);
  } else if (first_task && task.task == "seg" && !switching && state_.cycle_generation == 0) {
    state_.cycle_phase = CyclePhase::SearchingTarget;
    state_.operation = MissionState::Operation::Idle;
    state_.reason = "initial seg recognition confirmed";
  } else if (changed) {
    // 未经握手的模式变化不能靠实际 field 状态自动放行坐标。
    enterFault("unexpected recognition session change; explicit start-search required");
  }
  if (changed && !observation_published) out_.publishStatus(state_);
}

void MissionController::cancelRoiForTaskChange(
  const MissionTaskState & task, TimePoint observed_at)
{
  const auto request_id = roi_check_.request_id;
  RoiRuleResult result;
  result.reason = "recognition phase changed during ROI check";
  armRoi(result, 0, MissionState::Outcome::RoiInvalid);
  // 先记下阶段事件，应答提交成功后再套用，避免末尾状态早于 ROI 收尾。
  deferred_task_ = DeferredTask{task, observed_at};
  out_.cancel(Kind::RoiCheck, request_id);
  publishArmedRoi(false);
}

void MissionController::enterFault(const std::string & message)
{
  state_.destination = MissionState::Destination::None;
  if (automatic_switch_.active()) {
    out_.cancel(Kind::AutomaticSwitch, automatic_switch_.request_id);
    automatic_switch_.clear();
  }
  automatic_.status_publication_pending = false;
  automatic_.service_accepted = false;
  automatic_.task_observed = false;
  state_.cycle_phase = CyclePhase::Fault;
  state_.operation = MissionState::Operation::Idle;
  state_.outcome = MissionState::Outcome::Rejected;
  state_.reason = message;
}

void MissionController::commandFailed(const std::string & message)
{
  enterFault(message);
  out_.publishStatus(state_);
}

void MissionController::arrivalInterrupted(const std::string & message)
{
  if (busy() || automatic_.status_publication_pending || roi_.pending_commit) return;
  // 阶段已改成检查中，但 PENDING 没发布成功，等待也还没 begin。外界仍看见寻找。
  if (state_.cycle_phase == CyclePhase::CheckingRoi &&
    state_.operation == MissionState::Operation::Checking &&
    state_.outcome == MissionState::Outcome::Pending)
  {
    state_.cycle_phase = CyclePhase::SearchingTarget;
    state_.operation = MissionState::Operation::Idle;
    state_.outcome = MissionState::Outcome::None;
    state_.reason = message;
    out_.publishStatus(state_);
    return;
  }
  if (state_.cycle_phase == CyclePhase::RoiFailed || state_.cycle_phase == CyclePhase::Fault) {
    out_.publishStatus(state_);
  }
}

void MissionController::observeExpectedTask(TimePoint now)
{
  if (automatic_switch_.expired(now)) {
    out_.cancel(Kind::AutomaticSwitch, automatic_switch_.request_id);
    timeout(Kind::AutomaticSwitch);
  } else if (automatic_.service_accepted) {
    finishAutomatic(
      MissionState::Outcome::Success,
      automatic_.expected_field ? "field recognition confirmed after turn" :
      "seg recognition confirmed; searching for target");
  } else {
    automatic_.task_observed = true;
  }
}

void MissionController::timeout(Kind kind)
{
  if (kind == Kind::AutomaticSwitch) {
    finishAutomatic(
      MissionState::Outcome::Timeout,
      "recognition switch timed out; actual state may still change");
  } else if (kind == Kind::RoiCheck) {
    failRoi("target ROI service response timed out", MissionState::Outcome::Timeout);
  }
}

bool MissionController::replyGate(Kind kind, std::uint64_t generation, TimePoint now)
{
  if (!accepts(kind, generation)) return false;
  if (!pending(kind).expired(now)) return true;
  guardPublish(*this, kind, generation, [&] {timeout(kind);});
  return false;
}

void MissionController::switchReply(
  Kind kind, std::uint64_t generation, bool success, const std::string & message)
{
  if (!accepts(kind, generation)) return;
  // finishAutomatic 会作废等待代次，但本回调内的发布异常仍须尝试失败收尾。
  automatic_.callback_generation = generation;
  if (!success) {
    finishAutomatic(MissionState::Outcome::Rejected, message);
    return;
  }
  automatic_.service_accepted = true;
  if (automatic_.task_observed) {
    finishAutomatic(
      MissionState::Outcome::Success,
      automatic_.expected_field ? "field recognition confirmed after turn" :
      "seg recognition confirmed; searching for target");
  }
}

void MissionController::switchCallbackFinished(Kind kind, std::uint64_t generation) noexcept
{
  if (kind == Kind::AutomaticSwitch && automatic_.callback_generation == generation) {
    automatic_.callback_generation.reset();
  }
}

void MissionController::roiReply(
  std::uint64_t generation, const RoiRuleResult & result, std::uint64_t request_id)
{
  if (!accepts(Kind::RoiCheck, generation)) return;
  armRoi(result, request_id, std::nullopt);
  publishArmedRoi(true);
}

void MissionController::requestFailed(
  Kind kind, std::uint64_t generation, const std::string & message)
{
  if (kind == Kind::RoiCheck && roi_.pending_commit &&
    roi_.pending_commit->generation == generation)
  {
    roiResponseFailed(generation);
    return;
  }
  if (kind == Kind::RoiCheck && roi_.callback_generation == generation) {
    recoverCommittedRoi();
    return;
  }
  if (kind == Kind::AutomaticSwitch && automatic_.callback_generation == generation) {
    finishAutomatic(MissionState::Outcome::Rejected, message);
    return;
  }
  failActiveRequest(kind, generation, message);
}

void MissionController::recoverRequest(
  Kind kind, std::uint64_t generation, const std::string & message)
{
  guardPublish(*this, kind, generation, [&] {requestFailed(kind, generation, message);});
}

void MissionController::recoverCommittedRoi()
{
  roi_.callback_generation.reset();
  automatic_.status_publication_pending = false;
  state_.cycle_phase = CyclePhase::Fault;
  state_.outcome = MissionState::Outcome::RoiInvalid;
  out_.publishStatus(state_);
}

void MissionController::failActiveRequest(
  Kind kind, std::uint64_t generation, const std::string & message)
{
  if (!accepts(kind, generation)) return;
  if (kind == Kind::RoiCheck) failRoi(message);
  else if (kind == Kind::AutomaticSwitch) {
    finishAutomatic(MissionState::Outcome::Rejected, message);
  }
}

void MissionController::checkTimeouts(TimePoint now)
{
  for (const auto kind : {Kind::AutomaticSwitch, Kind::RoiCheck}) {
    if (!pending(kind).expired(now)) continue;
    const auto generation = pending(kind).generation;
    const auto request_id = pending(kind).request_id;
    // 先装好 ROI 候选再撤 future。撤失败时候选已经在，补救走“应答没发出去”，不会再答一次。
    guardPublish(*this, kind, generation, [&] {
      if (kind == Kind::RoiCheck) {
        RoiRuleResult result;
        result.reason = "target ROI service response timed out";
        armRoi(result, 0, MissionState::Outcome::Timeout);
      }
      out_.cancel(kind, request_id);
      if (kind == Kind::RoiCheck) publishArmedRoi(false);
      else timeout(kind);
    });
  }
}

MissionTransportDecision applyTransportRules(
  const RoiRuleResult & input, MissionState::Transport transport, TransportPolicy policy)
{
  MissionTransportDecision decision;
  decision.result = input;
  if (!input.valid || !input.passed) return decision;

  const cargo::Counts counts{input.green_count, input.blue_count, input.black_count,
    input.orange_count, input.other_count};
  const bool first_trip = transport == MissionState::Transport::First;
  if (first_trip) {
    const bool green_only = input.green_count > 0 &&
      input.black_count == 0 && input.orange_count == 0;
    if (!green_only) {
      decision.result.passed = false;
      decision.result.reason = "first transport accepts only a green target";
      return decision;
    }
    if (policy.first_green_count != 0 && input.green_count != policy.first_green_count) {
      decision.result.passed = false;
      decision.result.reason = "first transport requires the configured green count";
      return decision;
    }
  }
  if (!cargo::withinSupplyLimit(counts, policy.max_supply_count)) {
    decision.result.passed = false;
    decision.result.reason = "green and black exceed the transport limit";
    return decision;
  }
  decision.accepted = true;
  decision.destination = static_cast<MissionState::Destination>(
    RecognitionValueValidation::cargoSide(input.orange_count));
  return decision;
}
