#include "check.hpp"
#include "control/controller.hpp"

#include <chrono>
#include <string>

namespace {

using Phase = MissionState::CyclePhase;

// 桩不发送 ROS。requestRoi 只记下代次，测试再 begin，和节点里“发布成功后才 begin”一致。
struct HostOut : MissionOut
{
  int status_count{0};
  int roi_sends{0};
  std::uint64_t roi_generation{0};
  int switch_sends{0};
  std::uint64_t switch_generation{0};
  bool roi_ready{true};
  bool detector_ready{true};
  RoiRuleResult published_roi;
  std::uint64_t published_roi_id{0};

  void publishStatus(const MissionState &) override {++status_count;}
  void publishRoi(const RoiRuleResult & result, std::uint64_t request_id, bool) override
  {
    published_roi = result;
    published_roi_id = request_id;
  }
  void forwardTask() override {}
  bool roiServiceReady() override {return roi_ready;}
  void sendRoiRequest(std::uint64_t generation) override
  {
    ++roi_sends;
    roi_generation = generation;
  }
  bool detectorReady() override {return detector_ready;}
  void sendSwitchRequest(bool, std::uint64_t generation) override
  {
    ++switch_sends;
    switch_generation = generation;
  }
  void cancel(PendingOperation::Kind, std::int64_t) override {}
};

struct Session
{
  HostOut out;
  MissionController controller;
  Session() : controller(out, 2.0, TransportPolicy{1, 3}) {}
};

MissionTaskState taskAt(const char * name, std::uint64_t epoch, std::int64_t stamp_ns)
{
  MissionTaskState task;
  task.task = name;
  task.epoch = epoch;
  task.source_time.nanoseconds = stamp_ns;
  return task;
}

RoiRuleResult oneGreen()
{
  RoiRuleResult result;
  result.valid = true;
  result.passed = true;
  result.green_count = 1;
  result.reason = "ROI accepted";
  return result;
}

const auto kNow = MissionController::TimePoint{std::chrono::seconds(1000)};

bool waitForRoi(Session & session, std::uint64_t & generation)
{
  const int sends_before = session.out.roi_sends;
  session.controller.requestRoi();
  CHECK(session.out.roi_sends == sends_before + 1);
  generation = session.out.roi_generation;
  const bool started = session.controller.beginRoiWait(generation, kNow);
  CHECK(started);
  return started;
}

bool confirmSwitch(
  Session & session, const char * task, std::int64_t stamp_ns, std::uint64_t epoch)
{
  CHECK(session.out.switch_sends > 0);
  const auto generation = session.out.switch_generation;
  const bool started = session.controller.beginSwitchWait(
    PendingOperation::Kind::AutomaticSwitch, generation, kNow);
  CHECK(started);
  if (!started) return false;
  session.controller.switchReply(
    PendingOperation::Kind::AutomaticSwitch, generation, true, "ok");
  session.controller.taskChanged(taskAt(task, epoch, stamp_ns), kNow);
  return true;
}

}  // namespace

void testMissionTrajectory()
{
  Group group("MissionController.trajectory");
  Session session;
  CHECK(session.controller.state().cycle_phase == Phase::Fault);

  session.controller.taskChanged(taskAt("seg", 1, 100), kNow);
  CHECK(session.controller.state().cycle_phase == Phase::SearchingTarget);
  CHECK(session.controller.state().cycle_generation == 0);

  std::uint64_t generation = 0;
  CHECK(waitForRoi(session, generation));
  CHECK(session.controller.state().cycle_phase == Phase::CheckingRoi);
  CHECK(session.controller.busy());
  session.controller.roiReply(generation, oneGreen(), 42);
  // 桩的 publishRoi 不表示应答已发出，提交仍由调用方在应答成功后做。
  session.controller.commitRoiAfterReply();
  CHECK(session.controller.state().cycle_phase == Phase::WaitingTurn);
  CHECK(session.controller.state().destination == MissionState::Destination::Left);
  CHECK(session.controller.state().roi_request_id == 42);
  CHECK(session.controller.state().outcome == MissionState::Outcome::Success);
  CHECK(!session.controller.busy());

  const auto turned = session.controller.requestCommand(MissionCommand::TurnCompleted, 42);
  CHECK(turned.accepted && session.controller.state().cycle_phase == Phase::SwitchingField);
  CHECK(session.controller.state().cycle_generation == 1);
  CHECK(confirmSwitch(session, "field", 200, 1));
  CHECK(session.controller.state().cycle_phase == Phase::NavigatingSafety);
  CHECK(session.controller.state().destination == MissionState::Destination::Left);

  const auto arrived = session.controller.requestCommand(MissionCommand::SafetyArrived, 42);
  CHECK(arrived.accepted && session.controller.state().cycle_phase == Phase::WaitingTransfer);

  const auto delivered = session.controller.requestCommand(MissionCommand::DeliveryCompleted, 42);
  CHECK(delivered.accepted);
  CHECK(session.controller.state().transport_phase == MissionState::Transport::Later);
  CHECK(session.controller.state().cycle_phase == Phase::WaitingTransfer);
  // 同一编号的再次确认只读回执，不发布、不推进阶段。
  const int statuses = session.out.status_count;
  const auto replay = session.controller.requestCommand(MissionCommand::DeliveryCompleted, 42);
  CHECK(replay.accepted && session.out.status_count == statuses);
  CHECK(session.controller.state().cycle_phase == Phase::WaitingTransfer);

  const auto search = session.controller.requestCommand(MissionCommand::StartSearch, 0);
  CHECK(search.accepted && session.controller.state().cycle_phase == Phase::SwitchingSeg);
  CHECK(confirmSwitch(session, "seg", 300, 2));
  CHECK(session.controller.state().cycle_phase == Phase::SearchingTarget);
  CHECK(session.controller.state().destination == MissionState::Destination::None);
  CHECK(session.controller.state().roi_request_id == 0);
  CHECK(session.controller.state().transport_phase == MissionState::Transport::Later);
  CHECK(session.controller.state().cycle_generation == 2);
}

void testMissionErrors()
{
  Group group("MissionController.errors");

  // 零编号和上一趟编号都不能推动当前路由。
  Session numbered;
  numbered.controller.taskChanged(taskAt("seg", 1, 100), kNow);
  std::uint64_t generation = 0;
  CHECK(waitForRoi(numbered, generation));
  numbered.controller.roiReply(generation, oneGreen(), 42);
  numbered.controller.commitRoiAfterReply();
  const auto zero = numbered.controller.requestCommand(MissionCommand::TurnCompleted, 0);
  const auto old = numbered.controller.requestCommand(MissionCommand::TurnCompleted, 41);
  CHECK(!zero.accepted && !old.accepted);
  CHECK(numbered.controller.state().cycle_phase == Phase::WaitingTurn);
  CHECK(numbered.controller.state().roi_request_id == 42);
  CHECK(numbered.controller.state().cycle_generation == 0);

  // 乱序：还在等转向时不能报到达或送达；故障态不能报转向。
  const auto early_arrival = numbered.controller.requestCommand(MissionCommand::SafetyArrived, 42);
  const auto early_delivery = numbered.controller.requestCommand(MissionCommand::DeliveryCompleted, 42);
  CHECK(!early_arrival.accepted && !early_delivery.accepted);
  CHECK(numbered.controller.state().cycle_phase == Phase::WaitingTurn);
  Session faulted;
  const auto fault_turn = faulted.controller.requestCommand(MissionCommand::TurnCompleted, 42);
  CHECK(!fault_turn.accepted && faulted.controller.state().cycle_phase == Phase::Fault);

  // 忙碌：ROI 等待已经 begin 之后，重找和再次到达都不改阶段。
  Session busy;
  busy.controller.taskChanged(taskAt("seg", 1, 100), kNow);
  CHECK(waitForRoi(busy, generation));
  const auto during = busy.controller.requestCommand(MissionCommand::StartSearch, 0);
  CHECK(!during.accepted);
  CHECK(during.message == "another mission operation is pending");
  busy.controller.requestRoi();
  CHECK(busy.controller.busy());
  CHECK(busy.controller.state().cycle_phase == Phase::CheckingRoi);
  CHECK(busy.controller.state().operation == MissionState::Operation::Checking);
}

void testMissionRoiIdentity()
{
  Group group("MissionController.roi_identity");
  Session session;
  session.controller.taskChanged(taskAt("seg", 1, 100), kNow);
  std::uint64_t generation = 0;
  CHECK(waitForRoi(session, generation));
  session.controller.roiReply(generation, oneGreen(), 1);
  session.controller.commitRoiAfterReply();
  CHECK(session.controller.requestCommand(MissionCommand::TurnCompleted, 1).accepted);
  CHECK(confirmSwitch(session, "field", 200, 2));
  CHECK(session.controller.requestCommand(MissionCommand::SafetyArrived, 1).accepted);
  CHECK(session.controller.requestCommand(MissionCommand::DeliveryCompleted, 1).accepted);
  CHECK(session.controller.requestCommand(MissionCommand::StartSearch, 0).accepted);
  CHECK(confirmSwitch(session, "seg", 300, 3));

  // Target 单独重启后从 1 开始；旧送达编号不能成为下一趟路由。
  CHECK(waitForRoi(session, generation));
  const auto cycle = session.controller.state().cycle_generation;
  session.controller.roiReply(generation, oneGreen(), 1);
  CHECK(!session.out.published_roi.valid && !session.out.published_roi.passed);
  CHECK(session.out.published_roi_id == 0);
  session.controller.commitRoiAfterReply();
  CHECK(session.controller.state().cycle_phase == Phase::RoiFailed);
  CHECK(session.controller.state().roi_request_id == 0);
  CHECK(session.controller.state().destination == MissionState::Destination::None);
  CHECK(session.controller.state().operation == MissionState::Operation::Idle);
  CHECK(session.controller.state().outcome == MissionState::Outcome::RoiInvalid);
  CHECK(session.controller.state().reason.find("reused or moved backwards") != std::string::npos);
  CHECK(session.controller.state().cycle_generation == cycle);
  CHECK(session.controller.state().transport_phase == MissionState::Transport::Later);
  CHECK(!session.controller.busy());

  const int statuses = session.out.status_count;
  CHECK(!session.controller.requestCommand(MissionCommand::TurnCompleted, 1).accepted);
  CHECK(!session.controller.requestCommand(MissionCommand::SafetyArrived, 1).accepted);
  // 旧送达收据仍可只读查询；不能因此恢复路由或发布状态。
  const auto receipt = session.controller.requestCommand(MissionCommand::DeliveryCompleted, 1);
  CHECK(receipt.accepted && !receipt.acted);
  CHECK(session.out.status_count == statuses);
  CHECK(session.controller.state().cycle_phase == Phase::RoiFailed);
  CHECK(session.controller.state().roi_request_id == 0);

  // 显式重找后严格递增编号仍按原协议完成下一趟。
  CHECK(session.controller.requestCommand(MissionCommand::StartSearch, 0).accepted);
  CHECK(confirmSwitch(session, "seg", 400, 4));
  CHECK(waitForRoi(session, generation));
  session.controller.roiReply(generation, oneGreen(), 2);
  session.controller.commitRoiAfterReply();
  CHECK(session.controller.state().cycle_phase == Phase::WaitingTurn);
  CHECK(session.controller.state().roi_request_id == 2);
  CHECK(session.controller.requestCommand(MissionCommand::TurnCompleted, 2).accepted);
  CHECK(confirmSwitch(session, "field", 500, 5));
  CHECK(session.controller.requestCommand(MissionCommand::SafetyArrived, 2).accepted);
  CHECK(session.controller.requestCommand(MissionCommand::DeliveryCompleted, 2).accepted);
  const auto second_receipt = session.controller.requestCommand(MissionCommand::DeliveryCompleted, 2);
  CHECK(second_receipt.accepted && !second_receipt.acted);

  // 有效代次的失败结论也消费编号；过期代次不得提高高水位。
  Session failed;
  failed.controller.taskChanged(taskAt("seg", 1, 100), kNow);
  CHECK(waitForRoi(failed, generation));
  RoiRuleResult invalid;
  invalid.reason = "frame unavailable";
  failed.controller.roiReply(generation, invalid, 10);
  failed.controller.commitRoiAfterReply();
  CHECK(failed.controller.state().cycle_phase == Phase::RoiFailed);
  CHECK(failed.controller.requestCommand(MissionCommand::StartSearch, 0).accepted);
  CHECK(confirmSwitch(failed, "seg", 200, 2));
  CHECK(waitForRoi(failed, generation));
  failed.controller.roiReply(generation, oneGreen(), 9);
  failed.controller.commitRoiAfterReply();
  CHECK(failed.controller.state().cycle_phase == Phase::RoiFailed);
  CHECK(failed.controller.state().roi_request_id == 0);
  CHECK(failed.controller.state().destination == MissionState::Destination::None);
  CHECK(failed.controller.state().transport_phase == MissionState::Transport::First);
  const auto stale_generation = generation;
  CHECK(failed.controller.requestCommand(MissionCommand::StartSearch, 0).accepted);
  CHECK(confirmSwitch(failed, "seg", 300, 3));
  CHECK(waitForRoi(failed, generation));
  const int before_stale = failed.out.status_count;
  failed.controller.roiReply(stale_generation, oneGreen(), 1000);
  failed.controller.commitRoiAfterReply();
  CHECK(failed.out.status_count == before_stale);
  CHECK(failed.controller.busy());
  failed.controller.roiReply(generation, oneGreen(), 11);
  failed.controller.commitRoiAfterReply();
  CHECK(failed.controller.state().cycle_phase == Phase::WaitingTurn);
  CHECK(failed.controller.state().roi_request_id == 11);
}
