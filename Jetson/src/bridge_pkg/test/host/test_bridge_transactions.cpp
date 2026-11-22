#include "check.hpp"
#include "state/bridge_transactions.hpp"

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <utility>
#include <vector>

namespace
{
using Clock = BridgeTransactions::Clock;
using Service = BridgeTransactions::Service;
using CommandRequest = BridgeTransactions::CommandRequest;
using namespace std::chrono_literals;

constexpr auto request_timeout = 100ms;
constexpr std::uint64_t roi_id = (std::uint64_t{1} << 32) + 77;
constexpr auto roi_token = static_cast<std::uint32_t>(roi_id);

struct Ports final : BridgeTransactions::Ports
{
  BridgeSourceTime source{1};
  Clock::time_point monotonic{};
  std::int64_t roi_requests{0};
  std::uint64_t roi_generation{0};
  std::int64_t pickup_requests{0};
  std::string pickup_class;
  std::uint32_t pickup_token{0};
  std::int64_t command_requests{0};
  CommandRequest last_command{};
  std::vector<canid_t> queued;
  std::vector<std::pair<Service, std::int64_t>> cancelled;

  BridgeSourceTime sourceNow() const override {return source;}
  Clock::time_point monotonicNow() const override {return monotonic;}
  bool serviceReady(Service) override {return true;}
  std::int64_t sendRoi(std::uint64_t generation) override
  {
    roi_generation = generation;
    return ++roi_requests;
  }
  std::int64_t sendCommand(const CommandRequest & request) override
  {
    last_command = request;
    return ++command_requests;
  }
  std::int64_t sendPickupRoute(std::uint64_t generation, const std::string & class_id, std::uint32_t token) override
  {
    roi_generation = generation;
    pickup_class = class_id;
    pickup_token = token;
    return ++pickup_requests;
  }
  void cancelService(Service service, std::int64_t request_id) override
  {cancelled.emplace_back(service, request_id);}
  bool socketOpen() const override {return true;}
  void queueReply(canid_t id) override {queued.push_back(id);}
  void dropReply(canid_t) override {}
  void dropRequest() override {}
  void setRequestError(std::string_view) override {}
  void warnRequest(const char *) override {}
};

struct Fixture
{
  Ports ports;
  BridgeState state;
  ArrivalPointSession arrival;
  BridgeTransactions transactions{state, arrival, ports, request_timeout};

  void mission(
    BridgeCyclePhase phase, std::uint64_t generation, std::uint64_t id = 0,
    BridgeDestination destination = BridgeDestination::None)
  {
    ++ports.source.ns;
    const auto update = state.mission(
      {ports.source, destination, BridgeOperation::Idle, BridgeOutcome::Success,
        BridgeTransportPhase::First, id, phase, generation}, ports.source);
    CHECK(update.accepted && !update.mission_restarted);
    transactions.onStateUpdate();
    transactions.refreshTurnRequest(ports.monotonic);
    transactions.resolvePendingModeReply(ports.monotonic);
  }

  bool replyIs(const can_frame & expected) const
  {
    const auto actual = transactions.replyFrame(expected.can_id);
    return actual.can_id == expected.can_id && actual.len == expected.len &&
           std::equal(expected.data, expected.data + expected.len, actual.data);
  }

  void task(const char * name)
  {
    ++ports.source.ns;
    CHECK(state.task({ports.source, name, 1}, ports.source).accepted);
    transactions.onStateUpdate();
    transactions.resolvePendingModeReply(ports.monotonic);
  }

  void acceptLatestCommand()
  {
    const auto request = ports.last_command;
    CHECK(transactions.acceptCommandReply(request.generation, ports.monotonic));
    transactions.commandReply(request.command, request.requested_cycle_generation, true, ports.monotonic);
  }

  void target(const char * class_id)
  {
    RecognitionDetection d;
    d.id = "selected";
    d.box = {960, 540, 20, 20, 0};
    d.hypothesis = RecognitionHypothesis{class_id, 0.9};
    ++ports.source.ns;
    CHECK(state.target({ports.source, "cam", d}, ports.source, {ports.monotonic}, true));
  }
};
}  // namespace

void testBridgeTransactions()
{
  Group group("BridgeTransactions arrival, ROI and command replies");

  Fixture pickup;
  pickup.arrival.complete(true);
  pickup.task("seg");
  pickup.mission(BridgeCyclePhase::SearchingTarget, 20);
  pickup.target("2"); // 首趟只可抓正式选出的绿色目标。
  CHECK(pickup.transactions.handleArrivalPoint(1, 41, true) == ArrivalPointDecision::Reject);
  pickup.target("0");
  pickup.ports.monotonic += 2s;
  CHECK(pickup.transactions.handleArrivalPoint(1, 41, true) == ArrivalPointDecision::Reject);
  pickup.target("0");
  CHECK(pickup.transactions.handleArrivalPoint(1, 41, true) == ArrivalPointDecision::Accept);
  pickup.transactions.handleArrival(Service::PickupRoute);
  CHECK(pickup.ports.pickup_requests == 0 && pickup.replyIs(CanProtocol::arrivalAck(false)));
  pickup.target("3");
  pickup.arrival.complete(true);
  pickup.transactions.handleArrival(Service::PickupRoute);
  pickup.transactions.handleArrival(Service::PickupRoute);
  CHECK(pickup.ports.pickup_requests == 1 && pickup.ports.roi_requests == 0);
  CHECK(pickup.ports.pickup_class == "0" && pickup.ports.pickup_token == 41 && pickup.replyIs(CanProtocol::arrivalAck()));
  pickup.transactions.handleArrival();
  CHECK(pickup.ports.roi_requests == 0 && pickup.replyIs(CanProtocol::arrivalAck(false)));
  pickup.mission(BridgeCyclePhase::WaitingTurn, 20, 41, BridgeDestination::Left);
  CHECK(!pickup.transactions.replyCurrent(CanProtocol::TURN_REQUEST_ID));
  CHECK(pickup.transactions.acceptRoiReply(pickup.ports.roi_generation, pickup.ports.monotonic));
  pickup.transactions.finishPickupRoute(true, BridgeDestination::Left, 41);
  pickup.transactions.refreshTurnRequest(pickup.ports.monotonic);
  CHECK(pickup.replyIs(CanProtocol::roiResult(true, true, 1)));
  CHECK(pickup.replyIs(CanProtocol::turnRequest(1, 41)));
  pickup.transactions.handleArrival(Service::PickupRoute);
  CHECK(pickup.ports.pickup_requests == 1 && pickup.ports.roi_requests == 0);

  for (bool disconnected : {false, true}) {
    Fixture rejected;
    rejected.arrival.complete(true);
    rejected.task("seg");
    rejected.mission(BridgeCyclePhase::SearchingTarget, 22);
    rejected.target("0");
    CHECK(rejected.transactions.handleArrivalPoint(1, 17, true) == ArrivalPointDecision::Accept);
    rejected.arrival.complete(true);
    rejected.transactions.handleArrival(Service::PickupRoute);
    if (disconnected) {
      rejected.transactions.disconnect();
      CHECK((rejected.ports.cancelled == std::vector<std::pair<Service, std::int64_t>>{{Service::PickupRoute, 1}}));
      CHECK(!rejected.transactions.acceptRoiReply(rejected.ports.roi_generation, rejected.ports.monotonic));
    } else {
      CHECK(rejected.transactions.acceptRoiReply(rejected.ports.roi_generation, rejected.ports.monotonic));
      rejected.transactions.finishPickupRoute(true, BridgeDestination::Left, 18);
    }
    rejected.mission(BridgeCyclePhase::WaitingTurn, 22, 18, BridgeDestination::Left);
    rejected.transactions.handleArrival(Service::PickupRoute);
    CHECK(!rejected.transactions.replyCurrent(CanProtocol::TURN_REQUEST_ID));
    CHECK(rejected.ports.pickup_requests == 1 && rejected.ports.roi_requests == 0);
  }

  Fixture failed;
  failed.mission(BridgeCyclePhase::SearchingTarget, 10);
  failed.transactions.handleArrival();
  CHECK(failed.state.locallyHeld());
  CHECK(failed.replyIs(CanProtocol::arrivalAck()));
  failed.transactions.handleArrival();
  CHECK(failed.ports.roi_requests == 1);

  // 恰好到截止时拒收服务应答；迟到的 Mission 提交不覆盖本地失败。
  failed.ports.monotonic += request_timeout;
  CHECK(!failed.transactions.acceptRoiReply(failed.ports.roi_generation, failed.ports.monotonic));
  CHECK((failed.ports.cancelled == std::vector<std::pair<Service, std::int64_t>>{{Service::Roi, 1}}));
  CHECK(failed.replyIs(CanProtocol::roiResult(false, false, 0)));
  for (const auto phase : {BridgeCyclePhase::CheckingRoi, BridgeCyclePhase::WaitingTurn}) {
    failed.mission(phase, 10, roi_id, BridgeDestination::Left);
    failed.ports.queued.clear();
    failed.transactions.handleArrival();
    CHECK((failed.ports.queued ==
      std::vector<canid_t>{CanProtocol::ARRIVAL_ACK_ID, CanProtocol::ROI_RESULT_ID}));
    CHECK(failed.replyIs(CanProtocol::roiResult(false, false, 0)));
    CHECK(!failed.transactions.replyCurrent(CanProtocol::TURN_REQUEST_ID));
  }
  CHECK(failed.ports.roi_requests == 1);

  // 本机从未尝试过 ROI 时，显式到达查询允许恢复 Mission 已经提交的转向。
  Fixture recovered;
  recovered.mission(BridgeCyclePhase::WaitingTurn, 10, roi_id, BridgeDestination::Left);
  CHECK(!recovered.transactions.replyCurrent(CanProtocol::TURN_REQUEST_ID));
  recovered.transactions.handleArrival();
  CHECK(recovered.transactions.replyCurrent(CanProtocol::TURN_REQUEST_ID));
  CHECK(recovered.replyIs(CanProtocol::turnRequest(1, roi_token)));
  CHECK(recovered.ports.roi_requests == 0);

  Fixture route;
  route.mission(BridgeCyclePhase::SearchingTarget, 10);
  route.transactions.handleArrival();
  CHECK(route.transactions.acceptRoiReply(route.ports.roi_generation, route.ports.monotonic));
  route.transactions.finishRoi({true, true, BridgeDestination::Left}, roi_id);
  CHECK(!route.transactions.replyCurrent(CanProtocol::TURN_REQUEST_ID));
  route.mission(BridgeCyclePhase::WaitingTurn, 10, roi_token, BridgeDestination::Left);
  CHECK(!route.transactions.replyCurrent(CanProtocol::TURN_REQUEST_ID));
  route.mission(BridgeCyclePhase::WaitingTurn, 10, roi_id, BridgeDestination::Right);
  CHECK(!route.transactions.replyCurrent(CanProtocol::TURN_REQUEST_ID));
  route.mission(BridgeCyclePhase::WaitingTurn, 10, roi_id, BridgeDestination::Left);
  CHECK(route.replyIs(CanProtocol::turnRequest(1, roi_token)));
  CHECK(route.transactions.replyCurrent(CanProtocol::TURN_REQUEST_ID));

  route.transactions.handleMissionCommand(CanProtocol::TURN_COMPLETED, roi_token);
  CHECK(route.ports.command_requests == 1 && route.ports.last_command.roi_request_id == roi_id);
  const auto command = route.ports.last_command;
  CHECK(route.transactions.acceptCommandReply(command.generation, route.ports.monotonic));
  route.ports.queued.clear();
  route.transactions.commandReply(
    command.command, command.requested_cycle_generation, true, route.ports.monotonic);
  // 服务成功只表示受理。Mission 确认切换后才能回成功，再换路由时旧回复失去发送资格。
  CHECK(route.ports.queued.empty());
  route.mission(BridgeCyclePhase::NavigatingSafety, 11, roi_id, BridgeDestination::Left);
  CHECK(route.replyIs(CanProtocol::commandResult(true, CanProtocol::TURN_COMPLETED, roi_token)));
  CHECK(route.transactions.replyCurrent(CanProtocol::COMMAND_RESULT_ID));
  route.mission(BridgeCyclePhase::NavigatingSafety, 11, roi_token, BridgeDestination::Left);
  CHECK(!route.transactions.replyCurrent(CanProtocol::COMMAND_RESULT_ID));

  // 首次对准只认无路由初态；任务实际切到 field 和 ROS 服务完成可以任意先后到达。
  constexpr auto align = CanProtocol::START_ALIGN;
  constexpr std::uint32_t token = 0x01020304u;
  for (bool status_first : {false, true}) {
    Fixture f;
    f.arrival.complete(true);
    f.task("seg");
    f.mission(BridgeCyclePhase::Fault, 0);
    f.transactions.handleMissionCommand(align, token);
    f.transactions.handleMissionCommand(align, token);
    CHECK(f.ports.command_requests == 1 && f.ports.last_command.command == align &&
      f.ports.last_command.roi_request_id == 0 && f.ports.last_command.requested_cycle_generation == 0);
    f.transactions.handleArrival();
    CHECK(f.ports.roi_requests == 0 && f.replyIs(CanProtocol::arrivalAck(false)));
    CHECK(f.transactions.handleArrivalPoint(1, 55, true) == ArrivalPointDecision::Reject);
    f.ports.queued.clear();
    if (status_first) {
      f.task("field");
      f.mission(BridgeCyclePhase::InitialLocalizing, 1);
      CHECK(f.ports.queued.empty());
      f.acceptLatestCommand();
    } else {
      f.acceptLatestCommand();
      f.transactions.handleMissionCommand(align, token);
      f.mission(BridgeCyclePhase::InitialLocalizing, 1);
      CHECK(f.ports.queued.empty());
      f.task("field");
    }
    CHECK(f.replyIs(CanProtocol::commandResult(true, align, token)));
    CHECK(f.transactions.replyCurrent(CanProtocol::COMMAND_RESULT_ID));
    f.transactions.handleMissionCommand(align, token);
    CHECK(f.ports.command_requests == 1 && f.replyIs(CanProtocol::commandResult(true, align, token)));
    f.mission(BridgeCyclePhase::SearchingTarget, 2);
    CHECK(!f.transactions.replyCurrent(CanProtocol::COMMAND_RESULT_ID));
    f.transactions.handleMissionCommand(align, token);
    CHECK(f.replyIs(CanProtocol::commandResult(false, align, token)));
  }
  // 两段共用第一次请求的截止，服务受理、USB 重传和晚到的状态均不能重新计时。
  Fixture late;
  late.arrival.complete(true);
  late.mission(BridgeCyclePhase::SearchingTarget, 0);
  late.transactions.handleMissionCommand(align, token);
  late.ports.monotonic += 90ms;
  late.acceptLatestCommand();
  late.transactions.handleMissionCommand(align, token);
  late.ports.monotonic += 10ms;
  late.task("field");
  late.mission(BridgeCyclePhase::InitialLocalizing, 1);
  late.transactions.handleMissionCommand(align, token);
  CHECK(late.ports.command_requests == 1 && late.replyIs(CanProtocol::commandResult(false, align, token)));

  Fixture occupied;
  occupied.mission(BridgeCyclePhase::SearchingTarget, 0);
  occupied.transactions.handleMissionCommand(align, token);  // 夹爪启动仍在途。
  CHECK(occupied.ports.command_requests == 0 && occupied.replyIs(CanProtocol::commandResult(false, align, token)));
  occupied.arrival.complete(true);
  occupied.transactions.handleMissionCommand(align, token);
  CHECK(occupied.ports.command_requests == 0);
  occupied.mission(BridgeCyclePhase::WaitingTurn, 1, roi_id, BridgeDestination::Left);
  occupied.transactions.handleMissionCommand(align, token + 1);
  CHECK(occupied.ports.command_requests == 0 && occupied.replyIs(CanProtocol::commandResult(false, align, token + 1)));

  Fixture lost;
  lost.arrival.complete(true);
  lost.mission(BridgeCyclePhase::SearchingTarget, 0);
  lost.transactions.handleMissionCommand(align, token);
  lost.transactions.disconnect();
  lost.transactions.handleMissionCommand(align, token);
  CHECK(lost.ports.command_requests == 1 && lost.replyIs(CanProtocol::commandResult(false, align, token)));
}
