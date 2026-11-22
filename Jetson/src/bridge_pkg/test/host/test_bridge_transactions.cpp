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
};
}  // namespace

void testBridgeTransactions()
{
  Group group("BridgeTransactions arrival, ROI and command replies");

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
}
