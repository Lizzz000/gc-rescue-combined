#include "check.hpp"
#include "node/bridge_node.hpp"

// The host build replaces only ROS I/O/types and the physical gripper.
// All request dispatch, callbacks, caches, coordinate gates and cancellation below are production code.
struct BridgeRuntimeTestAccess
{
  static bool receive(BridgeRuntime & runtime, canid_t type, std::uint32_t id = 0)
  {
    can_frame frame{};
    frame.can_id = type;
    frame.len = type == CanProtocol::ARRIVAL_ID ? 1 : 5;
    frame.data[0] = 1;
    for (unsigned i = 0; i < 4; ++i) {frame.data[i + 1] = static_cast<std::uint8_t>(id >> (8 * i));}
    return runtime.handleCommand(frame);
  }
  static const can_frame & reply(const BridgeRuntime & runtime)
  {return runtime.replies_.frame(CanProtocol::COMMAND_RESULT_ID);}
  static bool held(const BridgeRuntime & runtime) {return runtime.state_.locallyHeld();}
  static bool aborted(const BridgeRuntime & runtime) {return runtime.route_aborted_;}
  static bool authorized(const BridgeRuntime & runtime) {return runtime.turnAuthorized();}
  static bool modePending(const BridgeRuntime & runtime) {return runtime.pending_mode_reply_.has_value();}
  static bool routeCachesEmpty(const BridgeRuntime & runtime)
  {
    for (const auto & accepted : runtime.accepted_commands_) {if (accepted) {return false;}}
    return !runtime.roi_confirmation_ && !runtime.turn_ && !runtime.last_delivery_request_ &&
      !runtime.command_reply_identity_ && !runtime.pending_command_identity_;
  }
  static void expire(BridgeRuntime & runtime)
  {runtime.checkRequestTimeouts(BridgeRuntime::Clock::now() + std::chrono::seconds(10));}
  static void disconnect(BridgeRuntime & runtime) {runtime.closeSocket("host test disconnect");}
  static bool targetValid(const BridgeRuntime & runtime)
  {return runtime.currentFrame(CanProtocol::TARGET_ID).data[0] != 0;}
};

namespace {
class HostTransport final : public CanTransport
{
public:
  bool opened{true};
  bool open(const std::string &, std::string &) override {opened = true; return true;}
  void close() override {opened = false;}
  bool socketOpen() const override {return opened;}
  ReadResult read(can_frame &, std::string &) override {return ReadResult::EMPTY;}
  WriteResult write(const can_frame &, std::string &) override {return WriteResult::SENT;}
};

struct Fixture
{
  rclcpp::Node node;
  std::shared_ptr<rclcpp::Client<interface_pkg::srv::CheckRoi>> roi =
    std::make_shared<rclcpp::Client<interface_pkg::srv::CheckRoi>>();
  std::shared_ptr<rclcpp::Client<interface_pkg::srv::MissionCommand>> command =
    std::make_shared<rclcpp::Client<interface_pkg::srv::MissionCommand>>();
  std::unique_ptr<BridgeRuntime> runtime;

  Fixture()
  {
    BridgeRuntime::Config config;
    config.request_timeout = std::chrono::seconds(1);
    config.tx_queue_timeout = std::chrono::seconds(1);
    config.mission_watchdog_timeout = std::chrono::seconds(60);
    runtime = std::make_unique<BridgeRuntime>(node, config, BridgeRuntime::Ports{
      std::make_shared<rclcpp::Publisher<interface_pkg::msg::BridgeStatus>>(), roi, command},
      std::make_unique<HostTransport>());
    task();
    mission(10);
  }
  void task()
  {
    auto message = std::make_shared<interface_pkg::msg::TaskState>();
    message->header.stamp = rclcpp::Time(++node.time_ns);
    message->task = "seg";
    runtime->taskCallback(message);
  }
  void mission(std::uint64_t generation, BridgeCyclePhase phase = BridgeCyclePhase::SearchingTarget,
    BridgeOutcome outcome = BridgeOutcome::Success, std::uint64_t roi_id = 0)
  {
    auto message = std::make_shared<interface_pkg::msg::MissionStatus>();
    message->header.stamp = rclcpp::Time(++node.time_ns);
    message->cycle_generation = generation;
    message->cycle_phase = static_cast<std::uint8_t>(phase);
    message->outcome = static_cast<std::uint8_t>(outcome);
    message->operation = phase == BridgeCyclePhase::SwitchingSeg ? 2 : 0;
    message->roi_request_id = roi_id;
    message->destination = roi_id == 0 ? 0 : 1;
    runtime->missionCallback(message);
  }
  void search(std::uint32_t id) {CHECK(BridgeRuntimeTestAccess::receive(*runtime, 19, id));}
  bool result(bool success, std::uint32_t id) const
  {
    const auto & frame = BridgeRuntimeTestAccess::reply(*runtime);
    std::uint32_t token = 0;
    for (unsigned i = 0; i < 4; ++i) {token |= std::uint32_t(frame.data[i + 2]) << (8 * i);}
    return frame.len == 6 && frame.data[0] == success && frame.data[1] == 3 && token == id;
  }
  void target()
  {
    auto message = std::make_shared<vision_msgs::msg::Detection2DArray>();
    message->header.stamp = rclcpp::Time(++node.time_ns);
    message->header.frame_id = "cam";
    RecognitionDetection detection;
    detection.id = "a";
    detection.box = {960, 540, 20, 20, 0};
    detection.hypothesis = RecognitionHypothesis{"0", 0.9};
    message->detections.push_back(detection);
    runtime->targetCallback(message);
  }
};
}

void testBridgeRuntimeSearch()
{
  Group group("BridgeRuntime search transactions and restart");
  for (int order = 0; order < 3; ++order) {
    Fixture fixture;
    fixture.search(100);
    fixture.search(100);
    CHECK(fixture.command->requests.size() == 1);
    if (order == 0) {fixture.command->reply(0, {true});}
    fixture.mission(11, BridgeCyclePhase::SwitchingSeg, BridgeOutcome::Pending);
    if (order == 1) {fixture.command->reply(0, {true});}
    fixture.mission(11);
    if (order == 2) {fixture.command->reply(0, {true});}
    CHECK(!BridgeRuntimeTestAccess::held(*fixture.runtime));
    CHECK(!BridgeRuntimeTestAccess::modePending(*fixture.runtime));
    CHECK(fixture.result(true, 100));
    fixture.search(100); // Lost terminal reply must not execute a second ROS switch.
    CHECK(fixture.command->requests.size() == 1 && fixture.result(true, 100));
    fixture.search(99);
    CHECK(fixture.command->requests.size() == 1 && fixture.result(false, 99));

    fixture.mission(0); // Mission alone restarts, STM32 request IDs continue.
    CHECK(BridgeRuntimeTestAccess::held(*fixture.runtime));
    CHECK(BridgeRuntimeTestAccess::aborted(*fixture.runtime));
    fixture.target();
    CHECK(!BridgeRuntimeTestAccess::targetValid(*fixture.runtime));
    fixture.search(100);
    CHECK(fixture.command->requests.size() == 1 && fixture.result(false, 100));
    fixture.search(101);
    fixture.command->reply(1, {true});
    CHECK(BridgeRuntimeTestAccess::held(*fixture.runtime));
    fixture.mission(1);
    CHECK(!BridgeRuntimeTestAccess::held(*fixture.runtime));
    CHECK(!BridgeRuntimeTestAccess::aborted(*fixture.runtime));
    CHECK(fixture.result(true, 101));
    fixture.target();
    CHECK(BridgeRuntimeTestAccess::targetValid(*fixture.runtime));
  }

  // Service rejection, unavailable service, service timeout, mode timeout and disconnect all cache failure.
  for (int failure = 0; failure < 5; ++failure) {
    Fixture fixture;
    if (failure == 0) {fixture.command->ready = false;}
    fixture.search(200);
    if (failure == 1) {fixture.command->reply(0, {false});}
    if (failure == 2) {BridgeRuntimeTestAccess::expire(*fixture.runtime);}
    if (failure == 3) {
      fixture.command->reply(0, {true});
      BridgeRuntimeTestAccess::expire(*fixture.runtime);
    }
    if (failure == 4) {BridgeRuntimeTestAccess::disconnect(*fixture.runtime);}
    const auto sent = fixture.command->requests.size();
    fixture.command->ready = true;
    fixture.search(200);
    CHECK(fixture.command->requests.size() == sent && fixture.result(false, 200));
    if (failure >= 2) {
      fixture.command->reply(0, {true}); // Queued late callback must not turn failure into success.
      fixture.search(200);
      CHECK(fixture.command->requests.size() == sent && fixture.result(false, 200));
    }
  }

  // Restart invalidates an outstanding service callback and preserves the search-ID watermark.
  Fixture pending;
  pending.search(300);
  pending.mission(0);
  CHECK(pending.command->removed.size() == 1);
  pending.command->reply(0, {true});
  CHECK(BridgeRuntimeTestAccess::held(*pending.runtime));
  pending.search(300);
  CHECK(pending.command->requests.size() == 1 && pending.result(false, 300));
  pending.search(301);
  pending.command->reply(0, {true}); // An older generation cannot consume the new wait.
  CHECK(BridgeRuntimeTestAccess::held(*pending.runtime));
  pending.command->reply(1, {true});
  pending.mission(1);
  CHECK(pending.result(true, 301));

  // A previously published Fault must not reject the new recovery search when its ROS reply arrives first.
  Fixture fault;
  fault.mission(10, BridgeCyclePhase::Fault, BridgeOutcome::Rejected);
  fault.search(400);
  fault.command->reply(0, {true});
  CHECK(BridgeRuntimeTestAccess::modePending(*fault.runtime));
  fault.mission(11);
  CHECK(fault.result(true, 400));

  // Restart while ROI is in flight cancels it; an old successful ROI cannot authorize a new turn.
  Fixture roi;
  CHECK(BridgeRuntimeTestAccess::receive(*roi.runtime, 16));
  CHECK(roi.roi->requests.size() == 1);
  roi.mission(0);
  CHECK(roi.roi->removed.size() == 1);
  interface_pkg::srv::CheckRoi::Response passed;
  passed.result.valid = passed.result.passed = true;
  passed.result.request_id = 77;
  roi.roi->reply(0, passed);
  roi.mission(1, BridgeCyclePhase::WaitingTurn, BridgeOutcome::Success, 77);
  CHECK(!BridgeRuntimeTestAccess::authorized(*roi.runtime));
  CHECK(BridgeRuntimeTestAccess::held(*roi.runtime));

  // Existing ROI authorization, route reply caches and an in-flight turn cannot cross a restart.
  Fixture route;
  CHECK(BridgeRuntimeTestAccess::receive(*route.runtime, 16));
  route.roi->reply(0, passed);
  route.mission(10, BridgeCyclePhase::WaitingTurn, BridgeOutcome::Success, 77);
  CHECK(BridgeRuntimeTestAccess::authorized(*route.runtime));
  CHECK(BridgeRuntimeTestAccess::receive(*route.runtime, 17, 77));
  CHECK(route.command->requests.size() == 1);
  route.mission(0);
  CHECK(route.command->removed.size() == 1);
  CHECK(BridgeRuntimeTestAccess::routeCachesEmpty(*route.runtime));
  route.command->reply(0, {true});
  route.mission(1, BridgeCyclePhase::NavigatingSafety, BridgeOutcome::Success, 77);
  CHECK(BridgeRuntimeTestAccess::routeCachesEmpty(*route.runtime));
  CHECK(BridgeRuntimeTestAccess::held(*route.runtime));

  // While another service owns the wait, a rejected new search ID is terminal and remains rejected.
  Fixture busy;
  CHECK(BridgeRuntimeTestAccess::receive(*busy.runtime, 16));
  busy.search(500);
  CHECK(busy.result(false, 500) && busy.command->requests.empty());
  busy.roi->reply(0, {});
  busy.search(500);
  CHECK(busy.result(false, 500) && busy.command->requests.empty());
  busy.search(501);
  CHECK(busy.command->requests.size() == 1);

  // Reboot during the topic-confirmation wait invalidates that wait too.
  Fixture mode;
  mode.search(600);
  mode.command->reply(0, {true});
  CHECK(BridgeRuntimeTestAccess::modePending(*mode.runtime));
  mode.mission(0);
  CHECK(!BridgeRuntimeTestAccess::modePending(*mode.runtime));
  mode.mission(1);
  CHECK(BridgeRuntimeTestAccess::held(*mode.runtime));
  mode.search(600);
  CHECK(mode.result(false, 600) && mode.command->requests.size() == 1);
}
