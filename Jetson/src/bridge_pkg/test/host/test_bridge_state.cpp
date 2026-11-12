#include "check.hpp"
#include "state/bridge_state.hpp"

#include <chrono>
#include <cmath>
#include <stdexcept>
#include <string>

namespace {

constexpr std::int64_t kSecond = 1000000000LL;

BridgeState makeState()
{
  BridgeStateConfig config;
  config.image_width = 1920;
  config.image_height = 1080;
  config.input_timeout_s = 1.0;
  config.target_classes = {"0", "2", "3"};
  config.first_class = "0";
  return BridgeState(config);
}

RecognitionDetection detection(const char * id, const char * class_id, double x, double y)
{
  RecognitionDetection value;
  value.id = id;
  value.box = {x, y, 20, 20, 0};
  value.hypothesis = RecognitionHypothesis{class_id, 0.9};
  return value;
}

BridgeMissionState missionAt(
  std::int64_t stamp_ns, BridgeCyclePhase phase, BridgeTransportPhase transport,
  std::uint64_t generation, BridgeDestination destination = BridgeDestination::None,
  BridgeOutcome outcome = BridgeOutcome::None)
{
  BridgeMissionState mission;
  mission.stamp = {stamp_ns};
  mission.operation = BridgeOperation::Idle;
  mission.cycle_phase = phase;
  mission.transport_phase = transport;
  mission.cycle_generation = generation;
  mission.destination = destination;
  mission.outcome = outcome;
  return mission;
}

// 寻找 + 实际 seg。返回后中心缓存是空的，调用方再喂帧。
void armSearch(BridgeState & state, std::int64_t now_ns, std::uint64_t generation)
{
  BridgeTaskState task;
  task.stamp = {now_ns - kSecond / 2};
  task.task = "seg";
  task.epoch = 1;
  state.task(task, {now_ns});
  state.mission(
    missionAt(now_ns - kSecond / 2, BridgeCyclePhase::SearchingTarget, BridgeTransportPhase::First, generation),
    {now_ns});
}

BridgeMonotonicTime monoAt(std::chrono::steady_clock::duration offset)
{
  return {std::chrono::steady_clock::time_point{} + offset};
}

}  // namespace

void testBridgeState()
{
  Group group("BridgeState");
  const std::int64_t now_ns = 10 * kSecond;
  const BridgeSourceTime now{now_ns};
  const auto arrival = monoAt(std::chrono::seconds(10));

  BridgeMissionWatchdog watchdog;
  CHECK(!watchdog.alive(monoAt(std::chrono::seconds(10))));
  watchdog.received(monoAt(std::chrono::seconds(10)));
  CHECK(watchdog.alive(monoAt(std::chrono::milliseconds(10499))));
  CHECK(!watchdog.alive(monoAt(std::chrono::milliseconds(10500))));
  CHECK(!watchdog.alive(monoAt(std::chrono::milliseconds(9999))));
  watchdog.received(monoAt(std::chrono::milliseconds(10500)));
  CHECK(watchdog.alive(monoAt(std::chrono::milliseconds(10500))));
  BridgeState state = makeState();
  armSearch(state, now_ns, 2);
  BridgeTargetInput target;
  target.stamp = {now_ns - kSecond / 10};
  target.frame = "cam";
  target.detection = detection("a", "0", 960, 540);
  CHECK(state.target(target, now, arrival, true));
  const auto output = state.targetOutput(now, arrival);
  CHECK(output.valid && output.x == 5000 && output.y == 5000 && output.source_frame == 9900);

  // 拒绝：插座关、不是 seg、源时间非法、旧于截止。这些帧不覆盖上一帧。
  CHECK(!state.target(target, now, arrival, false));
  BridgeState field_task = makeState();
  BridgeTaskState not_seg;
  not_seg.stamp = {now_ns - 10};
  not_seg.task = "field";
  not_seg.epoch = 1;
  field_task.task(not_seg, now);
  CHECK(!field_task.target(target, now, arrival, true));
  BridgeTargetInput future = target;
  future.stamp = {now_ns + 1};
  CHECK(!state.target(future, now, arrival, true));
  BridgeTargetInput zero = target;
  zero.stamp = {0};
  CHECK(!state.target(zero, now, arrival, true));
  BridgeTargetInput stale = target;
  stale.stamp = {now_ns - kSecond - 1};
  CHECK(!state.target(stale, now, arrival, true));
  CHECK(state.targetOutput(now, arrival).valid);

  // 首趟只收类别 0。空检测覆盖成无效，不再沿用上一帧坐标。
  BridgeTargetInput wrong_class = target;
  wrong_class.stamp = {target.stamp.ns + 1};
  wrong_class.detection = detection("b", "2", 960, 540);
  CHECK(state.target(wrong_class, now, arrival, true) && !state.targetOutput(now, arrival).valid);
  BridgeTargetInput empty = wrong_class;
  empty.stamp = {wrong_class.stamp.ns + 1};
  empty.detection.reset();
  CHECK(state.target(empty, now, arrival, true) && !state.targetOutput(now, arrival).valid);

  // 换 frame 的第一帧只建立来源。这一步必须在 Mission 清缓存之前，清掉之后没有“上一帧”可比。
  BridgeTargetInput changed = empty;
  changed.stamp = {empty.stamp.ns + 1};
  changed.frame = "cam2";
  changed.detection = detection("c", "0", 960, 540);
  CHECK(state.target(changed, now, arrival, true) && !state.targetOutput(now, arrival).valid);
  changed.stamp = {changed.stamp.ns + 1};
  CHECK(state.target(changed, now, arrival, true));
  CHECK(state.targetOutput(now, arrival).valid && state.targetOutput(now, arrival).x == 5000);

  // 后续运输才收类别 2。状态戳不晚于下一帧，否则 cutoff 会把这帧拒掉。
  auto later = missionAt(
    now_ns - kSecond / 2, BridgeCyclePhase::SearchingTarget, BridgeTransportPhase::Later, 2);
  const auto later_update = state.mission(later, now);
  CHECK(later_update.accepted && later_update.reset_stream && !state.targetOutput(now, arrival).valid);
  BridgeTargetInput class_two = changed;
  class_two.stamp = {changed.stamp.ns + 1};
  class_two.detection = detection("d", "2", 960, 540);
  CHECK(state.target(class_two, now, arrival, true) && state.targetOutput(now, arrival).valid);

  // 源龄和到达龄取等号仍有效；再老 1 ms 就无效。
  // 任务戳不能晚于这帧，否则 cutoff 会先于 1 秒年龄边界把帧拒掉。
  BridgeState fresh = makeState();
  BridgeTaskState old_task;
  old_task.stamp = {now_ns - kSecond};
  old_task.task = "seg";
  old_task.epoch = 1;
  fresh.task(old_task, now);
  fresh.mission(
    missionAt(now_ns - kSecond, BridgeCyclePhase::SearchingTarget, BridgeTransportPhase::First, 2), now);
  BridgeTargetInput edge = target;
  edge.stamp = {now_ns - kSecond};
  CHECK(fresh.target(edge, now, monoAt(std::chrono::seconds(9)), true));
  CHECK(fresh.targetOutput(now, monoAt(std::chrono::seconds(10))).valid);
  CHECK(!fresh.targetOutput(now, monoAt(std::chrono::milliseconds(10001))).valid);

  // 同代心跳不解闭锁。新代次且仍在寻找 + seg 才放行，旧中心不复活。
  fresh.holdCoordinates(now, false);
  CHECK(fresh.locallyHeld() && !fresh.targetOutput(now, arrival).valid);
  auto same_gen = missionAt(
    now_ns, BridgeCyclePhase::SearchingTarget, BridgeTransportPhase::First, 2);
  CHECK(fresh.mission(same_gen, now).accepted && !fresh.mission(same_gen, now).reset_stream);
  CHECK(fresh.locallyHeld());
  auto next_gen = missionAt(
    now_ns, BridgeCyclePhase::SearchingTarget, BridgeTransportPhase::First, 3);
  CHECK(fresh.mission(next_gen, now).reset_stream);
  CHECK(!fresh.locallyHeld() && !fresh.targetOutput(now, arrival).valid);

  // 205 的直到重新寻找：中间切到前往安全区也不解锁。
  BridgeState held = makeState();
  armSearch(held, now_ns, 4);
  held.holdCoordinates(now, true);
  BridgeTaskState field;
  field.stamp = {now_ns - 1000};
  field.task = "field";
  field.epoch = 2;
  CHECK(held.task(field, now).accepted);
  auto navigating = missionAt(
    now_ns, BridgeCyclePhase::NavigatingSafety, BridgeTransportPhase::First, 5,
    BridgeDestination::Left, BridgeOutcome::Success);
  CHECK(held.mission(navigating, now).accepted && held.locallyHeld());
  BridgeTaskState seg_again;
  seg_again.stamp = {now_ns};
  seg_again.task = "seg";
  seg_again.epoch = 3;
  held.task(seg_again, now);
  auto search_again = missionAt(
    now_ns, BridgeCyclePhase::SearchingTarget, BridgeTransportPhase::First, 6);
  held.mission(search_again, now);
  CHECK(!held.locallyHeld());

  // Mission 非法枚举、更旧时间直接丢掉。完全相同的心跳不抬截止、不撤周期帧。
  BridgeState gate = makeState();
  auto bad = missionAt(now_ns - 1, BridgeCyclePhase::SearchingTarget, BridgeTransportPhase::First, 1);
  bad.destination = static_cast<BridgeDestination>(9);
  CHECK(!gate.mission(bad, now).accepted && !gate.cyclePhase());
  auto first = missionAt(now_ns - 1, BridgeCyclePhase::SearchingTarget, BridgeTransportPhase::First, 1);
  const auto first_update = gate.mission(first, now);
  const auto duplicate = gate.mission(first, now);
  CHECK(first_update.accepted && first_update.reset_stream && duplicate.accepted && !duplicate.reset_stream);
  auto older = first;
  older.stamp = {now_ns - 2};
  CHECK(!gate.mission(older, now).accepted);

  // 安全区中心已经是归一化值。侧别必须和已成功提交的目的区一致。
  BridgeState safety = makeState();
  BridgeTaskState field_only;
  field_only.stamp = {now_ns - 3000};
  field_only.task = "field";
  field_only.epoch = 1;
  safety.task(field_only, now);
  auto safety_mission = navigating;
  safety_mission.stamp = {now_ns - 2000};
  safety.mission(safety_mission, now);
  BridgeSafetyInput right_side;
  right_side.stamp = {now_ns - 1000};
  right_side.frame = "field";
  right_side.valid = true;
  right_side.side = 2;
  right_side.x = 0.25;
  right_side.y = 0.5;
  CHECK(safety.safety(right_side, now, arrival, true));
  CHECK(!safety.safetyOutput(now, arrival).valid);
  BridgeSafetyInput left_side = right_side;
  left_side.stamp = {right_side.stamp.ns + 1};
  left_side.side = 1;
  CHECK(safety.safety(left_side, now, arrival, true));
  const auto safety_output = safety.safetyOutput(now, arrival);
  CHECK(safety_output.valid && safety_output.x == 2500 && safety_output.y == 5000);

  // 桥收到的 ROI 是另一进程的消息，组合和 3 件上限在这里再看一次。
  const auto left = BridgeRoiResult::checked(true, true, {1, 0, 0, 0, 0});
  CHECK(left.valid && left.passed && left.destination == BridgeDestination::Left);
  const auto blue = BridgeRoiResult::checked(true, true, {1, 1, 0, 0, 0});
  CHECK(blue.valid && !blue.passed && blue.destination == BridgeDestination::None);
  const auto too_many = BridgeRoiResult::checked(true, true, {3, 0, 1, 0, 0});
  CHECK(!too_many.passed);
  const auto right = BridgeRoiResult::checked(true, true, {0, 0, 0, 1, 0});
  CHECK(right.passed && right.destination == BridgeDestination::Right);
  const auto not_valid = BridgeRoiResult::checked(false, true, {1, 0, 0, 0, 0});
  CHECK(!not_valid.valid && !not_valid.passed);

  // 服务等待：忙不刷新代次；恰好到截止不能完成，只能由 expire 收尾。
  BridgeServiceRequest request;
  const auto t0 = std::chrono::steady_clock::time_point{};
  const auto generation = request.begin({t0}, std::chrono::milliseconds(100), 4);
  CHECK(generation && *generation == 1 && !request.begin({t0}, std::chrono::milliseconds(100), 1));
  CHECK(!request.bind(2, 7) && request.bind(1, 7));
  CHECK(request.complete(1, {t0 + std::chrono::milliseconds(99)}) && !request.waiting());
  const auto again = request.begin({t0}, std::chrono::milliseconds(100), 1);
  CHECK(again && *again == 2 && !request.complete(2, {t0 + std::chrono::milliseconds(100)}));
  const auto expired = request.expire({t0 + std::chrono::milliseconds(100)});
  // begin 不清除上一次 bind 的 request_id。新 bind 之前超时，撤单编号仍是 7。
  CHECK(expired && expired->generation == 2 && expired->command == 1 && expired->request_id == 7 && !request.waiting());
  CHECK(request.begin({t0}, std::chrono::milliseconds(50), 3) && request.cancel()->command == 3 && !request.cancel());
}

void testSafetyHold()
{
  Group group("BridgeState safety hold");
  using std::chrono::milliseconds;
  const std::int64_t now_ns = 10 * kSecond;
  const BridgeSourceTime now{now_ns};
  // 前往左侧安全区且结果成功；makeState 的 safety_hold_s 为默认 0.20 s、input_timeout_s 为 1.0 s。
  auto navigatingState = [&]() {
      BridgeState state = makeState();
      BridgeTaskState field;
      field.stamp = {now_ns - 3000};
      field.task = "field";
      field.epoch = 1;
      state.task(field, now);
      state.mission(missionAt(now_ns - 2000, BridgeCyclePhase::NavigatingSafety,
        BridgeTransportPhase::First, 5, BridgeDestination::Left, BridgeOutcome::Success), now);
      return state;
    };
  auto frame = [&](std::int64_t stamp_ns, bool valid, double x) {
      BridgeSafetyInput input;
      input.stamp = {stamp_ns};
      input.frame = "field";
      input.valid = valid;
      input.side = 1;
      input.x = x;
      input.y = 0.5;
      return input;
    };
  const auto t0 = monoAt(std::chrono::seconds(10));

  // 有效中心到达后 0.20 s 内可发（含边界），之后撤销。
  BridgeState state = navigatingState();
  CHECK(state.safety(frame(now_ns - 1000, true, 0.25), now, t0, true));
  const auto measured = state.safetyOutput(now, monoAt(milliseconds(10200)));
  CHECK(measured.valid);
  CHECK(!state.safetyOutput(now, monoAt(milliseconds(10201))).valid);

  // 分区不可靠的无效帧不撤销保持中心，下一帧有效中心替换它。
  CHECK(state.safety(frame(now_ns - 900, false, 0.0), now, monoAt(milliseconds(10050)), true));
  auto output = state.safetyOutput(now, monoAt(milliseconds(10100)));
  CHECK(output.valid && output.x == 2500);
  // 保持帧发的是最近一次实测：source_frame 不随无效帧前移，STM32 按同一源帧去重。
  CHECK(output.source_frame == measured.source_frame);
  CHECK(state.safety(frame(now_ns - 800, true, 0.30), now, monoAt(milliseconds(10150)), true));
  output = state.safetyOutput(now, monoAt(milliseconds(10340)));
  CHECK(output.valid && output.x == 3000);

  // 无效帧仍推进源时间水位：更旧的帧被拒，不能复活保持。
  CHECK(!state.safety(frame(now_ns - 850, true, 0.40), now, monoAt(milliseconds(10160)), true));

  // frame_id 改变立即撤销保持。
  auto other = frame(now_ns - 700, true, 0.30);
  other.frame = "other";
  CHECK(state.safety(other, now, monoAt(milliseconds(10170)), true));
  CHECK(!state.safetyOutput(now, monoAt(milliseconds(10170))).valid);

  // 源帧龄仍按 input_timeout_s：保持期内但源时间过旧也不可发。
  BridgeState stale = navigatingState();
  CHECK(stale.safety(frame(now_ns - 1000, true, 0.25), now, t0, true));
  CHECK(!stale.safetyOutput({now_ns + kSecond}, monoAt(milliseconds(10100))).valid);

  // 任务阶段变化清掉保持。
  BridgeState reset = navigatingState();
  CHECK(reset.safety(frame(now_ns - 1000, true, 0.25), now, t0, true));
  reset.mission(missionAt(now_ns - 500, BridgeCyclePhase::SearchingTarget,
    BridgeTransportPhase::First, 6), now);
  CHECK(!reset.safetyOutput(now, t0).valid);

  // 入口校验：保持须在 (0, input_timeout_s]，且保持加一个 field 帧周期短于 STM32 的 0.25 s。
  auto rejects = [](double hold_s, double fps) {
      BridgeStateConfig config;
      config.input_timeout_s = 0.25;
      config.safety_hold_s = hold_s;
      config.field_max_fps = fps;
      try {BridgeState state(config);} catch (const std::invalid_argument &) {return true;}
      return false;
    };
  CHECK(rejects(0.0, 100.0));
  CHECK(rejects(0.26, 100.0));
  CHECK(rejects(std::nan(""), 100.0));
  CHECK(rejects(0.24, 100.0));
  CHECK(rejects(0.2, 0.0));
  CHECK(rejects(0.2, 20.0));
  CHECK(!rejects(0.2, 100.0));
}
