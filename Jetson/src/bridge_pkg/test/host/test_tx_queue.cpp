#include "check.hpp"
#include "transport/transport.hpp"
#include "state/bridge_state.hpp"

#include <stdexcept>

void testCanTxQueue()
{
  Group group("CanTxQueue");
  using Clock = CanTxQueue::Clock;
  const auto t0 = Clock::time_point{};
  const auto timeout = std::chrono::milliseconds(20);

  // 即使某次机会丢失，轮转仍进入下一类，避免背压时安全区帧一直排在最后。
  CanPeriodicSchedule schedule;
  CHECK(CanPeriodicSchedule::interval == std::chrono::milliseconds(5));
  CHECK(CanPeriodicSchedule::interval * 2 == std::chrono::milliseconds(10));
  for (int i = 0; i < 3; ++i) {
    CHECK(schedule.next(true) == CanProtocol::TARGET_ID);
    CHECK(schedule.next(true) == CanProtocol::SAFETY_ID);
  }

  // Mission 尚未到达或刚好过期时，不能用无效坐标继续伪装业务存活。
  // 没目标及各等待阶段不影响发送机会：只按既有 MissionStatus 时效准入。
  BridgeMissionWatchdog mission;
  CanPeriodicSchedule guarded;
  CHECK(!guarded.next(mission.alive({t0})));
  mission.received({t0});
  auto periodic = guarded.next(mission.alive({t0}));
  CHECK(periodic && *periodic == CanProtocol::TARGET_ID);
  const auto no_target = CanProtocol::center(*periodic, false, 0, 0);
  CHECK(no_target.can_id == CanProtocol::TARGET_ID && no_target.data[0] == 0);
  periodic = guarded.next(mission.alive({t0 + std::chrono::milliseconds(499)}));
  CHECK(periodic && *periodic == CanProtocol::SAFETY_ID);
  CHECK(!guarded.next(mission.alive({t0 + std::chrono::milliseconds(500)})));
  CHECK(!guarded.next(mission.alive({t0 + std::chrono::seconds(5)})));
  mission.received({t0 + std::chrono::seconds(5)});
  CHECK(guarded.next(mission.alive({t0 + std::chrono::seconds(5)})) == CanProtocol::TARGET_ID);

  // 同 ID 再入队不改变顺序，也不把 TTL 往后推。
  CanTxQueue merged;
  merged.queue(CanProtocol::ARRIVAL_ACK_ID, t0);
  merged.queue(CanProtocol::GRIPPER_RESULT_ID, t0 + std::chrono::milliseconds(5));
  merged.queue(CanProtocol::ARRIVAL_ACK_ID, t0 + std::chrono::milliseconds(15));
  CHECK(merged.size() == 2);
  std::uint64_t drops = 0;
  auto front = merged.front(t0 + std::chrono::milliseconds(19), timeout, drops);
  CHECK(front && *front == CanProtocol::ARRIVAL_ACK_ID && drops == 0);
  // 队首按第一次入队的 t0 计算，恰好 20 ms 就淘汰。后面的结果只过了 15 ms，还留在队里。
  front = merged.front(t0 + timeout, timeout, drops);
  CHECK(front && *front == CanProtocol::GRIPPER_RESULT_ID && drops == 1 && merged.size() == 1);
  front = merged.front(t0 + timeout, timeout, drops);
  CHECK(front && *front == CanProtocol::GRIPPER_RESULT_ID && drops == 1);

  // 五类回复刚好满。第六个是容量合同被打破，不是正常背压。
  CanTxQueue full;
  const canid_t ids[] = {
    CanProtocol::GRIPPER_RESULT_ID,
    CanProtocol::ARRIVAL_ACK_ID, CanProtocol::ROI_RESULT_ID,
    CanProtocol::COMMAND_RESULT_ID, CanProtocol::TURN_REQUEST_ID};
  for (canid_t id : ids) {full.queue(id, t0);}
  CHECK(full.size() == CanTxQueue::capacity);
  bool threw = false;
  try {
    full.queue(10, t0);
  } catch (const std::logic_error &) {
    threw = true;
  }
  CHECK(threw && full.size() == 5);
  // 满队重复已有回复仍只保留一次发送机会，不误报溢出。
  full.queue(CanProtocol::ROI_RESULT_ID, t0 + std::chrono::milliseconds(1));
  CHECK(full.size() == 5);

  // 拔掉中间的回复，前后顺序与第一次入队时间保持。
  CanTxQueue order;
  order.queue(CanProtocol::GRIPPER_RESULT_ID, t0);
  order.queue(CanProtocol::ARRIVAL_ACK_ID, t0);
  order.queue(CanProtocol::ROI_RESULT_ID, t0);
  CHECK(order.drop(CanProtocol::ARRIVAL_ACK_ID) == 1 && order.size() == 2);
  drops = 0;
  front = order.front(t0, timeout, drops);
  CHECK(front && *front == CanProtocol::GRIPPER_RESULT_ID);
  order.sent();
  front = order.front(t0, timeout, drops);
  CHECK(front && *front == CanProtocol::ROI_RESULT_ID && order.size() == 1);
  order.queue(CanProtocol::COMMAND_RESULT_ID, t0);
  order.queue(CanProtocol::TURN_REQUEST_ID, t0);
  CHECK(order.drop(CanProtocol::ROI_RESULT_ID) == 1 && order.size() == 2);
  CHECK(order.drop(CanProtocol::COMMAND_RESULT_ID) == 1 && order.size() == 1);
  front = order.front(t0, timeout, drops);
  CHECK(front && *front == CanProtocol::TURN_REQUEST_ID);

  order.reset();
  CHECK(order.size() == 0);
  order.sent();
  CHECK(order.size() == 0 && order.drop(CanProtocol::GRIPPER_RESULT_ID) == 0);

  // 类型 9 有独立槽，保存结果不覆盖命令 5 的受理快照。
  BridgeReplySlots replies;
  replies.store(CanProtocol::COMMAND_RESULT_ID,
    CanProtocol::commandResult(true, CanProtocol::ARRIVAL_POINT, 44));
  replies.store(CanProtocol::GRIPPER_RESULT_ID, CanProtocol::gripperResult(1, false, 44));
  CHECK(replies.frame(CanProtocol::COMMAND_RESULT_ID).data[0] == 1);
  CHECK(replies.frame(CanProtocol::COMMAND_RESULT_ID).data[2] == 44);
  CHECK(replies.frame(CanProtocol::GRIPPER_RESULT_ID).data[1] == 0);
  CHECK(replies.frame(CanProtocol::GRIPPER_RESULT_ID).data[2] == 44);
}
