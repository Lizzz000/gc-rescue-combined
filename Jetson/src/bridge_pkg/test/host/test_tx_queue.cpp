#include "check.hpp"
#include "transport/transport.hpp"

#include <stdexcept>

void testCanTxQueue()
{
  Group group("CanTxQueue");
  using Clock = CanTxQueue::Clock;
  const auto t0 = Clock::time_point{};
  const auto timeout = std::chrono::milliseconds(20);

  // 同 ID 再入队不改变顺序，也不把 TTL 往后推。
  CanTxQueue merged;
  merged.queue(CanProtocol::TARGET_ID, t0);
  merged.queue(CanProtocol::VELOCITY_ID, t0 + std::chrono::milliseconds(5));
  merged.queue(CanProtocol::TARGET_ID, t0 + std::chrono::milliseconds(15));
  CHECK(merged.size() == 2);
  std::uint64_t drops = 0;
  auto front = merged.front(t0 + std::chrono::milliseconds(19), timeout, drops);
  CHECK(front && *front == CanProtocol::TARGET_ID && drops == 0);
  // 队首按第一次入队的 t0 计算，恰好 20 ms 就淘汰。后面的 0x180 只过了 15 ms，还留在队里。
  front = merged.front(t0 + timeout, timeout, drops);
  CHECK(front && *front == CanProtocol::VELOCITY_ID && drops == 1 && merged.size() == 1);
  front = merged.front(t0 + timeout, timeout, drops);
  CHECK(front && *front == CanProtocol::VELOCITY_ID && drops == 1);

  // 七个发送 ID 刚好满。第八个是容量合同被打破，不是正常背压。
  CanTxQueue full;
  const canid_t ids[] = {
    CanProtocol::VELOCITY_ID, CanProtocol::TARGET_ID, CanProtocol::SAFETY_ID,
    CanProtocol::ARRIVAL_ACK_ID, CanProtocol::ROI_RESULT_ID,
    CanProtocol::COMMAND_RESULT_ID, CanProtocol::TURN_REQUEST_ID};
  for (canid_t id : ids) {full.queue(id, t0);}
  CHECK(full.size() == CanTxQueue::capacity);
  bool threw = false;
  try {
    full.queue(0x194, t0);
  } catch (const std::logic_error &) {
    threw = true;
  }
  CHECK(threw && full.size() == 7);

  // 拔掉中间的事件帧，前后周期帧的顺序还在。dropStream 只撤三条周期帧。
  CanTxQueue order;
  order.queue(CanProtocol::VELOCITY_ID, t0);
  order.queue(CanProtocol::ARRIVAL_ACK_ID, t0);
  order.queue(CanProtocol::TARGET_ID, t0);
  CHECK(order.drop(CanProtocol::ARRIVAL_ACK_ID) == 1 && order.size() == 2);
  drops = 0;
  front = order.front(t0, timeout, drops);
  CHECK(front && *front == CanProtocol::VELOCITY_ID);
  order.sent();
  front = order.front(t0, timeout, drops);
  CHECK(front && *front == CanProtocol::TARGET_ID && order.size() == 1);
  order.queue(CanProtocol::SAFETY_ID, t0);
  order.queue(CanProtocol::TURN_REQUEST_ID, t0);
  CHECK(order.dropStream() == 2 && order.size() == 1);
  front = order.front(t0, timeout, drops);
  CHECK(front && *front == CanProtocol::TURN_REQUEST_ID);

  order.reset();
  CHECK(order.size() == 0);
  order.sent();
  CHECK(order.size() == 0 && order.drop(CanProtocol::VELOCITY_ID) == 0);
}
