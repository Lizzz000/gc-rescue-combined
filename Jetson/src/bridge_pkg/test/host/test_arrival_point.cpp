#include "check.hpp"
#include "protocol/arrival_point.hpp"
#include "protocol/usb_protocol.hpp"
#include "transport/transport.hpp"

namespace {

bool resultIs(const std::optional<GripperResult> & result,
  std::uint8_t kind, std::uint32_t request_id, bool success)
{
  return result && result->kind == kind && result->request_id == request_id &&
         result->success == success;
}

}  // namespace

void testArrivalPoint()
{
  Group group("Gripper action identity and replay");
  using Decision = ArrivalPointDecision;

  // 启动完成早于首查：结果仍在，查询不能重启电机。失败也同样保存。
  ArrivalPointSession completed;
  CHECK(completed.busy());
  CHECK(!completed.complete(true));
  CHECK(!completed.busy());
  CHECK(completed.onRequest(0, 10) == Decision::Resend);
  CHECK(resultIs(completed.result(0, 10), 0, 10, true));
  CHECK(completed.onRequest(0, 10) == Decision::Resend);
  CHECK(!completed.busy());
  CHECK(completed.onRequest(0, 11) == Decision::Resend);
  CHECK(resultIs(completed.result(0, 11), 0, 11, true));
  CHECK(completed.onRequest(0, 10) == Decision::Reject);

  ArrivalPointSession failed_startup;
  failed_startup.complete(false);
  CHECK(failed_startup.onRequest(0, 1) == Decision::Resend);
  CHECK(resultIs(failed_startup.result(0, 1), 0, 1, false));

  // 首查先于完成：先 ACK，无终态；相同请求只重发 ACK，完成时关联原编号。
  ArrivalPointSession session;
  CHECK(session.onRequest(0, 1) == Decision::Resend);
  CHECK(!session.result(0, 1));
  CHECK(session.onRequest(0, 1) == Decision::Resend);
  CHECK(session.onRequest(1, 1) == Decision::Reject);
  CHECK(resultIs(session.complete(true), 0, 1, true));
  CHECK(!session.complete(false));
  CHECK(resultIs(session.result(0, 1), 0, 1, true));

  CHECK(session.onRequest(1, 1) == Decision::Reject);
  CHECK(session.onRequest(1, 2) == Decision::Accept);
  CHECK(session.busy());
  CHECK(session.onRequest(1, 2) == Decision::Resend);
  CHECK(!session.result(1, 2));
  CHECK(session.onRequest(2, 3) == Decision::Reject);
  CHECK(session.onRequest(1, 3) == Decision::Reject);
  CHECK(resultIs(session.complete(true), 1, 2, true));
  CHECK(session.onRequest(1, 2, false) == Decision::Resend);

  // 释放不覆盖夹取结果。旧夹取查询绝不重新执行，即使另一 kind 正在动作。
  CHECK(session.onRequest(2, 3) == Decision::Accept);
  CHECK(session.onRequest(1, 2) == Decision::Resend);
  CHECK(resultIs(session.result(1, 2), 1, 2, true));
  CHECK(resultIs(session.complete(false), 2, 3, false));
  CHECK(session.onRequest(2, 3) == Decision::Resend);
  CHECK(resultIs(session.result(2, 3), 2, 3, false));
  CHECK(!session.busy());

  // 明确的新搜索仅撤旧拒绝授权，不抹掉已处理编号或启动结果。
  session.authorizeRejectOpen();
  session.beginSearch();
  CHECK(session.onRequest(3, 4) == Decision::Reject);
  CHECK(session.onRequest(1, 2) == Decision::Resend);
  CHECK(resultIs(session.result(0, 1), 0, 1, true));
  CHECK(session.onRequest(1, 5) == Decision::Accept);
  CHECK(session.onRequest(1, 2) == Decision::Reject);
  session.beginSearch();
  CHECK(session.busy());
  CHECK(resultIs(session.complete(true), 1, 5, true));
  CHECK(session.onRequest(1, 2) == Decision::Reject);
  CHECK(session.onRequest(1, 4) == Decision::Reject);

  // 拒绝张开只在授权后的首次请求执行；重复 ACK、失败/成功终态均不再次开爪。
  CHECK(session.onRequest(3, 6) == Decision::Reject);
  session.authorizeRejectOpen();
  CHECK(session.onRequest(3, 6) == Decision::Accept);
  CHECK(session.onRequest(3, 6) == Decision::Resend);
  CHECK(!session.result(3, 6));
  CHECK(resultIs(session.complete(true), 3, 6, true));
  CHECK(session.onRequest(3, 6) == Decision::Resend);
  CHECK(session.onRequest(3, 7) == Decision::Reject);

  // 单槽结果被其他 kind 覆盖或 USB 清队后，按原编号查询可重建相同结果。
  BridgeReplySlots replies;
  CanTxQueue queue;
  const auto now = CanTxQueue::Clock::time_point{};
  auto first = *session.result(1, 5);
  replies.store(CanProtocol::GRIPPER_RESULT_ID,
    CanProtocol::gripperResult(first.kind, first.success, first.request_id));
  queue.queue(CanProtocol::GRIPPER_RESULT_ID, now);
  auto other = *session.result(3, 6);
  replies.store(CanProtocol::GRIPPER_RESULT_ID,
    CanProtocol::gripperResult(other.kind, other.success, other.request_id));
  queue.reset();
  CHECK(session.onRequest(1, 5) == Decision::Resend);
  first = *session.result(1, 5);
  replies.store(CanProtocol::GRIPPER_RESULT_ID,
    CanProtocol::gripperResult(first.kind, first.success, first.request_id));
  queue.queue(CanProtocol::GRIPPER_RESULT_ID, now);
  CHECK(replies.frame(CanProtocol::GRIPPER_RESULT_ID).data[0] == 1);
  CHECK(replies.frame(CanProtocol::GRIPPER_RESULT_ID).data[2] == 5);
  std::uint64_t drops = 0;
  CHECK(!queue.front(now + std::chrono::milliseconds(20), std::chrono::milliseconds(20), drops));
  CHECK(drops == 1 && resultIs(session.result(1, 5), 1, 5, true));
  CHECK(!session.busy());

  // 非法编号不改变状态；uint32 串号正常绕回，旧的最大编号不能再次动作。
  CHECK(session.onRequest(4, 7) == Decision::Reject);
  CHECK(session.onRequest(1, 0) == Decision::Reject);
  CHECK(!session.result(4, 1));
  ArrivalPointSession wrapped;
  wrapped.complete(true);
  CHECK(wrapped.onRequest(1, 0xffffffffu) == Decision::Accept);
  wrapped.complete(true);
  CHECK(wrapped.onRequest(1, 1) == Decision::Accept);
  wrapped.complete(false);
  CHECK(wrapped.onRequest(1, 0xffffffffu) == Decision::Reject);
  CHECK(resultIs(wrapped.result(1, 1), 1, 1, false));
}
