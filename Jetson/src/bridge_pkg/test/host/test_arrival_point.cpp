#include "check.hpp"
#include "protocol/arrival_point.hpp"

#include <vector>

namespace {

enum class WriteResult { Sent, Backpressure, Failed };

struct MockWriter
{
  WriteResult next{WriteResult::Sent};
  std::vector<can_frame> frames;
  WriteResult operator()(const can_frame & frame)
  {
    if (next == WriteResult::Sent) {frames.push_back(frame);}
    return next;
  }
};

}  // namespace

void testArrivalPoint()
{
  Group group("ArrivalPoint");

  ArrivalPointSession session;
  CHECK(session.onRequest(1) == ArrivalPointDecision::Accept);
  CHECK(session.onRequest(1) == ArrivalPointDecision::Discard);
  CHECK(session.onRequest(2) == ArrivalPointDecision::Discard);
  session.markReplied();
  CHECK(session.onRequest(1) == ArrivalPointDecision::Resend);
  session.markComplete();
  CHECK(session.onRequest(1) == ArrivalPointDecision::Resend);
  CHECK(session.onRequest(2) == ArrivalPointDecision::Accept);
  CHECK(session.kind() == 2);
  CHECK(session.onRequest(1) == ArrivalPointDecision::Discard);
  session.markReplied();
  CHECK(session.onRequest(1) == ArrivalPointDecision::Discard);
  CHECK(session.onRequest(2) == ArrivalPointDecision::Resend);
  session.markComplete();
  CHECK(session.onRequest(2) == ArrivalPointDecision::Resend);
  CHECK(session.onRequest(1) == ArrivalPointDecision::Accept);
  session.reset();
  CHECK(session.onRequest(2) == ArrivalPointDecision::Accept);

  MockWriter writer;
  std::uint8_t pause = 0;
  CHECK(writePauseHeartbeat(writer, pause) == WriteResult::Sent);
  CHECK(pause == 1 && writer.frames.size() == 1);
  CHECK(writer.frames[0].can_id == CanProtocol::VELOCITY_ID && writer.frames[0].len == 8);
  CHECK(writer.frames[0].data[0] == 1 && writer.frames[0].data[1] == 1);
  CHECK(writer.frames[0].data[2] == 0 && writer.frames[0].data[7] == 0);
  const auto reply = CanProtocol::commandResult(true, CanProtocol::ARRIVAL_POINT);
  writer.frames.push_back(reply);
  CHECK(writer.frames[0].can_id == CanProtocol::VELOCITY_ID);
  CHECK(writer.frames[1].can_id == CanProtocol::COMMAND_RESULT_ID);
  CHECK(writer.frames[1].data[1] == CanProtocol::ARRIVAL_POINT);

  MockWriter blocked;
  blocked.next = WriteResult::Backpressure;
  pause = 0;
  CHECK(writePauseHeartbeat(blocked, pause) == WriteResult::Backpressure);
  CHECK(pause == 1 && blocked.frames.empty());
}
