#include "check.hpp"
#include "protocol/search_request.hpp"

void testSearchRequest()
{
  Group group("Search request identity");
  SearchRequestSession session;
  CHECK(session.onRequest(0) == SearchRequestDecision::Reject);
  CHECK(session.onRequest(100) == SearchRequestDecision::Start);
  CHECK(session.busy() && !session.result());
  CHECK(session.onRequest(100) == SearchRequestDecision::Pending);
  CHECK(session.onRequest(101) == SearchRequestDecision::Reject);
  session.complete(99, true);
  CHECK(session.busy());
  session.complete(100, true);
  CHECK(!session.busy() && session.result() == true);
  CHECK(session.onRequest(100) == SearchRequestDecision::Replay);
  CHECK(session.onRequest(99) == SearchRequestDecision::Reject);
  // A replay/error cannot rewrite the terminal outcome.
  session.complete(100, false);
  CHECK(session.result() == true);
  CHECK(session.onRequest(101) == SearchRequestDecision::Start);
  session.complete(101, false);
  CHECK(session.onRequest(101) == SearchRequestDecision::Replay && session.result() == false);
  CHECK(session.onRequest(100) == SearchRequestDecision::Reject);
  session.invalidate();
  CHECK(session.onRequest(101) == SearchRequestDecision::Replay && session.result() == false);
  CHECK(session.onRequest(102) == SearchRequestDecision::Start);
  session.invalidate();
  session.complete(102, true); // Old service response cannot restore the result.
  CHECK(session.onRequest(102) == SearchRequestDecision::Replay && session.result() == false);
  CHECK(session.onRequest(103) == SearchRequestDecision::Start);
  session.complete(103, true);
  session.invalidate();
  CHECK(session.onRequest(103) == SearchRequestDecision::Replay && session.result() == false);
  CHECK(session.onRequest(1) == SearchRequestDecision::Reject);

  SearchRequestSession wrapping;
  CHECK(wrapping.onRequest(0xffffffffU) == SearchRequestDecision::Start);
  wrapping.complete(0xffffffffU, true);
  CHECK(wrapping.onRequest(1) == SearchRequestDecision::Start);
  wrapping.complete(1, true);
  CHECK(wrapping.onRequest(0xffffffffU) == SearchRequestDecision::Reject);
  CHECK(wrapping.onRequest(0x80000001U) == SearchRequestDecision::Reject);
}
