#include "control_requests.hpp"

#include <cstdint>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>

namespace {
int failures = 0;
int checks = 0;
void expect(bool okay, const char* description) {
  ++checks;
  if (!okay) { ++failures; std::cerr << "FAIL: " << description << '\n'; }
}
template<class Action>
void rejected(Action action, const char* description) {
  bool refused = false;
  try { action(); } catch (const std::runtime_error&) { refused = true; }
  expect(refused, description);
}
}  // namespace

int main() {
  using record::check_start_request;
  using record::check_stop_request;
  using record::StartRequestDecision;

  expect(check_start_request("request-A", 0, "", 0) == StartRequestDecision::Start,
         "first request starts a recording");
  expect(check_start_request("request-A", 0, "request-A", 1) == StartRequestDecision::Retry,
         "lost start response can be retried without starting another recording");
  rejected([] { check_start_request("request-B", 0, "request-A", 1); },
           "another page cannot reuse the generation consumed by recording A");

  // A completed, then B started: A's delayed stop must not stop the live recording B.
  rejected([] { check_stop_request("run-A", "run-B"); },
           "a delayed stop for A cannot stop B");
  rejected([] { check_stop_request("", "run-B"); },
           "a missing recording identity cannot stop B");
  try {
    check_stop_request("run-B", "run-B");
    check_stop_request("run-B", "run-B");
    expect(true, "current recording accepts its stop and an identical retry");
  } catch (...) { expect(false, "current recording accepts its stop and an identical retry"); }

  // B completed and the console is idle. Replaying A must not create a third recording.
  rejected([] { check_start_request("request-A", 0, "request-B", 2); },
           "an older start cannot be replayed after a later recording completed");
  expect(check_start_request("request-B", 1, "request-B", 2) == StartRequestDecision::Retry,
         "the last successful start remains idempotent after completion");
  expect(check_start_request("request-C", 2, "request-B", 2) == StartRequestDecision::Start,
         "a fresh request can start the next recording");
  rejected([] { check_start_request("request-C", 3, "request-B", 2); },
           "a future generation is refused");
  rejected([] { check_start_request("request-B", 0, "request-B", 2); },
           "a matching request ID alone does not authorize an old generation");
  rejected([] {
    check_start_request("", std::numeric_limits<std::uint64_t>::max(), "", 0);
  }, "unsigned wraparound cannot turn an invalid request into a retry");

  std::cout << checks << " checks, " << failures << " failures\n";
  return failures ? 1 : 0;
}
