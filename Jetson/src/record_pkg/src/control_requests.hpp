#pragma once

#include <cstdint>
#include <stdexcept>
#include <string_view>

namespace record {
// Called under RecordConsole's session mutex; accepting a request does not commit a new job.
enum class StartRequestDecision { Start, Retry };

inline StartRequestDecision check_start_request(
    std::string_view request_id, std::uint64_t generation,
    std::string_view last_request, std::uint64_t next_generation) {
  // Only the immediately preceding successful request may be retried. Compare by
  // subtraction from the current generation so an untrusted UINT64_MAX cannot wrap.
  if (next_generation > 0 && generation == next_generation - 1 && request_id == last_request)
    return StartRequestDecision::Retry;
  if (generation != next_generation)
    throw std::runtime_error("开始请求已过期，请刷新录像状态");
  return StartRequestDecision::Start;
}

inline void check_stop_request(std::string_view requested_run, std::string_view current_run) {
  if (requested_run != current_run)
    throw std::runtime_error("停止请求不属于当前录像");
}
}  // namespace record
