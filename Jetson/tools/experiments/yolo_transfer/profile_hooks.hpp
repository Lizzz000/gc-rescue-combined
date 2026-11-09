#pragma once

// Diagnostic-only hooks. The original inferGpu statements stay in order; CUDA
// events use its own stream and its existing final synchronization.
#include <array>
#include <chrono>
#include <cstddef>
#include <stdexcept>
#include <string>
#include <cuda_runtime_api.h>

namespace rescue_yolo_profile {
using Clock = std::chrono::steady_clock;
inline double milliseconds(Clock::duration elapsed)
{
  return std::chrono::duration<double, std::milli>(elapsed).count();
}
inline void check(cudaError_t code, const char * operation)
{
  if (code != cudaSuccess) {
    throw std::runtime_error(std::string(operation) + ": " + cudaGetErrorString(code));
  }
}
struct Trace
{
  std::array<cudaEvent_t, 5> events{};
  std::array<bool, 5> recorded{};
  Clock::time_point pack_start{}, pin_start{}, sync_start{};
  double pack_ms{}, pin_ms{}, sync_ms{};
  std::size_t h2d_bytes{}, d2h_bytes{}, d2h_copies{};
  bool graph{};
  Trace()
  {
    try {
      for (auto & event : events) {check(cudaEventCreate(&event), "cudaEventCreate");}
    } catch (...) {
      for (auto event : events) {if (event) {cudaEventDestroy(event);}}
      throw;
    }
  }
  ~Trace()
  {
    for (auto event : events) {if (event) {cudaEventDestroy(event);}}
  }
  Trace(const Trace &) = delete;
  Trace & operator=(const Trace &) = delete;
  void reset()
  {
    recorded.fill(false);
    pack_ms = pin_ms = sync_ms = 0;
    h2d_bytes = d2h_bytes = d2h_copies = 0;
    graph = false;
  }
  double elapsed(unsigned begin, unsigned end) const
  {
    if (!recorded.at(begin) || !recorded.at(end)) {
      throw std::runtime_error("missing CUDA profiling event");
    }
    float ms = 0;
    check(cudaEventElapsedTime(&ms, events.at(begin), events.at(end)),
      "cudaEventElapsedTime");
    return ms;
  }
};
inline thread_local Trace * active = nullptr;
inline void packBegin() {if (active) {active->pack_start = Clock::now();}}
inline void packEnd()
{
  if (active) {active->pack_ms = milliseconds(Clock::now() - active->pack_start);}
}
inline void pinBegin() {if (active) {active->pin_start = Clock::now();}}
inline void pinEnd(std::size_t bytes)
{
  if (active) {
    active->pin_ms = milliseconds(Clock::now() - active->pin_start);
    active->h2d_bytes = bytes;
  }
}
inline void event(unsigned index, cudaStream_t stream)
{
  if (active) {
    check(cudaEventRecord(active->events.at(index), stream), "cudaEventRecord");
    active->recorded.at(index) = true;
  }
}
inline void graph(bool enabled) {if (active) {active->graph = enabled;}}
inline void copied(std::size_t bytes)
{
  if (active) {active->d2h_bytes += bytes; ++active->d2h_copies;}
}
inline void syncBegin() {if (active) {active->sync_start = Clock::now();}}
inline void syncEnd()
{
  if (active) {active->sync_ms = milliseconds(Clock::now() - active->sync_start);}
}
}  // namespace rescue_yolo_profile
