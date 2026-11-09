#pragma once

#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>

namespace record {
// capture_ns是Jetson采集回调的单调时间，只在同一source_id内相减；不是PC时钟。
struct Frame {
  std::string source_id;
  std::uint64_t sequence = 0, capture_ns = 0;
  int width = 0, height = 0, fps = 0;
  std::string jpeg;
};
using FramePtr = std::shared_ptr<const Frame>;
constexpr std::size_t max_jpeg_bytes = 8 * 1024 * 1024;
inline std::uint64_t monotonic_ns() {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::steady_clock::now().time_since_epoch()).count();
}
std::string frame_header(const Frame& frame);

// 解析本包的multipart协议；半包留到下一次read，异常/换源/倒序立即断开本连接。
class FrameParser {
 public:
  explicit FrameParser(std::function<void(FramePtr)> receive) : receive_(std::move(receive)) {}
  void feed(const char* data, std::size_t length);
 private:
  std::function<void(FramePtr)> receive_;
  std::string buffer_, source_;
  Frame frame_;
  std::size_t expected_ = 0;
  std::uint64_t previous_sequence_ = 0, previous_ns_ = 0;
};
}  // namespace record
