#include "stream_protocol.hpp"

#include <charconv>
#include <map>
#include <sstream>
#include <stdexcept>

namespace record {
std::string frame_header(const Frame& f) {
  return "--gc-frame\r\nContent-Type: image/jpeg\r\nContent-Length: " + std::to_string(f.jpeg.size()) +
      "\r\nX-Source-Id: " + f.source_id + "\r\nX-Sequence: " + std::to_string(f.sequence) +
      "\r\nX-Capture-Ns: " + std::to_string(f.capture_ns) + "\r\nX-Width: " + std::to_string(f.width) +
      "\r\nX-Height: " + std::to_string(f.height) + "\r\nX-Fps: " + std::to_string(f.fps) + "\r\n\r\n";
}
void FrameParser::feed(const char* data, std::size_t length) {
  if (length > max_jpeg_bytes || buffer_.size() + length > max_jpeg_bytes * 2 + 4096)
    throw std::runtime_error("frame receive buffer exceeded its limit");
  buffer_.append(data, length);
  for (;;) {
    if (!expected_) {
      const auto end = buffer_.find("\r\n\r\n");
      if (end == std::string::npos) {
        if (buffer_.size() > 2048) throw std::runtime_error("frame header too large");
        return;
      }
      if (end > 2048 || buffer_.rfind("--gc-frame\r\n", 0) != 0)
        throw std::runtime_error("unsupported camera stream framing");
      std::istringstream input(buffer_.substr(12, end - 12));
      std::map<std::string, std::string> fields;
      std::string line;
      while (std::getline(input, line)) {
        if (!line.empty() && line.back() == '\r') line.pop_back();
        const auto colon = line.find(": ");
        if (colon == std::string::npos) throw std::runtime_error("invalid or duplicate frame header");
        fields.emplace(line.substr(0, colon), line.substr(colon + 2));
      }
      auto number = [&](const char* key, std::uint64_t maximum) {
        const auto& text = fields.at(key);
        std::uint64_t value = 0;
        auto parsed = std::from_chars(text.data(), text.data() + text.size(), value);
        if (parsed.ec != std::errc{} || parsed.ptr != text.data() + text.size() || !value || value > maximum)
          throw std::runtime_error(std::string("invalid camera header: ") + key);
        return value;
      };
      frame_ = {};
      frame_.source_id = fields.at("X-Source-Id");
      frame_.sequence = number("X-Sequence", UINT64_MAX);
      frame_.capture_ns = number("X-Capture-Ns", INT64_MAX);
      frame_.width = static_cast<int>(number("X-Width", 8192));
      frame_.height = static_cast<int>(number("X-Height", 8192));
      frame_.fps = static_cast<int>(number("X-Fps", 240));
      // 换源与时间单调只在 Mp4Writer::write 检查一次。
      expected_ = number("Content-Length", max_jpeg_bytes);
      buffer_.erase(0, end + 4);
    }
    if (buffer_.size() < expected_ + 2) return;
    // 帧边界只查结尾 \r\n；JPEG 完整性交给 FFmpeg 解码器。
    if (buffer_.compare(expected_, 2, "\r\n"))
      throw std::runtime_error("incomplete camera JPEG");
    frame_.jpeg = buffer_.substr(0, expected_);
    buffer_.erase(0, expected_ + 2);
    source_ = frame_.source_id;
    previous_sequence_ = frame_.sequence;
    previous_ns_ = frame_.capture_ns;
    expected_ = 0;
    receive_(std::make_shared<const Frame>(std::move(frame_)));
  }
}
}  // namespace record
