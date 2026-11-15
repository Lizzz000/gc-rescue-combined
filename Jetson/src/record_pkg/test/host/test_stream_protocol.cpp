#include "stream_protocol.hpp"

#include <algorithm>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

namespace {
int failures = 0;
int checks = 0;
void expect(bool okay, const char* description) {
  ++checks;
  if (!okay) { ++failures; std::cerr << "FAIL: " << description << '\n'; }
}
std::string wire(const record::Frame& frame) {
  return record::frame_header(frame) + frame.jpeg + "\r\n";
}
bool same_frame(const record::Frame& actual, const record::Frame& expected) {
  return actual.source_id == expected.source_id && actual.sequence == expected.sequence &&
    actual.capture_ns == expected.capture_ns && actual.width == expected.width &&
    actual.height == expected.height && actual.fps == expected.fps && actual.jpeg == expected.jpeg;
}
void feed_chunks(record::FrameParser& parser, const std::string& bytes, std::size_t chunk_size) {
  for (std::size_t offset = 0; offset < bytes.size(); offset += chunk_size) {
    parser.feed(bytes.data() + offset, std::min(chunk_size, bytes.size() - offset));
  }
}
}  // namespace

int main() {
  const record::Frame first{"camera-A", 10, 1000000, 640, 480, 30,
    std::string{char(0xff), char(0xd8), '\0', '\r', '\n', char(0xff), char(0xd9)}};
  const record::Frame second{"camera-A", 11, 1033333, 640, 480, 30, "next JPEG"};
  // 换源和时间顺序由录像写入端检查；协议解析仍须如实交付元数据。
  const record::Frame restarted{"camera-B", 1, 10, 1280, 720, 60, "restarted JPEG"};
  const record::Frame older{"camera-B", 1, 9, 1280, 720, 60, "older JPEG"};

  std::vector<record::FramePtr> frames;
  record::FrameParser parser([&](record::FramePtr frame) { frames.push_back(std::move(frame)); });
  const auto first_wire = wire(first);
  parser.feed(first_wire.data(), first_wire.size() - 1);
  expect(frames.empty(), "partial trailing delimiter does not publish a frame");
  parser.feed(first_wire.data() + first_wire.size() - 1, 1);
  expect(frames.size() == 1, "completing a split frame publishes exactly once");
  expect(frames.size() == 1 && same_frame(*frames[0], first),
         "header round trip preserves binary payload and all metadata");
  feed_chunks(parser, wire(second), 1);
  expect(frames.size() == 2 && same_frame(*frames[1], second),
         "bytewise header and payload fragmentation reassembles the next frame");
  expect(same_frame(*frames[0], first), "later parsing leaves retained frames immutable");
  const auto batch = wire(restarted) + wire(older);
  parser.feed(batch.data(), batch.size());
  expect(frames.size() == 4, "a single read can publish multiple frames");
  expect(frames.size() == 4 && same_frame(*frames[2], restarted) && same_frame(*frames[3], older),
         "source and timestamp changes reach the downstream recording checks unchanged");

  bool published_bad = false;
  record::FrameParser malformed([&](record::FramePtr) { published_bad = true; });
  auto bad_wire = wire(first);
  bad_wire.back() = '!';
  bool rejected = false;
  try { malformed.feed(bad_wire.data(), bad_wire.size()); }
  catch (const std::runtime_error&) { rejected = true; }
  expect(rejected, "a malformed trailing delimiter is rejected");
  expect(!published_bad, "a rejected frame is not delivered");

  std::cout << checks << " checks, " << failures << " failures\n";
  return failures ? 1 : 0;
}
