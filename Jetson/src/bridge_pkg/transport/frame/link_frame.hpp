#pragma once

// 板间 USB 全速 CDC 分帧，12 Mbit/s。C 板没有 ULPI，不是 480 Mbit/s。
// 布局：A5、版本 01、类型 uint8、长度 uint8、载荷、CRC-16/IBM 小端。
// CRC 覆盖版本到载荷。最大载荷 8。

#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>

namespace link_frame
{

constexpr std::uint8_t kSof = 0xA5;
constexpr std::uint8_t kVersion = 0x01;
constexpr std::size_t kMaxPayload = 8;
constexpr std::size_t kMaxBytes = 6 + kMaxPayload;

struct Frame
{
  std::uint8_t type{0};
  std::uint8_t length{0};
  std::uint8_t data[kMaxPayload]{};
};

inline std::uint16_t crc16(const std::uint8_t * data, std::size_t size)
{
  // 32-byte table: two nibble steps preserve the original reflected polynomial.
  static constexpr std::uint16_t nibble[16] = {
    0x0000U, 0xCC01U, 0xD801U, 0x1400U, 0xF001U, 0x3C00U, 0x2800U, 0xE401U,
    0xA001U, 0x6C00U, 0x7800U, 0xB401U, 0x5000U, 0x9C01U, 0x8801U, 0x4400U
  };
  std::uint16_t crc = 0xFFFF;
  for (std::size_t i = 0; i < size; ++i) {
    crc = static_cast<std::uint16_t>(crc ^ data[i]);
    crc = static_cast<std::uint16_t>((crc >> 4U) ^ nibble[crc & 0x0FU]);
    crc = static_cast<std::uint16_t>((crc >> 4U) ^ nibble[crc & 0x0FU]);
  }
  return crc;
}

// 长度非法时返回 0，不写输出。
inline std::size_t encode(
  std::uint8_t type, const std::uint8_t * data, std::uint8_t length, std::uint8_t * out)
{
  if (out == nullptr || length > kMaxPayload || (length > 0 && data == nullptr)) {return 0;}
  out[0] = kSof;
  out[1] = kVersion;
  out[2] = type;
  out[3] = length;
  if (length > 0) {std::memcpy(out + 4, data, length);}
  const auto crc = crc16(out + 1, static_cast<std::size_t>(3 + length));
  out[4 + length] = static_cast<std::uint8_t>(crc);
  out[5 + length] = static_cast<std::uint8_t>(crc >> 8U);
  return static_cast<std::size_t>(6 + length);
}

// 一次喂一个字节。CRC 错则丢掉一个字节再同步，不交出半包。
class Parser
{
public:
  bool feed(std::uint8_t byte, Frame & out)
  {
    if (count_ == 0 && byte != kSof) {return false;}
    if (count_ >= buffer_.size()) {count_ = 0;}
    buffer_[count_++] = byte;
    while (count_ >= 4) {
      const auto payload = buffer_[3];
      if (buffer_[0] != kSof || buffer_[1] != kVersion || payload > kMaxPayload) {
        discard_to_sof(1);
        continue;
      }
      const std::size_t total = static_cast<std::size_t>(6 + payload);
      if (count_ < total) {return false;}
      const auto expect = crc16(buffer_.data() + 1, static_cast<std::size_t>(3 + payload));
      const auto actual = static_cast<std::uint16_t>(
        buffer_[4 + payload] | (buffer_[5 + payload] << 8U));
      if (expect != actual) {
        discard_to_sof(1);
        continue;
      }
      out.type = buffer_[2];
      out.length = payload;
      std::memset(out.data, 0, sizeof(out.data));
      if (payload > 0) {std::memcpy(out.data, buffer_.data() + 4, payload);}
      discard_to_sof(total);
      return true;
    }
    return false;
  }

private:
  void discard_to_sof(std::size_t first)
  {
    auto skip = first;
    while (skip < count_ && buffer_[skip] != kSof) {++skip;}
    count_ -= skip;
    if (count_ > 0 && skip > 0) {std::memmove(buffer_.data(), buffer_.data() + skip, count_);}
  }

  std::array<std::uint8_t, kMaxBytes> buffer_{};
  std::size_t count_{0};
};

}  // namespace link_frame
