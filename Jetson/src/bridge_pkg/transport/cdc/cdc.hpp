#pragma once

// USB 全速 CDC（12 Mbit/s，C 板没有 ULPI）。背压不缓存帧；半帧写入属于不可重试错误。

#include "transport/frame/link_frame.hpp"
#include "transport/transport.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <string>

class CdcLink final : public CanTransport
{
public:
  static bool validDevice(const std::string & path);

  ~CdcLink() override;

  bool open(const std::string & interface_name, std::string & error) override;
  void close() override;
  bool socketOpen() const override;
  ReadResult read(can_frame & frame, std::string & error) override;
  WriteResult write(const can_frame & frame, std::string & error) override;

private:
  int fd_{-1};
  link_frame::Parser parser_;
  std::array<std::uint8_t, 256> stash_{};
  std::size_t stash_count_{0};
  std::size_t stash_pos_{0};
};
