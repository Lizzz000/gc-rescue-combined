#pragma once

// USB 全速 CDC（12 Mbit/s，C 板没有 ULPI）。读到一帧完整分帧才返回 FRAME，短写和 EAGAIN 是背压。

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

  bool open(const std::string & interface_name, int & index, std::string & error) override;
  void close() override;
  bool socketOpen() const override;
  bool check(int & index, std::string & error) override;
  bool interfaceUp() const override;
  ReadResult read(can_frame & frame, std::string & error) override;
  WriteResult write(const can_frame & frame, std::string & error) override;

private:
  WriteResult flush(std::string & error);

  int fd_{-1};
  int index_{0};
  bool up_{false};
  link_frame::Parser parser_;
  std::array<std::uint8_t, 256> stash_{};
  std::size_t stash_count_{0};
  std::size_t stash_pos_{0};
  std::array<std::uint8_t, link_frame::kMaxBytes> pending_{};
  std::size_t pending_size_{0};
  std::size_t pending_sent_{0};
};
