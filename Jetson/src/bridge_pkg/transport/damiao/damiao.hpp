#pragma once

// ---- 达妙 USB2CAN ----
// 职责：原厂串口帧的编码、设备路径和波特率。
// Copyright 2026 li
// SPDX-License-Identifier: Apache-2.0

#include <array>
#include <chrono>
#include <string>
#include <linux/can.h>

#include "transport/transport.hpp"
#include <cstddef>
#include <cstdint>
#include <sys/types.h>

// 普通款USB2CAN原厂协议；不是SLCAN，也不是达妙电机MIT业务协议。
// 依据 dmBots/usb-2-can 的帧格式及开源上位机。多字节字段显式小端，不依赖位域布局。
namespace damiao
{
using Packet = std::array<std::uint8_t, 30>;
Packet encode(const can_frame & frame);
bool validBitrate(std::int64_t bitrate);
bool validDevice(const std::string & path);

struct Event
{
  enum class Kind { Frame, Notice, Failure, Malformed };
  Kind kind{Kind::Malformed};
  can_frame frame{};
  std::uint8_t command{0};
};
// 仅在互斥回调内使用。固定缓存，无逐帧分配；保留半包，坏包逐字节寻找下一个AA帧头。
class Decoder
{
public:
  static constexpr std::size_t capacity = 256;
  std::size_t available() const {return capacity - size_;}
  void append(const std::uint8_t * data, std::size_t size);
  bool next(Event & event);
  void reset() {size_ = 0;}
private:
  std::array<std::uint8_t, capacity> bytes_{};
  std::size_t size_{0};
  void consume(std::size_t count);
};
}  // namespace damiao

// 独占一个USB tty；不启动线程。open只取得设备，check在后续轮询中有界初始化。
// interfaceUp仅表示本机会话可收发，不证明STM32在线或实际CAN码率匹配。
// V3沿用模块已有设置；不发送短控制命令，码率须在启动前与STM32核对。
class DamiaoCan final : public CanTransport
{
public:
  explicit DamiaoCan(std::int64_t expected_bitrate);
  ~DamiaoCan() override;
  bool open(const std::string & path, int & index, std::string & error) override;
  void close() override;
  bool socketOpen() const override {return fd_ >= 0;}
  bool interfaceUp() const override {return phase_ == Phase::Ready;}
  bool check(int & index, std::string & error) override;
  ReadResult read(can_frame & frame, std::string & error) override;
  WriteResult write(const can_frame & frame, std::string & error) override;
private:
  friend struct DamiaoCanTestAccess;
  using Clock = std::chrono::steady_clock;
  enum class Phase { Closed, Drain, Activate, Ready };
  ReadResult receive(damiao::Event & event, std::string & error);
  void incomplete();
  int fd_{-1}, session_{0};
  std::string path_;
  dev_t device_{0};
  ino_t inode_{0};
  Phase phase_{Phase::Closed};
  Clock::time_point deadline_{}, drain_started_{};
  damiao::Decoder decoder_;
  // 半包无法证明已被模块丢弃：本对象不再自动重开，退出后须重插设备再启动。
  bool poisoned_{false}, exclusive_{false};
};
