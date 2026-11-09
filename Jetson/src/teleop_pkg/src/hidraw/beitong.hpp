// ---- 本地手柄声明 ----
// 职责：北通报告解码和串口输入，不包含 CLI。
// Copyright 2026 li
// SPDX-License-Identifier: Apache-2.0
#pragma once
#include "teleop_pkg/control.hpp"
#include <string>

namespace teleop {
struct HidReport { std::array<std::uint8_t, 64> bytes{}; std::size_t size = 0; };
struct PadReading { Control requested; bool neutral = false, stop = false; };
// 只接受本次实测的20bc:5159布局。鼠标报告不续期；未知/损坏的手柄报告不能猜测映射。
std::optional<PadReading> decode_beitong(const HidReport& report);
class GamepadReports {
 public:
  virtual ~GamepadReports() = default;
  virtual std::optional<HidReport> receive() = 0;
};
// 只读拥有hidraw描述符，不写USB、不抢占内核手柄驱动；设备断开后不自动重开旧编号。
class BeitongHid final : public GamepadReports {
 public:
  explicit BeitongHid(const std::string& path);
  ~BeitongHid() override;
  BeitongHid(const BeitongHid&) = delete;
  BeitongHid& operator=(const BeitongHid&) = delete;
  std::optional<HidReport> receive() override;
 private:
  int fd_ = -1;
};
// 主循环独占。只以新收到的完整手柄报告续期；报告内容相同仍是心跳，轴不变化不代表掉线。
// 每轮最多64报告；停车/松开优先于同批后续运动。超时或循环停顿后须新报告松开并回中。
class LocalPadInput {
 public:
  LocalPadInput(GamepadReports& source, Clock clock, Stop stop, double timeout = .25);
  Control sample();
  Control current();  // 在串口提交前复核报告期限与停止信号。
  bool connected() const;
  bool ready() const { return ready_ && connected(); }
  std::uint64_t reports() const { return reports_; }
 private:
  void revoke();
  GamepadReports& source_;
  Clock clock_;
  Stop stop_;
  double timeout_;
  std::optional<double> sampled_at_, received_at_;
  Control control_;
  bool ready_ = false;
  std::uint64_t reports_ = 0;
};
}  // namespace teleop
