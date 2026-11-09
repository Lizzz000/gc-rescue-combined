// ---- 本地手柄实现 ----
// 职责：串口收发和板端进程入口。
// Copyright 2026 li
// SPDX-License-Identifier: Apache-2.0
#include "hidraw/beitong.hpp"
#include <algorithm>
#include <cerrno>
#include <cmath>
#include <cstring>
#include <fcntl.h>
#include <linux/hidraw.h>
#include <linux/input.h>
#include <poll.h>
#include <stdexcept>
#include <sys/ioctl.h>
#include <unistd.h>

namespace teleop {
namespace {
void fail(const char* operation) {
  throw std::runtime_error(std::string(operation) + ": " + std::strerror(errno));
}
}
// HID报告合同：仅接受既有20bc:5159布局（VID/PID 校验在打开设备时一次完成）。
std::optional<PadReading> decode_beitong(const HidReport& r) {
  if (r.size == 6 && r.bytes[0] == 4) return std::nullopt;
  if (r.size != 11 || r.bytes[0] != 3 || !(r.bytes[2] <= 7 || r.bytes[2] == 15))
    throw std::runtime_error("invalid BEITONG gamepad report; manual control stopped");
  const unsigned buttons = unsigned(r.bytes[3]) | unsigned(r.bytes[4]) << 8;
  const auto pressed = [&](unsigned index) { return (buttons & (1u << index)) != 0; };
  // 原始轴中心128，换算为无单位输入；gamepad执行死区与整数映射，再由串口codec换速度。
  const double forward = (128 - int(r.bytes[6])) / 127.0;
  const double left = (128 - int(r.bytes[5])) / 127.0;
  const double yaw = (128 - int(r.bytes[7])) / 127.0;
  // 右Y仅参与回中；报告字节9/10为扳机，扳机、十字键及A/B均不参与回中。
  PadReading result;
  result.requested = gamepad(forward, left, yaw, pressed(6));
  result.neutral = !axis(forward) && !axis(left) && !axis(yaw) &&
                   !axis((128 - int(r.bytes[8])) / 127.0);
  result.stop = pressed(14);  // 右摇杆按下；不复用A/B或使能按钮。
  return result;
}
// HID设备生命周期：自持只读fd，打开时校验一次USB身份；断开后不自动重开。
BeitongHid::BeitongHid(const std::string& path) {
  fd_ = ::open(path.c_str(), O_RDONLY | O_NONBLOCK | O_CLOEXEC);
  if (fd_ < 0) fail("open gamepad hidraw (see teleop_pkg udev rule)");
  try {
    hidraw_devinfo info{};
    if (ioctl(fd_, HIDIOCGRAWINFO, &info)) fail("query gamepad identity");
    if (info.bustype != BUS_USB || info.vendor != 0x20bc || info.product != 0x5159)
      throw std::runtime_error("gamepad must be BEITONG 20bc:5159; refusing unknown mapping");
  } catch (...) { ::close(fd_); fd_ = -1; throw; }
}
BeitongHid::~BeitongHid() { if (fd_ >= 0) ::close(fd_); }
std::optional<HidReport> BeitongHid::receive() {
  pollfd p{fd_, POLLIN, 0};
  const int ready = poll(&p, 1, 0);
  if (ready < 0) { if (errno == EINTR) return std::nullopt; fail("poll gamepad"); }
  if (p.revents & (POLLHUP | POLLERR | POLLNVAL))
    throw std::runtime_error("gamepad disconnected; restart and re-arm");
  if (!(p.revents & POLLIN)) return std::nullopt;
  HidReport result;
  const auto size = ::read(fd_, result.bytes.data(), result.bytes.size());
  if (size < 0) {
    if (errno == EINTR || errno == EAGAIN) return std::nullopt;
    fail("read gamepad");
  }
  if (!size) throw std::runtime_error("gamepad disconnected; restart and re-arm");
  result.size = static_cast<std::size_t>(size); return result;
}
// 输入租期与重新使能：借用source，仅主循环访问；时刻和timeout均为单调秒。
// 只有完整控制报告续期，鼠标报告不续期；失效后须松LB且双摇杆回中，再按LB。
LocalPadInput::LocalPadInput(GamepadReports& source, Clock clock, Stop stop, double timeout)
    : source_(source), clock_(std::move(clock)), stop_(std::move(stop)), timeout_(timeout) {
}
void LocalPadInput::revoke() { ready_ = false; control_ = {}; }
bool LocalPadInput::connected() const { return received_at_ && clock_() - *received_at_ < timeout_; }
Control LocalPadInput::current() {
  if (!connected() || stop_()) revoke();
  return control_;
}
Control LocalPadInput::sample() {
  try {
    const auto now = clock_();
    const bool stalled = sampled_at_ && now - *sampled_at_ >= timeout_;
    sampled_at_ = now;
    current();
    if (stalled) { revoke(); received_at_.reset(); }
    // 每轮最多处理64份报告，余下留到下一轮；同批松LB/R3优先于后续运动。
    bool stop_seen = false;
    for (unsigned i = 0; i < 64; ++i) {
      const auto report = source_.receive();
      if (stop_()) { revoke(); return {}; }
      if (!report) break;
      const auto input = decode_beitong(*report);
      if (!input || stalled) continue;  // 停顿后丢弃整批旧队列，下一轮新报告才可重新就绪。
      current(); received_at_ = clock_(); ++reports_;
      if (input->stop) { revoke(); stop_seen = true; }
      else if (!input->requested.enabled) {
        control_ = {}; ready_ = input->neutral; stop_seen = true;
      } else control_ = ready_ && !stop_seen ? input->requested : Control{};
    }
    return current();
  } catch (...) { revoke(); received_at_.reset(); throw; }
}
}  // namespace teleop
