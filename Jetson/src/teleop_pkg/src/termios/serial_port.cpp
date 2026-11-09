// Copyright 2026 li
// SPDX-License-Identifier: Apache-2.0
#include "termios/serial_port.hpp"
#include "teleop_pkg/control.hpp"
#include <memory>
#include <optional>
#include <algorithm>
#include <cerrno>
#include <cmath>
#include <cstring>
#include <fcntl.h>
#include <iostream>
#include <poll.h>
#include <stdexcept>
#include <sys/file.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>

namespace teleop {
namespace {
/**
 * @brief 将当前系统调用错误转为异常，交给串口所有者统一清理。
 * @param[in] operation 非空操作名字符串，不得为空指针。
 */
void serial_fail(const char* operation) {
  throw std::runtime_error(std::string(operation) + ": " + std::strerror(errno));
}
}  // namespace
/**
 * @brief 在 CLI 入口一次把支持的波特率转换为 termios 常量。
 * @param[in] baud 串口 bit/s，仅接受 switch 中列出的 9600..921600 档位。
 * @return 对应 termios 速率常量；不支持时抛异常，不打开设备。
 */
speed_t speed(int baud) {
  switch (baud) {
    case 9600: return B9600; case 19200: return B19200; case 38400: return B38400;
    case 57600: return B57600; case 115200: return B115200; case 230400: return B230400;
    case 460800: return B460800; case 921600: return B921600;
    default: throw std::invalid_argument("unsupported baud rate");
  }
}
SerialPort::SerialPort(const std::string& path, speed_t usb_baud, const std::string& adapter) : damiao_(adapter != "raw") {
  fd_ = ::open(path.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
  if (fd_ < 0) serial_fail("open serial");
  try {
    if (flock(fd_, LOCK_EX | LOCK_NB)) serial_fail("serial is already in use");
    if (damiao_) {
      // 阻止后续非合作打开；此前已打开的fd仍依赖flock协作，不能因此省略它。
      if (ioctl(fd_, TIOCEXCL)) serial_fail("acquire Damiao serial exclusive access");
      exclusive_ = true;
      // 前任的CAN半包不能先丢输出再接新命令；配置/清缓冲/发送前先拒绝残留。
      int queued = 0;
      if (ioctl(fd_, TIOCOUTQ, &queued)) serial_fail("query inherited Damiao output");
      if (queued != 0)
        throw std::runtime_error("inherited Damiao output pending; reconnect adapter before restart");
    }
    termios settings{};
    if (tcgetattr(fd_, &settings)) serial_fail("tcgetattr");
    cfmakeraw(&settings);
    settings.c_cflag = (settings.c_cflag & ~(CSIZE | PARENB | CSTOPB | CRTSCTS)) | CS8 | CLOCAL | CREAD;
    settings.c_iflag &= ~(IXON | IXOFF | IXANY);
    settings.c_cc[VMIN] = 0; settings.c_cc[VTIME] = 0;
    if (cfsetispeed(&settings, usb_baud) || cfsetospeed(&settings, usb_baud) ||
        tcsetattr(fd_, TCSANOW, &settings) || tcflush(fd_, TCIFLUSH))
      serial_fail("configure serial 8N1");
    // 实测V3直接采用CDC设置，不发送未经确认的模块命令，也不自动回退旧协议。
    if (damiao_) drain_on_close_ = startup_cleared_ = true;
  } catch (...) { close(); throw; }
}
SerialPort::~SerialPort() { close(); }
int SerialPort::waiting() {
  if (damiao_) discard_replies();
  int bytes = 0;
  if (ioctl(fd_, TIOCOUTQ, &bytes)) serial_fail("query serial output");
  return bytes;
}
std::size_t SerialPort::write(const SerialFrame& bytes) {
  if (output_blocked_)
    throw std::runtime_error("Damiao output state unknown; reset/reconnect adapter before restarting or switching modes");
  // 仅由 SerialOutput 提交已编码的速度行或固定同步标记，无外部裸长度输入。
  return write_bytes(bytes.data(), bytes.size(), bytes.moving);
}
void SerialPort::clear() {
  // 达妙接管时已确认输出为空；首次不清输出，后续仅在收尾时撤销待发速度。
  if (startup_cleared_) { startup_cleared_ = false; return; }
  if (tcflush(fd_, TCOFLUSH)) {
    // 清理失败后旧输出边界未知；SerialOutput仍会尝试收尾，本设备必须拒绝追加文本。
    output_blocked_ = damiao_;
    serial_fail("clear serial output");
  }
}
void SerialPort::close() noexcept {
  if (fd_ < 0) return;
  if (drain_on_close_ && !output_blocked_) {
    // 持锁等最终同步/零速离开驱动队列，再允许CAN接管；停止信号不能跳过收尾。
    // 仍不证明模块/STM32已接收，超时或错误须重插，不能靠重新打开清模块半包。
    try { drain_output(); }
    catch (const std::exception& e) {
      std::cerr << "WARNING Damiao final output: " << e.what()
                << "; reset/reconnect adapter before restarting or switching modes\n";
    }
  }
  // 只撤销本对象成功设置的独占标志；构造失败和重复close也遵守同一所有权。
  if (exclusive_) {
    if (ioctl(fd_, TIOCNXCL))
      std::cerr << "WARNING release Damiao serial exclusive access: " << std::strerror(errno) << '\n';
    exclusive_ = false;
  }
  ::close(fd_); fd_ = -1;
}
std::size_t SerialPort::write_bytes(const std::uint8_t* data, std::size_t size, bool cancel_on_stop) {
  const double deadline = monotonic() + .02;
  for (;;) {
    if (Signals::stopped() && cancel_on_stop) return 0;
    const double remaining = deadline - monotonic();
    if (remaining <= 0) throw std::runtime_error("serial write timeout");
    pollfd poller{fd_, POLLOUT, 0};
    const int ready = poll(&poller, 1, static_cast<int>(std::ceil(remaining * 1000)));
    if (ready < 0) { if (errno == EINTR) continue; serial_fail("poll serial"); }
    if (!ready) throw std::runtime_error("serial write timeout");
    if (poller.revents & (POLLERR | POLLHUP | POLLNVAL))
      throw std::runtime_error("serial disconnected");
    if (Signals::stopped() && cancel_on_stop) return 0;
    const auto count = ::write(fd_, data, size);
    if (count < 0) { if (errno == EINTR || errno == EAGAIN) continue; serial_fail("write serial"); }
    return static_cast<std::size_t>(count);
  }
}
void SerialPort::drain_output() {
  const double deadline = monotonic() + .02;
  for (;;) {
    int queued = 0;
    if (ioctl(fd_, TIOCOUTQ, &queued)) serial_fail("query Damiao output");
    if (!queued) return;
    if (monotonic() >= deadline) throw std::runtime_error("Damiao output timeout; reset/reconnect adapter before restart");
    poll(nullptr, 0, 1);
  }
}
void SerialPort::discard_replies() {
  // USB承载模块通知和外部UART回包；均不作为输入/ACK。每周期至多8次、4096字节，
  // EINTR也消耗预算，避免持续回包拖延输入期限或停止；无数据时立即返回。
  pollfd poller{fd_, POLLIN, 0};
  if (poll(&poller, 1, 0) < 0) { if (errno == EINTR) return; serial_fail("poll Damiao input"); }
  if (poller.revents & (POLLERR | POLLHUP | POLLNVAL))
    throw std::runtime_error("Damiao serial disconnected");
  std::uint8_t bytes[512];
  for (unsigned i = 0; i < 8 && !Signals::stopped(); ++i) {
    const auto count = ::read(fd_, bytes, sizeof(bytes));
    if (count > 0) continue;
    if (!count || errno == EAGAIN || errno == EWOULDBLOCK) return;
    if (errno != EINTR) serial_fail("read Damiao input");
  }
}
}  // namespace teleop
