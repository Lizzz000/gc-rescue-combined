#include "transport/damiao/damiao.hpp"

// ---- 达妙传输实现 ----
// 职责：USB 串口上的达妙帧收发。
// Copyright 2026 li
// SPDX-License-Identifier: Apache-2.0
#include <algorithm>
#include <cerrno>
#include <cstring>
#include <cstdio>
#include <fcntl.h>
#include <limits>
#include <poll.h>
#include <sys/file.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <termios.h>
#include <unistd.h>

namespace
{
constexpr auto startup_timeout = std::chrono::seconds(1);
constexpr auto drain_window = std::chrono::milliseconds(50);
std::uint32_t little32(const std::uint8_t * p)
{
  return std::uint32_t(p[0]) | (std::uint32_t(p[1]) << 8) |
         (std::uint32_t(p[2]) << 16) | (std::uint32_t(p[3]) << 24);
}
// VMIN=0 的空闲读数与刚发生的挂断都表现为无数据，只有 revents 能区分；
// EINTR 是本次轮询被信号打断，不代表设备消失，调用方按自己的预算处理。
enum class Hangup {No, Interrupted, Yes};
Hangup hangup(int fd)
{
  pollfd descriptor{fd, POLLIN, 0};
  const int result = ::poll(&descriptor, 1, 0);
  if (result < 0) {return errno == EINTR ? Hangup::Interrupted : Hangup::Yes;}
  return (descriptor.revents & (POLLHUP | POLLERR | POLLNVAL)) != 0 ? Hangup::Yes : Hangup::No;
}
}  // namespace

namespace damiao
{
Packet encode(const can_frame & frame)
{
  if (frame.can_id > CAN_SFF_MASK || frame.len > CAN_MAX_DLEN) {
    throw std::invalid_argument("Damiao requires a standard CAN data frame with DLC <= 8");
  }
  Packet bytes{};
  bytes[0] = 0x55; bytes[1] = 0xaa; bytes[2] = 30; bytes[3] = 1;
  bytes[4] = 1;  // 只发一次；不用模块周期发送，否则断开主机后仍可能重复旧运动指令。
  bytes[8] = 10;  // 协议单位0.1ms；单次发送无重复间隔。
  for (unsigned i = 0; i < 4; ++i) {
    bytes[13 + i] = static_cast<std::uint8_t>(frame.can_id >> (8 * i));
  }
  bytes[18] = frame.len;
  std::copy_n(frame.data, frame.len, bytes.begin() + 21);
  // ID/帧类型、递增标志均为0。原厂表明确CRC暂未校验；不宣称USB封装有端到端CRC保护。
  return bytes;
}

bool validBitrate(std::int64_t bitrate)
{
  constexpr std::array<std::int64_t, 15> values{{
    1000000, 800000, 666000, 500000, 400000, 250000, 200000, 125000,
    100000, 80000, 50000, 40000, 20000, 10000, 5000}};
  return std::find(values.begin(), values.end(), bitrate) != values.end();
}

bool validDevice(const std::string & path)
{
  if (path.size() <= 5 || path.compare(0, 5, "/dev/") != 0 ||
    std::any_of(path.begin(), path.end(), [](unsigned char c) {return c < 0x21 || c > 0x7e;}))
  {return false;}
  bool child = false;
  for (std::size_t begin = 5; begin < path.size();) {
    const auto end = path.find('/', begin);
    const auto part = path.substr(begin, end == std::string::npos ? end : end - begin);
    if (part == "..") {return false;}
    if (!part.empty() && part != ".") {child = true;}
    if (end == std::string::npos) {break;}
    begin = end + 1;
  }
  return child;
}

void Decoder::append(const std::uint8_t * data, std::size_t size)
{
  if (size > available()) {throw std::length_error("Damiao receive buffer overflow");}
  std::copy_n(data, size, bytes_.begin() + size_);
  size_ += size;
}

void Decoder::consume(std::size_t count)
{
  size_ -= count;
  std::memmove(bytes_.data(), bytes_.data() + count, size_);
}

bool Decoder::next(Event & event)
{
  if (size_ == 0) {return false;}
  event = {};
  if (bytes_[0] != 0xaa) {
    const auto next = std::find(bytes_.begin(), bytes_.begin() + size_, 0xaa);
    consume(static_cast<std::size_t>(next - bytes_.begin()));
    return true;
  }
  if (size_ < 16) {return false;}
  const auto command = bytes_[1];
  const bool known = command == 0 || command == 1 || command == 2 ||
    command == 0x11 || command == 0x12 || command == 0xee;
  if (bytes_[15] != 0x55 || !known) {consume(1); return true;}
  event.command = command;
  if (command == 0x11) {
    const auto id = little32(bytes_.data() + 3);
    // 本项目只接标准数据帧。flags的高两位为IDE/RTR，其余六位为DLC。
    if (bytes_[2] <= 8 && id <= CAN_SFF_MASK) {
      event.kind = Event::Kind::Frame;
      event.frame.can_id = id;
      event.frame.len = bytes_[2];
      std::copy_n(bytes_.data() + 7, event.frame.len, event.frame.data);
    }
  } else if (command == 0 || command == 0x12 || (command == 0xee && bytes_[2] == 0)) {
    event.kind = Event::Kind::Notice;
  } else {
    event.kind = Event::Kind::Failure;
  }
  consume(16);
  return true;
}
}  // namespace damiao

DamiaoCan::DamiaoCan(std::int64_t /*expected_bitrate*/) {}

DamiaoCan::~DamiaoCan() {close();}

bool DamiaoCan::open(const std::string & path, int & index, std::string & error)
{
  close();
  error.clear(); index = 0;
  if (poisoned_) {
    throw FatalError("Damiao stream may contain an incomplete packet; unplug/replug the adapter before restarting");
  }
  fd_ = ::open(path.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
  if (fd_ < 0) {
    if (errno == EBUSY) {throw FatalError("Damiao serial device is already in use");}
    error = std::strerror(errno); return false;
  }
  // 与teleop使用同一个内核设备锁，路径别名也不能绕过；不等待另一控制程序退出。
  if (::flock(fd_, LOCK_EX | LOCK_NB) < 0) {
    const auto reason = errno;
    ::close(fd_); fd_ = -1;
    if (reason == EWOULDBLOCK || reason == EAGAIN) {
      throw FatalError("Damiao serial device is already in use; stop its owner before switching modes");
    }
    error = std::strerror(reason); return false;
  }
  if (::ioctl(fd_, TIOCEXCL) < 0) {error = std::strerror(errno); close(); return false;}
  exclusive_ = true;
  int queued = 0;
  if (::ioctl(fd_, TIOCOUTQ, &queued) < 0) {
    error = std::strerror(errno);
    poisoned_ = true; close();
    throw FatalError("cannot verify inherited Damiao output: " + error +
      "; unplug/replug before restarting");
  }
  if (queued != 0) {
    poisoned_ = true; close();
    throw FatalError("Damiao has residual output from a previous owner; unplug/replug before restarting");
  }
  termios settings{};
  struct stat identity{};
  if (::tcgetattr(fd_, &settings) < 0 || ::fstat(fd_, &identity) < 0 ||
    !S_ISCHR(identity.st_mode))
  {
    error = "Damiao device must be a tty"; close(); return false;
  }
  ::cfmakeraw(&settings);
  settings.c_iflag &= ~(IXON | IXOFF | IXANY);  // 关闭继承的软件流控，避免驱动插入XON/XOFF字节。
  settings.c_cflag = (settings.c_cflag & ~(CSIZE | PARENB | CSTOPB | CRTSCTS)) | CS8 | CLOCAL | CREAD;
  settings.c_cc[VMIN] = 0; settings.c_cc[VTIME] = 0;
  if (::cfsetispeed(&settings, B921600) < 0 || ::cfsetospeed(&settings, B921600) < 0 ||
    ::tcsetattr(fd_, TCSANOW, &settings) < 0 || ::tcflush(fd_, TCIFLUSH) < 0)
  {
    error = std::strerror(errno); close(); return false;
  }
  path_ = path; device_ = identity.st_rdev; inode_ = identity.st_ino;
  // serial会话标识不冒充Linux网络接口索引；check另查设备身份和挂断。
  session_ = session_ == std::numeric_limits<int>::max() ? 1 : session_ + 1;
  index = session_;
  decoder_.reset();
  phase_ = Phase::Drain;
  drain_started_ = Clock::now();
  deadline_ = drain_started_ + startup_timeout;
  return true;
}

void DamiaoCan::close()
{
  const bool was_poisoned = poisoned_;
  if (fd_ >= 0) {
    // 不追加控制字节，也不丢弃待发数据；队列非空或无法查询时，禁止自动重开。
    // 本机队列为空不证明模块内部没有残留，异常后仍须物理重插再切换模式。
    if (phase_ != Phase::Closed && !poisoned_) {
      int queued = 0;
      poisoned_ = ::ioctl(fd_, TIOCOUTQ, &queued) < 0 || queued != 0;
    }
    if (exclusive_) {::ioctl(fd_, TIOCNXCL); exclusive_ = false;}
    ::close(fd_); fd_ = -1;
  }
  phase_ = Phase::Closed;
  decoder_.reset();
  if (poisoned_ && !was_poisoned) {
    // 析构也可能发现未完成输出，不能抛异常，更不能静默让用户直接切换到UART。
    std::fputs("Damiao close could not verify clean output; unplug/replug the adapter before restarting or switching modes\n", stderr);
  }
}

void DamiaoCan::incomplete()
{
  poisoned_ = true;
  throw FatalError("incomplete Damiao packet submission; stop, unplug/replug the adapter and restart");
}

bool DamiaoCan::check(int & index, std::string & error)
{
  error.clear(); index = session_;
  if (fd_ < 0) {error = "Damiao serial device is closed"; return false;}
  struct stat identity{};
  if (::stat(path_.c_str(), &identity) < 0 || identity.st_rdev != device_ || identity.st_ino != inode_) {
    error = "Damiao serial device was removed or replaced"; return false;
  }
  if (hangup(fd_) == Hangup::Yes) {
    error = "Damiao serial device disconnected"; return false;
  }
  if (phase_ == Phase::Ready) {return true;}
  const auto now = Clock::now();
  if (now >= deadline_) {error = "Damiao initialization timed out"; return false;}
  if (phase_ == Phase::Drain) {
    std::array<std::uint8_t, damiao::Decoder::capacity> discarded{};
    const auto count = ::read(fd_, discarded.data(), discarded.size());
    if (count < 0 && errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR) {
      error = std::strerror(errno);
    } else if (now - drain_started_ >= drain_window) {
      // 固定排空窗口，不要求总线静默；STM32持续查询也不能让初始化永远等不到空闲。
      decoder_.reset();
      phase_ = Phase::Activate;
    }
  }
  if (phase_ == Phase::Activate) {
    // 最后一次read最多取256字节；其后的内核积压/半包也必须清掉，不能放入新会话。
    // EINTR下保留待清理阶段，下一周期再试；仍受本次初始化原截止约束。
    if (::tcflush(fd_, TCIFLUSH) < 0) {
      if (errno == EINTR) {return true;}
      error = std::strerror(errno); return false;
    }
    decoder_.reset(); phase_ = Phase::Ready;
  }
  return error.empty();
}

CanTransport::ReadResult DamiaoCan::receive(damiao::Event & event, std::string & error)
{
  if (decoder_.next(event)) {return ReadResult::FRAME;}
  std::array<std::uint8_t, damiao::Decoder::capacity> bytes{};
  const auto count = ::read(fd_, bytes.data(), decoder_.available());
  if (count < 0) {
    if (errno == EINTR) {return ReadResult::INTERRUPTED;}
    if (errno == EAGAIN || errno == EWOULDBLOCK) {return ReadResult::EMPTY;}
    error = std::strerror(errno); return ReadResult::FAILED;
  }
  if (count == 0) {
    // VMIN=0的空闲与check之后刚发生的挂断都可能返回0，不能把后者永久当成空闲。
    const auto state = hangup(fd_);
    if (state == Hangup::Interrupted) {return ReadResult::INTERRUPTED;}
    if (state == Hangup::Yes) {
      error = "Damiao serial device disconnected"; return ReadResult::FAILED;
    }
    return ReadResult::EMPTY;
  }
  decoder_.append(bytes.data(), static_cast<std::size_t>(count));
  return decoder_.next(event) ? ReadResult::FRAME : ReadResult::EMPTY;
}

CanTransport::ReadResult DamiaoCan::read(can_frame & frame, std::string & error)
{
  error.clear();
  if (!interfaceUp()) {return ReadResult::EMPTY;}
  damiao::Event event;
  const auto result = receive(event, error);
  if (result != ReadResult::FRAME) {return result;}
  switch (event.kind) {
    case damiao::Event::Kind::Frame: frame = event.frame; return ReadResult::FRAME;
    case damiao::Event::Kind::Notice: return ReadResult::IGNORED;
    case damiao::Event::Kind::Malformed: return ReadResult::SHORT;
    case damiao::Event::Kind::Failure:
      error = "Damiao adapter CAN failure, command=" + std::to_string(event.command);
      return ReadResult::FAILED;
  }
  return ReadResult::FAILED;
}

CanTransport::WriteResult DamiaoCan::write(const can_frame & frame, std::string & error)
{
  error.clear();
  if (!interfaceUp()) {error = "Damiao adapter is not ready"; return WriteResult::BACKPRESSURE;}
  // 至多保留一轮(8帧)的USB发送字节，不把主机背压藏进无界串口队列。
  int queued = 0;
  if (::ioctl(fd_, TIOCOUTQ, &queued) < 0) {error = std::strerror(errno); return WriteResult::FAILED;}
  if (queued > 7 * static_cast<int>(damiao::Packet{}.size())) {return WriteResult::BACKPRESSURE;}
  const auto bytes = damiao::encode(frame);
  const auto count = ::write(fd_, bytes.data(), bytes.size());
  if (count == static_cast<ssize_t>(bytes.size())) {return WriteResult::SENT;}
  if (count > 0) {incomplete();}
  if (count == 0) {return WriteResult::BACKPRESSURE;}
  if (errno == EINTR) {return WriteResult::INTERRUPTED;}
  if (errno == EAGAIN || errno == EWOULDBLOCK || errno == ENOBUFS) {return WriteResult::BACKPRESSURE;}
  error = std::strerror(errno); return WriteResult::FAILED;
}
