#include "transport/cdc/cdc.hpp"

#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <string>
#include <sys/file.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <termios.h>
#include <unistd.h>

namespace
{

bool device_path_ok(const std::string & path)
{
  if (path.size() <= 5 || path.compare(0, 5, "/dev/") != 0) {return false;}
  for (unsigned char c : path) {
    if (c < 0x21 || c > 0x7e) {return false;}
  }
  bool child = false;
  for (std::size_t begin = 5; begin < path.size();) {
    const auto end = path.find('/', begin);
    const auto part = path.substr(begin, end == std::string::npos ? std::string::npos : end - begin);
    if (part == "..") {return false;}
    if (!part.empty() && part != ".") {child = true;}
    if (end == std::string::npos) {break;}
    begin = end + 1;
  }
  return child;
}

}  // namespace

bool CdcLink::validDevice(const std::string & path) {return device_path_ok(path);}

CdcLink::~CdcLink() {close();}

bool CdcLink::open(const std::string & interface_name, int & index, std::string & error)
{
  error.clear();
  index = 0;
  if (fd_ >= 0) {close();}
  fd_ = ::open(interface_name.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
  if (fd_ < 0) {
    if (errno == EBUSY) {
      throw CanTransport::FatalError("CDC device is already in use");
    }
    error = std::strerror(errno);
    return false;
  }
  // 与手动程序同一把锁。占用中直接退出，不能等对方释放后再抢 link.device。
  if (::flock(fd_, LOCK_EX | LOCK_NB) < 0 || ::ioctl(fd_, TIOCEXCL) < 0) {
    const int reason = errno;
    close();
    if (reason == EBUSY || reason == EWOULDBLOCK || reason == EAGAIN) {
      throw CanTransport::FatalError(
        "CDC device is already in use; stop the other program before opening link.device");
    }
    error = std::strerror(reason);
    return false;
  }
  termios settings{};
  struct stat identity{};
  if (::tcgetattr(fd_, &settings) < 0 || ::fstat(fd_, &identity) < 0 || !S_ISCHR(identity.st_mode)) {
    error = "CDC device must be a tty";
    close();
    return false;
  }
  ::cfmakeraw(&settings);
  settings.c_cflag |= CLOCAL | CREAD;
  // 非阻塞模式下无数据返回 EAGAIN，避免把正常空读当作断线。
  settings.c_cc[VMIN] = 1;
  settings.c_cc[VTIME] = 0;
  if (::tcsetattr(fd_, TCSANOW, &settings) < 0) {
    error = std::strerror(errno);
    close();
    return false;
  }
  index_ = index_ == 0x7fffffff ? 1 : index_ + 1;
  index = index_;
  up_ = true;
  parser_ = link_frame::Parser{};
  stash_count_ = stash_pos_ = 0;
  pending_size_ = pending_sent_ = 0;
  return true;
}

void CdcLink::close()
{
  if (fd_ < 0) {return;}
  ::ioctl(fd_, TIOCNXCL);
  ::close(fd_);
  fd_ = -1;
  up_ = false;
  pending_size_ = pending_sent_ = 0;
}

bool CdcLink::socketOpen() const {return fd_ >= 0;}

bool CdcLink::check(int & index, std::string & error)
{
  error.clear();
  index = index_;
  if (fd_ < 0) {
    up_ = false;
    return true;
  }
  struct stat identity{};
  if (::fstat(fd_, &identity) < 0) {
    error = std::strerror(errno);
    up_ = false;
    return false;
  }
  up_ = true;
  return true;
}

bool CdcLink::interfaceUp() const {return up_;}

CanTransport::ReadResult CdcLink::read(can_frame & frame, std::string & error)
{
  error.clear();
  if (fd_ < 0) {
    error = "CDC closed";
    return ReadResult::FAILED;
  }
  const auto emit = [this, &frame](std::uint8_t byte) {
      link_frame::Frame decoded;
      if (!parser_.feed(byte, decoded)) {return false;}
      frame = {};
      frame.can_id = decoded.type;
      frame.len = decoded.length;
      if (decoded.length > 0) {std::memcpy(frame.data, decoded.data, decoded.length);}
      return true;
    };
  while (stash_pos_ < stash_count_) {
    const auto byte = stash_[stash_pos_++];
    if (stash_pos_ == stash_count_) {stash_pos_ = stash_count_ = 0;}
    if (emit(byte)) {return ReadResult::FRAME;}
  }
  std::uint8_t chunk[256];
  for (int spins = 0; spins < 32; ++spins) {
    const auto n = ::read(fd_, chunk, sizeof(chunk));
    if (n < 0) {
      if (errno == EINTR) {return ReadResult::INTERRUPTED;}
      if (errno == EAGAIN || errno == EWOULDBLOCK) {return ReadResult::EMPTY;}
      error = std::strerror(errno);
      return ReadResult::FAILED;
    }
    if (n == 0) {
      error = "CDC disconnected";
      return ReadResult::FAILED;
    }
    for (ssize_t i = 0; i < n; ++i) {
      if (!emit(chunk[i])) {continue;}
      const auto rest = static_cast<std::size_t>(n - i - 1);
      if (rest > 0 && rest <= stash_.size()) {
        std::memcpy(stash_.data(), chunk + i + 1, rest);
        stash_count_ = rest;
        stash_pos_ = 0;
      }
      return ReadResult::FRAME;
    }
  }
  return ReadResult::EMPTY;
}

CanTransport::WriteResult CdcLink::flush(std::string & error)
{
  while (pending_sent_ < pending_size_) {
    const auto n = ::write(fd_, pending_.data() + pending_sent_, pending_size_ - pending_sent_);
    if (n > 0) {
      pending_sent_ += static_cast<std::size_t>(n);
      continue;
    }
    if (n < 0 && errno == EINTR) {return WriteResult::INTERRUPTED;}
    if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {return WriteResult::BACKPRESSURE;}
    error = std::strerror(errno);
    return WriteResult::FAILED;
  }
  pending_size_ = 0;
  pending_sent_ = 0;
  return WriteResult::SENT;
}

CanTransport::WriteResult CdcLink::write(const can_frame & frame, std::string & error)
{
  error.clear();
  if (fd_ < 0) {
    error = "CDC closed";
    return WriteResult::FAILED;
  }
  if (frame.can_id > 0xff || frame.len > link_frame::kMaxPayload) {
    error = "CDC payload too long";
    return WriteResult::FAILED;
  }
  std::array<std::uint8_t, link_frame::kMaxBytes> encoded{};
  const auto size = link_frame::encode(
    static_cast<std::uint8_t>(frame.can_id), frame.data, frame.len, encoded.data());
  if (size == 0) {
    error = "CDC encode failed";
    return WriteResult::FAILED;
  }
  if (pending_sent_ < pending_size_) {
    const bool retry = size == pending_size_ &&
      std::memcmp(encoded.data(), pending_.data(), size) == 0;
    const auto result = flush(error);
    if (result != WriteResult::SENT || retry) {return result;}
  }
  pending_ = encoded;
  pending_size_ = size;
  pending_sent_ = 0;
  return flush(error);
}
