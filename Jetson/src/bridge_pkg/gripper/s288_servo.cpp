#include "gripper/s288_servo.hpp"

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cmath>
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include <cstdint>
#include <cstring>
#include <fcntl.h>
#include <memory>
#include <stdexcept>
#include <sys/file.h>
#include <sys/ioctl.h>
#include <sys/select.h>
#include <thread>
#include <unistd.h>
#include <vector>

namespace s288
{
namespace
{
constexpr double kRatio = 70070.0 / 243.0;
constexpr int kBaud = 6000000;
constexpr double kMinMoveDeg = -360.0, kMaxMoveDeg = 360.0;
constexpr double kFeedbackWaitS = 0.2;
constexpr int kStopAttempts = 3;
constexpr double kCommandPeriodS = 0.02;
constexpr std::uint8_t kControlMark[] = {0xfe, 0xee};
constexpr std::uint8_t kFeedbackMark[] = {0xfc, 0xee};
constexpr int kFeedbackBytes = 26;
constexpr int kReadBytes = 64;
constexpr int kModeStop = 0, kModeRun = 1;
constexpr unsigned kTcgets2 = 0x802C542A, kTcsets2 = 0x402C542B;
constexpr unsigned kBother = 4096, kCbaud = 4111;
constexpr int kTermios2Bytes = 44, kCflagOffset = 8, kSpeedOffset = 36;
constexpr double kTorqueScale = 256000, kSpeedScale = 2.56, kPositionScale = 32768;
constexpr double kKpScale = 1280000, kKdScale = 128000000;
constexpr unsigned kCs8 = 48, kClocal = 2048, kCread = 128;

void put_le16(std::uint8_t * dest, std::int16_t value)
{
  const auto bits = static_cast<std::uint16_t>(value);
  dest[0] = static_cast<std::uint8_t>(bits);
  dest[1] = static_cast<std::uint8_t>(bits >> 8);
}

void put_le32(std::uint8_t * dest, std::uint32_t value)
{
  for (int i = 0; i < 4; ++i) {dest[i] = static_cast<std::uint8_t>(value >> (8 * i));}
}


std::int32_t get_le32(const std::uint8_t * src)
{
  std::uint32_t value = 0;
  for (int i = 0; i < 4; ++i) {value |= static_cast<std::uint32_t>(src[i]) << (8 * i);}
  return static_cast<std::int32_t>(value);
}


std::uint32_t crc32(const std::uint8_t * data, std::size_t length)
{
  if (length % 4 != 0) {throw std::invalid_argument("CRC 数据长度必须是 4 的倍数");}
  std::uint32_t crc = 0xFFFFFFFF;
  for (std::size_t offset = 0; offset < length; offset += 4) {
    std::uint32_t word = 0;
    for (int i = 0; i < 4; ++i) {word |= static_cast<std::uint32_t>(data[offset + i]) << (8 * i);}
    crc ^= word;
    for (int bit = 0; bit < 32; ++bit) {
      crc = ((crc << 1) ^ ((crc & 0x80000000u) ? 0x04C11DB7u : 0u)) & 0xFFFFFFFFu;
    }
  }
  return crc;
}

void check_goto(double target_deg, double seconds)
{
  if (!std::isfinite(target_deg) || !std::isfinite(seconds)) {
    throw std::invalid_argument("角度和时长必须是有限数");
  }
  if (target_deg < kMinMoveDeg || target_deg > kMaxMoveDeg) {
    throw std::invalid_argument("目标角超出范围");
  }
  if (seconds < kMinMoveS || seconds > kMaxMoveS) {
    throw std::invalid_argument("时长超出范围");
  }
}

std::vector<std::uint8_t> control_packet(
  int motor_id, int mode, int timeout, double torque, double speed, double position,
  double kp, double kd)
{
  if (motor_id < 0 || motor_id > 14 || (mode != 0 && mode != 1) || (timeout != 0 && timeout != 1)) {
    throw std::invalid_argument("舵机 ID、模式或超时位不合法");
  }
  if (!std::isfinite(torque) || !std::isfinite(speed) || !std::isfinite(position) ||
    !std::isfinite(kp) || !std::isfinite(kd) || kp < 0 || kd < 0)
  {
    throw std::invalid_argument("控制量必须有限，增益不能为负");
  }
  const long raw_torque = std::lround(torque / kRatio * kTorqueScale);
  const long raw_speed = std::lround(speed * kRatio * kSpeedScale / (2 * M_PI));
  const long raw_position = std::lround(position * kRatio * kPositionScale / (2 * M_PI));
  const long raw_kp = std::lround(kp / (kRatio * kRatio) * kKpScale);
  const long raw_kd = std::lround(kd / (kRatio * kRatio) * kKdScale);
  if (raw_torque < -32768 || raw_torque > 32767 || raw_speed < -32768 || raw_speed > 32767 ||
    raw_position < (-2147483647L - 1) || raw_position > 2147483647L ||
    raw_kp < -32768 || raw_kp > 32767 || raw_kd < -32768 || raw_kd > 32767)
  {
    throw std::invalid_argument("控制量超出 S288 协议范围");
  }
  std::vector<std::uint8_t> payload(16);
  payload[0] = kControlMark[0];
  payload[1] = kControlMark[1];
  payload[2] = static_cast<std::uint8_t>(motor_id | (mode << 4) | (timeout << 7));
  payload[3] = 0;
  put_le16(payload.data() + 4, static_cast<std::int16_t>(raw_torque));
  put_le16(payload.data() + 6, static_cast<std::int16_t>(raw_speed));
  put_le32(payload.data() + 8, static_cast<std::uint32_t>(static_cast<std::int32_t>(raw_position)));
  put_le16(payload.data() + 12, static_cast<std::int16_t>(raw_kp));
  put_le16(payload.data() + 14, static_cast<std::int16_t>(raw_kd));
  const auto crc = crc32(payload.data(), payload.size());
  payload.resize(20);
  put_le32(payload.data() + 16, crc);
  return payload;
}

struct Feedback
{
  int id{0}, mode{0};
  bool timeout{false};
  double position{0};
  std::uint32_t error{0};
};

Feedback parse_feedback(const std::uint8_t * packet, int expected_id)
{
  if (std::memcmp(packet, kFeedbackMark, 2) != 0) {
    throw std::invalid_argument("反馈长度或帧头不正确");
  }
  if (crc32(packet + 2, 20) != static_cast<std::uint32_t>(get_le32(packet + 22))) {
    throw std::invalid_argument("反馈 CRC 不正确");
  }
  const int mode = packet[2];
  if ((mode & 15) != expected_id) {throw std::invalid_argument("反馈舵机 ID 不匹配");}
  Feedback feedback;
  feedback.id = mode & 15;
  feedback.mode = (mode >> 4) & 7;
  feedback.timeout = (mode & 128) != 0;
  feedback.position = static_cast<double>(get_le32(packet + 10)) * (2 * M_PI) / (kPositionScale * kRatio);
  feedback.error = static_cast<std::uint32_t>(get_le32(packet + 14));
  return feedback;
}

}  // namespace

class Session::Link
{
public:
  Link(const std::string & device, int motor_id)
  : motor_id_(motor_id)
  {
    fd_ = ::open(device.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd_ < 0) {throw std::runtime_error(device + " 无法打开");}
    try {
      if (::flock(fd_, LOCK_EX | LOCK_NB) != 0) {
        throw std::runtime_error(device + " 被占用或不可配置");
      }
      if (::ioctl(fd_, TIOCEXCL) != 0) {
        throw std::runtime_error(device + " 被占用或不可配置");
      }
      unsigned char buf[kTermios2Bytes]{};
      if (::ioctl(fd_, kTcgets2, buf) != 0) {
        throw std::runtime_error(device + " 被占用或不可配置");
      }
      std::uint32_t cflag = 0;
      std::memcpy(&cflag, buf + kCflagOffset, 4);
      cflag = (cflag & ~kCbaud & ~kCs8) | kBother | kCs8 | kClocal | kCread;
      std::memset(buf, 0, 16);
      std::memcpy(buf + kCflagOffset, &cflag, 4);
      const std::uint32_t baud = static_cast<std::uint32_t>(kBaud);
      std::memcpy(buf + kSpeedOffset, &baud, 4);
      std::memcpy(buf + kSpeedOffset + 4, &baud, 4);
      if (::ioctl(fd_, kTcsets2, buf) != 0) {
        throw std::runtime_error(device + " 被占用或不可配置");
      }
    } catch (...) {
      ::close(fd_);
      fd_ = -1;
      throw;
    }
  }

  ~Link() {close();}

  Link(const Link &) = delete;
  Link & operator=(const Link &) = delete;

  Feedback exchange(const std::vector<std::uint8_t> & packet)
  {
    const auto deadline = std::chrono::steady_clock::now() +
      std::chrono::duration_cast<std::chrono::steady_clock::duration>(
        std::chrono::duration<double>(kFeedbackWaitS));
    // 非阻塞串口的 EAGAIN 和短写是背压，在同一次反馈时限内写完。设备错误才算失败。
    std::size_t sent = 0;
    while (sent < packet.size()) {
      const auto remaining = deadline - std::chrono::steady_clock::now();
      if (remaining <= std::chrono::steady_clock::duration::zero()) {
        throw std::runtime_error("串口写入不完整");
      }
      const auto n = ::write(fd_, packet.data() + sent, packet.size() - sent);
      if (n > 0) {
        sent += static_cast<std::size_t>(n);
        continue;
      }
      if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR)) {
        fd_set write_fds;
        FD_ZERO(&write_fds);
        FD_SET(fd_, &write_fds);
        timeval tv{};
        const auto us = std::chrono::duration_cast<std::chrono::microseconds>(remaining).count();
        tv.tv_sec = static_cast<long>(us / 1000000);
        tv.tv_usec = static_cast<long>(us % 1000000);
        ::select(fd_ + 1, nullptr, &write_fds, nullptr, &tv);
        continue;
      }
      throw std::runtime_error("串口写入不完整");
    }
    std::vector<std::uint8_t> data;
    while (std::chrono::steady_clock::now() < deadline) {
      const auto remaining = deadline - std::chrono::steady_clock::now();
      if (remaining <= std::chrono::steady_clock::duration::zero()) {break;}
      fd_set read_fds;
      FD_ZERO(&read_fds);
      FD_SET(fd_, &read_fds);
      timeval tv{};
      const auto us = std::chrono::duration_cast<std::chrono::microseconds>(remaining).count();
      tv.tv_sec = us / 1000000;
      tv.tv_usec = us % 1000000;
      const int ready = ::select(fd_ + 1, &read_fds, nullptr, nullptr, &tv);
      if (ready <= 0) {continue;}
      std::uint8_t chunk[kReadBytes];
      const auto n = ::read(fd_, chunk, sizeof(chunk));
      if (n < 0) {continue;}
      data.insert(data.end(), chunk, chunk + n);
      while (true) {
        auto start = std::search(data.begin(), data.end(), kFeedbackMark, kFeedbackMark + 2);
        if (start == data.end()) {
          if (!data.empty()) {data.erase(data.begin(), data.end() - 1);}
          break;
        }
        if (start != data.begin()) {data.erase(data.begin(), start);}
        if (data.size() < static_cast<std::size_t>(kFeedbackBytes)) {break;}
        try {
          auto feedback = parse_feedback(data.data(), motor_id_);
          data.erase(data.begin(), data.begin() + kFeedbackBytes);
          return feedback;
        } catch (const std::invalid_argument &) {
          data.erase(data.begin());
        }
      }
    }
    throw std::runtime_error("未收到有效的舵机反馈");
  }

  Feedback stop()
  {
    const auto packet = control_packet(motor_id_, kModeStop, 0, 0, 0, 0, 0, 0);
    for (int attempt = 0; attempt < kStopAttempts; ++attempt) {
      try {
        auto feedback = exchange(packet);
        if (feedback.mode == kModeStop) {return feedback;}
      } catch (const std::runtime_error &) {
        continue;
      }
    }
    throw std::runtime_error("已发送停机命令，但未确认 Stop 模式");
  }

  Feedback prepare(double kp, double kd)
  {
    control_packet(motor_id_, kModeRun, 1, 0, 0, 0, kp, kd);
    auto initial = stop();
    if (initial.error || initial.timeout) {
      throw std::runtime_error("舵机反馈有错误或超时，未开始转动");
    }
    return initial;
  }

  Feedback run_to(double target, double seconds, double kp, double kd, const std::atomic<bool> & cancel)
  {
    const auto command = control_packet(motor_id_, kModeRun, 1, 0, 0, target, kp, kd);
    const auto started = std::chrono::steady_clock::now();
    const auto limit = std::chrono::duration<double>(seconds);
    try {
      while (std::chrono::steady_clock::now() - started < limit) {
        if (cancel.load()) {break;}
        auto feedback = exchange(command);
        if (feedback.error || feedback.timeout) {
          throw std::runtime_error("舵机反馈错误或超时");
        }
        const auto period = std::chrono::duration<double>(kCommandPeriodS);
        std::this_thread::sleep_for(std::chrono::duration_cast<std::chrono::nanoseconds>(period));
      }
    } catch (...) {
      try {stop();} catch (...) {}
      throw;
    }
    return stop();
  }

  void close()
  {
    if (fd_ < 0) {return;}
    ::ioctl(fd_, TIOCNXCL);
    ::close(fd_);
    fd_ = -1;
  }

private:
  int fd_{-1};
  int motor_id_{0};
};

Session::Session(SessionConfig config)
: config_(std::move(config))
{
  if (config_.device.empty()) {return;}
  try {
    control_packet(config_.motor_id, kModeRun, 1, 0, 0, 0, config_.kp, config_.kd);
    link_ = std::make_unique<Link>(config_.device, config_.motor_id);
  } catch (const std::exception & error) {
    message_ = error.what();
    return;
  }
  try {
    const auto feedback = link_->stop();
    zero_rad_ = feedback.position;
    have_zero_ = true;
    message_ = "已连接，上电零点已记录";
  } catch (const std::exception & error) {
    message_ = std::string("已打开串口，但未能记录上电零点：") + error.what();
    link_.reset();
  }
}

Session::~Session() {close();}

bool Session::available() const {return static_cast<bool>(link_) && have_zero_;}

std::string Session::message() const
{
  std::lock_guard<std::mutex> guard(lock_);
  return message_;
}

bool Session::start_open(double seconds) {return start_goto(config_.open_deg, seconds);}

bool Session::start_close(double seconds) {return start_goto(config_.closed_deg, seconds);}

bool Session::start_goto(double target_deg, double seconds)
{
  try {
    check_goto(target_deg, seconds);
  } catch (const std::exception & error) {
    std::lock_guard<std::mutex> guard(lock_);
    message_ = error.what();
    return false;
  }
  if (!available()) {return false;}
  {
    std::lock_guard<std::mutex> guard(lock_);
    if (status_.load() == MotionStatus::Busy) {return false;}
    if (worker_.joinable()) {worker_.join();}
    cancel_.store(false);
    status_.store(MotionStatus::Busy);
    message_ = "正在转到目标角";
    worker_ = std::thread([this, target_deg, seconds] {run(target_deg, seconds);});
  }
  return true;
}

void Session::run(double target_deg, double seconds)
{
  try {
    if (!have_zero_) {throw std::runtime_error("尚未记录上电零点");}
    auto * link = link_.get();
    const auto initial = link->prepare(config_.kp, config_.kd);
    (void)initial;
    link->run_to(zero_rad_ + target_deg * M_PI / 180.0, seconds, config_.kp, config_.kd, cancel_);
    std::lock_guard<std::mutex> guard(lock_);
    message_ = "已停机";
    status_.store(MotionStatus::Succeeded);
  } catch (const std::exception & error) {
    std::lock_guard<std::mutex> guard(lock_);
    message_ = error.what();
    status_.store(MotionStatus::Failed);
  }
}

void Session::acknowledge()
{
  const auto current = status_.load();
  if (current == MotionStatus::Succeeded || current == MotionStatus::Failed) {
    status_.store(MotionStatus::Idle);
  }
}

void Session::close()
{
  cancel_.store(true);
  if (worker_.joinable()) {worker_.join();}
  std::lock_guard<std::mutex> guard(lock_);
  if (link_) {
    try {link_->stop();} catch (...) {}
    link_.reset();
  }
  have_zero_ = false;
  status_.store(MotionStatus::Idle);
}

}  // namespace s288
