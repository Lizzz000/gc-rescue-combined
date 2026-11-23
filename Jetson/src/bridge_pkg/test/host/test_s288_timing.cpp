// Exercise the real Session with fault-injected POSIX calls or a private Linux PTY.
#include "check.hpp"
#include "gripper/s288_servo.hpp"
#include <sys/select.h>
#include <unistd.h>
#include <algorithm>
#include <array>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <limits>
#include <thread>
#include <vector>
#ifdef S288_LINUX_PTY
#include <poll.h>
#include <pty.h>
#include <stdexcept>
#endif
#ifdef _WIN32
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#endif

namespace {
std::array<unsigned char, 26> feedback{};
std::atomic<int> stop_calls{0};
bool timeout_reply = false;
bool have_feedback = false;
int fault_stop_call = 7;
unsigned stop_fault = 0;
// 会话零点与每次动作准备时的实际位置不同，检查命令是否选对起点和终点。
constexpr std::int32_t session_zero = 800000, action_start = 1010000;
struct RunCommand {std::chrono::steady_clock::time_point time; std::int32_t position;};
std::vector<RunCommand> commands;
std::atomic<int> run_count{0};
bool record_motion = false;
#ifndef S288_LINUX_PTY
bool delay_near_deadline = false;
#endif
bool check_hold = false;
std::atomic<int> encoder{8190}, raw_speed{0}, active_warning{0};
std::atomic<bool> drop_run{false}, warn_stop{false};
std::atomic<bool> unexpected_stop{false};
bool previous_stop = false;

enum class OpenPath {None, Slow, Stalled, Reversed, FastMotor, FastOutput, StillProgressing, SingleJump, LateStable};
OpenPath open_path = OpenPath::None;
constexpr int plant_zero = 8190;
constexpr double plant_ratio = 70070.0 / 243.0;
constexpr double plant_start_deg = -45.0;
constexpr std::int16_t plant_torque = -64;
struct CaptureSample {double error_deg, speed_rad_s, command_deg;};
std::vector<CaptureSample> captures;
std::atomic<int> quiet_replies{0}, crossing_replies{0};
int injected_replies = 0;

// 独立输出编码器沿实时时间演进，动作目标和反馈由生产协议各自编解码。
void modelOpenFeedback(bool moving, bool power_on)
{
  double angle = power_on ? 0.0 : open_path == OpenPath::LateStable ? -0.3 : plant_start_deg;
  int speed = 0;
  if (moving) {
    const double t = std::chrono::duration<double>(commands.back().time - commands.front().time).count();
    switch (open_path) {
      case OpenPath::Slow:
        angle = std::min(0.0, plant_start_deg + 22.0 * t);
        speed = t < 2.2 ? 45 : 0;
        break;
      case OpenPath::Stalled: break;
      case OpenPath::Reversed: angle -= 4.0 * t; speed = -8; break;
      case OpenPath::FastMotor:
      case OpenPath::FastOutput:
        angle = std::min(4.0, plant_start_deg + 22.0 * t);
        speed = open_path == OpenPath::FastMotor && angle < 4.0 ? 45 : 0;
        break;
      case OpenPath::StillProgressing: angle += 4.0 * t; speed = 8; break;
      case OpenPath::SingleJump:
        if (t >= 0.84 && injected_replies == 0) {angle += 4.0; ++injected_replies;}
        break;
      case OpenPath::LateStable:
        if (t < 0.90) {speed = 40;}
        else if (++injected_replies == 3) {
          // 第三张到位反馈在 1 s 捕获期限之后才返回，串口本身仍在 200 ms 读取时限内。
          std::this_thread::sleep_for(std::chrono::milliseconds(135));
        }
        break;
      case OpenPath::None: return;
    }
    if (std::abs(angle) <= 2.0 && open_path == OpenPath::Slow && speed == 0) {++quiet_replies;}
    if (std::abs(angle) <= 2.0 &&
      (open_path == OpenPath::FastMotor || open_path == OpenPath::FastOutput)) {++crossing_replies;}
  }
  const auto counts = std::lround(angle * 8192.0 / 360.0);
  encoder = (plant_zero + counts + 8192) % 8192;
  raw_speed = speed;
  const auto motor = static_cast<std::uint32_t>(session_zero +
    std::lround(angle * plant_ratio * 32768.0 / 360.0));
  for (int i = 0; i < 4; ++i) {feedback[10 + i] = static_cast<unsigned char>(motor >> (8 * i));}
  const auto torque = static_cast<std::uint16_t>(plant_torque);
  feedback[6] = static_cast<unsigned char>(torque);
  feedback[7] = static_cast<unsigned char>(torque >> 8);
  if (moving) {
    captures.push_back({-counts * 360.0 / 8192.0, speed * 2.0 * std::acos(-1.0) / (2.56 * plant_ratio),
      (commands.back().position - session_zero) * 360.0 / (plant_ratio * 32768.0)});
  }
}

#ifndef S288_LINUX_PTY
bool waitStatus(s288::Session & session, s288::MotionStatus wanted,
  std::chrono::seconds timeout = std::chrono::seconds(2))
{
  const auto until = std::chrono::steady_clock::now() + timeout;
  while (session.status() != wanted && session.status() != s288::MotionStatus::Failed &&
         std::chrono::steady_clock::now() < until) {
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
  return session.status() == wanted;
}
#endif
std::uint32_t crc(const unsigned char *data, int bytes = 20)
{
  std::uint32_t value = 0xffffffff;
  for (int offset = 0; offset < bytes; offset += 4) {
    std::uint32_t word = 0;
    for (int i = 0; i < 4; ++i) {word |= std::uint32_t(data[offset + i]) << (8 * i);}
    value ^= word;
    for (int i = 0; i < 32; ++i) {value = (value << 1) ^ ((value & 0x80000000u) ? 0x04c11db7u : 0u);}
  }
  return value;
}
}

static void respondToCommand(const unsigned char *packet)
{
  const auto mode = (packet[2] >> 4) & 7;
  const int count = mode == 0 ? ++stop_calls : 0;
  // Prepare and final Stop each time out twice before receiving feedback.
  timeout_reply = (drop_run && mode == 1) ||
    (!record_motion && (count == 2 || count == 3 || count == 5 || count == 6));
  feedback.fill(0);
  feedback[0] = 0xfc;
  feedback[1] = 0xee;
  feedback[2] = packet[2] & 0x7f;
  if (record_motion) {
    const auto position = static_cast<std::uint32_t>(check_hold || count == 1 ? session_zero : action_start);
    for (int i = 0; i < 4; ++i) {feedback[10 + i] = static_cast<unsigned char>(position >> (8 * i));}
    if (mode == 1) {
      std::uint32_t raw_position = 0;
      for (int i = 0; i < 4; ++i) {raw_position |= std::uint32_t(packet[8 + i]) << (8 * i);}
      commands.push_back({std::chrono::steady_clock::now(), static_cast<std::int32_t>(raw_position)});
      ++run_count;
    }
  }
  if (check_hold) {
    if (open_path != OpenPath::None) {modelOpenFeedback(mode == 1, count == 1);}
    if (mode == 1 && (previous_stop || unexpected_stop)) {feedback[2] &= 0x0f;}
    previous_stop = mode == 0;
    int warning = mode == 1 ? active_warning.load() : int(warn_stop.load());
    const auto output = std::uint16_t(encoder.load() | (warning << 13));
    feedback[18] = static_cast<unsigned char>(output);
    feedback[19] = static_cast<unsigned char>(output >> 8);
    const auto speed = std::uint16_t(raw_speed.load());
    feedback[8] = static_cast<unsigned char>(speed);
    feedback[9] = static_cast<unsigned char>(speed >> 8);
  }
  if (count == fault_stop_call && stop_fault == 1) {feedback[14] = 1;}
  if (count == fault_stop_call && stop_fault == 2) {feedback[2] |= 0x80;}
  const auto check = crc(feedback.data() + 2);
  for (int i = 0; i < 4; ++i) {feedback[22 + i] = static_cast<unsigned char>(check >> (8 * i));}
  have_feedback = true;
}

#ifndef S288_LINUX_PTY
int open(const char *, int, ...) {return 42;}
int close(int) {return 0;}
int flock(int, int) {return 0;}
int ioctl(int, unsigned long, ...) {return 0;}
ssize_t write(int, const void *raw, std::size_t size)
{
  respondToCommand(static_cast<const unsigned char *>(raw));
  return static_cast<ssize_t>(size);
}
ssize_t read(int, void *out, std::size_t size)
{
  if (!have_feedback) {errno = EAGAIN; return -1;}
  if (delay_near_deadline && !commands.empty() &&
      commands.back().time - commands.front().time >= std::chrono::milliseconds(90)) {
    delay_near_deadline = false;
    std::this_thread::sleep_for(std::chrono::milliseconds(130));
  }
  const auto count = std::min(size, feedback.size());
  std::memcpy(out, feedback.data(), count);
  have_feedback = false;
  return static_cast<ssize_t>(count);
}
int select(int, fd_set *, fd_set *, fd_set *, timeval *timeout)
{
  if (!timeout_reply) {return 1;}
  std::this_thread::sleep_for(std::chrono::seconds(timeout->tv_sec) +
                            std::chrono::microseconds(timeout->tv_usec));
  return 0;
}
#else
// 此目标链接真实的生产串口调用，只打开本用例新建的 PTY，从不接受设备路径参数。
class TestTerminal {
public:
  TestTerminal()
  {
    if (::openpty(&master_, &slave_, name_, nullptr, nullptr) < 0) {
      throw std::runtime_error("openpty failed");
    }
    responder_ = std::thread([this] {serve();});
  }
  ~TestTerminal() {stop(); ::close(master_); ::close(slave_);}
  const char *device() const {return name_;}
  void stop()
  {
    stopping_ = true;
    if (responder_.joinable()) {responder_.join();}
  }
  bool healthy() const {return healthy_;}

private:
  void serve()
  {
    std::array<unsigned char, 20> command{};
    size_t used = 0;
    while (!stopping_) {
      pollfd ready{master_, POLLIN, 0};
      const int polled = ::poll(&ready, 1, 20);
      if (polled < 0 && errno == EINTR) {continue;}
      if (polled == 0) {continue;}
      if (polled < 0 || !(ready.revents & POLLIN)) {healthy_ = false; return;}
      const auto count = ::read(master_, command.data() + used, command.size() - used);
      if (count < 0 && errno == EINTR) {continue;}
      if (count <= 0) {healthy_ = false; return;}
      used += static_cast<size_t>(count);
      if (used < command.size()) {continue;}
      std::uint32_t expected_crc = 0;
      for (unsigned i = 0; i < 4; ++i) {expected_crc |= std::uint32_t(command[16 + i]) << (8 * i);}
      if (command[0] != 0xfe || command[1] != 0xee || crc(command.data(), 16) != expected_crc) {
        healthy_ = false; return;
      }
      respondToCommand(command.data());
      used = 0;
      if (timeout_reply) {continue;}
      size_t sent = 0;
      while (sent < feedback.size()) {
        const auto written = ::write(master_, feedback.data() + sent, feedback.size() - sent);
        if (written < 0 && errno == EINTR) {continue;}
        if (written <= 0) {healthy_ = false; return;}
        sent += static_cast<size_t>(written);
      }
    }
  }
  int master_{-1}, slave_{-1};
  char name_[128]{};
  std::atomic<bool> stopping_{false};
  bool healthy_{true};  // Only inspected after join.
  std::thread responder_;
};
#endif

double diagnosticValue(const std::string & message, const std::string & key)
{
  const auto at = message.find(key + "=");
  if (at == std::string::npos) {return std::numeric_limits<double>::quiet_NaN();}
  try {return std::stod(message.substr(at + key.size() + 1));}
  catch (const std::exception &) {return std::numeric_limits<double>::quiet_NaN();}
}

void testProgressiveOpen(s288::SessionConfig config)
{
  Group group("open progress, motor and output speed, measured diagnostics and absolute deadline");
  config.output_tolerance_deg = 2.0;
  for (const auto path : {OpenPath::Slow, OpenPath::Stalled, OpenPath::Reversed,
      OpenPath::FastMotor, OpenPath::FastOutput, OpenPath::StillProgressing, OpenPath::SingleJump, OpenPath::LateStable}) {
    // 现场配置 1 s 的旧截止为 1.5 s，同时覆盖默认动作时长 0.5 s。
    const double move_s = path == OpenPath::Slow ? 1.0 : 0.5;
    const double hard_s = s288::openCaptureLimitS(move_s);
    open_path = path;
    stop_calls = 0;
    quiet_replies = crossing_replies = 0;
    injected_replies = 0;
    commands.clear();
    captures.clear();
#ifdef S288_LINUX_PTY
    TestTerminal terminal;
    config.device = terminal.device();
#endif
    s288::Session session(config);
    CHECK(session.available());
    const auto start = std::chrono::steady_clock::now();
    CHECK(session.start_open(move_s));
    const bool success = path == OpenPath::Slow;
    while (session.status() == s288::MotionStatus::Busy &&
      std::chrono::steady_clock::now() - start < std::chrono::seconds(6)) {
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    // 以第一次退出 Busy 为准，高速穿窗不能先上报成功、随后才因漂移而失败。
    CHECK(session.status() == (success ? s288::MotionStatus::Succeeded : s288::MotionStatus::Failed));
    const auto elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
    const auto message = session.message();
    const auto reported_elapsed = diagnosticValue(message, "elapsed_s");
    CHECK(std::abs(reported_elapsed - elapsed) < 0.18);
    CHECK(elapsed <= hard_s + 0.30);
    if (success) {
      CHECK(elapsed >= 2.2 && quiet_replies >= 3);
      CHECK(diagnosticValue(message, "good_samples") == 3);
      CHECK(stop_calls == 2);  // 已保位，直到显式 close 才发 Stop。
    } else {
      CHECK(stop_calls == 3);
      CHECK(diagnosticValue(message, "good_samples") < 3);
      if (path == OpenPath::Stalled || path == OpenPath::Reversed || path == OpenPath::SingleJump) {
        CHECK(elapsed <= move_s + s288::kHoldCaptureS + 0.20);
      }
      if (path == OpenPath::FastMotor || path == OpenPath::FastOutput) {CHECK(crossing_replies >= 3);}
      if (path == OpenPath::StillProgressing) {CHECK(elapsed >= hard_s - 0.08);}
    }
    session.close();  // 工作线程退出后再检查它记录的帧。
#ifdef S288_LINUX_PTY
    terminal.stop();
    CHECK(terminal.healthy());
#endif
    const double error = diagnosticValue(message, "error_deg");
    const double speed = diagnosticValue(message, "speed_rad_s");
    const double command = diagnosticValue(message, "command_deg");
    CHECK(std::abs(diagnosticValue(message, "torque_nm") - plant_torque * plant_ratio / 256000.0) < 0.002);
    CHECK(std::any_of(captures.begin(), captures.end(), [&](const auto & sample) {
      return std::abs(error - sample.error_deg) < 0.02 && std::abs(speed - sample.speed_rad_s) < 0.002 &&
        std::abs(command - sample.command_deg) < 0.02;
    }));
    if (path == OpenPath::SingleJump) {CHECK(injected_replies == 1);}
    if (path == OpenPath::LateStable) {CHECK(injected_replies == 3);}
    if (success || path == OpenPath::LateStable) {CHECK(std::abs(error) <= 2.0 && std::abs(speed) <= 0.12);}
    else {CHECK(std::abs(error) > 2.0);}
  }
  open_path = OpenPath::None;
}

int main()
{
#ifdef S288_LINUX_PTY
  s288::SessionConfig config;
  config.kp = 2.0;
  config.kd = 0.10;
  config.open_trim_deg = 5.0;
  config.open_hold_max_nm = 0.14;
  config.ramp_fraction = 0.8;
  check_hold = record_motion = true;
  testProgressiveOpen(config);
  return finishTests("S288 Linux PTY");
#else
#ifdef _WIN32
  // 默认计时粒度可超过一个命令周期；让 Windows 的实时时序回归请求 1 ms 定时分辨率。
  const auto timer = LoadLibraryW(L"winmm.dll");
  if (!timer) {return 1;}
  using TimerPeriod = UINT (WINAPI *)(UINT);
  const auto timer_function = [timer](const char * name) {
    return reinterpret_cast<TimerPeriod>(reinterpret_cast<void *>(GetProcAddress(timer, name)));
  };
  const auto begin_period = timer_function("timeBeginPeriod");
  const auto end_period = timer_function("timeEndPeriod");
  if (!begin_period || !end_period || begin_period(1) != 0) {return 1;}
#endif
  Group group("maximum S288 motion includes both Stop retry sequences");
  s288::SessionConfig config;
  config.device = "/dev/mock";
  s288::Session session(config);
  CHECK(session.available());
  const auto began = std::chrono::steady_clock::now();
  CHECK(session.start_close(s288::kMaxMoveS));
  while (session.status() == s288::MotionStatus::Busy &&
         std::chrono::steady_clock::now() - began < std::chrono::seconds(20)) {
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  const double elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - began).count();
  CHECK(session.status() == s288::MotionStatus::Succeeded);
  CHECK(stop_calls == 7);
  CHECK(elapsed >= s288::kMaxMoveS + 4 * s288::kFeedbackWaitS);
  std::cout << "successful action including feedback retries: " << elapsed << " s\n";
  session.close();

  Group failure_group("S288 final Stop reports hardware error or timeout");
  for (unsigned fault : {1U, 2U}) {
    stop_calls = 0;
    stop_fault = fault;
    s288::Session failing_session(config);
    CHECK(failing_session.available());
    CHECK(failing_session.start_close(s288::kMinMoveS));
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);
    while (failing_session.status() == s288::MotionStatus::Busy &&
           std::chrono::steady_clock::now() < deadline) {
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    CHECK(failing_session.status() == s288::MotionStatus::Failed);
    CHECK(stop_calls == 7);
    failing_session.close();
  }
  Group zero_group("S288 rejects a faulted power-on zero");
  fault_stop_call = 1;
  for (unsigned fault : {1U, 2U}) {
    stop_calls = 0;
    stop_fault = fault;
    s288::Session failing_session(config);
    CHECK(!failing_session.available());
    CHECK(!failing_session.start_open(s288::kMinMoveS));
  }

  Group ramp_group("S288 uses prepare position, session target and smoothstep");
  record_motion = true;
  stop_fault = 0;
  const double seconds = 0.4;
  for (double fraction : {0.0, 0.8, 1.0}) {
    stop_calls = 0;
    commands.clear();
    config.ramp_fraction = fraction;
    s288::Session moving(config);
    CHECK(moving.start_open(seconds));
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
    while (moving.status() == s288::MotionStatus::Busy && std::chrono::steady_clock::now() < deadline) {
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    CHECK(moving.status() == s288::MotionStatus::Succeeded);
    moving.close();
    CHECK(commands.size() >= 3);
    if (commands.size() < 3) {continue;}
    CHECK(commands.front().position == (fraction == 0.0 ? session_zero : action_start));
    CHECK(commands.back().position == session_zero);
    double last_progress = 0.0;
    int checked_curve = 0;
    for (const auto &command : commands) {
      const double progress = double(action_start - command.position) / (action_start - session_zero);
      CHECK(progress >= last_progress && progress <= 1.0);
      last_progress = progress;
      if (fraction == 0.0) {CHECK(command.position == session_zero); continue;}
      const double elapsed = std::chrono::duration<double>(command.time - commands.front().time).count();
      const double u = elapsed / (seconds * fraction);
      if (u > 0.10 && u < 0.85) {
        CHECK(std::abs(progress - u * u * (3.0 - 2.0 * u)) < 0.04);
        ++checked_curve;
      }
    }
    if (fraction > 0.0) {CHECK(checked_curve > 0);}
  }
  Group cancel_group("S288 cancellation stops without sending final target");
  stop_calls = 0;
  run_count = 0;
  commands.clear();
  config.ramp_fraction = 1.0;
  s288::Session cancelled(config);
  CHECK(cancelled.start_open(2.0));
  const auto cancel_deadline = std::chrono::steady_clock::now() + std::chrono::seconds(1);
  while (run_count < 2 && std::chrono::steady_clock::now() < cancel_deadline) {
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  cancelled.close();
  CHECK(run_count >= 2);
  CHECK(std::none_of(commands.begin(), commands.end(),
    [](const RunCommand &command) {return command.position == session_zero;}));
  Group deadline_group("S288 feedback over final slot cannot complete the action");
  commands.clear();
  stop_calls = 0;
  delay_near_deadline = true;
  s288::Session delayed(config);
  CHECK(delayed.start_open(s288::kMinMoveS));
  const auto stop_deadline = std::chrono::steady_clock::now() + std::chrono::seconds(1);
  while (delayed.status() == s288::MotionStatus::Busy && std::chrono::steady_clock::now() < stop_deadline) {
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  CHECK(delayed.status() == s288::MotionStatus::Failed);
  CHECK(stop_calls == 3);
  delayed.close();
  CHECK(std::none_of(commands.begin(), commands.end(),
    [](const RunCommand &command) {return command.position == session_zero;}));
  Group hold_group("open encoder feedback, capture, keep holding, handoff to Stop");
  check_hold = true;
  config.kp = 2.0;
  config.kd = 0.1;
  config.open_hold_max_nm = 0.14;
  config.output_tolerance_deg = 1.0;
  config.open_trim_deg = 5.0;
  config.closed_trim_deg = 0.35;
  config.closed_deg = -50.0;
  config.ramp_fraction = 0.8;
  stop_calls = 0;
  run_count = 0;
  commands.clear();
  s288::Session holding(config);
  // 8190 -> 2 穿过独立编码器的回绕点，角差在 1° 内，速度过高仍应继续捕获。
  encoder = 2;
  raw_speed = 80;
  CHECK(holding.start_open(s288::kMinMoveS));
  std::this_thread::sleep_for(std::chrono::milliseconds(260));
  CHECK(holding.status() == s288::MotionStatus::Busy);
  CHECK(!holding.start_close(0.2));
  raw_speed = 0;
  const int before_stable = run_count;
  CHECK(waitStatus(holding, s288::MotionStatus::Succeeded));
  CHECK(run_count - before_stable >= 2);
  holding.acknowledge(s288::MotionStatus::Succeeded);
  const int before_hold = run_count;
  std::this_thread::sleep_for(std::chrono::milliseconds(80));
  CHECK(run_count > before_hold);
  const auto submit = std::chrono::steady_clock::now();
  CHECK(holding.start_close(s288::kMinMoveS));
  CHECK(std::chrono::steady_clock::now() - submit < std::chrono::milliseconds(100));
  CHECK(waitStatus(holding, s288::MotionStatus::Succeeded));
  CHECK(stop_calls == 5);
  CHECK(holding.message().find("-50.") != std::string::npos);
  const int at_close = run_count;
  std::this_thread::sleep_for(std::chrono::milliseconds(60));
  CHECK(run_count == at_close);
  holding.close();
  CHECK(!commands.empty() && commands.front().position == session_zero);
  // 闭合输出轴被挡住仍允许成功 Stop。末个运行帧按逻辑目标加闭合修正量发出。
  const auto closed_target = session_zero + std::lround((-50.0 + 0.35) * (70070.0 / 243.0) * 32768.0 / 360.0);
  CHECK(!commands.empty() && commands.back().position == closed_target);

  Group trim_group("open and close calibration remain effective with holding disabled");
  config.open_hold_max_nm = 0;
  commands.clear();
  stop_calls = 0;
  s288::Session trimmed(config);
  CHECK(trimmed.start_open(s288::kMinMoveS));
  CHECK(waitStatus(trimmed, s288::MotionStatus::Succeeded));
  const auto open_target = session_zero + std::lround(5.0 * (70070.0 / 243.0) * 32768.0 / 360.0);
  CHECK(!commands.empty() && commands.back().position == open_target);
  CHECK(stop_calls == 3);
  CHECK(trimmed.start_close(s288::kMinMoveS));
  CHECK(waitStatus(trimmed, s288::MotionStatus::Succeeded));
  CHECK(!commands.empty() && commands.back().position == closed_target);
  CHECK(stop_calls == 5);
  trimmed.close();

  Group max_hold_group("maximum-duration full ramp leaves time for stable output feedback");
  config.open_hold_max_nm = 0.14;
  config.ramp_fraction = 1.0;
  encoder = 8190;
  stop_calls = 0;
  run_count = 0;
  s288::Session full_ramp(config);
  encoder = 286;
  const auto open_started = std::chrono::steady_clock::now();
  CHECK(full_ramp.start_open(s288::kMaxMoveS));
  while (run_count == 0 && std::chrono::steady_clock::now() - open_started < std::chrono::seconds(2)) {
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  CHECK(run_count > 0);
  encoder = 8190;
  CHECK(waitStatus(full_ramp, s288::MotionStatus::Succeeded, std::chrono::seconds(12)));
  CHECK(std::chrono::duration<double>(std::chrono::steady_clock::now() - open_started).count() >= s288::kMaxMoveS);
  full_ramp.close();
  config.ramp_fraction = 0.8;

  Group fault_group("open hold losses stop and remain visible after acknowledgement");
  enum class Fault {Drift, Warning, Link, Handoff, Mode};
  for (Fault fault : {Fault::Drift, Fault::Warning, Fault::Link, Fault::Handoff, Fault::Mode}) {
    commands.clear();
    encoder = 8190;
    stop_calls = 0;
    s288::Session stalled(config);
    CHECK(stalled.start_open(s288::kMinMoveS));
    CHECK(waitStatus(stalled, s288::MotionStatus::Succeeded));
    stalled.acknowledge();
    if (fault == Fault::Drift) {encoder = 286;}
    if (fault == Fault::Warning) {active_warning = 1;}
    if (fault == Fault::Link) {drop_run = true;}
    if (fault == Fault::Mode) {unexpected_stop = true;}
    if (fault == Fault::Handoff) {
      warn_stop = true;
      CHECK(stalled.start_close(s288::kMinMoveS));
    }
    CHECK(waitStatus(stalled, s288::MotionStatus::Failed));
    stalled.acknowledge(s288::MotionStatus::Succeeded);
    CHECK(stalled.status() == s288::MotionStatus::Failed);
    CHECK(stop_calls == 3);
    active_warning = 0;
    unexpected_stop = false;
    drop_run = false;
    warn_stop = false;
    stalled.close();
    if (fault == Fault::Drift) {
      const double proportional_limit = config.open_hold_max_nm / config.kp;
      const auto limit_counts = std::ceil(proportional_limit * (70070.0 / 243.0) * 32768.0 / (2 * 3.141592653589793));
      CHECK(std::all_of(commands.begin(), commands.end(), [&](const RunCommand & cmd) {
        return std::abs(cmd.position - session_zero) <= limit_counts;
      }));
    }
  }
  Group unreached_group("open capture expires without successful acknowledgement");
  stop_calls = 0;
  encoder = 8190;
  s288::Session unreached(config);
  encoder = 286;
  CHECK(unreached.start_open(s288::kMinMoveS));
  CHECK(waitStatus(unreached, s288::MotionStatus::Failed));
  CHECK(stop_calls == 3);
  unreached.close();
  Group mode_group("only the first run may echo the preceding clean Stop");
  CHECK(s288::validHoldConfig(0.6, 1.0, 2.0, 0, 5.0, -50.0, 0.35));
  CHECK(!s288::validHoldConfig(0.61, 1.0, 2.0, 0, 0, 0, 0));
  CHECK(!s288::validHoldConfig(0, 180, 0, 0, 0, 0, 0));
  CHECK(!s288::validHoldConfig(0.14, 1.0, 2.0, 359.0, 2.0, 0, 0));
  for (bool warning : {true, false}) {
    encoder = 8190;
    stop_calls = 0;
    run_count = 0;
    active_warning = warning ? 1 : 0;
    unexpected_stop = true;
    s288::Session invalid(config);
    CHECK(invalid.start_open(s288::kMinMoveS));
    CHECK(waitStatus(invalid, s288::MotionStatus::Failed));
    CHECK(run_count == (warning ? 1 : 2));
    CHECK(stop_calls == 3);
    invalid.close();
  }
  active_warning = 0;
  unexpected_stop = false;
  testProgressiveOpen(config);
#ifdef _WIN32
  end_period(1);
  FreeLibrary(timer);
#endif
  return finishTests("S288 timing");
#endif
}
