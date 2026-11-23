// Real Session worker, exchange and Stop retry loops; only POSIX device calls are fake.
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
#include <thread>
#include <vector>
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
bool delay_near_deadline = false;
bool check_hold = false;
std::atomic<int> encoder{8190}, raw_speed{0}, active_warning{0};
std::atomic<bool> drop_run{false}, warn_stop{false};
std::atomic<bool> unexpected_stop{false};
bool previous_stop = false;

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
std::uint32_t crc(const unsigned char *data)
{
  std::uint32_t value = 0xffffffff;
  for (int offset = 0; offset < 20; offset += 4) {
    std::uint32_t word = 0;
    for (int i = 0; i < 4; ++i) {word |= std::uint32_t(data[offset + i]) << (8 * i);}
    value ^= word;
    for (int i = 0; i < 32; ++i) {value = (value << 1) ^ ((value & 0x80000000u) ? 0x04c11db7u : 0u);}
  }
  return value;
}
}

int open(const char *, int, ...) {return 42;}
int close(int) {return 0;}
int flock(int, int) {return 0;}
int ioctl(int, unsigned long, ...) {return 0;}
ssize_t write(int, const void *raw, std::size_t size)
{
  const auto *packet = static_cast<const unsigned char *>(raw);
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

int main()
{
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
#ifdef _WIN32
  end_period(1);
  FreeLibrary(timer);
#endif
  return finishTests("S288 timing");
}
