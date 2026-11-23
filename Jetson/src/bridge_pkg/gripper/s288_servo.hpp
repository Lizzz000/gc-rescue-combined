#pragma once

// C++ 宇树 S288 协议，须与 teleop_pkg/servo/s288.py 保持一致。
// 供 bridge 自动开合在进程内调用；不创建新进程，工作线程不得阻塞 io()。

#include <algorithm>
#include <atomic>
#include <cmath>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>

namespace s288
{

// 单次开合时长的允许范围（秒），须与 teleop_pkg/servo/s288.py 一致；bridge 入口按此校验。
constexpr double kMinMoveS = 0.2, kMaxMoveS = 10.0;
// STM32 跨端回归用这些常量核对保位交接、动作和停机的完整时间预算。
constexpr double kFeedbackWaitS = 0.2;
constexpr int kStopAttempts = 3;
constexpr double kCommandPeriodS = 0.02;
constexpr double kHoldCaptureS = 0.5;
constexpr double kMaxOpenProgressExtensionS = 2.5;
// 必须由独立输出编码器的新进展赢得宽限，最长动作沿用两端原有总预算。
constexpr double openCaptureLimitS(double move_s)
{
  return std::min(move_s + kHoldCaptureS + kMaxOpenProgressExtensionS, kMaxMoveS + kHoldCaptureS);
}
constexpr bool validRampFraction(double fraction) {return fraction >= 0.0 && fraction <= 1.0;}

inline bool validHoldConfig(double max_nm, double tolerance_deg, double kp,
                            double open_deg, double open_trim_deg, double closed_deg, double closed_trim_deg)
{
  return max_nm >= 0 && max_nm <= 0.6 && tolerance_deg > 0 && tolerance_deg < 180 &&
    (max_nm == 0 || kp > 0) && std::abs(open_deg + open_trim_deg) <= 360 &&
    std::abs(closed_deg + closed_trim_deg) <= 360;
}

enum class MotionStatus { Idle, Busy, Succeeded, Failed };

struct SessionConfig
{
  std::string device;
  int motor_id{0};
  double kp{0}, kd{0};
  double closed_deg{0}, open_deg{0};
  double ramp_fraction{0};
  double open_hold_max_nm{0}, output_tolerance_deg{1.0};
  double open_trim_deg{0}, closed_trim_deg{0};
};

class Session
{
public:
  explicit Session(SessionConfig config);
  ~Session();
  Session(const Session &) = delete;
  Session & operator=(const Session &) = delete;

  bool available() const;
  std::string message() const;
  // 提交相对上电零点的开/合。失败（未打开、忙碌、非法时长或角度）返回 false。
  bool start_open(double seconds);
  bool start_close(double seconds);
  MotionStatus status() const {return status_.load();}
  void acknowledge(MotionStatus observed);
  void acknowledge() {acknowledge(status());}
  void close();

private:
  class Link;
  struct Motion {double target_deg, seconds; bool open;};
  bool start_goto(Motion motion);
  void run();
  void execute(const Motion & motion);

  SessionConfig config_;
  std::unique_ptr<Link> link_;
  mutable std::mutex lock_;
  std::condition_variable wake_;
  std::optional<Motion> pending_;
  std::thread worker_;
  std::once_flag close_once_;
  std::atomic<MotionStatus> status_{MotionStatus::Idle};
  std::atomic<bool> cancel_{false};
  std::atomic<bool> leave_hold_{false};
  std::string message_{"未配置舵机串口"};
  double zero_rad_{0};
  int zero_encoder_{0};
  bool have_zero_{false};
};

}  // namespace s288
