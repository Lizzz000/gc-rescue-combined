#pragma once

// C++ 宇树 S288 协议，须与 teleop_pkg/servo/s288.py 保持一致。
// 供 bridge 自动开合在进程内调用；不创建新进程，工作线程不得阻塞 io()。

#include <atomic>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

namespace s288
{

// 单次开合时长的允许范围（秒），须与 teleop_pkg/servo/s288.py 一致；bridge 入口按此校验。
constexpr double kMinMoveS = 0.2, kMaxMoveS = 10.0;

enum class MotionStatus { Idle, Busy, Succeeded, Failed };

struct SessionConfig
{
  std::string device;
  int motor_id{0};
  double kp{0}, kd{0};
  double closed_deg{0}, open_deg{0};
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
  // 运行层消费完成功/失败后清回空闲，才能接受下一次提交。
  void acknowledge();
  void close();

private:
  class Link;
  bool start_goto(double target_deg, double seconds);
  void run(double target_deg, double seconds);

  SessionConfig config_;
  std::unique_ptr<Link> link_;
  mutable std::mutex lock_;
  std::thread worker_;
  std::atomic<MotionStatus> status_{MotionStatus::Idle};
  std::atomic<bool> cancel_{false};
  std::string message_{"未配置舵机串口"};
  double zero_rad_{0};
  bool have_zero_{false};
};

}  // namespace s288
