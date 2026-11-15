#pragma once
#include <string>
namespace s288 {
enum class MotionStatus {Idle, Busy, Succeeded, Failed};
struct SessionConfig {std::string device; int motor_id; double kp,kd,closed_deg,open_deg;};
class Session {
public:
  explicit Session(SessionConfig) {}
  bool available() const {return false;}
  bool start_open(double) {return false;}
  bool start_close(double) {return false;}
  MotionStatus status() const {return MotionStatus::Idle;}
  void acknowledge() {}
  void close() {}
};
}
