#pragma once
#include <string>
namespace s288 {
enum class MotionStatus {Idle, Busy, Succeeded, Failed};
struct SessionConfig {
  std::string device; int motor_id; double kp,kd,closed_deg,open_deg,ramp_fraction{0};
  double open_hold_max_nm{0},output_tolerance_deg{1.0},open_trim_deg{0},closed_trim_deg{0};
};
class Session {
public:
  explicit Session(SessionConfig) {}
  bool available() const {return false;}
  bool start_open(double) {return false;}
  bool start_close(double) {return false;}
  MotionStatus status() const {return MotionStatus::Idle;}
  void acknowledge() {}
  void acknowledge(MotionStatus) {}
  std::string message() const {return {};}
  void close() {}
};
}
