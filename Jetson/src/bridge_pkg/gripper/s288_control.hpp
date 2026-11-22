#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>

namespace s288 {

inline constexpr double kRadiansPerDegree = 0.01745329251994329577;

inline double trimmedTargetRad(double motor_zero_rad, double logical_target_deg, double trim_deg)
{
  return motor_zero_rad + (logical_target_deg + trim_deg) * kRadiansPerDegree;
}

// 独立输出编码器每圈 8192 格。跨过零点时取最短角差，保留纠偏方向。
inline double outputErrorRad(double logical_target_deg, std::uint16_t zero_encoder, std::uint16_t encoder)
{
  const int counts = static_cast<int>(encoder) - static_cast<int>(zero_encoder);
  const double measured_deg = counts * (360.0 / 8192.0);
  return std::remainder(logical_target_deg - measured_deg, 360.0) * kRadiansPerDegree;
}

// 保位沿用驱动器的位置和阻尼环。误差合格时维持上一拍指令，避免卸载后回弹。
// 调用方仅在 kp 和 max_nm 为正时启用此模式。位置差限幅约束比例项的指令力矩。
inline double openHoldAnchorRad(
  double previous_anchor_rad, double output_error_rad, double tolerance_rad,
  double motor_position_rad, double kp, double max_nm, double dt_s)
{
  double anchor = previous_anchor_rad;
  if (std::abs(output_error_rad) > tolerance_rad) {
    const double correction_speed = std::clamp(6.0 * output_error_rad, -0.45, 0.45);
    anchor += correction_speed * std::clamp(dt_s, 0.0, 0.04);
  }
  const double offset_limit = max_nm / kp;
  return std::clamp(anchor, motor_position_rad - offset_limit, motor_position_rad + offset_limit);
}

}  // namespace s288
