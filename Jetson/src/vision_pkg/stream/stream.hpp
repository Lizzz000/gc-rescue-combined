#pragma once
// ---- 帧率限制 ----
// 职责：图传和检测共用的发帧节流，不持有图像。
// Copyright 2026 li
// SPDX-License-Identifier: Apache-2.0
#include <algorithm>
#include <chrono>
#include <cmath>

// 单调时钟、两帧容量的有限额度。允许一帧时间抖动余量，空闲不累积追赶突发。
// 每个实例由所属线程独占，或由调用者持同一把锁；预检查不消耗，最终交付才提交。
class FrameRateLimiter {
public:
  using Clock = std::chrono::steady_clock;
  bool ready(Clock::time_point now, double max_fps) const
  {
    return valid(max_fps) && (max_fps == 0.0 || available(now, max_fps) >= 1.0);
  }
  bool allow(Clock::time_point now, double max_fps)
  {
    if (!valid(max_fps)) {return false;}
    const double credit = available(now, max_fps);
    if (max_fps > 0.0 && credit < 1.0) {return false;}
    credits_ = max_fps > 0.0 ? credit - 1.0 : 2.0;
    last_ = now;
    rate_ = max_fps;
    initialized_ = true;
    return true;
  }
  void reset() noexcept {initialized_ = false;}
private:
  static bool valid(double rate) {return std::isfinite(rate) && rate >= 0.0;}
  double available(Clock::time_point now, double rate) const
  {
    // 改速、时钟回退和新会话都从有限初始额度重新开始。
    if (!initialized_ || rate != rate_ || now < last_ || rate == 0.0) {return 2.0;}
    return std::min(2.0, credits_ + std::chrono::duration<double>(now - last_).count() * rate);
  }
  Clock::time_point last_{};
  double credits_{2.0};
  double rate_{0.0};
  bool initialized_{false};
};
