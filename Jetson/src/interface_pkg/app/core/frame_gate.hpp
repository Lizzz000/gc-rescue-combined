#pragma once
// 源时间与断流。调用方负责互斥，本文件不加锁。

// ---- 帧准入 ----
// 职责：源时间与断流判断，调用方负责互斥。
// Copyright 2026 li
// SPDX-License-Identifier: Apache-2.0

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <optional>
#include <stdexcept>
#include <string>
#include <string_view>

// 源时间判断帧龄，单调时间判断断流。调用方使用互斥回调组，本类不加锁。
class FrameGate
{
public:
  using Clock = std::chrono::steady_clock;

  /** @brief 使用已知合法的1秒开发默认值，不重复校验常量。 */
  FrameGate() = default;

  /**
   * @brief 在显式参数入口检查超时并同时建立两种时钟的期限。
   * @param[in] timeout_s 有限正秒数。
   * @throws std::invalid_argument 参数非法时构造失败。
   */
  explicit FrameGate(double timeout_s) {setTimeout(timeout_s);}

  /**
   * @brief 在配置入口检查一次超时，换算源时间与单调时间的期限。
   * @param[in] seconds 有限正秒数。
   * @throws std::invalid_argument 非法值不改变原超时和帧历史。
   */
  void setTimeout(double seconds)
  {
    if (!std::isfinite(seconds) || seconds <= 0) {
      throw std::invalid_argument("frame timeout must be finite and positive");
    }
    timeout_s_ = seconds;
    timeout_ns_ = seconds * 1e9;
  }

  // stamp/now 均为同一 ROS 时间域的纳秒；截止与帧龄上限相等时允许接收。
  // 已记录帧则要求源时间严格更新；只判断准入，不在这里修改历史。
  bool accepts(std::int64_t stamp, std::int64_t now) const noexcept
  {
    return stamp > 0 && stamp >= cutoff_ns_ && stamp <= now &&
           static_cast<double>(now - stamp) <= timeout_ns_ &&
           (!have_frame_ || stamp > last_stamp_ns_);
  }

  // 坐标系改变或源帧间隔过长时，调用方须清理跟踪／选择历史。
  bool historyChanged(std::int64_t stamp, std::string_view frame_id) const noexcept
  {
    return have_frame_ && (frame_id != frame_id_ ||
           (stamp > last_stamp_ns_ &&
           static_cast<double>(stamp - last_stamp_ns_) > timeout_ns_));
  }

  // 调用方须先通过 accepts()，并在输出完成后记录；空检测数组同样推进帧历史。
  void record(std::int64_t stamp, std::string_view frame_id, Clock::time_point arrival)
  {
    last_stamp_ns_ = stamp;
    frame_id_.assign(frame_id);
    last_arrival_ = arrival;
    have_frame_ = true;
  }

  // 撤销帧历史但保留截止高水位；默认 cutoff=0 也不能重新放行旧阶段帧。
  void clear(std::int64_t cutoff = 0) noexcept
  {
    cutoff_ns_ = std::max(cutoff_ns_, cutoff);
    last_stamp_ns_ = 0;
    have_frame_ = false;
  }

  // ROS 时间暂停或跳变不影响断流判断；无历史帧时无需重复发无效结果。
  bool expired(Clock::time_point now) const noexcept
  {
    return have_frame_ &&
           std::chrono::duration<double>(now - last_arrival_).count() > timeout_s_;
  }

  std::int64_t cutoff() const noexcept {return cutoff_ns_;}
  bool hasFrame() const noexcept {return have_frame_;}

private:
  // 两种单位对应两种判断：源帧龄用纳秒差，断流用 steady_clock 的到达间隔。
  double timeout_s_{1.0}, timeout_ns_{1e9};
  // clear 保留 cutoff 高水位，却撤销 last_stamp 对应的历史资格；不能整体归零。
  std::int64_t cutoff_ns_{0}, last_stamp_ns_{0};
  std::string frame_id_;
  Clock::time_point last_arrival_{};
  bool have_frame_{false};
};

namespace recognition
{

// task、epoch、源时间共同标记一次识别会话。保留未知 task 的原始文本。
struct TaskSession
{
  std::string task;
  std::uint64_t epoch{0};
  // 阶段生效的 ROS 纳秒，不是图像帧号或单调到达时间。
  std::int64_t stamp_ns{0};
};

/**
 * @brief 比较收到的会话身份，不更新历史或触发业务动作。
 * @param[in] current 当前会话；空值表示尚未接收阶段。
 * @param[in] next 新会话，时间单位为ROS纳秒；外层负责时间编码和未来时间。
 * @return 新身份返回true，非正时间、旧时间或重复身份返回false，输入均不变。
 */
inline bool isNewSession(
  const std::optional<TaskSession> & current, const TaskSession & next)
{
  if (next.stamp_ns <= 0) {
    return false;
  }
  if (!current) {
    return true;
  }
  if (next.stamp_ns < current->stamp_ns) {
    return false;
  }
  // epoch 只比较是否相同，可在进程重启后回零；相同源时间换 task/epoch 仍是新会话。
  // 完全重复才返回 false；同模式但更新源时间仍需要调用方清理旧会话。
  return next.task != current->task || next.epoch != current->epoch ||
         next.stamp_ns != current->stamp_ns;
}

}  // namespace recognition
