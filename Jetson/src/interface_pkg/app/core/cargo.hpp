#pragma once
// 货物件数和组合拒绝。不保存流程，不访问 ROS。

// ---- 货物规则 ----
// 职责：件数和组合拒绝，不保存流程、不访问 ROS。
// Copyright 2026 li
// SPDX-License-Identifier: Apache-2.0
// 货物组合与生产件数的唯一规则定义；不保存流程状态、不访问 ROS，也不提交运输。

#include <cstdint>

namespace cargo
{
struct Counts
{
  std::uint32_t green{0}, blue{0}, black{0}, orange{0}, other{0};
};

// ROI 纯值调用可显式放宽开关；生产三个接收边界使用下面同一份严格组合策略。
struct CombinationPolicy
{
  bool reject_blue{true};
  bool reject_other{true};
  bool orange_exclusive{true};
};

inline constexpr CombinationPolicy kProductionCombination{};
inline constexpr std::uint32_t kProductionFirstGreenCount = 1;
inline constexpr std::uint32_t kProductionMaxSupplyCount = 3;

// 单位为件；相加前提升到 64 位，多个 uint32 极值不会回绕。
constexpr std::uint64_t supplyCount(const Counts & counts) noexcept
{
  return static_cast<std::uint64_t>(counts.green) + counts.black;
}

constexpr std::uint64_t totalCount(const Counts & counts) noexcept
{
  return supplyCount(counts) + counts.blue + counts.orange + counts.other;
}

// nullptr 表示组合通过；字面量保持 ROI 原拒绝顺序和诊断，无字符串分配。
// Mission/Bridge 只据此独立复核，是否改写诊断仍由各调用方决定。
constexpr const char * combinationRejection(
  const Counts & counts, CombinationPolicy policy = {}) noexcept
{
  if (policy.reject_blue && counts.blue > 0) {return "blue in ROI";}
  if (policy.reject_other && counts.other > 0) {return "unknown class in ROI";}
  const auto total = totalCount(counts);
  if (total == 0) {return "empty ROI";}
  // 即使放行蓝色或未知类别，它们仍占件数，不能与要求独占的橙色同装。
  if (policy.orange_exclusive &&
    !(counts.orange == 0 || (counts.orange == 1 && total == 1)))
  {
    return "orange must be exactly one and alone";
  }
  return nullptr;
}

// 容量只限绿加黑，不属于原始 ROI 颜色检查；0 保留纯核心“不限量”的语义。
constexpr bool withinSupplyLimit(const Counts & counts, std::uint32_t maximum) noexcept
{
  return maximum == 0 || supplyCount(counts) <= maximum;
}
}  // namespace cargo
