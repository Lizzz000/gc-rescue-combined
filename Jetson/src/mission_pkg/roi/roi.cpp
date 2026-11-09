#include "roi/roi.hpp"

// ---- ROI 计数实现 ----
// 职责：区域几何和颜色件数。
// Copyright 2026 li
// SPDX-License-Identifier: Apache-2.0
// 实现原始 ROI 规则与构造校验，不读取 ROS 消息或系统时间。

#include <interface_pkg/app/core.hpp>

#include <cmath>
#include <limits>
#include <stdexcept>

RoiValidator::RoiValidator(
  std::int64_t image_width, std::int64_t image_height,
  double x, double y, double width, double height, const RoiClassIds & class_ids,
  RoiPolicy policy)
: image_width_(image_width), image_height_(image_height), roi_{x, y, width, height},
  green_class_(std::to_string(class_ids.green)),
  blue_class_(std::to_string(class_ids.blue)),
  black_class_(std::to_string(class_ids.black)),
  orange_class_(std::to_string(class_ids.orange)),
  policy_(policy)
{
  // 原图尺寸必须可表示为正 int；ROI 是原图比例，非空且完整落在 [0,1] 内。
  if (image_width <= 0 || image_height <= 0 ||
    image_width > std::numeric_limits<int>::max() ||
    image_height > std::numeric_limits<int>::max() ||
    !std::isfinite(x) || !std::isfinite(y) || !std::isfinite(width) ||
    !std::isfinite(height) || x < 0 || y < 0 || width <= 0 || height <= 0 ||
    x + width > 1 || y + height > 1)
  {
    throw std::invalid_argument("invalid normalized ROI or image dimensions");
  }
  // 四色编号必须非负且两两不同，否则蓝色“禁止出现”的规则会失效。
  const std::int64_t class_id_values[] = {
    class_ids.green, class_ids.blue, class_ids.black, class_ids.orange};
  for (std::size_t i = 0; i < 4; ++i) {
    if (class_id_values[i] < 0) {throw std::invalid_argument("invalid ROI class identifiers");}
    for (std::size_t j = i + 1; j < 4; ++j) {
      if (class_id_values[i] == class_id_values[j]) {
        throw std::invalid_argument("invalid ROI class identifiers");
      }
    }
  }
}

// valid 表示整帧检查完成，passed 表示颜色组合允许；运输件数由 Mission 另行判定。
// 空数组为 valid=true / passed=false；任一非法记录使整帧无效，已有计数只保留扫描前缀。
// 只统计本帧原始检测，不补跟踪预测，也不推断已经抓取的货物。
RoiRuleResult RoiValidator::evaluate(const RecognitionDetections & input) const
{
  RoiRuleResult result;
  result.roi = roi_;
  for (const auto & detection : input) {
    const auto & box = detection.box;
    // ROI 外的框也先检查完整性；不能先筛掉非法框，再把剩余检测当作一帧有效输入。
    if (!RecognitionValueValidation::axisAlignedBox(box) || !detection.hypothesis) {
      result.reason = "malformed detection";
      return result;
    }
    const auto & hypothesis = *detection.hypothesis;
    if (!RecognitionValueValidation::hypothesis(hypothesis)) {
      result.reason = "malformed detection hypothesis";
      return result;
    }
    // 检测框中心和宽高是像素；四条边分别除以原图尺寸后才与归一化 ROI 比较。
    const double left = (box.center_x - box.width / 2) / image_width_;
    const double right = (box.center_x + box.width / 2) / image_width_;
    const double top = (box.center_y - box.height / 2) / image_height_;
    const double bottom = (box.center_y + box.height / 2) / image_height_;
    // 相交或数值上触边都计入，不要求整框进入 ROI；比较无像素容差，保留浮点舍入结果。
    if (right < roi_.x || left > roi_.x + roi_.width ||
      bottom < roi_.y || top > roi_.y + roi_.height) {continue;}
    // classId 统一合法数字串的前导零："0001" 与 "1" 是同一类别。
    // 未映射到四色的合法数字编号计入 other；非数字类别已在假设校验时使整帧无效。
    // 每条相交记录分别计数，检测 ID 相同也不能合并。
    const auto canonical = RecognitionValueValidation::classId(hypothesis.class_id);
    if (canonical == green_class_) {++result.green_count;} else if (canonical == blue_class_) {
      ++result.blue_count;
    } else if (canonical == black_class_) {++result.black_count;} else if (
      canonical == orange_class_)
    {
      ++result.orange_count;
    } else {++result.other_count;}
  }
  result.valid = true;
  const cargo::Counts counts{result.green_count, result.blue_count, result.black_count,
    result.orange_count, result.other_count};
  // 共用判据保留蓝色、未知、空 ROI、橙色不独占的拒绝顺序；不在此限制运输件数。
  if (const auto * reason = cargo::combinationRejection(counts, policy_)) {
    result.reason = reason;
    return result;
  }
  result.passed = true;
  result.reason = "ROI accepted";
  return result;
}
