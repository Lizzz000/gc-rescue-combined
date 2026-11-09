#pragma once

// 检测框的区域几何和颜色件数，不选导航目标，也不判断左右目的区。
// Copyright 2026 li
// SPDX-License-Identifier: Apache-2.0


#include <cstdint>
#include <string>
#include <interface_pkg/app/core.hpp>

// 颜色身份使用模型类别编号，四个编号必须非负且两两不同。
struct RoiClassIds
{
  std::int64_t green, blue, black, orange;
};

// 生产调用由 TargetNode::roiPolicy() 传入公共规则，非运行期 ROS 调参项。
// 纯值接口保留策略参数，便于独立调用；这些开关不包含首趟运输或目的区逻辑。
using RoiPolicy = cargo::CombinationPolicy;

struct RoiRuleResult
{
  // valid 表示整帧检查完成，passed 才是规则结论；非法框提前返回时计数可能只是扫描前缀。
  bool valid{false}, passed{false};
  std::string reason;
  NormalizedRoi roi;
  // 以相交/触边框记录数计件，不按中心或面积判断，也不按检测/轨迹 ID 去重。
  std::uint32_t green_count{0}, blue_count{0}, black_count{0}, orange_count{0}, other_count{0};
};

// 检查完整原始检测集合的颜色组合；目标选择与首趟运输限制均不属于本类。
class RoiValidator
{
public:
  // 图像宽高为原图像素，ROI x/y 为左上角比例、width/height 为宽高比例。
  // 一次性校验这些尺寸、矩形与类别编号，配置不合法则拒绝构造。
  RoiValidator(
    std::int64_t image_width, std::int64_t image_height,
    double x, double y, double width, double height, const RoiClassIds & class_ids,
    RoiPolicy policy = {});
  // 检查完整原始检测集合；非法记录使整帧无效，几何和组合判据见 evaluate 实现。
  RoiRuleResult evaluate(const RecognitionDetections & input) const;
  // 失败应答只读取配置范围，不再用空检测执行一次规则检查。
  const NormalizedRoi & region() const noexcept {return roi_;}

private:
  // 尺寸转为浮点供像素→比例换算；不裁剪原框，ROI 相交判断允许框部分位于画面外。
  double image_width_, image_height_;
  NormalizedRoi roi_;
  // 与检测的规范化数字字符串比较，避免把类名、轨迹身份和类别编号混用。
  std::string green_class_, blue_class_, black_class_, orange_class_;
  RoiPolicy policy_;
};
