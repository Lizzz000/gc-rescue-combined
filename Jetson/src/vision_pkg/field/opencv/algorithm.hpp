#pragma once
#include "field/field.hpp"

// ---- 场地算法 ----
// 职责：几何、评分和黑带辅助，只给场地实现使用。
// Copyright 2026 jetson_yolo integration
// SPDX-License-Identifier: Apache-2.0
// 声明给 field.cpp、color.cpp、debug.cpp 共用。实现在 morphology.cpp、geometry.cpp、score.cpp。
// 仅声明跨实现文件复用的算法；直线、分边和颜色阈值等局部细节保留在实现内，避免扩大调用边界。



#include <opencv2/core.hpp>

#include <algorithm>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

// field_detail 是包内跨实现文件使用的命名空间；声明不属于节点或对外消息接口。
namespace field_detail {

// 对样本副本求上中位数（偶数个取 sorted[n/2]）；空样本抛 std::invalid_argument。
// 只保证 middle 落在完整排序后的对应位置，不为取一个值排序全部样本；调用方原始顺序不变。
template<typename T>
T upperMedian(std::vector<T> values)
{
  if (values.empty()) {throw std::invalid_argument("Median requires nonempty samples");}
  auto middle = values.begin() + values.size() / 2;
  std::nth_element(values.begin(), middle, values.end());
  return *middle;
}

struct FitOut
{
  // ok 只表示得到可用的几何拟合；checkQuad 与颜色/边证据硬门槛仍需由调用方检查。
  // quad/prior 与传入 pts、预测先验必须处于同一搜索图坐标，不能混入全图或 BEV 坐标。
  bool ok = false;
  std::vector<cv::Point2f> quad;
  std::vector<cv::Point2f> prior;
  std::string reason;
  EdgeState edge_state[4] = {EdgeState::LOST, EdgeState::LOST, EdgeState::LOST, EdgeState::LOST};
  int measured_edges = 0;
  bool used_prior_fallback = false;
  bool out_of_frame = false;   // 某条边只有贴边点：整侧出画，几何不完整
};

// ---- 颜色 ----
// 相对色度上限：保证 HueSector 定点乘积留在 uint16 内。
constexpr double kMaxChromaRatio = 4.0;

// Lab 色相扇区判据（八位 Lab 输入），扇区宽度须 < 180°。标量与 NEON 路径共用这些定点常量。
struct HueSector
{
  HueSector(double hue_lo_deg, double hue_hi_deg, double chroma_ratio_min,
    double light_min, double light_max);
  static HueSector purple(const FieldConfig & cfg);
  // use_blue_interior 选择红或蓝扇区。
  static HueSector interior(const FieldConfig & cfg);
  uchar classify(const uchar * lab) const;

  int16_t lo_a, lo_b, hi_a, hi_b;
  uint16_t ratio_q;
  uchar light_lo, light_hi;
};

// 严格3:2的BGR8面积采样使用整数核，其余类型/比例交回INTER_AREA；返回独占Mat，输入只读。
cv::Mat resizeForField(const cv::Mat & source, cv::Size target);

// ---- 几何与拟合 ----
double dist(const cv::Point2f & p, const cv::Point2f & q);
double polyArea(const std::vector<cv::Point2f> & q);
bool isConvexQuad(const std::vector<cv::Point2f> & q);
// 返回值是较长边、较短边的像素长度估计，不是固定方向的场地宽、高；输入至少有四个有序角点。
std::pair<double, double> quadSize(const std::vector<cv::Point2f> & q);
cv::Mat morph(const cv::Mat & mask, const FieldConfig & cfg);
// 候选外轮廓点：pts 参与拟合；clipped 是贴处理图边界的点，只用于分边先验和判断整侧出画。
struct QuadEvidence
{
  std::vector<cv::Point2f> pts, clipped;
};
FitOut fitQuad(const QuadEvidence & evidence, const FieldConfig & cfg);
bool checkQuad(
  const std::vector<cv::Point2f> & q, const cv::Size & sz, const FieldConfig & cfg,
  std::string & why);
// 内沿 BEV 尺寸（内部地面），见 wall_thickness_m。
cv::Size bevSize(const FieldConfig & cfg);
// 内沿四边形（内部地面，四角都在 z=0）：远端和两侧取紫色围边到内部颜色的过渡线，近端内沿被近墙挡住，
// 由地面单应从外沿近边（前立面底边）内推 wall_thickness_m 得到。corners、鸟瞰、分区和区域都用它；
// 外沿四边形只用于搜索、评分与跟踪。interior 与 outer 同属一张搜索图坐标；没测到的边沿用外沿内缩先验。
struct InnerQuad
{
  std::vector<cv::Point2f> quad;   // TL/TR/BR/BL，与 outer 同坐标
  int measured_edges = 0;          // 远端与两侧内沿中本帧实测的条数（0..3）
};
InnerQuad fitInnerQuad(
  const cv::Mat & interior, const std::vector<cv::Point2f> & outer, const FieldConfig & cfg);
// 同序角点平均距离 / a 的对角线长度，是无量纲跳变比例；输入应保持 TL/TR/BR/BL 顺序。
double quadJumpRatio(const std::vector<cv::Point2f> & a, const std::vector<cv::Point2f> & b);

// ---- 证据与评分 ----
// 掩膜与 quad 同属当前搜索图；red 参数在蓝色内部模式下装的是蓝色证据，名称保持兼容。
void scoreEdges(
  const cv::Mat & purple_probe, const std::vector<cv::Point2f> & quad,
  const FieldConfig & cfg, CandidateScore & score);
double redInsideGrid(
  const cv::Mat & red, const std::vector<cv::Point2f> & quad,
  const FieldConfig & cfg, int & cells_red);
double redOutsideRing(
  const cv::Mat & red, const std::vector<cv::Point2f> & quad,
  const FieldConfig & cfg);
// 缺少紫色支持的边两侧连续出现内部颜色，说明补线穿过场地内部；画外样本不能作为反证。
bool crossesInterior(
  const cv::Mat & interior, const cv::Mat & purple_probe,
  const std::vector<cv::Point2f> & quad, const FieldConfig & cfg);
double geometryScore(
  const std::vector<cv::Point2f> & q, const cv::Size & sz,
  const FieldConfig & cfg);
void finalizeScore(CandidateScore & sc, const FieldConfig & cfg);
std::string rejectCandidate(
  const CandidateScore & sc, int measured, const FieldConfig & cfg, bool tracking);

// 黑带是同行两侧均为内部颜色时的局部色度下陷，须沿资格行拟合成唯一窄直带；端点在 BEV 顶/底边。
// 不以几何中线替代测量；多条合格直线、覆盖不足或超宽下陷均返回无效。
struct SplitMeasurement
{
  bool valid = false;
  double top_x = 0.0, bottom_x = 0.0, confidence = 0.0;
};
SplitMeasurement detectSplitBlack(const cv::Mat & bev, const FieldConfig & cfg);

}  // namespace field_detail
