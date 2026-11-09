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
FitOut fitQuad(
  const std::vector<cv::Point2f> & pts, const FieldConfig & cfg,
  const std::vector<cv::Point2f> & assign_prior,
  const std::vector<cv::Point2f> & fallback_prior);
bool checkQuad(
  const std::vector<cv::Point2f> & q, const cv::Size & sz, const FieldConfig & cfg,
  std::string & why);
cv::Size bevSize(const FieldConfig & cfg);
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

// 黑带须横跨配置的内部采样行区间，并能拟合成一条窄直带；端点在 BEV 顶/底边。
// 不以几何中线替代测量；多个完整暗带或不完整/过宽暗区均返回无效。
struct SplitMeasurement
{
  bool valid = false;
  double top_x = 0.0, bottom_x = 0.0, confidence = 0.0;
};
SplitMeasurement detectSplitBlack(const cv::Mat & bev, const FieldConfig & cfg);

}  // namespace field_detail
