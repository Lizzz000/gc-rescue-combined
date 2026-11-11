// 安全区指标的纯函数：只做几何计算和统计，不含阈值与判定。
#pragma once

#include "field/field.hpp"
#include "../../../interface_pkg/app/core/projective_center.hpp"

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <optional>
#include <vector>

namespace field_test {

inline double lineXAt(const cv::Point2f & a, const cv::Point2f & b, double y)
{
  return a.x + (b.x - a.x) * (y - a.y) / (b.y - a.y);
}

inline bool inFrame(const cv::Point2f & p, cv::Size size)
{
  return p.x >= 0 && p.y >= 0 && p.x < size.width && p.y < size.height;
}

// 点到无限长直线 ab 的距离。
inline double pointLineDistance(const cv::Point2f & p, const cv::Point2f & a, const cv::Point2f & b)
{
  const cv::Point2f d = b - a;
  const double n = std::hypot(d.x, d.y);
  return n > 0 ? std::abs(d.x * (p.y - a.y) - d.y * (p.x - a.x)) / n : cv::norm(p - a);
}

// 结果是否声称本帧有完整外框。
inline bool committedOutline(const FieldResult & r)
{
  return r.ok && r.committed && r.corners.size() == 4;
}

// 分区有效：外框提交且中线本帧有完整证据。
inline bool splitValid(const FieldResult & r)
{
  return committedOutline(r) && r.split_valid;
}

// 输出了有效分区，但质量标记为时间预测/保持：即“拿预测冒充测量”。
inline bool predictedAsMeasured(const FieldResult & r)
{
  return splitValid(r) && (r.quality == "predicted" || r.quality == "hold");
}

// 输出了有效分区且有边为 PREDICTED（上一帧预测或候选先验补边）：只作诊断，画外边补齐属正常。
inline bool hasPredictedEdge(const FieldResult & r)
{
  return splitValid(r) && std::any_of(std::begin(r.edge_state), std::end(r.edge_state),
           [](EdgeState s) {return s == EdgeState::PREDICTED;});
}

// 算法分界线（BEV 上下端点投回处理图）与真值中线在给定行上的横向偏差（有符号）。
inline double splitLateralError(const FieldResult & r, const cv::Point2f & far, const cv::Point2f & near)
{
  std::vector<cv::Point2f> bev{{float(r.split_top_x), 0.f}, {float(r.split_bottom_x), float(r.bev.rows - 1)}}, img;
  cv::perspectiveTransform(bev, img, r.homography_inv);
  const double y = 0.5 * (far.y + near.y);
  return lineXAt(img[0], img[1], y) - lineXAt(far, near, y);
}

// 四条边各自在画内的采样点到真值内沿直线的平均距离；返回四边均值。没有画内采样点的边不计。
inline std::optional<double> edgeLineDistance(const std::vector<cv::Point2f> & corners,
  const std::array<cv::Point2f, 4> & truth, cv::Size size, int samples)
{
  double sum = 0;
  int edges = 0;
  for (int e = 0; e < 4; ++e) {
    const cv::Point2f a = corners[e], b = corners[(e + 1) % 4];
    double edge_sum = 0;
    int n = 0;
    for (int i = 0; i < samples; ++i) {
      const float t = (i + 0.5f) / samples;
      const cv::Point2f p = a + t * (b - a);
      if (!inFrame(p, size)) {continue;}
      edge_sum += pointLineDistance(p, truth[e], truth[(e + 1) % 4]);
      ++n;
    }
    if (n == 0) {continue;}
    sum += edge_sum / n;
    ++edges;
  }
  if (edges == 0) {return std::nullopt;}
  return sum / edges;
}

// 画内真值角点的平均误差；画外角点不计。
inline std::optional<double> inFrameCornerError(const std::vector<cv::Point2f> & corners,
  const std::array<cv::Point2f, 4> & truth, cv::Size size)
{
  double sum = 0;
  int n = 0;
  for (int k = 0; k < 4; ++k) {
    if (!inFrame(truth[k], size)) {continue;}
    sum += cv::norm(corners[k] - truth[k]);
    ++n;
  }
  if (n == 0) {return std::nullopt;}
  return sum / n;
}

// 与任务节点共用对角线交点算法，先投影实际区域四角再计算中心。
inline std::optional<std::array<cv::Point2f, 2>> halfCenters(const FieldResult & r)
{
  const float w = float(r.bev.cols - 1), h = float(r.bev.rows - 1);
  const float top = float(r.split_top_x), bottom = float(r.split_bottom_x);
  std::vector<cv::Point2f> bev{{0, 0}, {top, 0}, {w, 0}, {0, h}, {bottom, h}, {w, h}}, img;
  cv::perspectiveTransform(bev, img, r.homography_inv);
  const std::array<cv::Point2f, 4> left{img[0], img[1], img[4], img[3]};
  const std::array<cv::Point2f, 4> right{img[1], img[2], img[5], img[4]};
  double lx, ly, rx, ry;
  if (!FieldGeometry::projectiveCenter(left, lx, ly) || !FieldGeometry::projectiveCenter(right, rx, ry)) {
    return std::nullopt;
  }
  return std::array<cv::Point2f, 2>{cv::Point2f(float(lx), float(ly)), cv::Point2f(float(rx), float(ry))};
}

// 有效↔无效的切换次数。
inline int countFlips(const std::vector<bool> & valid)
{
  int flips = 0;
  for (size_t i = 1; i < valid.size(); ++i) {flips += valid[i] != valid[i - 1];}
  return flips;
}

inline double stddev(const std::vector<double> & v)
{
  if (v.size() < 2) {return 0.0;}
  double mean = 0;
  for (double x : v) {mean += x;}
  mean /= v.size();
  double ss = 0;
  for (double x : v) {ss += (x - mean) * (x - mean);}
  return std::sqrt(ss / (v.size() - 1));
}

inline double median(std::vector<double> v)
{
  if (v.empty()) {return 0.0;}
  std::nth_element(v.begin(), v.begin() + v.size() / 2, v.end());
  return v[v.size() / 2];
}

inline double ratio(int num, int den)
{
  return den > 0 ? double(num) / den : 0.0;
}

}  // namespace field_test
