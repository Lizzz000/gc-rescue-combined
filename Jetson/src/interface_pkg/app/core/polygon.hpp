#pragma once
// 四边形的有限性、平面性和凸性。不持有状态。

// ---- 多边形 ----
// 职责：四边形有限性、平面性和凸性，不持有状态。
// Copyright 2026 li
// SPDX-License-Identifier: Apache-2.0
// 接收端独立复核四边形：有限值、平面性、重复点、退化与凸性；不持有状态。


#include <algorithm>
#include <cmath>
#include <cstddef>

// 容差按自身对角线归一化，
// 不依赖外部图像尺寸，也不混用像素和鸟瞰坐标的尺度。
namespace PolygonValidation
{
/// 纯几何点，坐标已转成 double。
struct Point2
{
  double x{0.0};
  double y{0.0};
};

/// 重复点判定：两点间距不超过对角线长度的该比例即视为重复点。
constexpr double kDuplicateRatio = 1e-9;
/// 退化判定：转角叉积不超过对角线平方的该比例即视为共线（含精确共线）。
constexpr double kTurnRatio = 1e-12;
/// 最小面积比例：面积不超过对角线平方的该比例即视为零面积。
constexpr double kMinAreaRatio = 1e-12;

/**
 * @brief 在消息接收入口校验单点有限性和图像平面约束。
 * @param[in] point 坐标单位一致的消息点，x/y 须有限。
 * @return 满足平面约束返回true，否则返回false；输入不变。
 */
template<typename Point>
bool planarPoint(const Point & point)
{
  return std::isfinite(static_cast<double>(point.x)) &&
         std::isfinite(static_cast<double>(point.y));
}

namespace detail
{
/**
 * @brief 对已归一化的四点检查重复边、转向一致性和面积。
 * @param[in] normalized 四个有限二维点，已按最远点对距离平移归一化，顺序保持原边界顺序。
 * @return 非退化凸四边形返回true；输入不变，不重排顶点。
 */
inline bool normalizedConvexQuad(const Point2 (&normalized)[4])
{
  int sign = 0;
  double double_area = 0.0;
  for (std::size_t i = 0; i < 4; ++i) {
    const Point2 & a = normalized[i];
    const Point2 & b = normalized[(i + 1) % 4];
    const Point2 & c = normalized[(i + 2) % 4];
    const double edge_x = b.x - a.x, edge_y = b.y - a.y;
    const double next_x = c.x - b.x, next_y = c.y - b.y;
    if (std::hypot(edge_x, edge_y) <= kDuplicateRatio) {return false;}
    const double cross = edge_x * next_y - edge_y * next_x;
    if (std::abs(cross) <= kTurnRatio) {return false;}
    const int turn = cross > 0.0 ? 1 : -1;
    // 转向反号说明自交或凹陷，不能按凸四边形接收。
    if (sign != 0 && turn != sign) {return false;}
    sign = turn;
    double_area += a.x * b.y - b.x * a.y;
  }
  // 面积按归一化尺度比较，等于最小容差也视为退化；不改变调用方原始坐标。
  return std::abs(double_area) * 0.5 > kMinAreaRatio;
}

/**
 * @brief 将入口已校验的四点按自身尺度归一化，再应用凸性与面积判据。
 * @param[in] points 恰好四个有限二维点，按边界顺时针或逆时针排列；不允许空指针。
 * @return 非退化凸四边形返回true；输入不变，不重排顶点。
 */
inline bool convexQuad(const Point2 (&points)[4])
{
  // 取最远点对距离作尺度基准，全部容差按它归一化，不依赖原图或 BEV 的配置尺寸。
  double diagonal = 0.0;
  for (std::size_t i = 0; i < 4; ++i) {
    for (std::size_t j = i + 1; j < 4; ++j) {
      diagonal = std::max(
        diagonal,
        std::hypot(points[i].x - points[j].x, points[i].y - points[j].y));
    }
  }
  if (!std::isfinite(diagonal) || diagonal <= 0.0) {return false;}

  // 先平移再归一化：面积不能用大绝对坐标相减，也不能让尺度平方上溢／下溢。
  Point2 normalized[4];
  for (std::size_t i = 0; i < 4; ++i) {
    normalized[i] = {(points[i].x - points[0].x) / diagonal,
      (points[i].y - points[0].y) / diagonal};
  }
  return normalizedConvexQuad(normalized);
}

}  // namespace detail

/**
 * @brief 在消息入口检查四个有限平面点，再复用不重复验值的几何算法。
 * @param[in] points 消息点数组，必须恰好四点；每点x/y有限。
 * @return 非退化凸四边形返回true；任何非法输入返回false，原数组不变。
 */
template<typename Points>
bool convexQuad(const Points & points)
{
  // 消息版检查点数和有限值后进入纯二维判据；有效几何不等于一定在图像边界内。
  if (points.size() != 4) {return false;}
  Point2 quad[4];
  for (std::size_t i = 0; i < 4; ++i) {
    if (!planarPoint(points[i])) {return false;}
    quad[i].x = static_cast<double>(points[i].x);
    quad[i].y = static_cast<double>(points[i].y);
  }
  return detail::convexQuad(quad);
}

}
