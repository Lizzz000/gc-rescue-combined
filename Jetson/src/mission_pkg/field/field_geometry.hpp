#pragma once

#include <cmath>
#include <interface_pkg/app/core.hpp>


// ---- 安全区消息入口几何与内部中心计算 ----
namespace FieldGeometry
{
// z 必须为 0：平面点。空集允许，点数由上层入口约束。
template<class Points>
bool finitePoints(const Points & points)
{
  for (const auto & point : points) {
    if (!std::isfinite(point.x) || !std::isfinite(point.y) ||
      !std::isfinite(point.z) || point.z != 0) {return false;}
  }
  return true;
}

// 画外投影仍允许。拒绝不修改输入，调用方据此撤销正式几何。
template<class Observation>
bool validObservation(const Observation & value)
{
  if (!value.valid || !value.committed ||
    !PolygonValidation::convexQuad(value.corners) ||
    !PolygonValidation::convexQuad(value.left_region) ||
    !PolygonValidation::convexQuad(value.right_region) ||
    value.split_line.size() != 2 || !finitePoints(value.split_line) ||
    value.bev_width <= 0 || value.bev_height <= 0) {return false;}
  // 两个分界端点必须不同；有限但重合的点不能定义一条分界线。
  if (value.split_line[0].x == value.split_line[1].x &&
    value.split_line[0].y == value.split_line[1].y) {return false;}
  return true;
}

namespace detail
{
// 入口已保证点有限，这里不再扫一遍。
template<class Points>
void normalizedCenter(const Points & points, double width, double height, double & x, double & y)
{
  x = 0;
  y = 0;
  for (const auto & point : points) {
    x += static_cast<double>(point.x) / width / 4.0;
    y += static_cast<double>(point.y) / height / 4.0;
  }
}
}  // namespace detail
}  // namespace FieldGeometry
