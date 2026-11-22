#pragma once

#include <cmath>
#include <interface_pkg/app/core/polygon.hpp>
#include <interface_pkg/app/core/projective_center.hpp>


// ---- 安全区消息入口几何与内部中心计算 ----
namespace FieldGeometry
{
// ROS 点的 z 必须为 0。x/y 和点数由对应的二维几何算法检查。
template<class Points>
bool inImagePlane(const Points & points)
{
  for (const auto & point : points) {
    if (point.z != 0) {return false;}
  }
  return true;
}

// 画外投影仍允许。拒绝不修改输入，调用方据此撤销正式几何。
template<class Observation>
bool validObservation(const Observation & value)
{
  const auto valid_quad = [](const auto & points) {
    return PolygonValidation::convexQuad(points) && inImagePlane(points);
  };
  const auto valid_endpoint = [](const auto & point) {
    return point.z == 0 && PolygonValidation::planarPoint(point);
  };
  if (!value.valid || !value.committed ||
    !valid_quad(value.corners) || !valid_quad(value.left_region) ||
    !valid_quad(value.right_region) || value.split_line.size() != 2 ||
    !valid_endpoint(value.split_line[0]) || !valid_endpoint(value.split_line[1]) ||
    value.bev_width <= 0 || value.bev_height <= 0) {return false;}
  // 两个分界端点必须不同；有限但重合的点不能定义一条分界线。
  if (value.split_line[0].x == value.split_line[1].x &&
    value.split_line[0].y == value.split_line[1].y) {return false;}
  return true;
}

namespace detail
{
// 矩形的透视中心是投影后对角线的交点，不是四个图像顶点的均值。
template<class Points>
bool normalizedCenter(const Points & points, double width, double height, double & x, double & y)
{
  if (!std::isfinite(width) ||
    !std::isfinite(height) || width <= 0 || height <= 0) {return false;}
  double center_x, center_y;
  if (!projectiveCenter(points, center_x, center_y) || !inImagePlane(points)) {return false;}
  center_x /= width;
  center_y /= height;
  if (!std::isfinite(center_x) || !std::isfinite(center_y)) {return false;}
  x = center_x;
  y = center_y;
  return true;
}
}  // namespace detail
}  // namespace FieldGeometry
