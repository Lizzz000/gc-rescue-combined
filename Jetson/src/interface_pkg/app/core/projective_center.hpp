#pragma once

#include <cmath>

// 按四边形对角线交点计算透视中心；退化或无穷远交点返回 false。
namespace FieldGeometry
{
template<class Points>
bool projectiveCenter(const Points & points, double & x, double & y)
{
  if (points.size() != 4) {return false;}
  for (const auto & point : points) {
    if (!std::isfinite(point.x) || !std::isfinite(point.y)) {return false;}
  }
  const double ax = points[2].x - points[0].x;
  const double ay = points[2].y - points[0].y;
  const double bx = points[3].x - points[1].x;
  const double by = points[3].y - points[1].y;
  const double denominator = ax * by - ay * bx;
  const double scale = std::hypot(ax, ay) * std::hypot(bx, by);
  if (!std::isfinite(scale) || scale == 0 || std::abs(denominator) <= scale * 1e-9) {return false;}
  const double dx = points[1].x - points[0].x;
  const double dy = points[1].y - points[0].y;
  const double t = (dx * by - dy * bx) / denominator;
  const double u = (dx * ay - dy * ax) / denominator;
  if (!std::isfinite(t) || !std::isfinite(u) || t < 0 || t > 1 || u < 0 || u > 1) {return false;}
  const double center_x = points[0].x + t * ax;
  const double center_y = points[0].y + t * ay;
  if (!std::isfinite(center_x) || !std::isfinite(center_y)) {return false;}
  x = center_x;
  y = center_y;
  return true;
}
}  // namespace FieldGeometry
