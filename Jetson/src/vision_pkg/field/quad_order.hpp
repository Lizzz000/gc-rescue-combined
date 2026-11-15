#pragma once

#include <algorithm>
#include <cmath>
#include <vector>

namespace field_detail {
// 四个有限凸包顶点排成 TL/TR/BR/BL（图像 y 向下）；不依赖 OpenCV 点类型。
// 方向约定：更水平的一对对边作为上/下，再选图像中更靠上的一条。
template<class Point>
std::vector<Point> orderCorners(std::vector<Point> points)
{
  double cx = 0, cy = 0;
  for (const auto & p : points) {cx += p.x; cy += p.y;}
  cx /= points.size(); cy /= points.size();
  std::sort(points.begin(), points.end(), [&](const Point & a, const Point & b) {
    return std::atan2(a.y - cy, a.x - cx) < std::atan2(b.y - cy, b.x - cx);
  });
  double area = 0;
  for (std::size_t i = 0; i < points.size(); ++i) {
    const auto & p = points[i];
    const auto & q = points[(i + 1) % points.size()];
    area += double(p.x) * q.y - double(q.x) * p.y;
  }
  if (area < 0) {std::reverse(points.begin(), points.end());}
  // 比较方向而非水平跨度；长斜边的 dx 更大，并不意味着它更水平。
  const auto horizontal = [&](std::size_t i) {
    const auto & a = points[i];
    const auto & b = points[(i + 1) % 4];
    const double dx = double(b.x) - a.x, dy = double(b.y) - a.y;
    const double length = std::hypot(dx, dy);
    return length > 0 ? std::abs(dx) / length : 0.0;
  };
  const double horizontal02 = horizontal(0) + horizontal(2);
  const double horizontal13 = horizontal(1) + horizontal(3);
  std::size_t start = horizontal02 >= horizontal13 ? 0 : 1;
  const std::size_t opposite = (start + 2) % 4;
  if (points[start].y + points[(start + 1) % 4].y >
      points[opposite].y + points[(opposite + 1) % 4].y) {start = opposite;}
  std::rotate(points.begin(), points.begin() + start, points.end());
  return points;
}
}  // namespace field_detail
