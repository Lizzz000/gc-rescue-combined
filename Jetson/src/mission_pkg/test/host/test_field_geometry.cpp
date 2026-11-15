#include "check.hpp"
#include "field/field_geometry.hpp"
#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <vector>

namespace {
struct Point {double x, y, z;};
using Points = std::vector<Point>;
struct Observation {
  bool valid = true, committed = true;
  Points corners, left_region, right_region, split_line;
  int bev_width = 640, bev_height = 480;
};

Point project(const std::array<double, 9> & h, double x, double y)
{
  const double w = h[6] * x + h[7] * y + h[8];
  return {(h[0] * x + h[1] * y + h[2]) / w,
          (h[3] * x + h[4] * y + h[5]) / w, 0};
}

Points region(const std::array<double, 9> & h, double left, double right)
{
  return {project(h, left, 0), project(h, right, 0),
          project(h, right, 1), project(h, left, 1)};
}

void centerMatches(const Points & points, const Point & expected)
{
  double x = -1, y = -1;
  CHECK(FieldGeometry::detail::normalizedCenter(points, 640, 480, x, y));
  CHECK(std::abs(x - expected.x / 640) < 1e-10);
  CHECK(std::abs(y - expected.y / 480) < 1e-10);
}
}

void testFieldGeometry()
{
  Group group("field geometry");
  double x = -1.0, y = -1.0;
  const std::array<Point, 4> rectangle{{{0, 0, 0}, {4, 0, 0}, {4, 2, 0}, {0, 2, 0}}};
  CHECK(FieldGeometry::detail::normalizedCenter(rectangle, 4, 2, x, y));
  CHECK(std::abs(x - 0.5) < 1e-9 && std::abs(y - 0.5) < 1e-9);
  const std::array<Point, 4> perspective{{{0, 0, 0}, {2, 0, 0}, {3, 3, 0}, {0, 3, 0}}};
  CHECK(FieldGeometry::detail::normalizedCenter(perspective, 3, 3, x, y));
  CHECK(std::abs(x - 0.4) < 1e-9 && std::abs(y - 0.4) < 1e-9);
  const std::array<Point, 4> parallel{{{0, 0, 0}, {1, 0, 0}, {2, 0, 0}, {3, 0, 0}}};
  CHECK(!FieldGeometry::detail::normalizedCenter(parallel, 3, 3, x, y));
  CHECK(!FieldGeometry::detail::normalizedCenter(perspective, 0, 3, x, y));
  auto nonfinite = perspective;
  nonfinite[0].x = std::numeric_limits<double>::infinity();
  CHECK(!FieldGeometry::detail::normalizedCenter(nonfinite, 3, 3, x, y));

  // 独立单应真值：强透视、斜视、画外以及仿射场景。目标仍是各半区对角线交点。
  const std::array<std::array<double, 9>, 4> transforms{{
    {{420, 30, 100, 70, 360, 50, 0.5, 0.2, 1}},
    {{200, -110, 350, 40, 450, -30, -0.6, 0.4, 1}},
    {{300, 60, 850, -30, 240, -300, 0.1, 0.2, 1}},
    {{400, 80, 20, -60, 200, 100, 0, 0, 1}}}};
  for (const auto & h : transforms) {
    for (const auto & interval : {std::array<double, 2>{0, 1},
                                  std::array<double, 2>{0, 0.5},
                                  std::array<double, 2>{0.5, 1}}) {
      const auto expected = project(h, (interval[0] + interval[1]) * 0.5, 0.5);
      auto points = region(h, interval[0], interval[1]);
      for (int winding = 0; winding < 2; ++winding) {
        for (int first = 0; first < 4; ++first) {
          centerMatches(points, expected);
          std::rotate(points.begin(), points.begin() + 1, points.end());
        }
        std::reverse(points.begin(), points.end());
      }
    }
  }

  const auto & h = transforms[0];
  const Observation valid{true, true, region(h, 0, 1), region(h, 0, 0.5), region(h, 0.5, 1),
                          {project(h, 0.5, 0), project(h, 0.5, 1)}, 640, 480};
  CHECK(FieldGeometry::validObservation(valid));
  auto bad = valid;
  bad.valid = false;
  CHECK(!FieldGeometry::validObservation(bad));
  bad = valid; bad.committed = false;
  CHECK(!FieldGeometry::validObservation(bad));
  bad = valid; bad.bev_width = 0;
  CHECK(!FieldGeometry::validObservation(bad));
  bad = valid; bad.split_line[1] = bad.split_line[0];
  CHECK(!FieldGeometry::validObservation(bad));
  bad = valid; bad.split_line[0].y = std::numeric_limits<double>::infinity();
  CHECK(!FieldGeometry::validObservation(bad));
  bad = valid; bad.split_line.pop_back();
  CHECK(!FieldGeometry::validObservation(bad));
  for (int component = 0; component < 3; ++component) {
    for (int problem = 0; problem < 5; ++problem) {
      bad = valid;
      auto & points = component == 0 ? bad.corners :
        component == 1 ? bad.left_region : bad.right_region;
      switch (problem) {
        case 0: points[0].x = std::numeric_limits<double>::quiet_NaN(); break;
        case 1: points[0] = points[1]; break;
        case 2: std::swap(points[1], points[2]); break;  // 自交，不能默默重排接收端消息。
        case 3: points = {{0, 0, 0}, {1, 0, 0}, {2, 0, 0}, {3, 0, 0}}; break;
        case 4: points.pop_back(); break;
      }
      CHECK(!FieldGeometry::validObservation(bad));
    }
  }

  // 坐标不裁剪；非法输入也不能把一半计算结果写回调用方。
  centerMatches(region(transforms[2], 0, 1), project(transforms[2], 0.5, 0.5));
  for (double dimension : {0.0, -1.0, std::numeric_limits<double>::infinity(),
                           std::numeric_limits<double>::quiet_NaN()}) {
    x = 17; y = 23;
    CHECK(!FieldGeometry::detail::normalizedCenter(valid.corners, dimension, 480, x, y));
    CHECK(x == 17 && y == 23);
    CHECK(!FieldGeometry::detail::normalizedCenter(valid.corners, 640, dimension, x, y));
    CHECK(x == 17 && y == 23);
  }
  auto nonplanar = valid.corners;
  nonplanar[0].z = 1;
  CHECK(!FieldGeometry::detail::normalizedCenter(nonplanar, 640, 480, x, y));
  CHECK(x == 17 && y == 23);
}
