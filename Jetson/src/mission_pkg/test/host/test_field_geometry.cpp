#include "check.hpp"
#include "field/field_geometry.hpp"
#include <array>
#include <cmath>
#include <limits>

namespace {
struct Point {double x, y, z;};
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
}
