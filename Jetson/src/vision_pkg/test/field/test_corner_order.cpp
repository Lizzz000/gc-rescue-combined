#include "check.hpp"
#include "field/quad_order.hpp"

#include <array>
#include <cmath>

namespace {
struct Point {double x, y;};
using Quad = std::vector<Point>;

bool sameOrder(const Quad & actual, const Quad & expected)
{
  if (actual.size() != expected.size()) {return false;}
  for (std::size_t i = 0; i < actual.size(); ++i) {
    if (std::hypot(actual[i].x - expected[i].x, actual[i].y - expected[i].y) > 1e-8) {
      return false;
    }
  }
  return true;
}

bool clockwiseRing(const Quad & points)
{
  for (std::size_t i = 0; i < points.size(); ++i) {
    const auto & a = points[i];
    const auto & b = points[(i + 1) % 4];
    const auto & c = points[(i + 2) % 4];
    if ((b.x - a.x) * (c.y - b.y) - (b.y - a.y) * (c.x - b.x) <= 0) {return false;}
  }
  return true;
}

Quad rotated(double width, double height, double degrees)
{
  const double angle = degrees * std::acos(-1.0) / 180.0;
  Quad points{{0, 0}, {width, 0}, {width, height}, {0, height}};
  for (auto & p : points) {
    const double x = p.x, y = p.y;
    p = {100 + std::cos(angle) * x - std::sin(angle) * y,
         200 + std::sin(angle) * x + std::cos(angle) * y};
  }
  return points;
}

void permutations(const Quad & expected)
{
  std::array<int, 4> order{0, 1, 2, 3};
  do {
    Quad input;
    for (int i : order) {input.push_back(expected[i]);}
    const auto result = field_detail::orderCorners(input);
    CHECK(sameOrder(result, expected));
    CHECK(clockwiseRing(result));
  } while (std::next_permutation(order.begin(), order.end()));
}
}  // namespace

int main()
{
  Group group("corner order");
  permutations(rotated(10, 2, 0));
  permutations(rotated(2, 10, 20));
  permutations(rotated(2, 10, -20));
  permutations(rotated(10, 10.001, 20));
  permutations({{200, 50}, {260, 60}, {330, 350}, {20, 300}});
  permutations(rotated(10, 10, 44.99));
  auto after_diagonal = rotated(10, 10, 45.01);
  std::rotate(after_diagonal.begin(), after_diagonal.begin() + 3, after_diagonal.end());
  permutations(after_diagonal);
  // 精确 45° 的对称正方形没有唯一物理宽边；只要求确定性、环绕顺序及两种合法方向之一。
  const auto diagonal = rotated(10, 10, 45);
  const auto diagonal_order = field_detail::orderCorners(diagonal);
  auto alternative = diagonal;
  std::rotate(alternative.begin(), alternative.begin() + 3, alternative.end());
  CHECK(sameOrder(diagonal_order, diagonal) || sameOrder(diagonal_order, alternative));
  permutations(diagonal_order);
  // 连续小幅视角变化不能因为长边投影更宽而突然把 BEV 旋转九十度。
  for (int i = -10; i <= 10; ++i) {
    const auto expected = rotated(2, 10, 20 + i * 0.01);
    auto shuffled = expected;
    std::rotate(shuffled.begin(), shuffled.begin() + (i + 12) % 4, shuffled.end());
    CHECK(sameOrder(field_detail::orderCorners(shuffled), expected));
  }
  return finishTests("field corner order");
}
