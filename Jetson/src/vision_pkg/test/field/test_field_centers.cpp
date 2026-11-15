#include "check.hpp"
#include "visibility.hpp"
#include "../../../interface_pkg/app/core/projective_center.hpp"

int main()
{
  using namespace field_scene;
  for (double lateral : {-0.2, 0.0, 0.2}) {
    Scene scene;
    scene.cam_x = lateral;
    const cv::Size size(1280, 720);
    const Camera camera(scene, size);
    const auto truth = field_test::innerTruth(scene, size);
    const auto split_far = camera.project({0, kH - kWall, 0});
    const auto split_near = camera.project({0, kWall, 0});
    const std::array<cv::Point2f, 4> left{
      truth.corners[0], split_far, split_near, truth.corners[3]};
    const std::array<cv::Point2f, 4> right{
      split_far, truth.corners[1], truth.corners[2], split_near};
    double lx = 0, ly = 0, rx = 0, ry = 0;
    CHECK(FieldGeometry::projectiveCenter(left, lx, ly));
    CHECK(FieldGeometry::projectiveCenter(right, rx, ry));
    CHECK(std::hypot(lx - truth.left_center.x, ly - truth.left_center.y) < 1e-3);
    CHECK(std::hypot(rx - truth.right_center.x, ry - truth.right_center.y) < 1e-3);
  }
  return finishTests("field center truth");
}
