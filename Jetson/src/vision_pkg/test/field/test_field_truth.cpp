#include "check.hpp"
#include "harness.hpp"
#include "visibility.hpp"

int main()
{
  using namespace field_scene;
  using namespace field_test;
  const cv::Size size(1280, 720);
  CHECK(productionConfig().black_width_m == kBand);
  CHECK(dividerVisibility(Scene{}, size).judge == Judge::JUDGEABLE);
  CHECK(innerEdgesMeetImage(innerTruth(Scene{}, size), size));

  // 单个角点出画，但四条边都还有画内片段，不能一概降为几何负例。
  InnerTruth cropped_corner;
  cropped_corner.corners = {cv::Point2f(-10, 20), cv::Point2f(50, 10), cv::Point2f(70, 70), cv::Point2f(20, 60)};
  CHECK(innerEdgesMeetImage(cropped_corner, cv::Size(100, 100)));
  // 两端分处相邻画外半平面，但连线仍不进入画面；仅检查端点同侧会漏掉它。
  InnerTruth missing_edge;
  missing_edge.corners = {cv::Point2f(-10, 5), cv::Point2f(5, -10), cv::Point2f(70, 70), cv::Point2f(20, 60)};
  CHECK(!innerEdgesMeetImage(missing_edge, cv::Size(100, 100)));
  missing_edge.corners[0] = {-10, 10};
  missing_edge.corners[1] = {10, -10};
  CHECK(!innerEdgesMeetImage(missing_edge, cv::Size(100, 100)));

  // 矩阵的三个相机种子都有整条右边出画；即使中线可见，也无法提交完整场地。
  for (int shift : {-1, 0, 1}) {
    Scene s;
    s.cam_x = 0.25 + kSeedCamShiftX * shift;
    s.cam_d += kSeedCamShiftD * shift;
    s.yaw_deg = 23;
    const auto inner = innerTruth(s, size);
    CHECK(inner.corners[1].x >= size.width && inner.corners[2].x >= size.width);
    CHECK(!innerEdgesMeetImage(inner, size));
    const auto visible = dividerVisibility(s, size);
    CHECK(visible.geometric >= kJudgeableGeometricMin);
    CHECK(visible.total >= kJudgeableMinRatio);
    CHECK(visible.judge == Judge::UNJUDGEABLE);
  }

  // 近色背景没有遮住内沿；单侧贴线物块仍留下可测中线，不得降级来掩盖漏检。
  Scene backdrop;
  backdrop.backdrop = {{-0.45, 0.15, 0.12, 0.10, kRed}, {0.0, 0.50, 0.30, 0.15, kPurple},
    {0.45, 0.25, 0.12, 0.08, kRed}};
  CHECK(dividerVisibility(backdrop, size).judge == Judge::JUDGEABLE);
  for (const auto & colour : {kBlack, kWhite, kBlue}) {
    Scene s;
    s.blocks = {{kBlockOffsets[1], kBlockRows[1], kBlockSize, 0.03, colour}};
    CHECK(dividerVisibility(s, size).judge == Judge::JUDGEABLE);
  }

  // 组合误检帧：黑块压住全部地面中线，远墙上的黑条不能代替地面证据。
  Scene blocked;
  blocked.tint = kTintWarm;
  blocked.awb_compensation = kAwbResidual;
  blocked.glare = 1;
  blocked.shadows = 1;
  blocked.floor = Floor::WOOD;
  blocked.blocks = {{0.0, 0.18, kBlockSize, 0.05, kBlack}};
  blocked.seed = (2u * 7919u + 37u) * 31u + 4u;
  const auto invisible = dividerVisibility(blocked, size);
  CHECK(invisible.total == 0.0);
  CHECK(invisible.judge == Judge::UNJUDGEABLE);
  return finishTests("field visibility truth");
}
