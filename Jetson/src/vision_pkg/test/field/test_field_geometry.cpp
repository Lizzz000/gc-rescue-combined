#include "check.hpp"
#include "production_config.hpp"
#include "field/opencv/algorithm.hpp"
#include "metrics.hpp"
#include "visibility.hpp"

#include <opencv2/imgproc.hpp>

namespace {
using namespace field_scene;
using namespace field_test;

void groundPerspectiveDirection()
{
  Group group("ground perspective preserves near/far direction");
  const auto cfg = productionConfig();
  const cv::Size size(1280, 720);
  std::string why;
  CHECK(field_detail::checkQuad({{200, 200}, {800, 200}, {900, 500}, {100, 500}}, size, cfg, why));
  CHECK(field_detail::checkQuad({{200, 200}, {800, 200}, {800, 500}, {200, 500}}, size, cfg, why));
  // 等宽边允许少量像素拟合误差；明显的反向透视不能被无方向的 max/min 比值掩盖。
  CHECK(field_detail::checkQuad({{199, 200}, {801, 200}, {800, 500}, {200, 500}}, size, cfg, why));
  CHECK(!field_detail::checkQuad({{100, 200}, {900, 200}, {800, 500}, {200, 500}}, size, cfg, why));
  // ARM 杂物负例 seed=496720：紫盒前叠红盒，颜色证据很强但近边向内收窄。
  CHECK(!field_detail::checkQuad({{50.490940f, 347.154541f}, {184.729416f, 345.609711f},
      {151.009018f, 411.819336f}, {62.000336f, 412.103973f}}, size, cfg, why));
}

void independentRows()
{
  Group group("independent interior rows");
  const auto cfg = productionConfig();
  const std::vector<cv::Point2f> outer{{0, 0}, {263, 0}, {263, 143}, {0, 143}};
  for (int rows : {1, 2, 5, 8, 16}) {
    cv::Mat mask = cv::Mat::zeros(144, 264, CV_8U);
    mask(cv::Rect(10, 90, 244, rows)).setTo(255);
    const auto fit = field_detail::fitInnerQuad(mask, outer, cfg);
    CHECK(fit.measured_edges == (rows == 1 ? 0 : rows < 8 ? 1 : 3));
    // 孤立点把非零包围框扩到整个候选，仍不应稀释真正的连续过渡或增加实测行数。
    mask.at<uchar>(1, 1) = 255;
    mask.at<uchar>(142, 262) = 255;
    const auto noisy = field_detail::fitInnerQuad(mask, outer, cfg);
    CHECK(noisy.measured_edges == fit.measured_edges);
    CHECK(cv::norm(cv::Mat(noisy.quad), cv::Mat(fit.quad)) < 1e-4);
    cv::Mat lab(mask.size(), CV_8UC3, cv::Scalar(100, 158, 85));
    lab.setTo(cv::Scalar(100, 163, 147), mask);
    const auto refined = field_detail::fitInnerQuad(mask, outer, cfg, lab);
    CHECK(refined.measured_edges == (rows == 1 ? 0 : rows < 8 ? 1 : 3));
  }
}

void nearAnchors()
{
  Group group("physical near-edge anchors");
  const auto cfg = productionConfig();
  const cv::Size size(1280, 720);
  const std::vector<cv::Point2f> rectangle{{0, 0}, {263, 0}, {263, 143}, {0, 143}};
  int observed = 0;
  for (double yaw : {-22., 0., 22.}) {
    for (double distance : {0.3, 0.75, 1.5}) {
      for (double height : {0.12, 0.22, 0.35}) {
        for (double lateral : {-0.2, 0., 0.2}) {
          Scene s;
          s.yaw_deg = yaw; s.cam_d = distance; s.cam_h = height; s.cam_x = lateral;
          const Camera camera(s, size);
          const std::vector<cv::Point2f> outer{
            camera.project({-kW / 2, kH, kWallH}), camera.project({kW / 2, kH, kWallH}),
            camera.project({kW / 2, 0, 0}), camera.project({-kW / 2, 0, 0})};
          const auto inverse = cv::getPerspectiveTransform(rectangle, outer);
          std::vector<cv::Point2f> anchors;
          cv::perspectiveTransform(std::vector<cv::Point2f>{{float(263 * kWall / kW), 143},
            {float(263 * (1 - kWall / kW)), 143}}, anchors, inverse);
          CHECK(cv::norm(anchors[0] - camera.project({-kW / 2 + kWall, 0, 0})) < 0.001);
          CHECK(cv::norm(anchors[1] - camera.project({kW / 2 - kWall, 0, 0})) < 0.001);

          if (!inFrame(outer[2], size) || !inFrame(outer[3], size)) {continue;}
          // 只画可见的远半段地面，近墙挡住其余部分；调用真实拟合再外推回可观察的地面近边。
          cv::Mat mask = cv::Mat::zeros(size, CV_8U);
          const auto polygon = camera.poly({{-kW / 2 + kWall, kH - kWall, 0},
            {kW / 2 - kWall, kH - kWall, 0}, {kW / 2 - kWall, 0.15, 0}, {-kW / 2 + kWall, 0.15, 0}});
          cv::fillConvexPoly(mask, polygon, cv::Scalar(255), cv::LINE_8, 3);
          const auto fit = field_detail::fitInnerQuad(mask, outer, cfg);
          // 极远低机位可能没有八个独立行；只有三条真实内沿足够时才使用近锚。
          // 下面的 observed 下限确保生产约束确实被多个姿态执行，而非全部跳过。
          if (fit.measured_edges != 3) {continue;}
          const float iw = float(kW - 2 * kWall), ih = float(kH - 2 * kWall);
          const auto ground = cv::getPerspectiveTransform(
            std::vector<cv::Point2f>{{0, 0}, {iw, 0}, {iw, ih}, {0, ih}}, fit.quad);
          std::vector<cv::Point2f> recovered;
          cv::perspectiveTransform(std::vector<cv::Point2f>{{0, float(ih + kWall)},
            {iw, float(ih + kWall)}}, recovered, ground);
          CHECK(cv::norm(recovered[0] - anchors[0]) < 0.002);
          CHECK(cv::norm(recovered[1] - anchors[1]) < 0.002);
          ++observed;
        }
      }
    }
  }
  CHECK(observed >= 27);
}

void checkScene(const Scene & scene, cv::Size source, bool expect_split = true)
{
  const auto cfg = productionConfig();
  FieldLocator locator(cfg);
  const cv::Size processing(cfg.proc_width, cfg.proc_height);
  const auto expected = innerTruth(scene, processing);
  for (int frame = 0; frame < 8; ++frame) {
    Scene s = scene;
    s.seed = scene.seed * 31 + frame;
    Truth truth;
    const auto image = render(s, source, truth);
    const auto result = locator.update(image);
    if (frame < 4) {continue;}
    CHECK(committedOutline(result));
    if (!committedOutline(result)) {continue;}
    CHECK(result.inner_measured_edges == 3);
    const auto error = edgeLineDistance(result.corners, expected.corners, processing, kEdgeSamples);
    CHECK(error && *error <= kEdgeLineTolPx);
    // 输出坐标属处理图；按两轴比例恢复到原输入后，应与原相机投影一致。
    auto restored = result.corners;
    for (auto & p : restored) {p.x *= float(source.width) / processing.width; p.y *= float(source.height) / processing.height;}
    const auto source_error = edgeLineDistance(restored, innerTruth(scene, source).corners, source, kEdgeSamples);
    CHECK(source_error && *source_error <= kEdgeLineTolPx * source.width / processing.width);
    if (expect_split) {
      CHECK(splitValid(result));
      if (!splitValid(result)) {continue;}
      const auto centers = halfCenters(result);
      CHECK(centers.has_value());
      if (centers) {
        CHECK(0.5 * (cv::norm((*centers)[0] - expected.left_center) +
          cv::norm((*centers)[1] - expected.right_center)) <= kCenterTolPx);
      }
    }
  }
}

void scenes()
{
  Group group("source-resolution field geometry");
  for (int shift : {-1, 0, 1}) {
    Scene s;
    s.seed = unsigned(shift + 2) * 7919 + 21;
    s.cam_x += kSeedCamShiftX * shift; s.cam_d += kSeedCamShiftD * shift;
    s.backdrop = {{-0.45, 0.15, 0.12, 0.10, kRed}, {0.0, 0.50, 0.30, 0.15, kPurple},
      {0.45, 0.25, 0.12, 0.08, kRed}};
    checkScene(s, {1280, 720});
    s.blocks = {{0.12, 0.28, 0.05, 0.05, kRed}};
    checkScene(s, {1280, 720});
    s = Scene{}; s.cam_d = 0.3 + kSeedCamShiftD * shift; s.cam_h = 0.18;
    s.cam_x += kSeedCamShiftX * shift;
    checkScene(s, {1280, 720});
  }
  Scene s;
  s.blocks = {{0, 0.28, 0.05, 0.05, kRed}};
  checkScene(s, {1280, 720}, false);
  s = Scene{};
  checkScene(s, {1920, 1080});  // 精确 3:2 面积采样路径。
  checkScene(s, {1600, 900});   // 通用 OpenCV INTER_AREA 路径。
}

void negatives()
{
  Group group("interior objects do not create a field");
  for (int type = 0; type < 3; ++type) {
    for (int shift : {-1, 0, 1}) {
      Scene s;
      s.cam_x += kSeedCamShiftX * shift; s.cam_d += kSeedCamShiftD * shift;
      s.seed = unsigned(shift + 2) * 7919 + 185;
      if (type == 0) {s.tray = false; s.clutter = 6;}
      else {
        s.interior = false;
        s.blocks = {{0.12, 0.28, 0.05, 0.05, kRed}};
        if (type == 2) {s.backdrop = {{0, 0.50, 0.30, 0.15, kPurple}};}
      }
      FieldLocator locator(productionConfig());
      for (int frame = 0; frame < 8; ++frame) {
        auto current = s; current.seed = s.seed * 31 + frame;
        Truth truth;
        const auto result = locator.update(render(current, {1280, 720}, truth));
        CHECK(!committedOutline(result));
        CHECK(!splitValid(result));
      }
    }
  }
}

void shadowSupport()
{
  Group group("shadow geometry requires strict connected colour seeds");
  const auto cfg = productionConfig();
  const auto sector = field_detail::HueSector::purple(cfg);
  cv::Mat lab(40, 80, CV_8UC3, cv::Scalar(0, 128, 128));
  lab(cv::Rect(4, 10, 20, 20)).setTo(cv::Scalar(10, 132, 120));
  lab(cv::Rect(24, 10, 8, 20)).setTo(cv::Scalar(100, 158, 85));
  lab(cv::Rect(50, 10, 20, 20)).setTo(cv::Scalar(10, 132, 120));
  lab(cv::Rect(32, 10, 8, 20)).setTo(cv::Scalar(10, 129, 127));
  cv::Mat strict, red;
  field_detail::segmentLab(lab, cfg, strict, red);
  const auto original = strict.clone();
  const auto geometry = field_detail::shadowGeometryMask(lab, strict, sector);
  CHECK(cv::countNonZero(geometry(cv::Rect(4, 10, 20, 20))) == 400);
  CHECK(cv::countNonZero(geometry(cv::Rect(50, 10, 20, 20))) == 0);
  CHECK(cv::countNonZero(geometry(cv::Rect(32, 10, 8, 20))) == 0);
  CHECK(cv::countNonZero(geometry.row(0)) == 0);
  CHECK(cv::countNonZero(strict != original) == 0);
  CHECK(cv::countNonZero(field_detail::shadowGeometryMask(lab, cv::Mat::zeros(strict.size(), CV_8U), sector)) == 0);
  // 灰/黑邻接强紫种子也没有色相方向，不能被连通重建吞进框。
  for (int light : {0, 1, 10, 19}) {
    cv::Mat neutral(40, 80, CV_8UC3, cv::Scalar(light, 128, 128));
    neutral(cv::Rect(0, 0, 8, 8)).setTo(cv::Scalar(100, 158, 85));
    field_detail::segmentLab(neutral, cfg, strict, red);
    const auto extended = field_detail::shadowGeometryMask(neutral, strict, sector);
    CHECK(cv::countNonZero(extended) == 64);
  }
}

void visibleGridEvidence()
{
  Group group("visible grid retains full-ground coverage");
  const auto cfg = productionConfig();
  const std::vector<cv::Point2f> quad{{0, 0}, {159, 0}, {159, 99}, {0, 99}};
  cv::Mat red = cv::Mat::zeros(100, 160, CV_8U), purple = red.clone();
  red(cv::Rect(48, 0, 112, 50)).setTo(255);
  purple(cv::Rect(0, 50, 160, 50)).setTo(255);
  int cells = 0;
  const auto covered = field_detail::redInsideGrid(red, purple, quad, cfg, cells);
  CHECK(cells >= cfg.red_cells_min);
  CHECK(std::abs(covered - 0.375) < 1e-9);
  purple.setTo(0);
  CHECK(field_detail::redInsideGrid(red, purple, quad, cfg, cells) == covered);
  CHECK(cells < cfg.red_cells_min);
  for (int height : {0, 1, 2, 5}) {
    purple.setTo(255); red.setTo(0);
    if (height) {red.rowRange(0, height).setTo(255); purple.rowRange(0, height).setTo(0);}
    CHECK(field_detail::redInsideGrid(red, purple, quad, cfg, cells) < cfg.red_inside_min);
  }
  purple.setTo(255); red.setTo(0);
  red(cv::Rect(0, 0, 40, 50)).setTo(255); purple(cv::Rect(0, 0, 40, 50)).setTo(0);
  const auto corner_coverage = field_detail::redInsideGrid(red, purple, quad, cfg, cells);
  CHECK(corner_coverage < cfg.red_inside_min || cells < cfg.red_cells_min);
}

void difficultLighting()
{
  Group group("measured boundaries under glare shadow and low contrast");
  const auto cfg = productionConfig();
  for (int index : {10, 14, 33, 37, 38}) {
    double edge_error = 0, center_error = 0;
    int committed = 0, valid = 0;
    for (int seed = 0; seed < 3; ++seed) {
      Scene s;
      if (index == 10) {s.glare = 2;}
      if (index == 14) {s.shadows = 2; s.shadow_strength = kShadowDark; s.gradient = 0.35;}
      if (index == 33) {s.contrast = 0.55;}
      if (index == 37) {
        s.tint = kTintWarm; s.awb_compensation = kAwbResidual; s.glare = 1; s.shadows = 1;
        s.floor = Floor::WOOD; s.blocks = {{0, 0.18, kBlockSize, 0.05, kBlack}};
      }
      if (index == 38) {
        s.exposure = 0.6; s.contrast = 0.7; s.jpeg = 40; s.motion_blur = 7; s.clutter = 3;
        s.blocks = {{0.08, 0.22, kBlockSize, 0.05, kWhite}};
      }
      s.cam_x += kSeedCamShiftX * (seed - 1); s.cam_d += kSeedCamShiftD * (seed - 1);
      const unsigned base = unsigned(seed + 1) * 7919u + index;
      FieldLocator locator(cfg);
      for (int frame = 0; frame < 8; ++frame) {
        s.seed = base * 31 + frame;
        Truth truth;
        const auto result = locator.update(render(s, {1280, 720}, truth));
        if (frame < 4) {continue;}
        const auto expected = innerTruth(s, {1280, 720});
        if (committedOutline(result)) {
          const auto error = edgeLineDistance(result.corners, expected.corners, {1280, 720}, kEdgeSamples);
          CHECK(error.has_value());
          if (error) {edge_error += *error; ++committed;}
        }
        if (splitValid(result)) {
          const auto centers = halfCenters(result);
          CHECK(centers.has_value());
          if (centers) {
            center_error += 0.5 * (cv::norm((*centers)[0] - expected.left_center) +
              cv::norm((*centers)[1] - expected.right_center));
            ++valid;
          }
        }
      }
    }
    if (index == 37) {
      CHECK(double(committed) / 12 >= kMinValidRateCombo);
      CHECK(valid == 0);  // 此固定黑块把中线全部遮住，原矩阵真值就是不可判。
    } else {
      CHECK(double(valid) / 12 >= (index == 38 ? kMinValidRateCombo : kMinValidRate));
    }
    CHECK(committed > 0 && edge_error / committed <= kEdgeLineTolPx);
    if (index != 37) {CHECK(valid > 0 && center_error / valid <= kCenterTolPx);}
  }
}

void sideMaterialSupport()
{
  Group group("side inliers require measured wall colour");
  const auto cfg = productionConfig();
  const std::vector<cv::Point2f> outer{{0, 0}, {263, 0}, {263, 143}, {0, 143}};
  cv::Mat interior = cv::Mat::zeros(144, 264, CV_8U);
  interior(cv::Rect(10, 90, 244, 16)).setTo(255);
  cv::Mat lab(interior.size(), CV_8UC3, cv::Scalar(100, 158, 85));
  lab.setTo(cv::Scalar(100, 163, 147), interior);
  cv::Mat purple = interior == 0;
  CHECK(field_detail::fitInnerQuad(interior, outer, cfg, lab, purple).measured_edges == 3);
  for (int supporting_rows : {0, cfg.min_side_inliers - 1, cfg.min_side_inliers}) {
    auto evidence = purple.clone();
    evidence(cv::Rect(0, 90, 10, 16)).setTo(0);
    // 放在可见段中部，避开原corner_exclude_frac排除的两端，确保每条支持行都会被实际采到。
    if (supporting_rows) {evidence(cv::Rect(0, 94, 10, supporting_rows)).setTo(255);}
    const auto fit = field_detail::fitInnerQuad(interior, outer, cfg, lab, evidence);
    CHECK(fit.measured_edges == (supporting_rows < cfg.min_side_inliers ? 2 : 3));
  }
}

void purplePanelsAreNotGround()
{
  Group group("purple panel and narrow red strip remain negative");
  for (bool divider : {false, true}) {
    const std::vector<int> heights = divider ? std::vector<int>{20, 40, 60, 80} :
      std::vector<int>{1, 2, 5, 8, 12, 20, 40};
    for (int height : heights) {
      cv::Mat image(720, 1280, CV_8UC3, cv::Scalar(205, 205, 205));
      cv::rectangle(image, {300, 200, 680, 400}, kPurple, cv::FILLED);
      cv::rectangle(image, {312, 212, 656, height}, kRed, cv::FILLED);
      if (divider) {cv::rectangle(image, {630, 212, 20, height}, kBlack, cv::FILLED);}
      FieldLocator locator(productionConfig());
      for (int frame = 0; frame < 8; ++frame) {
        const auto result = locator.update(image);
        CHECK(!committedOutline(result)); CHECK(!splitValid(result));
      }
    }
  }
}
}  // namespace

int main()
{
  groundPerspectiveDirection();
  independentRows();
  nearAnchors();
  scenes();
  negatives();
  shadowSupport();
  visibleGridEvidence();
  difficultLighting();
  sideMaterialSupport();
  purplePanelsAreNotGround();
  return finishTests("field geometry");
}
