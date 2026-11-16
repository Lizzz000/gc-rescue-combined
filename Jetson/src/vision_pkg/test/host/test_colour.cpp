// 颜色判据单元测试：色相扇区与相对色度的判定、增益不变性、整图分割与逐像素判据一致
// （板端编译时覆盖 NEON 路径），以及扇区参数的入口校验。
#include "check.hpp"
#include "production_config.hpp"
#include "field/opencv/algorithm.hpp"

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include <stdexcept>
#include <limits>

namespace {

using field_detail::HueSector;

uchar classifyBgr(const HueSector & sector, const cv::Vec3b & bgr)
{
  cv::Mat pixel(1, 1, CV_8UC3, cv::Scalar(bgr[0], bgr[1], bgr[2])), lab;
  cv::cvtColor(pixel, lab, cv::COLOR_BGR2Lab);
  return sector.classify(lab.ptr<uchar>());
}

cv::Vec3b scaled(const cv::Vec3b & bgr, double gain)
{
  return cv::Vec3b(cv::saturate_cast<uchar>(bgr[0] * gain), cv::saturate_cast<uchar>(bgr[1] * gain),
    cv::saturate_cast<uchar>(bgr[2] * gain));
}

bool rejects(FieldConfig config)
{
  try {validateFieldConfig(config);} catch (const std::invalid_argument &) {return true;}
  return false;
}

// 原标量阴影支持流程作为精确参考：候选扫描、4 连通和严格种子传播均保留。
cv::Mat scalarShadow(const cv::Mat & lab, const cv::Mat & strict, const HueSector & sector)
{
  cv::Mat support;
  for (int y = 0; y < lab.rows; ++y) {
    const auto * pixels = lab.ptr<cv::Vec3b>(y);
    const auto * seeds = strict.ptr<uchar>(y);
    for (int x = 0; x < lab.cols; ++x) {
      const auto & p = pixels[x];
      if (seeds[x] || p[0] >= sector.light_lo || (p[1] == 128 && p[2] == 128)) {continue;}
      const uchar at_minimum[3] = {sector.light_lo, p[1], p[2]};
      if (!sector.classify(at_minimum)) {continue;}
      if (support.empty()) {support = strict.clone();}
      support.at<uchar>(y, x) = 255;
    }
  }
  if (support.empty()) {return strict;}
  const cv::Rect bounds = cv::boundingRect(support);
  cv::Mat labels;
  const int count = cv::connectedComponents(support(bounds), labels, 4, CV_32S);
  std::vector<uchar> seeded(count, 0);
  for (int y = 0; y < bounds.height; ++y) {
    const auto * ids = labels.ptr<int>(y);
    const auto * seeds = strict.ptr<uchar>(y + bounds.y) + bounds.x;
    for (int x = 0; x < bounds.width; ++x) {if (seeds[x]) {seeded[ids[x]] = 255;}}
  }
  for (int y = 0; y < bounds.height; ++y) {
    auto * output = support.ptr<uchar>(y + bounds.y) + bounds.x;
    const auto * ids = labels.ptr<int>(y);
    for (int x = 0; x < bounds.width; ++x) {output[x] = seeded[ids[x]];}
  }
  return support;
}

void shadowScalarEquivalence(const FieldConfig & config)
{
  FieldConfig blue = config; blue.use_blue_interior = true;
  const HueSector sectors[] = {HueSector::purple(config), HueSector::interior(config), HueSector::interior(blue)};
  cv::RNG random(192837u);
  for (const auto & sector : sectors) {
    for (int width : {1, 15, 16, 17, 31, 32, 33, 101}) {
      for (int mode = 0; mode < 2; ++mode) {
        cv::Mat lab_parent(23, width + 8, CV_8UC3), seed_parent = cv::Mat::zeros(23, width + 8, CV_8U);
        random.fill(lab_parent, cv::RNG::UNIFORM, 0, 256);
        cv::Mat lab = lab_parent(cv::Rect(3, 2, width, 19));
        cv::Mat seeds = seed_parent(cv::Rect(3, 2, width, 19));
        for (int y = 0; y < lab.rows; ++y) {
          for (int x = 0; x < lab.cols; ++x) {
            auto & p = lab.at<cv::Vec3b>(y, x);
            if (mode == 0) {
              // 覆盖整个原始 Lab 空间、最低亮度边界和任意非零种子值。
              if ((x + y) % 3 == 0) {p[0] = sector.light_lo;}
              if (random.uniform(0, 19) == 0) {seeds.at<uchar>(y, x) = uchar(1 + random.uniform(0, 255));}
            } else {
              // 大片暗色连接与无种子孤岛；中性竖列切断四连通，不能被阴影补色。
              p[0] = sector.light_lo == 0 ? 0 : uchar(random.uniform(0, int(sector.light_lo)));
              p[1] = cv::saturate_cast<uchar>(128 + sector.lo_a + sector.hi_a);
              p[2] = cv::saturate_cast<uchar>(128 + sector.lo_b + sector.hi_b);
              if (x == width / 2 || (x + 7 * y) % 17 == 0) {p[1] = p[2] = 128;}
              if (x == 0 && y == 0) {seeds.at<uchar>(y, x) = 127;}
            }
          }
        }
        const cv::Mat lab_before = lab_parent.clone(), seeds_before = seed_parent.clone();
        const auto expected = scalarShadow(lab, seeds, sector);
        const auto actual = field_detail::shadowGeometryMask(lab, seeds, sector);
        CHECK(cv::countNonZero(actual != expected) == 0);
        CHECK(cv::norm(lab_parent, lab_before, cv::NORM_INF) == 0);
        CHECK(cv::norm(seed_parent, seeds_before, cv::NORM_INF) == 0);
      }
    }
  }
}

}  // namespace

void testColourClassification()
{
  Group group("colour classification");
  const FieldConfig config = productionConfig();
  const HueSector purple = HueSector::purple(config);
  const HueSector red = HueSector::interior(config);
  // 场景生成器与真机录屏的代表色（BGR）。
  const cv::Vec3b tape(190, 95, 120), board(70, 72, 160), recorded_red(72, 72, 146);
  const cv::Vec3b wood(70, 110, 160), white(222, 224, 226), black(32, 32, 35), blue(170, 80, 30);
  CHECK(classifyBgr(purple, tape) == 255);
  CHECK(classifyBgr(red, tape) == 0);
  CHECK(classifyBgr(red, board) == 255);
  CHECK(classifyBgr(red, recorded_red) == 255);
  CHECK(classifyBgr(purple, board) == 0);
  for (const auto & other : {wood, white, black}) {
    CHECK(classifyBgr(purple, other) == 0);
    CHECK(classifyBgr(red, other) == 0);
  }
  // 偏紫的蓝色（色相约 -71°）与录屏紫框正面同扇区，只能保证不算内部颜色。
  CHECK(classifyBgr(red, blue) == 0);
  // 阴影/曝光：同一材质整体增益 0.4..1.3 判定不变。
  for (double gain : {0.4, 0.6, 0.8, 1.3}) {
    CHECK(classifyBgr(purple, scaled(tape, gain)) == 255);
    CHECK(classifyBgr(red, scaled(board, gain)) == 255);
    CHECK(classifyBgr(red, scaled(wood, gain)) == 0);
  }

  // 整图分割必须与逐像素判据逐字节一致；随机 Lab 覆盖全部取值与非 16 对齐的行尾。
  cv::Mat bgr(37, 101, CV_8UC3);
  cv::randu(bgr, cv::Scalar::all(0), cv::Scalar::all(256));
  cv::Mat lab, purple_mask, red_mask;
  cv::cvtColor(bgr, lab, cv::COLOR_BGR2Lab);
  FieldLocator::segment(bgr, config, purple_mask, red_mask);
  int mismatches = 0;
  for (int y = 0; y < bgr.rows; ++y) {
    for (int x = 0; x < bgr.cols; ++x) {
      const uchar * pixel = lab.ptr<uchar>(y) + 3 * x;
      mismatches += purple_mask.at<uchar>(y, x) != purple.classify(pixel);
      mismatches += red_mask.at<uchar>(y, x) != red.classify(pixel);
    }
  }
  CHECK(mismatches == 0);

  // 黑带检测复用已经转换的 Lab；覆盖红/蓝内部、非连续 ROI 及 NEON 的整块和行尾。
  for (bool blue_interior : {false, true}) {
    FieldConfig selected = config;
    selected.use_blue_interior = blue_interior;
    const HueSector selected_interior = HueSector::interior(selected);
    for (int width : {1, 15, 16, 17, 33, 101}) {
      cv::Mat parent(23, width + 8, CV_8UC3);
      cv::randu(parent, cv::Scalar::all(0), cv::Scalar::all(256));
      const cv::Mat source = parent(cv::Rect(3, 2, width, 19));
      const cv::Mat before = parent.clone();
      cv::Mat from_bgr_purple, from_bgr_interior;
      FieldLocator::segment(source, selected, from_bgr_purple, from_bgr_interior);
      cv::Mat lab_parent(parent.size(), CV_8UC3);
      cv::Mat lab_roi = lab_parent(cv::Rect(3, 2, width, 19));
      cv::cvtColor(source, lab_roi, cv::COLOR_BGR2Lab);
      const cv::Mat lab_before = lab_roi.clone();
      cv::Mat reused_purple, reused_interior;
      field_detail::segmentLab(lab_roi, selected, reused_purple, reused_interior);
      CHECK(cv::countNonZero(reused_purple != from_bgr_purple) == 0);
      CHECK(cv::countNonZero(reused_interior != from_bgr_interior) == 0);
      CHECK(cv::norm(parent, before, cv::NORM_INF) == 0);
      CHECK(cv::norm(lab_roi, lab_before, cv::NORM_INF) == 0);
      int scalar_mismatches = 0;
      for (int y = 0; y < lab_roi.rows; ++y) {
        for (int x = 0; x < lab_roi.cols; ++x) {
          const auto * pixel = lab_roi.ptr<uchar>(y) + 3 * x;
          scalar_mismatches += reused_purple.at<uchar>(y, x) != purple.classify(pixel);
          scalar_mismatches += reused_interior.at<uchar>(y, x) != selected_interior.classify(pixel);
        }
      }
      CHECK(scalar_mismatches == 0);
    }
  }

  // 入口校验：扇区须有序、宽度 < 180°，内部扇区不得与紫色重叠。
  FieldConfig bad = config;
  bad.purple_hue_lo_deg = bad.purple_hue_hi_deg;
  CHECK(rejects(bad));
  bad = config;
  bad.red_hue_lo_deg = -100.0; bad.red_hue_hi_deg = 100.0;
  CHECK(rejects(bad));
  bad = config;
  bad.red_hue_lo_deg = config.purple_hue_hi_deg - 1.0;
  CHECK(rejects(bad));
  bad = config;
  bad.red_chroma_ratio_min = field_detail::kMaxChromaRatio + 0.1;
  CHECK(rejects(bad));
  CHECK(!rejects(config));

  // 实测黑带宽度须能经配置装配进入核心；零值、非有限值及宽过内部地面均无物理含义。
  CHECK(FieldConfig{}.black_width_m == 0.020);
  CHECK(config.black_width_m == 0.020);
  for (double width : {0.0, -0.020, std::numeric_limits<double>::quiet_NaN(),
      std::numeric_limits<double>::infinity(), config.field_w_m - 2 * config.wall_thickness_m + 0.001}) {
    bad = config;
    bad.black_width_m = width;
    CHECK(rejects(bad));
  }
  shadowScalarEquivalence(config);
}
