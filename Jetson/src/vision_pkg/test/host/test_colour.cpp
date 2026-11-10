// 颜色判据单元测试：色相扇区与相对色度的判定、增益不变性、整图分割与逐像素判据一致
// （板端编译时覆盖 NEON 路径），以及扇区参数的入口校验。
#include "check.hpp"
#include "production_config.hpp"
#include "field/opencv/algorithm.hpp"

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include <stdexcept>

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
}
