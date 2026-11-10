// 紫框和内部颜色分割。搜索、跟踪和输出在 field.cpp。

#include "field/field.hpp"
#include "field/opencv/algorithm.hpp"

// Copyright 2026 jetson_yolo integration
// SPDX-License-Identifier: Apache-2.0


#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <numeric>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <cstdint>
#include <vector>
#if defined(__aarch64__) && defined(__ARM_NEON)
#include <arm_neon.h>
#endif

using namespace field_detail;

// ---- 颜色分类 ----
// Lab 色相扇区 + 相对色度 C/L，紫框与内部颜色共用同一判据。两者对整体增益近似不变，
// 曝光、阴影不改变判定；色温只旋转色相，由扇区宽度覆盖。不做逐帧颜色归一化：
// 分位/灰世界类白平衡在场地只占画面小部分或地面有色时会把红色内部推向紫色。
// 鸟瞰去边后的占比属于诊断，不能代替候选的完整颜色证据。

namespace field_detail {

namespace {

// 扇区单位向量的定点倍数：叉积 |a|,|b| ≤ 128 时 2×128×64 = 16384，留在 int16 内，角度分辨率约 1°。
constexpr double kSectorScale = 64.0;
// 相对色度定点倍数 2^6：C×64 ≤ 16320；ratio ≤ kMaxChromaRatio 时 L8×ratio_q ≤ 255×100，均在 uint16 内。
constexpr int kChromaShift = 6;
constexpr double kChromaScale = 1 << kChromaShift;
// OpenCV 八位 Lab：L8 = L×255/100，a、b 偏移 128。
constexpr double kLightTo8bit = 2.55;
constexpr int kLabOffset = 128;

}  // namespace

HueSector::HueSector(
  double hue_lo_deg, double hue_hi_deg, double chroma_ratio_min, double light_min,
  double light_max)
: lo_a(static_cast<int16_t>(std::lround(kSectorScale * std::cos(hue_lo_deg * CV_PI / 180.0)))),
  lo_b(static_cast<int16_t>(std::lround(kSectorScale * std::sin(hue_lo_deg * CV_PI / 180.0)))),
  hi_a(static_cast<int16_t>(std::lround(kSectorScale * std::cos(hue_hi_deg * CV_PI / 180.0)))),
  hi_b(static_cast<int16_t>(std::lround(kSectorScale * std::sin(hue_hi_deg * CV_PI / 180.0)))),
  // C/L ≥ ratio（L 0..100）换到八位：C×64 ≥ round(ratio×64/2.55)×L8。
  ratio_q(static_cast<uint16_t>(std::lround(chroma_ratio_min * kChromaScale / kLightTo8bit))),
  light_lo(static_cast<uchar>(std::lround(light_min * kLightTo8bit))),
  light_hi(static_cast<uchar>(std::lround(light_max * kLightTo8bit))) {}

HueSector HueSector::purple(const FieldConfig & cfg)
{
  return HueSector(cfg.purple_hue_lo_deg, cfg.purple_hue_hi_deg, cfg.purple_chroma_ratio_min,
    cfg.purple_L_min, cfg.purple_L_max);
}

HueSector HueSector::interior(const FieldConfig & cfg)
{
  constexpr double kLightMax = 100.0;
  return cfg.use_blue_interior ?
    HueSector(cfg.blue_hue_lo_deg, cfg.blue_hue_hi_deg, cfg.blue_chroma_ratio_min,
      cfg.blue_L_min, kLightMax) :
    HueSector(cfg.red_hue_lo_deg, cfg.red_hue_hi_deg, cfg.red_chroma_ratio_min,
      cfg.red_L_min, kLightMax);
}

uchar HueSector::classify(const uchar * lab) const
{
  const int a = lab[1] - kLabOffset, b = lab[2] - kLabOffset;
  // 扇区宽度 < 180°：在下界逆时针侧且在上界顺时针侧。
  const bool inside = lo_a * b - lo_b * a >= 0 && a * hi_b - b * hi_a >= 0;
  const int chroma = std::abs(a) + std::abs(b);
  const bool vivid = (chroma << kChromaShift) >= ratio_q * lab[0];
  return (inside && vivid && lab[0] >= light_lo && lab[0] <= light_hi) ? 255 : 0;
}

#if defined(__aarch64__) && defined(__ARM_NEON)
namespace {

// 与 HueSector::classify 同一整数运算，16 像素一批；所有中间量在 int16/uint16 内（见定点倍数说明）。
struct NeonSector
{
  explicit NeonSector(const HueSector & s)
  : lo_a(vdupq_n_s16(s.lo_a)), lo_b(vdupq_n_s16(s.lo_b)),
    hi_a(vdupq_n_s16(s.hi_a)), hi_b(vdupq_n_s16(s.hi_b)),
    ratio(vdupq_n_u16(s.ratio_q)), light_lo(vdupq_n_u8(s.light_lo)), light_hi(vdupq_n_u8(s.light_hi)) {}

  uint16x8_t half(int16x8_t a, int16x8_t b, uint16x8_t light) const
  {
    const auto zero = vdupq_n_s16(0);
    const auto left = vcgeq_s16(vmlsq_s16(vmulq_s16(lo_a, b), lo_b, a), zero);
    const auto right = vcgeq_s16(vmlsq_s16(vmulq_s16(a, hi_b), b, hi_a), zero);
    const auto chroma = vreinterpretq_u16_s16(vaddq_s16(vabsq_s16(a), vabsq_s16(b)));
    const auto vivid = vcgeq_u16(vshlq_n_u16(chroma, kChromaShift), vmulq_u16(ratio, light));
    return vandq_u16(vandq_u16(left, right), vivid);
  }

  uint8x16_t classify(const uint8x16x3_t & lab) const
  {
    const auto offset = vdupq_n_s16(kLabOffset);
    auto centred = [&](uint8x8_t v) {return vsubq_s16(vreinterpretq_s16_u16(vmovl_u8(v)), offset);};
    const auto low = half(centred(vget_low_u8(lab.val[1])), centred(vget_low_u8(lab.val[2])),
      vmovl_u8(vget_low_u8(lab.val[0])));
    const auto high = half(centred(vget_high_u8(lab.val[1])), centred(vget_high_u8(lab.val[2])),
      vmovl_u8(vget_high_u8(lab.val[0])));
    const auto light = vandq_u8(vcgeq_u8(lab.val[0], light_lo), vcleq_u8(lab.val[0], light_hi));
    return vandq_u8(vcombine_u8(vmovn_u16(low), vmovn_u16(high)), light);
  }

  int16x8_t lo_a, lo_b, hi_a, hi_b;
  uint16x8_t ratio;
  uint8x16_t light_lo, light_hi;
};

}  // namespace
#endif

}  // namespace field_detail

cv::Mat FieldLocator::purpleMask(const cv::Mat & bgr, const FieldConfig & cfg)
{
  cv::Mat purple, red;
  segment(bgr, cfg, purple, red);
  return purple;
}

cv::Mat FieldLocator::redMask(const cv::Mat & bgr, const FieldConfig & cfg)
{
  cv::Mat purple, red;
  segment(bgr, cfg, purple, red);
  return red;
}

void FieldLocator::segment(
  const cv::Mat & bgr, const FieldConfig & cfg,
  cv::Mat & purple, cv::Mat & red)
{
  cv::Mat lab;
  cv::cvtColor(bgr, lab, cv::COLOR_BGR2Lab);
  const HueSector purple_sector = HueSector::purple(cfg);
  const HueSector interior_sector = HueSector::interior(cfg);
  purple.create(bgr.size(), CV_8U);
  red.create(bgr.size(), CV_8U);
#if defined(__aarch64__) && defined(__ARM_NEON)
  const NeonSector neon_purple(purple_sector);
  const NeonSector neon_interior(interior_sector);
#endif
  // 行间互不依赖，输出各写各行；约 64K 像素一条带，小 ROI 不做并行调度，不改全局线程数。
  cv::parallel_for_(cv::Range(0, bgr.rows), [&](const cv::Range & rows) {
    for (int y = rows.start; y < rows.end; ++y) {
      const uchar * pixel = lab.ptr<uchar>(y);
      uchar * pp = purple.ptr<uchar>(y);
      uchar * pr = red.ptr<uchar>(y);
      int x = 0;
#if defined(__aarch64__) && defined(__ARM_NEON)
      for (; x <= bgr.cols - 16; x += 16, pixel += 48) {
        const auto channels = vld3q_u8(pixel);
        vst1q_u8(pp + x, neon_purple.classify(channels));
        vst1q_u8(pr + x, neon_interior.classify(channels));
      }
#endif
      for (; x < bgr.cols; ++x, pixel += 3) {
        pp[x] = purple_sector.classify(pixel);
        pr[x] = interior_sector.classify(pixel);
      }
    }
  }, std::max(1.0, static_cast<double>(bgr.total()) / 65536.0));
}

void FieldLocator::segmentEvidence(
  const cv::Mat & bgr, const FieldConfig & cfg, cv::Mat & purple, cv::Mat & interior)
{
  segment(bgr, cfg, purple, interior);
  // 内部颜色是成片平面；紫框与地面的混色边只有一两像素宽，色相会落进内部扇区，开运算把它去掉。
  if (cfg.interior_open_ksize > 1) {
    const cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT,
      cv::Size(cfg.interior_open_ksize, cfg.interior_open_ksize));
    cv::morphologyEx(interior, interior, cv::MORPH_OPEN, kernel);
  }
}

double FieldLocator::redRatioInBev(const cv::Mat & bev, const FieldConfig & cfg)
{
  // 去边后的所选内部颜色比例仅作为结果统计；候选准入仍看搜索图中的网格/外环等完整证据。
  if (bev.empty()) {return 0.0;}
  const int m = static_cast<int>(cfg.bev_border_margin_frac * std::min(bev.cols, bev.rows));
  if (bev.cols - 2 * m < 4 || bev.rows - 2 * m < 4) {return 0.0;}
  cv::Mat roi = bev(cv::Rect(m, m, bev.cols - 2 * m, bev.rows - 2 * m));
  // 所选内部颜色比例按半宽高取样，宽高下限为 16×12；这里只生成统计，不参与候选准入。
  cv::Mat half;
  cv::resize(roi, half, cv::Size(std::max(16, roi.cols / 2), std::max(12, roi.rows / 2)),
               0, 0, cv::INTER_AREA);
  cv::Mat rm = redMask(half, cfg);
  return static_cast<double>(cv::countNonZero(rm)) / static_cast<double>(half.total());
}
