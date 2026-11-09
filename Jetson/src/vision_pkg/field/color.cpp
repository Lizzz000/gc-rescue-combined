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
#include <functional>
#include <vector>
#if defined(__aarch64__) && defined(__ARM_NEON)
#include <arm_neon.h>
#endif

using namespace field_detail;

// ---- 紫框与内部颜色分割 ----
// 联合分割复用一次 Lab 转换，紫色可叠加 HSV 条件；内部颜色由配置选择红或蓝。
// 鸟瞰去边后的占比属于诊断，不能代替候选的完整颜色证据。

namespace field_detail {

// 名称沿用红色版本；实际阈值由 use_blue_interior 一次选定，联合分割与 BEV 统计共用此判定。
struct RedThresholds
{
  explicit RedThresholds(const FieldConfig & cfg)
  : a_min(static_cast<int>(std::lround(cfg.use_blue_interior ? cfg.blue_a_min : cfg.red_a_min)) + 128),
    b_min(static_cast<int>(std::lround(cfg.use_blue_interior ? cfg.blue_b_min : cfg.red_b_min)) + 128),
    b_max(static_cast<int>(std::lround(cfg.use_blue_interior ? cfg.blue_b_max : cfg.red_b_max)) + 128),
    chroma_min(static_cast<int>(std::lround(
      cfg.use_blue_interior ? cfg.blue_chroma_min : cfg.red_chroma_min))),
    light_min(static_cast<int>(std::lround(
      (cfg.use_blue_interior ? cfg.blue_L_min : cfg.red_L_min) * 2.55))) {}

  // 输入为 OpenCV 八位 Lab；a、b 先去掉 128 偏移，色度继续使用既定的 a+abs(b) 判据。
  uchar classify(const uchar * lab) const
  {
    if (lab[1] < a_min || lab[2] < b_min || lab[2] > b_max || lab[0] < light_min) {
      return 0;
    }
    const int a = lab[1] - 128, b = lab[2] - 128;
    return ((b >= 0 ? a + b : a - b) >= chroma_min) ? 255 : 0;
  }

  int a_min, b_min, b_max, chroma_min, light_min;
};

#if defined(__aarch64__) && defined(__ARM_NEON)
// 仅把已经完成的八位 Lab/HSV 分类改成16像素一批；颜色转换和阈值取整仍使用原实现。
// 公开静态入口也可能绕过配置校验，向量表示域之外须回到旧标量比较，不能截断阈值。
bool isByteColourThreshold(int value) {return value >= 0 && value <= 255;}
bool isSignedColourThreshold(int value) {return value >= -32768 && value <= 32767;}

struct NeonLabPixels
{
  explicit NeonLabPixels(const uchar * source)
  : channels(vld3q_u8(source)),
    a_low(vreinterpretq_s16_u16(vmovl_u8(vget_low_u8(channels.val[1])))),
    a_high(vreinterpretq_s16_u16(vmovl_u8(vget_high_u8(channels.val[1])))),
    b_low(vreinterpretq_s16_u16(vmovl_u8(vget_low_u8(channels.val[2])))),
    b_high(vreinterpretq_s16_u16(vmovl_u8(vget_high_u8(channels.val[2])))) {}

  uint8x16x3_t channels;
  int16x8_t a_low, a_high, b_low, b_high;
};

uint8x16_t colourMaskBytes(uint16x8_t low, uint16x8_t high)
{
  // 比较结果只有0或0xffff，取低字节精确得到原二值掩膜的0或255。
  return vcombine_u8(vmovn_u16(low), vmovn_u16(high));
}

struct NeonRedThresholds
{
  explicit NeonRedThresholds(const RedThresholds & scalar)
  : supported(isByteColourThreshold(scalar.a_min) &&
      isByteColourThreshold(scalar.b_min) && isByteColourThreshold(scalar.b_max) &&
      isByteColourThreshold(scalar.light_min) && isSignedColourThreshold(scalar.chroma_min))
  {
    if (!supported) {return;}
    a_min = vdupq_n_u8(static_cast<uchar>(scalar.a_min));
    b_min = vdupq_n_u8(static_cast<uchar>(scalar.b_min));
    b_max = vdupq_n_u8(static_cast<uchar>(scalar.b_max));
    light_min = vdupq_n_u8(static_cast<uchar>(scalar.light_min));
    chroma_min = vdupq_n_s16(static_cast<short>(scalar.chroma_min));
  }

  uint8x16_t classify(const NeonLabPixels & pixels) const
  {
    auto mask = vandq_u8(vcgeq_u8(pixels.channels.val[1], a_min),
      vandq_u8(vcgeq_u8(pixels.channels.val[2], b_min),
      vandq_u8(vcleq_u8(pixels.channels.val[2], b_max),
      vcgeq_u8(pixels.channels.val[0], light_min))));
    const auto center = vdupq_n_s16(128);
    // a、b恢复到[-128,127]，再按原a+abs(b)计算；中间值[-128,255]不会发生16位溢出。
    const auto low = vaddq_s16(vsubq_s16(pixels.a_low, center),
      vabsq_s16(vsubq_s16(pixels.b_low, center)));
    const auto high = vaddq_s16(vsubq_s16(pixels.a_high, center),
      vabsq_s16(vsubq_s16(pixels.b_high, center)));
    return vandq_u8(mask, colourMaskBytes(vcgeq_s16(low, chroma_min),
      vcgeq_s16(high, chroma_min)));
  }

  bool supported;
  uint8x16_t a_min, b_min, b_max, light_min;
  int16x8_t chroma_min;
};

struct NeonPurpleThresholds
{
  NeonPurpleThresholds(int a, int b, int chroma, int light_low, int light_high,
    int saturation, int hue_low, int hue_high)
  : supported(a >= -128 && a <= 127 && b >= -128 && b <= 127 &&
      isSignedColourThreshold(chroma) && isByteColourThreshold(light_low) &&
      isByteColourThreshold(light_high) && isByteColourThreshold(saturation) &&
      isByteColourThreshold(hue_low) && isByteColourThreshold(hue_high))
  {
    if (!supported) {return;}
    a_min = vdupq_n_u8(static_cast<uchar>(a + 128));
    b_max = vdupq_n_u8(static_cast<uchar>(b + 128));
    chroma_min = vdupq_n_s16(static_cast<short>(chroma));
    light_min = vdupq_n_u8(static_cast<uchar>(light_low));
    light_max = vdupq_n_u8(static_cast<uchar>(light_high));
    saturation_min = vdupq_n_u8(static_cast<uchar>(saturation));
    hue_min = vdupq_n_u8(static_cast<uchar>(hue_low));
    hue_max = vdupq_n_u8(static_cast<uchar>(hue_high));
  }

  uint8x16_t classify(
    const NeonLabPixels & pixels, const uchar * hsv, bool use_lab, bool use_hsv) const
  {
    auto mask = vdupq_n_u8(255);
    if (use_lab) {
      // Lab偏移在a-b中抵消；必须先扩展为有符号16位，保留负差值，不能用八位饱和减法。
      const auto chroma = colourMaskBytes(
        vcgeq_s16(vsubq_s16(pixels.a_low, pixels.b_low), chroma_min),
        vcgeq_s16(vsubq_s16(pixels.a_high, pixels.b_high), chroma_min));
      mask = vandq_u8(chroma, vandq_u8(vcgeq_u8(pixels.channels.val[1], a_min),
        vandq_u8(vcleq_u8(pixels.channels.val[2], b_max),
        vandq_u8(vcgeq_u8(pixels.channels.val[0], light_min),
        vcleq_u8(pixels.channels.val[0], light_max)))));
    }
    if (use_hsv) {
      const auto channels = vld3q_u8(hsv);
      mask = vandq_u8(mask, vandq_u8(vcgeq_u8(channels.val[1], saturation_min),
        vandq_u8(vcgeq_u8(channels.val[0], hue_min), vcleq_u8(channels.val[0], hue_max))));
    }
    return mask;
  }

  bool supported;
  uint8x16_t a_min, b_max, light_min, light_max, saturation_min, hue_min, hue_max;
  int16x8_t chroma_min;
};
#endif

}  // namespace field_detail

using namespace field_detail;

// 阶段: 分割 —— 一次 LAB 转换产出紫/红掩膜（可选叠加 HSV 判“紫 vs 蓝”），阈值全部来自 FieldConfig。
cv::Mat FieldLocator::purpleMask(const cv::Mat & bgr, const FieldConfig & cfg)
{
  cv::Mat purple, red;
  segment(bgr, cfg, purple, red);
  return purple;
}

cv::Mat FieldLocator::redMask(const cv::Mat & bgr, const FieldConfig & cfg)
{
  cv::Mat lab;
  cv::cvtColor(bgr, lab, cv::COLOR_BGR2Lab);
  cv::Mat red(bgr.size(), CV_8U);
  const RedThresholds threshold(cfg);
#if defined(__aarch64__) && defined(__ARM_NEON)
  const NeonRedThresholds neon_threshold(threshold);
#endif
  // 行间互不依赖；复用 OpenCV 现有线程池，不修改全局线程数。小幅 BEV 统计保持单条带。
  cv::parallel_for_(cv::Range(0, bgr.rows), [&](const cv::Range & rows) {
    for (int y = rows.start; y < rows.end; ++y) {
      const uchar * pixel = lab.ptr<uchar>(y);
      uchar * output = red.ptr<uchar>(y);
      int x = 0;
#if defined(__aarch64__) && defined(__ARM_NEON)
      if (neon_threshold.supported) {
        for (; x <= bgr.cols - 16; x += 16, pixel += 48) {
          vst1q_u8(output + x, neon_threshold.classify(NeonLabPixels(pixel)));
        }
      }
#endif
      for (; x < bgr.cols; ++x, pixel += 3) {
        output[x] = threshold.classify(pixel);
      }
    }
  }, std::max(1.0, static_cast<double>(bgr.total()) / 65536.0));
  return red;
}

// 一次 LAB 转换同时算出紫色和红色掩膜; 紫色是否叠加 HSV 由 seg_mode 决定。
void FieldLocator::segment(
  const cv::Mat & bgr, const FieldConfig & cfg,
  cv::Mat & purple, cv::Mat & red)
{
    // 加颜色/阈值分支：要同时在 FieldConfig 加字段、节点参数映射、统一配置并补 validateFieldConfig。
    // LAB 8bit 编码: L' = L*255/100, a' = a+128, b' = b+128。
    // 色度用 L1 近似（紫色 a>0,b<0 时为 a-b），它与 L2 数值不同，阈值按当前近似定义调节。
  cv::Mat lab, hsv;
  cv::cvtColor(bgr, lab, cv::COLOR_BGR2Lab);
  const bool use_lab = (cfg.seg_mode != static_cast<int>(SegMode::HSV_ONLY));
  // HSV 的 S 用 0..255、H 用 0..180；配置中饱和度是百分比，色相已是 OpenCV 的 H 单位。
  const bool use_hsv = (cfg.purple_use_hue && cfg.seg_mode != static_cast<int>(SegMode::LAB_ONLY));
  if (use_hsv) {cv::cvtColor(bgr, hsv, cv::COLOR_BGR2HSV);}

  const int pa_min = static_cast<int>(std::lround(cfg.purple_a_min));
  const int pb_max = static_cast<int>(std::lround(cfg.purple_b_max));
  const int pc_min = static_cast<int>(std::lround(cfg.purple_chroma_min));
  const int pL_lo = static_cast<int>(std::lround(cfg.purple_L_min * 2.55));
  const int pL_hi = static_cast<int>(std::lround(cfg.purple_L_max * 2.55));
  const int ps_min = static_cast<int>(std::lround(cfg.purple_sat_min * 2.55));
  const int ph_lo = static_cast<int>(std::lround(cfg.purple_hue_min));
  const int ph_hi = static_cast<int>(std::lround(cfg.purple_hue_max));

  const RedThresholds red_threshold(cfg);

  // 后续会覆盖每个输出像素，因此无需先清零整张掩膜。
  // 生产调用提供独立输出，只写 0/255 二值证据；公开辅助入口也保留共享输出底图的旧写入顺序。
  purple.create(bgr.size(), CV_8U);
  red.create(bgr.size(), CV_8U);
  // 不能只比较Mat::u：外部数据头可以没有u，错位ROI也可能让不同输出行写到同一字节。
  // 首/末有效字节组成保守范围（包含行间隙）；共享输出须同时禁用并行和批量写入。
  const std::less<const uchar *> before;
  const auto * purple_end = purple.ptr<uchar>(purple.rows - 1) + purple.cols;
  const auto * red_end = red.ptr<uchar>(red.rows - 1) + red.cols;
  const bool overlap = before(purple.data, red_end) && before(red.data, purple_end);
#if defined(__aarch64__) && defined(__ARM_NEON)
  const NeonRedThresholds neon_red(red_threshold);
  const NeonPurpleThresholds neon_purple(pa_min, pb_max, pc_min, pL_lo, pL_hi, ps_min, ph_lo, ph_hi);
  const bool vectorize = !overlap && neon_red.supported && neon_purple.supported;
#endif
  // 独立输出按行分工，输入只读，每个输出字节由唯一行写入；像素整数比较与旧实现相同。
  const auto classify_rows = [&](const cv::Range & rows) {
    for (int y = rows.start; y < rows.end; ++y) {
      const uchar * pl = lab.ptr<uchar>(y);
      const uchar * ph = use_hsv ? hsv.ptr<uchar>(y) : nullptr;
      uchar * pp = purple.ptr<uchar>(y);
      uchar * pr = red.ptr<uchar>(y);
      int x = 0;
#if defined(__aarch64__) && defined(__ARM_NEON)
      if (vectorize) {
        for (; x <= bgr.cols - 16; x += 16, pl += 48) {
          const NeonLabPixels pixels(pl);
          vst1q_u8(pp + x, neon_purple.classify(pixels, use_hsv ? ph + 3 * x : nullptr,
            use_lab, use_hsv));
          vst1q_u8(pr + x, neon_red.classify(pixels));
        }
      }
#endif
      for (; x < bgr.cols; ++x, pl += 3) {
        bool ok;
        const int a = pl[1] - 128, b = pl[2] - 128;
        if (use_lab) {
          ok = a >= pa_min && b <= pb_max && (a - b) >= pc_min &&
            pl[0] >= pL_lo && pl[0] <= pL_hi;
        } else {
          ok = true;  // 仅 HSV 模式的紫色判定仍由下方色相/饱和度完成。
        }
        if (ok && use_hsv) {
          const uchar * p = ph + 3 * x;
          ok = p[1] >= ps_min && p[0] >= ph_lo && p[0] <= ph_hi;
        }
        pp[x] = ok ? 255 : 0;

        // 内部色始终沿用 Lab 判据，即使紫色使用 HSV_ONLY。
        pr[x] = red_threshold.classify(pl);
      }
    }
  };
  if (overlap) {
    classify_rows(cv::Range(0, bgr.rows));
  } else {
    // 约 64K 像素一条带，避免给小 ROI 增加并行调度；不改变全局线程数或业务线程。
    cv::parallel_for_(cv::Range(0, bgr.rows), classify_rows,
      std::max(1.0, static_cast<double>(bgr.total()) / 65536.0));
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
