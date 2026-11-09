#include "camera/neon/i420.hpp"

#include <opencv2/imgproc.hpp>

// ---- 相机色转 ----
// 职责：全范围 I420 到 BGR，只读输入、独占输出。
// Original OpenCV 4.10.0 color_yuv.simd.hpp notice (retained verbatim):
// This file is part of OpenCV project.
// It is subject to the license terms in the LICENSE file found in the top-level directory
// of this distribution and at http://opencv.org/license.html
//
// Adapted portions are distributed under Apache-2.0; the complete upstream license
// is retained in camera/neon/LICENSE-OpenCV.txt. Upstream license: camera/neon/LICENSE-OpenCV.txt.
// Copyright 2026 jetson_yolo integration
// SPDX-License-Identifier: Apache-2.0
// 相机私有融合实现：范围映射与 I420 -> BGR 合并遍历，输入只读、输出独占。
// 定点系数/舍入来自 OpenCV 4.10.0 color_yuv.simd.hpp；实际库输出仍是测试判据。
#include <opencv2/core.hpp>
#include <algorithm>
#include <array>
#include <cstdint>
#if defined(__aarch64__)
#include <arm_neon.h>
#endif

// Copyright 2026 jetson_yolo integration
// SPDX-License-Identifier: Apache-2.0
// 相机私有的8位单通道查表：AArch64直接逐行执行，不占用Field共用的OpenCV并行调度。
// 表值仍由原转换代码生成；这里只替换读取方式，不近似颜色公式或修改输入像素。

namespace vision_camera_detail {

inline void applyByteLut(const cv::Mat & source, const cv::Mat & table, cv::Mat & output)
{
#if defined(__aarch64__)
  CV_Assert(source.type() == CV_8UC1 && source.dims <= 2 &&
    table.type() == CV_8UC1 && table.total() == 256 && table.isContinuous());
  output.create(source.size(), CV_8UC1);
  const auto * values = table.ptr<unsigned char>();
  // 256项分为4组，每组占4个NEON寄存器。TBL越界返回0；减去组起点后，
  // 每个8位索引恰好只在一组落入0..63，四组按位或就是原表的精确字节。
  const uint8x16x4_t group0{{vld1q_u8(values), vld1q_u8(values + 16),
      vld1q_u8(values + 32), vld1q_u8(values + 48)}};
  const uint8x16x4_t group1{{vld1q_u8(values + 64), vld1q_u8(values + 80),
      vld1q_u8(values + 96), vld1q_u8(values + 112)}};
  const uint8x16x4_t group2{{vld1q_u8(values + 128), vld1q_u8(values + 144),
      vld1q_u8(values + 160), vld1q_u8(values + 176)}};
  const uint8x16x4_t group3{{vld1q_u8(values + 192), vld1q_u8(values + 208),
      vld1q_u8(values + 224), vld1q_u8(values + 240)}};
  for (int row = 0; row < source.rows; ++row) {
    const auto * input = source.ptr<unsigned char>(row);
    auto * destination = output.ptr<unsigned char>(row);
    int column = 0;
    for (; column <= source.cols - 16; column += 16) {
      const auto indices = vld1q_u8(input + column);
      const auto first = vqtbl4q_u8(group0, indices);
      const auto second = vqtbl4q_u8(group1, vsubq_u8(indices, vdupq_n_u8(64)));
      const auto third = vqtbl4q_u8(group2, vsubq_u8(indices, vdupq_n_u8(128)));
      const auto fourth = vqtbl4q_u8(group3, vsubq_u8(indices, vdupq_n_u8(192)));
      vst1q_u8(destination + column, vorrq_u8(vorrq_u8(first, second), vorrq_u8(third, fourth)));
    }
    // 行跨度可能大于可见宽度；尾部仅访问本行可见字节，不能越界借用下一行。
    for (; column < source.cols; ++column) {
      destination[column] = values[input[column]];
    }
  }
#else
  cv::LUT(source, table, output);
#endif
}

}  // namespace vision_camera_detail

namespace vision_camera_detail::fused_i420 {
constexpr int kY = 1220542, kUB = 2116026, kUG = -409993, kVG = -852492, kVR = 1673527;
constexpr int kRound = 1 << 19, kShift = 20;

struct RangeTables {
  std::array<unsigned char, 256> luma{}, chroma{};
  bool arithmetic_equivalent{true};
  RangeTables()
  {
    for (int value = 0; value < 256; ++value) {
      // 保留原表的完整表达式，核对当前编译/舍入环境，不仅靠数学推导。
      luma[value] = cv::saturate_cast<unsigned char>(16.0 + value * (219.0 / 255.0));
      chroma[value] = cv::saturate_cast<unsigned char>(
        128.0 + (value - 128) * (224.0 / 255.0));
      arithmetic_equivalent = arithmetic_equivalent &&
        luma[value] == 16 + (219 * value + 127) / 255 &&
        chroma[value] == (224 * value + 4095) / 255;
    }
  }
};

inline const RangeTables & tables()
{
  static const RangeTables result;
  return result;
}

inline bool nativeNeonEnabled()
{
#if defined(__aarch64__)
  return tables().arithmetic_equivalent;
#else
  return false;
#endif
}

// 原路径逐行复制到连续 limited；此处以相同逻辑字节序跳过输入行尾填充。
// height%4==2 时，V 从某一物理行的后半段开始。
inline const unsigned char * packedPointer(const cv::Mat & input, std::size_t offset)
{
  return input.ptr<unsigned char>(static_cast<int>(offset / input.cols)) +
         offset % static_cast<std::size_t>(input.cols);
}

inline unsigned char channel(int value)
{
  // 显式 floor 除法，避免依赖 C++17 对负整数右移的实现约定。
  const int shifted = value >= 0 ? value >> kShift :
    -static_cast<int>((-static_cast<std::int64_t>(value) + (1 << kShift) - 1) >> kShift);
  return cv::saturate_cast<unsigned char>(shifted);
}

inline void scalarPixel(
  unsigned char y, int red, int green, int blue, unsigned char * destination,
  const RangeTables & mapping)
{
  const int scaled = std::max(0, static_cast<int>(mapping.luma[y]) - 16) * kY;
  destination[0] = channel(scaled + blue);
  destination[1] = channel(scaled + green);
  destination[2] = channel(scaled + red);
}

#if defined(__aarch64__)
inline uint16x8_t divide255(uint16x8_t value)
{
  // n 最大 61215，n+1+(n>>8) 不溢出 uint16；该区间严格等于 floor(n/255)。
  return vshrq_n_u16(vaddq_u16(vaddq_u16(value, vdupq_n_u16(1)),
                            vshrq_n_u16(value, 8)), 8);
}

inline uint16x8_t lumaDelta(uint8x8_t value)
{
  return divide255(vaddq_u16(vmull_u8(value, vdup_n_u8(219)), vdupq_n_u16(127)));
}

inline int16x8_t chromaDelta(uint8x8_t value)
{
  return vsubq_s16(vreinterpretq_s16_u16(
    divide255(vaddq_u16(vmull_u8(value, vdup_n_u8(224)), vdupq_n_u16(4095)))),
    vdupq_n_s16(128));
}

struct ColourTerms {
  std::array<int32x4_t, 4> red, green, blue;
};

inline ColourTerms colourTerms(const unsigned char * u, const unsigned char * v)
{
  const auto uu = chromaDelta(vld1_u8(u));
  const auto vv = chromaDelta(vld1_u8(v));
  const std::array<int32x4_t, 2> us{
    vmovl_s16(vget_low_s16(uu)), vmovl_s16(vget_high_s16(uu))};
  const std::array<int32x4_t, 2> vs{
    vmovl_s16(vget_low_s16(vv)), vmovl_s16(vget_high_s16(vv))};
  ColourTerms result;
  for (int part = 0; part < 2; ++part) {
    const auto red = vmlaq_n_s32(vdupq_n_s32(kRound), vs[part], kVR);
    const auto green = vmlaq_n_s32(
      vmlaq_n_s32(vdupq_n_s32(kRound), vs[part], kVG), us[part], kUG);
    const auto blue = vmlaq_n_s32(vdupq_n_s32(kRound), us[part], kUB);
    // 每个色度值覆盖相邻两列，并在上下两行共享。
    result.red[part * 2] = vzip1q_s32(red, red);
    result.red[part * 2 + 1] = vzip2q_s32(red, red);
    result.green[part * 2] = vzip1q_s32(green, green);
    result.green[part * 2 + 1] = vzip2q_s32(green, green);
    result.blue[part * 2] = vzip1q_s32(blue, blue);
    result.blue[part * 2 + 1] = vzip2q_s32(blue, blue);
  }
  return result;
}

inline uint8x16_t neonChannel(
  const std::array<int32x4_t, 4> & scaled, const std::array<int32x4_t, 4> & terms)
{
  const auto first = vcombine_s16(
    vqmovn_s32(vshrq_n_s32(vaddq_s32(scaled[0], terms[0]), kShift)),
    vqmovn_s32(vshrq_n_s32(vaddq_s32(scaled[1], terms[1]), kShift)));
  const auto second = vcombine_s16(
    vqmovn_s32(vshrq_n_s32(vaddq_s32(scaled[2], terms[2]), kShift)),
    vqmovn_s32(vshrq_n_s32(vaddq_s32(scaled[3], terms[3]), kShift)));
  return vcombine_u8(vqmovun_s16(first), vqmovun_s16(second));
}

inline void neonRow(
  const unsigned char * luma, const ColourTerms & terms, unsigned char * destination)
{
  const auto y = vld1q_u8(luma);
  const auto first = lumaDelta(vget_low_u8(y));
  const auto second = lumaDelta(vget_high_u8(y));
  const std::array<int32x4_t, 4> scaled{
    vmulq_n_s32(vreinterpretq_s32_u32(vmovl_u16(vget_low_u16(first))), kY),
    vmulq_n_s32(vreinterpretq_s32_u32(vmovl_u16(vget_high_u16(first))), kY),
    vmulq_n_s32(vreinterpretq_s32_u32(vmovl_u16(vget_low_u16(second))), kY),
    vmulq_n_s32(vreinterpretq_s32_u32(vmovl_u16(vget_high_u16(second))), kY)};
  const uint8x16x3_t bgr{{
    neonChannel(scaled, terms.blue), neonChannel(scaled, terms.green),
    neonChannel(scaled, terms.red)}};
  vst3q_u8(destination, bgr);
}
#endif

inline cv::Mat convert(const cv::Mat & input)
{
  if (input.empty() || input.type() != CV_8UC1 || input.rows % 3 != 0 ||
    input.cols % 2 != 0)
  {
    CV_Error(cv::Error::StsBadArg, "expected packed 8-bit I420 with even width and height");
  }
  const int height = input.rows / 3 * 2, width = input.cols;
  const auto & mapping = tables();
  cv::Mat output(height, width, CV_8UC3);
  const auto luma_bytes = static_cast<std::size_t>(height) * width;
  const auto work = [&](const cv::Range & range) {
      for (int pair = range.start; pair < range.end; ++pair) {
        const auto chroma_offset = static_cast<std::size_t>(pair) * (width / 2);
        const auto * u = packedPointer(input, luma_bytes + chroma_offset);
        const auto * v = packedPointer(input, luma_bytes + luma_bytes / 4 + chroma_offset);
        const auto * y0 = input.ptr<unsigned char>(pair * 2);
        const auto * y1 = input.ptr<unsigned char>(pair * 2 + 1);
        auto * out0 = output.ptr<unsigned char>(pair * 2);
        auto * out1 = output.ptr<unsigned char>(pair * 2 + 1);
        int x = 0;
#if defined(__aarch64__)
        if (mapping.arithmetic_equivalent) {
          for (; x <= width - 16; x += 16) {
            const auto terms = colourTerms(u + x / 2, v + x / 2);
            neonRow(y0 + x, terms, out0 + x * 3);
            neonRow(y1 + x, terms, out1 + x * 3);
          }
        }
#endif
        for (; x < width; x += 2) {
          const int uu = static_cast<int>(mapping.chroma[u[x / 2]]) - 128;
          const int vv = static_cast<int>(mapping.chroma[v[x / 2]]) - 128;
          const int red = kRound + kVR * vv;
          const int green = kRound + kVG * vv + kUG * uu;
          const int blue = kRound + kUB * uu;
          scalarPixel(y0[x], red, green, blue, out0 + x * 3, mapping);
          scalarPixel(y0[x + 1], red, green, blue, out0 + (x + 1) * 3, mapping);
          scalarPixel(y1[x], red, green, blue, out1 + x * 3, mapping);
          scalarPixel(y1[x + 1], red, green, blue, out1 + (x + 1) * 3, mapping);
        }
      }
    };
  // 与原 OpenCV 色转相同的阈值/行对范围；不建线程、不改变全局线程数。
  if (static_cast<std::size_t>(width) * height >= 320u * 240u) {
    cv::parallel_for_(cv::Range(0, height / 2), work);
  } else {
    work(cv::Range(0, height / 2));
  }
  return output;
}
}  // namespace vision_camera_detail::fused_i420

namespace vision_camera {

// NVDEC 的 JPEG I420 像素使用全范围；OpenCV 的 I420 转换按视频范围解释。
// ARM在表值严格一致时融合范围映射与色转，省掉中间I420的整图写入/重读。
// 其他平台或特殊舍入环境继续使用原查表+OpenCV路径；输入像素始终只读。
cv::Mat fullRangeI420ToBgr(const cv::Mat & image)
{
  if (image.empty() || image.type() != CV_8UC1 || image.rows % 3 != 0 ||
    image.cols % 2 != 0)
  {
    CV_Error(cv::Error::StsBadArg, "expected packed 8-bit I420 with even width and height");
  }
#if defined(__aarch64__)
  if (vision_camera_detail::fused_i420::nativeNeonEnabled()) {
    return vision_camera_detail::fused_i420::convert(image);
  }
#endif
  const int height = image.rows / 3 * 2;
  // 范围表只在 本文件 维护一份：NEON 路径与查表路径共用同一组字节，避免公式漂移。
  const auto & mapping = vision_camera_detail::fused_i420::tables();
  cv::Mat luma_table(1, 256, CV_8UC1,
    const_cast<unsigned char *>(mapping.luma.data()));
  cv::Mat chroma_table(1, 256, CV_8UC1,
    const_cast<unsigned char *>(mapping.chroma.data()));
  cv::Mat limited(image.rows, image.cols, CV_8UC1);
  cv::Mat luma = limited.rowRange(0, height);
  cv::Mat chroma = limited.rowRange(height, limited.rows);
  // AArch64查表只用当前采集线程，避免两次LUT抢占Field共用的OpenCV并行区。
  // 非ARM仍调用原cv::LUT；表值与后续I420转换保持不变。
  vision_camera_detail::applyByteLut(image.rowRange(0, height), luma_table, luma);
  vision_camera_detail::applyByteLut(image.rowRange(height, image.rows), chroma_table, chroma);
  cv::Mat bgr;
  cv::cvtColor(limited, bgr, cv::COLOR_YUV2BGR_I420);
  return bgr;
}

}  // namespace vision_camera
