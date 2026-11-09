#pragma once
// ---- 图传叠层绘制（OpenCV）----
// 职责：着色、编码和 HTTP 预览。
// Copyright 2026 jetson_yolo integration
// SPDX-License-Identifier: Apache-2.0
// 把检测框、场地几何与掩膜消息画成 BGR 像素的对外接口（只依赖 ROS 消息与 OpenCV，可独立进程运行）。
// 绘图只属于图传展示层，不在推理线程执行；输入使用 ROS 消息，因此无需链接推理后端。


#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include <opencv2/core.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>

#include "interface_pkg/msg/field_observation.hpp"

/// 256 项 BGR 标签颜色表；0 是背景，同一标签跨帧使用相同颜色。
const cv::Mat & labelLut();

/// 将标签图叠加到 BGR 画布；整图最近邻缩放保留类别边界，仅混合非背景包围框。
/// canvas 可写，label_mask 只读；全背景不修改画布，两者均支持有行步长的视图。
void overlayLabelMask(cv::Mat & canvas, const cv::Mat & label_mask);

/// 绘制分割框与可选标签。0–3 显示为 green/bright_blue/black/orange 加置信度百分比。
/// 坐标已属于当前画布；本展示函数不执行跟踪、目标选择或消息重投影。
void drawDetectionArray(
  cv::Mat & canvas,
  const vision_msgs::msg::Detection2DArray & detections,
  bool with_labels = true);

/// 绘制场地几何。无效观测只留 SAFE INVALID；有效观测区分 LEFT/RIGHT，并标出是否 committed。
void drawFieldObservation(
  cv::Mat & canvas,
  const interface_pkg::msg::FieldObservation & observation);

/// 左上角显示状态文字和半透明底板，供观看端区分静止场景与处理状态。
void drawStats(cv::Mat & canvas, const std::string & text);


// Copyright 2026 jetson_yolo integration
// SPDX-License-Identifier: Apache-2.0
// 场地染色的私有实现：保留 OpenCV 多边形光栅和逐次叠加顺序，减少整块 BGR 临时量读写。
#include <opencv2/imgproc.hpp>

#include <array>
#include <cstdint>
#include <vector>

#if defined(__aarch64__) && defined(__ARM_NEON)
#include <arm_neon.h>
#endif

namespace vision_stream_detail
{

struct PolygonBlendTable
{
  std::array<cv::Vec3b, 256> values;
  bool background_is_identity = true;
  std::array<std::uint16_t, 3> integer_bias{};
  bool integer_is_equivalent = false;
};

inline std::uint32_t polygonDivide25(std::uint32_t numerator)
{
  return (numerator * 5243U) >> 17U;
}

inline bool polygonDivisionIsExact()
{
  // N = 18*x + 7*c + 12，字节输入使 N 落在 0..6387；初始化穷尽核对，不能依赖近似除法。
  static const bool exact = [] {
      for (std::uint32_t numerator = 0; numerator <= 6387U; ++numerator) {
        if (polygonDivide25(numerator) != numerator / 25U) {return false;}
      }
      return true;
    }();
  return exact;
}

inline bool polygonIntegerBlendIsEquivalent(
  const PolygonBlendTable & table, const cv::Scalar & colour,
  std::array<std::uint16_t, 3> & bias)
{
  // 只为已有两种整数色建立快路；任意 Scalar（含负数、NaN、越界值）都不做无符号转换。
  if (colour == cv::Scalar(255, 190, 40)) {
    bias = {7U * 255U + 12U, 7U * 190U + 12U, 7U * 40U + 12U};
  } else if (colour == cv::Scalar(40, 140, 255)) {
    bias = {7U * 40U + 12U, 7U * 140U + 12U, 7U * 255U + 12U};
  } else {
    return false;
  }
  if (!table.background_is_identity || !polygonDivisionIsExact()) {return false;}
  // OpenCV 后端取整行为是最终依据；每个通道的全部输入字节一致才启用整数向量混合。
  for (unsigned int value = 0; value < 256U; ++value) {
    for (int channel = 0; channel < 3; ++channel) {
      if (polygonDivide25(18U * value + bias[channel]) != table.values[value][channel]) {
        return false;
      }
    }
  }
  return true;
}

inline PolygonBlendTable polygonBlendTable(const cv::Scalar & colour)
{
  cv::Mat source(1, 256, CV_8UC3);
  for (int value = 0; value < 256; ++value) {
    source.at<cv::Vec3b>(0, value) = cv::Vec3b(value, value, value);
  }
  const cv::Mat tint(source.size(), source.type(), colour);
  cv::Mat blended, unchanged;
  cv::addWeighted(source, 0.72, tint, 0.28, 0.0, blended);
  cv::addWeighted(source, 0.72, source, 0.28, 0.0, unchanged);
  PolygonBlendTable result;
  for (int value = 0; value < 256; ++value) {
    result.values[value] = blended.at<cv::Vec3b>(0, value);
    for (int channel = 0; channel < 3; ++channel) {
      if (unchanged.at<cv::Vec3b>(0, value)[channel] != value) {
        result.background_is_identity = false;
      }
    }
  }
  result.integer_is_equivalent = polygonIntegerBlendIsEquivalent(
    result, colour, result.integer_bias);
  return result;
}

inline const PolygonBlendTable * cachedPolygonBlend(const cv::Scalar & colour)
{
  // 只缓存生产现有两色；其他颜色继续使用原 OpenCV 路径，不引入可增长的全局缓存。
  if (colour == cv::Scalar(255, 190, 40)) {
    static const auto left = polygonBlendTable(colour);
    return &left;
  }
  if (colour == cv::Scalar(40, 140, 255)) {
    static const auto right = polygonBlendTable(colour);
    return &right;
  }
  return nullptr;
}

#if defined(__aarch64__) && defined(__ARM_NEON)
inline uint16x8_t polygonDivide25Neon(uint16x8_t numerator)
{
  const auto low = vmull_n_u16(vget_low_u16(numerator), 5243U);
  const auto high = vmull_n_u16(vget_high_u16(numerator), 5243U);
  // 乘积不超过 uint32；先宽位移再窄化，因为 vshrn_n_u32 的移位立即数不能取 17。
  return vcombine_u16(
    vmovn_u32(vshrq_n_u32(low, 17)), vmovn_u32(vshrq_n_u32(high, 17)));
}

inline uint8x16_t polygonBlend16Neon(uint8x16_t source, std::uint16_t bias)
{
  const auto low = vmlaq_n_u16(
    vdupq_n_u16(bias), vmovl_u8(vget_low_u8(source)), 18U);
  const auto high = vmlaq_n_u16(
    vdupq_n_u16(bias), vmovl_u8(vget_high_u8(source)), 18U);
  return vcombine_u8(
    vmovn_u16(polygonDivide25Neon(low)), vmovn_u16(polygonDivide25Neon(high)));
}
#endif

inline void applyPolygonBlend(
  cv::Mat & roi, const cv::Mat & coverage, const PolygonBlendTable & table)
{
  // 调用者已限定 CV_8UC3 和同尺寸 LINE_8 掩膜；按行处理，兼容父图 ROI 与外部步长。
  for (int row = 0; row < roi.rows; ++row) {
    const auto * covered = coverage.ptr<unsigned char>(row);
    auto * pixels = roi.ptr<cv::Vec3b>(row);
    int column = 0;
#if defined(__aarch64__) && defined(__ARM_NEON)
    if (table.integer_is_equivalent) {
      for (; column <= roi.cols - 16; column += 16) {
        const auto mask = vld1q_u8(covered + column);
        auto * bytes = reinterpret_cast<unsigned char *>(pixels + column);
        auto source = vld3q_u8(bytes);
        for (int channel = 0; channel < 3; ++channel) {
          source.val[channel] = vbslq_u8(
            mask, polygonBlend16Neon(source.val[channel], table.integer_bias[channel]),
            source.val[channel]);
        }
        vst3q_u8(bytes, source);
      }
    }
#endif
    // 整数守卫失败、非 ARM 平台及向量尾部继续使用既有 OpenCV 表值。
    for (; column < roi.cols; ++column) {
      if (covered[column] == 0) {continue;}
      auto & pixel = pixels[column];
      for (int channel = 0; channel < 3; ++channel) {
        pixel[channel] = table.values[pixel[channel]][channel];
      }
    }
  }
}

inline void tintPolygon(
  cv::Mat & canvas, const std::vector<cv::Point> & points, const cv::Scalar & colour)
{
  if (points.size() < 3) {return;}
  const cv::Rect bounds = cv::boundingRect(points) & cv::Rect(0, 0, canvas.cols, canvas.rows);
  if (bounds.area() <= 0) {return;}
  std::vector<cv::Point> local;
  local.reserve(points.size());
  for (const auto & point : points) {
    local.emplace_back(point.x - bounds.x, point.y - bounds.y);
  }
  cv::Mat roi = canvas(bounds);
  const auto * table = canvas.type() == CV_8UC3 ? cachedPolygonBlend(colour) : nullptr;
  if (table == nullptr || !table->background_is_identity) {
    // 保留原辅助函数对灰度、其他深度/颜色的处理及 OpenCV 异常，不扩大快路类型范围。
    cv::Mat tint = roi.clone();
    cv::fillPoly(tint, std::vector<std::vector<cv::Point>>{local}, colour);
    cv::addWeighted(roi, 0.72, tint, 0.28, 0.0, roi);
    return;
  }

  // LINE_8 的覆盖位置与原三通道 fillPoly 相同；灰度掩膜只记录覆盖，不改变边缘透明度。
  cv::Mat coverage = cv::Mat::zeros(roi.size(), CV_8UC1);
  cv::fillPoly(coverage, std::vector<std::vector<cv::Point>>{local}, cv::Scalar(255));
  applyPolygonBlend(roi, coverage, *table);
}

}  // namespace vision_stream_detail
