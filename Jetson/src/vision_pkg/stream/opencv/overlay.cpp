#include "stream/opencv/overlay.hpp"

// ---- 图传叠层绘制实现 ----
// Copyright 2026 jetson_yolo integration
// SPDX-License-Identifier: Apache-2.0
// 把检测框、场地几何与分割掩膜画到 BGR 画布上（展示层：只读入参、原地改 canvas）。


#include <algorithm>
#include <memory>
#include <vector>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdio>
#include <limits>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

static cv::Scalar colourForId(int id)
{
  // 颜色表只构建一次，避免逐框分配小图和执行颜色空间转换。
  static const std::array<cv::Scalar, 180> palette = [] {
      std::array<cv::Scalar, 180> values;
      cv::Mat hsv(1, 180, CV_8UC3);
      for (int hue = 0; hue < 180; ++hue) {
        hsv.at<cv::Vec3b>(0, hue) = cv::Vec3b(hue, 255, 255);
      }
      cv::Mat bgr;
      cv::cvtColor(hsv, bgr, cv::COLOR_HSV2BGR);
      for (int hue = 0; hue < 180; ++hue) {
        const auto value = bgr.at<cv::Vec3b>(0, hue);
        values[hue] = cv::Scalar(value[0], value[1], value[2]);
      }
      return values;
    }();
  const int hue = ((id % 180) + 180) % 180;
  return palette[hue];
}

// 按画布短边缩放框和标签，限制小图与大图的字号范围。
static double overlayScale(const cv::Mat & canvas)
{
  const int shorter = std::min(canvas.cols, canvas.rows);
  return std::clamp(static_cast<double>(shorter) / 900.0, 0.55, 1.45);
}

// OpenCV Hershey 字体不支持中文；类别和安全区状态使用英文短标签。
static void putLabel(
  cv::Mat & canvas, const cv::Rect & box, const std::string & text,
  const cv::Scalar & colour, double font_scale)
{
  const int thickness = font_scale >= 1.0 ? 2 : 1;
  int baseline = 0;
  const cv::Size extent =
    cv::getTextSize(text, cv::FONT_HERSHEY_SIMPLEX, font_scale, thickness, &baseline);
  // 优先把标签放在框上方，空间不足时放进框内，并裁到画布范围。
  const int top = (box.y - extent.height - 6 >= 0) ? box.y - extent.height - 6 : box.y + 2;
  const cv::Rect plate(box.x, top, extent.width + 10, extent.height + 8);
  const cv::Rect clipped = plate & cv::Rect(0, 0, canvas.cols, canvas.rows);
  if (clipped.area() <= 0) {return;}
  cv::rectangle(canvas, clipped, cv::Scalar(12, 16, 12), cv::FILLED);
  const cv::Rect bar = cv::Rect(clipped.x, clipped.y, std::min(4, clipped.width), clipped.height) &
    cv::Rect(0, 0, canvas.cols, canvas.rows);
  if (bar.area() > 0) {cv::rectangle(canvas, bar, colour, cv::FILLED);}
  cv::putText(canvas, text, cv::Point(clipped.x + 6, clipped.y + extent.height + 3),
    cv::FONT_HERSHEY_SIMPLEX, font_scale, cv::Scalar(245, 245, 245), thickness,
    cv::LINE_AA);
}

// 0–3 与 classes.txt 一致。未知编号保留原文本，颜色仍由名称哈希决定。
static const char * knownClassName(const std::string & id, cv::Scalar & colour)
{
  if (id == "0") {
    colour = cv::Scalar(40, 190, 40);
    return "green";
  }
  if (id == "1") {
    colour = cv::Scalar(235, 150, 20);
    return "bright_blue";
  }
  if (id == "2") {
    colour = cv::Scalar(230, 230, 230);
    return "black";
  }
  if (id == "3") {
    colour = cv::Scalar(20, 140, 255);
    return "orange";
  }
  return nullptr;
}

// 掩膜配色：256 项颜色表在此，叠加混合比例在 overlayLabelMask()。
const cv::Mat & labelLut()
{
  static const cv::Mat lut = [] {
      cv::Mat table(256, 1, CV_8UC3, cv::Scalar(0, 0, 0));
      for (int i = 1; i < 256; ++i) {
        const cv::Scalar colour = colourForId(i * 47);
        table.at<cv::Vec3b>(i, 0) = cv::Vec3b(static_cast<unsigned char>(colour[0]),
                                            static_cast<unsigned char>(colour[1]),
                                            static_cast<unsigned char>(colour[2]));
      }
      return table;
    }();
  return lut;
}

void overlayLabelMask(cv::Mat & canvas, const cv::Mat & label_mask)
{
  if (canvas.empty() || label_mask.empty() || label_mask.type() != CV_8UC1) {return;}

  cv::Rect bounds = cv::boundingRect(label_mask);
  if (bounds.empty()) {return;}
  cv::Mat upscaled;
  if (label_mask.size() == canvas.size()) {
    upscaled = label_mask;
  } else {
    // 保持整张标签图的最近邻坐标；裁源 ROI 再 resize 会改变类别边界。
    cv::resize(label_mask, upscaled, canvas.size(), 0, 0, cv::INTER_NEAREST);
    bounds = cv::boundingRect(upscaled);
    if (bounds.empty()) {return;}  // 缩小时原图的非零像素可能没有被采样。
  }

  // 非零包围框外都是背景；逐行访问继续支持 mask 和 canvas 的非连续 ROI。
  // 混合查表只构建一次，并由 OpenCV 生成以保留 addWeighted 的取整结果。
  static const cv::Mat blend_table = [] {
      cv::Mat source(256, 256, CV_8UC3);
      cv::Mat colour(256, 256, CV_8UC3);
      const auto * palette = labelLut().ptr<cv::Vec3b>();
      for (int label = 0; label < 256; ++label) {
        auto * values = source.ptr<cv::Vec3b>(label);
        auto * colours = colour.ptr<cv::Vec3b>(label);
        for (int value = 0; value < 256; ++value) {
          values[value] = cv::Vec3b(value, value, value);
          colours[value] = palette[label];
        }
      }
      cv::Mat table;
      cv::addWeighted(source, 0.55, colour, 0.45, 0.0, table);
      return table;
    }();
  for (int y = bounds.y; y < bounds.y + bounds.height; ++y) {
    const auto * labels = upscaled.ptr<unsigned char>(y);
    auto * pixels = canvas.ptr<cv::Vec3b>(y);
    for (int x = bounds.x; x < bounds.x + bounds.width; ++x) {
      const auto label = labels[x];
      if (label == 0) {continue;}
      const auto * blend = blend_table.ptr<cv::Vec3b>(label);
      for (int channel = 0; channel < 3; ++channel) {
        pixels[x][channel] = blend[pixels[x][channel]][channel];
      }
    }
  }
}

// 框与标签：框的裁剪/类别配色/线宽在此，标签样式在 putLabel()。
static void drawBox(
  cv::Mat & canvas, const vision_msgs::msg::BoundingBox2D & box,
  const std::string & name, double score, bool with_labels)
{
  if (canvas.empty()) {return;}
  const double cx = box.center.position.x;
  const double cy = box.center.position.y;
  if (!std::isfinite(cx) || !std::isfinite(cy) || !std::isfinite(box.size_x) ||
    !std::isfinite(box.size_y) || box.size_x <= 0.0 || box.size_y <= 0.0 ||
    !std::isfinite(score)) {return;}
  // 先校验有限性并裁剪，再转整数，防止外部消息中的极大值导致越界转换。
  const int left = static_cast<int>(std::clamp(cx - box.size_x / 2.0, 0.0,
      static_cast<double>(canvas.cols)));
  const int top = static_cast<int>(std::clamp(cy - box.size_y / 2.0, 0.0,
      static_cast<double>(canvas.rows)));
  const int right = static_cast<int>(std::clamp(cx + box.size_x / 2.0, 0.0,
      static_cast<double>(canvas.cols)));
  const int bottom = static_cast<int>(std::clamp(cy + box.size_y / 2.0, 0.0,
      static_cast<double>(canvas.rows)));
  const cv::Rect clipped(left, top, right - left, bottom - top);
  if (clipped.area() <= 0) {return;}
  cv::Scalar colour;
  const char * known = knownClassName(name, colour);
  const std::string shown = known ? known : name;
  if (!known) {
    // 对类别文本作无符号哈希得到稳定颜色；溢出按模回绕，避免有符号溢出的未定义行为。
    unsigned int hash = 7u;
    for (unsigned char c : shown) {
      hash = hash * 31u + c;
    }
    colour = colourForId(static_cast<int>(hash % 180u));
  }

  const double scale = overlayScale(canvas);
  const int thickness = scale >= 1.0 ? 3 : 2;
  cv::rectangle(canvas, clipped, colour, thickness);
  if (with_labels) {
    char text[96];
    std::snprintf(text, sizeof(text), "%s %.0f%%", shown.c_str(), score * 100.0);
    // 类别与置信度文字使用框线缩放的一半，保持框线宽度。
    putLabel(canvas, clipped, text, colour, scale * 0.5);
  }
}

void drawDetectionArray(
  cv::Mat & canvas,
  const vision_msgs::msg::Detection2DArray & detections,
  bool with_labels)
{
  // 仅展示每个检测的首个类别假设；无假设时仍画通用 object 框，不改变检测数组。
  for (const auto & det : detections.detections) {
    if (det.results.empty()) {
      drawBox(canvas, det.bbox, "object", 0.0, with_labels);
    } else {
      const auto & hypothesis = det.results.front().hypothesis;
      drawBox(canvas, det.bbox, hypothesis.class_id, hypothesis.score, with_labels);
    }
  }
}

template<typename Points>
static std::vector<cv::Point> fieldPoints(const Points & points, std::size_t expected)
{
  std::vector<cv::Point> result;
  if (points.size() != expected) {return result;}
  result.reserve(expected);
  for (const auto & point : points) {
    if (!std::isfinite(point.x) || !std::isfinite(point.y) ||
      std::abs(point.x) > 1000000.0f || std::abs(point.y) > 1000000.0f)
    {
      return {};
    }
    result.emplace_back(cvRound(point.x), cvRound(point.y));
  }
  return result;
}

// 只染色多边形内部；包围盒里多边形以外的像素与原图相同，混合后保持不变。
using vision_stream_detail::tintPolygon;

// 文字锚点取顶点等权均值并裁到画布内，仅用于排版，不是业务安全目标的中心计算。
static cv::Point regionAnchor(const std::vector<cv::Point> & points, const cv::Size & size)
{
  long long x = 0;
  long long y = 0;
  for (const auto & point : points) {
    x += point.x;
    y += point.y;
  }
  const int count = static_cast<int>(points.size());
  return {
    std::clamp(static_cast<int>(x / count), 0, std::max(0, size.width - 1)),
    std::clamp(static_cast<int>(y / count), 0, std::max(0, size.height - 1)),
  };
}

static void drawTag(cv::Mat & canvas, const std::string & text, const cv::Point & anchor, double scale)
{
  const int thickness = scale >= 1.0 ? 2 : 1;
  int baseline = 0;
  const cv::Size extent =
    cv::getTextSize(text, cv::FONT_HERSHEY_SIMPLEX, scale, thickness, &baseline);
  const cv::Point origin(
    std::clamp(anchor.x - extent.width / 2, 0, std::max(0, canvas.cols - extent.width)),
    std::clamp(anchor.y + extent.height / 2, extent.height, std::max(extent.height, canvas.rows - 2)));
  cv::putText(canvas, text, origin, cv::FONT_HERSHEY_SIMPLEX, scale, cv::Scalar(0, 0, 0),
    thickness + 2, cv::LINE_AA);
  cv::putText(canvas, text, origin, cv::FONT_HERSHEY_SIMPLEX, scale, cv::Scalar(250, 250, 250),
    thickness, cv::LINE_AA);
}

static void drawBottomChip(cv::Mat & canvas, const std::string & text, const cv::Scalar & accent)
{
  const double scale = std::min(overlayScale(canvas), 0.9);
  const int thickness = 1;
  int baseline = 0;
  const cv::Size extent =
    cv::getTextSize(text, cv::FONT_HERSHEY_SIMPLEX, scale, thickness, &baseline);
  constexpr int kPad = 6;
  const int width = extent.width + 2 * kPad + 8;
  const int height = extent.height + 2 * kPad;
  const int y = std::max(0, canvas.rows - height - kPad);
  const cv::Rect plate(kPad, y, width, height);
  const cv::Rect clipped = plate & cv::Rect(0, 0, canvas.cols, canvas.rows);
  if (clipped.area() <= 0) {return;}
  cv::rectangle(canvas, clipped, cv::Scalar(12, 16, 12), cv::FILLED);
  const cv::Rect bar(clipped.x, clipped.y, std::min(4, clipped.width), clipped.height);
  cv::rectangle(canvas, bar, accent, cv::FILLED);
  cv::putText(canvas, text, cv::Point(clipped.x + kPad + 4, clipped.y + kPad + extent.height - 1),
    cv::FONT_HERSHEY_SIMPLEX, scale, cv::Scalar(245, 245, 245), thickness, cv::LINE_AA);
}

// 场地几何：无效结果只留状态条；有效结果才画左右区和分界线。
void drawFieldObservation(
  cv::Mat & canvas,
  const interface_pkg::msg::FieldObservation & observation)
{
  if (canvas.empty()) {return;}
  // 被拒候选不能当成已确认场地。valid 为假时左右点即使残留也不上屏。
  if (observation.valid) {
    const auto corners = fieldPoints(observation.corners, 4);
    const auto left = fieldPoints(observation.left_region, 4);
    const auto right = fieldPoints(observation.right_region, 4);
    const auto split = fieldPoints(observation.split_line, 2);
    const cv::Scalar left_colour(255, 190, 40);
    const cv::Scalar right_colour(40, 140, 255);
    const double scale = overlayScale(canvas);
    const int thickness = scale >= 1.0 ? 3 : 2;
    if (!left.empty()) {
      tintPolygon(canvas, left, left_colour);
      cv::polylines(canvas, left, true, left_colour, thickness, cv::LINE_AA);
      drawTag(canvas, "LEFT", regionAnchor(left, canvas.size()), scale);
    }
    if (!right.empty()) {
      tintPolygon(canvas, right, right_colour);
      cv::polylines(canvas, right, true, right_colour, thickness, cv::LINE_AA);
      drawTag(canvas, "RIGHT", regionAnchor(right, canvas.size()), scale);
    }
    if (!corners.empty()) {
      cv::polylines(canvas, corners, true,
        observation.committed ? cv::Scalar(230, 230, 230) : cv::Scalar(140, 140, 140),
        1, cv::LINE_AA);
    }
    if (!split.empty()) {
      cv::line(canvas, split[0], split[1], observation.split_suspect ?
               cv::Scalar(0, 180, 255) : cv::Scalar(255, 255, 0), 2, cv::LINE_AA);
    }
  }

  const char * status = !observation.valid ? "SAFE INVALID" :
    (observation.committed ? "SAFE VALID" : "SAFE UNCOMMITTED");
  std::string text = status;
  if (!observation.state.empty()) {
    text += " ";
    text += observation.state;
  }
  if (!observation.quality.empty() && observation.quality != "none") {
    text += " ";
    text += observation.quality;
  }
  const cv::Scalar accent = !observation.valid ? cv::Scalar(40, 40, 220) :
    (observation.committed ? cv::Scalar(40, 200, 60) : cv::Scalar(20, 170, 255));
  drawBottomChip(canvas, text, accent);
}

// 状态文字：字号与底板样式的唯一落点。
void drawStats(cv::Mat & canvas, const std::string & text)
{
  if (canvas.empty() || text.empty()) {return;}

  constexpr double kFontScale = 0.55;
  constexpr int kThickness = 1;
  constexpr int kPad = 6;
  int baseline = 0;
  const cv::Size extent =
    cv::getTextSize(text, cv::FONT_HERSHEY_SIMPLEX, kFontScale, kThickness, &baseline);

  const cv::Rect plate(kPad, kPad, extent.width + 2 * kPad, extent.height + 2 * kPad);
  const cv::Rect clipped = plate & cv::Rect(0, 0, canvas.cols, canvas.rows);
  if (clipped.area() <= 0) {return;}

  // 半透明底板提高文字对比度，同时保留底下的场景轮廓。
  cv::Mat roi = canvas(clipped);
  const cv::Mat shade(roi.size(), roi.type(), cv::Scalar(0, 0, 0));
  cv::addWeighted(roi, 0.35, shade, 0.65, 0.0, roi);

  cv::putText(canvas, text, cv::Point(2 * kPad, 2 * kPad + extent.height),
    cv::FONT_HERSHEY_SIMPLEX, kFontScale, cv::Scalar(255, 255, 255), kThickness,
    cv::LINE_AA);
}
