#pragma once
// 把场地结果和 BGR 图像收成 ROS 消息。不跑定位，也不发布。

#include "field/field.hpp"

// Copyright 2026 jetson_yolo integration
// SPDX-License-Identifier: Apache-2.0
// 场地定位结果到对外 ROS 消息的转换边界：FieldResult -> FieldObservation 的映射与坐标换算。
// 这是纯场地结果与 ROS 的适配边界；原始输入尺寸用于恢复像素坐标，header 由调用者原样提供。


#include <interface_pkg/msg/field_observation.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/header.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <memory>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <stdexcept>

// 前置条件：ok && committed、4 个有效角点、BEV 非空、homography_inv 为有限可逆的 3x3 CV_32F/64F；
// 黑带须 split_valid、置信度有限且为正，上下端点严格位于 (0, bev.cols-1)，split_x 与端点均值一致。
// 任一不满足则 valid=false，不输出正式几何；贴边端点按既有几何合同拒绝，不能按闭区间接纳。
// 反投影：用 homography_inv 把 BEV 上 6 个唯一顶点（中间两点由黑带上下端点决定）投回处理分辨率再拼 left/right/split_line；分母为零或变号会静默出错，故整帧拒绝。corners 由处理分辨率还原，split_x 不换算。
// 加/改对外发布字段：字段先在 FieldResult 产生再在此映射，坐标语义改动要同步订阅端。
// 所有多边形统一还原为源图像素；无效结果保留诊断及可选候选角点，不携带正式目标几何。
// 纯 field/ 算法仍不依赖 ROS。
// 诊断字段可以描述未提交结果；目标几何必须经过本函数的有限值、可逆性和轮廓检查。
// 无效结果仍使用本帧 header；不会把旧几何填成有效观测来掩盖本帧失败。
inline interface_pkg::msg::FieldObservation toFieldObservation(
  const FieldResult & r, const FieldConfig & c,
  const cv::Size & original, const std_msgs::msg::Header & header)
{
  interface_pkg::msg::FieldObservation m;
  // 先填写诊断字段；后续任一几何检查失败都返回这份消息，valid/committed 保持默认 false。
  m.header = header;
  m.state = r.state; m.quality = r.quality; m.reason = r.reason;
  m.hold_reason = r.hold_reason;
  m.bev_width = r.bev.cols; m.bev_height = r.bev.rows;
  m.split_x = r.split_x; m.split_measured = r.split_measured;
  m.split_suspect = r.split_suspect; m.split_confidence = r.split_conf;
  m.red_ratio = r.red_ratio; m.score = r.score.total; m.jump_ratio = r.jump_ratio;
  m.measured_edges = static_cast<uint8_t>(std::clamp(r.measured_edges, 0, 4));
  for (int i = 0; i < 4; ++i) {
    m.edge_states[i] = static_cast<uint8_t>(r.edge_state[i]);
  }
  if (original.width <= 0 || original.height <= 0 || c.proc_width <= 0) {return m;}
  const int ph = c.proc_height > 0 ? c.proc_height :
    static_cast<int>(std::lround(original.height * double(c.proc_width) / original.width));
  if (ph <= 0) {return m;}
  const double sx = double(original.width) / c.proc_width;
  const double sy = double(original.height) / ph;
  // 核心点位处于 proc 处理图坐标；对外角点/区域/分割线统一还原为原图像素，不是 [0,1] 归一化坐标。
  auto convert = [&](const std::vector<cv::Point2f> & points) {
      decltype(m.corners) out;
      out.reserve(points.size());
      for (const auto & p : points) {
        if (!std::isfinite(p.x) || !std::isfinite(p.y)) {return decltype(out)();}
        geometry_msgs::msg::Point32 q;
        q.x = static_cast<float>(p.x * sx); q.y = static_cast<float>(p.y * sy);
        if (!std::isfinite(q.x) || !std::isfinite(q.y)) {return decltype(out)();}
        out.push_back(q);
      }
      return out;
    };
  if (r.candidate_corners.size() == 4) {m.candidate_corners = convert(r.candidate_corners);}
  const bool split_usable = r.split_valid && std::isfinite(r.split_conf) && r.split_conf > 0 &&
    std::isfinite(r.split_x) && std::isfinite(r.split_top_x) && std::isfinite(r.split_bottom_x) &&
    r.split_top_x > 0 && r.split_top_x < r.bev.cols - 1 &&
    r.split_bottom_x > 0 && r.split_bottom_x < r.bev.cols - 1 &&
    std::abs(r.split_x - (r.split_top_x + r.split_bottom_x) * 0.5) <= 1e-6;
  if (r.ok && r.committed && !split_usable) {
    // 跟踪状态描述外框历史，展示质量必须同时反映分区不可用，不能显示 INVALID ... good。
    m.quality = "split_unreliable";
    if (m.reason.find("黑带证据不足") == std::string::npos) {
      m.reason += (m.reason.empty() ? "" : "; ") + std::string("黑带证据不足, 不输出左右分区");
    }
  }
  // 候选角点仅供诊断；只有本帧确认提交的几何才能置 valid，历史保持结果不能绕过 committed 检查。
  if (!r.ok || !r.committed || r.corners.size() != 4 || r.bev.empty() ||
    r.homography_inv.rows != 3 || r.homography_inv.cols != 3 ||
    r.homography_inv.channels() != 1 ||
    (r.homography_inv.depth() != CV_32F && r.homography_inv.depth() != CV_64F) ||
    !cv::checkRange(r.homography_inv) || std::abs(cv::determinant(r.homography_inv)) < 1e-12 ||
    !split_usable) {return m;}
  const auto corners = convert(r.corners);
  if (corners.size() != 4 || !cv::isContourConvex(r.corners) ||
    std::abs(cv::contourArea(r.corners)) < 1e-6) {return m;}
  const float xt = static_cast<float>(r.split_top_x), xb = static_cast<float>(r.split_bottom_x);
  const float w = r.bev.cols - 1.f, h = r.bev.rows - 1.f;
  // 六个独立顶点只投影一次；拒绝穿越 BEV 的投影地平线，避免零分母被 OpenCV 映为看似有限的原点。
  cv::Mat inverse;
  r.homography_inv.convertTo(inverse, CV_64F);
  const std::vector<cv::Point2f> bev{{0, 0}, {xt, 0}, {w, 0}, {0, h}, {xb, h}, {w, h}};
  std::vector<cv::Point2f> proc;
  double sign = 0;
  for (const auto & p : bev) {
    // 齐次分母趋近 0 或变号说明投影接近/跨过无穷远；拒绝整组几何，避免有限数值伪装成有效区域。
    const double denominator = inverse.at<double>(2, 0) * p.x +
      inverse.at<double>(2, 1) * p.y + inverse.at<double>(2, 2);
    if (!std::isfinite(denominator) || std::abs(denominator) < 1e-9 ||
      (sign != 0 && std::signbit(denominator) != std::signbit(sign))) {return m;}
    sign = denominator;
    proc.emplace_back(
      (inverse.at<double>(0, 0) * p.x + inverse.at<double>(0, 1) * p.y + inverse.at<double>(0, 2)) / denominator,
      (inverse.at<double>(1, 0) * p.x + inverse.at<double>(1, 1) * p.y + inverse.at<double>(1, 2)) / denominator);
  }
  // 左右区域复用同一组投影顶点，proc[1]/proc[4] 是共同分界，保证两侧与 split_line 完全一致。
  const std::vector<cv::Point2f> left_proc{proc[0], proc[1], proc[4], proc[3]};
  const std::vector<cv::Point2f> right_proc{proc[1], proc[2], proc[5], proc[4]};
  if (!cv::isContourConvex(left_proc) || !cv::isContourConvex(right_proc) ||
    std::abs(cv::contourArea(left_proc)) < 1e-6 || std::abs(cv::contourArea(right_proc)) < 1e-6) {return m;}
  const auto left = convert(left_proc);
  const auto right = convert(right_proc);
  if (left.size() != 4 || right.size() != 4) {return m;}
  m.corners = corners;
  m.left_region = left;
  m.right_region = right;
  m.split_line = {left[1], left[2]};
  m.valid = true;
  m.committed = true;
  m.switched = r.switched;
  return m;
}

// 单次像素复制到 ROS 消息，返回独占所有权；支持有行跨度的 OpenCV ROI。
namespace vision_ros
{
inline std::unique_ptr<sensor_msgs::msg::Image> makeBgrImageMessage(
  const cv::Mat & image, const std_msgs::msg::Header & header)
{
  // 等价于 cv_bridge::CvImage(header, "bgr8", image).toImageMsg()，但直接写进 unique_ptr 持有的消息，
  // 省去 toImageMsg() 返回 shared_ptr 后 publish(*msg) 在进程内路径上的第二次整帧拷贝。
  if (image.empty() || image.type() != CV_8UC3) {
    throw std::invalid_argument("expected a non-empty CV_8UC3 image");
  }
  auto message = std::make_unique<sensor_msgs::msg::Image>();
  message->header = header;
  message->height = static_cast<std::uint32_t>(image.rows);
  message->width = static_cast<std::uint32_t>(image.cols);
  message->encoding = sensor_msgs::image_encodings::BGR8;
  message->is_bigendian = 0;
  const std::size_t row_bytes = static_cast<std::size_t>(image.cols) * image.elemSize();
  message->step = static_cast<std::uint32_t>(row_bytes);
  const std::size_t image_bytes = row_bytes * static_cast<std::size_t>(image.rows);
  // 直接用源像素构造消息字节，避免先 resize 清零再覆盖整帧；消息仍独占一份像素。
  if (image.isContinuous()) {
    message->data.assign(image.data, image.data + image_bytes);
  } else {
    // ROI 的行尾可能含父图像素或填充；只打包可见行，输出 step 与原行为一致。
    message->data.reserve(image_bytes);
    for (int row = 0; row < image.rows; ++row) {
      const auto * pixels = image.ptr(row);
      message->data.insert(message->data.end(), pixels, pixels + row_bytes);
    }
  }
  return message;
}
}  // namespace vision_ros
