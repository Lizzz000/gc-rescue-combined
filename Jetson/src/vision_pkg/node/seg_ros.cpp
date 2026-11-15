#include "node/seg_ros.hpp"
// Copyright 2026 jetson_yolo integration
// SPDX-License-Identifier: AGPL-3.0
#include <algorithm>
#include <cstdint>
#include <utility>

static void fillBox(vision_msgs::msg::Detection2D & msg, const BBox2i & bbox)
{
  msg.bbox.center.position.x = static_cast<double>(bbox.x) + bbox.width / 2.0;
  msg.bbox.center.position.y = static_cast<double>(bbox.y) + bbox.height / 2.0;
  msg.bbox.center.theta = 0.0;
  msg.bbox.size_x = static_cast<double>(bbox.width);
  msg.bbox.size_y = static_cast<double>(bbox.height);
}

static void fillHypothesis(
  vision_msgs::msg::Detection2D & msg, int class_id, float confidence)
{
  vision_msgs::msg::ObjectHypothesisWithPose hypothesis;
  // 线上类别编号始终使用模型索引；显示标签不改变跨包协议中的类别 ID。
  hypothesis.hypothesis.class_id = std::to_string(class_id);
  hypothesis.hypothesis.score = static_cast<double>(confidence);
  msg.results.push_back(std::move(hypothesis));
}


vision_msgs::msg::Detection2DArray toDetection2DArray(
  const std::vector<SegDetection> & detections, const std_msgs::msg::Header & header)
{
  vision_msgs::msg::Detection2DArray msg;
  msg.header = header;
  msg.detections.reserve(detections.size());

  for (const auto & det : detections) {
    vision_msgs::msg::Detection2D det_msg;
    det_msg.header = header;
    fillBox(det_msg, det.bbox);
    fillHypothesis(det_msg, det.class_id, det.confidence);
    msg.detections.push_back(std::move(det_msg));
  }
  return msg;
}

sensor_msgs::msg::Image toLabelMaskImage(
  const cv::Mat & label_mask,
  const std_msgs::msg::Header & header)
{
  sensor_msgs::msg::Image msg;
  msg.header = header;
  msg.encoding = "mono8";
  msg.is_bigendian = 0;

  if (label_mask.empty() || label_mask.type() != CV_8UC1) {
    msg.width = 0;
    msg.height = 0;
    msg.step = 0;
    return msg;
  }

  msg.width = static_cast<std::uint32_t>(label_mask.cols);
  msg.height = static_cast<std::uint32_t>(label_mask.rows);
  msg.step = static_cast<std::uint32_t>(label_mask.cols);
  msg.data.resize(static_cast<std::size_t>(label_mask.cols) * label_mask.rows);

  // 逐行读取支持有行跨度的 ROI 矩阵；消息输出是紧密排列的 mono8，不复制原矩阵的填充字节。
  for (int r = 0; r < label_mask.rows; ++r) {
    std::copy_n(label_mask.ptr<std::uint8_t>(r), label_mask.cols,
                msg.data.begin() + static_cast<std::ptrdiff_t>(r) * label_mask.cols);
  }
  return msg;
}
