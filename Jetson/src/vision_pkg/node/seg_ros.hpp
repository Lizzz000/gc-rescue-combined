#pragma once
// Copyright 2026 jetson_yolo integration
// SPDX-License-Identifier: AGPL-3.0
// 纯 DTO 到 ROS 消息；不依赖 GPU。
#include "yolo/types.hpp"
#include <vector>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/header.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>

// 数组及每个检测复用同一源 header；类别编号保持模型索引，显示名称不改线上类别身份。
vision_msgs::msg::Detection2DArray toDetection2DArray(
  const std::vector<SegDetection> & detections,
  const std_msgs::msg::Header & header);

// mono8 编码和降采样分辨率是上下游契约；本函数按传入矩阵的实际宽高逐行复制。
/// `mono8`，0表示背景；非零标签由适配器将class_id加mask_label_offset后饱和到[1,255]。
/// 该图像是原始帧的 1/mask_scale；它自身的 width/height 已经携带了这一信息，
/// 因此使用方不需要该参数。
sensor_msgs::msg::Image toLabelMaskImage(
  const cv::Mat & label_mask,
  const std_msgs::msg::Header & header);
