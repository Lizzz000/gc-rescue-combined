#pragma once

// Copyright 2026 li
// SPDX-License-Identifier: Apache-2.0

#include <cstddef>
#include <optional>
#include <interface_pkg/app/core/recognition.hpp>
#include <std_msgs/msg/header.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>

struct TrackRosConversion
{
  // 按原数组下标复制完整检测，空结果仍保留源帧头。下标越界时抛出 out_of_range。
  static vision_msgs::msg::Detection2DArray selected(
    const vision_msgs::msg::Detection2DArray & input, std::optional<std::size_t> index);

  // 将像素框和轨迹身份转换成 ROS 检测，数组和单框沿用源帧头。
  static vision_msgs::msg::Detection2DArray tracked(
    const RecognitionDetections & tracks, const std_msgs::msg::Header & header);
};
