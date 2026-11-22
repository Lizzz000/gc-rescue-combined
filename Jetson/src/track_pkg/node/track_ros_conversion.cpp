// Copyright 2026 li
// SPDX-License-Identifier: Apache-2.0

#include "node/track_ros_conversion.hpp"

#include <utility>
#include <vision_msgs/msg/detection2_d.hpp>
#include <vision_msgs/msg/object_hypothesis_with_pose.hpp>

vision_msgs::msg::Detection2DArray TrackRosConversion::selected(
  const vision_msgs::msg::Detection2DArray & input, std::optional<std::size_t> index)
{
  vision_msgs::msg::Detection2DArray output;
  output.header = input.header;
  // 没有选中项时仍保留这一源帧的数组头，用空数组明确表达本帧没有安全目标。
  if (index) {
    // 索引指向原始消息；完整复制包括全部假设、pose、ID 与检测自己的 header。
    // 使用 at 保留索引越界的显式失败；不能静默改成另一条轨迹或吞掉错误。
    output.detections.push_back(input.detections.at(*index));
  }
  return output;
}

vision_msgs::msg::Detection2DArray TrackRosConversion::tracked(
  const RecognitionDetections & tracks, const std_msgs::msg::Header & header)
{
  vision_msgs::msg::Detection2DArray output;
  output.header = header;
  // 跟踪是重新生成框，沿用源帧时间；不复制原检测的额外假设或推测其 pose。
  output.detections.reserve(tracks.size());
  for (const auto & track : tracks) {
    vision_msgs::msg::Detection2D detection;
    detection.header = header;
    detection.id = track.id;
    detection.bbox.center.position.x = track.box.center_x;
    detection.bbox.center.position.y = track.box.center_y;
    detection.bbox.size_x = track.box.width;
    detection.bbox.size_y = track.box.height;
    if (track.hypothesis) {
      vision_msgs::msg::ObjectHypothesisWithPose result;
      result.hypothesis.class_id = track.hypothesis->class_id;
      result.hypothesis.score = track.hypothesis->score;
      detection.results.push_back(result);
    }
    output.detections.push_back(std::move(detection));
  }
  return output;
}
