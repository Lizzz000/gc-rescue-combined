#pragma once

// Copyright 2026 jetson_yolo integration
// SPDX-License-Identifier: Apache-2.0

#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>

#include "interface_pkg/msg/field_observation.hpp"
#include "interface_pkg/msg/stream_status.hpp"
#include "interface_pkg/msg/task_state.hpp"

class StreamRuntime;
struct StreamRuntimeConfig;

// 图传节点负责 ROS 参数、话题和日志；图像合成与 HTTP 服务由运行会话管理。
class StreamNode : public rclcpp::Node
{
public:
  explicit StreamNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~StreamNode() override;

private:
  void declareParameters();
  StreamRuntimeConfig loadParameters();
  void createRosInterfaces(const StreamRuntimeConfig & config);

  void activeTaskCallback(const interface_pkg::msg::TaskState::SharedPtr msg);
  void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr & msg);
  void segDetectionCallback(const vision_msgs::msg::Detection2DArray::SharedPtr msg);
  void segMaskCallback(const sensor_msgs::msg::Image::ConstSharedPtr & msg);
  void fieldCallback(const interface_pkg::msg::FieldObservation::SharedPtr msg);
  void reportHttpFailure();
  void publishStatus();

  std::string image_topic_;
  std::string seg_detections_topic_;
  std::string seg_masks_topic_;
  std::string field_results_topic_;
  std::string active_task_topic_;

  bool publish_annotated_{false};
  bool publish_status_{true};

  rclcpp::Subscription<interface_pkg::msg::TaskState>::SharedPtr active_task_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Subscription<vision_msgs::msg::Detection2DArray>::SharedPtr seg_det_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr seg_mask_sub_;
  rclcpp::Subscription<interface_pkg::msg::FieldObservation>::SharedPtr field_sub_;

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr annotated_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr compressed_pub_;
  rclcpp::Publisher<interface_pkg::msg::StreamStatus>::SharedPtr status_pub_;
  rclcpp::TimerBase::SharedPtr status_timer_;

  std::unique_ptr<StreamRuntime> runtime_;
};
