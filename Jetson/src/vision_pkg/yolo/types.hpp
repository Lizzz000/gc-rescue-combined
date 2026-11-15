#pragma once
// Copyright 2026 jetson_yolo integration
// SPDX-License-Identifier: AGPL-3.0
// 纯值交付数据；不依赖 ROS、CUDA 或 TensorRT。
#include <string>
#include <opencv2/core.hpp>

/// 轴对齐框，以左上角加宽高表示，单位为原始图像像素。
struct BBox2i
{
  int x{0};
  int y{0};
  int width{0};
  int height{0};
};

/// 来自分割模型的一个实例。
struct SegDetection
{
  BBox2i bbox;
  float confidence{0.0f};
  int class_id{-1};
  std::string class_name;
  /// 该实例的 CV_8UC1 mask，分辨率为 1/mask_scale；调用方未请求时为空。
  /// 其原点是 *原始* 帧的 (0,0) 除以 mask_scale，
  /// 而不是框的原点。
  cv::Mat mask;
};

// 以下为适配器直接调用时的开发初值；生产由 DetectorConfig 逐项覆盖。
// 检测数量与掩膜数量预算独立，标签重叠仍由高置信度实例覆盖。
struct SegOptions
{
  float conf_threshold{0.4f};
  // 两种输出布局均在解码后执行同类别 NMS，IoU 严格大于此阈值时抑制。
  float iou_threshold{0.45f};
  int max_detections{300};

  // 掩膜相对原图的降采样倍数，>= 1。1 表示全分辨率。
  int mask_scale{2};
  // 类别 c 加 mask_label_offset 后饱和到[1,255]，0始终保留为背景。
  int mask_label_offset{1};

  bool want_label_mask{true};
  bool want_instance_masks{false};
  // 拥挤画面里最多给这么多个高置信度实例生成掩膜。
  int max_masks{32};
  bool needsMasks() const noexcept
  {return max_masks > 0 && (want_label_mask || want_instance_masks);}
};
