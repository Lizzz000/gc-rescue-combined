#pragma once
// 推理会话：分割走 TensorRT，场地走 OpenCV。节点只在 node/detect_node.hpp。

#include "field/field.hpp"

// Copyright 2026 jetson_yolo integration
// SPDX-License-Identifier: Apache-2.0
// 进程内推理门面。segment() 仅走 TensorRT GPU。
// field() 始终走 OpenCV 场地定位。两条路径互不调用。

#include <cstddef>
#include <functional>
#include <memory>
#include <string>
#include <vector>
#include <opencv2/core.hpp>
#include "yolo/yolo.hpp"

// Segment 只建分割，Field 只建场地，Both 都建。没有 TensorRT 的构建只能选 Field。
enum class DetectTask { Segment, Field, Both };
const char * toString(DetectTask task);
bool parseTask(const std::string & text, DetectTask & out);

// 已解析的运行配置；成员初值用于纯 C++ 调用，生产节点逐项写入统一配置的有效值。
// create() 只加载预构建引擎，不执行模型构建；conf_seg/iou 为 [0,1] 的无量纲阈值。
struct DetectorConfig
{
  DetectTask task{DetectTask::Both};
  std::string seg_model, seg_labels;
  int dla_core{-1};
  float conf_seg{0.25f}, iou{0.7f};
  int max_detections{300};
  int mask_scale{2}, mask_label_offset{1}, max_masks{32};
  FieldConfig field;
  // ROS 适配层提供当前 context 的停止状态；后端仅调用标准库谓词，不依赖 ROS。
  std::function<bool()> cancel_requested;
};
struct SegRun
{
  // 检测框已映射回原始输入图像；label_mask 是降采样的 mono8 类别图，不能直接套用原图像素坐标。
  std::vector<SegDetection> detections;
  cv::Mat label_mask;
  // 毫秒：inference_ms 用本机单调时钟测量整次分割调用，另两项拆分后端工作，不含 ROS 发布。
  double inference_ms{0.0}, gpu_ms{0.0}, post_ms{0.0};
};
class FastSegmentor;
// 生命周期由节点的检测会话持有；同一实例交给一个推理线程使用，不在此门面内另开线程或加锁。
class Detector {
public:
  ~Detector();
  static std::unique_ptr<Detector> create(
    const DetectorConfig & config,
    std::string & error, std::vector<std::string> & notes, bool * cancelled = nullptr);
  bool hasSeg() const {return seg_available_;}
  bool hasField() const {return field_available_;}
  cv::Size segInput() const;
  // nvmm：同一帧的 NvBufSurface（见 CameraFrame::nvmm），非空时推理前处理直接在 GPU 上读它。
  SegRun segment(const cv::Mat & image, bool want_label_mask, const void * nvmm = nullptr) const;
  FieldResult field(const cv::Mat & image);
  void resetField();
  const DetectorConfig & config() const {return config_;}

private:
  Detector();
  DetectorConfig config_;
  bool seg_available_{false}, field_available_{false};
#if VISION_ENABLE_TENSORRT
  std::unique_ptr<FastSegmentor> segmentor_;
#endif
  FieldLocator field_locator_;
};
