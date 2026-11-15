#pragma once
// Copyright 2026 jetson_yolo integration
// SPDX-License-Identifier: AGPL-3.0
// 真实 CPU 解码、同类别 NMS 和掩膜合成；仅依赖 OpenCV。
#include "yolo/types.hpp"
#include <cstddef>
#include <memory>
#include <vector>

/// 针对一组（源图像, engine 输入）解析出的 letterbox 参数。
struct Letterbox
{
  float scale{1.0f};
  int new_w{0};
  int new_h{0};
  int pad_left{0};
  int pad_top{0};
};

Letterbox letterboxFor(const cv::Size & source, const cv::Size & input);

// 引擎初始化已验收的主机张量视图。只在 run() 内借用，返回值不引用它。
// detections: E2E [rows, features]；legacy [features, rows]。
// prototypes: [prototype_count, prototype.height * prototype.width]，无掩膜需求可为空。
// gain/pad 来自同一帧 GPU letterbox 的实际缓存，不重新猜测 padding。
struct SegOutputView
{
  const float * detections{nullptr};
  const float * prototypes{nullptr};
  std::size_t rows{0}, features{0};
  int prototype_count{0};
  cv::Size prototype_size, image_size, input_size;
  float gain{1}, pad_x{0}, pad_y{0};
  bool end_to_end{false};
};

// 单个推理线程独占；候选、系数和 mask scratch 跨帧复用。
// 每次交付的标签图及实例图拥有独立存储，下一帧不会覆盖它们。
class SegPostprocessor
{
public:
  SegPostprocessor();
  ~SegPostprocessor();
  SegPostprocessor(const SegPostprocessor &) = delete;
  SegPostprocessor & operator=(const SegPostprocessor &) = delete;
  // options 的阈值已由调用入口验收；与引擎形状一样，不在逐帧核心重复校验。
  std::vector<SegDetection> run(
    const SegOutputView & output, const SegOptions & options,
    const std::vector<std::string> & names, cv::Mat & label_mask);
private:
  struct Workspace;
  std::unique_ptr<Workspace> workspace_;
};
