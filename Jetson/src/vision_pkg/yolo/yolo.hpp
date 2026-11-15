#pragma once
// Copyright 2026 jetson_yolo integration
// SPDX-License-Identifier: AGPL-3.0
// GPU 提交适配器；纯 CPU 后处理与交付 DTO 在 postprocess.hpp/types.hpp。
#include "yolo/postprocess.hpp"
// 上游补丁依赖 GraphCaptureDecision，先引入项目 TensorRT 合同。
#include "yolo/tensorrt/tensorrt.hpp"
#include <yolos/tasks/segmentation.hpp>

// 单次调用耗时，单位毫秒；gpu_ms 用主机单调计时包围 inferNvmm，并非 CUDA event 纯 kernel 时间。
// 两种输出布局均分开统计 GPU 提交/回传与 CPU 后处理。
struct SegTiming
{
  double gpu_ms{0.0};
  double post_ms{0.0};
};

// 有界掩膜数量和局部后处理的分割适配器。
class FastSegmentor : public yolos::seg::YOLOSegDetector {
public:
  FastSegmentor(
    const std::string & engine_path, const std::string & labels_path,
    int dla_core = -1, std::optional<SegOutputLayout> declared_layout = std::nullopt);

  // 启动时已结合类别/可信导出合同确定为 end-to-end 时返回 true。
  bool isEndToEnd() const noexcept {return end_to_end_;}

  int prototypeCount() const noexcept {return proto_count_;}
  // 优先用标签数量。无标签的 legacy 按输出形状推断；无标签的 end2end 返回 0。
  int classCount() const noexcept {return num_classes_;}

  // 非空 image 必须是 CV_8UC3 BGR（同一帧的 CPU 副本），源图保持只读。
  // label_mask 是边长按mask_scale向上取整降采样的CV_8UC1；0为背景，类别加偏移后饱和到[1,255]。未请求时为空。
  // timing 可选。gpu_ms + post_ms 不包含调用方自己的打包和发布。
  // nvmm 是同一帧的 NvBufSurface（NVMM I420 或 BGRx），前处理只在 GPU 上读它；image 是 CPU 侧
  // BGR 副本，这里只用来校验和给后处理提供尺寸。推理不再接受仅有 cv::Mat 的帧。
  std::vector<SegDetection> segmentFast(
    const cv::Mat & image, const SegOptions & options,
    cv::Mat & label_mask, SegTiming * timing = nullptr, const void * nvmm = nullptr);

protected:
  // 静态引擎只在构造时设置；显式声明用于已知输出合同的适配调用，不从张量内容猜测。
  void configureOutputLayout(
    std::size_t class_count, std::optional<SegOutputLayout> declared_layout = std::nullopt);
  // 只处理本次已同步到主机的输出；不得在下一次 inferNvmm 覆盖缓冲后调用。
  std::vector<SegDetection> postprocessOutputs(
    const cv::Size & image_size, const SegOptions & options, cv::Mat & label_mask);

private:
  // NVMM（I420/BGRx）→ 融合核 → TRT。
  void inferNvmm(const void * nvmm, std::size_t host_output_count);

  SegPostprocessor postprocessor_;
  bool end_to_end_{false};
  int proto_count_{32};
  cv::Size proto_size_{0, 0};
  int num_classes_{0};
};
