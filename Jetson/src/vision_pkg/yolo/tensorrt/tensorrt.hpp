#pragma once

// Copyright 2026 jetson_yolo integration
// SPDX-License-Identifier: AGPL-3.0
//
// 离线 INT8 标定器 ImageFolderCalibrator（生产分割适配器 FastSegmentor 在 yolo/yolo.hpp）。
// 标定器只供离线构建引擎，生产节点不调用它。
//
// 标定段对一个图像目录做 Entropy-calibrator 2，预处理遵循推理输入约定：
// NCHW RGB float、按 1/255 缩放、114/255 填充、保持宽高比。本实现使用 OpenCV
// 在 CPU 上生成张量，getBatch() 把张量复制到标定器自有设备缓冲并填写 TensorRT
// 输入绑定。推理侧 NVMM 融合核（yolo/nvmm_preprocess.cu）使用像素中心采样，
// 与 OpenCV INTER_LINEAR 相同的采样约定。CPU 路径先插值到 uint8，
// CUDA 路径进行浮点插值，因此不承诺逐元素完全一致；标定效果仍须板端验证。

// 预处理必须和网络真实输入一致；标定器仅提供批次，engine 构建由调用方负责。

#include <cstddef>
#include <limits>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <utility>

namespace vision
{
enum class GraphCaptureDecision {captured, fallback, failure};

// 调用者先收齐真实 CUDA 状态；致命错误优先于不兼容，缺提交/资源本身不能授权回退。
constexpr GraphCaptureDecision graphCaptureDecision(
  bool complete, bool unsupported, bool cuda_failed) noexcept
{
  if (cuda_failed) {return GraphCaptureDecision::failure;}
  if (unsupported) {return GraphCaptureDecision::fallback;}
  return complete ? GraphCaptureDecision::captured : GraphCaptureDecision::failure;
}

}  // namespace vision

#include "yolo/engine/engine.hpp"

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include <NvInfer.h>
#include <opencv2/core.hpp>



// 归一化尺度、填充值和通道顺序必须与网络真实输入以及 CUDA letterbox kernel 对齐。
class ImageFolderCalibrator : public nvinfer1::IInt8EntropyCalibrator2 {
public:
  /// @param dir 要扫描图像的目录（不递归）
  /// @param input_size engine 输入尺寸，即 letterbox 的目标尺寸
  /// @param cache_file calibration cache 路径；非空缓存交给 TensorRT 决定是否复用
  /// @param input_name ONNX 计算图的输入张量名
  /// @param cancellation 可选共享停止状态（非拥有），须比标定器活得更久；空指针保持原行为
  ImageFolderCalibrator(
    const std::string & dir, const cv::Size & input_size,
    std::string cache_file, std::string input_name,
    const BuildCancellation * cancellation = nullptr);

  ~ImageFolderCalibrator() override;

  ImageFolderCalibrator(const ImageFolderCalibrator &) = delete;
  ImageFolderCalibrator & operator=(const ImageFolderCalibrator &) = delete;

  // 固定每批一张图片；缓存指纹和批次必须使用同一份清单。
  int getBatchSize() const noexcept override;

  // 设备缓冲归标定器所有，绑定仅在成功时可用；异常转成错误信息并返回 false。
  bool getBatch(void * bindings[], char const * names[], int32_t nb_bindings) noexcept override;

  void const * readCalibrationCache(std::size_t & length) noexcept override;

  void writeCalibrationCache(void const * ptr, std::size_t length) noexcept override;

  nvinfer1::CalibrationAlgoType getAlgorithm() noexcept override
  {
    return nvinfer1::CalibrationAlgoType::kENTROPY_CALIBRATION_2;
  }

  /// 候选图片数量。为零无法开始标定；非零仍可能包含取批次时被跳过的无法解码图片。
  std::size_t imageCount() const noexcept {return files_.size();}

  /// 构建期间锁存的格式/资源错误；正常耗尽、跳过坏图或协作取消不会设置此错误。
  const std::string & lastError() const noexcept {return error_;}

private:
  /// 用经过 letterbox、归一化后的平面 RGB 张量填充 `host_blob_`。
  bool loadAndPreprocess(const std::string & path);

  bool stopped() const noexcept
  {return cancellation_ && cancellation_->requested();}

  const BuildCancellation * cancellation_{nullptr};
  std::vector<std::string> files_;
  std::size_t next_{0};
  cv::Size input_size_;
  std::string cache_file_;
  std::string input_name_;
  std::vector<float> host_blob_;
  void * device_blob_{nullptr};
  std::vector<char> cache_;
  std::string error_;
};

// Copyright 2026 jetson_yolo integration
// SPDX-License-Identifier: AGPL-3.0
#include <NvInfer.h>


// TensorRT 在优化步骤结束时读取停止状态；不能打断当前 CUDA 工作或承诺固定退出时限。
// cancellation 与 monitor 都必须比使用它们的 builder/config 活得更久。
class BuildProgressMonitor : public nvinfer1::IProgressMonitor {
public:
  explicit BuildProgressMonitor(const BuildCancellation & cancellation)
  : cancellation_(cancellation) {}
  void phaseStart(char const *, char const *, int32_t) noexcept override {}
  bool stepComplete(char const *, int32_t) noexcept override
  {return !cancellation_.requested();}
  void phaseFinish(char const *) noexcept override {}

private:
  const BuildCancellation & cancellation_;
};
