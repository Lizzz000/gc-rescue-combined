#include "yolo/yolo.hpp"
#include "yolo/cuda/nvmm_preprocess.hpp"
// Copyright 2026 jetson_yolo integration
// SPDX-License-Identifier: AGPL-3.0
#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>
#include <stdexcept>

using ms_d = std::chrono::duration<double, std::milli>;

/// 作用域出口回填耗时，覆盖多条早退路径。
class ScopedMilliseconds {
public:
  explicit ScopedMilliseconds(double * out)
  : out_(out), start_(std::chrono::steady_clock::now()) {}
  ~ScopedMilliseconds()
  {
    if (out_ != nullptr) {
      *out_ = ms_d(std::chrono::steady_clock::now() - start_).count();
    }
  }
  ScopedMilliseconds(const ScopedMilliseconds &) = delete;
  ScopedMilliseconds & operator=(const ScopedMilliseconds &) = delete;

private:
  double * out_;
  std::chrono::steady_clock::time_point start_;
};

FastSegmentor::FastSegmentor(
  const std::string & engine_path,
  const std::string & labels_path, int dla_core,
  std::optional<SegOutputLayout> declared_layout)
: yolos::seg::YOLOSegDetector(engine_path, labels_path, dla_core)
{
  configureOutputLayout(getClassNames().size(), declared_layout);

  std::cout << "[segmentor] engine input " << getInputShape().width << "x"
            << getInputShape().height << ", prototypes " << proto_count_ << " @ "
            << proto_size_.width << "x" << proto_size_.height << ", layout "
            << (end_to_end_ ? "yolo26 end-to-end (bounded class-aware NMS)" : "legacy (bounded class-aware NMS)")
            << std::endl;
}

void FastSegmentor::configureOutputLayout(
  std::size_t class_count, std::optional<SegOutputLayout> declared_layout)
{
  const auto & detections = getOutputShape(0);
  const auto & prototypes = getOutputShape(1);
  SegOutputLayout layout;
  std::string error;
  if (!resolveSegOutputLayout(detections, prototypes, class_count, layout, error, declared_layout)) {
    throw std::invalid_argument(error);
  }
  proto_count_ = static_cast<int>(prototypes[1]);
  proto_size_ = cv::Size(static_cast<int>(prototypes[3]), static_cast<int>(prototypes[2]));
  end_to_end_ = layout == SegOutputLayout::EndToEnd;
  num_classes_ = class_count ? static_cast<int>(class_count) :
    (end_to_end_ ? 0 : static_cast<int>(detections[1] - 4 - prototypes[1]));
}

std::vector<SegDetection> FastSegmentor::segmentFast(
  const cv::Mat & image,
  const SegOptions & options,
  cv::Mat & label_mask,
  SegTiming * timing,
  const void * nvmm)
{
  if (timing != nullptr) {
    timing->gpu_ms = 0.0;
    timing->post_ms = 0.0;
  }
  label_mask.release();
  if (image.empty()) {return {};}
  if (image.dims != 2 || image.rows <= 0 || image.cols <= 0 || image.type() != CV_8UC3) {
    throw std::invalid_argument(
      "segmentation expects a two-dimensional 8-bit three-channel BGR image");
  }
  if (!std::isfinite(options.conf_threshold) || options.conf_threshold < 0 ||
    options.conf_threshold > 1 || !std::isfinite(options.iou_threshold) ||
    options.iou_threshold < 0 || options.iou_threshold > 1)
  {throw std::invalid_argument("segmentation confidence and IoU thresholds must be in [0,1]");}
  // 两种布局均只在确有掩膜消费者且预算非零时回传 prototype。
  const bool need_masks = options.needsMasks();
  // file、V4L2 和未协商出 NVMM 的管线没有表面。空指针不能进驱动映射，否则推理线程会直接崩溃。
  if (nvmm == nullptr) {
    throw std::invalid_argument(
      "segmentation requires an NVMM frame; file, V4L2 and pipelines that did not negotiate NVMM cannot segment");
  }
  {
    const auto gpu_start = std::chrono::steady_clock::now();
    const std::size_t host_outputs = need_masks ? getNumOutputNodes() : std::size_t{1};
    inferNvmm(nvmm, host_outputs);
    if (timing != nullptr) {
      timing->gpu_ms = ms_d(std::chrono::steady_clock::now() - gpu_start).count();
    }
  }
  ScopedMilliseconds post_timer(timing != nullptr ? &timing->post_ms : nullptr);
  return postprocessOutputs(image.size(), options, label_mask);
}

void FastSegmentor::inferNvmm(const void * nvmm, std::size_t host_output_count)
{
  const ::vision::NvmmView view(nvmm);
  ::vision::nvmmLetterbox(view, static_cast<float *>(inputTensors_[0].deviceBuf.data()),
    inputShape_.width, inputShape_.height, stream_.get());
  CUDA_CHECK(cudaGetLastError());
  if (graphCaptured_ && graphExec_) {
    CUDA_CHECK(cudaGraphLaunch(graphExec_, stream_.get()));
  } else {
    ::vision::requireBackendSubmission(context_->enqueueV3(stream_.get()), "inferNvmm");
  }
  for (std::size_t i = 0; i < std::min(host_output_count, outputTensors_.size()); ++i) {
    auto & out = outputTensors_[i];
    CUDA_CHECK(cudaMemcpyAsync(out.hostBuf.data(), out.deviceBuf.data(), out.bytes,
      cudaMemcpyDeviceToHost, stream_.get()));
  }
  // EGL 注册在 view 析构时解除，必须在同步之后。
  stream_.synchronize();
  // 与上游 computeScalePad（private）同一公式，供后处理反算坐标。
  const int dst_w = inputShape_.width;
  const int dst_h = inputShape_.height;
  cachedScale_ = std::min(static_cast<float>(dst_h) / view.height,
    static_cast<float>(dst_w) / view.width);
  const int new_w = static_cast<int>(std::round(view.width * cachedScale_));
  const int new_h = static_cast<int>(std::round(view.height * cachedScale_));
  cachedPadX_ = std::round((dst_w - new_w) / 2.0f - 0.1f);
  cachedPadY_ = std::round((dst_h - new_h) / 2.0f - 0.1f);
}

std::vector<SegDetection> FastSegmentor::postprocessOutputs(
  const cv::Size & image_size, const SegOptions & options, cv::Mat & label_mask)
{
  const auto & shape = getOutputShape(0);
  SegOutputView output;
  output.detections = getOutputData(0);
  output.prototypes = options.needsMasks() ? getOutputData(1) : nullptr;
  output.rows = static_cast<std::size_t>(shape[end_to_end_ ? 1 : 2]);
  output.features = static_cast<std::size_t>(shape[end_to_end_ ? 2 : 1]);
  output.prototype_count = proto_count_;
  output.prototype_size = proto_size_;
  output.image_size = image_size;
  output.input_size = getInputShape();
  output.gain = getCachedScale();
  output.pad_x = getCachedPadX();
  output.pad_y = getCachedPadY();
  output.end_to_end = end_to_end_;
  return postprocessor_.run(output, options, getClassNames(), label_mask);
}
