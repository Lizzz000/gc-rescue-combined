#include "yolo/yolo.hpp"
#include "yolo/cuda/nvmm_preprocess.hpp"
#include <algorithm>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <iterator>
#include <stdexcept>
#include <utility>

// Copyright 2026 jetson_yolo integration
// SPDX-License-Identifier: AGPL-3.0
//
// FastSegmentor 共用推理和交付路径；两种布局先筛选框，再按预算生成掩膜。


#include <chrono>
#include <cmath>
#include <cstdint>
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

/// 当前主机输出中的候选；掩膜系数只借用，不进入跨线程结果。
struct Candidate
{
  float conf{0.0f};
  int cls{0};
  BBox2i box{};
  cv::Rect2d letterbox_box{};  // 两种布局的 NMS 均使用未裁剪、未取整的 letterbox 坐标。
  const float * coeff{nullptr};
  std::size_t coeff_stride{1};
};

namespace {
// 两种布局共用原 legacy 的同类别 greedy NMS；置信度降序，同分保留解码顺序。
// 每个候选最多比较 max_detections 个已保留框，达到输出预算立即停止。
void limitCandidates(std::vector<Candidate> & candidates, const SegOptions & options)
{
  std::stable_sort(candidates.begin(), candidates.end(),
    [](const Candidate & a, const Candidate & b) {return a.conf > b.conf;});
  const std::size_t maximum = effectiveMaxDetections(options.max_detections);
  std::size_t kept = 0;
  for (std::size_t i = 0; i < candidates.size() && kept < maximum; ++i) {
    const Candidate & candidate = candidates[i];
    const auto & box = candidate.letterbox_box;
    bool suppressed = false;
    for (std::size_t j = 0; j < kept; ++j) {
      const Candidate & previous = candidates[j];
      if (candidate.cls != previous.cls) {continue;}
      const auto & other = previous.letterbox_box;
      const double width = std::min(box.x + box.width, other.x + other.width) -
        std::max(box.x, other.x);
      const double height = std::min(box.y + box.height, other.y + other.height) -
        std::max(box.y, other.y);
      if (width <= 0 || height <= 0) {continue;}
      const double intersection = width * height;
      const double area = box.area() + other.area() - intersection;
      if (area > 0 && intersection / area > options.iou_threshold) {
        suppressed = true;
        break;
      }
    }
    if (!suppressed) {candidates[kept++] = candidate;}
  }
  candidates.resize(kept);
}

// 保留 legacy 原有去 padding 区域，后续按原图插值再裁到实例框。
cv::Rect legacyMaskContent(
  const cv::Size & image, const cv::Size & input, const cv::Size & prototype)
{
  float gain, pad_x, pad_y;
  yolos::preprocessing::getScalePad(image, input, gain, pad_x, pad_y);
  const float scale_x = static_cast<float>(prototype.width) / input.width;
  const float scale_y = static_cast<float>(prototype.height) / input.height;
  const int left = std::clamp(static_cast<int>(std::round((pad_x - 0.1f) * scale_x)),
    0, prototype.width - 1);
  const int top = std::clamp(static_cast<int>(std::round((pad_y - 0.1f) * scale_y)),
    0, prototype.height - 1);
  const int right = std::clamp(
    static_cast<int>(std::round((input.width - pad_x + 0.1f) * scale_x)), left, prototype.width);
  const int bottom = std::clamp(
    static_cast<int>(std::round((input.height - pad_y + 0.1f) * scale_y)), top, prototype.height);
  return cv::Rect(left, top, right - left, bottom - top);
}
}  // namespace

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
  const bool need_masks = options.max_masks > 0 &&
    (options.want_label_mask || options.want_instance_masks);
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
  label_mask.release();
  const float * out0 = getOutputData(0);
  const auto & shape0 = getOutputShape(0);
  const bool end_to_end = end_to_end_;
  const int mask_scale = std::max(1, options.mask_scale);
  const bool need_masks = options.max_masks > 0 &&
    (options.want_label_mask || options.want_instance_masks);
  const float gain = getCachedScale();
  const float pad_x = getCachedPadX();
  const float pad_y = getCachedPadY();
  if (!std::isfinite(gain) || gain <= 0.0f) {return {};}
  if (options.want_label_mask) {
    label_mask = cv::Mat::zeros(labelMaskSize(image_size, mask_scale), CV_8UC1);
  }

  const std::size_t rows = static_cast<std::size_t>(shape0[end_to_end ? 1 : 2]);
  std::vector<Candidate> candidates;
  candidates.reserve(std::min<std::size_t>(rows, 64));
  if (end_to_end) {
    const std::size_t stride = static_cast<std::size_t>(shape0[2]);
    for (std::size_t i = 0; i < rows; ++i) {
      const float * row = out0 + i * stride;
      if (row[4] < options.conf_threshold) {continue;}
      Candidate candidate;
      if (!decodeEndToEndBox(row, stride, image_size, gain, pad_x, pad_y,
          candidate.box, candidate.cls, candidate.conf)) {continue;}
      candidate.letterbox_box = cv::Rect2d(row[0], row[1],
        static_cast<double>(row[2]) - row[0], static_cast<double>(row[3]) - row[1]);
      candidate.coeff = row + 6;
      candidates.push_back(candidate);
    }
  } else {
    const int classes = static_cast<int>(shape0[1]) - 4 - proto_count_;
    for (std::size_t i = 0; i < rows; ++i) {
      Candidate candidate;
      bool valid_scores = classes > 0;
      for (int cls = 0; cls < classes; ++cls) {
        const float score = out0[(static_cast<std::size_t>(4) + cls) * rows + i];
        if (!std::isfinite(score) || score < 0 || score > 1) {valid_scores = false; break;}
        if (cls == 0 || score > candidate.conf) {candidate.conf = score; candidate.cls = cls;}
      }
      if (!valid_scores || candidate.conf < options.conf_threshold) {continue;}
      const double center_x = out0[i], center_y = out0[rows + i];
      const double width = out0[2 * rows + i], height = out0[3 * rows + i];
      const double left = center_x - width * 0.5, top = center_y - height * 0.5;
      if (!decodeBoxCorners(left, top, left + width, top + height, image_size,
          gain, pad_x, pad_y, candidate.box)) {continue;}
      candidate.letterbox_box = cv::Rect2d(left, top, width, height);
      candidate.coeff = out0 + (static_cast<std::size_t>(4) + classes) * rows + i;
      candidate.coeff_stride = rows;
      candidates.push_back(candidate);
    }
  }
  limitCandidates(candidates, options);

  std::vector<SegDetection> results;
  results.reserve(candidates.size());
  for (const Candidate & candidate : candidates) {
    SegDetection detection;
    detection.confidence = candidate.conf;
    detection.class_id = candidate.cls;
    detection.bbox = candidate.box;
    const auto & names = getClassNames();
    if (candidate.cls >= 0 && static_cast<std::size_t>(candidate.cls) < names.size()) {
      detection.class_name = names[static_cast<std::size_t>(candidate.cls)];
    }
    results.push_back(std::move(detection));
  }
  if (!need_masks || results.empty() || proto_count_ <= 0) {return results;}

  // 只有预算内实例进入 mask 工作区；未回传 prototype 的帧完全不读取其旧主机缓冲。
  const cv::Size mask_frame = labelMaskSize(image_size, mask_scale);
  const float * out1 = getOutputData(1);
  cv::Mat prototypes(proto_count_, proto_size_.width * proto_size_.height, CV_32F,
    const_cast<float *>(out1));
  const Letterbox lb = letterboxFor(image_size, getInputShape());
  const cv::Rect legacy_content = end_to_end ? cv::Rect() :
    legacyMaskContent(image_size, getInputShape(), proto_size_);
  std::vector<float> legacy_coefficients(end_to_end ? 0 : static_cast<std::size_t>(proto_count_));
  const std::size_t mask_count = std::min(static_cast<std::size_t>(options.max_masks), results.size());
  // 置信度升序写入标签图，高分实例覆盖重叠区域。
  for (std::size_t count = mask_count; count > 0; --count) {
    const std::size_t index = count - 1;
    const Candidate & candidate = candidates[index];
    SegDetection & detection = results[index];
    cv::Mat binary;
    cv::Rect destination;
    if (end_to_end) {
      const cv::Rect2f source_box(static_cast<float>(detection.bbox.x),
        static_cast<float>(detection.bbox.y), static_cast<float>(detection.bbox.width),
        static_cast<float>(detection.bbox.height));
      cv::Rect mask_rect;
      if (!mapBoxToMaskRect(source_box, lb, getInputShape(), proto_size_, mask_rect)) {continue;}
      destination = scaleBoxToMask(detection.bbox, mask_scale);
      binary = mask_workspace_.render(prototypes, candidate.coeff, proto_size_, mask_rect,
        destination.size());
    } else {
      for (std::size_t i = 0; i < legacy_coefficients.size(); ++i) {
        legacy_coefficients[i] = candidate.coeff[i * candidate.coeff_stride];
      }
      binary = mask_workspace_.renderLegacy(prototypes, legacy_coefficients.data(),
        proto_size_, legacy_content, image_size, detection.bbox, mask_frame);
      destination = cv::Rect(0, 0, mask_frame.width, mask_frame.height);
    }
    if (binary.empty()) {continue;}
    const cv::Rect clipped = destination & cv::Rect(0, 0, mask_frame.width, mask_frame.height);
    if (clipped.empty()) {continue;}
    const cv::Mat patch = binary(cv::Rect(clipped.x - destination.x, clipped.y - destination.y,
      clipped.width, clipped.height));
    if (options.want_label_mask) {
      label_mask(clipped).setTo(cv::Scalar(maskLabelFor(detection.class_id,
          options.mask_label_offset)), patch);
    }
    if (options.want_instance_masks) {
      detection.mask = cv::Mat::zeros(mask_frame, CV_8UC1);
      patch.copyTo(detection.mask(clipped));
    }
  }
  return results;
}

static void fillBox(vision_msgs::msg::Detection2D & msg, const BBox2i & bbox)
{
  msg.bbox.center.position.x = static_cast<double>(bbox.x) + bbox.width / 2.0;
  msg.bbox.center.position.y = static_cast<double>(bbox.y) + bbox.height / 2.0;
  msg.bbox.center.theta = 0.0;
  msg.bbox.size_x = static_cast<double>(bbox.width);
  msg.bbox.size_y = static_cast<double>(bbox.height);
}

static void fillHypothesis(
  vision_msgs::msg::Detection2D & msg, int class_id, float confidence)
{
  vision_msgs::msg::ObjectHypothesisWithPose hypothesis;
  // 线上类别编号始终使用模型索引；显示标签不改变跨包协议中的类别 ID。
  hypothesis.hypothesis.class_id = std::to_string(class_id);
  hypothesis.hypothesis.score = static_cast<double>(confidence);
  msg.results.push_back(std::move(hypothesis));
}


vision_msgs::msg::Detection2DArray toDetection2DArray(
  const std::vector<SegDetection> & detections, const std_msgs::msg::Header & header)
{
  vision_msgs::msg::Detection2DArray msg;
  msg.header = header;
  msg.detections.reserve(detections.size());

  for (const auto & det : detections) {
    vision_msgs::msg::Detection2D det_msg;
    det_msg.header = header;
    fillBox(det_msg, det.bbox);
    fillHypothesis(det_msg, det.class_id, det.confidence);
    msg.detections.push_back(std::move(det_msg));
  }
  return msg;
}

sensor_msgs::msg::Image toLabelMaskImage(
  const cv::Mat & label_mask,
  const std_msgs::msg::Header & header)
{
  sensor_msgs::msg::Image msg;
  msg.header = header;
  msg.encoding = "mono8";
  msg.is_bigendian = 0;

  if (label_mask.empty() || label_mask.type() != CV_8UC1) {
    msg.width = 0;
    msg.height = 0;
    msg.step = 0;
    return msg;
  }

  msg.width = static_cast<std::uint32_t>(label_mask.cols);
  msg.height = static_cast<std::uint32_t>(label_mask.rows);
  msg.step = static_cast<std::uint32_t>(label_mask.cols);
  msg.data.resize(static_cast<std::size_t>(label_mask.cols) * label_mask.rows);

  // 逐行读取支持有行跨度的 ROI 矩阵；消息输出是紧密排列的 mono8，不复制原矩阵的填充字节。
  for (int r = 0; r < label_mask.rows; ++r) {
    std::copy_n(label_mask.ptr<std::uint8_t>(r), label_mask.cols,
                msg.data.begin() + static_cast<std::ptrdiff_t>(r) * label_mask.cols);
  }
  return msg;
}
