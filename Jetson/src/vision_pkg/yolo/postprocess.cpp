#include "yolo/postprocess.hpp"
// Copyright 2026 jetson_yolo integration
// SPDX-License-Identifier: AGPL-3.0
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <utility>
#include <opencv2/imgproc.hpp>

Letterbox letterboxFor(const cv::Size & src, const cv::Size & input)
{
  Letterbox lb;
  if (src.width <= 0 || src.height <= 0 || input.width <= 0 || input.height <= 0) {
    return lb;
  }
  lb.scale = std::min(static_cast<float>(input.height) / static_cast<float>(src.height),
                      static_cast<float>(input.width) / static_cast<float>(src.width));
  lb.new_w = static_cast<int>(std::round(static_cast<float>(src.width) * lb.scale));
  lb.new_h = static_cast<int>(std::round(static_cast<float>(src.height) * lb.scale));
  lb.pad_left = static_cast<int>(
    std::round((static_cast<float>(input.width - lb.new_w) / 2.0f) - 0.1f));
  lb.pad_top = static_cast<int>(
    std::round((static_cast<float>(input.height - lb.new_h) / 2.0f) - 0.1f));
  return lb;
}

namespace {
inline bool decodeBoxCorners(
  double x0, double y0, double x1, double y1,
  const cv::Size & image, double gain, double pad_x, double pad_y, BBox2i & box)
{
  if (image.width <= 0 || image.height <= 0 || !std::isfinite(gain) || gain <= 0 ||
    !std::isfinite(pad_x) || !std::isfinite(pad_y) || !std::isfinite(x0) ||
    !std::isfinite(y0) || !std::isfinite(x1) || !std::isfinite(y1) || x1 <= x0 || y1 <= y0)
  {return false;}
  x0 = (x0 - pad_x) / gain;
  y0 = (y0 - pad_y) / gain;
  x1 = (x1 - pad_x) / gain;
  y1 = (y1 - pad_y) / gain;
  if (!std::isfinite(x0) || !std::isfinite(y0) || !std::isfinite(x1) ||
    !std::isfinite(y1) || x1 <= 0 || y1 <= 0 || x0 >= image.width || y0 >= image.height)
  {return false;}
  const int left = std::min(image.width - 1,
    static_cast<int>(std::lround(std::clamp(x0, 0.0, static_cast<double>(image.width)))));
  const int top = std::min(image.height - 1,
    static_cast<int>(std::lround(std::clamp(y0, 0.0, static_cast<double>(image.height)))));
  const int right = static_cast<int>(std::lround(std::clamp(x1, 0.0,
      static_cast<double>(image.width))));
  const int bottom = static_cast<int>(std::lround(std::clamp(y1, 0.0,
      static_cast<double>(image.height))));
  if (right <= left || bottom <= top) {return false;}
  box = BBox2i{left, top, right - left, bottom - top};
  return true;
}

// 解码 end-to-end 行的前六项。类号和置信度合法后才写入交付结果。
inline bool decodeEndToEndBox(
  const float * row, std::size_t values, const cv::Size & image,
  double gain, double pad_x, double pad_y,
  BBox2i & box, int & class_id, float & confidence)
{
  if (row == nullptr || values < 6) {return false;}
  const double cls = row[5];
  if (!std::isfinite(row[4]) || !std::isfinite(cls) || row[4] < 0 || row[4] > 1 ||
    cls < 0 || cls > std::numeric_limits<int>::max() || std::trunc(cls) != cls ||
    !decodeBoxCorners(row[0], row[1], row[2], row[3], image, gain, pad_x, pad_y, box))
  {return false;}
  class_id = static_cast<int>(cls);
  confidence = row[4];
  return true;
}

/// 将类别编号映射为 mono8 标签；先用宽整数相加，避免大类别编号溢出。
inline int maskLabelFor(int class_id, int label_offset)
{
  return static_cast<int>(std::clamp<std::int64_t>(
    static_cast<std::int64_t>(class_id) + label_offset, 1, 255));
}

/// 把原始图像中的一个框映射为 mask 平面上覆盖它的整数像素矩形。
/// 结果为空时返回 false，此时调用方必须跳过该实例的
/// mask 生成。
inline bool mapBoxToMaskRect(
  const cv::Rect2f & src_box, const Letterbox & lb,
  const cv::Size & input, const cv::Size & mask,
  cv::Rect & out)
{
  out = {};
  if (mask.width <= 0 || mask.height <= 0 || input.width <= 0 || input.height <= 0 ||
    !std::isfinite(lb.scale) || lb.scale <= 0 ||
    !std::isfinite(src_box.x) || !std::isfinite(src_box.y) ||
    !std::isfinite(src_box.width) || !std::isfinite(src_box.height) ||
    src_box.width <= 0 || src_box.height <= 0) {return false;}

  const auto map_x = [&](double x) {
      return (x * lb.scale + lb.pad_left) * mask.width / input.width;
    };
  const auto map_y = [&](double y) {
      return (y * lb.scale + lb.pad_top) * mask.height / input.height;
    };
  const double x0 = map_x(src_box.x), y0 = map_y(src_box.y);
  const double x1 = map_x(static_cast<double>(src_box.x) + src_box.width);
  const double y1 = map_y(static_cast<double>(src_box.y) + src_box.height);
  if (!std::isfinite(x0) || !std::isfinite(y0) || !std::isfinite(x1) || !std::isfinite(y1)) {
    return false;
  }
  const int ix0 = static_cast<int>(std::floor(std::clamp(x0, 0.0, static_cast<double>(mask.width))));
  const int iy0 = static_cast<int>(std::floor(std::clamp(y0, 0.0, static_cast<double>(mask.height))));
  const int ix1 = static_cast<int>(std::ceil(std::clamp(x1, 0.0, static_cast<double>(mask.width))));
  const int iy1 = static_cast<int>(std::ceil(std::clamp(y1, 0.0, static_cast<double>(mask.height))));

  if (ix1 - ix0 < 1 || iy1 - iy0 < 1) {return false;}

  out = cv::Rect(ix0, iy0, ix1 - ix0, iy1 - iy0);
  return true;
}

// 标签图尺寸向上取整，确保非整除的源图边缘仍有存储；框位置/范围沿用下方既有整数映射规则。
/// 给定源尺寸与降采样倍数时，全帧 label mask 的尺寸。
inline cv::Size labelMaskSize(const cv::Size & src, int mask_scale)
{
  const int ms = std::max(1, mask_scale);
  const int w = std::max(1, src.width / ms + (src.width % ms > 0 ? 1 : 0));
  const int h = std::max(1, src.height / ms + (src.height % ms > 0 ? 1 : 0));
  return cv::Size(w, h);
}

/// 按原有整数除法规约映射检测框，框宽高至少保留一像素；调用者仍须与标签图实际范围求交。
inline cv::Rect scaleBoxToMask(const BBox2i & b, int mask_scale)
{
  const int ms = std::max(1, mask_scale);
  return cv::Rect(b.x / ms, b.y / ms,
                  std::max(1, b.width / ms), std::max(1, b.height / ms));
}

class MaskWorkspace
{
public:
  // 返回内部二值缓冲的只读借用视图，下次 render() 会覆盖它，扩容也可能使旧视图失效。
  // 调用方必须在下一次 render 前完成 setTo/copyTo，不能直接放入跨线程结果。
  cv::Mat render(
    const cv::Mat & prototypes, const float * coefficients,
    const cv::Size & prototype_size, const cv::Rect & roi,
    const cv::Size & output_size)
  {
    const auto plane = static_cast<std::int64_t>(prototype_size.width) * prototype_size.height;
    if (!coefficients || prototype_size.width <= 0 || prototype_size.height <= 0 ||
      plane > std::numeric_limits<int>::max() || prototypes.type() != CV_32FC1 ||
      prototypes.rows <= 0 || prototypes.cols != plane || roi.width <= 0 || roi.height <= 0 ||
      (roi & cv::Rect(0, 0, prototype_size.width, prototype_size.height)) != roi ||
      output_size.width <= 0 || output_size.height <= 0) {return {};}
    if (std::any_of(coefficients, coefficients + prototypes.rows,
        [](float value) {return !std::isfinite(value);})) {return {};}

    // coefficients 和原型列带均为视图，不复制模型输出；GEMM 保留原型的行步长。
    const cv::Mat weights(1, prototypes.rows, CV_32F, const_cast<float *>(coefficients));
    const int begin = roi.y * prototype_size.width;
    const int end = (roi.y + roi.height) * prototype_size.width;
    cv::Mat logits = logits_buffer_.view(1, end - begin, CV_32F);
    cv::gemm(weights, prototypes.colRange(begin, end), 1.0, cv::Mat(), 0.0, logits);
    logits_buffer_.verifyView(logits);
    const cv::Mat band = logits.reshape(1, roi.height);
    const cv::Mat cropped = band(cv::Rect(roi.x, 0, roi.width, roi.height));

    // 尺寸不同仍复用容量；各算子在当前尺寸视图内写入，不初始化或复制旧 scratch。
    cv::Mat probability = probability_buffer_.view(roi.height, roi.width, CV_32F);
    cv::Mat resized = resized_buffer_.view(output_size.height, output_size.width, CV_32F);
    cv::Mat binary = binary_buffer_.view(output_size.height, output_size.width, CV_8UC1);
    cv::multiply(cropped, -1.0, probability, 1.0, CV_32F);
    cv::exp(probability, probability);
    cv::add(probability, 1.0, probability, cv::noArray(), CV_32F);
    cv::divide(1.0, probability, probability, CV_32F);
    probability_buffer_.verifyView(probability);
    cv::resize(probability, resized, output_size, 0, 0, cv::INTER_LINEAR);
    resized_buffer_.verifyView(resized);
    // 1x1 数组与标量在部分 OpenCV 版本存在形状歧义，显式处理单像素。
    if (resized.total() == 1) {
      binary.at<std::uint8_t>(0, 0) = resized.at<float>(0, 0) > 0.5f ? 255 : 0;
    } else {
      cv::compare(resized, 0.5, binary, cv::CMP_GT);
    }
    binary_buffer_.verifyView(binary);
    return binary;
  }

  // legacy 沿用去 padding 后整图插值、阈值、框裁剪、最近邻降采样的顺序。
  // 只有预算内的实例会进入这里；中间全图只保留一份线程私有缓冲。
  cv::Mat renderLegacy(
    const cv::Mat & prototypes, const float * coefficients,
    const cv::Size & prototype_size, const cv::Rect & content,
    const cv::Size & image_size, const BBox2i & box, const cv::Size & output_size)
  {
    cv::Mat binary = render(prototypes, coefficients, prototype_size, content, image_size);
    if (binary.empty()) {return {};}
    const cv::Rect clipped = cv::Rect(box.x, box.y, box.width, box.height) &
      cv::Rect(0, 0, image_size.width, image_size.height);
    if (clipped.empty()) {return {};}
    if (clipped.y > 0) {binary.rowRange(0, clipped.y).setTo(0);}
    if (clipped.br().y < binary.rows) {binary.rowRange(clipped.br().y, binary.rows).setTo(0);}
    if (clipped.x > 0) {binary.colRange(0, clipped.x).setTo(0);}
    if (clipped.br().x < binary.cols) {binary.colRange(clipped.br().x, binary.cols).setTo(0);}
    if (output_size == image_size) {return binary;}
    cv::Mat small = legacy_scaled_buffer_.view(output_size.height, output_size.width, CV_8UC1);
    cv::resize(binary, small, output_size, 0, 0, cv::INTER_NEAREST);
    legacy_scaled_buffer_.verifyView(small);
    return small;
  }

private:
  class ScratchBuffer
  {
  public:
    cv::Mat view(int rows, int cols, int type)
    {
      const auto bytes = static_cast<std::size_t>(rows) * cols * CV_ELEM_SIZE(type);
      if (bytes > capacity_) {
        // 预留增长余量，逐像素变大的目标也无需逐帧重新分配；scratch 不复制、不清零。
        const auto half = capacity_ / 2;
        const auto grown_capacity =
          capacity_ <= std::numeric_limits<std::size_t>::max() - half ?
          capacity_ + half : capacity_;
        const auto next_capacity = std::max(bytes, grown_capacity);
        std::unique_ptr<unsigned char, AlignedFree> next(
          static_cast<unsigned char *>(cv::fastMalloc(next_capacity)));
        data_ = std::move(next);
        capacity_ = next_capacity;
      }
      return cv::Mat(rows, cols, type, data_.get());
    }

    void verifyView(const cv::Mat & output) const
    {
      // OpenCV 应使用给定的相同尺寸/type 视图；不允许自动分配悄悄破坏容量复用。
      if (output.data != data_.get()) {
        throw std::runtime_error("OpenCV reallocated a mask workspace output view");
      }
    }

  private:
    struct AlignedFree
    {
      void operator()(unsigned char * pointer) const noexcept {cv::fastFree(pointer);}
    };

    std::unique_ptr<unsigned char, AlignedFree> data_;
    std::size_t capacity_{0};
  };

  ScratchBuffer logits_buffer_;
  ScratchBuffer probability_buffer_;
  ScratchBuffer resized_buffer_;
  ScratchBuffer binary_buffer_;
  ScratchBuffer legacy_scaled_buffer_;
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

std::size_t effectiveMaxDetections(int maximum)
{return static_cast<std::size_t>(std::max(1, maximum));}

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
  // 固定上游 v3.1.0 getScalePad：新尺寸 round 后，padding 保留半像素。
  const Letterbox lb = letterboxFor(image, input);
  const float pad_x = (input.width - lb.new_w) / 2.0f;
  const float pad_y = (input.height - lb.new_h) / 2.0f;
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

struct SegPostprocessor::Workspace
{
  MaskWorkspace mask;
  std::vector<Candidate> candidates;
  std::vector<float> legacy_coefficients;
};

SegPostprocessor::SegPostprocessor() : workspace_(std::make_unique<Workspace>()) {}
SegPostprocessor::~SegPostprocessor() = default;

std::vector<SegDetection> SegPostprocessor::run(
  const SegOutputView & output, const SegOptions & options,
  const std::vector<std::string> & names, cv::Mat & label_mask)
{
  label_mask.release();
  const float * out0 = output.detections;
  const bool end_to_end = output.end_to_end;
  const auto & image_size = output.image_size;
  const int proto_count_ = output.prototype_count;
  const auto & proto_size_ = output.prototype_size;
  const int mask_scale = std::max(1, options.mask_scale);
  const bool need_masks = options.needsMasks();
  const float gain = output.gain;
  const float pad_x = output.pad_x;
  const float pad_y = output.pad_y;
  if (!std::isfinite(gain) || gain <= 0.0f) {return {};}
  if (options.want_label_mask) {
    label_mask = cv::Mat::zeros(labelMaskSize(image_size, mask_scale), CV_8UC1);
  }

  const std::size_t rows = output.rows;
  auto & candidates = workspace_->candidates;
  candidates.clear();
  candidates.reserve(std::min<std::size_t>(rows, 64));
  if (end_to_end) {
    const std::size_t stride = output.features;
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
    const int classes = static_cast<int>(output.features) - 4 - proto_count_;
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
    if (candidate.cls >= 0 && static_cast<std::size_t>(candidate.cls) < names.size()) {
      detection.class_name = names[static_cast<std::size_t>(candidate.cls)];
    }
    results.push_back(std::move(detection));
  }
  if (!need_masks || results.empty() || proto_count_ <= 0) {return results;}

  // 只有预算内实例进入 mask 工作区；未回传 prototype 的帧完全不读取其旧主机缓冲。
  const cv::Size mask_frame = labelMaskSize(image_size, mask_scale);
  const float * out1 = output.prototypes;
  cv::Mat prototypes(proto_count_, proto_size_.width * proto_size_.height, CV_32F,
    const_cast<float *>(out1));
  const Letterbox lb = letterboxFor(image_size, output.input_size);
  const cv::Rect legacy_content = end_to_end ? cv::Rect() :
    legacyMaskContent(image_size, output.input_size, proto_size_);
  auto & legacy_coefficients = workspace_->legacy_coefficients;
  legacy_coefficients.resize(end_to_end ? 0 : static_cast<std::size_t>(proto_count_));
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
      if (!mapBoxToMaskRect(source_box, lb, output.input_size, proto_size_, mask_rect)) {continue;}
      destination = scaleBoxToMask(detection.bbox, mask_scale);
      binary = workspace_->mask.render(prototypes, candidate.coeff, proto_size_, mask_rect,
        destination.size());
    } else {
      for (std::size_t i = 0; i < legacy_coefficients.size(); ++i) {
        legacy_coefficients[i] = candidate.coeff[i * candidate.coeff_stride];
      }
      binary = workspace_->mask.renderLegacy(prototypes, legacy_coefficients.data(),
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
