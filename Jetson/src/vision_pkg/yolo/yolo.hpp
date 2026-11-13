#pragma once
// Copyright 2026 jetson_yolo integration
// SPDX-License-Identifier: AGPL-3.0
//
// 包内本地数据传输对象（DTO）。
//
// 这里不包含任何 TensorRT 或 `yolos::` 头文件，所以转换层
// 和单元测试无需 GPU 即可构建并运行。

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

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

// Copyright 2026 jetson_yolo integration
// SPDX-License-Identifier: AGPL-3.0
//
// 两种布局共用 effectiveMaxDetections() 归一化检测数量上限：非正数按 1 处理。
// 两种布局在 yolo.cpp 的 limitCandidates() 中按同一上限执行有界同类别 NMS，
// 同分保留解码顺序。下方 limitByConfidence() 仅提供纯值排序截断，不执行 NMS。
/// 上限规则：非正数按 1 处理。
inline std::size_t effectiveMaxDetections(int max_detections)
{
  return static_cast<std::size_t>(std::max(1, max_detections));
}

/// 按置信度降序稳定排序并截断，返回保留数量。
/// @param confidence 从候选或 SegDetection 读取置信度；本函数不执行 NMS。
template<typename Item, typename Confidence>
std::size_t limitByConfidence(
  std::vector<Item> & items, int max_detections, Confidence confidence)
{
  const std::size_t keep = std::min(items.size(), effectiveMaxDetections(max_detections));
  std::stable_sort(items.begin(), items.end(),
    [&confidence](const Item & a, const Item & b) {return confidence(a) > confidence(b);});
  items.resize(keep);
  return keep;
}

/// SegDetection 的纯值截断入口；生产两种布局均使用 yolo.cpp 中的有界 NMS。
inline std::size_t limitDetections(std::vector<SegDetection> & detections, int max_detections)
{
  return limitByConfidence(
    detections, max_detections,
    [](const SegDetection & detection) {return detection.confidence;});
}

// Copyright 2026 jetson_yolo integration
// SPDX-License-Identifier: AGPL-3.0
//
// letterbox 几何计算，刻意做到不依赖 CUDA 与 TensorRT，因此可以在
// 没有 GPU 的机器上做单元测试。
//
// 这些辅助函数必须与上游 CUDA kernel 保持一致，该 kernel 位于
// YOLOs-CPP-TensorRT/include/yolos/core/cuda_preprocessing.cu:
//
// scale = min(input.h / src.h, input.w / src.w)
// new_w = round(src.w * scale)
// new_h = round(src.h * scale)
// pad_left = round((input.w - new_w) / 2 - 0.1) <-- 注意这里的 -0.1
// pad_top = round((input.h - new_h) / 2 - 0.1)
//
// 而 `yolos::preprocessing::getScalePad()` 返回的反而是 *未取整* 的 padding，
// 所以它在这里不能用：用它会让每个 mask 相对于 decoder 产出的框偏移
// 最多一个像素。

// TensorRT 后处理使用的掩膜尺度、边界检查和标签饱和约定。

// 新的几何或边界校验不满足就返回 false 或空矩形。调用方按跳过该实例处理。
// 先检查浮点端点，再裁剪两端并转整数；完全画外的框不能夹成边缘假目标。
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

/// 针对一组（源图像, engine 输入）解析出的 letterbox 参数。
struct Letterbox
{
  float scale{1.0f};
  int new_w{0};
  int new_h{0};
  int pad_left{0};
  int pad_top{0};
};

/// 计算 CUDA 预处理 kernel 将会使用的 letterbox 参数。
inline Letterbox letterboxFor(const cv::Size & src, const cv::Size & input)
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

// Copyright 2026 jetson_yolo integration
// SPDX-License-Identifier: AGPL-3.0
// 一次 GEMM 只计算框覆盖的原型行带；sigmoid 仅作用于实际 ROI。
// 与旧路径保持相同的 sigmoid、双线性插值、0.5 阈值顺序。

// 单个推理线程独占可复用中间缓冲；只算框所需行带，最终输出由调用方另行持有。
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

// Copyright 2026 jetson_yolo integration
// SPDX-License-Identifier: AGPL-3.0
//
// DTO -> ROS message。纯函数，不涉及 GPU，可做单元测试。
//
// 检测变成 Detection2DArray，标签掩膜变成 mono8 Image。不依赖 CUDA 或 TensorRT。
// 检测框使用原图像素；掩膜尺寸已由推理适配器生成，本转换只打包既有像素，不再缩放。


#include <vector>

#include <opencv2/core.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/header.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>




// 数组及每个检测复用同一源 header；类别编号保持模型索引，显示名称不改线上类别身份。
vision_msgs::msg::Detection2DArray toDetection2DArray(
  const std::vector<SegDetection> & detections,
  const std_msgs::msg::Header & header);

// mono8 编码和降采样分辨率是上下游契约；本函数按传入矩阵的实际宽高逐行复制。
/// `mono8`，0表示背景；非零标签由适配器将class_id加mask_label_offset后饱和到[1,255]。
/// 该图像是原始帧的 1/mask_scale；它自身的 width/height 已经携带了这一信息，
/// 因此使用方不需要该参数。
sensor_msgs::msg::Image toLabelMaskImage(
  const cv::Mat & label_mask,
  const std_msgs::msg::Header & header);

// Copyright 2026 jetson_yolo integration
// SPDX-License-Identifier: AGPL-3.0
//
// TensorRT 分割适配器：沿用上游模型加载、GPU 前处理和推理，
// 按掩膜消费者决定主机输出回传范围，并在本地解码框与掩膜。
// 无掩膜需求或预算为零时只回传检测输出；需要掩膜时回传本帧全部输出，
// 再由 MaskWorkspace 计算原型行带，
// 对框内概率做 sigmoid、resize 和二值化，再写入独立输出图。
// 两种布局均在本地完成合法性检查和有数量上限的同类别 NMS，再给预算内实例生成掩膜。
// 可复用工作缓冲只归推理线程使用，跨线程结果不得引用这些缓冲。



// 上游分割头文件经 patch_host_outputs.cmake 引用 vision::GraphCaptureDecision；
// 这里先引入项目适配头，同时提供 SegOutputLayout 与 std::optional 的声明。
#include "yolo/tensorrt/tensorrt.hpp"

#include <yolos/tasks/segmentation.hpp>




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
};

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

  MaskWorkspace mask_workspace_;  // 仅推理线程使用，输出不得引用其内部缓冲。
  bool end_to_end_{false};
  int proto_count_{32};
  cv::Size proto_size_{0, 0};
  int num_classes_{0};
};
