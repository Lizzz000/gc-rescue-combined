// 阅读：把 BGR 图像编成 JPEG。观看页和 HTTP 在 stream/mjpeg。
// Jetson 上优先走 nvjpeg 插件；不支持的尺寸交给 OpenCV。
// 不采集相机，也不跑推理。
#include "stream/jpeg/jpeg.hpp"

#include <algorithm>
#include <cmath>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

// ---- 原 stream/jpeg/jpeg.hpp 的 BGR→I420 色转（只有本文件使用） ----
// Copyright 2026 jetson_yolo integration
// SPDX-License-Identifier: Apache-2.0
// JPEG全范围色转：保留每像素16位定点舍入，再做h2v2色度均值，不能先平均BGR。
// 公式/交替舍入依据 libjpeg-turbo 2.1.2 的 jccolor.c、jcsample.c（IJG许可）；
// 本文件为本项目独立实现，未编入或修改第三方源码。
// 上游源码及许可：https://github.com/libjpeg-turbo/libjpeg-turbo/tree/2.1.2
// 标量与AArch64 NEON使用相同整数公式；已验证的差分合同仍须随修改复测。
#include <opencv2/core.hpp>
#include <algorithm>
#include <array>
#include <cstddef>
#include <cstring>
#include <stdexcept>
#include <vector>

namespace vision_stream_detail {
using Plane = std::vector<unsigned char>;
using ChromaRows = std::array<Plane, 4>;  // 两行Cb、两行Cr，跨帧复用工作区。

inline void checkSize(int width, int height) {
  if (width <= 0 || height <= 0 || width > 32768 || height > 32768 || width % 2 || height % 2)
    throw std::invalid_argument("JPEG full-range I420 requires positive even dimensions <=32768");
}

// 先生成可见行，再按需填充行末；两个源行的色度先各自舍入，再平均。
// 先平均BGR会改变颜色取整结果，不能作为等价替代。
inline void convertVisible(const cv::Mat & bgr, unsigned char * y, unsigned char * u,
  unsigned char * v, int y_stride, int c_stride, int chroma_columns, ChromaRows & rows) {
  checkSize(bgr.cols, bgr.rows);
  if (bgr.type() != CV_8UC3 || y_stride < bgr.cols || c_stride < bgr.cols / 2 ||
      chroma_columns < bgr.cols / 2 || chroma_columns > c_stride)
    throw std::invalid_argument("JPEG full-range I420 type/stride mismatch");
  for (auto & row : rows) row.resize(bgr.cols);
  constexpr int half = 32768, chroma_offset = (128 << 16) + half - 1;
  for (int pair = 0; pair < bgr.rows / 2; ++pair) {
    for (int local = 0; local < 2; ++local) {
      const auto * src = bgr.ptr<cv::Vec3b>(pair * 2 + local);
      auto * yy = y + (pair * 2 + local) * y_stride;
      auto * cb = rows[local].data(); auto * cr = rows[local + 2].data();
      for (int x = 0; x < bgr.cols; ++x) {
        const int b = src[x][0], g = src[x][1], r = src[x][2];
        yy[x] = static_cast<unsigned char>((19595 * r + 38470 * g + 7471 * b + half) >> 16);
        cb[x] = static_cast<unsigned char>((-11059 * r - 21709 * g + 32768 * b + chroma_offset) >> 16);
        cr[x] = static_cast<unsigned char>((32768 * r - 27439 * g - 5329 * b + chroma_offset) >> 16);
      }
      std::fill(yy + bgr.cols, yy + y_stride, yy[bgr.cols - 1]);
    }
    auto * uu = u + pair * c_stride; auto * vv = v + pair * c_stride;
    for (int cx = 0; cx < chroma_columns; ++cx) {
      const int left = std::min(cx * 2, bgr.cols - 1), right = std::min(cx * 2 + 1, bgr.cols - 1);
      const int bias = 1 + (cx & 1);  // 每条色度输出行都从1开始，在1/2间交替，保持原下采样舍入。
      uu[cx] = static_cast<unsigned char>((rows[0][left] + rows[0][right] + rows[1][left] + rows[1][right] + bias) >> 2);
      vv[cx] = static_cast<unsigned char>((rows[2][left] + rows[2][right] + rows[3][left] + rows[3][right] + bias) >> 2);
    }
  }
}

class PackedI420Converter {
public:
  // 返回Mat独占新像素，必须保留到消费者释放；下一次转换不会覆盖它。
  cv::Mat convert(const cv::Mat & bgr) {
    checkSize(bgr.cols, bgr.rows);
    cv::Mat output(bgr.rows * 3 / 2, bgr.cols, CV_8UC1);
    fill(bgr, output.data); return output;
  }
  // 调用方可复用vector，但消费者持有期间不能再次写入同一缓冲。
  void convert(const cv::Mat & bgr, Plane & output) {
    checkSize(bgr.cols, bgr.rows);
    output.resize(std::size_t(bgr.cols) * bgr.rows * 3 / 2); fill(bgr, output.data());
  }
private:
  void fill(const cv::Mat & bgr, unsigned char * output) {
    const std::size_t y_size = std::size_t(bgr.cols) * bgr.rows;
    convertVisible(bgr, output, output + y_size, output + y_size + y_size / 4,
      bgr.cols, bgr.cols / 2, bgr.cols / 2, rows_);
  }
  ChromaRows rows_;
};
inline cv::Mat libjpegBgrToI420(const cv::Mat & bgr) {PackedI420Converter converter; return converter.convert(bgr);}

}  // namespace vision_stream_detail

#if defined(__aarch64__)
#include <arm_neon.h>
namespace vision_stream_detail {
namespace neon_detail {
struct Four {uint16x4_t y, u, v;};
inline Four four(uint16x4_t b, uint16x4_t g, uint16x4_t r) {
  auto yy = vmull_n_u16(r, 19595);
  yy = vmlal_n_u16(yy, g, 38470); yy = vmlal_n_u16(yy, b, 7471);
  yy = vaddq_u32(yy, vdupq_n_u32(32768));
  auto uu = vdupq_n_u32((128u << 16) + 32767u);
  uu = vmlal_n_u16(uu, b, 32768); uu = vmlsl_n_u16(uu, r, 11059); uu = vmlsl_n_u16(uu, g, 21709);
  auto vv = vdupq_n_u32((128u << 16) + 32767u);
  vv = vmlal_n_u16(vv, r, 32768); vv = vmlsl_n_u16(vv, g, 27439); vv = vmlsl_n_u16(vv, b, 5329);
  return {vshrn_n_u32(yy, 16), vshrn_n_u32(uu, 16), vshrn_n_u32(vv, 16)};
}
struct Eight {uint8x8_t y, u, v;};
inline Eight eight(uint8x8_t b8, uint8x8_t g8, uint8x8_t r8) {
  const auto b = vmovl_u8(b8), g = vmovl_u8(g8), r = vmovl_u8(r8);
  const auto lo = four(vget_low_u16(b), vget_low_u16(g), vget_low_u16(r));
  const auto hi = four(vget_high_u16(b), vget_high_u16(g), vget_high_u16(r));
  return {vmovn_u16(vcombine_u16(lo.y, hi.y)), vmovn_u16(vcombine_u16(lo.u, hi.u)),
    vmovn_u16(vcombine_u16(lo.v, hi.v))};
}
}  // namespace neon_detail

class NeonPackedI420Converter {
public:
  cv::Mat convert(const cv::Mat & bgr) {
    validate(bgr); cv::Mat result(bgr.rows * 3 / 2, bgr.cols, CV_8UC1);
    fill(bgr, result.data); return result;
  }
  void convert(const cv::Mat & bgr, Plane & result) {
    validate(bgr); result.resize(std::size_t(bgr.cols) * bgr.rows * 3 / 2); fill(bgr, result.data());
  }
private:
  static void validate(const cv::Mat & bgr) {
    checkSize(bgr.cols, bgr.rows);
    if (bgr.type() != CV_8UC3) throw std::invalid_argument("JPEG NEON I420 requires CV_8UC3 BGR");
  }
  void fill(const cv::Mat & bgr, unsigned char * output) {
    const auto y_size = std::size_t(bgr.cols) * bgr.rows;
    auto * y = output; auto * u = output + y_size; auto * v = u + y_size / 4;
    for (auto & row : rows_) row.resize(bgr.cols);
    alignas(16) static constexpr uint16_t alternating[8]{1, 2, 1, 2, 1, 2, 1, 2};
    const auto bias8 = vld1q_u16(alternating);
    for (int pair = 0; pair < bgr.rows / 2; ++pair) {
      for (int local = 0; local < 2; ++local) {
        const auto * src = bgr.ptr<unsigned char>(pair * 2 + local);
        auto * yy = y + (pair * 2 + local) * bgr.cols;
        auto * cb = rows_[local].data(); auto * cr = rows_[local + 2].data();
        int x = 0;
        // 一次解交织16个BGR像素；各分量先舍入为8位，再做h2v2平均。
        // 与标量相同的整数系数和舍入，不使用近似或有限范围转换。
        for (; x + 16 <= bgr.cols; x += 16) {
          const auto bgr16 = vld3q_u8(src + x * 3);
          const auto lo = neon_detail::eight(vget_low_u8(bgr16.val[0]), vget_low_u8(bgr16.val[1]), vget_low_u8(bgr16.val[2]));
          const auto hi = neon_detail::eight(vget_high_u8(bgr16.val[0]), vget_high_u8(bgr16.val[1]), vget_high_u8(bgr16.val[2]));
          vst1q_u8(yy + x, vcombine_u8(lo.y, hi.y));
          vst1q_u8(cb + x, vcombine_u8(lo.u, hi.u));
          vst1q_u8(cr + x, vcombine_u8(lo.v, hi.v));
        }
        for (; x < bgr.cols; ++x) {
          const int b = src[x * 3], g = src[x * 3 + 1], r = src[x * 3 + 2];
          yy[x] = static_cast<unsigned char>((19595 * r + 38470 * g + 7471 * b + 32768) >> 16);
          cb[x] = static_cast<unsigned char>((-11059 * r - 21709 * g + 32768 * b + (128 << 16) + 32767) >> 16);
          cr[x] = static_cast<unsigned char>((32768 * r - 27439 * g - 5329 * b + (128 << 16) + 32767) >> 16);
        }
      }
      auto * uu = u + pair * (bgr.cols / 2); auto * vv = v + pair * (bgr.cols / 2);
      int cx = 0;
      for (; cx + 8 <= bgr.cols / 2; cx += 8) {
        const auto cb0 = vpaddlq_u8(vld1q_u8(rows_[0].data() + cx * 2));
        const auto cb1 = vpaddlq_u8(vld1q_u8(rows_[1].data() + cx * 2));
        const auto cr0 = vpaddlq_u8(vld1q_u8(rows_[2].data() + cx * 2));
        const auto cr1 = vpaddlq_u8(vld1q_u8(rows_[3].data() + cx * 2));
        vst1_u8(uu + cx, vshrn_n_u16(vaddq_u16(vaddq_u16(cb0, cb1), bias8), 2));
        vst1_u8(vv + cx, vshrn_n_u16(vaddq_u16(vaddq_u16(cr0, cr1), bias8), 2));
      }
      for (; cx < bgr.cols / 2; ++cx) {
        const int x = cx * 2, bias = 1 + (cx & 1);
        uu[cx] = static_cast<unsigned char>((rows_[0][x] + rows_[0][x + 1] + rows_[1][x] + rows_[1][x + 1] + bias) >> 2);
        vv[cx] = static_cast<unsigned char>((rows_[2][x] + rows_[2][x + 1] + rows_[3][x] + rows_[3][x + 1] + bias) >> 2);
      }
    }
  }
  ChromaRows rows_;
};
}  // namespace vision_stream_detail

#endif

namespace vision_stream_detail {
#if defined(__aarch64__)
using JpegI420Converter = NeonPackedI420Converter;
#else
using JpegI420Converter = PackedI420Converter;
#endif
}

// JPEG 编码负责参数归一化、缩放、显式后端选择与单帧缓存。
// 缩放插值方式、JPEG 参数与缓存命中判据都在 encode()。

/// 非有限或非正的缩放没有意义：统一按 1.0（不缩放）处理，供所有编码入口共用。
static double normalizedScale(double scale)
{
  return (!std::isfinite(scale) || scale <= 0.0) ? 1.0 : scale;
}

/// 既有对外质量合同为1～100；在统一位置归一化，两个编码后端使用相同质量。
static int normalizedQuality(int quality)
{
  return std::max(1, std::min(100, quality));
}

class NvJpegEncoder {
public:
  NvJpegEncoder();
  ~NvJpegEncoder();
  NvJpegEncoder(const NvJpegEncoder &) = delete;
  NvJpegEncoder & operator=(const NvJpegEncoder &) = delete;
  static bool supportsSize(cv::Size size);
  std::vector<unsigned char> encode(const cv::Mat & bgr, int quality);
  void failNextEncodeForTest();
private:
  struct Runtime;
  std::unique_ptr<Runtime> runtime_;
};

JpegEncoder::JpegEncoder() = default;
JpegEncoder::~JpegEncoder() = default;

void JpegEncoder::setBackend(JpegBackend backend)
{
  if (backend == backend_) {return;}
  if (encodes_ != 0) {throw std::logic_error("JPEG backend cannot change after encoding");}
  if (backend != JpegBackend::OpenCv && backend != JpegBackend::NvJpeg) {
    throw std::invalid_argument("unknown JPEG backend");
  }
  // 先完成初始化再提交选择；异常后仍保留原后端状态。
  auto replacement = backend == JpegBackend::NvJpeg ? std::make_unique<NvJpegEncoder>() : nullptr;
  hardware_ = std::move(replacement);
  backend_ = backend;
}

JpegBackend JpegEncoder::backend() const {return backend_;}
JpegBackend JpegEncoder::lastEncodeBackend() const {return last_backend_;}
std::uint64_t JpegEncoder::hardwareEncodeCount() const {return hardware_encodes_;}
std::uint64_t JpegEncoder::sizeCompatibilityEncodeCount() const {return size_compatibility_encodes_;}

void JpegEncoder::failNextHardwareEncodeForTest()
{
  if (!hardware_) {throw std::logic_error("NV JPEG backend is not configured");}
  hardware_->failNextEncodeForTest();
}

std::shared_ptr<const std::vector<unsigned char>> JpegEncoder::encode(
  const cv::Mat & bgr, std::uint64_t frame_key, double scale, int quality)
{
  if (bgr.empty()) {return nullptr;}

  // 编码层的归一化供独立开发调用；生产入口在启动时已拒绝非法参数，不依赖此处兜底。
  const double wanted_scale = normalizedScale(scale);
  const int wanted_quality = normalizedQuality(quality);
  // 缓存比较归一化后的参数；非法值落到相同默认值时可以复用已有结果。
  if (valid_ && bytes_ && !bytes_->empty() && frame_key == frame_key_ &&
    wanted_scale == scale_ && wanted_quality == quality_)
  {
    return bytes_;
  }

  // 显式取整输出尺寸；合法的小比例至少保留一像素，接近 1 的比例也不能按容差忽略。
  const double width = bgr.cols * wanted_scale;
  const double height = bgr.rows * wanted_scale;
  if (!std::isfinite(width) || !std::isfinite(height) ||
    width > std::numeric_limits<int>::max() || height > std::numeric_limits<int>::max())
  {
    CV_Error(cv::Error::StsOutOfRange, "scaled JPEG dimensions exceed the integer range");
  }
  const cv::Size output_size(std::max(1, cvRound(width)), std::max(1, cvRound(height)));
  cv::Mat image;
  if (output_size != bgr.size()) {
    cv::resize(bgr, image, output_size, 0, 0, cv::INTER_AREA);
  } else {
    // cv::Mat 浅拷贝仅共享输入像素；后续编码只读源图，不在原图上缩放或画图。
    image = bgr;
  }

  std::vector<unsigned char> jpeg;
  const std::vector<int> params{cv::IMWRITE_JPEG_QUALITY, wanted_quality};
  const bool use_hardware = backend_ == JpegBackend::NvJpeg && NvJpegEncoder::supportsSize(output_size);
  bool success = true;
  if (use_hardware) {
    jpeg = hardware_->encode(image, wanted_quality);
  } else {
    // 仅确定性的尺寸不支持走原实现；硬件初始化/运行异常不会进入此分支。
    success = cv::imencode(".jpg", image, jpeg, params);
  }
  if (!success || jpeg.empty()) {
    // 编码失败不留半份缓存：下一次调用按新帧重新编码，而不是复用上一次的字节。
    reset();
    return nullptr;
  }

  // 缓存载体分配也可能失败；先取得新缓冲，再一起提交键和成功计数。
  // 否则bad_alloc会留下“新帧键＋旧字节”，下次命中会把上一帧错误当成新帧。
  auto encoded_bytes = std::make_shared<const std::vector<unsigned char>>(std::move(jpeg));
  ++encodes_;
  last_backend_ = use_hardware ? JpegBackend::NvJpeg : JpegBackend::OpenCv;
  if (use_hardware) {++hardware_encodes_;}
  else if (backend_ == JpegBackend::NvJpeg) {++size_compatibility_encodes_;}
  frame_key_ = frame_key;
  scale_ = wanted_scale;
  quality_ = wanted_quality;
  valid_ = true;
  // 缓存与多个输出端共享只读缓冲，换缓存时不影响仍在发送上一帧的消费者。
  bytes_ = std::move(encoded_bytes);
  return bytes_;
}

std::uint64_t JpegEncoder::encodeCount() const
{
  return encodes_;
}

std::size_t JpegEncoder::size() const
{
  return bytes_ ? bytes_->size() : 0;
}

void JpegEncoder::reset()
{
  bytes_.reset();
  valid_ = false;
}

#include <stdexcept>
#include <array>
#include <cstdint>
#include <limits>
#include <string>
#include <utility>

bool NvJpegEncoder::supportsSize(cv::Size size)
{
  // 插件caps上限为65535；本项目色转的受控整数/内存边界更窄，统一取32768。
  // 不支持的合法尺寸交给原OpenCV，保留1像素和奇数边长的开发功能，不裁图或补边输出。
  return size.width >= 16 && size.height >= 16 && size.width <= 32768 &&
         size.height <= 32768 && size.width % 2 == 0 && size.height % 2 == 0;
}

#if defined(RESCUE_HAS_NVJPEG) && RESCUE_HAS_NVJPEG
#include <dlfcn.h>
#include <gst/app/gstappsrc.h>
#include <gst/app/gstappsink.h>
#include <gst/video/video.h>
#include <algorithm>
#include <cstring>

namespace {
void check(bool condition, const std::string & message)
{
  if (!condition) {throw std::runtime_error("NV JPEG: " + message);}
}

// 注册后的GType仍可引用插件函数，句柄保持进程生命周期。两路编码器共享这一次初始化，
// 不将本模块已加载的插件误判成外来加载，也不向全局符号表替换系统libjpeg。
void initializePlugin()
{
  static void * const handle = []() -> void * {
      constexpr const char * path = "/usr/lib/aarch64-linux-gnu/gstreamer-1.0/libgstnvjpeg.so";
      if (void * existing = dlopen(path, RTLD_LAZY | RTLD_NOLOAD)) {
        dlclose(existing);
        throw std::runtime_error("NV JPEG plugin was loaded before JPEG symbol isolation");
      }
      dlerror();
      void * plugin = dlopen(path, RTLD_NOW | RTLD_LOCAL | RTLD_DEEPBIND);
      const char * loader_error = dlerror();
      check(plugin != nullptr, std::string("plugin load failed: ") + (loader_error ? loader_error : "unknown"));
      try {
        void * vendor = dlsym(plugin, "jpeg_CreateCompress");
        Dl_info info{};
        check(vendor && dladdr(vendor, &info) && info.dli_fname &&
          std::string(info.dli_fname).find("/nvidia/libnvjpeg.so") != std::string::npos &&
          vendor != dlsym(RTLD_DEFAULT, "jpeg_CreateCompress"), "JPEG symbol isolation failed");
        GError * error = nullptr;
        if (!gst_init_check(nullptr, nullptr, &error)) {
          const std::string description = error ? error->message : "unknown error";
          if (error) {g_error_free(error);}
          throw std::runtime_error("GStreamer initialization failed: " + description);
        }
      } catch (...) {
        dlclose(plugin);
        throw;
      }
      return plugin;
    }();
  (void)handle;
}

struct InputPixels {
  // 每帧独占 CPU 像素，由 GstBuffer 最后一个引用释放；不能借用转换器的跨帧工作区。
  cv::Mat bytes;
};
void releasePixels(gpointer data)
{
  delete static_cast<InputPixels *>(data);
}
}  // namespace

struct NvJpegEncoder::Runtime {
  static constexpr int nominal_fps = 100;
  GstElement * pipeline{nullptr};
  GstElement * source{nullptr};
  GstElement * sink{nullptr};
  GstBus * bus{nullptr};
  GstVideoInfo input_info{};
  cv::Size dimensions{};
  int quality{0};
  std::uint64_t sequence{0};
  bool inject_failure{false};
  vision_stream_detail::JpegI420Converter converter;

  Runtime() {initializePlugin();}
  ~Runtime() {close();}

  void close() noexcept
  {
    if (pipeline) {
      gst_element_set_state(pipeline, GST_STATE_NULL);
      gst_element_get_state(pipeline, nullptr, nullptr, 2 * GST_SECOND);
    }
    if (bus) {gst_object_unref(bus); bus = nullptr;}
    for (GstElement ** element : {&source, &sink, &pipeline}) {
      if (*element) {gst_object_unref(*element); *element = nullptr;}
    }
    dimensions = {};
    quality = 0;
  }

  void checkBus()
  {
    while (GstMessage * message = gst_bus_pop_filtered(
        bus, static_cast<GstMessageType>(GST_MESSAGE_ERROR | GST_MESSAGE_EOS)))
    {
      GError * error = nullptr;
      gchar * debug = nullptr;
      if (GST_MESSAGE_TYPE(message) == GST_MESSAGE_ERROR) {
        gst_message_parse_error(message, &error, &debug);
      }
      std::string description = error ? error->message : "unexpected end of stream";
      if (debug) {description += " [" + std::string(debug) + "]";}
      if (error) {g_error_free(error);}
      g_free(debug);
      gst_message_unref(message);
      throw std::runtime_error("NV JPEG: " + description);
    }
  }

  void prepare(cv::Size size, int requested_quality)
  {
    if (pipeline && dimensions == size && quality == requested_quality) {return;}
    close();
    try {
      const std::string description =
        "appsrc name=source is-live=false format=time block=true max-buffers=1 "
        "! nvjpegenc name=encoder quality=" + std::to_string(requested_quality) +
        " idct-method=0 ! appsink name=sink emit-signals=false sync=false async=false "
        "max-buffers=1 drop=false wait-on-eos=false";
      GError * error = nullptr;
      pipeline = gst_parse_launch(description.c_str(), &error);
      const std::string detail = error ? error->message : "";
      if (error) {g_error_free(error);}
      check(pipeline && detail.empty(), "pipeline creation failed: " + detail);
      source = gst_bin_get_by_name(GST_BIN(pipeline), "source");
      sink = gst_bin_get_by_name(GST_BIN(pipeline), "sink");
      bus = gst_element_get_bus(pipeline);
      check(source && sink && bus, "pipeline lacks source/sink/bus");
      GstCaps * caps = gst_caps_new_simple(
        "video/x-raw", "format", G_TYPE_STRING, "I420", "width", G_TYPE_INT, size.width,
        "height", G_TYPE_INT, size.height, "framerate", GST_TYPE_FRACTION, nominal_fps, 1,
        "colorimetry", G_TYPE_STRING, "1:4:7:1", "chroma-site", G_TYPE_STRING, "jpeg", nullptr);
      check(caps != nullptr, "caps allocation failed");
      const bool valid_layout = gst_video_info_from_caps(&input_info, caps);
      gst_app_src_set_caps(GST_APP_SRC(source), caps);
      gst_caps_unref(caps);
      check(valid_layout, "invalid I420 input layout");
      check(gst_element_set_state(pipeline, GST_STATE_PLAYING) != GST_STATE_CHANGE_FAILURE,
        "pipeline PLAYING failed");
      const auto state = gst_element_get_state(pipeline, nullptr, nullptr, 2 * GST_SECOND);
      checkBus();
      check(state != GST_STATE_CHANGE_FAILURE && state != GST_STATE_CHANGE_ASYNC,
        "pipeline PLAYING timed out");
      dimensions = size;
      quality = requested_quality;
    } catch (...) {
      close();
      throw;
    }
  }

  cv::Mat convert(const cv::Mat & bgr)
  {
    // 转换器会写满紧排 Y/U/V；独占 Mat 避免 vector 先清零整帧再被完整覆盖。
    cv::Mat packed = converter.convert(bgr);
    const auto y_size = static_cast<std::size_t>(bgr.cols) * bgr.rows;
    const std::array<gsize, 3> offsets{0, y_size, y_size + y_size / 4};
    const std::array<int, 3> widths{bgr.cols, bgr.cols / 2, bgr.cols / 2};
    const std::array<int, 3> heights{bgr.rows, bgr.rows / 2, bgr.rows / 2};
    bool tight = GST_VIDEO_INFO_SIZE(&input_info) == packed.total() * packed.elemSize();
    for (std::size_t plane = 0; plane < 3; ++plane) {
      tight = tight && GST_VIDEO_INFO_PLANE_OFFSET(&input_info, plane) == offsets[plane] &&
        GST_VIDEO_INFO_PLANE_STRIDE(&input_info, plane) == widths[plane];
    }
    if (tight) {return packed;}
    // 小偶数尺寸可能需要额外行对齐。按caps的真实平面布局填充，不依赖插件尊重自定义stride；
    // 1080p等紧排布局不走此复制分支。可见宽高始终不变。
    const auto aligned_size = GST_VIDEO_INFO_SIZE(&input_info);
    check(aligned_size > 0 && aligned_size <= static_cast<gsize>(std::numeric_limits<int>::max()),
      "aligned I420 input size exceeds the Mat range");
    // 对齐布局仍将所有间隙初始化为零；可见像素与每行 padding 沿用原来的复制规则。
    cv::Mat aligned(1, static_cast<int>(aligned_size), CV_8UC1, cv::Scalar(0));
    for (std::size_t plane = 0; plane < 3; ++plane) {
      for (int row = 0; row < heights[plane]; ++row) {
        auto * destination = aligned.data + GST_VIDEO_INFO_PLANE_OFFSET(&input_info, plane) +
          row * GST_VIDEO_INFO_PLANE_STRIDE(&input_info, plane);
        const auto * source_row = packed.data + offsets[plane] + row * widths[plane];
        std::memcpy(destination, source_row, widths[plane]);
        std::fill(destination + widths[plane],
          destination + GST_VIDEO_INFO_PLANE_STRIDE(&input_info, plane), source_row[widths[plane] - 1]);
      }
    }
    return aligned;
  }

  std::vector<unsigned char> encode(const cv::Mat & bgr, int requested_quality)
  {
    if (bgr.type() != CV_8UC3 || !NvJpegEncoder::supportsSize(bgr.size())) {
      throw std::invalid_argument("NV JPEG requires supported even-sized CV_8UC3 BGR input");
    }
    try {
      prepare(bgr.size(), requested_quality);
      checkBus();
      // 测试仅在真实管线已就绪后注入一次错误；不替换转换器、驱动或成功输出。
      if (inject_failure) {
        inject_failure = false;
        throw std::runtime_error("NV JPEG injected failure before push");
      }
      auto pixels = std::make_unique<InputPixels>();
      pixels->bytes = convert(bgr);
      const auto input_size = pixels->bytes.total() * pixels->bytes.elemSize();
      check(pixels->bytes.type() == CV_8UC1 && pixels->bytes.isContinuous() &&
        input_size > 0 && input_size == GST_VIDEO_INFO_SIZE(&input_info),
        "converted I420 input layout mismatch");
      GstBuffer * input = gst_buffer_new_wrapped_full(
        GST_MEMORY_FLAG_READONLY, pixels->bytes.data, input_size, 0,
        input_size, pixels.get(), releasePixels);
      check(input != nullptr, "input buffer allocation failed");
      pixels.release();  // 由GstBuffer持有，异步读取期间不会被后续帧覆盖。
      gsize offsets[GST_VIDEO_MAX_PLANES]{};
      gint strides[GST_VIDEO_MAX_PLANES]{};
      for (std::size_t plane = 0; plane < 3; ++plane) {
        offsets[plane] = GST_VIDEO_INFO_PLANE_OFFSET(&input_info, plane);
        strides[plane] = GST_VIDEO_INFO_PLANE_STRIDE(&input_info, plane);
      }
      if (!gst_buffer_add_video_meta_full(input, GST_VIDEO_FRAME_FLAG_NONE, GST_VIDEO_FORMAT_I420,
          bgr.cols, bgr.rows, 3, offsets, strides))
      {
        gst_buffer_unref(input);
        throw std::runtime_error("NV JPEG video metadata allocation failed");
      }
      const GstClockTime pts = gst_util_uint64_scale(++sequence, GST_SECOND, nominal_fps);
      GST_BUFFER_PTS(input) = pts;
      GST_BUFFER_DTS(input) = GST_CLOCK_TIME_NONE;
      GST_BUFFER_DURATION(input) = gst_util_uint64_scale(1, GST_SECOND, nominal_fps);
      const auto flow = gst_app_src_push_buffer(GST_APP_SRC(source), input);
      check(flow == GST_FLOW_OK, std::string("push failed: ") + gst_flow_get_name(flow));
      GstSample * sample = gst_app_sink_try_pull_sample(GST_APP_SINK(sink), 2 * GST_SECOND);
      if (!sample) {checkBus(); throw std::runtime_error("NV JPEG output timed out");}
      const std::unique_ptr<GstSample, void (*)(GstSample *)> owned_sample(
        sample, [](GstSample * value) {gst_sample_unref(value);});
      GstBuffer * output = gst_sample_get_buffer(sample);
      check(output && GST_BUFFER_PTS(output) == pts, "output PTS differs from input");
      const gsize length = gst_buffer_get_size(output);
      std::vector<unsigned char> bytes(length);
      check(gst_buffer_extract(output, 0, bytes.data(), length) == length, "JPEG extraction failed");
      check(bytes.size() >= 4 && bytes[0] == 0xff && bytes[1] == 0xd8 &&
        bytes[bytes.size() - 2] == 0xff && bytes.back() == 0xd9, "incomplete JPEG output");
      checkBus();
      return bytes;
    } catch (...) {
      // 损坏管线不继续复用；后续调用仍是同一硬件后端，不能静默返回旧JPEG或转软件。
      close();
      throw;
    }
  }
};

NvJpegEncoder::NvJpegEncoder() : runtime_(std::make_unique<Runtime>()) {}
NvJpegEncoder::~NvJpegEncoder() = default;
std::vector<unsigned char> NvJpegEncoder::encode(const cv::Mat & bgr, int quality)
{
  return runtime_->encode(bgr, quality);
}
void NvJpegEncoder::failNextEncodeForTest() {runtime_->inject_failure = true;}

#else
struct NvJpegEncoder::Runtime {};
NvJpegEncoder::NvJpegEncoder()
{
  throw std::runtime_error("NV JPEG backend is unavailable in this build");
}
NvJpegEncoder::~NvJpegEncoder() = default;
std::vector<unsigned char> NvJpegEncoder::encode(const cv::Mat &, int)
{
  throw std::runtime_error("NV JPEG backend is unavailable in this build");
}
void NvJpegEncoder::failNextEncodeForTest()
{
  throw std::logic_error("NV JPEG backend is unavailable in this build");
}
#endif
