#include "camera/gstreamer/gstreamer.hpp"

#include <algorithm>
#include <cstring>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include <gst/app/gstappsink.h>
#include <gst/video/video.h>
#if VISION_ENABLE_NVMM
#include <nvbufsurface.h>
#endif

namespace vision_camera {
namespace {
using Clock = std::chrono::steady_clock;
constexpr auto kWaitBudget = std::chrono::seconds(2);
constexpr auto kPollPeriod = std::chrono::milliseconds(100);

// 不运行 GLib main loop：同步丢弃无消费者的通知，只保留第一个终止消息。
// callback 在发消息的插件线程执行，不能在这里等待采集线程或修改其诊断字符串。
struct TerminalMessage {
  std::atomic<GstMessage *> message{nullptr};
  ~TerminalMessage()
  {
    if (auto * retained = message.load(std::memory_order_relaxed)) {gst_message_unref(retained);}
  }
};

GstBusSyncReply captureBusMessage(GstBus *, GstMessage * message, gpointer data)
{
  auto & terminal = *static_cast<TerminalMessage *>(data);
  if (GST_MESSAGE_TYPE(message) == GST_MESSAGE_ERROR || GST_MESSAGE_TYPE(message) == GST_MESSAGE_EOS) {
    // 单个强引用使 ERROR/EOS 风暴也保持常量内存；首个终止事件确定本次会话的结局。
    GstMessage * retained = gst_message_ref(message);
    GstMessage * expected = nullptr;
    if (!terminal.message.compare_exchange_strong(expected, retained,
        std::memory_order_release, std::memory_order_relaxed))
    {
      gst_message_unref(retained);
    }
  }
  // GstBusSyncHandler 的 DROP 合同要求 handler 释放原始消息引用。
  gst_message_unref(message);
  return GST_BUS_DROP;
}

// GstVideoFrame 可能含行/平面对齐填充，不能直接按总字节数解释成紧密 Mat。
// 每张交付图独立拥有像素，释放 sample 或下一帧到来都不会修改它。
#if VISION_ENABLE_NVMM
// H13：NVMM 帧（MJPEG 为 I420，raw 为 VIC 转好的 BGRx）。CPU 侧（field/叠层/推流）仍需一份：
// 从同一 NvBufSurface 映射后拷出 I420，或去掉 x 通道得到 BGR（等价于原 videoconvert）；
// 推理侧保留 sample 引用，由 CUDA 经 EGL 直接读同一块 NVMM。
bool copyNvmm(GstSample * sample, GstBuffer * buffer, cv::Mat & image, cv::Size & size,
  std::string & error, std::shared_ptr<void> & nvmm)
{
  GstMapInfo map;
  if (!gst_buffer_map(buffer, &map, GST_MAP_READ)) {error = "cannot map NVMM buffer"; return false;}
  auto * surface = reinterpret_cast<NvBufSurface *>(map.data);
  gst_buffer_unmap(buffer, &map);
  if (NvBufSurfaceMap(surface, 0, -1, NVBUF_MAP_READ) != 0) {error = "NvBufSurfaceMap failed"; return false;}
  NvBufSurfaceSyncForCpu(surface, 0, -1);
  const auto & params = surface->surfaceList[0];
  const int width = static_cast<int>(params.width);
  const int height = static_cast<int>(params.height);
  if (params.colorFormat == NVBUF_COLOR_FORMAT_BGRx) {
    const cv::Mat bgrx(height, width, CV_8UC4, params.mappedAddr.addr[0], params.planeParams.pitch[0]);
    cv::cvtColor(bgrx, image, cv::COLOR_BGRA2BGR);
  } else {
    image.create(height + height / 2, width, CV_8UC1);
    const int rows[3] = {height, height / 2, height / 2};
    const int bytes[3] = {width, width / 2, width / 2};
    unsigned char * destination = image.data;
    for (int plane = 0; plane < 3; ++plane) {
      const auto * source = static_cast<const unsigned char *>(params.mappedAddr.addr[plane]);
      const auto stride = params.planeParams.pitch[plane];
      for (int row = 0; row < rows[plane]; ++row) {
        std::memcpy(destination, source + static_cast<std::size_t>(row) * stride, bytes[plane]);
        destination += bytes[plane];
      }
    }
  }
  NvBufSurfaceUnMap(surface, 0, -1);
  gst_sample_ref(sample);
  nvmm = std::shared_ptr<void>(surface, [sample](void *) {gst_sample_unref(sample);});
  size = {width, height};
  return true;
}
#endif

bool copyPixels(GstSample * sample, cv::Mat & image, cv::Size & size, std::string & error, bool require_bgr,
  std::shared_ptr<void> & nvmm)
{
  nvmm.reset();
  GstVideoInfo info;
  gst_video_info_init(&info);
  GstCaps * caps = gst_sample_get_caps(sample);
  GstBuffer * buffer = gst_sample_get_buffer(sample);
  if (!caps || !buffer || !gst_video_info_from_caps(&info, caps)) {
    error = "appsink sample has no supported raw video caps/buffer";
    return false;
  }
#if VISION_ENABLE_NVMM
  GstCapsFeatures * features = gst_caps_get_features(caps, 0);
  if (features && gst_caps_features_contains(features, "memory:NVMM")) {
    return copyNvmm(sample, buffer, image, size, error, nvmm);
  }
#endif
  if (require_bgr && GST_VIDEO_INFO_FORMAT(&info) != GST_VIDEO_FORMAT_BGR) {
    error = "custom camera pipeline must deliver BGR at appsink; negotiated " +
      std::string(gst_video_format_to_string(GST_VIDEO_INFO_FORMAT(&info)));
    return false;
  }
  GstVideoFrame mapped{};
  if (!gst_video_frame_map(&mapped, &info, buffer, GST_MAP_READ)) {
    error = "cannot map appsink video frame";
    return false;
  }
  struct Unmap {GstVideoFrame * frame; ~Unmap() {gst_video_frame_unmap(frame);}} unmap{&mapped};
  const int width = GST_VIDEO_INFO_WIDTH(&info);
  const int height = GST_VIDEO_INFO_HEIGHT(&info);
  if (width <= 0 || height <= 0) {
    error = "appsink delivered non-positive video dimensions";
    return false;
  }
  auto copyPlane = [&](int plane, unsigned char * destination, int rows, std::size_t bytes) {
      const auto * source = static_cast<const unsigned char *>(GST_VIDEO_FRAME_PLANE_DATA(&mapped, plane));
      const int stride = GST_VIDEO_FRAME_PLANE_STRIDE(&mapped, plane);
      if (!source || stride < 0 || static_cast<std::size_t>(stride) < bytes) {return false;}
      for (int row = 0; row < rows; ++row) {
        std::memcpy(destination + static_cast<std::size_t>(row) * bytes,
          source + static_cast<std::size_t>(row) * stride, bytes);
      }
      return true;
    };
  const auto format = GST_VIDEO_INFO_FORMAT(&info);
  bool copied = false;
  if (format == GST_VIDEO_FORMAT_I420 || format == GST_VIDEO_FORMAT_YV12) {
    if (width % 2 || height % 2 || height > G_MAXINT / 3 * 2) {
      error = "planar 4:2:0 capture requires even, bounded dimensions";
      return false;
    }
    image.create(height + height / 2, width, CV_8UC1);
    const auto y_bytes = static_cast<std::size_t>(width) * height;
    copied = copyPlane(0, image.data, height, width) &&
      copyPlane(1, image.data + y_bytes, height / 2, width / 2) &&
      copyPlane(2, image.data + y_bytes + y_bytes / 4, height / 2, width / 2);
  } else if (format == GST_VIDEO_FORMAT_NV12 || format == GST_VIDEO_FORMAT_NV21) {
    if (width % 2 || height % 2 || height > G_MAXINT / 3 * 2) {
      error = "semiplanar 4:2:0 capture requires even, bounded dimensions";
      return false;
    }
    image.create(height + height / 2, width, CV_8UC1);
    copied = copyPlane(0, image.data, height, width) &&
      copyPlane(1, image.data + static_cast<std::size_t>(width) * height, height / 2, width);
  } else {
    int type = -1;
    switch (format) {
      case GST_VIDEO_FORMAT_BGR:
      case GST_VIDEO_FORMAT_RGB: type = CV_8UC3; break;
      case GST_VIDEO_FORMAT_BGRx:
      case GST_VIDEO_FORMAT_RGBx:
      case GST_VIDEO_FORMAT_BGRA:
      case GST_VIDEO_FORMAT_RGBA: type = CV_8UC4; break;
      case GST_VIDEO_FORMAT_GRAY8: type = CV_8UC1; break;
      case GST_VIDEO_FORMAT_YUY2:
      case GST_VIDEO_FORMAT_UYVY: type = CV_8UC2; break;
      default: break;
    }
    if (type < 0) {
      error = std::string("unsupported appsink raw format: ") + gst_video_format_to_string(format);
      return false;
    }
    image.create(height, width, type);
    copied = copyPlane(0, image.data, height, static_cast<std::size_t>(width) * image.elemSize());
  }
  if (!copied) {error = "appsink plane stride is smaller than its image row"; return false;}
  size = {width, height};
  return true;
}
}  // namespace

struct GstreamerCapture::Impl {
  GstElement * pipeline{nullptr};
  GstAppSink * sink{nullptr};
  GstBus * bus{nullptr};
  TerminalMessage * terminal{nullptr};  // 归 bus handler 的 destroy-notify 所有。
  cv::Mat primed;
  std::shared_ptr<void> primed_nvmm;
  Clock::time_point primed_at{};
  cv::Size delivered_size;
  std::string error;
  bool received_sample{false};
  bool require_bgr{false};

  bool terminalError()
  {
    // 引用直到解绑 handler 才释放；open/read/close 由同一采集线程顺序执行。
    GstMessage * message = terminal ? terminal->message.load(std::memory_order_acquire) : nullptr;
    if (!message) {return false;}
    if (GST_MESSAGE_TYPE(message) == GST_MESSAGE_ERROR) {
      GError * detail = nullptr;
      gchar * debug = nullptr;
      gst_message_parse_error(message, &detail, &debug);
      error = std::string("GStreamer capture failed: ") + (detail ? detail->message : "unknown error");
      if (detail) {g_error_free(detail);}
      g_free(debug);
    } else {
      error = "GStreamer input reached EOS";
    }
    return true;
  }

  bool poll(cv::Mat & image, Clock::time_point & at, const std::atomic<bool> & cancelled,
    Clock::time_point deadline, std::shared_ptr<void> & nvmm)
  {
    image.release();
    nvmm.reset();
    at = {};
    while (!cancelled.load(std::memory_order_relaxed)) {
      if (terminalError()) {return false;}
      const auto now = Clock::now();
      if (now >= deadline) {error = "GStreamer delivered no frame within 2000 ms"; return false;}
      const auto wait = std::min(std::chrono::duration_cast<std::chrono::nanoseconds>(deadline - now),
        std::chrono::duration_cast<std::chrono::nanoseconds>(kPollPeriod));
      GstSample * sample = gst_app_sink_try_pull_sample(sink, static_cast<GstClockTime>(wait.count()));
      const auto received_at = Clock::now();
      struct Release {GstSample * sample; ~Release() {if (sample) {gst_sample_unref(sample);}}} release{sample};
      if (terminalError()) {return false;}
      if (sample) {
        if (cancelled.load(std::memory_order_relaxed)) {break;}
        // 源时间来自 appsink 交付时刻，刻意不采用插件可能重写的 PTS。
        if (!copyPixels(sample, image, delivered_size, error, require_bgr, nvmm)) {
          image.release();
          nvmm.reset();
          return false;
        }
        if (terminalError()) {image.release(); nvmm.reset(); return false;}
        if (cancelled.load(std::memory_order_relaxed)) {image.release(); nvmm.reset(); break;}
        at = received_at;
        received_sample = true;
        error.clear();
        return true;
      }
      // 尚未启动的 appsink 也会报告 is_eos；首帧前只采用总预算和真实 bus EOS。
      if (received_sample && gst_app_sink_is_eos(sink)) {
        error = "GStreamer input reached EOS";
        return false;
      }
    }
    error = "GStreamer capture cancelled";
    return false;
  }

  bool start(GstElement * owned, const std::atomic<bool> & cancelled, Clock::time_point deadline)
  {
    pipeline = owned;
    if (!pipeline || !GST_IS_BIN(pipeline)) {
      error = "GStreamer capture requires a pipeline containing one appsink";
      return false;
    }
    if (cancelled.load(std::memory_order_relaxed)) {error = "GStreamer capture cancelled"; return false;}
    GstIterator * iterator = gst_bin_iterate_recurse(GST_BIN(pipeline));
    GValue item = G_VALUE_INIT;
    bool done = false;
    unsigned count = 0;
    while (!done) {
      switch (gst_iterator_next(iterator, &item)) {
        case GST_ITERATOR_OK: {
          auto * element = GST_ELEMENT(g_value_get_object(&item));
          if (GST_IS_APP_SINK(element)) {
            ++count;
            if (!sink) {sink = GST_APP_SINK(gst_object_ref(element));}
          }
          g_value_reset(&item);
          break;
        }
        case GST_ITERATOR_RESYNC:
          if (sink) {gst_object_unref(sink); sink = nullptr;}
          count = 0;
          gst_iterator_resync(iterator);
          break;
        default: done = true; break;
      }
    }
    if (G_VALUE_TYPE(&item)) {g_value_unset(&item);}
    gst_iterator_free(iterator);
    if (count != 1) {error = "GStreamer capture requires exactly one appsink"; return false;}
    bus = gst_element_get_bus(pipeline);
    if (!bus) {error = "GStreamer capture pipeline has no bus"; return false;}
    // 新会话丢弃启动前的通知；PLAYING 前安装 handler，后续消息不进入异步队列。
    gst_bus_set_flushing(bus, true);
    terminal = new TerminalMessage;
    gst_bus_set_sync_handler(bus, captureBusMessage, terminal,
      [](gpointer data) {delete static_cast<TerminalMessage *>(data);});
    gst_bus_set_flushing(bus, false);
    gst_app_sink_set_emit_signals(sink, false);
    gst_app_sink_set_drop(sink, true);
    gst_app_sink_set_max_buffers(sink, 1);
    gst_app_sink_set_wait_on_eos(sink, false);
    // 不使用无限 gst_element_get_state；PLAYING 的异步协商与首帧共享同一预算。
    if (gst_element_set_state(pipeline, GST_STATE_PLAYING) == GST_STATE_CHANGE_FAILURE) {
      error = "GStreamer capture cannot enter PLAYING";
      return false;
    }
    return poll(primed, primed_at, cancelled, deadline, primed_nvmm);
  }
};

GstreamerCapture::GstreamerCapture() : impl_(std::make_unique<Impl>()) {}
GstreamerCapture::~GstreamerCapture() {close();}

bool GstreamerCapture::open(const std::string & description, const std::atomic<bool> & cancelled,
  bool require_bgr)
{
  const auto deadline = Clock::now() + kWaitBudget;
  close();
  impl_->error.clear();
  if (cancelled.load(std::memory_order_relaxed)) {impl_->error = "GStreamer capture cancelled"; return false;}
  impl_->require_bgr = require_bgr;
  GError * error = nullptr;
  if (!gst_init_check(nullptr, nullptr, &error)) {
    impl_->error = std::string("GStreamer init failed: ") + (error ? error->message : "unknown error");
    if (error) {g_error_free(error);}
    return false;
  }
  GstElement * pipeline = gst_parse_launch(description.c_str(), &error);
  if (error) {
    impl_->error = std::string("GStreamer pipeline parse failed: ") + error->message;
    g_error_free(error);
    if (pipeline) {gst_object_unref(pipeline);}
    return false;
  }
  if (!impl_->start(pipeline, cancelled, deadline)) {close(); return false;}
  return true;
}

bool GstreamerCapture::open(GstElement * pipeline, const std::atomic<bool> & cancelled)
{
  const auto deadline = Clock::now() + kWaitBudget;
  close();
  impl_->error.clear();
  if (!impl_->start(pipeline, cancelled, deadline)) {close(); return false;}
  return true;
}

bool GstreamerCapture::read(cv::Mat & image, Clock::time_point & at,
  const std::atomic<bool> & cancelled, std::shared_ptr<void> & nvmm)
{
  image.release();
  nvmm.reset();
  at = {};
  if (cancelled.load(std::memory_order_relaxed)) {impl_->error = "GStreamer capture cancelled"; return false;}
  if (!isOpen()) {impl_->error = "GStreamer capture is not open"; return false;}
  if (impl_->terminalError()) {
    impl_->primed.release();
    impl_->primed_nvmm.reset();
    impl_->primed_at = {};
    return false;
  }
  if (!impl_->primed.empty()) {
    image = std::move(impl_->primed);
    nvmm = std::move(impl_->primed_nvmm);
    at = impl_->primed_at;
    impl_->primed_at = {};
    return true;
  }
  return impl_->poll(image, at, cancelled, Clock::now() + kWaitBudget, nvmm);
}

void GstreamerCapture::close()
{
  // 采集线程可在打开失败或重连时自行释放；所有者须先 join，取消线程只置标志，不跨线程释放资源。
  impl_->primed_nvmm.reset();
  if (impl_->pipeline) {gst_element_set_state(impl_->pipeline, GST_STATE_NULL);}
  if (impl_->sink) {gst_object_unref(impl_->sink); impl_->sink = nullptr;}
  if (impl_->bus) {
    // GStreamer 1.16.3+ 的解绑是线程安全的；notify 在 callback 不再使用状态后释放。
    gst_bus_set_sync_handler(impl_->bus, nullptr, nullptr, nullptr);
    impl_->terminal = nullptr;
    gst_object_unref(impl_->bus);
    impl_->bus = nullptr;
  }
  if (impl_->pipeline) {gst_object_unref(impl_->pipeline); impl_->pipeline = nullptr;}
  impl_->primed.release();
  impl_->primed_at = {};
  impl_->delivered_size = {};
  impl_->received_sample = false;
  impl_->require_bgr = false;
}
bool GstreamerCapture::isOpen() const {return impl_->pipeline && impl_->sink;}
cv::Size GstreamerCapture::size() const {return impl_->delivered_size;}
const std::string & GstreamerCapture::lastError() const {return impl_->error;}
}  // namespace vision_camera
