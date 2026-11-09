// ---- 板端采集实现 ----
// 职责：GStreamer 采集管线与帧环。
#include "gstreamer/camera_stream.hpp"
#include <gst/app/gstappsink.h>
#include <algorithm>
#include <stdexcept>

namespace record {
// 管线所有权与生命周期：元素交给 pipeline_，appsink 另持一个查询所得引用。
CameraStream::CameraStream(CameraOptions options, const std::string& test_pipeline)
    : options_(std::move(options)) {
  gchar* id = g_uuid_string_random();
  source_id_ = id; g_free(id);
  source_id_.erase(std::remove(source_id_.begin(), source_id_.end(), '-'), source_id_.end());
  try {
    if (!test_pipeline.empty()) {
      GError* error = nullptr;
      pipeline_ = gst_parse_launch(test_pipeline.c_str(), &error);
      if (error) { const std::string text = error->message; g_error_free(error); throw std::runtime_error(text); }
    } else {
      pipeline_ = gst_pipeline_new("record-camera-stream");
      auto add = [&](const char* factory, const char* name) {
        auto* e = gst_element_factory_make(factory, name);
        if (!e) throw std::runtime_error(std::string("missing GStreamer element: ") + factory);
        gst_bin_add(GST_BIN(pipeline_), e);
        return e;
      };
      auto* source = add("v4l2src", "camera");
      auto* filter = add("capsfilter", "format");
      GError* error = nullptr;
      auto* processing = gst_parse_bin_from_description(camera_jpeg_processing, TRUE, &error);
      if (error || !processing) {
        const std::string detail = error ? error->message : "cannot create hardware JPEG pipeline";
        g_clear_error(&error);
        if (processing) gst_object_unref(processing);
        throw std::runtime_error(detail);
      }
      gst_bin_add(GST_BIN(pipeline_), processing);
      auto* sink = add("appsink", "frames");
      g_object_set(source, "device", options_.device.c_str(), "do-timestamp", TRUE, nullptr);
      GstCaps* caps = gst_caps_new_simple("video/x-raw", "format", G_TYPE_STRING, "YUY2",
          "width", G_TYPE_INT, options_.width,
          "height", G_TYPE_INT, options_.height, "framerate", GST_TYPE_FRACTION, options_.fps, 1, nullptr);
      g_object_set(filter, "caps", caps, nullptr); gst_caps_unref(caps);
      if (!gst_element_link_many(source, filter, processing, sink, nullptr)) throw std::runtime_error("cannot link camera stream");
    }
    if (!pipeline_) throw std::runtime_error("cannot create camera pipeline");
    sink_ = gst_bin_get_by_name(GST_BIN(pipeline_), "frames");
    if (!sink_) throw std::runtime_error("camera pipeline has no frames appsink");
    g_object_set(sink_, "emit-signals", TRUE, "sync", FALSE, "max-buffers", 2u, "drop", FALSE, nullptr);
    g_signal_connect(sink_, "new-sample", G_CALLBACK(sample), this);
  } catch (...) {
    if (sink_) gst_object_unref(sink_);
    if (pipeline_) gst_object_unref(pipeline_);
    throw;
  }
}
CameraStream::~CameraStream() {
  stop();
  if (sink_) gst_object_unref(sink_);
  if (pipeline_) gst_object_unref(pipeline_);
}
void CameraStream::start() {
  started_ns_ = monotonic_ns();
  if (gst_element_set_state(pipeline_, GST_STATE_PLAYING) == GST_STATE_CHANGE_FAILURE) {
    const auto detail = poll_error();
    throw std::runtime_error(detail.empty() ? "camera cannot start; check device ownership, YUYV format and Jetson hardware JPEG plugins" : detail);
  }
}
void CameraStream::stop() {
  { std::lock_guard<std::mutex> lock(mutex_); stopped_ = true; }
  changed_.notify_all();
  // 停止标志与回调发布共用锁；释放锁并唤醒读者后再停媒体线程，避免死锁；无需文件 EOS。
  if (pipeline_) gst_element_set_state(pipeline_, GST_STATE_NULL);
}
// 采集回调：锁外复制 JPEG，锁内检查停止边界并发布；异常不能跨越 GStreamer 回调。
int CameraStream::sample(GstElement* sink, void* data) noexcept {
  auto& self = *static_cast<CameraStream*>(data);
  GstSample* sample = gst_app_sink_pull_sample(GST_APP_SINK(sink));
  if (!sample) return GST_FLOW_EOS;
  GstMapInfo map{};
  GstBuffer* buffer = gst_sample_get_buffer(sample);
  bool mapped = false;
  try {
    auto f = std::make_shared<Frame>();
    // 时间取自本对象的板端回调，不是传感器曝光时间，也不与 PC 时钟相减。
    f->capture_ns = monotonic_ns(); f->source_id = self.source_id_;
    // caps 已在建管线时固定为参数值，不再逐帧比对协商结果。
    f->width = self.options_.width; f->height = self.options_.height; f->fps = self.options_.fps;
    mapped = buffer && gst_buffer_map(buffer, &map, GST_MAP_READ);
    if (!mapped || map.size < 4 || map.size > max_jpeg_bytes) throw std::runtime_error("camera JPEG size is invalid");
    // Frame 自有 JPEG 副本；退出回调即可解除映射并归还 GstSample，不依赖网络读者寿命。
    f->jpeg.assign(reinterpret_cast<const char*>(map.data), map.size);
    { std::lock_guard<std::mutex> lock(self.mutex_);
      if (!self.stopped_) {
        f->sequence = ++self.sequence_;
        self.frames_.push_back(std::move(f));
        if (self.frames_.size() > 8) self.frames_.pop_front();
      }
    }
    self.changed_.notify_all();
  } catch (const std::exception& e) {
    std::lock_guard<std::mutex> lock(self.mutex_);
    if (self.error_.empty()) self.error_ = e.what();
  } catch (...) {
    std::lock_guard<std::mutex> lock(self.mutex_);
    if (self.error_.empty()) self.error_ = "camera frame callback failed";
  }
  if (mapped) gst_buffer_unmap(buffer, &map);
  gst_sample_unref(sample);
  return GST_FLOW_OK;
}
// 读帧与健康检查：返回共享只读引用，调用者离开锁后仍可安全使用帧。
FramePtr CameraStream::latest() {
  std::lock_guard<std::mutex> lock(mutex_);
  return frames_.empty() ? nullptr : frames_.back();
}
FramePtr CameraStream::next(std::uint64_t& cursor) {
  std::unique_lock<std::mutex> lock(mutex_);
  changed_.wait_for(lock, std::chrono::seconds(2), [&] { return stopped_ || !error_.empty() || (!frames_.empty() && frames_.back()->sequence > cursor); });
  if (stopped_ || !error_.empty() || frames_.empty()) return {};
  // 新连接取最新帧；旧连接所需的下一帧已被8帧环淘汰时断开，不静默跳帧。
  if (!cursor) { cursor = frames_.back()->sequence; return frames_.back(); }
  if (cursor + 1 < frames_.front()->sequence) return {};
  for (const auto& f : frames_) if (f->sequence > cursor) { cursor = f->sequence; return f; }
  return {};
}
std::string CameraStream::poll_error() {
  GstBus* bus = gst_element_get_bus(pipeline_);
  GstMessage* msg = gst_bus_pop_filtered(bus, GstMessageType(GST_MESSAGE_ERROR | GST_MESSAGE_EOS));
  std::string error;
  if (msg) {
    if (GST_MESSAGE_TYPE(msg) == GST_MESSAGE_ERROR) {
      GError* e = nullptr; gchar* debug = nullptr;
      gst_message_parse_error(msg, &e, &debug);
      error = e ? e->message : "camera error"; g_clear_error(&e); g_free(debug);
    } else error = "camera stream ended unexpectedly";
    gst_message_unref(msg);
  }
  gst_object_unref(bus);
  std::lock_guard<std::mutex> lock(mutex_);
  if (error_.empty() && !error.empty()) error_ = error;
  // 首帧容许10秒启动等待，出帧后5秒无新帧判失败；首个媒体/回调错误保持不变。
  const auto last = frames_.empty() ? started_ns_ : frames_.back()->capture_ns;
  if (!stopped_ && started_ns_ && error_.empty() && monotonic_ns() - last > (frames_.empty() ? 10ULL : 5ULL) * 1000000000ULL)
    error_ = "camera produced no fresh JPEG frames";
  return error_;
}
}  // namespace record
