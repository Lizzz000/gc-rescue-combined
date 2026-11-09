// ---- 板端采集声明 ----
// 职责：相机出帧并按协议发送，不写 MP4。
#pragma once
#include "stream_protocol.hpp"
#include <condition_variable>
#include <deque>
#include <gst/gst.h>
#include <mutex>

namespace record {
struct CameraOptions {
  std::string device = "/dev/video0", bind = "127.0.0.1";
  int width = 1920, height = 1080, fps = 30, port = 8091, idle_seconds = 30;
};
// YUYV（GStreamer 名称 YUY2）→ VIC RGBA → CPU 全范围 I420 → nvjpegenc 质量95。
// CPU videoconvert 展开范围，避免当前 VIC 直接输出 I420 的有限范围数值使 JPEG 发灰。
// 合成色阶合同复用同一处理片段，检验范围和颜色。
inline constexpr char camera_jpeg_processing[] =
    "nvvidconv compute-hw=2 ! video/x-raw,format=RGBA ! videoconvert ! "
    "video/x-raw,format=I420,colorimetry=1:4:5:1 ! nvjpegenc quality=95";

// 独占一条相机管线；GStreamer 回调复制 JPEG 到8帧环，HTTP 持有只读帧引用发送。
// 生命周期所有者串行调用 start/stop/poll_error；回调与读帧通过 mutex_ 交接。
class CameraStream {
 public:
  explicit CameraStream(CameraOptions options, const std::string& test_pipeline = {});
  ~CameraStream();
  void start();
  void stop();
  std::string poll_error();
  // cursor 属于单个连接，0 表示从最新帧加入；空返回要求该连接结束传输。
  FramePtr next(std::uint64_t& cursor);
  FramePtr latest();
 private:
  static int sample(GstElement*, void*) noexcept;
  CameraOptions options_;
  std::string source_id_;
  // pipeline_ 拥有管线元素；sink_ 是额外引用，析构时先停管线再分别释放。
  GstElement* pipeline_ = nullptr;
  GstElement* sink_ = nullptr;
  // 保护帧环、首个错误、停止标志及序号；不在锁内切换管线状态或发送网络数据。
  std::mutex mutex_;
  std::condition_variable changed_;
  std::deque<FramePtr> frames_;
  std::string error_;
  bool stopped_ = false;
  std::uint64_t sequence_ = 0, started_ns_ = 0;
};
}  // namespace record
