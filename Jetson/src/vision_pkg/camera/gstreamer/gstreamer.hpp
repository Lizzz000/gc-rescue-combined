#pragma once
#include <atomic>
#include <chrono>
#include <memory>
#include <sstream>
#include <string>
#include <utility>

#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>

struct _GstElement;

namespace vision_camera {
class GstreamerCapture {
public:
  GstreamerCapture();
  ~GstreamerCapture();
  GstreamerCapture(const GstreamerCapture &) = delete;
  GstreamerCapture & operator=(const GstreamerCapture &) = delete;

  bool open(const std::string & pipeline, const std::atomic<bool> & cancelled,
    bool require_bgr = false);
  bool open(_GstElement * owned_pipeline, const std::atomic<bool> & cancelled);
  /// nvmm：appsink 协商为 memory:NVMM 时持有本帧 NvBufSurface（get() 即 NvBufSurface*，
  /// 释放即归还 GstSample）；否则为空。image 照常是 CPU 拷贝。
  bool read(cv::Mat & image, std::chrono::steady_clock::time_point & at,
    const std::atomic<bool> & cancelled, std::shared_ptr<void> & nvmm);
  void close();
  bool isOpen() const;
  cv::Size size() const;
  const std::string & lastError() const;

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
}
