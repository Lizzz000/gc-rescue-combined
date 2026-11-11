// Copyright 2026 jetson_yolo integration
// SPDX-License-Identifier: Apache-2.0
// 相机实现：按 backend 打开设备（V4L2/GStreamer/仿真 file）、按 fourcc -> 尺寸 -> fps -> 缓冲协商并探测首帧。
// 全范围 I420 转 BGR 在 camera/neon/i420.cpp。相机状态只由采集线程顺序使用。
#include "camera/camera.hpp"

#include <cstring>

#include <algorithm>
#include <cstdio>
#include <sstream>
#include <thread>

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include "camera/neon/i420.hpp"

namespace {
// 自动管线的组帧/丢帧顺序与 read() 的本地 I420 转 BGR 必须同时认这两个 fourcc。
bool isMjpegFourcc(const std::string & fourcc)
{
  return fourcc == "MJPG" || fourcc == "JPEG";
}
}  // namespace

bool UsbCamera::acceptDelivered(const cv::Mat & image)
{
  // 检查实际帧而非 CAP_PROP 报值，保护下游归一化坐标所依赖的固定宽高合同。
  if (checkDeliveredFrameSize(image.cols, image.rows, config_.width, config_.height) !=
    FrameSizeCheck::Mismatch)
  {
    return true;
  }
  std::ostringstream message;
  message << "camera delivered " << image.cols << "x" << image.rows
          << " but camera_width/camera_height is " << config_.width << "x" << config_.height
          << "; refusing this frame so normalized coordinates stay in the configured frame";
  last_error_ = message.str();
  return false;
}

bool UsbCamera::open()
{
  try {
    close();
    last_error_.clear();
    reconnect_required_ = false;
    if (cancelRequested()) {last_error_ = "camera capture cancelled"; return false;}

    // 后端在本次会话内固定；打开失败由上层采集循环重试。
    if (config_.backend == "file") {
      if (!openFile()) {
        close();
        return false;
      }
    } else if (config_.backend == "gstreamer") {
      // 管线负责取流和 appsink；自动 MJPEG 管线输出 I420，read() 再在本地完成 BGR 转换。
      const std::string pipeline =
        config_.pipeline.empty() ? vision_camera::gstreamerPipeline(config_) : config_.pipeline;
      if (!gstreamer_.open(pipeline, cancel_requested_, !config_.pipeline.empty())) {
        last_error_ = gstreamer_.lastError();
        close();
        return false;
      }
    } else if (!camera_.open(config_.device, cv::CAP_V4L2)) {
      // 后端由配置固定；设备忙或不可用时不再隐式进入 GStreamer/文件解码。
      last_error_ = "cannot open " + config_.device + " with V4L2 backend";
      close();
      return false;
    }

    if (config_.backend == "v4l2") {
      negotiateV4l2();
    }

    // 探测帧走完整 read() 尺寸检查但不交付给节点；它已消耗一帧，下一次 read() 才是正常出帧。
    CameraFrame probe;
    if (!read(probe) || probe.image.empty()) {
      if (last_error_.empty()) {
        last_error_ = "camera opened but delivered no frame";
      }
      close();
      return false;
    }
    return true;
  } catch (const cv::Exception & error) {
    // 解码、协商或探测异常按打开失败交回采集重试，不能让异常逃出工作线程终止进程。
    last_error_ = std::string("camera open failed: ") + error.what();
    try {
      close();
    } catch (const cv::Exception & close_error) {
      // 保留原始错误；释放后端本身失败也不能绕过采集线程的恢复边界。
      last_error_ += std::string("; camera close failed: ") + close_error.what();
    }
    return false;
  }
}

std::string vision_camera::gstreamerPipeline(const CameraConfig & config)
{
  // 在解码和尺寸/颜色转换前各保留一个最新缓冲：慢消费者只跳帧，不积压旧画面。
  // MJPEG 由 NVDEC 解码。compute-hw=2 把这块板的默认 VIC 写死，只把 NVMM 导出为系统 I420。
  // 全范围矩阵仍在 read() 里完成：同板 VIC 与 GPU 的 BGRx 都对不上约定的整数结果。
  // 未压缩格式由同一 VIC 转到 BGRx，videoconvert 只去掉填充通道。
  // MJPEG先组为完整边界再丢帧，避免队列先丢掉同一JPEG的部分载荷；不要照搬到H.264/H.265。
  const bool mjpeg = isMjpegFourcc(config.fourcc);
  std::ostringstream pipeline;
  pipeline << "v4l2src device=" << config.device << " io-mode=2";
  if (config.brightness >= 0) {
    // 由相机驱动设置亮度；采集重连时自动管线会再次应用。
    pipeline << " extra-controls=\"c,brightness=" << config.brightness << "\"";
  }
  if (mjpeg) {
    // jpegparse只按SOI/EOI恢复边界，不证明熵数据或RST完整；已缺失的数据不会被补回。
    pipeline << " ! image/jpeg,width=" << config.width << ",height=" << config.height
             << ",framerate=" << config.fps << "/1"
             << " ! jpegparse"
             << " ! queue max-size-buffers=1 max-size-bytes=0 max-size-time=0 leaky=downstream"
             << " ! nvv4l2decoder mjpeg=1";
  } else {
    pipeline << " ! video/x-raw,width=" << config.width << ",height=" << config.height
             << ",framerate=" << config.fps << "/1,format="
             << (config.fourcc == "YUYV" ? "YUY2" : config.fourcc);
  }
  pipeline << " ! queue max-size-buffers=1 max-size-bytes=0 max-size-time=0 leaky=downstream"
           << " ! nvvidconv compute-hw=2";
  if (mjpeg) {
    // 保留 JPEG 全范围 YUV；read() 精确映射范围并转 BGR，避免 VIC 范围误解。
#if VISION_ENABLE_NVMM
    // H13：留在 NVMM，推理前处理由 CUDA 直接读同一缓冲；CPU 侧在 appsink 映射后拷出 I420。
    pipeline << " ! video/x-raw(memory:NVMM),format=I420";
#else
    pipeline << " ! video/x-raw,format=I420";
#endif
  } else {
#if VISION_ENABLE_NVMM
    // H13：同一 VIC 转到 BGRx 但留在 NVMM；CPU 侧映射后去掉填充通道（等价于原 videoconvert），
    // 推理由 CUDA 直接读同一缓冲，颜色结果与原路径同为 VIC 的转换。
    pipeline << " ! video/x-raw(memory:NVMM),format=BGRx";
#else
    pipeline << " ! video/x-raw,format=BGRx"
             << " ! videoconvert n-threads=4 ! video/x-raw,format=BGR";
#endif
  }
  pipeline << " ! appsink drop=1 max-buffers=1 sync=false";
  return pipeline.str();
}

void UsbCamera::negotiateV4l2()
{
  // 先请求配置的像素格式，再请求尺寸与帧率；压缩或未压缩格式均由 fourcc 显式决定。
  // set() 只是向驱动请求，可能被忽略；最终宽高由 open()/read() 的实际帧验收，不能据此认定协商成功。
  if (config_.fourcc.size() == 4) {
    camera_.set(cv::CAP_PROP_FOURCC,
                cv::VideoWriter::fourcc(config_.fourcc[0], config_.fourcc[1],
                                        config_.fourcc[2], config_.fourcc[3]));
  }
  if (config_.width > 0) {camera_.set(cv::CAP_PROP_FRAME_WIDTH, config_.width);}
  if (config_.height > 0) {camera_.set(cv::CAP_PROP_FRAME_HEIGHT, config_.height);}
  if (config_.fps > 0) {camera_.set(cv::CAP_PROP_FPS, config_.fps);}
  // 请求较浅驱动队列，减少采集端积压旧帧；是否支持由驱动决定。
  if (config_.buffer_size > 0) {
    camera_.set(cv::CAP_PROP_BUFFERSIZE, config_.buffer_size);
  }
}

bool UsbCamera::openFile()
{
  // 先当图片读：成功就进入单张循环模式；否则当视频文件交给 OpenCV（FFmpeg 等后端）。
  still_ = cv::imread(config_.device, cv::IMREAD_COLOR);
  if (still_.empty() && !camera_.open(config_.device, cv::CAP_ANY)) {
    return false;
  }
  // 单张图片的内容与输出尺寸在本次打开期间固定，只缩放一次；每次 readFile() 仍克隆，
  // 让消费者可以修改自己的帧。视频保留逐帧缩放，重新 open() 会重新读取原文件。
  if (!still_.empty() && config_.width > 0 && config_.height > 0 &&
    (still_.cols != config_.width || still_.rows != config_.height))
  {
    cv::resize(still_, still_, cv::Size(config_.width, config_.height));
  }
  next_frame_ = std::chrono::steady_clock::now();
  return true;
}

// 文件节拍只模拟输入速率；不能据此验证真实设备的带宽、曝光或驱动缓冲延迟。
bool UsbCamera::readFile(CameraFrame & out)
{
  cv::Mat frame;
  if (!still_.empty()) {
    frame = still_.clone();
  } else if (!camera_.read(frame) || frame.empty()) {
    // 视频播完：回到开头继续循环；仍读不到就按读失败处理（由调用方重开）。
    camera_.set(cv::CAP_PROP_POS_FRAMES, 0);
    if (!camera_.read(frame) || frame.empty()) {
      out.image.release();
      out.at = {};
      return false;
    }
  }
  if (frame.channels() == 1) {
    cv::cvtColor(frame, frame, cv::COLOR_GRAY2BGR);
  }
  if (config_.width > 0 && config_.height > 0 &&
    (frame.cols != config_.width || frame.rows != config_.height))
  {
    cv::resize(frame, frame, cv::Size(config_.width, config_.height));
  }
  // 按 fps 节拍出帧，模拟相机的帧间隔；fps<=0 时不限速。
  if (config_.fps > 0) {
    while (!cancelRequested() && std::chrono::steady_clock::now() < next_frame_) {
      std::this_thread::sleep_until(std::min(next_frame_,
        std::chrono::steady_clock::now() + std::chrono::milliseconds(100)));
    }
    if (cancelRequested()) {
      out.image.release();
      out.at = {};
      last_error_ = "camera capture cancelled";
      return false;
    }
    const auto period = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
      std::chrono::duration<double>(1.0 / config_.fps));
    // 延误后不连发补齐旧节拍，下一截止时间至少推进到现在，避免视频仿真形成追赶突发。
    next_frame_ = std::max(next_frame_ + period, std::chrono::steady_clock::now());
  }
  out.image = frame.isContinuous() ? std::move(frame) : frame.clone();
  out.at = std::chrono::steady_clock::now();
  if (!acceptDelivered(out.image)) {
    out.image.release();
    out.at = {};
    return false;
  }
  last_error_.clear();
  return true;
}

bool UsbCamera::read(CameraFrame & out)
{
  try {
    out.image.release();
    out.nvmm.reset();
    out.at = {};
    if (cancelRequested()) {last_error_ = "camera capture cancelled"; return false;}
    if (config_.backend == "file" && isOpen()) {return readFile(out);}

    cv::Mat frame;
    std::chrono::steady_clock::time_point grabbed_at;
    if (config_.backend == "gstreamer") {
      if (!gstreamer_.read(frame, grabbed_at, cancel_requested_, out.nvmm)) {
        last_error_ = gstreamer_.lastError();
        reconnect_required_ = !cancelRequested();
        return false;
      }
    } else {
      if (!camera_.isOpened() || !camera_.grab()) {return false;}
      // V4L2 的 JPEG 解码在 retrieve()；时间仍取后端交付时刻，不是曝光时刻。
      grabbed_at = std::chrono::steady_clock::now();
      if (!camera_.retrieve(frame) || frame.empty()) {return false;}
    }
    if (config_.backend == "gstreamer" && config_.pipeline.empty() &&
      isMjpegFourcc(config_.fourcc))
    {
      frame = vision_camera::fullRangeI420ToBgr(frame);
    }

    if (cancelRequested()) {last_error_ = "camera capture cancelled"; return false;}

    // 只有当后端交回的缓冲区不归这一帧独占时才 clone（1920x1080x3 = 6.2 MB）：
    //   * u == nullptr：Mat 只是包着后端/驱动内存的头，下一次 grab 会被覆盖；
    //   * refcount > 1：后端自己还持有同一块内存，下一次 retrieve 可能写进去。
    // 否则这块内存是 retrieve 新分配、只被本地 frame 引用的，直接移交，省掉一次整帧拷贝。
    const bool exclusive = frame.u != nullptr && frame.u->refcount == 1;
    out.image = exclusive ? std::move(frame) : frame.clone();
    out.at = grabbed_at;
    if (!acceptDelivered(out.image)) {
      out.image.release();
      out.nvmm.reset();
      out.at = {};
      return false;
    }
    last_error_.clear();
    return true;
  } catch (const cv::Exception & error) {
    // 解码或缩放失败的帧不能沿用上次像素和时间；连续失败仍由采集循环按原门槛重开。
    out.image.release();
    out.nvmm.reset();
    out.at = {};
    last_error_ = std::string("camera read failed: ") + error.what();
    return false;
  }
}

void UsbCamera::close()
{
  // 只释放取流资源，保留 last_error_；open() 清它，成功 read() 也清它，便于重连日志说明上次失败。
  gstreamer_.close();
  if (camera_.isOpened()) {
    camera_.release();
  }
  still_.release();
}

// 供启动日志描述后端属性：帧率/格式可能只是驱动报值，缓冲数打印的是请求值（不是查询值），实际速率由节点统计。
std::string UsbCamera::description() const
{
  char buffer[192];
  if (config_.backend == "file") {
    const cv::Size s = size();
    std::snprintf(buffer, sizeof(buffer), "%s (simulated file camera, %s) %dx%d @ %d fps",
                  config_.device.c_str(), still_.empty() ? "video loop" : "still image",
                  s.width, s.height, config_.fps);
    return std::string(buffer);
  }
  // GStreamer 尺寸来自已验证 sample 的 caps；帧率和格式仍明确是请求值。
  if (config_.backend == "gstreamer") {
    std::snprintf(buffer, sizeof(buffer), "%s (gstreamer/NVDEC) %dx%d @ %d fps, format %s",
                  config_.device.c_str(),
                  gstreamer_.size().width, gstreamer_.size().height, config_.fps,
                  config_.fourcc.c_str());
    return std::string(buffer);
  }
  const int fourcc = static_cast<int>(camera_.get(cv::CAP_PROP_FOURCC));
  char code[5] = {0};
  for (int i = 0; i < 4; ++i) {
    const char c = static_cast<char>((fourcc >> (8 * i)) & 0xFF);
    code[i] = (c >= 32 && c <= 126) ? c : '?';
  }
  std::snprintf(buffer, sizeof(buffer),
                "%s %dx%d @ %.1f fps, fourcc %s, buffer %d",
                config_.device.c_str(),
                static_cast<int>(camera_.get(cv::CAP_PROP_FRAME_WIDTH)),
                static_cast<int>(camera_.get(cv::CAP_PROP_FRAME_HEIGHT)),
                camera_.get(cv::CAP_PROP_FPS), code, config_.buffer_size);
  return std::string(buffer);
}

cv::Size UsbCamera::size() const
{
  if (config_.backend == "gstreamer") {return gstreamer_.size();}
  if (config_.backend == "file") {
    if (config_.width > 0 && config_.height > 0) {return {config_.width, config_.height};}
    if (!still_.empty()) {return still_.size();}
  }
  return cv::Size(static_cast<int>(camera_.get(cv::CAP_PROP_FRAME_WIDTH)),
                  static_cast<int>(camera_.get(cv::CAP_PROP_FRAME_HEIGHT)));
}
