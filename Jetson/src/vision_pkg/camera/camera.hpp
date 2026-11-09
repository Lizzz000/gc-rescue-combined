#pragma once
// ---- 相机 ----
// 职责：采集设备、帧尺寸和交付给检测的图像。
// Copyright 2026 jetson_yolo integration
// SPDX-License-Identifier: Apache-2.0
// UVC 相机取流封装（V4L2 或 GStreamer/NVDEC）：按配置协商格式与尺寸、探测首帧，read() 返回独占的帧。
// 自动管线、V4L2 与 file 交付 BGR；自定义 pipeline 必须输出 BGR，逐帧核验 caps 后才允许交付。
//       另有 file 后端：把图片或视频当作相机循环播放，可在无摄像头的板端验证处理链路。
// 协商顺序为像素格式→尺寸→帧率，避免驱动在未压缩格式下因带宽不足退回低分辨率。
// read() 每次取到新的 Mat；仅外部内存或后端仍引用的像素需要克隆，防止下一帧覆盖已交付图像。


#include <atomic>
#include <chrono>
#include <memory>
#include <sstream>
#include <string>
#include <utility>

#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>

#include "camera/gstreamer/gstreamer.hpp"


// 启动时固定的输入配置；这些成员初值用于直接调用封装，生产值由统一配置覆盖。
// 宽高单位为像素，fps 为帧/秒；尺寸请求必须由实际交付帧验收，驱动属性报值不能代替验收。
struct CameraConfig
{
  std::string device{"/dev/video0"};
  int width{1920};
  int height{1080};
  int fps{60};
  /// V4L2 四字符像素格式；MJPG 用于压缩传输，实际支持能力由相机决定。
  std::string fourcc{"YUYV"};
  /// 请求的驱动队列深度；驱动可能忽略该属性，较浅队列用于减少旧帧积压。
  int buffer_size{2};

  /// 后端可选 v4l2（默认）、gstreamer 或 file；v4l2 打开失败不切换到其他后端。
  ///
  /// file（仿真用）：device 填一张图片或一个视频文件的路径，按 fps 节拍循环输出，
  /// width/height 都 > 0 时缩放到该尺寸，模拟相机；静态图只在打开时缩放，视频逐帧缩放。它不验证真实相机协商。
  ///
  /// gstreamer 自动管线在请求 MJPG/JPEG 时使用 NVDEC 解码；原始格式走对应的原始图像管线。
  /// MJPEG 在本地转为 BGR，其他自动管线输出 BGR；自定义管线必须在 appsink 前协商为 BGR。
  /// 自动MJPEG先按JPEG边界组帧，再由解码和颜色转换前的单项队列保留最新画面；组帧不校验熵数据。
  /// appsink仍只缓存一项，自定义管线不会被额外插入组帧步骤。
  /// 是否支持及其实际帧率、CPU 开销需在目标设备验证。
  std::string backend{"v4l2"};

  /// 完整自定义 GStreamer 管线；为空时按相机配置生成默认管线。
  std::string pipeline;
};

/// 配置的宽和高都为正时，交付帧必须与之一致。任一请求尺寸 <= 0 表示不检查，调用方自行对齐下游 image_width。
enum class FrameSizeCheck { Match, Unspecified, Mismatch };

inline FrameSizeCheck checkDeliveredFrameSize(
  int actual_width, int actual_height, int requested_width, int requested_height)
{
  if (requested_width <= 0 || requested_height <= 0) {return FrameSizeCheck::Unspecified;}
  if (actual_width == requested_width && actual_height == requested_height) {
    return FrameSizeCheck::Match;
  }
  return FrameSizeCheck::Mismatch;
}

struct CameraFrame
{
  cv::Mat image;  ///< BGR，本帧独占（必要时 read() 内部已 clone），可以安全长期持有
  /// V4L2 grab()/GStreamer appsink 返回的取帧时刻；V4L2尚待retrieve解码，GStreamer已在管线解码。
  /// 自动MJPEG管线此后还需本地I420转BGR，其他管线按其输出格式交付。
  /// detect_node 用它回推 ROS 时间戳，不读取jpegparse可能重写的GST PTS；它不是传感器曝光时刻。
  /// file 后端没有曝光/驱动队列，此处记录节拍等待结束后的交付时刻；所有后端均用 steady_clock。
  std::chrono::steady_clock::time_point at{};
  /// H13：GStreamer 自动管线（TensorRT 构建）时为同一帧的 NvBufSurface（MJPEG→I420，raw→BGRx），
  /// 推理只从这里取输入；file/V4L2/自定义管线为空，分割推理无法进行。持有期间占用一个 NVMM 缓冲。
  std::shared_ptr<void> nvmm;
};

// 相机状态不带锁；采集线程负责 open/read 与重连关闭，所有者仅在 join 后关闭或销毁。
class UsbCamera {
public:
  explicit UsbCamera(CameraConfig config)
  : config_(std::move(config)) {}

  UsbCamera(const UsbCamera &) = delete;
  UsbCamera & operator=(const UsbCamera &) = delete;

  ~UsbCamera() {close();}

  /// 打开并协商，再读取一张探测帧；仅设备能按尺寸契约实际出帧时返回成功。
  bool open();

  /// 失败返回 false；调用方仅在成功时消费 out。
  /// V4L2/file 连续失败门槛、重连计数和 ROS 时间映射属于节点；GStreamer 预算耗尽会要求立即重连。
  bool read(CameraFrame & out);

  void close();

  bool isOpen() const {return gstreamer_.isOpen() || camera_.isOpened() || !still_.empty();}

  /// 唯一允许跨线程的操作：仅置取消标志，不释放后端。停止在此对象生命周期内不可撤销。
  void requestStop() noexcept {cancel_requested_.store(true, std::memory_order_relaxed);}
  bool cancelRequested() const noexcept {return cancel_requested_.load(std::memory_order_relaxed);}
  /// 原生 GStreamer 已耗尽无帧预算或收到 EOS/error，调用方应立即开始新相机会话。
  bool reconnectRequired() const {return reconnect_required_;}

  /// 描述后端可报告的属性；GStreamer 分支也包含请求的帧率/格式，不保证它们是实测值。
  /// 正尺寸请求若与交付图像不符，open/read 会拒绝该帧。
  std::string description() const;

  /// 已捕获的失败诊断，供重连日志使用；常规 false 返回可能没有文本，close 不清除。
  const std::string & lastError() const {return last_error_;}

  cv::Size size() const;
  const CameraConfig & config() const {return config_;}

private:
  /// 仅 V4L2 后端使用：依次请求格式、尺寸、帧率和驱动队列深度。
  void negotiateV4l2();

  /// file 后端：打开图片（循环同一张）或视频（播完从头再来）。
  bool openFile();
  /// file 后端：取下一帧、缩放到配置尺寸并按 fps 节拍等待。
  bool readFile(CameraFrame & out);

  /// 校验实际帧的正尺寸请求；未指定尺寸时调用方须自行对齐下游归一化宽高。
  bool acceptDelivered(const cv::Mat & image);

  CameraConfig config_;
  cv::VideoCapture camera_;
  vision_camera::GstreamerCapture gstreamer_;
  std::atomic<bool> cancel_requested_{false};
  bool reconnect_required_{false};
  std::string last_error_;
  // file 后端状态：still_ 非空表示单张图片模式；next_frame_ 是下一帧的节拍时刻。
  cv::Mat still_;
  std::chrono::steady_clock::time_point next_frame_{};
};

namespace vision_camera {
/// 自动管线的纯值装配入口，不打开设备；UsbCamera仅在未指定自定义pipeline时调用。
/// MJPG/JPEG先组帧再允许队列丢帧，保持其他格式及像素转换路径不变。
std::string gstreamerPipeline(const CameraConfig & config);

}
