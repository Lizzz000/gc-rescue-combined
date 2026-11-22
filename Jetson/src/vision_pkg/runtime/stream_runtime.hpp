#pragma once

// Copyright 2026 jetson_yolo integration
// SPDX-License-Identifier: Apache-2.0

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <sensor_msgs/msg/compressed_image.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>

#include "interface_pkg/msg/field_observation.hpp"
#include "node/diagnostics.hpp"
#include "node/image_view.hpp"
#include "node/stream_sync.hpp"
#include "node/worker_group.hpp"
#include "stream/jpeg/jpeg.hpp"

class MjpegServer;

struct StreamRuntimeConfig
{
  bool draw_boxes{true};
  bool draw_labels{true};
  bool draw_masks{false};
  bool draw_field{true};
  double overlay_wait_ms{150.0};
  double input_timeout_s{1.0};
  bool stream_enabled{true};
  std::string stream_address{"0.0.0.0"};
  std::int64_t stream_port{8080};
  JpegBackend jpeg_backend{JpegBackend::OpenCv};
  std::int64_t stream_quality{80};
  double stream_max_fps{0.0};
  double stream_scale{1.0};
  bool publish_compressed{false};
  std::int64_t compressed_quality{80};
  bool overlay_stats{true};
  std::string frame_id;
};

// 有界帧队列、同帧图层与 HTTP 会话共用代次。调用方在释放输出端口前停止运行。
class StreamRuntime
{
public:
  using Clock = std::chrono::steady_clock;
  struct Demand
  {
    bool annotated;
    bool compressed;
  };
  struct Ports
  {
    std::function<std::int64_t()> source_now;
    std::function<Demand()> demand;
    std::function<void(
      std::unique_ptr<sensor_msgs::msg::Image>,
      std::unique_ptr<sensor_msgs::msg::CompressedImage>)> publish;
    std::function<void(bool, const std::string &)> http_started;
    std::function<void(const char *)> compose_error;
  };
  struct Status
  {
    double image_fps;
    double compose_fps;
    double compose_ms;
    double stream_fps;
    std::size_t viewers;
    std::size_t instances;
    std::uint64_t replaced;
  };

  StreamRuntime(StreamRuntimeConfig config, Ports ports);
  ~StreamRuntime();

  void start();
  void stop();
  void rethrowIfFailed() const;
  std::exception_ptr takeHttpFailure();
  Status status();
  double inputTimeout() const {return config_.input_timeout_s;}

  void setTask(const std::string & task, std::uint64_t epoch, std::int64_t cutoff);
  void acceptFrame(
    vision_ros::ImageViewConstPtr view, std::int64_t stamp, std::int64_t current,
    Clock::time_point received_at);
  void acceptDetections(
    const OverlayFrameKey & key, std::int64_t current,
    vision_msgs::msg::Detection2DArray::ConstSharedPtr msg);
  void acceptMask(
    const OverlayFrameKey & key, std::int64_t current, vision_ros::ImageViewConstPtr view);
  void acceptField(
    const OverlayFrameKey & key, std::int64_t current,
    interface_pkg::msg::FieldObservation::ConstSharedPtr msg);

private:
  struct Frame
  {
    // 视图持有源消息，图像可跨线程共享且不复制。
    vision_ros::ImageViewConstPtr view;
    std::int64_t stamp;
    Clock::time_point received_at;
    Clock::time_point valid_until;
    std::uint64_t generation{0};
  };
  struct Overlay
  {
    vision_msgs::msg::Detection2DArray::ConstSharedPtr seg;
    interface_pkg::msg::FieldObservation::ConstSharedPtr field;
    vision_ros::ImageViewConstPtr mask;
  };
  enum class OverlayWait { Ready, Skip, Stopped };
  // 准备在锁外完成，发布前还须复核代次和新鲜度。
  struct PreparedOutput
  {
    cv::Mat canvas;
    std::unique_ptr<sensor_msgs::msg::Image> annotated;
    std::unique_ptr<sensor_msgs::msg::CompressedImage> compressed;
    std::shared_ptr<const std::vector<unsigned char>> jpeg;
    bool share_jpeg{false};
    Clock::time_point encode_started;
    double encode_ms{-1.0};
  };

  void startServer();
  void wakeWaiters();
  void composeLoop();
  bool freshFrame(const Frame & frame) const;
  static bool matchesFrame(const Frame & frame, const std_msgs::msg::Header & header);
  OverlayWait waitForOverlay(const Frame & frame, Overlay & snapshot);
  cv::Mat buildCanvas(const Frame & frame, const Overlay & overlay, bool & drew_anything);
  std::string statsText(bool drew_anything) const;
  PreparedOutput prepareOutput(const Frame & frame, const Overlay & snapshot, Demand demand);
  bool publishCurrentFrame(const Frame & frame, PreparedOutput & output);

  // 只在锁内替换共享载荷，绘制和编码使用取出的只读快照。
  template<class Update>
  void insertOverlay(
    const OverlayFrameKey & key, OverlayLayer layer, std::int64_t current, Update update)
  {
    std::lock_guard<std::mutex> lock(overlay_mutex_);
    if (synchronizer_.insert(key, layer, current, config_.input_timeout_s, update)) {
      overlay_cv_.notify_all();
    }
  }

  const StreamRuntimeConfig config_;
  const Ports ports_;
  std::unique_ptr<MjpegServer> streamer_;
  JpegEncoder jpeg_encoder_;
  // 编码器与帧键仅由合成线程访问。
  std::uint64_t encode_frame_key_{0};
  PendingFrames<Frame> frame_slot_;

  // 赋代次与入槽、清缓存和 ROS 最终发布共享此锁。
  std::mutex overlay_mutex_;
  OverlaySynchronizer<Overlay> synchronizer_;
  std::condition_variable overlay_cv_;
  std::atomic<std::size_t> composed_instances_{0};

  RateMeter compose_meter_;
  std::atomic<std::uint64_t> image_arrivals_{0};
  WindowCounter image_window_;
  WindowCounter compose_window_;
  WorkerGroup workers_;
};
