#pragma once
#include "node/detect_node.hpp"
#include "stream/jpeg/jpeg.hpp"
#include "stream/mjpeg/mjpeg.hpp"

// Copyright 2026 jetson_yolo integration
// SPDX-License-Identifier: Apache-2.0
// 图传节点：订阅配置的源图（生产默认处理帧）与 seg／field 结果，在独立线程按时间戳和 frame_id 叠加，并内置 MJPEG。
// 不参与目标选择和 CAN。同帧队列在 node/stream_sync.hpp，合成、编码和服务会话在 stream_node.cpp。

#include <chrono>
#include <cstdint>
#include <memory>
#include <string>

#include <opencv2/core.hpp>

// 兼容 cv_bridge 在 Humble/Jazzy 中不同的头文件命名。
#if __has_include("cv_bridge/cv_bridge.hpp")
#include <cv_bridge/cv_bridge.hpp>
#else
#include <cv_bridge/cv_bridge.h>
#endif
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>
#include "interface_pkg/msg/stream_status.hpp"
#include "interface_pkg/msg/task_state.hpp"
#include <vision_msgs/msg/detection2_d_array.hpp>

#include "interface_pkg/msg/field_observation.hpp"

#include <atomic>
#include <condition_variable>
#include <mutex>

#include "node/stream_sync.hpp"

// ROS 输入只做校验与移交；运行会话拥有合成线程、同步缓存和 HTTP 服务。
class StreamNode : public rclcpp::Node
{
public:
  explicit StreamNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~StreamNode() override;

private:
  // 窄测试访问器直接调用真实准备／最终发布步骤，不在生产运行期加入替代回调。
  friend struct StreamRuntimeTestAccess;
  struct Frame
  {
    cv::Mat image;
    rclcpp::Time stamp;
    std::string frame_id;
    // 保留源消息，供合成线程安全共享图像。
    cv_bridge::CvImageConstPtr holder;
    // stamp 是源 ROS 时间；到达和过期时刻用单调时钟，排队及图层等待都消耗同一有效期。
    std::chrono::steady_clock::time_point received_at;
    std::chrono::steady_clock::time_point valid_until;
    std::uint64_t generation{0};
  };

  // 图层按源时间戳和光学坐标系严格配对；共享载荷只读。
  struct Overlay
  {
    vision_msgs::msg::Detection2DArray::ConstSharedPtr seg;
    interface_pkg::msg::FieldObservation::ConstSharedPtr field;
    // 缓存淘汰后仍保留掩膜像素，供已取得快照的合成线程绘制。
    cv_bridge::CvImageConstPtr mask;
  };

  // ---- 启动配置与接口装配 ----
  void declareParameters();
  void loadParameters();
  // 参数校验完成后建立接口，最后启动运行会话。
  void createRosInterfaces();

  // 阶段过滤与帧匹配：同步策略在本头的图传节，ROS 时间校验保留在回调。
  void activeTaskCallback(const interface_pkg::msg::TaskState::SharedPtr msg);
  bool matchesFrame(const Frame & frame, const std_msgs::msg::Header & header) const;
  bool freshFrame(const Frame & frame) const;

  // ---- 源图和结果仅移交 Runtime，不在 ROS 回调绘制或编码 ----
  void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr & msg);
  void segDetectionCallback(const vision_msgs::msg::Detection2DArray::SharedPtr msg);
  void segMaskCallback(const sensor_msgs::msg::Image::ConstSharedPtr & msg);
  void fieldCallback(const interface_pkg::msg::FieldObservation::SharedPtr msg);

  void publishStatus();
  // ROS 状态适配；画布绘制由运行会话负责。
  std::string statsText(bool drew_anything) const;

  // 以下参数均为启动只读配置；运行中修改会由 ROS 参数描述拒绝。
  std::string image_topic_;
  std::string seg_detections_topic_;
  std::string seg_masks_topic_;
  std::string field_results_topic_;
  std::string active_task_topic_;

  bool draw_boxes_{true};
  bool draw_labels_{true};
  bool draw_masks_{false};
  bool draw_field_{true};
  // 图层等待单位毫秒、输入新鲜度单位秒；成员初值用于构造，生产实际值必须由统一配置注入。
  double overlay_wait_ms_{150.0};
  double input_timeout_s_{1.0};

  bool stream_enabled_{true};
  std::string stream_address_{"0.0.0.0"};
  std::int64_t stream_port_{8080};
  std::string jpeg_backend_;  // 只读选择；ROS 压缩与 HTTP 使用同一编码后端。
  std::int64_t stream_quality_{80};
  double stream_max_fps_{0.0};
  double stream_scale_{1.0};

  bool publish_annotated_{false};
  bool publish_compressed_{false};
  std::int64_t compressed_quality_{80};
  bool overlay_stats_{true};
  bool publish_status_{true};
  std::string frame_id_{""};

  // 订阅回调与状态定时器属于默认互斥组，后台合成线程经 Runtime 的锁与之交换数据。
  rclcpp::Subscription<interface_pkg::msg::TaskState>::SharedPtr active_task_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Subscription<vision_msgs::msg::Detection2DArray>::SharedPtr seg_det_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr seg_mask_sub_;
  rclcpp::Subscription<interface_pkg::msg::FieldObservation>::SharedPtr field_sub_;

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr annotated_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr compressed_pub_;
  rclcpp::Publisher<interface_pkg::msg::StreamStatus>::SharedPtr status_pub_;
  rclcpp::TimerBase::SharedPtr status_timer_;

  struct Runtime;
  std::unique_ptr<Runtime> runtime_;
};

struct StreamNode::Runtime
{
  explicit Runtime(StreamNode & owner)
  : owner_(owner) {}
  ~Runtime() {stop();}
  void start();
  void stop();
  void wakeWaiters();
  void startServer();
  void reportHttpFailure();
  // 就绪包含“等待预算用尽但取得部分图层”；跳过处理旧代次，停止则退出循环，三者不能混同。
  enum class OverlayWait { Ready, Skip, Stopped };
  // 仅持有锁外准备好的画布和消息；准备成功尚未承诺发布，最终准入仍可能拒绝整帧。
  struct PreparedOutput
  {
    cv::Mat canvas;
    std::unique_ptr<sensor_msgs::msg::Image> annotated;
    std::unique_ptr<sensor_msgs::msg::CompressedImage> compressed;
    std::shared_ptr<const std::vector<unsigned char>> jpeg;
    bool have_compressed{false};
    bool share_jpeg{false};
    std::chrono::steady_clock::time_point encode_started;
    double encode_ms{-1.0};
  };
  OverlayWait waitForOverlay(const Frame &, Overlay &);
  PreparedOutput prepareOutput(
    const Frame &, const Overlay &,
    bool want_annotated, bool want_compressed);
  bool publishCurrentFrame(const Frame &, PreparedOutput &);
  // 等待预算从到达时开始。当前帧等到自己的图层或预算结束，后到的帧留在队列里。
  void composeLoop();
  // 有图层或状态文字需要改像素时克隆源图；否则共享只读像素。
  cv::Mat buildCanvas(const Frame &, const Overlay &, bool &);
  void acceptFrame(std::shared_ptr<Frame> frame);
  void setTask(const std::string & task, std::uint64_t epoch, std::int64_t cutoff);
  // update 在 overlay_mutex_ 内只更新载荷；不要在回调中绘图、编码或重入本会话的加锁入口。
  template<class Update>
  void insertOverlay(
    const OverlayFrameKey & key, OverlayLayer layer,
    std::int64_t current, double timeout, Update update)
  {
    std::lock_guard<std::mutex> lock(overlay_mutex_);
    if (synchronizer_.insert(key, layer, current, timeout, update)) {
      overlay_cv_.notify_all();
    }
  }
  StreamNode & owner_;
  std::unique_ptr<MjpegServer> streamer_;
  JpegEncoder jpeg_encoder_;
  // 编码器与帧键仅由合成线程访问；订阅回调经邮箱和同步缓存交接，不直接操作编码缓存。
  std::uint64_t encode_frame_key_{0};
  // 按到达顺序保留最多三帧；满时丢最旧待处理帧，已取出的合成帧不受覆盖。
  PendingFrames<Frame> frame_slot_;

  // 运行期 synchronizer_ 的共享状态由此锁保护；启动前的 setRequirements() 不与工作线程并发。
  mutable std::mutex overlay_mutex_;
  OverlaySynchronizer<Overlay> synchronizer_;
  std::condition_variable overlay_cv_;
  std::atomic<std::size_t> composed_instances_{0};

  RateMeter compose_meter_;
  std::atomic<std::uint64_t> image_arrivals_{0};
  WindowCounter image_window_;
  WindowCounter compose_window_;
  WorkerGroup workers_;
};
