#pragma once
#include "camera/camera.hpp"
#include "field/field.hpp"
#include "stream/stream.hpp"

// ---- 视觉节点 ----
// 职责：检测节点的 ROS 装配。图传在 stream_node.hpp。
// Copyright 2026 jetson_yolo integration
// SPDX-License-Identifier: Apache-2.0
// 检测节点：装配独占 USB 摄像头的运行会话，seg／field 两阶段互斥。
//
// 图像来源的两种选择（消费方按需订阅，两者互不影响）：
//   * `~/image_raw`：采集线程投递、原图线程按 raw_max_fps 发布，只保证"最近的一帧"，不保证被推理过。
//   * `~/image_processed`：结果发布线程按订阅需求发布该结果实际使用的源帧，不受 raw_max_fps 限制。
//     header 与同帧的 ~/seg/detections、~/field/results 完全一致（同 stamp 同 frame_id）。
//     结果邮箱过载会覆盖尚未发布的结果，旧阶段/相机会话也会被拒绝，因此不保证每次推理都交付一帧。
//     图传默认订阅处理帧；publish_processed=false 时不创建此发布者。

// 线程工具在相邻头文件，类定义没有改：
// latest_slot.hpp、diagnostics.hpp、stage_gate.hpp、worker_group.hpp。

#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <functional>
#include <exception>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <thread>
#include <utility>
#include <vector>

#include "node/latest_slot.hpp"
#include "node/diagnostics.hpp"
#include "node/stage_gate.hpp"
#include "node/worker_group.hpp"

#include "node/detector.hpp"
#include "node/field_parameters.hpp"
#include "node/ros_convert.hpp"

#include <chrono>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <opencv2/core.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/header.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>

#include "interface_pkg/msg/detector_status.hpp"
#include "interface_pkg/msg/field_observation.hpp"
#include "interface_pkg/msg/inference_timing.hpp"
#include "interface_pkg/msg/task_state.hpp"


#include <atomic>
#include <variant>

// ROS 适配层：声明参数与接口，并保证运行会话在线程回收前仍可访问节点资源。
class DetectNode : public rclcpp::Node {
public:
  explicit DetectNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~DetectNode() override;

  /// 对应图像发布者上的进程内订阅数（没有发布者时为 0），供合成部署自检实际使用的图像路径。
  /// get_subscription_count() 把进程内与跨进程订阅都算进去，只有这个计数能区分两条投递路径。
  std::size_t rawIntraProcessSubscribers() const;
  std::size_t processedIntraProcessSubscribers() const;

private:
  friend struct DetectRuntimeTestAccess;
  // 采集后只读的源帧：cv::Mat 持有像素，两个邮箱可共享同一个 Frame，处理帧不另拍一张图。
  // generation 对应识别阶段，camera_session 对应相机重连；推理复位和最终发布均核对两者。
  struct Frame
  {
    cv::Mat image;
    // H13：推理用 NVMM 缓冲，推理线程用完即释放（Frame 以 const 共享，故 mutable）；field/发布只用 image。
    mutable std::shared_ptr<void> nvmm;
    rclcpp::Time stamp;
    std::uint64_t sequence{0};
    std::uint64_t generation{0};
    std::uint64_t camera_session{0};
  };

  // ---- 启动配置与接口装配；运行会话尚未启动 ----
  void declareParameters();
  void loadParameters();
  DetectorConfig loadDetectorConfig() const;
  // 初始化顺序由构造函数显式编排：参数／后端 → ROS 输出／初始状态 → 服务 → 后台线程。
  // 返回实际装配配置供启动日志使用；退出导致的初始化取消返回空值，尚未启动相机线程。
  // task 所需后端缺失即失败；开发场地配置必须显式选择 field。
  std::optional<DetectorConfig> initializeDetector();
  void createRosOutputs();
  void createStageService();
  rcl_interfaces::msg::SetParametersResult onParameters(
    const std::vector<rclcpp::Parameter> & parameters);

  // 运行会话拥有线程与邮箱；ROS 结果在 StageGate 的发布闸门内核对代次后发布。
  void publishSeg(const Frame & frame, const SegRun & run);
  void publishField(const Frame & frame, const FieldResult & run);
  void publishRaw(const Frame & frame);
  void publishStatus();
  // 组装"该结果消息真正用的那一帧"（处理帧）：只由结果发布线程调用；无发布者或订阅者时返回 nullptr。
  // header 由调用方复用结果消息的 header，真正发布在 publishSeg/publishField 的闸门内与结果一起完成。
  std::unique_ptr<sensor_msgs::msg::Image> prepareProcessed(
    const Frame & frame, const std_msgs::msg::Header & header);
  // 发布闸门内核对阶段与相机会话；task_bit 1/2 要求对应阶段激活，0 原图还检查阶段起点。
  bool frameIsCurrent(const Frame & frame, int task_bit) const;

  // ---- 阶段状态：源时间截止与代次由同一个 StageGate 维护 ----
  bool segActive() const;
  bool fieldActive() const;
  bool applyActiveTask(const std::string & value, std::string & reason);
  void applyModeParameters();
  // 墙钟定期重发当前阶段；消息构造只在 StageGate 发布锁内调用，不改已建立的水位和代次。
  void refreshActiveTask();
  void publishTaskState(int mask, std::int64_t stamp_ns, std::uint64_t epoch);

  // 启动配置只在后台线程启动前写入；可热更新帧率保存在 Runtime 的原子字段中。
  bool publish_masks_{false};
  bool publish_field_results_{true};

  bool publish_raw_{true};
  // 处理帧话题的开关：false 时根本不创建发布者，~/image_processed 不存在。
  bool publish_processed_{true};
  // 这些成员初值不是第二份生产配置；构造时必须先成功装载启动参数，才会启动后台线程。
  double raw_max_fps_{120.0};
  CameraConfig camera_config_;
  bool publish_timing_{true};
  bool publish_status_{true};
  std::string frame_id_{"camera"};

  // ROS 接口由节点持有，析构时先停止运行会话，再由成员析构释放接口。
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr raw_pub_;
  // 处理帧发布者：与 raw_pub_ 各自独立，publish_processed=false 时为空。
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr processed_pub_;
  rclcpp::Publisher<vision_msgs::msg::Detection2DArray>::SharedPtr seg_det_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr seg_mask_pub_;
  rclcpp::Publisher<interface_pkg::msg::InferenceTiming>::SharedPtr seg_timing_pub_;
  rclcpp::Publisher<interface_pkg::msg::FieldObservation>::SharedPtr field_results_pub_;
  rclcpp::Publisher<interface_pkg::msg::InferenceTiming>::SharedPtr field_timing_pub_;
  rclcpp::Publisher<interface_pkg::msg::DetectorStatus>::SharedPtr status_pub_;
  rclcpp::Publisher<interface_pkg::msg::TaskState>::SharedPtr mode_pub_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr set_field_service_;
  rclcpp::TimerBase::SharedPtr status_timer_;

  // Runtime 定义保留在私有头；本类析构的实现包含完整定义，确保 unique_ptr 正确释放线程和后端。
  struct Runtime;
  std::unique_ptr<Runtime> runtime_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr mode_param_cb_;
};

struct DetectNode::Runtime
{
  explicit Runtime(DetectNode & owner)
  : owner_(owner) {}
  ~Runtime() {stop();}
  void start();
  void stop();
  // 相机只归采集循环使用；重连清槽并使此前的相机会话失效。
  void captureLoop();
  // 逐帧选择一个后端；切阶段、重连或尺寸变化后复位有状态场地定位器。
  void inferenceLoop();
  // 两条发布循环分别消费原图/结果，均在节点最终发布闸门内复核会话与阶段。
  void rawLoop();
  void resultLoop();
  void wakeWaiters();
  // 由 StageGate 在原发布锁和采集锁内调用；只清待处理载荷，不关闭邮箱。
  void clearPendingFrames();
  void logLetterboxScale();
  DetectNode & owner_;
  std::unique_ptr<Detector> detector_;
  std::unique_ptr<UsbCamera> camera_;
  // 两个消费者各取最新帧；采集端投递后不再修改 Frame 或其图像数据。
  LatestSlot<const Frame> inference_slot_;
  LatestSlot<const Frame> raw_slot_;
  // 结果持有其真实输入 Frame；即使邮箱覆盖或线程交错，也不会改配到后来采集的图像。
  struct CompletedFrame
  {
    std::shared_ptr<const Frame> frame;
    std::variant<SegRun, FieldResult> result;
  };
  // 最多一个待发布结果，过载时整体替换，图像与识别结果不会拆开排队。
  LatestSlot<const CompletedFrame> result_slot_;
  std::atomic<std::uint64_t> sequence_{0};

  // 原子计数供状态定时器采样；统计不能作为阶段/相机会话的准入依据。
  std::atomic<std::uint64_t> captured_frames_{0};
  RateMeter seg_meter_;
  RateMeter field_meter_;
  std::atomic<std::uint64_t> raw_published_{0};
  std::atomic<std::uint64_t> camera_reopens_{0};
  WindowCounter capture_window_;
  WindowCounter seg_window_;
  WindowCounter field_window_;
  WindowCounter raw_window_;
  std::atomic<double> seg_gpu_ms_{0.0};
  std::atomic<double> seg_post_ms_{0.0};
  std::atomic<double> field_post_ms_{0.0};
  std::atomic<double> seg_max_fps_{0.0};
  std::atomic<double> field_max_fps_{0.0};
  std::atomic<std::uint64_t> seg_skipped_{0};
  std::atomic<std::uint64_t> field_skipped_{0};

  // StageGate 保护阶段与相机会话交接，WorkerGroup 保护线程寿命；二者不能互相替代。
  StageGate stage_;
  WorkerGroup workers_;
  FrameRateLimiter raw_limiter_;
};
