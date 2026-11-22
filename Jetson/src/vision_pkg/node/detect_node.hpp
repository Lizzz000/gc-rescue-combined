#pragma once

// Copyright 2026 jetson_yolo integration
// SPDX-License-Identifier: Apache-2.0

#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "camera/camera.hpp"
#include "node/detector.hpp"

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

class DetectRuntime;
struct DetectFrame;

// 参数和 ROS 消息适配。相机与工作线程由独立会话持有。
class DetectNode : public rclcpp::Node {
public:
  explicit DetectNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~DetectNode() override;

  /// 对应图像发布者上的进程内订阅数（没有发布者时为 0），供合成部署自检实际使用的图像路径。
  /// get_subscription_count() 把进程内与跨进程订阅都算进去，只有这个计数能区分两条投递路径。
  std::size_t rawIntraProcessSubscribers() const;
  std::size_t processedIntraProcessSubscribers() const;

private:
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

  // 同一源帧的图像和结果一起通过会话的发布闸门。
  bool publishSeg(const DetectFrame & frame, const SegRun & run);
  bool publishField(const DetectFrame & frame, const FieldResult & run);
  void publishRaw(const DetectFrame & frame);
  void publishStatus();
  std::unique_ptr<sensor_msgs::msg::Image> prepareProcessed(
    const DetectFrame & frame, const std_msgs::msg::Header & header);

  bool applyActiveTask(const std::string & value, std::string & reason);
  void applyModeParameters();
  // 墙钟定期重发当前阶段；消息构造只在 StageGate 发布锁内调用，不改已建立的水位和代次。
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

  std::unique_ptr<DetectRuntime> runtime_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr mode_param_cb_;
};
