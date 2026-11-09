#pragma once

// ---- 任务节点 ----
// 职责：任务、目标和安全三个角色的 ROS 装配。
// Copyright 2026 li
// SPDX-License-Identifier: Apache-2.0


#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <interface_pkg/msg/roi_check.hpp>
#include <interface_pkg/msg/task_state.hpp>
#include <interface_pkg/srv/check_roi.hpp>
#include <interface_pkg/app/core.hpp>
#include <chrono>
#include <cstdint>
#include <optional>
#include <vision_msgs/msg/detection2_d_array.hpp>
#include "roi/roi.hpp"

// ---- TargetNode：目标转发与原始检测 ROI ----
// input_topic 转发零或一个完整合法目标；detections_topic 用完整原始检测计数，两路帧历史独立。
// 目标转发只认实际 seg 会话；运输资格和到达后的 CAN 停发由 Mission、Bridge 处理。
// ROI 只受理一个请求，使用源时间不早于受理时刻的首张合格原始帧，计入相交或贴边的框。
// 完成时先取走等待对象，再发布诊断、应答服务；忙或非 seg 的拒绝不覆盖旧等待。
// 所有回调使用默认互斥组；矩形、类别和原图尺寸由只读参数提供，生产组合取自公共规则。
class TargetNode : public rclcpp::Node
{
public:
  explicit TargetNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

protected:
  using Check = interface_pkg::srv::CheckRoi;
  using RequestHeader = std::shared_ptr<rmw_request_id_t>;
  // 诊断和应答的 ROS 边界；完成请求前已取走等待对象，异常不会导致重复应答。
  virtual void publishRoiResult(const interface_pkg::msg::RoiCheck & result);
  virtual void sendCheckResponse(const RequestHeader & header, Check::Response & response);

private:
  friend struct TargetNodeTestAccess;
  using RoiClock = std::chrono::steady_clock;
  struct Settings
  {
    // input_topic 接选择后的目标，detections_topic 接完整原始检测，control_topic 接识别会话。
    std::string input_topic, control_topic, detections_topic;
    // 单位均为秒：reset 同时约束源帧龄与目标断流；roi_check 仅约束一次服务等待。
    double reset_timeout_s, roi_check_timeout_s;
    // 原图像素尺寸用于把检测框换算到归一化 ROI，不是缩放后图传尺寸。
    std::int64_t image_width, image_height;
    // 左上角 x/y 与宽高均为 [0,1] 比例；不是中心点或像素坐标。
    NormalizedRoi roi;
    // 与模型类别顺序一致的四色数字编号，不能用轨迹 ID 或标签名称代替。
    RoiClassIds class_ids;
  };
  Settings loadParameters();
  void createRosInterfaces();
  // 一个请求的关联信息集中保存；optional 有值才表示忙，拒绝新请求不能覆盖它。
  struct PendingRoiCheck
  {
    RequestHeader response_header;
    std::uint64_t id{0};
    // 受理时的 ROS 时间（纳秒）：排除请求前已采集、随后才到达的帧。
    std::int64_t earliest_frame_ns{0};
    // 受理时已知的原始检测坐标系；尚无原始帧时为空。
    std::string expected_frame_id;
    // 单调时钟截止不受 ROS 时间暂停/跳变影响，与源帧时间不得直接比较。
    RoiClock::time_point deadline;
  };
  // 只允许零个或一个合法目标输出，沿用其源 header；不作为 ROI 计数输入。
  void targetCallback(const vision_msgs::msg::Detection2DArray::ConstSharedPtr & msg);
  // 记录完整原始检测的时间与坐标系；等待中的请求消费源时间不早于受理时间的首帧。
  void detectionsCallback(const vision_msgs::msg::Detection2DArray::ConstSharedPtr & msg);
  // 原始回调先处理坐标系变化，再检查此处的单调截止，最后检查请求源时间；恰好到截止即失败。
  void consumeRoiFrame(
    const vision_msgs::msg::Detection2DArray & msg, std::int64_t stamp,
    RoiClock::time_point observed_at);
  bool expireRoiCheck(RoiClock::time_point observed_at);
  // 同时只受理一个检查，忙或非 seg 时立即拒绝且不覆盖旧请求头。
  void requestCheck(const RequestHeader & header);
  // 先取走整个等待对象，再按“诊断发布→服务应答”完成；异常不会遗留忙状态或重复应答。
  void finishCheck(interface_pkg::msg::RoiCheck result);
  void respondCheck(
    const RequestHeader & header,
    const interface_pkg::msg::RoiCheck & result);
  interface_pkg::msg::RoiCheck failedCheck(const std::string & reason) const;
  // 会话变化使等待检查失败，同时关闭旧源帧的准入资格。
  void modeCallback(const interface_pkg::msg::TaskState::ConstSharedPtr & msg);
  void clearOutput(const builtin_interfaces::msg::Time & stamp);
  // ROI 截止与选中目标断流分开处理，均使用单调时钟等待。
  void checkTimeout();
  static RoiPolicy roiPolicy();

  // ---- 参数、会话与独立等待 ----
  Settings settings_;
  // enabled 只在 seg 会话为真；have_raw 只描述当前会话是否已收到合格原始帧。
  bool enabled_{false}, have_raw_{false};
  std::optional<recognition::TaskSession> session_;
  std::string raw_frame_id_;
  // 每次服务调用（含忙/停用拒绝）递增；只在本节点进程内关联请求。
  std::uint64_t request_id_{0};
  // 原始源图时间与 pending_roi_ 中的单调截止分别用于帧先后和请求超时。
  std::int64_t raw_stamp_ns_{0};
  // 选中目标通路的源 header 用于发布清空结果，与原始检测的坐标系快照分开。
  std_msgs::msg::Header last_header_;
  FrameGate frame_gate_;
  std::unique_ptr<RoiValidator> validator_;
  std::optional<PendingRoiCheck> pending_roi_;
  rclcpp::Subscription<vision_msgs::msg::Detection2DArray>::SharedPtr input_sub_, detections_sub_;
  rclcpp::Subscription<interface_pkg::msg::TaskState>::SharedPtr mode_sub_;
  rclcpp::Publisher<vision_msgs::msg::Detection2DArray>::SharedPtr output_pub_;
  rclcpp::Publisher<interface_pkg::msg::RoiCheck>::SharedPtr roi_pub_;
  rclcpp::Service<Check>::SharedPtr check_service_;
  rclcpp::TimerBase::SharedPtr timeout_timer_;
};
