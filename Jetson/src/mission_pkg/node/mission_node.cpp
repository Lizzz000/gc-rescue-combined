#include "node/mission_node.hpp"
#include "roi/ros/roi_ros.hpp"

// onArrived 和 onCommand 把 STM32 的事件交给控制器。控制器直接调用本文件的 MissionOut。
// Copyright 2026 li
// SPDX-License-Identifier: Apache-2.0

#include <chrono>
#include <exception>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <interface_pkg/app/core.hpp>
#include <stdexcept>
#include <memory>
#include <rclcpp_components/register_node_macro.hpp>

MissionNode::MissionNode(const rclcpp::NodeOptions & options)
try : Node("mission_node", options), settings_(loadParameters()),
  controller_(*this, settings_.request_timeout_s, transportPolicy())
{
  createRosInterfaces(settings_);
  publishStatus(controller_.state());
  // 50 ms 是超时扫描周期；实际是否超时仍按每个请求自己的单调截止判断。
  timeout_timer_ = create_wall_timer(
    std::chrono::milliseconds(50), [this] {checkTimeouts();});
} catch (const std::invalid_argument & error) {
  throw rescue::ConfigurationError(error.what());
}

MissionNode::~MissionNode() = default;

MissionNode::Settings MissionNode::loadParameters()
{
  Settings settings;
  rcl_interfaces::msg::ParameterDescriptor descriptor;
  descriptor.read_only = true;
  settings.detector_service = rescue::declareRequired<std::string>(
    *this, "detector_service", descriptor);
  settings.detector_task_topic = rescue::declareRequired<std::string>(
    *this, "detector_task_topic", descriptor);
  settings.target_roi_service = rescue::declareRequired<std::string>(
    *this, "target_roi_service", descriptor);
  settings.request_timeout_s = rescue::declareRequired<double>(
    *this, "request_timeout_s", descriptor);
  return settings;
}

TransportPolicy MissionNode::transportPolicy()
{
  return {cargo::kProductionFirstGreenCount, cargo::kProductionMaxSupplyCount};
}

void MissionNode::createRosInterfaces(const Settings & settings)
{
  // 这些接口均使用节点默认互斥组；否则控制器、等待代次与请求头槽需要额外同步。
  detector_client_ = create_client<Switch>(settings.detector_service);
  roi_client_ = create_client<CheckRoi>(settings.target_roi_service);
  roi_pub_ = create_publisher<interface_pkg::msg::RoiCheck>("~/roi_result", rclcpp::QoS(10));
  control_pub_ = create_publisher<interface_pkg::msg::TaskState>(
    "~/active_task", rclcpp::QoS(1).reliable().transient_local());
  status_pub_ = create_publisher<interface_pkg::msg::MissionStatus>(
    "~/status", rclcpp::QoS(1).reliable().transient_local());
  state_sub_ = create_subscription<interface_pkg::msg::TaskState>(
    settings.detector_task_topic, rclcpp::QoS(1).reliable().transient_local(),
    [this](interface_pkg::msg::TaskState::ConstSharedPtr state) {onTaskChanged(state);});
  arrived_service_ = create_service<CheckRoi>(
    "~/arrived",
    [this](RequestHeader header, std::shared_ptr<CheckRoi::Request>) {onArrived(header);});
  command_service_ = create_service<Command>(
    "~/command",
    [this](std::shared_ptr<Command::Request> request,
      std::shared_ptr<Command::Response> response) {onCommand(*request, *response);});
}

void MissionNode::onArrived(const RequestHeader & header)
{
  arrival_answered_ = false;
  roi_call_header_ = header;
  try {
    controller_.requestRoi();
  } catch (const std::exception & error) {
    roi_call_header_.reset();
    settleArrivalFailure(header, error.what());
    return;
  }
  roi_call_header_.reset();
}

void MissionNode::onCommand(const Command::Request & request, Command::Response & response)
{
  // 1/2/4 传完整 ROI 编号给核心核对，不能在此截成 CAN token，也不能用当前路由替换请求身份。
  try {
    const auto result = controller_.requestCommand(
      static_cast<MissionCommand>(request.command), request.roi_request_id);
    response.accepted = result.accepted;
    response.message = result.message;
    // 检测服务不可用时切换会在同一次调用里进入 Fault。只读收据没有发布，不能改成拒绝。
    if (result.accepted && result.acted &&
      controller_.state().cycle_phase == MissionState::CyclePhase::Fault)
    {
      response.accepted = false;
      response.message = controller_.state().reason;
    }
  } catch (const std::exception & error) {
    // 发布失败关闭坐标。206 可能已记账，STM32 须先重查同编号收据，不能重推。
    response.accepted = false;
    response.message = error.what();
    controller_.commandFailed(error.what());
  }
}

void MissionNode::onTaskChanged(interface_pkg::msg::TaskState::ConstSharedPtr state)
{
  std::int64_t source_stamp = 0;
  if (!RecognitionValidation::sourceTime(
      state->header.stamp, now().nanoseconds(), source_stamp)) return;
  const MissionTaskState task{state->task, state->epoch, {source_stamp}};
  task_message_ = state.get();
  try {
    controller_.taskChanged(task, std::chrono::steady_clock::now());
  } catch (...) {
    task_message_ = nullptr;
    throw;
  }
  task_message_ = nullptr;
}

void MissionNode::publishRoi(
  const RoiRuleResult & result, std::uint64_t request_id, bool use_source_stamp)
{
  std_msgs::msg::Header source_header;
  if (use_source_stamp && roi_source_) source_header = roi_source_->header;
  else source_header.stamp = now();
  CheckRoi::Response message;
  message.result = RoiRosConversion::message(result, source_header);
  message.result.request_id = request_id;
  // 立即拒绝使用这次 arrived 的头，不清除已经发出的 ROI 槽。
  // 异步完成只使用槽内的头，并在发送前取走。
  const auto header = roi_call_header_ ? roi_call_header_ : roi_header_;
  if (!roi_call_header_) roi_header_.reset();
  if (!header) return;
  sendRoiResponse(header, message);
  publishRoiResult(message.result);
  // 应答和诊断都已发出才提交。提交后的状态发布失败交给所属回调补救。
  if (controller_.roiPublishCommits()) controller_.commitRoiAfterReply();
}

void MissionNode::forwardTask()
{
  if (task_message_) control_pub_->publish(*task_message_);
}

bool MissionNode::roiServiceReady()
{
  return roi_client_->service_is_ready();
}

void MissionNode::sendRoiRequest(std::uint64_t generation)
{
  if (!controller_.beginRoiWait(generation, std::chrono::steady_clock::now())) return;
  roi_header_ = roi_call_header_;
  const auto kind = PendingOperation::Kind::RoiCheck;
  try {
    auto future = roi_client_->async_send_request(
      std::make_shared<CheckRoi::Request>(),
      [this, generation](rclcpp::Client<CheckRoi>::SharedFuture result) {
        onRoiReply(generation, result);
      });
    controller_.registerRequest(kind, generation, future.request_id);
  } catch (const std::exception & error) {
    // 发送没出去。清掉请求头再补救，避免失败应答用到这次没发出去的槽。
    roi_header_.reset();
    controller_.recoverRequest(kind, generation, error.what());
  }
}

bool MissionNode::detectorReady()
{
  return detector_client_->service_is_ready();
}

void MissionNode::sendSwitchRequest(bool field, std::uint64_t generation)
{
  const auto kind = PendingOperation::Kind::AutomaticSwitch;
  if (!controller_.beginSwitchWait(kind, generation, std::chrono::steady_clock::now())) return;
  auto request = std::make_shared<Switch::Request>();
  request->data = field;
  try {
    auto future = detector_client_->async_send_request(
      request,
      [this, kind, generation](rclcpp::Client<Switch>::SharedFuture result) {
        onSwitchReply(kind, generation, result);
      });
    controller_.registerRequest(kind, generation, future.request_id);
  } catch (const std::exception & error) {
    controller_.recoverRequest(kind, generation, error.what());
  }
}

void MissionNode::cancel(PendingOperation::Kind kind, std::int64_t request_id)
{
  if (kind == PendingOperation::Kind::RoiCheck) roi_client_->remove_pending_request(request_id);
  else detector_client_->remove_pending_request(request_id);
}

void MissionNode::onRoiReply(
  std::uint64_t generation, rclcpp::Client<CheckRoi>::SharedFuture result)
{
  const auto kind = PendingOperation::Kind::RoiCheck;
  // 先看代次和截止。过期的回调不能 future.get()，也不能完成新的等待。
  if (!controller_.replyGate(kind, generation, std::chrono::steady_clock::now())) return;
  try {
    const auto response = result.get();
    roi_source_ = &response->result;
    controller_.roiReply(
      generation, RoiRosConversion::rule(response->result), response->result.request_id);
    roi_source_ = nullptr;
    controller_.roiCallbackFinished(generation);
  } catch (const std::exception & error) {
    roi_source_ = nullptr;
    roi_header_.reset();
    try {
      controller_.recoverRequest(kind, generation, error.what());
    } catch (...) {
      controller_.roiCallbackFinished(generation);
      throw;
    }
    controller_.roiCallbackFinished(generation);
  }
}

void MissionNode::onSwitchReply(
  PendingOperation::Kind kind, std::uint64_t generation,
  rclcpp::Client<Switch>::SharedFuture result)
{
  if (!controller_.replyGate(kind, generation, std::chrono::steady_clock::now())) return;
  try {
    const auto response = result.get();
    controller_.switchReply(kind, generation, response->success, response->message);
    controller_.switchCallbackFinished(kind, generation);
  } catch (const std::exception & error) {
    // 这里只补救一次。发送失败那条路径才在 recoverRequest 里再试一次。
    try {
      controller_.requestFailed(kind, generation, error.what());
    } catch (...) {
      controller_.switchCallbackFinished(kind, generation);
      throw;
    }
    controller_.switchCallbackFinished(kind, generation);
  }
}

void MissionNode::sendRoiResponse(const RequestHeader & header, CheckRoi::Response & response)
{
  arrived_service_->send_response(*header, response);
  arrival_answered_ = true;
}

void MissionNode::publishRoiResult(const interface_pkg::msg::RoiCheck & result)
{
  roi_pub_->publish(result);
}

void MissionNode::publishStatus(const MissionState & state)
{
  interface_pkg::msg::MissionStatus message;
  message.header.stamp = now();
  message.cycle_phase = static_cast<std::uint8_t>(state.cycle_phase);
  message.cycle_generation = state.cycle_generation;
  message.destination = static_cast<std::uint8_t>(state.destination);
  message.operation = static_cast<std::uint8_t>(state.operation);
  message.outcome = static_cast<std::uint8_t>(state.outcome);
  message.transport_phase = static_cast<std::uint8_t>(state.transport_phase);
  message.roi_request_id = state.roi_request_id;
  message.reason = state.reason;
  status_pub_->publish(message);
}

void MissionNode::checkTimeouts()
{
  controller_.checkTimeouts(std::chrono::steady_clock::now());
  // 周期状态同时作为桥接失联保护的存活信号。
  publishStatus(controller_.state());
}

void MissionNode::settleArrivalFailure(
  const RequestHeader & header, const std::string & message)
{
  if (!arrival_answered_ && header) {
    CheckRoi::Response response;
    response.result.header.stamp = now();
    response.result.reason = message;
    sendRoiResponse(header, response);
    publishRoiResult(response.result);
  }
  controller_.arrivalInterrupted(message);
}

RCLCPP_COMPONENTS_REGISTER_NODE(MissionNode)
