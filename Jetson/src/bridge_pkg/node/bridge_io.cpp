// 每 5 毫秒轮询 USB。事务模块处理到达及命令，ROS 服务通过这里提交。

#include "node/bridge_node.hpp"

// Copyright 2026 li
// SPDX-License-Identifier: Apache-2.0
// BridgeRuntime 与 BridgeNode 共用默认互斥回调组。源帧用 ROS 时间，队列期限用单调时间。

#include <interface_pkg/app/core/recognition.hpp>
#include <chrono>
#include <cstdint>
#include <memory>
#include <string>
#include <utility>
#include <exception>


// ---- BridgeRuntime：传输轮询 ----

// 单轮顺序：服务超时 → socket 准入 → 有界读取 → 刷新转向授权 → 事件帧 → 一帧周期数据。
// EINTR 计入收发预算；异常仍由本轮 catch 收尾。
void BridgeRuntime::io()
{
  try {
    transactions_.checkRequestTimeouts(Clock::now());
    // 夹爪工作线程独立于 USB；断线期间也先保存其终态，重连后按编号查询。
    pollGripper();
    if (!ensureSocketReady()) {return;}
    if (!readAvailableFrames()) {return;}
    transactions_.refreshTurnRequest(Clock::now());
    if (!can_->socketOpen()) {return;}
    flushTxQueue();
    if (can_->socketOpen()) {sendPeriodicFrame();}
  } catch (const CanTransport::FatalError & error) {
    // 独占失败或半包无法安全恢复时退出；不能等待后自动抢占刚被teleop释放的设备。
    closeSocket(error.what());
    throw;
  } catch (const std::exception & error) {
    handleIoFailure(error);
  }
}

// 每 5 ms 轮发一类坐标帧；目标和安全区各为 100 Hz。
// 背压或中断时丢弃本次机会，下一拍轮到另一类；载荷始终在提交前重建。
void BridgeRuntime::sendPeriodicFrame()
{
  // 沿用 MissionStatus 有效期；停止周期流让 STM32 识别业务掉线。
  // 正常丢目标及等待/夹爪/转向阶段仍发送 valid=0，不额外增加心跳帧。
  const auto current = Clock::now();
  if (motion_lease_suspended_until_ && current < *motion_lease_suspended_until_) {return;}
  const auto id = periodic_schedule_.next(mission_watchdog_.alive({current}));
  if (!id) {return;}
  const can_frame frame = currentFrame(*id);
  std::string error;
  const auto result = can_->write(frame, error);
  if (result == CanTransport::WriteResult::SENT) {
    ++tx_frames_;
  } else if (result == CanTransport::WriteResult::BACKPRESSURE) {
    last_error_ = "USB transport backpressure";
  } else if (result != CanTransport::WriteResult::INTERRUPTED) {
    closeSocket(error);
  }
}

bool BridgeRuntime::ensureSocketReady()
{
  // socket 未打开时只按 next_open_ 触发重连（失败后固定 1 秒一次）。
  if (!can_->socketOpen()) {
    if (Clock::now() < next_open_) {return false;}
    openSocket();
    if (!can_->socketOpen()) {return false;}
  }
  // CDC 在 open 中完成初始化；拔插由真实 read/write 错误触发重连。
  return true;
}

bool BridgeRuntime::readAvailableFrames()
{
  // 为突发流量留余量，
  // 同时限制非阻塞尝试次数（含 EINTR），确保持续收包时本拍仍会推进发送。
  constexpr int max_reads_per_cycle = 32;
  for (int attempt = 0; attempt < max_reads_per_cycle; ++attempt) {
    can_frame frame{};
    std::string error;
    const auto result = can_->read(frame, error);
    if (result == CanTransport::ReadResult::EMPTY) {break;}
    if (result == CanTransport::ReadResult::INTERRUPTED) {continue;}
    if (result == CanTransport::ReadResult::FAILED) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "USB read from %s failed: %s", can_interface_.c_str(), error.c_str());
      closeSocket(error);
      return false;
    }
    ++rx_frames_;
    if (!handleCommand(frame)) {++dropped_frames_;}
  }
  return true;
}

// 校验业务心跳及帧格式后，将合法事件交给对应入口。
bool BridgeRuntime::handleCommand(const can_frame & frame)
{
  if (!mission_watchdog_.alive({Clock::now()})) {return false;}
  const auto command = CanProtocol::parseCommand(frame);
  if (command.kind == CanProtocol::CommandKind::NONE) {return false;}
  if (command.kind == CanProtocol::CommandKind::MALFORMED) {
    ++dropped_frames_;
    return true;
  }
  if (command.kind == CanProtocol::CommandKind::ARRIVAL) {
    transactions_.handleArrival();
    return true;
  }
  if (command.kind == CanProtocol::CommandKind::ARRIVAL_POINT) {
    handleArrivalPoint(command.command, command.token);
    return true;
  }
  transactions_.handleMissionCommand(command.command, command.token);
  return true;
}

bool BridgeRuntime::serviceReady(BridgeTransactions::Service service)
{
  return service == BridgeTransactions::Service::Roi ?
    ports_.roi_client->service_is_ready() : ports_.command_client->service_is_ready();
}

std::int64_t BridgeRuntime::sendRoi(std::uint64_t generation)
{
  using CheckRoi = interface_pkg::srv::CheckRoi;
  return ports_.roi_client->async_send_request(
    std::make_shared<CheckRoi::Request>(),
    [this, generation](rclcpp::Client<CheckRoi>::SharedFuture result) {
      onRoiReply(generation, std::move(result));
    }).request_id;
}

std::int64_t BridgeRuntime::sendCommand(const BridgeTransactions::CommandRequest & command)
{
  using Command = interface_pkg::srv::MissionCommand;
  auto request = std::make_shared<Command::Request>();
  request->command = command.command;
  request->roi_request_id = command.roi_request_id;
  return ports_.command_client->async_send_request(
    request, [this, command](rclcpp::Client<Command>::SharedFuture result) {
      onCommandReply(command, std::move(result));
    }).request_id;
}

void BridgeRuntime::cancelService(BridgeTransactions::Service service, std::int64_t request_id)
{
  if (service == BridgeTransactions::Service::Roi) {
    ports_.roi_client->remove_pending_request(request_id);
  } else {
    ports_.command_client->remove_pending_request(request_id);
  }
}

void BridgeRuntime::onRoiReply(
  std::uint64_t generation, rclcpp::Client<interface_pkg::srv::CheckRoi>::SharedFuture result)
{
  const auto current = Clock::now();
  if (!transactions_.acceptRoiReply(generation, current)) {return;}
  try {
    const auto & checked = result.get()->result;
    BridgeRoiResult outcome;
    outcome.valid = checked.valid;
    outcome.passed = checked.valid && checked.passed;
    if (outcome.passed) {
      outcome.destination = static_cast<BridgeDestination>(
        RecognitionValueValidation::cargoSide(checked.orange_count));
    }
    transactions_.finishRoi(outcome, checked.request_id);
    transactions_.refreshTurnRequest(current);
  } catch (const std::exception & error) {
    last_error_ = error.what();
    transactions_.finishRoi();
  }
}

void BridgeRuntime::onCommandReply(
  const BridgeTransactions::CommandRequest & command,
  rclcpp::Client<interface_pkg::srv::MissionCommand>::SharedFuture result)
{
  const auto current = Clock::now();
  if (!transactions_.acceptCommandReply(command.generation, current)) {return;}
  bool accepted = false;
  try {
    accepted = result.get()->accepted;
  } catch (const std::exception & error) {
    last_error_ = error.what();
  }
  transactions_.commandReply(command.command, command.requested_cycle_generation, accepted, current);
}

void BridgeRuntime::warnRequest([[maybe_unused]] const char * message)
{
  RCLCPP_WARN(get_logger(), "%s", message);
}
