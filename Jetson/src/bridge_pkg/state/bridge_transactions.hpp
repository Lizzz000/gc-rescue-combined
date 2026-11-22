#pragma once

// Copyright 2026 li
// SPDX-License-Identifier: Apache-2.0

#include <array>
#include <chrono>
#include <cstdint>
#include <optional>
#include <string_view>

#include "protocol/arrival_point.hpp"
#include "protocol/search_request.hpp"
#include "state/bridge_state.hpp"
#include "transport/transport.hpp"

// 到达、ROI 和命令回执共用业务身份。调用方串行传入事件，由端口执行服务和排队。
class BridgeTransactions
{
public:
  using Clock = std::chrono::steady_clock;
  enum class Service {Roi, Command};
  struct CommandRequest
  {
    std::uint8_t command;
    std::uint64_t roi_request_id;
    std::uint64_t generation;
    std::uint64_t requested_cycle_generation;
  };

  class Ports
  {
  public:
    virtual ~Ports() = default;
    virtual BridgeSourceTime sourceNow() const = 0;
    virtual Clock::time_point monotonicNow() const = 0;
    virtual bool serviceReady(Service service) = 0;
    virtual std::int64_t sendRoi(std::uint64_t generation) = 0;
    virtual std::int64_t sendCommand(const CommandRequest & request) = 0;
    virtual void cancelService(Service service, std::int64_t request_id) = 0;
    virtual bool socketOpen() const = 0;
    virtual void queueReply(canid_t id) = 0;
    virtual void dropReply(canid_t id) = 0;
    virtual void dropRequest() = 0;
    virtual void setRequestError(std::string_view message) = 0;
    virtual void warnRequest(const char * message) = 0;
  };

  BridgeTransactions(
    BridgeState & state, ArrivalPointSession & arrival_point, Ports & ports,
    Clock::duration request_timeout);

  void handleArrival();
  ArrivalPointDecision handleArrivalPoint(
    std::uint8_t kind, std::uint32_t request_id, bool motion_lease_open);
  // command 由板间协议解析，取值为运输命令 1–4。
  void handleMissionCommand(std::uint8_t command, std::uint32_t token);

  // 先处理截止和代次，再由 ROS 适配层读取 future。
  bool acceptRoiReply(std::uint64_t generation, Clock::time_point current);
  void finishRoi(BridgeRoiResult result = {}, std::uint64_t request_id = 0);
  bool acceptCommandReply(std::uint64_t generation, Clock::time_point current);
  void commandReply(
    std::uint8_t command, std::uint64_t requested_cycle_generation, bool accepted,
    Clock::time_point current);

  void onStateUpdate();
  void checkRequestTimeouts(Clock::time_point current);
  void disconnect();
  void missionRestarted();
  void refreshTurnRequest(Clock::time_point current, bool force = false);
  void resolvePendingModeReply(Clock::time_point current);
  void armGripperResult(const GripperResult & result);
  const can_frame & replyFrame(canid_t id) const {return replies_.frame(id);}
  bool replyCurrent(canid_t id) const;

private:
  friend struct BridgeRuntimeTestAccess;

  struct TurnIdentity
  {
    std::uint64_t roi_request_id;
    BridgeDestination destination;
    std::uint64_t cycle_generation;
  };
  struct CommandIdentity
  {
    std::uint8_t command;
    std::uint64_t expected_generation;
    std::uint64_t roi_request_id;
    BridgeDestination destination;
    BridgeCyclePhase origin_phase;
    bool delivery_query{false};
  };
  struct PendingModeReply
  {
    std::uint8_t command;
    std::uint32_t token;
    std::uint64_t requested_generation;
    Clock::time_point deadline;
  };

  void requestRoi();
  void requestCommand(std::uint8_t command, std::uint32_t token, bool delivery_query = false);
  bool turnAuthorized() const;
  bool turnRequestCurrent() const;
  bool commandAlreadyAccepted(std::uint8_t command, std::uint32_t token, bool delivery_query) const;
  bool commandIdentityCurrent(const CommandIdentity & identity) const;
  const CommandIdentity * deliveryQueryIdentity(std::uint32_t token) const;
  void armReply(canid_t id, const can_frame & frame);
  void armCommandReply(bool accepted, std::uint8_t command, std::uint32_t token);
  bool switchFailed() const;
  bool modeSwitchFailed(std::uint8_t command, std::uint64_t requested_generation) const;
  bool switchConfirmed(std::uint8_t command) const;
  void commitSearchAbort(std::uint64_t requested_cycle_generation);

  BridgeState & state_;
  ArrivalPointSession & arrival_point_;
  Ports & ports_;
  Clock::duration request_timeout_;
  BridgeServiceRequest roi_request_, command_request_;
  SearchRequestSession search_request_;
  BridgeReplySlots replies_;
  std::optional<TurnIdentity> turn_, roi_confirmation_;
  Clock::time_point next_turn_;
  std::uint64_t roi_cycle_generation_{0};
  bool roi_attempted_{false};
  std::optional<bool> roi_passed_;
  // 重找一经受理即撤销旧路线。跨 USB 重连保留，等新搜索解除保持。
  bool route_aborted_{false};
  std::optional<CommandIdentity> pending_command_identity_, command_reply_identity_;
  std::array<std::optional<CommandIdentity>, 4> accepted_commands_{};
  // 最近尝试与最近成功可以属于不同趟，均跨重找和 USB 重连保留。
  std::optional<CommandIdentity> last_delivery_request_;
  std::uint32_t pending_command_token_{0};
  std::optional<PendingModeReply> pending_mode_reply_;
};
