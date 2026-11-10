#pragma once

// ---- 桥接节点 ----
// 职责：ROS 参数、话题和服务装配。
// Copyright 2026 li
// SPDX-License-Identifier: Apache-2.0

#include <chrono>
#include <array>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>
#include <interface_pkg/msg/safety_target.hpp>
#include <interface_pkg/msg/task_state.hpp>
#include <interface_pkg/msg/mission_status.hpp>
#include <cstdint>
#include <exception>
#include <string>
#include <interface_pkg/msg/bridge_status.hpp>
#include <interface_pkg/msg/roi_check.hpp>
#include <interface_pkg/srv/check_roi.hpp>
#include <interface_pkg/srv/mission_command.hpp>


#include "transport/transport.hpp"
#include "state/bridge_state.hpp"

class BridgeRuntime;

// ---- BridgeNode：参数与 ROS 装配 ----
// 节点创建只读参数、ROS 端口和定时器；CAN 会话与服务等待由 BridgeRuntime 持有。
class BridgeNode : public rclcpp::Node
{
public:
  explicit BridgeNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  /**
   * @brief 装配生产节点或注入传输；各配置只由对应接收入口检查一次。
   * @param[in] options ROS 选项与完整显式参数。
   * @param[in] transport 可为空；为空时按配置创建后端，否则接管传入对象。
   * @throws rescue::ConfigurationError 参数非法，构造中止，不注册轮询或打开设备。
   */
  BridgeNode(const rclcpp::NodeOptions & options, std::unique_ptr<CanTransport> transport);
  ~BridgeNode() override;

private:
  using Clock = std::chrono::steady_clock;
  struct StartupSettings;
  /**
   * @brief 读取参数并检查传输、端口和等待期限；状态域交给核心构造入口。
   * @return 一次装配设置；缺项或本层非法值抛出配置错误，不创建设备。
   */
  StartupSettings declareAndValidateParameters();
  // 端口与运行上下文共用节点默认互斥回调组。
  void createRosInterfaces(StartupSettings & settings);
  void createCanTimers();
  rclcpp::Subscription<vision_msgs::msg::Detection2DArray>::SharedPtr target_sub_;
  rclcpp::Subscription<interface_pkg::msg::SafetyTarget>::SharedPtr safety_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr velocity_sub_;
  rclcpp::Subscription<interface_pkg::msg::TaskState>::SharedPtr task_sub_;
  rclcpp::Subscription<interface_pkg::msg::MissionStatus>::SharedPtr mission_sub_;
  rclcpp::TimerBase::SharedPtr io_timer_, status_timer_;
  std::unique_ptr<BridgeRuntime> runtime_;
};

// ---- BridgeRuntime：CAN 会话与异步握手 ----
// 所有入口共用节点默认互斥回调组，不创建执行器或线程；节点必须比运行上下文存活更久。
// 销毁前停止执行器访问。七槽队列保存发送机会，回复槽保存载荷；ROI 与命令各有独立等待。
class BridgeRuntime
{
public:
  using Clock = std::chrono::steady_clock;
  struct Config
  {
    std::string can_interface;
    // 两者都是单调期限，但前者限制发送机会寿命，后者限制服务等待，不能合成一个超时。
    Clock::duration tx_queue_timeout, request_timeout;
    BridgeStateConfig state;
  };
  struct Ports
  {
    rclcpp::Publisher<interface_pkg::msg::BridgeStatus>::SharedPtr status_pub;
    rclcpp::Client<interface_pkg::srv::CheckRoi>::SharedPtr roi_client;
    rclcpp::Client<interface_pkg::srv::MissionCommand>::SharedPtr command_client;
  };
  BridgeRuntime(
    rclcpp::Node & node, Config config, Ports ports, std::unique_ptr<CanTransport> transport);
  BridgeRuntime(const BridgeRuntime &) = delete;
  BridgeRuntime & operator=(const BridgeRuntime &) = delete;

  // 有界轮询读取指令、淘汰过期发送机会并处理断线重连，不阻塞 ROS 回调。
  void io();
  // 每拍轮流发送一类周期帧，载荷按发送时的新鲜度生成。
  void sendPeriodicFrame();
  // 诊断只反映本机传输句柄、就绪状态与提交计数，不证明STM32在线或指令完成。
  void publishStatus();
  void targetCallback(vision_msgs::msg::Detection2DArray::ConstSharedPtr msg);
  void safetyCallback(interface_pkg::msg::SafetyTarget::ConstSharedPtr msg);
  void velocityCallback(geometry_msgs::msg::Twist::ConstSharedPtr msg);
  void taskCallback(interface_pkg::msg::TaskState::ConstSharedPtr msg);
  void missionCallback(interface_pkg::msg::MissionStatus::ConstSharedPtr msg);

private:
  friend struct BridgeRuntimeTestAccess;
  struct CommandIdentity;
  // socket 准入与有界收发；打开失败或接口被替换时本轮不再读写。
  void openSocket();
  void closeSocket(const std::string & reason);
  bool ensureSocketReady();
  bool readAvailableFrames();
  void flushTxQueue();
  void handleIoFailure(const std::exception & error);
  // ROS 输入只入队。周期载荷在写 socket 前由 currentFrame 重建。
  can_frame currentFrame(canid_t id) const;
  // STM32 指令、ROI、转向授权和回复快照。
  bool handleCommand(const can_frame & frame);
  void holdCoordinates(bool until_search = false);
  void handleArrival();
  void requestRoi();
  void onRoiReply(
    std::uint64_t generation,
    rclcpp::Client<interface_pkg::srv::CheckRoi>::SharedFuture result);
  void requestCommand(std::uint8_t command, std::uint32_t token = 0, bool delivery_query = false);
  void onCommandReply(
    std::uint64_t generation, std::uint8_t command,
    rclcpp::Client<interface_pkg::srv::MissionCommand>::SharedFuture result);
  void refreshTurnRequest(Clock::time_point current, bool force = false);
  bool turnAuthorized() const;
  bool turnRequestCurrent() const;
  bool commandAlreadyAccepted(std::uint8_t command, std::uint32_t token, bool delivery_query) const;
  bool commandIdentityCurrent(const CommandIdentity & identity) const;
  const CommandIdentity * deliveryQueryIdentity(std::uint32_t token) const;
  void checkRequestTimeouts(Clock::time_point current = Clock::now());
  void clearRequests();
  void armReply(canid_t id, const can_frame & frame);
  void finishRoi(BridgeRoiResult result = {}, std::uint64_t request_id = 0);
  rclcpp::Time now() const {return node_.now();}
  rclcpp::Logger get_logger() const {return node_.get_logger();}
  rclcpp::Clock::SharedPtr get_clock() {return node_.get_clock();}

  // ---- 会话、队列与服务身份 ----
  rclcpp::Node & node_;
  std::string can_interface_;
  std::unique_ptr<CanTransport> can_;
  int open_index_{0};
  Clock::duration tx_queue_timeout_, request_timeout_;
  BridgeState state_;
  CanTxQueue queue_;
  CanPeriodicSchedule periodic_schedule_;
  Clock::time_point next_open_;
  BridgeMissionWatchdog mission_watchdog_;
  std::uint64_t tx_frames_{0}, rx_frames_{0}, bus_errors_{0}, dropped_frames_{0};
  std::string last_error_;
  // ROI 与命令各一等待槽，可同时在途；四个回复快照独立于服务等待。
  // roi_confirmation_ 是本机通过的 ROI，匹配 Mission 身份后才授权转向；turn_ 标识已入队的授权。
  // 各命令缓存受理身份，共用192发送槽；1/2未缓存成功可在本趟后续阶段再询问Mission。
  // 查询只补回执，不能新建运动或续期；205恢复始终由Mission判断当前资格。
  // roi_attempted_ 防止重复到达把本机失败改成成功。
  BridgeServiceRequest roi_request_, command_request_;
  BridgeReplySlots replies_;
  struct TurnIdentity
  {
    std::uint64_t roi_request_id;
    BridgeDestination destination;
    std::uint64_t cycle_generation;
  };
  std::optional<TurnIdentity> turn_;
  std::optional<TurnIdentity> roi_confirmation_;
  std::uint64_t roi_cycle_generation_{0};
  bool roi_attempted_{false};
  // 205立即撤销本轮运动授权，跨CAN重连保留；新search解除保持后才清除。
  bool route_aborted_{false};
  struct CommandIdentity
  {
    std::uint8_t command;
    std::uint64_t expected_generation;
    std::uint64_t roi_request_id;
    BridgeDestination destination;
    BridgeCyclePhase origin_phase;
    bool delivery_query{false};
  };
  std::optional<CommandIdentity> pending_command_identity_, command_reply_identity_;
  std::array<std::optional<CommandIdentity>, 4> accepted_commands_{};
  // 最近已发起206保留不确定结果的完整身份；最近成功另存命令4缓存，均跨205/socket重连。
  std::optional<CommandIdentity> last_delivery_request_;
  std::uint32_t pending_command_token_{0};
  Clock::time_point next_turn_;
  // 空值表示尚无本轮结果；判定失败重放使用语义，不能反解 CAN 字节。
  std::optional<bool> roi_passed_;
  Ports ports_;
};
