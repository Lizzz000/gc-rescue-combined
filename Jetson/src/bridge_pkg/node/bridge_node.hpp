#pragma once

// ---- 桥接节点 ----
// 职责：ROS 参数、话题和服务装配。
// Copyright 2026 li
// SPDX-License-Identifier: Apache-2.0

#include <chrono>
#include <optional>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>
#include <interface_pkg/msg/safety_target.hpp>
#include <interface_pkg/msg/task_state.hpp>
#include <interface_pkg/msg/mission_status.hpp>
#include <cstdint>
#include <exception>
#include <string>
#include <string_view>
#include <interface_pkg/msg/bridge_status.hpp>
#include <interface_pkg/msg/roi_check.hpp>
#include <interface_pkg/srv/check_roi.hpp>
#include <interface_pkg/srv/register_pickup_route.hpp>
#include <interface_pkg/srv/mission_command.hpp>


#include "transport/transport.hpp"
#include "state/bridge_transactions.hpp"

class BridgeRuntime;
namespace s288 { class Session; }

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
  rclcpp::Subscription<interface_pkg::msg::TaskState>::SharedPtr task_sub_;
  rclcpp::Subscription<interface_pkg::msg::MissionStatus>::SharedPtr mission_sub_;
  rclcpp::TimerBase::SharedPtr io_timer_, status_timer_;
  std::unique_ptr<BridgeRuntime> runtime_;
};

// ---- BridgeRuntime：CAN 会话与异步握手 ----
// 所有入口共用节点默认互斥回调组，不创建执行器。自动夹爪在进程内用工作线程，不得阻塞 io()。
// 节点必须比运行上下文存活更久。
// 销毁前停止执行器访问。发送队列保留机会，事务模块保存载荷和应答资格。
class BridgeRuntime : private BridgeTransactions::Ports
{
public:
  using Clock = std::chrono::steady_clock;
  struct Config
  {
    std::string can_interface;
    // 两者都是单调期限，但前者限制发送机会寿命，后者限制服务等待，不能合成一个超时。
    Clock::duration tx_queue_timeout, request_timeout;
    // 默认 500 ms，与原硬编码相同；生产路径由 yaml bridge.mission_watchdog_timeout_s 传入。
    Clock::duration mission_watchdog_timeout{std::chrono::milliseconds(500)};
    struct GripperConfig
    {
      std::string device;
      std::int64_t id{0};
      double kp{0}, kd{0}, closed_deg{0}, open_deg{0}, move_s{0};
        double ramp_fraction{0};
        double open_hold_max_nm{0}, output_tolerance_deg{1.0};
        double open_trim_deg{0}, closed_trim_deg{0};
    } gripper;
    BridgeStateConfig state;
  };
  struct Ports
  {
    rclcpp::Publisher<interface_pkg::msg::BridgeStatus>::SharedPtr status_pub;
    rclcpp::Client<interface_pkg::srv::CheckRoi>::SharedPtr roi_client;
    rclcpp::Client<interface_pkg::srv::MissionCommand>::SharedPtr command_client;
    rclcpp::Client<interface_pkg::srv::RegisterPickupRoute>::SharedPtr pickup_client;
  };
  BridgeRuntime(
    rclcpp::Node & node, Config config, Ports ports, std::unique_ptr<CanTransport> transport);
  ~BridgeRuntime() override;
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
  void taskCallback(interface_pkg::msg::TaskState::ConstSharedPtr msg);
  void missionCallback(interface_pkg::msg::MissionStatus::ConstSharedPtr msg);

private:
  friend struct BridgeRuntimeTestAccess;
  // socket 准入与有界收发；打开或读写失败时本轮不再提交。
  void openSocket();
  void closeSocket(const std::string & reason);
  bool ensureSocketReady();
  bool readAvailableFrames();
  void flushTxQueue();
  void handleIoFailure(const std::exception & error);
  // ROS 输入只入队。周期载荷在写 socket 前由 currentFrame 重建。
  can_frame currentFrame(canid_t id) const;
  bool handleCommand(const can_frame & frame);
  void missionRestarted();
  void handleArrivalPoint(std::uint8_t kind, std::uint32_t request_id, std::uint32_t source_frame);
  void pollGripper();

  BridgeSourceTime sourceNow() const override {return {now().nanoseconds()};}
  Clock::time_point monotonicNow() const override {return Clock::now();}
  bool socketOpen() const override {return can_->socketOpen();}
  bool serviceReady(BridgeTransactions::Service service) override;
  std::int64_t sendRoi(std::uint64_t generation) override;
  std::int64_t sendPickupRoute(std::uint64_t generation, const std::string & class_id,
    std::uint32_t grasp_token) override;
  std::int64_t sendCommand(const BridgeTransactions::CommandRequest & command) override;
  void cancelService(BridgeTransactions::Service service, std::int64_t request_id) override;
  void queueReply(canid_t id) override
  {
    if (can_->socketOpen()) {queue_.queue(id, Clock::now());}
  }
  void dropReply(canid_t id) override {dropped_frames_ += queue_.drop(id);}
  void dropRequest() override {++dropped_frames_;}
  void setRequestError(std::string_view message) override {last_error_ = message;}
  void warnRequest(const char * message) override;
  void onRoiReply(
    std::uint64_t generation, rclcpp::Client<interface_pkg::srv::CheckRoi>::SharedFuture result);
  void onCommandReply(
    const BridgeTransactions::CommandRequest & command,
    rclcpp::Client<interface_pkg::srv::MissionCommand>::SharedFuture result);

  rclcpp::Time now() const {return node_.now();}
  rclcpp::Logger get_logger() const {return node_.get_logger();}
  rclcpp::Clock::SharedPtr get_clock() {return node_.get_clock();}

  // ---- 会话、队列与服务身份 ----
  rclcpp::Node & node_;
  std::string can_interface_;
  std::unique_ptr<CanTransport> can_;
  Clock::duration tx_queue_timeout_;
  BridgeState state_;
  CanTxQueue queue_;
  CanPeriodicSchedule periodic_schedule_;
  Clock::time_point next_open_;
  BridgeMissionWatchdog mission_watchdog_;
  static constexpr auto motion_lease_break_time_ = std::chrono::milliseconds(300);
  std::optional<Clock::time_point> motion_lease_suspended_until_;
  Config::GripperConfig gripper_config_;
  ArrivalPointSession arrival_point_;
  std::unique_ptr<s288::Session> gripper_;
  std::uint64_t tx_frames_{0}, rx_frames_{0}, dropped_frames_{0};
  std::string last_error_;
  Ports ports_;
  BridgeTransactions transactions_;
};
