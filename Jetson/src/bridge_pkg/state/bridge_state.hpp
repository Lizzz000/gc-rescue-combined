#pragma once

// ---- 桥接状态 ----
// 职责：请求、队列和货物复核，不打开设备。
// Copyright 2026 li
// SPDX-License-Identifier: Apache-2.0

#include <chrono>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>
#include <interface_pkg/app/core.hpp>

// 明确区分源时间与本机单调时间，供 Bridge 状态和请求组件使用。

struct BridgeSourceTime
{
  // 来自 ROS 消息 Header，与节点的 ROS now 比较；不能与 steady_clock 做差。
  std::int64_t ns{0};
};

struct BridgeMonotonicTime
{
  // 本机 steady_clock，专用于请求、队列及到达后的等待时间，不受源时钟回拨影响。
  std::chrono::steady_clock::time_point value;
};

// 由任务状态心跳续期；单调时钟避免系统时间变化影响失联判定。
class BridgeMissionWatchdog
{
public:
  void received(BridgeMonotonicTime now) {last_ = now.value;}
  bool alive(BridgeMonotonicTime now) const
  {
    return last_ && now.value >= *last_ && now.value - *last_ < std::chrono::milliseconds(500);
  }

private:
  std::optional<std::chrono::steady_clock::time_point> last_;
};
// 集中 Bridge 输入缓存、阶段／任务门控及双时钟新鲜度，不持有 ROS 消息或 CAN 队列。

// 数值与 MissionStatus.msg 相同。Bridge 核心不包含该消息头，所以在此单独声明。
enum class BridgeDestination : std::uint8_t {None = 0, Left = 1, Right = 2};

// ROI 接收边界的语义结果；保留图像有效位，独立复核货物后才允许通过及目的侧。
struct BridgeRoiResult
{
  bool valid{false}, passed{false};
  BridgeDestination destination{BridgeDestination::None};
  static BridgeRoiResult checked(bool valid, bool passed, const cargo::Counts & counts);
};

enum class BridgeOperation : std::uint8_t {Idle = 0, Checking = 1, Switching = 2};
enum class BridgeOutcome : std::uint8_t
{
  None = 0, Pending = 1, Success = 2, Rejected = 3, Timeout = 4,
  RoiRejected = 5, RoiInvalid = 6
};
enum class BridgeTransportPhase : std::uint8_t {First = 0, Later = 1};
// 0 到 8 必须与 MissionState::CyclePhase、MissionStatus.msg 的 CYCLE_* 相同。
// 增加阶段时三处一起改。这里只复制数值，坐标门仍由 SearchingTarget 和 NavigatingSafety 决定。
enum class BridgeCyclePhase : std::uint8_t
{
  SearchingTarget = 0, CheckingRoi = 1, WaitingTurn = 2, SwitchingField = 3,
  NavigatingSafety = 4, WaitingTransfer = 5, SwitchingSeg = 6, RoiFailed = 7, Fault = 8
};

struct BridgeTaskState
{
  BridgeSourceTime stamp;
  std::string task;
  std::uint64_t epoch{0};
};

struct BridgeMissionState
{
  BridgeSourceTime stamp;
  BridgeDestination destination{BridgeDestination::None};
  BridgeOperation operation{BridgeOperation::Idle};
  BridgeOutcome outcome{BridgeOutcome::None};
  BridgeTransportPhase transport_phase{BridgeTransportPhase::First};
  std::uint64_t roi_request_id{0};
  BridgeCyclePhase cycle_phase{BridgeCyclePhase::SearchingTarget};
  std::uint64_t cycle_generation{0};
};

// 此处是纯核心开发默认值；生产节点在装配前用全部必填参数覆盖，不作为配置缺项回退。
struct BridgeStateConfig
{
  std::int64_t image_width{1920}, image_height{1080};
  double input_timeout_s{1.0}, velocity_timeout_s{0.30};
  // 安全区中心保持：最近一次有效 field 中心在本机到达后该秒数内持续可发，期间的无效帧不撤销它；
  // 须在 (0, input_timeout_s]，且 safety_hold_s + 1/field_max_fps < STM32 中心接收期限 0.25 s，
  // 保证两个新源帧之间的间隔不触发 STM32 重置；源帧龄仍受 input_timeout_s 约束。
  double safety_hold_s{0.20};
  // field 帧率上限（vision.detect.field_max_fps），只用于上面的入口校验，须 > 0。
  double field_max_fps{100.0};
  std::vector<std::string> target_classes{"0", "2", "3"};
  std::string first_class{"0"};
  // 类别使用无前导零的非负十进制编号，重复类别仍允许；BridgeState 构造时统一校验。
};

struct BridgeStateUpdate
{
  // accepted 表示接收状态消息；reset_stream 通知节点撤周期帧，不要求同时撤一次性应答。
  bool accepted{false}, reset_stream{false};
};

struct BridgeTargetInput
{
  BridgeSourceTime stamp;
  std::string frame;
  // 仅原输入恰好有一个检测时填写；不改写假设、ID 或类别串。
  std::optional<RecognitionDetection> detection;
};

struct BridgeSafetyInput
{
  BridgeSourceTime stamp;
  std::string frame;
  bool valid{false};
  std::uint8_t side{0};
  double x{0}, y{0};
};

struct BridgeCenterOutput
{
  // 坐标已乘 10000 量化；valid=false 时成员可保留旧值，协议编码器负责将无效帧载荷清零。
  bool valid{false};
  std::int16_t x{0}, y{0};
  std::uint32_t source_frame{0};  // ROS 源时间的低 24 位毫秒。
};

struct BridgeVelocityOutput
{
  // 线速度和角速度均乘 1000 量化；是否有效与分量恰好为零是不同含义。
  bool valid{false};
  std::int16_t x{0}, y{0}, yaw{0};
};

// 纯值状态不自行读时钟、开线程或发送 CAN；调用方负责串行更新及输出资格检查。
class BridgeState
{
public:
  /**
   * @brief 在构造入口校验启动配置，然后建立空缓存。
   * @param[in] config 图像尺寸为 1..INT_MAX 像素，期限为 (0,3600] 秒，类别为规范非负编号。
   * @throws std::invalid_argument 配置非法，对象未构造，不保存输入。
   */
  explicit BridgeState(BridgeStateConfig config = {});
  // 链路恢复清理中心和速度，接收截止只前移；保留任务、运输阶段及到达保持。
  void resetInputs(BridgeSourceTime current);
  // 被接受的 TaskState 清中心：完全重复忽略，同模式但新源时间／epoch 仍接收。
  // task 本身不能解除到达保持；还须 Mission 进入新的可移动阶段，epoch 不要求递增。
  BridgeStateUpdate task(const BridgeTaskState & input, BridgeSourceTime current);
  // 业务状态变化清中心，内容相同的较新心跳只更新快照，不反复抬高截止时间。
  BridgeStateUpdate mission(const BridgeMissionState & input, BridgeSourceTime current);
  // 接受新源帧后即覆盖旧中心；单目标、类别和框校验决定这一帧的有效位。
  bool target(
    const BridgeTargetInput & input, BridgeSourceTime current,
    BridgeMonotonicTime arrival, bool socket_open);
  // 输入中心已归一化；只接收 field 帧，目的侧匹配在输出时检查。有效帧刷新保持中心，
  // 无效帧只推进源时间水位，不撤销保持；frame_id 改变时撤销。
  bool safety(
    const BridgeSafetyInput & input, BridgeSourceTime current,
    BridgeMonotonicTime arrival, bool socket_open);
  // Twist 无源时间，只按单调到达龄过期；合法零速度仍为有效输入。
  bool velocity(double x, double y, double yaw, BridgeMonotonicTime arrival, bool socket_open);
  // 读取时再次检查任务、任务忙碌状态及两种帧龄，防止排队后才过期的输入被发送。
  BridgeCenterOutput targetOutput(BridgeSourceTime current, BridgeMonotonicTime now) const;
  BridgeCenterOutput safetyOutput(BridgeSourceTime current, BridgeMonotonicTime now) const;
  BridgeVelocityOutput velocityOutput(BridgeMonotonicTime now) const;
  // 本地到达立即闭锁；同代次心跳不能解锁。恢复需新授权代次、可移动终态、匹配模式及后续新帧。
  // until_search用于205放弃运输，迟到field提交不能解除；等待新的SearchingTarget。
  void holdCoordinates(BridgeSourceTime current, bool until_search = false);
  bool coordinatesHeld() const;
  bool locallyHeld() const {return coordinates_held_;}
  std::optional<BridgeCyclePhase> cyclePhase() const;
  const std::optional<BridgeMissionState> & missionState() const {return mission_;}
  bool waitingTurn() const;
  BridgeSourceTime cutoff() const {return cutoff_;}

private:
  struct Center
  {
    bool valid{false}, have_frame{false};
    std::int16_t x{0}, y{0};
    BridgeSourceTime stamp;
    std::string frame;
    BridgeMonotonicTime arrival;
    std::uint8_t side{0};
  };
  struct Velocity
  {
    bool valid{false};
    std::int16_t x{0}, y{0}, yaw{0};
    BridgeMonotonicTime arrival;
  };
  bool sourceStamp(BridgeSourceTime stamp, BridgeSourceTime current, const Center & center) const;
  bool fresh(const Center & center, BridgeSourceTime current, BridgeMonotonicTime now) const;
  bool held(BridgeSourceTime current, BridgeMonotonicTime now) const;
  bool idle() const;
  bool movingPhase() const;
  void releaseCoordinates(BridgeSourceTime current);
  BridgeStateConfig config_;
  BridgeSourceTime cutoff_;
  // safety_ 是最近接收的 field 帧（源时间水位与 frame_id），safety_hold_ 是最近一次有效中心。
  Center target_, safety_, safety_hold_;
  Velocity velocity_;
  std::optional<BridgeTaskState> task_;
  std::optional<BridgeMissionState> mission_;
  bool coordinates_held_{false};
  bool held_until_search_{false};
  std::uint64_t held_generation_{0};
};

// 管理单个 Bridge 服务请求的代次、编号、命令回显及单调截止，节点执行 ROS 撤单和应答。

class BridgeServiceRequest
{
public:
  struct Info
  {
    // generation 是本地回调隔离标记；request_id 是 rclcpp 的 pending 撤单编号。
    // CAN 协议没有这两个编号，不能将它们解释为下位机事务序号。
    std::uint64_t generation{0};
    std::int64_t request_id{0};
    std::uint8_t command{0};
  };
  // 每类服务仅有一个等待槽；忙时不刷新截止，下次合法 begin 才生成新代次。
  std::optional<std::uint64_t> begin(
    BridgeMonotonicTime now, std::chrono::steady_clock::duration timeout,
    std::uint8_t command = 0);
  // 异步发送返回后绑定 pending 编号，旧代次不得覆盖当前撤单上下文。
  bool bind(std::uint64_t generation, std::int64_t request_id);
  // 只有同代且未到截止的回调能消费等待；迟到响应由 expire 统一失败收尾。
  bool complete(std::uint64_t generation, BridgeMonotonicTime now);
  // 返回待撤销的 ROS 请求元数据；恰好到截止也算超时，调用方负责执行失败应答。
  std::optional<Info> expire(BridgeMonotonicTime now);
  // 断线取走当前撤单上下文，后续旧回调失效；重复取消为空操作。
  std::optional<Info> cancel();
  bool waiting() const {return waiting_;}
  std::uint64_t generation() const {return info_.generation;}
  std::uint8_t command() const {return info_.command;}

private:
  bool waiting_{false};
  Info info_;
  BridgeMonotonicTime deadline_;
};
