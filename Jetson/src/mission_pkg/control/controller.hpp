#pragma once

// 阶段、等待和运输规则。不读时钟、不创建线程。
// 先看 CyclePhase 的九个阶段，再看 requestRoi 和 requestCommand。
// Copyright 2026 li
// SPDX-License-Identifier: Apache-2.0

#include <cstdint>
#include <string>
#include <chrono>
#include <optional>
#include <interface_pkg/app/core.hpp>
#include "roi/roi.hpp"

// 枚举值与 MissionStatus.msg 对齐；核心不依赖 ROS，节点负责转换。
struct MissionState
{
  enum class Destination : std::uint8_t {None = 0, Left = 1, Right = 2};
  enum class Operation : std::uint8_t {Idle = 0, Checking = 1, Switching = 2};
  enum class Outcome : std::uint8_t
  {None = 0, Pending = 1, Success = 2, Rejected = 3, Timeout = 4,
    RoiRejected = 5, RoiInvalid = 6};
  enum class Transport : std::uint8_t {First = 0, Later = 1};
  // 物理握手阶段独立于识别模式和服务等待。只有 SearchingTarget / NavigatingSafety
  // 允许对应坐标；WaitingTurn / WaitingTransfer 等待 STM32，不受服务超时控制。
  enum class CyclePhase : std::uint8_t
  {
    SearchingTarget = 0,  // 寻找目标，允许 arrived 或显式重找。
    CheckingRoi = 1,      // 等待原始 ROI；候选尚未提交时仍停坐标。
    WaitingTurn = 2,      // ROI 已提交；只等去程转向完成，或显式放弃路由。
    SwitchingField = 3,   // 等待 field 服务与实际阶段双确认。
    NavigatingSafety = 4, // 按已提交目的区导航，等待安全区到达。
    WaitingTransfer = 5,  // 盲推、送达确认和返程共用此态；由 StartSearch 离开。
    SwitchingSeg = 6,     // seg 双确认成功后清目的区和 ROI 编号，恢复寻找。
    RoiFailed = 7,        // ROI 拒绝、无效或超时，等待显式重找。
    Fault = 8            // 启动初态或运行故障；仅初态可由首个合法 seg 解除。
  };

  // 检测端首个合法 seg 状态到来前保持故障门控，不发布可行驶坐标。
  CyclePhase cycle_phase{CyclePhase::Fault};
  // 每次受理 field/seg 切换递增；同模式重找也能使下游撤销旧坐标，不是 CAN 事务号。
  std::uint64_t cycle_generation{0};
  // 当前已提交的路由；检查进行中、拒绝或切换失败时可为空。
  Destination destination{Destination::None};
  Operation operation{Operation::Idle};
  Outcome outcome{Outcome::None};
  // 首绿实际送达确认后 First → Later；重找不回滚，重建控制器才重置。
  Transport transport_phase{Transport::First};
  // Target 给出的检查编号，不是 ROS 客户端用于移除 pending request 的编号。
  std::uint64_t roi_request_id{0};
  std::string reason;
};

// 阶段消息的 ROS 源时间与等待用的 steady_clock::time_point 分开，不能相互比较。
struct MissionSourceTime
{
  // 单位 ns，来自已由 ROS 适配层校验的源 header；核心只用它排序阶段事件。
  std::int64_t nanoseconds{0};
};

struct MissionTaskState
{
  // 三项共同确定会话；epoch 不要求单调递增，源时间未变时换 task／epoch 仍算变化。
  // task 保留检测端原字符串；只有精确的 seg 与 field 参与对应业务分支。
  std::string task;
  std::uint64_t epoch{0};
  MissionSourceTime source_time;
};

// 一次本机等待的身份与单调截止；两类等待的互斥由 MissionController 保证。
struct PendingOperation
{
  enum class Kind {None, RoiCheck, AutomaticSwitch};
  using TimePoint = std::chrono::steady_clock::time_point;

  Kind kind{Kind::None};
  // request_id 只供节点撤销本机 future；generation 才用来辨认回调属于哪次等待。
  std::int64_t request_id{0};
  std::uint64_t generation{0};
  // steady_clock 截止，无 ROS 时间含义；只有 active() 时才参与超时判断。
  TimePoint deadline{};

  // 仅反映本机仍在等待，不说明请求在下游是否尚未执行。
  bool active() const {return kind != Kind::None;}

  // PENDING 发布后、发请求前才 begin；调用方已核对互斥、预期代次及秒制超时。
  std::uint64_t begin(Kind value, TimePoint now, double timeout_s)
  {
    kind = value;
    ++generation;
    deadline = now + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
      std::chrono::duration<double>(timeout_s));
    return generation;
  }

  bool accepts(Kind value, std::uint64_t callback_generation) const
  {
    // 类型与代次匹配还不等于未超时；控制器的 replyGate 会继续检查单调截止。
    return kind == value && generation == callback_generation;
  }

  // 恰好到截止即过期；这里只判断，不清等待。撤销 future 由控制器在超时路径上做。
  bool expired(TimePoint now) const {return active() && now >= deadline;}

  // 清理本机等待并使已捕获的回调令牌失效，调用方另行撤销 ROS pending request。
  void clear()
  {
    kind = Kind::None;
    request_id = 0;
    ++generation;
  }
};

// ROI 证据叠加运输限制后的候选；只有 accepted 为真才能建立目的区。
struct MissionTransportDecision
{
  // result 保留原始诊断并叠加运输数量限制；计数矛盾只拒绝内部 accepted，未必改写 passed。
  // accepted 为真才允许提交本趟路由；ROI 不记录送达，也不推进 First/Later。
  RoiRuleResult result;
  bool accepted{false};
  MissionState::Destination destination{MissionState::Destination::None};
};

// 单位为件。生产值由 MissionNode 从 cargo_rules.hpp 传入；{0,0} 是纯核心的宽松默认。
struct TransportPolicy
{
  // 0：首趟纯绿且至少一件；正数：绿色数量必须恰好等于该值。
  std::uint32_t first_green_count{0};
  // 绿加黑的数量上限，0 表示不限；单独运输的橙色不计入。
  std::uint32_t max_supply_count{0};
};

// 在 ROI 证据副本上做本趟准入，不提交路由，也不改送达事实。
// passed 已为真时不再复查颜色组合；后续运输里橙加绿仍按橙色去右区。
MissionTransportDecision applyTransportRules(
  const RoiRuleResult & result, MissionState::Transport transport,
  TransportPolicy policy = {});

// 与 MissionCommand.srv 对齐：1/2/3 应答表示受理，4 成功表示送达已记录。
// 错序或未知命令不改状态和截止；模式切换受理后仍须服务与实际阶段双确认。
enum class MissionCommand : std::uint8_t
{
  // ROI 授权后的去程转向完成；闭合与检查前后退已在请求 ROI 前结束。
  // 投送后的返程转向不发此命令，完成后用 StartSearch 开启下一轮。
  // 首次受理仅 WaitingTurn + 匹配非零完整 ROI 编号；受理后请求 field，目的区保持到双确认。
  TurnCompleted = 1,
  // 首次受理仅 NavigatingSafety + 匹配非零完整 ROI 编号；进入待搬运，不调用检测模式服务。
  SafetyArrived = 2,
  // 显式重找或恢复。无活动等待、无待提交候选的 CheckingRoi 也可恢复；完整准入见 requestCommand。
  // 受理后请求 seg，双确认成功才清目的区；首趟/后续进度保留。
  StartSearch = 3,
  // 新确认仅 WaitingTransfer + 匹配 ROI 编号；最近已确认编号可只读重试，不触发发布。
  DeliveryCompleted = 4
};

// 控制器向外的 ROS。节点实现它，控制器直接调用。
class MissionOut
{
public:
  virtual ~MissionOut() = default;
  virtual void publishStatus(const MissionState & state) = 0;
  virtual void publishRoi(
    const RoiRuleResult & result, std::uint64_t request_id, bool use_source_stamp) = 0;
  virtual void forwardTask() = 0;
  virtual bool roiServiceReady() = 0;
  virtual void sendRoiRequest(std::uint64_t generation) = 0;
  virtual bool detectorReady() = 0;
  virtual void sendSwitchRequest(bool field, std::uint64_t generation) = 0;
  virtual void cancel(PendingOperation::Kind kind, std::int64_t request_id) = 0;
};

struct MissionCommandResult
{
  bool accepted{false};
  // 本次发出了状态或切换。只读回执为 false，节点据此保留 Fault 里的送达收据。
  bool acted{false};
  std::string message;
};

// 运输状态机。调用方串行传入事件、源时间和单调时间。
// 物理握手、ROI 提交和实际识别阶段分别确认；旧代次不能修改新请求。
class MissionController
{
public:
  using Kind = PendingOperation::Kind;
  using TimePoint = PendingOperation::TimePoint;

  // 请求超时单位 s，纯核心默认 2.0，允许范围 (0, 3600]；生产值由节点明确传入。
  explicit MissionController(
    MissionOut & out, double request_timeout_s = 2.0, TransportPolicy policy = {});
  const MissionState & state() const {return state_;}
  // busy 仅反映两个已激活的服务等待；物理握手是否合法还必须检查 cycle_phase。
  bool busy() const;
  // 代次资格仅说明仍在等待；回调读取 future 前还须调用 replyGate 检查截止。
  bool accepts(Kind kind, std::uint64_t generation) const;
  // 记录客户端编号仅供取消本机 future，不改变下游已经受理的服务请求。
  void registerRequest(Kind kind, std::uint64_t generation, std::int64_t request_id);
  // PENDING 状态发布成功后，适配层在实际发请求前激活等待，核心不提前 begin。
  bool beginSwitchWait(Kind kind, std::uint64_t generation, TimePoint now);
  bool beginRoiWait(std::uint64_t generation, TimePoint now);

  // STM32 张开接近、闭合限位、后退并停车后请求；检查期间保持闭合。
  // 核心不观测机械位置。只在 SearchingTarget 受理，继续用当前 seg 的原始帧做 ROI。
  void requestRoi();
  // 转向/安全区到达/新送达须匹配非零完整 ROI 编号；StartSearch 不要求编号。
  // 1/2 在本趟对应后续阶段只读确认；Fault、重找后拒绝。
  // 4 的最近送达收据可跨 Fault/重找查询；所有查询均不刷新等待截止。
  MissionCommandResult requestCommand(
    MissionCommand command, std::uint64_t roi_request_id = 0);
  void commandFailed(const std::string & message);
  // 到达动作失败后：未 begin 的 CheckingRoi 退回寻找，RoiFailed/Fault 只补发快照。
  void arrivalInterrupted(const std::string & message);
  // 只跟随不早于当前会话的源阶段事件；切阶段时取消旧 ROI。
  void taskChanged(const MissionTaskState & task, TimePoint now);
  // false：节点不得 future.get()。超时收尾已经做完。
  bool replyGate(Kind kind, std::uint64_t generation, TimePoint now);
  void switchReply(
    Kind kind, std::uint64_t generation, bool success, const std::string & message);
  void switchCallbackFinished(Kind kind, std::uint64_t generation) noexcept;
  void roiReply(
    std::uint64_t generation, const RoiRuleResult & result, std::uint64_t request_id);
  // 应答和诊断都已发出时由节点调用。没发出去就不能提交路由。
  void commitRoiAfterReply();
  // 这次 publishRoi 是否带着待提交候选。到达拒绝不能把上一趟没发出的候选提交掉。
  bool roiPublishCommits() const {return publish_commits_roi_;}
  void roiCallbackFinished(std::uint64_t generation) noexcept;
  void requestFailed(Kind kind, std::uint64_t generation, const std::string & message);
  // 发布失败时补救一次再清令牌；补救再次抛出则清令牌后继续抛。
  void recoverRequest(Kind kind, std::uint64_t generation, const std::string & message);
  void checkTimeouts(TimePoint now);

private:
  struct RoiCommit;
  enum class TaskEvent {Older, Duplicate, NewSession};
  static TaskEvent classifyTaskEvent(
    const std::optional<MissionTaskState> & current, const MissionTaskState & next);
  PendingOperation & pending(Kind kind);
  const PendingOperation & pending(Kind kind) const;
  void armRoi(
    const RoiRuleResult & result, std::uint64_t request_id,
    std::optional<MissionState::Outcome> failure);
  void publishArmedRoi(bool use_source_stamp);
  void failRoi(
    const std::string & message,
    MissionState::Outcome outcome = MissionState::Outcome::RoiInvalid);
  void startAutomatic(bool field);
  // 状态已经发布。服务不可用就结束切换，不再绕回节点问一次。
  void continueSwitch();
  void finishAutomatic(MissionState::Outcome outcome, const std::string & message);
  void timeout(Kind kind);
  void recoverCommittedRoi();
  void failActiveRequest(Kind kind, std::uint64_t generation, const std::string & message);
  void cancelRoiForTaskChange(const MissionTaskState & task, TimePoint observed_at);
  void enterFault(const std::string & message);
  void observeExpectedTask(TimePoint now);
  void applyTaskChange(const MissionTaskState & task, TimePoint now);
  void roiResponseFailed(std::uint64_t generation);
  void applyRoiDecision(const RoiCommit & completion);
  void resumeDeferredTask();

  struct RoiCommit
  {
    MissionTransportDecision decision;
    std::uint64_t request_id{0};
    std::uint64_t generation{0};
    // 本机超时等失败在 ROI 结论发布后追加的结果状态，保留原来的两份状态快照。
    std::optional<MissionState::Outcome> failure_outcome;
  };
  struct RoiContext
  {
    std::optional<RoiCommit> pending_commit;
    // 只用于本次完成动作的异常恢复，不替代 PendingOperation 的等待资格。
    std::optional<std::uint64_t> callback_generation;
  };
  struct AutomaticSwitchContext
  {
    bool status_publication_pending{false};
    // 两个确认独立保存，可按任一顺序到达；任一确认都不延长原等待截止。
    bool service_accepted{false};
    bool expected_field{false};
    bool task_observed{false};
    std::optional<std::uint64_t> callback_generation;
  };
  struct DeferredTask
  {
    MissionTaskState task;
    TimePoint observed_at{};
  };

  // 对外业务结果、检测端实际 task_ 与请求等待是独立事实：
  // PENDING 可先于 begin，等待清理后也可能仍需本次回调的异常恢复资格。
  MissionOut & out_;
  MissionState state_;
  // 仅缓存最近一次已记录的完整送达编号，不依赖当前路由；发布失败和重找不能抹掉回执。
  // 零表示尚未确认。仅进程内有效，新一次送达替换；不提供跨重启或编号重用防重放。
  std::uint64_t delivered_roi_request_id_{0};
  TransportPolicy policy_;
  RoiContext roi_;
  AutomaticSwitchContext automatic_;
  std::optional<DeferredTask> deferred_task_;
  std::optional<MissionTaskState> task_;
  // 两个独立槽保留各自代次与取消编号；busy()/begin* 保证同一时刻只激活一个。
  PendingOperation roi_check_, automatic_switch_;
  double timeout_s_;
  // 延后的阶段事件正在套用，避免同一次收尾再包一层发布补救。
  bool in_deferred_task_{false};
  bool publish_commits_roi_{false};
};
