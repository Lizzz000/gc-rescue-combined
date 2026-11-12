#include "state/bridge_state.hpp"
#include "protocol/can_protocol.hpp"

// ---- 桥接状态实现 ----
// 职责：请求等待、队列和货物复核。
// Copyright 2026 li
// SPDX-License-Identifier: Apache-2.0
#include <interface_pkg/app/core.hpp>
#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

namespace
{
/**
 * @brief 在构造入口校验感知输入有效期限。
 * @param[in] input_timeout_s 感知输入有效期，单位秒，须有限且位于(0,3600]。
 * @throws std::invalid_argument 期限非法；不修改配置或运行状态。
 */
void validateTimeouts(double input_timeout_s)
{
  if (!std::isfinite(input_timeout_s) || input_timeout_s <= 0 || input_timeout_s > 3600) {
    throw std::invalid_argument("Bridge input timeout must be finite in (0, 3600]");
  }
}
}  // namespace

BridgeRoiResult BridgeRoiResult::checked(bool valid, bool passed, const cargo::Counts & counts)
{
  // 首趟限制由 Mission 判断；不能用回包时可能已变化的运输阶段重判。
  passed = valid && passed &&
    !cargo::combinationRejection(counts, cargo::kProductionCombination) &&
    cargo::withinSupplyLimit(counts, cargo::kProductionMaxSupplyCount);
  return {valid, passed, passed ?
    static_cast<BridgeDestination>(RecognitionValueValidation::cargoSide(counts.orange)) :
    BridgeDestination::None};
}

// 纯值缓存和输出资格。源时间和单调时间都由调用方传入，这里不读时钟、不写 CAN。
//
// 坐标能不能发出，看 targetOutput / safetyOutput，四项都要成立：
//   没有本地保持，操作是空闲，实际阶段匹配（目标要 seg，安全区要 field），
//   Mission 阶段也匹配（目标是寻找，安全区是前往安全区且结果成功、左右一致），帧还新鲜。
// holdCoordinates 清掉两路中心并记住当时的 cycle_generation。
// releaseCoordinates 要求新代次及可移动阶段；205 放弃后只允许新的寻找阶段解除。

/**
 * @brief 接收一次启动配置并建立空缓存；非法配置在本入口拒绝。
 * @param[in] config 图像尺寸为 1..INT_MAX 像素，期限为 (0,3600] 秒；类别须为规范非负编号。
 * @throws std::invalid_argument 配置非法，状态对象尚未构造完成，不保存输入缓存。
 */
BridgeState::BridgeState(BridgeStateConfig config)
: config_(std::move(config))
{
  if (config_.image_width <= 0 || config_.image_height <= 0 ||
    config_.image_width > std::numeric_limits<int>::max() ||
    config_.image_height > std::numeric_limits<int>::max())
  {
    throw std::invalid_argument("Bridge image dimensions must be 1..INT_MAX");
  }
  validateTimeouts(config_.input_timeout_s);
  if (!std::isfinite(config_.safety_hold_s) || config_.safety_hold_s <= 0 ||
    config_.safety_hold_s > config_.input_timeout_s || !std::isfinite(config_.field_max_fps) ||
    config_.field_max_fps <= 0 ||
    config_.safety_hold_s + 1.0 / config_.field_max_fps >= CanProtocol::STM32_CENTER_TIMEOUT_S)
  {
    throw std::invalid_argument(
      "Bridge safety hold must be in (0, input_timeout_s] and hold + 1/field_max_fps < 0.25 s");
  }
  if (config_.target_classes.empty() || std::any_of(
      config_.target_classes.begin(), config_.target_classes.end(), [](const std::string & class_id) {
        return class_id.empty() || RecognitionValueValidation::classId(class_id) != class_id;
      }))
  {
    throw std::invalid_argument("Bridge target classes must contain canonical non-negative IDs");
  }
  if (std::find(
      config_.target_classes.begin(), config_.target_classes.end(),
      config_.first_class) == config_.target_classes.end())
  {
    throw std::invalid_argument("Bridge first class must be one of target classes");
  }
}

void BridgeState::resetInputs(BridgeSourceTime current)
{
  // 断线／恢复重新建立感知基线；任务、运输阶段与到达保持不属于本次清理范围。
  target_ = Center{};
  safety_ = Center{};
  safety_hold_ = Center{};
  // ROS 源时钟回拨也不能重新放行会话切换前的旧帧；等待时钟追上原截止后再恢复中心。
  cutoff_.ns = std::max(cutoff_.ns, current.ns);
}

// 接受则 accepted=true，并 reset_stream：调用方撤掉旧的 0x180/0x181/0x182 机会。
// 完全重复、更旧、或源时间还在将来：两个标志都是假，队列不动。
// 同模式但源时间或 epoch 变了也算新阶段，中心清掉。任务字符串本身不解坐标保持。
BridgeStateUpdate BridgeState::task(const BridgeTaskState & input, BridgeSourceTime current)
{
  if (input.stamp.ns <= 0 || input.stamp.ns > current.ns ||
    (task_ && input.stamp.ns < task_->stamp.ns)) {return {};}
  if (task_ && task_->task == input.task && task_->epoch == input.epoch &&
    task_->stamp.ns == input.stamp.ns) {return {};}
  task_ = input;
  cutoff_.ns = std::max(cutoff_.ns, input.stamp.ns);
  target_ = Center{};
  safety_ = Center{};
  safety_hold_ = Center{};
  releaseCoordinates(current);
  return {true, true};
}

// 合法状态都 accepted。只有目的区、请求号、操作、结果、运输阶段、cycle 变了才 reset_stream。
// 同内容的心跳只更新快照，不抬高截止，也不清掉正在收的中心。
// 源时间相等仍比较字段，所以同一时刻可以先 PENDING 再 SUCCESS。
BridgeStateUpdate BridgeState::mission(
  const BridgeMissionState & input, BridgeSourceTime current)
{
  if (input.stamp.ns <= 0 || input.stamp.ns > current.ns ||
    input.destination > BridgeDestination::Right || input.operation > BridgeOperation::Switching ||
    input.outcome > BridgeOutcome::RoiInvalid ||
    input.transport_phase > BridgeTransportPhase::Later ||
    input.cycle_phase > BridgeCyclePhase::Fault ||
    (mission_ && input.stamp.ns < mission_->stamp.ns)) {return {};}
  // 源时刻相等仍比较业务字段，因此 PENDING 与 SUCCESS 可以在同刻按接收顺序生效。
  const bool changed = !mission_ || input.destination != mission_->destination ||
    input.roi_request_id != mission_->roi_request_id || input.operation != mission_->operation ||
    input.outcome != mission_->outcome || input.transport_phase != mission_->transport_phase ||
    input.cycle_phase != mission_->cycle_phase ||
    input.cycle_generation != mission_->cycle_generation;
  // 业务字段不变的状态心跳只更新 mission 快照，不反复清掉正在接收的中心或抬高 cutoff。
  if (changed) {
    target_ = Center{};
    safety_ = Center{};
    safety_hold_ = Center{};
    cutoff_.ns = std::max(cutoff_.ns, input.stamp.ns);
  }
  mission_ = input;
  releaseCoordinates(current);
  return {true, changed};
}

bool BridgeState::sourceStamp(
  BridgeSourceTime stamp, BridgeSourceTime current, const Center & center) const
{
  // cutoff 可取等号，但同一路已接收帧必须严格更新；无效载荷也占用其源时间水位。
  return stamp.ns > 0 && stamp.ns <= current.ns && stamp.ns >= cutoff_.ns &&
         current.ns - stamp.ns <= config_.input_timeout_s * 1e9 &&
         (!center.have_frame || stamp.ns > center.stamp.ns);
}

bool BridgeState::target(
  const BridgeTargetInput & input, BridgeSourceTime current,
  BridgeMonotonicTime arrival, bool socket_open)
{
  if (!socket_open || !task_ || task_->task != "seg" ||
    !sourceStamp(input.stamp, current, target_)) {return false;}
  Center center;
  // 时戳闸门通过后，即使检测无效也用本帧覆盖缓存，不能继续沿用上一帧的有效位置。
  center.have_frame = true;
  center.stamp = input.stamp;
  center.arrival = arrival;
  center.frame = input.frame;
  const bool changed_frame = target_.have_frame && target_.frame != center.frame;
  // frame_id 改变的首帧只建立新来源，中心无效；后续同来源的新帧才可能恢复有效。
  if (!changed_frame && input.detection) {
    const auto & detection = *input.detection;
    const auto & box = detection.box;
    if (!detection.id.empty() && detection.hypothesis &&
      RecognitionValueValidation::axisAlignedBox(box))
    {
      const auto & result = *detection.hypothesis;
      const auto class_id = RecognitionValueValidation::classId(result.class_id);
      const bool later = mission_ &&
        mission_->transport_phase == BridgeTransportPhase::Later;
      // 0x181 使用框的几何中心除以配置图像尺寸。选择节点比较的是框底边，两套参考点各自保留。
      const bool class_allowed = std::find(
        config_.target_classes.begin(), config_.target_classes.end(), class_id) !=
        config_.target_classes.end() && (later || class_id == config_.first_class);
      center.valid = RecognitionValueValidation::hypothesis(result) && class_allowed &&
        bridge_scale::fixedPoint(box.center_x / config_.image_width, 10000.0, center.x) &&
        bridge_scale::fixedPoint(box.center_y / config_.image_height, 10000.0, center.y);
    }
  }
  target_ = std::move(center);
  return true;
}

bool BridgeState::safety(
  const BridgeSafetyInput & input, BridgeSourceTime current,
  BridgeMonotonicTime arrival, bool socket_open)
{
  if (!socket_open || !task_ || task_->task != "field" ||
    !sourceStamp(input.stamp, current, safety_)) {return false;}
  Center center;
  center.have_frame = true;
  center.stamp = input.stamp;
  center.arrival = arrival;
  center.frame = input.frame;
  center.side = input.side;
  // SafetyTarget 已提供归一化中心，直接乘 10000；这里不再除图像宽高。
  const bool changed_frame = safety_.have_frame && safety_.frame != center.frame;
  center.valid = !changed_frame && input.valid &&
    (input.side == static_cast<std::uint8_t>(BridgeDestination::Left) ||
    input.side == static_cast<std::uint8_t>(BridgeDestination::Right)) &&
    bridge_scale::fixedPoint(input.x, 10000.0, center.x) &&
    bridge_scale::fixedPoint(input.y, 10000.0, center.y);
  if (center.valid) {safety_hold_ = center;} else if (changed_frame) {safety_hold_ = Center{};}
  safety_ = std::move(center);
  return true;
}

bool BridgeState::fresh(
  const Center & center, BridgeSourceTime current, BridgeMonotonicTime now) const
{
  // 源龄与本机到达龄都须不过期，边界取等号；源时钟停住也不能让旧中心一直有效。
  return center.have_frame && center.valid && current.ns >= center.stamp.ns &&
         current.ns - center.stamp.ns <= config_.input_timeout_s * 1e9 &&
         std::chrono::duration<double>(now.value - center.arrival.value).count() <=
         config_.input_timeout_s;
}

bool BridgeState::held(BridgeSourceTime current, BridgeMonotonicTime now) const
{
  // 保持中心与最近接收帧同源；源龄按 input_timeout_s，本机到达龄按 safety_hold_s，边界取等号。
  const auto & center = safety_hold_;
  return center.valid && center.frame == safety_.frame && current.ns >= center.stamp.ns &&
         current.ns - center.stamp.ns <= config_.input_timeout_s * 1e9 &&
         std::chrono::duration<double>(now.value - center.arrival.value).count() <=
         config_.safety_hold_s;
}

bool BridgeState::idle() const
{
  return mission_ && mission_->operation == BridgeOperation::Idle;
}

BridgeCenterOutput BridgeState::targetOutput(
  BridgeSourceTime current, BridgeMonotonicTime now) const
{
  // 0x181：寻找 + 实际 seg + 空闲 + 未保持 + 帧新鲜。缺一项 valid 就是假，坐标值仍可能留着旧数。
  // coordinatesHeld 已检查空闲与阶段/实际模式匹配，通过后 mission_ 必然存在。
  return {!coordinatesHeld() &&
    mission_->cycle_phase == BridgeCyclePhase::SearchingTarget &&
    fresh(target_, current, now), target_.x, target_.y,
    static_cast<std::uint32_t>((target_.stamp.ns / 1000000) & 0xffffff)};
}

BridgeCenterOutput BridgeState::safetyOutput(
  BridgeSourceTime current, BridgeMonotonicTime now) const
{
  // 场地区域必须与 Mission 已成功提交的目的区同侧；仅有一个有效 SafetyTarget 还不够。
  const bool allowed = !coordinatesHeld() &&
    mission_->cycle_phase == BridgeCyclePhase::NavigatingSafety &&
    mission_->outcome == BridgeOutcome::Success &&
    mission_->destination != BridgeDestination::None &&
    static_cast<std::uint8_t>(mission_->destination) == safety_hold_.side &&
    held(current, now);
  return {allowed, safety_hold_.x, safety_hold_.y,
    static_cast<std::uint32_t>((safety_hold_.stamp.ns / 1000000) & 0xffffff)};
}

void BridgeState::holdCoordinates(BridgeSourceTime current, bool until_search)
{
  if (until_search && !held_until_search_) {
    held_until_search_ = true;
    held_generation_ = mission_ ? mission_->cycle_generation : 0;
  }
  // 重复到达不能刷新本次闭锁的起点，否则会吞掉已经观察到的中间阶段。
  if (coordinates_held_) {return;}
  coordinates_held_ = true;
  held_generation_ = mission_ ? mission_->cycle_generation : 0;
  target_ = Center{};
  safety_ = Center{};
  safety_hold_ = Center{};
  cutoff_.ns = std::max(cutoff_.ns, current.ns);
}

std::optional<BridgeCyclePhase> BridgeState::cyclePhase() const
{
  return mission_ ? std::optional<BridgeCyclePhase>{mission_->cycle_phase} : std::nullopt;
}

bool BridgeState::movingPhase() const
{
  return mission_ && task_ && idle() &&
         ((mission_->cycle_phase == BridgeCyclePhase::SearchingTarget && task_->task == "seg") ||
         (mission_->cycle_phase == BridgeCyclePhase::NavigatingSafety && task_->task == "field"));
}

bool BridgeState::coordinatesHeld() const
{
  return coordinates_held_ || !movingPhase();
}

void BridgeState::releaseCoordinates(BridgeSourceTime current)
{
  if (!coordinates_held_ || !movingPhase() ||
    mission_->cycle_generation == held_generation_ ||
    (held_until_search_ && mission_->cycle_phase != BridgeCyclePhase::SearchingTarget)) {return;}
  coordinates_held_ = false;
  held_until_search_ = false;
  // 新授权代次不依赖订阅者收到中间态；同代心跳不能解锁，等待期旧缓存不能复活。
  target_ = Center{};
  safety_ = Center{};
  safety_hold_ = Center{};
  cutoff_.ns = std::max(cutoff_.ns, current.ns);
}

bool BridgeState::waitingTurn() const
{
  return mission_ && idle() && mission_->cycle_phase == BridgeCyclePhase::WaitingTurn &&
         mission_->outcome == BridgeOutcome::Success && mission_->roi_request_id != 0 &&
         mission_->destination != BridgeDestination::None;
}

// 实现 Bridge 包内服务等待生命周期，不持有 ROS 客户端或读取时钟。
// 截止与回调完成共用单调时间；本组件只返回收尾信息，ROS 撤单与 CAN 回复由运行层执行。

std::optional<std::uint64_t> BridgeServiceRequest::begin(
  BridgeMonotonicTime now, std::chrono::steady_clock::duration timeout, std::uint8_t command)
{
  // 忙时既不产生新代次，也不延长原截止；两类服务由节点各自持有一个实例。
  if (waiting_) {return std::nullopt;}
  waiting_ = true;
  info_.command = command;
  ++info_.generation;
  deadline_.value = now.value + timeout;
  return info_.generation;
}

bool BridgeServiceRequest::bind(std::uint64_t generation, std::int64_t request_id)
{
  // ROS 请求编号只绑定到仍在等待的同一代，不能由旧回调覆写新请求的撤单编号。
  if (!waiting_ || info_.generation != generation) {return false;}
  info_.request_id = request_id;
  return true;
}

bool BridgeServiceRequest::complete(std::uint64_t generation, BridgeMonotonicTime now)
{
  // 超期只拒绝完成，等待状态留给 expire 收尾；不能在迟到回调里当作正常成功消费。
  if (!waiting_ || info_.generation != generation || now.value >= deadline_.value) {return false;}
  waiting_ = false;
  return true;
}

std::optional<BridgeServiceRequest::Info> BridgeServiceRequest::expire(BridgeMonotonicTime now)
{
  if (!waiting_ || now.value < deadline_.value) {return std::nullopt;}
  waiting_ = false;
  return info_;
}

std::optional<BridgeServiceRequest::Info> BridgeServiceRequest::cancel()
{
  // 取消只取出本机撤单上下文；waiting=false 使旧回调失效，下次 begin 再产生新代次。
  if (!waiting_) {return std::nullopt;}
  waiting_ = false;
  return info_;
}
