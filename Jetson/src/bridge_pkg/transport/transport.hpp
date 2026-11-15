#pragma once

#include "protocol/usb_protocol.hpp"

// ---- CAN 传输 ----
// 职责：USB 帧收发抽象，不含重连节奏。
// Copyright 2026 li
// SPDX-License-Identifier: Apache-2.0

#include <string>
#include <linux/can.h>
#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <stdexcept>

// BridgeNode 使用的收发抽象——打开／非阻塞读写／关闭，以及读写结果分类。
//       板间实现是 CdcLink；测试可注入进程内替身。

/// CAN 收发抽象：只表达“一个已绑定接口上的一次收发成败”，不含重连节奏、背压重试或计数，
/// 这些由 BridgeRuntime 管理。节点在同一默认互斥回调组内调用本接口，实现不额外拥有线程或锁。
class CanTransport
{
public:
  /// 非阻塞读的结果：FRAME 完整分帧；EMPTY 无数据；INTERRUPTED 信号中断；FAILED 故障。
  enum class ReadResult { FRAME, EMPTY, INTERRUPTED, FAILED };
  // 占用冲突/串口半包不能自动重试；Runtime清理后交入口报告并退出。
  struct FatalError : std::runtime_error {using std::runtime_error::runtime_error;};
  /// 非阻塞写的结果：SENT 整帧提交；BACKPRESSURE 队列满；INTERRUPTED 被信号中断；FAILED 其他错误。
  /// SENT 只表示内核接受本次提交，不表示对端收到报文或 STM32 已执行指令。
  enum class WriteResult { SENT, BACKPRESSURE, INTERRUPTED, FAILED };

  virtual ~CanTransport() = default;

  /// 打开并配置 USB 设备。失败返回 false 并写 error，且不保留打开状态。
  virtual bool open(const std::string & interface_name, std::string & error) = 0;
  /// 关闭；已关闭时什么也不做。不改变网络接口状态。
  virtual void close() = 0;
  virtual bool socketOpen() const = 0;
  /// 每次只做一次收发尝试；EINTR 等交给调用方在本周期的次数预算内重试。
  /// FRAME 只保证读到完整 can_frame；具体指令的 ID、DLC 和请求字节仍由协议层解析。
  virtual ReadResult read(can_frame & frame, std::string & error) = 0;
  virtual WriteResult write(const can_frame & frame, std::string & error) = 0;
};

// 两个坐标周期帧错开发送，每个 ID 的目标周期均为 10 ms。
class CanPeriodicSchedule
{
public:
  // 两个坐标 ID 各 10 ms 一轮，I/O 每 5 ms 推进一次。
  static constexpr auto kPeriodicIdPeriod = std::chrono::nanoseconds(10'000'000);
  static constexpr unsigned kPeriodicIdCount = 2;
  static constexpr auto interval = std::chrono::nanoseconds(
    (kPeriodicIdPeriod.count() + kPeriodicIdCount - 1) / kPeriodicIdCount);
  static_assert(interval.count() == 5000000, "periodic interval must stay 5 ms");

  // 业务状态失联时停止周期流；无目标、夹爪与转向阶段仍由调用方保持 enabled。
  std::optional<canid_t> next(bool enabled)
  {
    if (!enabled) {return std::nullopt;}
    constexpr canid_t ids[] = {
      CanProtocol::TARGET_ID, CanProtocol::SAFETY_ID};
    static_assert(sizeof(ids) / sizeof(ids[0]) == kPeriodicIdCount);
    const canid_t id = ids[index_];
    index_ = (index_ + 1U) % kPeriodicIdCount;
    return id;
  }

private:
  unsigned index_{0U};
};

// 五类回复各占至多一槽，按第一次入队顺序发送；周期坐标由独立调度器直接提交。
// 调用方串行访问，本类不加锁；合并同 ID 仅保留发送机会，不承诺逐条交付。
class CanTxQueue
{
public:
  using Clock = std::chrono::steady_clock;
  static constexpr std::size_t capacity = 5;

  // 同 ID 不延长最初 TTL，持续背压时也能淘汰；调用方只传固定发送 ID。
  void queue(canid_t id, Clock::time_point now)
  {
    for (std::size_t i = 0; i < size_; ++i) {
      if (entries_[i].id == id) {return;}
    }
    if (size_ == capacity) {throw std::logic_error("CAN queue capacity mismatch");}
    entries_[size_++] = {id, now};
  }

  // 入队时间递增且合并不改时间，过期项必定从队首连续出现；恰好到截止即淘汰。
  std::optional<canid_t> front(
    Clock::time_point now, Clock::duration timeout, std::uint64_t & drops)
  {
    while (size_ && now - entries_[0].queued_at >= timeout) {
      sent();
      ++drops;
    }
    if (!size_) {return std::nullopt;}
    return entries_[0].id;
  }

  // 移除队首；成功提交与过期淘汰共用此操作，重试不延长最初的有效期。
  void sent() noexcept
  {
    if (!size_) {return;}
    for (std::size_t i = 1; i < size_; ++i) {entries_[i - 1] = entries_[i];}
    --size_;
  }

  // 撤掉一个 ID 的发送机会，不影响其他 ID 的顺序或有效期。
  std::size_t drop(canid_t id) noexcept
  {
    std::size_t kept = 0;
    for (std::size_t i = 0; i < size_; ++i) {
      if (entries_[i].id != id) {entries_[kept++] = entries_[i];}
    }
    const auto removed = size_ - kept;
    size_ = kept;
    return removed;
  }

  // 断线清队后，旧应答失去发送机会；转向协议重试由 Runtime 独立重新准入。
  void reset() noexcept {size_ = 0;}
  std::size_t size() const noexcept {return size_;}

private:
  // 槽只记录 ID 与首次单调入队时间；回复载荷保存在独立快照槽。
  struct Entry {canid_t id{}; Clock::time_point queued_at{};};
  std::array<Entry, capacity> entries_{};
  std::size_t size_{0};
};

// 保存五类回复载荷快照；发送资格、过期与清队由 CanTxQueue 管理。

// 固定保存到达、ROI、命令应答、转向及夹爪结果；同 ID 覆盖不延长队列 TTL。
class BridgeReplySlots
{
public:
  // 保存已编码的完整帧，不改变 DLC、字节或发送资格。
  void store(canid_t id, const can_frame & frame);
  // 只在队列给出发送机会时读取；引用指向组件存储，后续 store 会覆写相应快照。
  const can_frame & frame(canid_t id) const;

private:
  static std::optional<std::size_t> index(canid_t id);
  std::array<can_frame, 5> frames_{};
};
