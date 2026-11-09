#pragma once
// 图传自己的待发队列和同帧叠加缓存。检测节点的 LatestSlot 不替代这里的顺序队列。

// ---- 原 stream/stream.hpp 的待发队列与叠层同步（只有 stream_node 使用） ----
// Copyright 2026 li
// SPDX-License-Identifier: Apache-2.0
// 有界顺序队列：按到达顺序保留少量待处理帧，编码当前帧时不丢掉下一帧识别结果。
// 容量满时丢掉最旧的等待帧；正在被 take() 取走的帧不在队列里，不会被后来的帧替换。


#include <algorithm>
#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <utility>

template<typename T>
class PendingFrames {
public:
  static constexpr std::size_t kCapacity = 3;

  void set(std::shared_ptr<T> value)
  {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (stopped_) {return;}
      if (items_.size() == kCapacity) {
        items_.pop_front();
        ++overwritten_;
      }
      items_.push_back(std::move(value));
    }
    cv_.notify_one();
  }

  std::shared_ptr<T> take()
  {
    std::unique_lock<std::mutex> lock(mutex_);
    cv_.wait(lock, [this] {return !items_.empty() || stopped_;});
    if (items_.empty()) {return nullptr;}
    auto value = std::move(items_.front());
    items_.pop_front();
    return value;
  }

  std::shared_ptr<T> tryTake()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (items_.empty()) {return nullptr;}
    auto value = std::move(items_.front());
    items_.pop_front();
    return value;
  }

  // 换阶段只清等待帧，不停止队列；正在合成的旧帧由外层 generation 最终复核拦截。
  void clear()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    items_.clear();
  }

  // 停止不可逆：清空并唤醒 take()，之后 set() 不再接纳帧；重新启动须构造新队列对象。
  void stop()
  {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      stopped_ = true;
      items_.clear();
    }
    cv_.notify_all();
  }

  std::uint64_t overwritten() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return overwritten_;
  }

  bool empty() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return items_.empty();
  }

private:
  mutable std::mutex mutex_;
  std::condition_variable cv_;
  std::deque<std::shared_ptr<T>> items_;
  bool stopped_{false};
  std::uint64_t overwritten_{0};
};

// ---- 图传阶段、源帧准入与同帧叠加缓存 ----
// 只依赖标准库，载荷由 StreamNode 提供；三帧原图队列与八组叠加缓存承担不同职责，
// 节点回调经 Runtime 交接，不互相替代容量或时间游标。

using OverlayFrameKey = std::pair<std::int64_t, std::string>;
// 匹配键同时包含源帧纳秒时间戳与 frame_id，时间相同但来源不同的图层不会混用。

enum class OverlayLayer : unsigned { Detections = 1, Mask = 2, Field = 4 };

struct OverlayRequirements
{
  bool boxes{true}, masks{false}, field{true};
};

// 本组件不另建锁或线程：节点在原 overlay_mutex_ 内调用，等待和发布也使用同一把锁。
// Payload 仅承载共享只读结果；核心只检查图层是否到达，不读取 ROS 消息或像素。
template<typename Payload>
class OverlaySynchronizer
{
public:
  // 当前由 StreamNode::loadParameters() 在启动前设置；如需热更新，还要同步调用方的加锁与唤醒。
  void setRequirements(OverlayRequirements requirements) {requirements_ = requirements;}

  bool setTask(const std::string & task, std::uint64_t epoch, std::int64_t cutoff)
  {
    // 截止时间不允许回退；重复通知不清缓存，实际阶段变化才递增本地 generation。
    if (cutoff < cutoff_) {return false;}
    if (task == task_ && epoch == epoch_ && cutoff == cutoff_ && generation_ != 0) {
      return false;
    }
    // epoch 只来自阶段通知，图层自身未携带 epoch；源时间回退时不能据此宣称严格的跨代隔离。
    epoch_ = epoch;
    task_ = task == "seg" || task == "field" ? task : "";
    cutoff_ = cutoff;
    groups_.clear();
    composed_stamp_ = accepted_stamp_ = 0;
    ++generation_;
    return true;
  }

  std::optional<std::uint64_t> acceptFrame(const OverlayFrameKey & key)
  {
    // accepted_stamp_ 表示已接纳的源帧；合成可能尚未完成，不能拿它替代 composed_stamp_ 判旧图层。
    if (task_.empty() || generation_ == 0 || key.first < cutoff_ ||
      key.first <= accepted_stamp_) {return std::nullopt;}
    // 原图在当前阶段按源时间严格递增；frame_id 用于图层匹配，不会为每个来源建立独立时间游标。
    accepted_stamp_ = key.first;
    return generation_;
  }

  // 新增图层时扩充 OverlayLayer、这里的阶段归属与 ready()；载荷更新仍由节点适配层完成。
  // current 和 key.first 须使用同一源消息时钟，不能混入 steady_clock 或图层接收时刻作为帧键。
  template<typename Update>
  bool insert(
    const OverlayFrameKey & key, OverlayLayer layer, std::int64_t current,
    double timeout_s, Update update)
  {
    const char * task = layer == OverlayLayer::Field ? "field" : "seg";
    if (task_ != task || key.first <= 0 || key.first > current ||
      static_cast<double>(current - key.first) > timeout_s * 1e9 ||
      key.first < cutoff_ || key.first <= composed_stamp_) {return false;}
    auto found = groups_.lower_bound(key);
    if (found == groups_.end() || found->first != key) {
      if (groups_.size() >= kMaxFrames) {
        // 只保留按源时间戳、frame_id 排序的最新八组；更旧的迟到组不能挤掉已有结果。
        // 已存在组的后续图层仍可补齐；容量淘汰只针对新键，不能因补掩膜而误删同帧检测框。
        if (found == groups_.begin()) {return false;}
        groups_.erase(groups_.begin());
      }
      found = groups_.emplace_hint(found, key, Group{});
    }
    auto & group = found->second;
    update(group.payload);
    group.layers |= static_cast<unsigned>(layer);
    return true;
  }

  bool ready(const OverlayFrameKey & key) const
  {
    unsigned required = 0;
    if (task_ == "seg") {
      if (requirements_.boxes) {required |= static_cast<unsigned>(OverlayLayer::Detections);}
      if (requirements_.masks) {required |= static_cast<unsigned>(OverlayLayer::Mask);}
    } else if (task_ == "field" && requirements_.field) {
      required = static_cast<unsigned>(OverlayLayer::Field);
    }
    const auto found = groups_.find(key);
    const unsigned arrived = found == groups_.end() ? 0 : found->second.layers;
    // 只等待配置启用的图层；required 为 0 时无需等检测结果即可合成源图。
    return (arrived & required) == required;
  }

  std::optional<Payload> snapshot(const OverlayFrameKey & key, std::uint64_t generation)
  {
    // generation 是本地合成任务的代次，防止等待期间切阶段后仍提交旧画面，与外部 epoch 分开维护。
    if (generation != generation_) {return std::nullopt;}
    const auto found = groups_.find(key);
    Payload payload = found == groups_.end() ? Payload{} : found->second.payload;
    // 取得快照即结束该帧的收集窗口，后续即使绘制失败也不再回填；此游标不是成功发布计数。
    composed_stamp_ = std::max(composed_stamp_, key.first);
    // 快照后清掉该时刻及更早的组；迟到图层会在 insert() 被拒绝，避免已输出帧重新入缓存。
    for (auto it = groups_.begin(); it != groups_.end() && it->first.first <= composed_stamp_; ) {
      it = groups_.erase(it);
    }
    return payload;
  }

  static std::chrono::steady_clock::time_point waitDeadline(
    std::chrono::steady_clock::time_point received_at, double wait_ms)
  {
    // 图层等待用单调时钟；与源消息时间戳的 ROS 时钟判旧分开，系统校时不会延长等待。
    return received_at + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
      std::chrono::duration<double, std::milli>(wait_ms));
  }

  std::uint64_t generation() const {return generation_;}

private:
  // 调大容量只增加可保存的源帧组数，不改变 composeLoop() 的等待预算或结果新鲜度判定。
  static constexpr std::size_t kMaxFrames = 8;
  struct Group
  {
    Payload payload;
    unsigned layers{0};
  };
  OverlayRequirements requirements_;
  std::map<OverlayFrameKey, Group> groups_;
  std::string task_;
  std::int64_t cutoff_{0}, composed_stamp_{0}, accepted_stamp_{0};
  std::uint64_t epoch_{0}, generation_{0};
};
