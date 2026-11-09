#pragma once
// 最新值邮箱。stop 之后不能重新打开，调用方要换一个新对象。
// 只给检测节点使用。图传队列不使用这些类型。

#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <functional>
#include <exception>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <thread>
#include <utility>
#include <vector>

// ---- 最新值邮箱：交换载荷后在锁外消费，stop 永久关闭并清槽 ----
// take 阻塞等待，tryTake 立即返回；关闭后均不再交付载荷，set 不再接收。
template<typename T>
class LatestSlot {
public:
  void set(std::shared_ptr<T> value)
  {
    // 互斥锁只保护槽位交换；消费者取走后在锁外处理，生产者不等待其处理结束。
    {
      std::lock_guard<std::mutex> lock(mutex_);
      // 关闭后拒收新载荷，避免停机期间继续持有无人消费的图像。
      if (stopped_) {return;}
      if (slot_) {++overwritten_;}
      slot_.swap(value);  // 旧载荷在解锁后随 value 释放。
    }
    cv_.notify_one();
  }

  /// 阻塞取走最新值；永久关闭且无待消费值时返回 nullptr。
  std::shared_ptr<T> take()
  {
    std::unique_lock<std::mutex> lock(mutex_);
    // 条件谓词处理虚假唤醒；stop() 清空槽位并唤醒等待者，使消费线程可以退出。
    cv_.wait(lock, [this] {return slot_ != nullptr || stopped_;});
    if (!slot_) {return nullptr;}
    auto value = std::move(slot_);
    slot_.reset();
    return value;
  }

  /// 非阻塞取值；空槽立即返回 nullptr。
  std::shared_ptr<T> tryTake()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!slot_) {return nullptr;}
    auto value = std::move(slot_);
    slot_.reset();
    return value;
  }

  // clear() 只丢弃待消费值，不改变运行状态；下一次 set() 仍可正常唤醒 take()。
  void clear()
  {
    std::shared_ptr<T> retired;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      retired.swap(slot_);
    }
  }

  // stop() 是永久关闭，没有恢复入口；需要重启时应由调用方重新创建邮箱。
  void stop()
  {
    std::shared_ptr<T> retired;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      stopped_ = true;
      retired.swap(slot_);
    }
    cv_.notify_all();
  }

  /// 累计覆盖未消费载荷的次数；用于观察消费端是否落后，不等于成功处理帧数。
  std::uint64_t overwritten() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return overwritten_;
  }

  bool empty() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return slot_ == nullptr;
  }

private:
  mutable std::mutex mutex_;
  std::condition_variable cv_;
  std::shared_ptr<T> slot_;
  bool stopped_{false};
  std::uint64_t overwritten_{0};
};
