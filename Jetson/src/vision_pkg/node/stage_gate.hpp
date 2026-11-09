#pragma once
// 阶段、相机会话和发布截止时间。采集与发布共用这一道边界。
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

// 限帧实现同时供检测、原图和 HTTP 使用。

// 分别串行化采集入槽与结果发布，保持阶段代次、相机会话和截止时间一致。
// 切换固定按发布锁→采集锁顺序加锁、清帧槽；释放采集锁后仍持发布锁公布新代次，阻止旧结果越过切换。
// ---- 阶段闸门：采集交接、会话失效与最终发布共用边界 ----
class StageGate {
public:
  int active() const {return active_.load();}
  std::uint64_t generation() const {return generation_.load();}
  std::int64_t cutoff() const {return cutoff_.load();}
  std::uint64_t cameraSession() const {return camera_session_.load();}
  // 原子读只用于提前过滤；发布前须持 publication_mutex_，防止检查后发生切换或重连。
  bool current(std::uint64_t generation, std::uint64_t camera_session) const
  {return generation == this->generation() && camera_session == cameraSession();}
  std::unique_lock<std::mutex> lockPublication()
  {return std::unique_lock<std::mutex>(publication_mutex_);}

  // 切阶段后清待处理工作并公布新代次；clock 与帧时间戳须属于同一源时钟，单位为纳秒。
  // 相同阶段且已有截止时仅重发，不把周期通知当作新阶段。
  template<class Clock, class Flush, class Announce>
  void transition(int mask, Clock clock, Flush flush, Announce announce)
  {
    auto publication = lockPublication();
    {
      std::lock_guard<std::mutex> capture(capture_mutex_);
      if (active() != mask || cutoff() == 0) {
        // 两锁都取得后才采样截止时间，避免将等待加锁期间的旧阶段帧误归新阶段。
        // ROS 时间可能回退；保留已公布的最高截止时间，直到源时间再次追上。
        cutoff_.store(std::max(cutoff(), static_cast<std::int64_t>(clock())));
        generation_.fetch_add(1);
        flush();
      }
      active_.store(mask);
    }
    // 公布状态仍在发布锁内；announce 不得再次获取发布锁，耗时任务也不要放进此回调。
    announce(cutoff(), generation());
  }

  // 按当前阶段重发稳定水位，不从锁外快照重新切阶段；时钟未启动或尚未追上时暂不公布。
  // 首次有效时间建立水位并清槽，之后的刷新不推进截止时间或代次。
  template<class Clock, class Flush, class Announce>
  void refresh(Clock clock, Flush flush, Announce announce)
  {
    auto publication = lockPublication();
    {
      std::lock_guard<std::mutex> capture(capture_mutex_);
      const auto now = static_cast<std::int64_t>(clock());
      if (now <= 0 || now < cutoff()) {return;}
      if (cutoff() == 0) {
        cutoff_.store(now);
        generation_.fetch_add(1);
        flush();
      }
    }
    // active 只在发布锁内修改，释放采集锁后仍能公布最新的一致状态。
    announce(active(), cutoff(), generation());
  }

  // 重连使在途结果失效，但不改变识别阶段；锁序与 transition() 一致。
  // flush 只清待处理邮箱，不等待工作线程，也不能再次获取发布锁。
  template<class Flush>
  void resetCamera(Flush flush)
  {
    auto publication = lockPublication();
    std::lock_guard<std::mutex> capture(capture_mutex_);
    camera_session_.fetch_add(1);
    flush();
  }

  // admit 持有采集锁，只完成帧准入和入槽；不要从中获取发布锁，否则与 transition() 的顺序相反。
  template<class Admit>
  void capture(std::int64_t stamp, Admit admit)
  {
    std::lock_guard<std::mutex> lock(capture_mutex_);
    if (stamp >= cutoff() && active() != 0) {admit(generation());}
  }

private:
  // 原子读取便于提前丢弃过期工作；多字段读取不代表一致快照，最终发布仍须持发布锁复核。
  std::atomic<int> active_{1};
  std::atomic<std::uint64_t> generation_{0};
  std::atomic<std::int64_t> cutoff_{0};
  std::atomic<std::uint64_t> camera_session_{0};
  std::mutex capture_mutex_;
  std::mutex publication_mutex_;
};
