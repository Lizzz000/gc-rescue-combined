#pragma once
// 耗时均值、累计完成数和窗口速率。不管线程寿命。
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

// ---- 诊断采样：耗时 EMA、累计完成数与窗口速率 ----
// RateMeter 维护 EMA 耗时及累计次数，WindowCounter 将累计次数换算成窗口速率。
// RateMeter 由所属工作线程单写、原子读取可用于状态定时器和画面统计；WindowCounter 的窗口游标仅由状态回调推进。
// EMA 是运行诊断的平滑值，不是验收使用的 P95。
// 百分位与端到端统计由独立采样工具按实际样本计算。

class RateMeter {
public:
  /// 记录一次完成及其耗时；帧率由状态回调采样累计次数，不在每帧重复维护另一套频率。
  void record(double latency_ms)
  {
    ema_ms_.store(blend(ema_ms_.load(), latency_ms));
    count_.fetch_add(1);
  }

  double latencyMs() const {return ema_ms_.load();}
  std::uint64_t count() const {return count_.load();}

private:
  static double blend(double previous, double sample, double weight = 0.1)
  {
    // 没有正的历史值时直接采用当前样本，避免从 0 缓慢爬升。
    return previous > 0.0 ? (1.0 - weight) * previous + weight * sample : sample;
  }

  std::atomic<double> ema_ms_{0.0};
  std::atomic<std::uint64_t> count_{0};
};

/// 将累计完成计数换算成相邻采样窗口的平均速率。
class WindowCounter {
public:
  using Clock = std::chrono::steady_clock;

  /// 推进采样窗口并返回速率；首次调用只建立基准，返回 0。
  // rate() 会推进窗口，必须由同一调用线程维护；累计 total 可以来自其他线程的原子计数。
  double rate(std::uint64_t total)
  {
    const auto now = Clock::now();
    if (last_at_.time_since_epoch().count() == 0) {
      last_at_ = now;
      last_total_ = total;
      return 0.0;
    }
    const double dt = std::chrono::duration<double>(now - last_at_).count();
    last_at_ = now;
    // 计数回退时本窗口按 0 增量处理，并把新总数设为下一窗口的基准。
    const std::uint64_t delta = total >= last_total_ ? total - last_total_ : 0;
    last_total_ = total;
    return dt > 0.0 ? static_cast<double>(delta) / dt : 0.0;
  }

private:
  Clock::time_point last_at_{};
  std::uint64_t last_total_{0};
};
