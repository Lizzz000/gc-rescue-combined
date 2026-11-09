#pragma once
// 工作线程的启动、失败回收和停止唤醒。
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

// 统一线程启动、失败清理与停止回收；已关闭的最新值邮箱不能重启。
// 所有者须在回调和资源仍有效时调用 stop()，生命周期操作只允许串行调用。
// ---- 工作线程组：启动失败回收、停止唤醒与异常上报 ----
class WorkerGroup {
public:
  using Task = std::function<void()>;
  using Factory = std::function<std::thread(Task)>;
  ~WorkerGroup() {if (!threads_.empty()) {stop(wake_);}}
  bool running() const {return running_.load();}

  // wake 负责唤醒各阻塞点；仅清 running_ 无法结束邮箱或条件变量等待。
  // 启动失败、工作线程异常和正常停止均调用 wake；回调须可重复且不抛异常。
  void start(
    std::vector<Task> tasks, Task wake,
    Factory factory = [] (Task task) {return std::thread(std::move(task));})
  {
    if (started_) {throw std::logic_error("worker session cannot be restarted");}
    // 先预留线程容器；此时分配失败尚无工作线程，无需唤醒清理。
    threads_.reserve(tasks.size());
    wake_ = std::move(wake);
    started_ = true;
    running_.store(true);
    try {
      for (auto & task : tasks) {
        threads_.push_back(factory([this, task = std::move(task)] {
            try {task();} catch (...) {
              {
                std::lock_guard<std::mutex> lock(failure_mutex_);
                if (!failure_) {failure_ = std::current_exception();}
              }
              // 工作线程只发停止通知，不能 join 自己；执行器随后重新抛出并让入口返回失败。
              requestStop();
            }
          }));
      }
    } catch (...) {
      stop(wake_);
      throw;
    }
  }

  void requestStop()
  {signalStop(wake_);}

  void rethrowIfFailed() const
  {
    std::exception_ptr failure;
    {std::lock_guard<std::mutex> lock(failure_mutex_); failure = failure_;}
    if (failure) {std::rethrow_exception(failure);}
  }

  void stop(const Task & wake)
  {
    signalStop(wake);
    for (auto & thread : threads_) {if (thread.joinable()) {thread.join();}}
    threads_.clear();
  }

private:
  void signalStop(const Task & wake)
  {
    // 工作线程只通知停止；相机关闭和资源释放仍由所有者在 join 后完成。
    {
      // 更新运行标志时持有等待者同一把锁，避免检查条件与进入等待之间错过停机通知。
      std::lock_guard<std::mutex> lock(retry_mutex_);
      running_.store(false);
    }
    retry_cv_.notify_all();
    if (wake) {wake();}
  }

public:
  // 重试等待用此入口才能响应 stop()；直接 sleep_for() 会把停机延迟扩大到整段重试间隔。
  template<class Rep, class Period>
  void waitFor(std::chrono::duration<Rep, Period> delay)
  {
    std::unique_lock<std::mutex> lock(retry_mutex_);
    retry_cv_.wait_for(lock, delay, [this] {return !running();});
  }

private:
  mutable std::mutex failure_mutex_;
  std::exception_ptr failure_;
  bool started_{false};
  std::atomic<bool> running_{false};
  std::vector<std::thread> threads_;
  Task wake_;
  std::mutex retry_mutex_;
  std::condition_variable retry_cv_;
};
