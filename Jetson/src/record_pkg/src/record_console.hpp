// ---- 预览控制台声明 ----
// 职责：PC 端预览、录制开关和页面。
#pragma once
#include "stream_protocol.hpp"
#include <nlohmann/json.hpp>
#include <condition_variable>
#include <deque>
#include <filesystem>
#include <mutex>
#include <thread>

namespace record {
std::string random_id();
// 同目录临时文件关闭后替换收据；不承诺断电持久性。
void atomic_json(const std::filesystem::path& path, const nlohmann::json& value);
struct ConsoleOptions {
  std::filesystem::path output_dir = "D:/A1_Workspace/GC_Rescue/MP4";
  std::string camera_host = "127.0.0.1", encoder = "x264";
  int camera_port = 18091, port = 8090, min_free_mib = 1024;
};
// 接收、控制请求和状态查询以 mutex_ 串行访问共享状态；FramePtr 共享只读 JPEG。
// 唯一 writer_ 在线程内持有编码器，锁外执行编码与文件 I/O；构造阶段先恢复历史收据。
class RecordConsole {
 public:
  explicit RecordConsole(ConsoleOptions options);
  ~RecordConsole();
  void receive(FramePtr frame);
  void disconnected(const std::string& error);
  void start(const std::string& request_id, std::uint64_t generation);
  void stop(const std::string& run_id);
  void tick();
  // 由生命周期所有者串行调用；先结束接收和 HTTP 线程，再等待本次编码收尾。
  void shutdown();
  FramePtr preview();
  nlohmann::json status();
 private:
  // 发布到 job_ 后，路径、source_id 和 first_ns 不变；队列、收据和阶段标志受 mutex_ 保护。
  struct Job {
    nlohmann::json receipt;
    std::filesystem::path partial, final, receipt_path;
    std::deque<FramePtr> queue;
    std::size_t queued_bytes = 0, queue_peak = 1;
    std::string source_id;
    std::uint64_t first_ns = 0, last_ns = 0, sequence = 0;
    // stopping 拒收新帧但允许排空；finalizing 冻结终态快照；done 才允许创建下一段。
    bool stopping = false, finalizing = false, done = false;
  };
  // *_locked 辅助函数要求调用者已持有 mutex_。
  void fail_locked(const std::string& error, const std::string& reason);
  void writer_loop();
  bool fresh_locked() const;
  ConsoleOptions options_;
  std::mutex mutex_;
  std::condition_variable changed_;
  // 预览使用最新帧；停止本段录像不停止接收，也不清空预览。
  FramePtr latest_;
  // 接收新鲜度与速率使用 PC 单调纳秒；Job 的采集时间来自同一板端 source_id。
  std::uint64_t last_received_ = 0, received_ = 0, rate_started_ = 0, rate_frames_ = 0;
  double receive_fps_ = 0;
  std::uintmax_t free_bytes_ = 0;
  std::string stream_error_ = "等待相机连接", token_;
  std::string last_request_;
  std::uint64_t start_generation_ = 0;
  std::shared_ptr<Job> job_;
  nlohmann::json recent_ = nlohmann::json::array();
  bool quitting_ = false;
  std::thread writer_;
};
}  // namespace record
