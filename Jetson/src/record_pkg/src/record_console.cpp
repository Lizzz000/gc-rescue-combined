// ---- 预览控制台实现 ----
// 职责：页面、录制循环和进程入口。
#include "record_console.hpp"
#include "ffmpeg/mp4_writer.hpp"
#include <algorithm>
#include <fstream>
#include <iomanip>
#include <random>
#include <sstream>
#include <stdexcept>
#ifdef _WIN32
#include <windows.h>
#else
#include <fcntl.h>
#include <unistd.h>
#endif

namespace record {
namespace fs = std::filesystem;
using Json = nlohmann::json;
// 会话标识与收据替换：先关闭临时文件，再一次替换目标路径。
std::string random_id() {
  std::random_device random;
  std::ostringstream out;
  for (int i = 0; i < 4; ++i) out << std::hex << std::setw(8) << std::setfill('0') << std::uint32_t(random());
  return out.str();
}
void atomic_json(const fs::path& path, const Json& value) {
  auto temp = path; temp += ".tmp";
  try {
  std::ofstream out(temp, std::ios::binary | std::ios::trunc);
  out.exceptions(std::ios::badbit | std::ios::failbit);
  out << value.dump(2) << '\n'; out.close();
#ifdef _WIN32
  if (!MoveFileExW(temp.c_str(), path.c_str(), MOVEFILE_REPLACE_EXISTING | MOVEFILE_WRITE_THROUGH))
    throw std::runtime_error("cannot replace recording receipt: " + std::to_string(GetLastError()));
#else
  fs::rename(temp, path);
#endif
  } catch (...) { std::error_code ignored; fs::remove(temp, ignored); throw; }
}
namespace {
std::string date_label() {
  const auto t = std::time(nullptr);
  std::tm date{};
#ifdef _WIN32
  localtime_s(&date, &t);
#else
  localtime_r(&t, &date);
#endif
  std::ostringstream out; out << std::put_time(&date, "%Y%m%d-%H%M%S"); return out.str();
}
}
// 初始化与历史恢复在线程启动前完成，不把上次未收尾的会话当作成功录像。
RecordConsole::RecordConsole(ConsoleOptions options) : options_(std::move(options)), token_(random_id()) {
  fs::create_directories(options_.output_dir);
  options_.output_dir = fs::canonical(options_.output_dir);
  free_bytes_ = fs::space(options_.output_dir).available;
  std::vector<fs::path> receipts;
  for (const auto& entry : fs::directory_iterator(options_.output_dir)) {
    const auto name = entry.path().filename().string();
    if (entry.is_regular_file() && name.size() > 13 && name.compare(name.size() - 13, 13, ".session.json") == 0)
      receipts.push_back(entry.path());
  }
  std::sort(receipts.rbegin(), receipts.rend());
  for (const auto& path : receipts) {
    if (recent_.size() == 10) break;
    try {
      std::ifstream input(path); auto receipt = Json::parse(input);
      input.close();  // Windows不能原子替换仍由本进程打开的收据。
      const auto state = receipt.value("state", "");
      if (state != "completed" && state != "failed") {
        // 上次程序崩溃留下的running不能继承成功；保留原片段，只修正收据。
        receipt["state"] = "failed"; receipt["status"] = "failed";
        receipt["error"] = "上次程序未正常收尾；请检查部分录像";
        const auto file = fs::u8path(receipt.value("file", ""));
        if (file != file.filename()) throw std::runtime_error("invalid receipt filename");
        auto name = file.u8string();
        if (name.size() > 12 && name.compare(name.size() - 12, 12, ".partial.mp4") == 0) {
          const auto final = name.substr(0, name.size() - 12) + ".mp4";
          if (!fs::exists(options_.output_dir / file) && fs::exists(options_.output_dir / final)) receipt["file"] = final;
        }
        atomic_json(path, receipt);
      }
      recent_.push_back(receipt);
    } catch (...) { /* 不把无效历史收据混入当前会话。 */ }
  }
  writer_ = std::thread(&RecordConsole::writer_loop, this);
}
RecordConsole::~RecordConsole() { shutdown(); }
// 接收与失败准入：以下 *_locked 辅助函数不自行加锁。
bool RecordConsole::fresh_locked() const {
  return latest_ && stream_error_.empty() && monotonic_ns() - last_received_ < 2000000000ULL;
}
void RecordConsole::fail_locked(const std::string& error, const std::string& reason) {
  // 保留首个失败原因；写线程冻结终态快照后，不再接受外部错误覆盖。
  if (!job_ || job_->done || job_->finalizing) return;
  if (job_->receipt["error"].get<std::string>().empty()) {
    job_->receipt["error"] = error; job_->receipt["reason"] = reason;
  }
  job_->receipt["state"] = "stopping"; job_->receipt["status"] = "stopping";
  job_->stopping = true;
  changed_.notify_all();
}
void RecordConsole::receive(FramePtr frame) {
  std::lock_guard<std::mutex> lock(mutex_);
  const auto now = monotonic_ns();
  // 先更新预览；录像的停止边界只决定是否继续入队。
  latest_ = frame; last_received_ = now; ++received_; stream_error_.clear();
  if (!rate_started_) { rate_started_ = now; rate_frames_ = received_; }
  if (now - rate_started_ >= 1000000000ULL) {
    receive_fps_ = double(received_ - rate_frames_) * 1e9 / double(now - rate_started_);
    rate_started_ = now; rate_frames_ = received_;
  }
  if (!job_ || job_->stopping || job_->done) return;
  // 只查换源；丢帧不再结束录像，时间单调由 Mp4Writer 检查。
  if (frame->source_id != job_->source_id) {
    fail_locked("相机帧不连续或来源改变，本次录像已结束", "frame_discontinuity"); return;
  }
  // 队列最多60帧/64 MiB JPEG（不含已出队帧及编码器内存）；默认30 FPS约容纳2秒抖动。
  if (job_->queue.size() >= 60 || frame->jpeg.size() > 64ULL * 1024 * 1024 - job_->queued_bytes) { fail_locked("电脑编码或磁盘跟不上相机，本次录像已结束", "writer_overload"); return; }
  job_->last_ns = frame->capture_ns; job_->sequence = frame->sequence;
  job_->queued_bytes += frame->jpeg.size();
  job_->queue.push_back(std::move(frame));
  job_->queue_peak = std::max(job_->queue_peak, job_->queue.size());
  changed_.notify_all();
}
void RecordConsole::disconnected(const std::string& error) {
  std::lock_guard<std::mutex> lock(mutex_);
  latest_.reset(); stream_error_ = error; rate_started_ = 0; receive_fps_ = 0;
  if (job_ && !job_->stopping) fail_locked("相机连接中断，本次录像已结束；连接恢复后请重新开始", "camera_disconnected");
}
// 控制请求与状态快照：开始代次、运行编号及停止标志在同一把锁下提交。
void RecordConsole::start(const std::string& request_id, std::uint64_t generation) {
  std::lock_guard<std::mutex> lock(mutex_);
  // 请求必须属于页面看到的开始代次；只允许当前请求重试，历史请求永不再执行。
  if (request_id == last_request_ && generation + 1 == start_generation_) return;
  if (quitting_) throw std::runtime_error("控制台正在退出");
  if (job_ && !job_->done) throw std::runtime_error("上一段录像尚未结束");
  if (!fresh_locked()) throw std::runtime_error("相机尚未连接，请等待新画面");
  if (free_bytes_ < std::uint64_t(options_.min_free_mib) * 1024 * 1024) throw std::runtime_error("电脑磁盘剩余空间不足");
  auto job = std::make_shared<Job>();
  const auto id = random_id(); const auto name = "GC_" + date_label() + "-" + id.substr(0, 8);
  job->partial = options_.output_dir / (name + ".partial.mp4");
  job->final = options_.output_dir / (name + ".mp4");
  job->receipt_path = options_.output_dir / (name + ".session.json");
  job->source_id = latest_->source_id; job->first_ns = job->last_ns = latest_->capture_ns;
  job->sequence = latest_->sequence; job->queue.push_back(latest_); job->queued_bytes = latest_->jpeg.size();
  job->receipt = {{"run_id", id}, {"state", "starting"}, {"status", "starting"}, {"file", job->partial.filename().u8string()},
    {"frames", 0}, {"frames_received", 1}, {"bad_jpeg_frames", 0}, {"warning", ""}, {"elapsed_seconds", 0.0}, {"bytes", 0}, {"error", ""}, {"reason", ""}, {"finalized", false},
    {"source_id", job->source_id}, {"first_capture_ns", job->first_ns},
    {"requested", {{"width", latest_->width}, {"height", latest_->height}, {"fps", latest_->fps}, {"output_dir", options_.output_dir.u8string()},
                    {"codec", "H264"}, {"encoder", options_.encoder}, {"container", "fragmented MP4"}}}};
  // 将当前预览作为首帧一起发布；本段完成前不能用新请求替换 Job。
  job_ = std::move(job); last_request_ = request_id; ++start_generation_;
  changed_.notify_all();
}
// 停止仅关闭本段入队，不等待 I/O；已入队帧由 writer_loop 排空，状态查询负责观察完成。
void RecordConsole::stop(const std::string& run_id) {
  std::lock_guard<std::mutex> lock(mutex_);
  (void)run_id;  // run_id 比对已收敛删除，停止当前会话。
  if (!job_) throw std::runtime_error("停止请求不属于当前录像");
  if (job_->done || job_->stopping) return;
  job_->stopping = true; job_->receipt["state"] = "stopping"; job_->receipt["status"] = "stopping";
  job_->receipt["reason"] = "user_stop"; changed_.notify_all();
}
void RecordConsole::tick() {
  // 空间查询在锁外执行；会话收据由写线程在开始和终态落盘，避免进度快照覆盖终态。
  std::uintmax_t available = 0;
  std::error_code disk_error;
  available = fs::space(options_.output_dir, disk_error).available;
  std::lock_guard<std::mutex> lock(mutex_);
  if (latest_ && !fresh_locked()) {
    latest_.reset(); stream_error_ = "相机画面已超时，正在重连";
    if (job_ && !job_->stopping) fail_locked("相机画面超时，本次录像已结束", "camera_timeout");
  }
  if (disk_error) fail_locked(disk_error.message(), "disk_error");
  else {
    free_bytes_ = available;
    if (free_bytes_ < std::uint64_t(options_.min_free_mib) * 1024 * 1024) fail_locked("电脑磁盘剩余空间不足", "low_disk_space");
  }
}
FramePtr RecordConsole::preview() { std::lock_guard<std::mutex> lock(mutex_); return fresh_locked() ? latest_ : nullptr; }
Json RecordConsole::status() {
  std::lock_guard<std::mutex> lock(mutex_);
  Json active = job_ ? job_->receipt : Json{{"state", "idle"}, {"run_id", ""}, {"file", ""}, {"frames", 0}, {"elapsed_seconds", 0}, {"bytes", 0}, {"error", ""}};
  Json result = {{"token", token_}, {"start_generation", start_generation_}, {"connected", fresh_locked()}, {"frame_age_ms", last_received_ ? (monotonic_ns() - last_received_) / 1000000ULL : 0},
    {"width", latest_ ? latest_->width : 0}, {"height", latest_ ? latest_->height : 0}, {"fps", latest_ ? latest_->fps : 0},
    {"received_frames", received_}, {"receive_fps", receive_fps_}, {"output_dir", options_.output_dir.u8string()},
    {"free_bytes", free_bytes_}, {"stream_error", stream_error_}, {"record", active}, {"recent", recent_}};
  return result;
}
// 编码与终态提交：唯一写线程保有 Job 引用，文件操作不占用会话锁。
void RecordConsole::writer_loop() {
  for (;;) {
    std::shared_ptr<Job> job;
    { std::unique_lock<std::mutex> lock(mutex_);
      changed_.wait(lock, [&] { return quitting_ || (job_ && !job_->done); });
      if (quitting_ && (!job_ || job_->done)) return;
      job = job_;
    }
    std::unique_ptr<Mp4Writer> writer;
    std::string failure;
    std::uint64_t count = 0, first_encoded_ns = 0;
    unsigned consecutive_bad = 0;
    try {
      FramePtr first;
      Json initial;
      { std::lock_guard<std::mutex> lock(mutex_); first = job->queue.front(); initial = job->receipt; }
      atomic_json(job->receipt_path, initial);
      writer = std::make_unique<Mp4Writer>(job->partial, first->width, first->height, first->fps, options_.encoder);
      { std::lock_guard<std::mutex> lock(mutex_);
        job->receipt["encoder"] = writer->encoder_name();
        job->receipt["quality"] = "x264 CRF18";
        if (!job->stopping) { job->receipt["state"] = "recording"; job->receipt["status"] = "recording"; }
      }
      for (;;) {
        FramePtr frame;
        { std::unique_lock<std::mutex> lock(mutex_);
          changed_.wait(lock, [&] { return job->stopping || !job->queue.empty(); });
          // 停止后继续消费已有帧；停止且队列为空才进入收尾。
          if (job->queue.empty()) break;
          frame = job->queue.front(); job->queue.pop_front();
          job->queued_bytes -= frame->jpeg.size();
        }
        // 出队后仅持有只读帧引用，编码不阻塞接收端获取会话锁。
        try { writer->write(*frame); }
        catch (const BadJpeg& e) {
          std::lock_guard<std::mutex> lock(mutex_);
          const auto bad = job->receipt["bad_jpeg_frames"].get<unsigned>() + 1;
          job->receipt["bad_jpeg_frames"] = bad;
          job->receipt["warning"] = "已跳过 " + std::to_string(bad) + " 个无法完整解码的JPEG帧，保留原采集时间间隔";
          job->receipt["frames_received"] = job->sequence - first->sequence + 1;
          if (++consecutive_bad >= 3) throw std::runtime_error(std::string("连续3个JPEG帧无法解码: ") + e.what());
          continue;
        }
        consecutive_bad = 0;
        if (!count) first_encoded_ns = frame->capture_ns;
        ++count;
        std::error_code size_error;
        const auto size = count % first->fps == 0 ? fs::file_size(job->partial, size_error) : 0;
        { std::lock_guard<std::mutex> lock(mutex_);
          job->receipt["frames"] = count;
          job->receipt["frames_received"] = job->sequence - first->sequence + 1;
          job->receipt["first_encoded_capture_ns"] = first_encoded_ns;
          job->receipt["elapsed_seconds"] = double(frame->capture_ns - first_encoded_ns) / 1e9;
          job->receipt["last_capture_ns"] = frame->capture_ns;
          if (size && !size_error) job->receipt["bytes"] = size;
        }
      }
    } catch (const std::exception& e) {
      failure = e.what();
      std::lock_guard<std::mutex> lock(mutex_);
      fail_locked(failure, "writer_error");
    }
    // 断线或编码失败仍尝试封装已编码帧；finish/析构均在锁外，关闭成功后才有资格改名。
    bool finalized = false;
    try { if (writer) { writer->finish(); finalized = true; } } catch (const std::exception& e) { if (failure.empty()) failure = e.what(); }
    writer.reset();
    Json final;
    { std::lock_guard<std::mutex> lock(mutex_);
      if (job->receipt["error"].get<std::string>().empty() && !failure.empty()) {
        job->receipt["error"] = failure; job->receipt["reason"] = "writer_error";
      }
      job->receipt["queue_peak_frames"] = job->queue_peak;
      // 锁内冻结外部失败入口，随后锁外改名并写最终收据；done 此时仍为 false。
      job->finalizing = true; final = job->receipt;
    }
      try {
        if (!count) throw std::runtime_error("未写入有效视频帧");
        const bool okay = final["error"].get<std::string>().empty();
        if (okay) {
          fs::rename(job->partial, job->final); final["file"] = job->final.filename().u8string();
        }
        final["finalized"] = finalized;
        final["bytes"] = fs::file_size(okay ? job->final : job->partial);
      } catch (const std::exception& e) { if (final["error"].get<std::string>().empty()) final["error"] = e.what(); }
      auto state = final["error"].get<std::string>().empty() ? "completed" : "failed";
      final["state"] = state; final["status"] = state;
      // completed 还须通过最终收据写入；写入失败时 MP4 可能已经改名，仍发布 failed。
      try { atomic_json(job->receipt_path, final); }
      catch (const std::exception& e) {
        final["state"] = "failed"; final["status"] = "failed"; final["receipt_error"] = e.what();
        if (final["error"].get<std::string>().empty()) { final["error"] = std::string("最终收据写入失败: ") + e.what(); final["reason"] = "receipt_error"; }
      }
    { std::lock_guard<std::mutex> lock(mutex_);
      // 对外发布终态后才释放队列并开放下一段，不让新会话覆盖正在提交的结果。
      job->receipt = std::move(final);
      job->queue.clear(); job->queued_bytes = 0; job->done = true;
      recent_.insert(recent_.begin(), job->receipt);
      if (recent_.size() > 10) recent_.erase(recent_.end() - 1);
    }
  }
}
void RecordConsole::shutdown() {
  { std::lock_guard<std::mutex> lock(mutex_);
    quitting_ = true;
    if (job_ && !job_->done && !job_->stopping) {
      job_->stopping = true; job_->receipt["reason"] = "console_shutdown";
      job_->receipt["state"] = "stopping"; job_->receipt["status"] = "stopping";
    }
  }
  changed_.notify_all();
  // 不持会话锁等待，写线程才能排空并提交；同步文件 I/O 卡住时这里也会等待。
  if (writer_.joinable()) writer_.join();
}
}  // namespace record
