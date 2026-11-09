#pragma once
#include "stream/jpeg/jpeg.hpp"
#include "stream/stream.hpp"

// Copyright 2026 jetson_yolo integration
// SPDX-License-Identifier: Apache-2.0
// MJPEG over HTTP 图传服务端：把 BGR 帧编码成 JPEG，以 multipart/x-mixed-replace 推给浏览器/VLC/ffplay。
// 可接收原图编码或共享的既有 JPEG；缩放、质量和提交限速控制开销，所有观看端共享只读帧字节。


#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <exception>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>
#include <utility>

#include <opencv2/core.hpp>



// 双线程：accept 做接入与一次性响应，send 负责连续图传；所有端共用一个最新候选。
// 每端最多一个已交给 TCP 的待确认帧加一个当前 part，两帧分别保留原始截止时间。
// 未发字节的旧帧可替换为最新帧；开始发送后保持 part 完整，背压时跳过中间帧。
// 半帧过期或换代后断开观看端，由客户端重连；不能把新 JPEG 接到旧帧剩余字节后。
// reset(generation) 同时使旧代次的提交失效。
class MjpegServer
{
public:
  // 服务配置在启动前固定；未提供鉴权，监听地址决定可接入的网络接口。
  struct Options
  {
    /// 绑定的 IPv4 地址；默认监听所有接口，127.0.0.1 只允许本机访问。
    std::string address{"0.0.0.0"};
    /// TCP 监听端口。
    int port{8080};
    /// JPEG 质量，取值 1–100。
    int quality{80};
    /// 启动前固定编码后端；运行失败不回退，尺寸兼容行为由 JpegEncoder 统一处理。
    JpegBackend jpeg_backend{JpegBackend::OpenCv};
    /// 限制编码/提交帧率；0 表示不限速，与模型推理帧率独立。
    double max_fps{0.0};
    /// 编码前的等比缩放，1.0 保持原分辨率。
    double scale{1.0};
    /// 观看端在此时限内没有发送进展则断开，单位为秒；无帧空闲也计入，实际使用值最短 1 秒。
    double stall_timeout_s{10.0};
    /// 快照和新接入观看端不能复用过期画面。
    /// 非有限或 <=0 的值按 1 秒处理；生产由图传输入新鲜度注入，快照请求不重新起算源帧年龄。
    double frame_timeout_s{1.0};  // 秒
  };

  explicit MjpegServer(Options options);
  ~MjpegServer();

  MjpegServer(const MjpegServer &) = delete;
  MjpegServer & operator=(const MjpegServer &) = delete;

  /// 绑定并启动服务；地址或套接字操作失败时返回 false，并写出错误原因。
  bool start(std::string & error);

  /// 停止并回收服务线程、关闭所有连接；重复调用安全。
  void stop();

  /// 所有者定时轮询：线程均已结束时回收并取走首异常，否则立即返回空且保留异常。
  /// 与 start/stop 同样由所有者串行调用；不会等待仍在读请求头的线程。
  std::exception_ptr takeFailure();

  /// 从任意线程提交 BGR 帧；先限速再编码，所有观看端共享同一份 JPEG。
  /// 缩放与质量由内置的 JpegEncoder 负责，本入口没有帧键，每次调用都按新帧编码。
  /// valid_until 使用 steady_clock；留空时按本次提交时间加 frame_timeout_s 计算，而非 ROS 消息时间戳。
  /// source_stamp_ns 原样写入诊断头，仅用于识别源帧，0 表示未知；不能与 steady_clock 相减。
  void submit(
    const cv::Mat & bgr, std::uint64_t generation = 0,
    std::chrono::steady_clock::time_point valid_until = {}, std::int64_t source_stamp_ns = 0);

  /// 提交一份已经编码好的 JPEG（例如 ROS 压缩话题与图传共用同一次编码时）。
  /// 限速、代次与过期检查与 submit(bgr, ...) 共用同一条路径，只有编码这一步在外面做；
  /// 字节按 shared_ptr 共享，调用方不能再改写这份缓冲。
  /// encode_started/encode_ms 用于 part 诊断；未提供时对应字段为 -1（未知）。
  /// 帧龄按 valid_until 与 frame_timeout_s 反推，调用方应沿用源帧剩余有效期。
  void submitJpeg(
    std::shared_ptr<const std::vector<unsigned char>> jpeg,
    std::uint64_t generation = 0,
    std::chrono::steady_clock::time_point valid_until = {},
    std::chrono::steady_clock::time_point encode_started = {}, double encode_ms = -1.0,
    std::int64_t source_stamp_ns = 0);

  // 绘制前的低成本预检查，与 submit 共用代次、有效期和限速状态。
  // 只作为跳过无效工作的提示，不能替代 submit 的最终校验。
  bool readyForFrame(
    std::uint64_t generation,
    std::chrono::steady_clock::time_point valid_until) const;

  /// 清除旧缓存和待发帧，使此前阶段仍在编码的结果无法提交。
  void reset(std::uint64_t generation);

  std::size_t clientCount() const;

  /// 成功提交速率的平滑值，单位 Hz；断流超时后返回 0。
  double submitFps() const;

  /// 累计成功提交的帧数，包括 submitJpeg()；不等于编码次数，也不保证观看端已接收完整帧。
  std::uint64_t submittedFrames() const;

  /// 累计接受的观看端数量。
  std::uint64_t acceptedClients() const;

  /// 已有观看端开始发送（sent>0）且仍持有未发完的帧时，跳过新帧的累计次数；按源帧计一次。
  std::uint64_t skippedForBackpressure() const;

private:
  friend struct MjpegServerTestAccess;
  // 描述符只有一个所有者；接入、连接表和快照槽之间只移动所有权，异常自动关闭。
  class SocketFd
  {
  public:
    explicit SocketFd(int fd = -1) noexcept : fd_(fd) {}
    ~SocketFd();
    SocketFd(const SocketFd &) = delete;
    SocketFd & operator=(const SocketFd &) = delete;
    SocketFd(SocketFd && other) noexcept : fd_(other.release()) {}
    SocketFd & operator=(SocketFd && other) noexcept;
    operator int() const noexcept {return fd_;}
    int release() noexcept {return std::exchange(fd_, -1);}
    void reset(int fd = -1) noexcept;
  private:
    int fd_;
  };
  struct Client
  {
    SocketFd fd;
    /// 保留当前 part；仅在未发送字节归零且上一个待确认水位已回收时转交唯一退休槽。
    std::shared_ptr<const std::string> frame;
    // multipart 头、JPEG 与尾部组成同一段字节；每个观看端各自保存发送偏移，允许 TCP 部分写入。
    std::size_t sent{0};
    std::uint64_t sequence{0};
    std::uint64_t socket_bytes_sent{0};
    std::uint64_t retired_frame_end{0};
    std::chrono::steady_clock::time_point retired_frame_valid_until{};
    std::string peer;
    std::chrono::steady_clock::time_point last_progress{};
    std::chrono::steady_clock::time_point valid_until{};

    // 累计 ACK 水位包含最初的 HTTP 头；先回收确认完成的旧帧，再判断剩余旧帧是否过期。
    bool advanceAcknowledgement(
      std::uint64_t acknowledged, std::chrono::steady_clock::time_point now)
    {
      if (retired_frame_end != 0 && acknowledged >= retired_frame_end) {
        retired_frame_end = 0;
        retired_frame_valid_until = {};
      }
      return retired_frame_end == 0 || now <= retired_frame_valid_until;
    }
    void retireTransmittedFrame(int queued, int not_sent)
    {
      if (!frame || sent != frame->size() || not_sent != 0 || retired_frame_end != 0) {return;}
      retired_frame_end = queued > 0 ? socket_bytes_sent : 0;
      retired_frame_valid_until = queued > 0 ? valid_until : std::chrono::steady_clock::time_point{};
      frame.reset();
      sent = 0;
    }
  };

  void acceptLoop();
  void sendLoop();
  void startWorker(std::thread & worker, void (MjpegServer::*loop)());
  // 两个线程共用异常出口；无额外任务分配，工作线程只通知停止，所有者负责 join。
  template<class Task>
  void runWorker(Task task) noexcept
  {
    try {task();} catch (...) {requestStop(std::current_exception());}
    workers_active_.fetch_sub(1, std::memory_order_release);
  }
  void requestStop(std::exception_ptr failure = {});
  void addStreamClient(SocketFd fd, const std::string & peer);
  // 接入线程独占一份快照；写入/确认受源截止时间、代次和停机共同约束。
  void serveSnapshot(SocketFd socket);
  bool writeSnapshot(
    int fd, const std::string & header, const unsigned char * body, std::size_t body_size,
    std::uint64_t generation, std::chrono::steady_clock::time_point deadline);
  // 调用者持 mutex_；撤销 slot 后才能关闭，发送者每次系统调用前复核所有权。
  void closeSnapshotLocked(bool discard_pending);
  void serveViewer(int fd);
  // 健康页返回服务计数与编码诊断；HTTP 200 本身不代表当前相机或算法有有效输出。
  void serveHealth(int fd);
  void serveNotFound(int fd);
  // 自行取得连接锁检查准入并捕获内部服务会话；调用方不能已持 mutex_，阶段 generation 不变。
  std::optional<std::uint64_t> admitFrame(
    std::uint64_t generation, std::chrono::steady_clock::time_point valid_until) const;
  // 入口预检查通过后仍可能等编码锁；取得锁时复核，失效帧不消耗编码键或额度。
  void encodeAndPublish(
    const cv::Mat & bgr, std::uint64_t generation, std::uint64_t session,
    std::chrono::steady_clock::time_point valid_until, std::int64_t source_stamp_ns);
  /// submit() 与 submitJpeg() 的共同收尾：拼 multipart part、复核代次与过期、发给每个观看端并记统计。
  void publishJpeg(
    const std::shared_ptr<const std::vector<unsigned char>> & jpeg,
    std::uint64_t generation, std::uint64_t session, std::chrono::steady_clock::time_point valid_until,
    double age_at_encode_ms, double encode_ms, std::int64_t source_stamp_ns);
  /// 关闭并移除观看端，返回下一个迭代器，供单遍清理循环继续使用。
  std::vector<Client>::iterator dropClient(
    std::vector<Client>::iterator it,
    const char * why);

  Options options_;
  std::atomic<bool> running_{false};
  SocketFd listen_fd_;
  std::thread accept_thread_;
  std::thread send_thread_;
  // 仅创建、退出时更新；定时器不能因回收故障 HTTP 而阻塞 ROS 单线程执行器。
  std::atomic<unsigned> workers_active_{0};

  // 连接列表、最新帧和代次共同受这把锁保护；发送只推进各观看端的偏移，不复制共享帧内容。
  mutable std::mutex mutex_;
  std::condition_variable cv_;
  std::vector<Client> clients_;
  // acceptLoop 只有一个快照在途；reset/stop 可撤销，避免描述符关闭后被发送者误用。
  SocketFd snapshot_fd_;
  // 与停止状态同锁；第一次失败使 running_ 变为 false，后续异常不能覆盖首因。
  std::exception_ptr failure_;
  std::shared_ptr<const std::string> latest_part_;
  std::shared_ptr<const std::vector<unsigned char>> latest_jpeg_;
  std::chrono::steady_clock::time_point latest_valid_until_{};
  std::uint64_t latest_sequence_{0};
  std::uint64_t wake_revision_{0};
  std::uint64_t generation_{0};
  // 每次显式启动递增；首次准入捕获，编码锁后和最终提交均在 mutex_ 中复核。
  std::uint64_t session_{0};
  // 预检查只读，最终提交消耗额度；与代次共同受 mutex_ 保护。
  FrameRateLimiter rate_limiter_;

  // submit() 可从任意线程调用，编码器的单帧缓存本身没有同步，因此编码与帧键计数器由这把小锁串行化。
  // 不复用连接锁：发送与代次检查也使用它，编码时持有会拖慢接入和阶段切换。
  mutable std::mutex encode_mutex_;
  JpegEncoder encoder_;
  std::uint64_t submit_key_{0};

  // 提交成功时更新速率，reset() 清空窗口，状态读取时取值；独立小锁避免统计读者争用发送锁。
  mutable std::mutex stats_mutex_;
  std::chrono::steady_clock::time_point last_submit_{};
  double submit_fps_{0.0};
  double last_age_at_encode_ms_{-1.0};
  double last_encode_ms_{-1.0};

  std::atomic<std::uint64_t> submitted_{0};
  std::atomic<std::uint64_t> accepted_{0};
  std::atomic<std::uint64_t> skipped_{0};
};
