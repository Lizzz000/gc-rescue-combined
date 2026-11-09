// 阅读：HTTP 接入、快照、健康检查、帧准入和统计。
// 观看页正文在 page.cpp，慢连接发送在 send.cpp。
// 不编码 JPEG。编码在 stream/jpeg/jpeg.cpp。
#include "stream/mjpeg/mjpeg.hpp"

#include <arpa/inet.h>
#include <fcntl.h>
#include <linux/sockios.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <poll.h>
#include <signal.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <unistd.h>
#include <cerrno>
#include <cmath>
#include <cstddef>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <limits>
#include <utility>

#ifndef MSG_NOSIGNAL
#define MSG_NOSIGNAL 0
#endif

// 观看页正文在 page.cpp。boundary=frame 仍必须和 publishJpeg() 的分隔符一致。
extern const char * kViewerPage;

/// 清除地址两侧空白；仅归一化文本，IPv4 合法性仍由 start 检查。
static std::string trimAddress(const std::string & value)
{
  const std::size_t first = value.find_first_not_of(" \t\r\n");
  if (first == std::string::npos) {return std::string();}
  const std::size_t last = value.find_last_not_of(" \t\r\n");
  return value.substr(first, last - first + 1);
}

static const char * kStreamHeader =
  // boundary=frame 必须与 publishJpeg() 中每帧的 --frame 分隔符一致。
  "HTTP/1.1 200 OK\r\n"
  "Connection: close\r\n"
  "Cache-Control: no-store, no-cache, must-revalidate, private\r\n"
  "Pragma: no-cache\r\n"
  "Content-Type: multipart/x-mixed-replace; boundary=frame\r\n"
  "\r\n";

static const char * kNotFoundBody = "try / or /stream.mjpg or /snapshot.jpg or /healthz\n";

/// 只认明确写出 text/html 的 Accept。`*/*` 仍走 MJPEG，避免 ffplay/VLC 拿到网页。
static bool acceptWantsHtml(const std::string & head)
{
  std::size_t cursor = 0;
  while (cursor < head.size()) {
    const auto line_end = head.find('\n', cursor);
    const auto count = line_end == std::string::npos ? head.size() - cursor : line_end - cursor;
    bool is_accept = count >= 7;
    if (is_accept) {
      const char prefix[] = "accept:";
      for (std::size_t i = 0; i < 7; ++i) {
        if (std::tolower(static_cast<unsigned char>(head[cursor + i])) != prefix[i]) {
          is_accept = false;
          break;
        }
      }
    }
    if (is_accept) {
      for (std::size_t i = cursor; i + 9 <= cursor + count; ++i) {
        bool match = true;
        const char token[] = "text/html";
        for (std::size_t j = 0; j < 9; ++j) {
          if (std::tolower(static_cast<unsigned char>(head[i + j])) != token[j]) {
            match = false;
            break;
          }
        }
        if (match) {return true;}
      }
    }
    if (line_end == std::string::npos) {break;}
    cursor = line_end + 1;
  }
  return false;
}

/// 请求头最大 8 KiB、总读取预算 1 秒，慢速或沉默客户端不能无限占用接入线程。
constexpr std::size_t kMaxRequestHeadBytes = 8192;
constexpr int kRequestReadTimeoutMs = 1000;
// 持续观看连接有固定上限；一次性健康/快照响应仍可访问。
constexpr std::size_t kMaxStreamClients = 8;
// 响应总预算不随部分写入续期；源图剩余有效期更短时优先源截止时间。
constexpr int kSnapshotWriteTimeoutMs = 2000;
constexpr std::size_t kSnapshotWriteChunkBytes = 64 * 1024;
constexpr std::int64_t kSnapshotPollIntervalMs = 10;

/// 完整写出阻塞响应或返回失败；套接字发送超时由调用者设置。
static bool writeAll(int fd, const void * data, std::size_t size)
{
  // 一次 send() 不保证写完，必须按实际字节数继续；这里只用于带超时的阻塞响应。
  const char * cursor = static_cast<const char *>(data);
  std::size_t left = size;
  while (left > 0) {
    const ssize_t written = ::send(fd, cursor, left, MSG_NOSIGNAL);
    if (written > 0) {
      cursor += written;
      left -= static_cast<std::size_t>(written);
    } else if (written < 0 && errno == EINTR) {
      continue;
    } else {
      return false;
    }
  }
  return true;
}

/// 分段读取请求头并返回首行用于路由；正常请求读到空行，避免未读请求数据使关闭触发 RST。
/// 所有分段共享绝对截止时间与容量上限，逐字节到达也不会持续延长读取预算。
static bool readRequestHead(int fd, std::string & request_line, bool & wants_html)
{
  // 所有分段共享一个绝对截止时刻，逐字节到达也不会反复续期。
  const auto deadline = std::chrono::steady_clock::now() +
    std::chrono::milliseconds(kRequestReadTimeoutMs);
  std::string head;
  char buffer[1024];
  while (head.size() <= kMaxRequestHeadBytes) {
    if (head.find("\r\n\r\n") != std::string::npos ||
      head.find("\n\n") != std::string::npos) {break;}
    const auto left = deadline - std::chrono::steady_clock::now();
    if (left <= std::chrono::steady_clock::duration::zero()) {break;}
    pollfd descriptor{};
    descriptor.fd = fd;
    descriptor.events = POLLIN;
    const auto wait_ms = std::chrono::duration_cast<std::chrono::milliseconds>(left).count();
    const int ready = ::poll(&descriptor, 1, static_cast<int>(std::max<std::int64_t>(1, wait_ms)));
    if (ready < 0 && errno == EINTR) {continue;}
    if (ready <= 0) {break;}
    const ssize_t got = ::recv(fd, buffer, sizeof(buffer), 0);
    if (got > 0) {
      head.append(buffer, static_cast<std::size_t>(got));
      continue;
    }
    if (got < 0 && errno == EINTR) {continue;}
    // 超时或对端关闭时保留已收到的首行；本函数只返回路由依据，不是完整 HTTP 合法性校验器。
    break;
  }
  if (head.size() > kMaxRequestHeadBytes) {return false;}
  wants_html = acceptWantsHtml(head);
  const std::size_t end = head.find('\n');
  request_line = head.substr(0, end == std::string::npos ? head.size() : end);
  while (!request_line.empty() && (request_line.back() == '\r' || request_line.back() == ' ')) {
    request_line.pop_back();
  }
  return !request_line.empty();
}

/// 从请求行提取路径并去除查询参数，例如快照请求统一路由到 /snapshot.jpg。
static std::string pathOf(const std::string & request_line)
{
  const std::size_t first_space = request_line.find(' ');
  if (first_space == std::string::npos) {return std::string();}
  const std::size_t second_space = request_line.find(' ', first_space + 1);
  std::string path = request_line.substr(
    first_space + 1,
    second_space == std::string::npos ? std::string::npos : second_space - first_space - 1);
  const std::size_t query = path.find('?');
  if (query != std::string::npos) {path.resize(query);}
  return path;
}

// ---- 服务生命周期：监听、发送线程与停止回收 ----
MjpegServer::SocketFd::~SocketFd() {reset();}

MjpegServer::SocketFd & MjpegServer::SocketFd::operator=(SocketFd && other) noexcept
{
  if (this != &other) {reset(other.release());}
  return *this;
}

void MjpegServer::SocketFd::reset(int fd) noexcept
{
  const int previous = std::exchange(fd_, fd);
  if (previous >= 0) {::close(previous);}
}

MjpegServer::MjpegServer(Options options)
: options_(options)
{
  encoder_.setBackend(options_.jpeg_backend);
  options_.address = trimAddress(options_.address);
  options_.port = std::max(1, std::min(65535, options_.port));
  options_.quality = std::max(1, std::min(100, options_.quality));
  // 非正 max_fps 表示不限速；负值归一为 0，避免多种默认含义。
  if (options_.max_fps < 0.0) {options_.max_fps = 0.0;}
  if (!std::isfinite(options_.stall_timeout_s) || options_.stall_timeout_s <= 0.0) {
    options_.stall_timeout_s = 10.0;
  }
}

MjpegServer::~MjpegServer()
{
  stop();
}

// 生命周期操作由所有者串行调用；绑定完成后再启动接入和发送线程。
bool MjpegServer::start(std::string & error)
{
  error.clear();
  if (running_.load()) {return true;}
  // 上次异常可能已清运行标志但仍有 joinable 线程；显式重启先完整回收。
  stop();
  {
    std::lock_guard<std::mutex> lock(mutex_);
    failure_ = {};
    ++session_;
  }

  // 客户端在半帧时断开不能让 SIGPIPE 终止整个节点进程。
  ::signal(SIGPIPE, SIG_IGN);

  // 创建套接字前拒绝非法监听地址，并在错误中保留原始配置值。
  sockaddr_in address{};
  address.sin_family = AF_INET;
  if (::inet_pton(AF_INET, options_.address.c_str(), &address.sin_addr) != 1) {
    error = "bind address '" + options_.address + "' is not an IPv4 address";
    return false;
  }
  address.sin_port = htons(static_cast<std::uint16_t>(options_.port));

  // Linux 原子建立非阻塞监听端；失败即拒绝启动，不允许退回可能阻塞 stop 的 accept。
  SocketFd listener(::socket(AF_INET, SOCK_STREAM | SOCK_NONBLOCK, 0));
  if (listener < 0) {
    error = std::string("socket(): ") + std::strerror(errno);
    return false;
  }

  int one = 1;
  ::setsockopt(listener, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));

  if (::bind(listener, reinterpret_cast<sockaddr *>(&address), sizeof(address)) != 0) {
    error = "bind(" + options_.address + ":" + std::to_string(options_.port) + "): " +
      std::strerror(errno);
    return false;
  }
  if (::listen(listener, 8) != 0) {
    error = std::string("listen(): ") + std::strerror(errno);
    return false;
  }

  listen_fd_ = std::move(listener);
  running_.store(true);
  try {
    startWorker(accept_thread_, &MjpegServer::acceptLoop);
    startWorker(send_thread_, &MjpegServer::sendLoop);
  } catch (...) {
    // 第二个线程创建失败时也必须先回收第一个，再向调用者报告。
    requestStop(std::current_exception());
    stop();
    error = "cannot start HTTP worker threads";
    return false;
  }
  return true;
}

void MjpegServer::startWorker(std::thread & worker, void (MjpegServer::*loop)())
{
  // 必须在线程可运行前计数；构造失败归还名额，避免定时器永远等待不存在的线程。
  workers_active_.fetch_add(1, std::memory_order_relaxed);
  try {
    worker = std::thread([this, loop] {runWorker([this, loop] {(this->*loop)();});});
  } catch (...) {
    workers_active_.fetch_sub(1, std::memory_order_release);
    throw;
  }
}

void MjpegServer::requestStop(std::exception_ptr failure)
{
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (failure && !failure_) {failure_ = failure;}
    running_.store(false);
    closeSnapshotLocked(true);
  }
  cv_.notify_all();
}

std::exception_ptr MjpegServer::takeFailure()
{
  std::exception_ptr failure;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!failure_ || workers_active_.load(std::memory_order_acquire) != 0) {return {};}
    failure = std::exchange(failure_, {});
  }
  // 仅在全部工作线程已走完出口后 join；显式 stop 仍可主动等待线程退出。
  stop();
  return failure;
}

void MjpegServer::stop()
{
  requestStop();
  // 先撤销快照并唤醒发送线程；无论运行标志为何值，都回收已经创建的线程。
  if (accept_thread_.joinable()) {accept_thread_.join();}
  if (send_thread_.joinable()) {send_thread_.join();}

  listen_fd_.reset();

  std::lock_guard<std::mutex> lock(mutex_);
  for (auto & client : clients_) {
    if (client.fd >= 0) {
      linger discard{1, 0};
      ::setsockopt(client.fd, SOL_SOCKET, SO_LINGER, &discard, sizeof(discard));
    }
  }
  clients_.clear();
  latest_part_.reset();
  latest_jpeg_.reset();
  latest_valid_until_ = {};
  rate_limiter_.reset();
  std::lock_guard<std::mutex> stats_lock(stats_mutex_);
  last_submit_ = {};
  submit_fps_ = 0.0;
  last_age_at_encode_ms_ = last_encode_ms_ = -1.0;
}

// ---- HTTP 入口：接入、观看连接、快照和状态响应 ----
void MjpegServer::acceptLoop()
{
  while (running_.load()) {
    pollfd descriptor{};
    descriptor.fd = listen_fd_;
    descriptor.events = POLLIN;

    // 轮询带超时；监听套接字为非阻塞，停机无需跨线程关闭正在等待的描述符。
    const int ready = ::poll(&descriptor, 1, 500);
    if (ready <= 0) {continue;}
    if ((descriptor.revents & POLLIN) == 0) {continue;}

    sockaddr_in peer_address{};
    socklen_t peer_length = sizeof(peer_address);
    SocketFd fd(::accept(listen_fd_, reinterpret_cast<sockaddr *>(&peer_address), &peer_length));
    // 连接可能在 poll 与 accept 之间被复位；EAGAIN/EWOULDBLOCK 一律回到 poll。
    if (fd < 0) {
      const int accept_error = errno;
      if (accept_error == EMFILE || accept_error == ENFILE ||
        accept_error == ENOBUFS || accept_error == ENOMEM)
      {
        // 此时监听队列仍可读；直接 poll 会忙转。停机通知可立即结束退避。
        std::unique_lock<std::mutex> lock(mutex_);
        cv_.wait_for(lock, std::chrono::milliseconds(100), [&] {return !running_.load();});
      }
      continue;
    }

    char peer[64] = {0};
    if (::inet_ntop(AF_INET, &peer_address.sin_addr, peer, sizeof(peer)) == nullptr) {
      std::snprintf(peer, sizeof(peer), "?");
    }

    int one = 1;
    ::setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one));
    timeval timeout{};
    timeout.tv_sec = 1;
    ::setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));

    std::string request_line;
    bool wants_html = false;
    if (!readRequestHead(fd, request_line, wants_html)) {
      continue;
    }
    const std::string path = pathOf(request_line);

    // 浏览器打开根路径时给观看页；不声明 text/html 的客户端仍接收 MJPEG。
    if (path == "/view" || (path == "/" && wants_html)) {
      serveViewer(fd);
    } else if (path == "/" || path == "/stream.mjpg" || path == "/video" || path == "/mjpeg") {
      addStreamClient(std::move(fd), peer);
    } else if (path == "/snapshot.jpg" || path == "/snapshot" || path == "/jpg") {
      serveSnapshot(std::move(fd));
    } else if (path == "/healthz") {
      serveHealth(fd);
    } else {
      // 未支持路径（含浏览器图标请求）统一返回 404。
      serveNotFound(fd);
    }
  }
}

void MjpegServer::addStreamClient(SocketFd fd, const std::string & peer)
{
  bool full;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!running_.load()) {return;}
    full = clients_.size() >= kMaxStreamClients;
  }
  // 仅 accept 线程添加连接，释放锁后数量只会减少；不能先回复 200 再拒绝。
  if (full) {
    static const char response[] =
      "HTTP/1.1 503 Service Unavailable\r\n"
      "Content-Type: text/plain\r\nConnection: close\r\n"
      "Cache-Control: no-store\r\nRetry-After: 1\r\n"
      "Content-Length: 24\r\n\r\nviewer limit reached: 8\n";
    timeval timeout{1, 0};
    ::setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, &timeout, sizeof(timeout));
    writeAll(fd, response, sizeof(response) - 1);
    return;
  }
  // 在有超时的阻塞模式中写完响应头，再切非阻塞；观看端偏移仅计 multipart 帧数据。
  timeval timeout{};
  timeout.tv_sec = 1;
  ::setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, &timeout, sizeof(timeout));
  if (!writeAll(fd, kStreamHeader, std::strlen(kStreamHeader))) {
    return;
  }

  const int flags = ::fcntl(fd, F_GETFL, 0);
  if (flags < 0 || ::fcntl(fd, F_SETFL, flags | O_NONBLOCK) < 0) {
    // 发送线程持连接锁调用 send，因此必须成功设置非阻塞模式，防止慢客户端堵住提交端。
    std::cout << "[mjpeg] refusing " << peer
              << ": cannot put the socket in non-blocking mode" << std::endl;
    return;
  }

  Client client;
  client.fd = std::move(fd);
  client.peer = peer;
  client.last_progress = std::chrono::steady_clock::now();
  client.socket_bytes_sent = std::strlen(kStreamHeader);
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!running_.load()) {return;}
    // 新接入观看端可以立即共享最近一帧，但不能复用过期画面。
    if (latest_part_ && std::chrono::steady_clock::now() <= latest_valid_until_) {
      client.frame = latest_part_;
      client.sent = 0;
      client.sequence = latest_sequence_;
      client.valid_until = latest_valid_until_;
    }
    clients_.push_back(std::move(client));
    ++wake_revision_;
  }
  accepted_.fetch_add(1);
  cv_.notify_all();
}

void MjpegServer::closeSnapshotLocked(bool discard_pending)
{
  SocketFd fd(std::move(snapshot_fd_));
  if (fd < 0) {return;}
  if (discard_pending) {
    // 半份快照或旧代次不能留在内核继续发送，撤销后由客户端重新请求。
    linger cancel{1, 0};
    ::setsockopt(fd, SOL_SOCKET, SO_LINGER, &cancel, sizeof(cancel));
  }
}

void MjpegServer::serveSnapshot(SocketFd socket)
{
  const int fd = socket;
  std::shared_ptr<const std::vector<unsigned char>> jpeg;
  std::uint64_t generation;
  auto deadline = std::chrono::steady_clock::now() +
    std::chrono::milliseconds(kSnapshotWriteTimeoutMs);
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!running_.load()) {return;}
    snapshot_fd_ = std::move(socket);
    generation = generation_;
    if (std::chrono::steady_clock::now() <= latest_valid_until_) {
      jpeg = latest_jpeg_;
      deadline = std::min(deadline, latest_valid_until_);
    }
  }
  const bool have_jpeg = jpeg && !jpeg->empty();
  const unsigned char * body = have_jpeg ? jpeg->data() :
    reinterpret_cast<const unsigned char *>(kNotFoundBody);
  const std::size_t body_size = have_jpeg ? jpeg->size() : std::strlen(kNotFoundBody);
  const std::string header =
    std::string(have_jpeg ? "HTTP/1.1 200 OK\r\n" : "HTTP/1.1 404 Not Found\r\n") +
    (have_jpeg ? "Content-Type: image/jpeg\r\n" : "Content-Type: text/plain\r\n") +
    "Cache-Control: no-store\r\n"
    "Connection: close\r\n"
    "Content-Length: " + std::to_string(body_size) + "\r\n\r\n";
  const bool complete = writeSnapshot(fd, header, body, body_size, generation, deadline);
  std::lock_guard<std::mutex> lock(mutex_);
  // reset/stop 可能已关闭该 fd；不能再次 close 一个已被进程其他线程复用的数字。
  if (snapshot_fd_ == fd) {closeSnapshotLocked(!complete);}
}

bool MjpegServer::writeSnapshot(
  int fd, const std::string & header, const unsigned char * body, std::size_t body_size,
  std::uint64_t generation, std::chrono::steady_clock::time_point deadline)
{
  std::size_t sent = 0;
  const std::size_t total = header.size() + body_size;
  const auto * header_data = reinterpret_cast<const unsigned char *>(header.data());
  while (true) {
    short events = 0;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (snapshot_fd_ != fd || !running_.load() || generation != generation_ ||
        std::chrono::steady_clock::now() > deadline) {return false;}
      if (sent == total) {
        int queued = -1;
        if (::ioctl(fd, SIOCOUTQ, &queued) != 0 || queued < 0) {return false;}
        // send 接纳全部字节不代表交付完成；全部 ACK 后才能正常关闭并保留完整 JPEG。
        if (queued == 0) {return true;}
        // POLLOUT 在等 ACK 时通常始终就绪；不订阅它，避免此阶段忙轮询。
      } else {
        const bool sending_header = sent < header.size();
        const auto * data = sending_header ? header_data + sent :
          body + (sent - header.size());
        const auto left = sending_header ? header.size() - sent : total - sent;
        const ssize_t written = ::send(
          fd, data, std::min(left, kSnapshotWriteChunkBytes), MSG_NOSIGNAL | MSG_DONTWAIT);
        if (written > 0) {
          sent += static_cast<std::size_t>(written);
          continue;
        }
        if (written < 0 && errno == EINTR) {continue;}
        if (written >= 0 || (errno != EAGAIN && errno != EWOULDBLOCK)) {return false;}
        events = POLLOUT;
      }
    }
    // 仅 poll 允许在锁外触及 fd；返回后先复核 slot，不会对关闭后复用的 fd 再 send/ioctl。
    const auto left = deadline - std::chrono::steady_clock::now();
    if (left <= std::chrono::steady_clock::duration::zero()) {return false;}
    pollfd descriptor{};
    descriptor.fd = fd;
    descriptor.events = events;
    const auto wait_ms = std::chrono::duration_cast<std::chrono::milliseconds>(left).count();
    const auto bounded_wait_ms = std::min<std::int64_t>(
      kSnapshotPollIntervalMs, std::max<std::int64_t>(1, wait_ms));
    const int ready = ::poll(&descriptor, 1, static_cast<int>(bounded_wait_ms));
    if (ready < 0 && errno != EINTR) {return false;}
    if (ready > 0 && (descriptor.revents & (POLLERR | POLLHUP | POLLNVAL))) {return false;}
  }
}

void MjpegServer::serveViewer(int fd)
{
  std::string body(kViewerPage);
  // 浏览器定时器使用毫秒且最大为有符号 32 位；沿用输入新鲜度，不另设用户参数。
  const double timeout_ms = std::min(2147483647.0,
    std::max(1.0, options_.frame_timeout_s * 1000.0));
  const std::string placeholder = "__FRAME_TIMEOUT_MS__";
  body.replace(body.find(placeholder), placeholder.size(), std::to_string(timeout_ms));
  const std::string header =
    "HTTP/1.1 200 OK\r\n"
    "Content-Type: text/html; charset=utf-8\r\n"
    "Cache-Control: no-store\r\n"
    "Connection: close\r\n"
    "Content-Length: " + std::to_string(body.size()) + "\r\n\r\n";
  timeval timeout{};
  timeout.tv_sec = 1;
  ::setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, &timeout, sizeof(timeout));
  if (writeAll(fd, header.data(), header.size())) {
    writeAll(fd, body.data(), body.size());
  }
}

void MjpegServer::serveHealth(int fd)
{
  double age_at_encode_ms;
  double encode_ms;
  {
    std::lock_guard<std::mutex> lock(stats_mutex_);
    age_at_encode_ms = last_age_at_encode_ms_;
    encode_ms = last_encode_ms_;
  }
  const std::string body =
    "ok\nsubmitted=" + std::to_string(submitted_.load()) +
    "\nviewers=" + std::to_string(clientCount()) +
    "\nskipped_backpressure=" + std::to_string(skipped_.load()) +
    "\nframe_age_at_encode_ms=" + std::to_string(age_at_encode_ms) +
    "\njpeg_encode_ms=" + std::to_string(encode_ms) + "\n";
  const std::string header =
    "HTTP/1.1 200 OK\r\n"
    "Content-Type: text/plain\r\n"
    "Connection: close\r\n"
    "Content-Length: " + std::to_string(body.size()) + "\r\n\r\n";
  timeval timeout{};
  timeout.tv_sec = 1;
  ::setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, &timeout, sizeof(timeout));
  if (writeAll(fd, header.data(), header.size())) {
    writeAll(fd, body.data(), body.size());
  }
}

void MjpegServer::serveNotFound(int fd)
{
  timeval timeout{};
  timeout.tv_sec = 1;
  ::setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, &timeout, sizeof(timeout));
  const std::string body(kNotFoundBody);
  const std::string header =
    "HTTP/1.1 404 Not Found\r\n"
    "Content-Type: text/plain\r\n"
    "Connection: close\r\n"
    "Content-Length: " + std::to_string(body.size()) + "\r\n\r\n";
  if (writeAll(fd, header.data(), header.size())) {
    writeAll(fd, body.data(), body.size());
  }
}

// 有界发送和慢连接清理在 send.cpp。

// ---- 阶段失效与帧准入；发送前仍复核有效期 ----
void MjpegServer::reset(std::uint64_t generation)
{
  std::lock_guard<std::mutex> lock(mutex_);
  generation_ = generation;
  rate_limiter_.reset();
  closeSnapshotLocked(true);
  latest_part_.reset();
  latest_jpeg_.reset();
  latest_valid_until_ = {};
  latest_sequence_ = 0;
  ++wake_revision_;
  // 空闲连接保持；已发送部分字节的 multipart 无法安全替换，断开后由客户端重连。
  for (auto it = clients_.begin(); it != clients_.end(); ) {
    int queued = 0;
    if ((it->frame && it->sent > 0) ||
      ::ioctl(it->fd, SIOCOUTQ, &queued) != 0 || queued > 0)
    {
      it = dropClient(it, "stage changed during frame");
    } else {
      it->frame.reset();
      it->retired_frame_end = 0;
      it->retired_frame_valid_until = {};
      ++it;
    }
  }
  // submit_key_ 逐次编码递增且换代不归零，不会命中上一代的编码缓存。
  // 在途编码由最终 generation 检查拒绝；此处不能持连接锁等待编码完成。
  // 累计 submitted_/accepted_/skipped_ 不清零；只清当前阶段的速率窗口与最近编码诊断。
  std::lock_guard<std::mutex> stats_lock(stats_mutex_);
  last_submit_ = {};
  submit_fps_ = 0.0;
  last_age_at_encode_ms_ = last_encode_ms_ = -1.0;
  cv_.notify_all();
}

void MjpegServer::publishJpeg(
  const std::shared_ptr<const std::vector<unsigned char>> & jpeg,
  std::uint64_t generation, std::uint64_t session, std::chrono::steady_clock::time_point valid_until,
  double age_at_encode_ms, double encode_ms, std::int64_t source_stamp_ns)
{
  // 一次拼好 multipart 头、JPEG 和尾部分隔符，所有观看端共享只读字节。
  std::string part;
  part.reserve(jpeg->size() + 384);
  part += "--frame\r\nContent-Type: image/jpeg\r\nContent-Length: ";
  part += std::to_string(jpeg->size());
  part += "\r\nX-Frame-Sequence: ";
  const auto sequence_offset = part.size();
  // 固定宽度占位，在最终准入锁内填序号；JPEG 字节复制仍在锁外完成。
  part.append(20, '0');
  part += "\r\nX-Frame-Age-At-Encode-Ms: " + std::to_string(age_at_encode_ms);
  part += "\r\nX-Jpeg-Encode-Ms: " + std::to_string(encode_ms);
  // source_stamp 只是源身份；帧龄全部来自 steady_clock 有效期，不相减两种时钟。
  part += "\r\nX-Source-Stamp-Ns: " + std::to_string(source_stamp_ns);
  part += "\r\nX-Frame-Age-At-Publish-Ms: ";
  const auto age_offset = part.size();
  part.append(20, '0');
  part += "\r\n\r\n";
  part.append(reinterpret_cast<const char *>(jpeg->data()), jpeg->size());
  part += "\r\n";
  // 先完成唯一的共享 part 分配；分配失败既不占最终发布锁，也不消耗额度。
  auto shared_part = std::make_shared<std::string>(std::move(part));

  {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto committed = std::chrono::steady_clock::now();
    // 编码/拼包期间可能已切阶段或过期，所以提交前必须再次检查，不能只依赖入口处的检查。
    if (session != session_ || generation != generation_ || committed > valid_until || !running_.load() ||
      !rate_limiter_.allow(committed, options_.max_fps)) {return;}
    // 多个生产者可能同时通过入口预检查；限速在最终提交锁内串行复核。
    const auto sequence = submitted_.load() + 1;
    char sequence_text[21];
    std::snprintf(sequence_text, sizeof(sequence_text), "%020llu",
      static_cast<unsigned long long>(sequence));
    shared_part->replace(sequence_offset, 20, sequence_text, 20);
    const double age_at_publish_ms = std::max(0.0, options_.frame_timeout_s * 1000.0 -
      std::chrono::duration<double, std::milli>(valid_until - committed).count());
    char age_text[32];
    std::snprintf(age_text, sizeof(age_text), "%020.6f", age_at_publish_ms);
    shared_part->replace(age_offset, 20, age_text, 20);
    latest_part_ = shared_part;
    latest_jpeg_ = jpeg;
    latest_valid_until_ = valid_until;
    latest_sequence_ = sequence;
    ++wake_revision_;
    {
      std::lock_guard<std::mutex> stats_lock(stats_mutex_);
      if (last_submit_.time_since_epoch().count() != 0) {
        const double dt = std::chrono::duration<double>(committed - last_submit_).count();
        if (dt > 0.0) {
          const double instant = 1.0 / dt;
          submit_fps_ = submit_fps_ > 0.0 ? 0.9 * submit_fps_ + 0.1 * instant : instant;
        }
      }
      last_submit_ = committed;
      last_age_at_encode_ms_ = age_at_encode_ms;
      last_encode_ms_ = encode_ms;
    }
    // 每端最多当前part加一个待ACK帧；背压按源帧计一次，不按观看端累计。
    bool skipped_any = false;
    for (auto & client : clients_) {
      if (client.frame && client.sent > 0) {
        // 已开始发送/仍有 TCP 待确认字节时保留旧 part，其后只保留全局最新 part。
        skipped_any = true;
        continue;
      }
      client.frame = shared_part;
      client.sent = 0;
      client.sequence = sequence;
      client.valid_until = valid_until;
    }
    if (skipped_any) {skipped_.fetch_add(1);}
    submitted_.fetch_add(1);
  }
  cv_.notify_all();
}

// 绘制前共享提交限速状态，避免两套计时器叠加造成额外降帧。
bool MjpegServer::readyForFrame(
  std::uint64_t generation,
  std::chrono::steady_clock::time_point valid_until) const
{
  return admitFrame(generation, valid_until).has_value();
}

std::optional<std::uint64_t> MjpegServer::admitFrame(
  std::uint64_t generation, std::chrono::steady_clock::time_point valid_until) const
{
  if (!running_.load(std::memory_order_relaxed)) {return std::nullopt;}
  std::lock_guard<std::mutex> lock(mutex_);
  const auto now = std::chrono::steady_clock::now();
  if (!running_.load() || generation != generation_ || now > valid_until ||
    !rate_limiter_.ready(now, options_.max_fps)) {return std::nullopt;}
  return session_;
}

void MjpegServer::submit(
  const cv::Mat & bgr, std::uint64_t generation,
  std::chrono::steady_clock::time_point valid_until, std::int64_t source_stamp_ns)
{
  if (!running_.load(std::memory_order_relaxed) || bgr.empty()) {return;}
  const auto started = std::chrono::steady_clock::now();
  if (valid_until.time_since_epoch().count() == 0) {
    valid_until = started + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
      std::chrono::duration<double>(options_.frame_timeout_s));
  }
  // 编码前先限速，跳过的帧不支付像素缩放和 JPEG 编码开销。
  const auto session = admitFrame(generation, valid_until);
  if (!session) {return;}
  encodeAndPublish(bgr, generation, *session, valid_until, source_stamp_ns);
}

void MjpegServer::encodeAndPublish(
  const cv::Mat & bgr, std::uint64_t generation, std::uint64_t session,
  std::chrono::steady_clock::time_point valid_until, std::int64_t source_stamp_ns)
{
  // 本入口没有帧键参数，每次调用都按新帧计数，因此缓存不会在这里命中（共享编码走 submitJpeg()）。
  std::shared_ptr<const std::vector<unsigned char>> jpeg;
  std::chrono::steady_clock::time_point encode_started;
  double encode_ms;
  {
    std::lock_guard<std::mutex> lock(encode_mutex_);
    // 排队期间可能停止、换代、过期或被其他生产者用尽额度；复核不预占额度。
    if (admitFrame(generation, valid_until) != session) {return;}
    encode_started = std::chrono::steady_clock::now();
    jpeg = encoder_.encode(bgr, ++submit_key_, options_.scale, options_.quality);
    encode_ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - encode_started).count();
  }
  if (!jpeg || jpeg->empty()) {return;}
  const double age_at_encode_ms = std::max(0.0, options_.frame_timeout_s * 1000.0 -
    std::chrono::duration<double, std::milli>(valid_until - encode_started).count());
  publishJpeg(jpeg, generation, session, valid_until, age_at_encode_ms, encode_ms, source_stamp_ns);
}

void MjpegServer::submitJpeg(
  std::shared_ptr<const std::vector<unsigned char>> jpeg, std::uint64_t generation,
  std::chrono::steady_clock::time_point valid_until,
  std::chrono::steady_clock::time_point encode_started, double encode_ms,
  std::int64_t source_stamp_ns)
{
  if (!running_.load(std::memory_order_relaxed) || !jpeg || jpeg->empty()) {return;}
  const auto started = std::chrono::steady_clock::now();
  if (valid_until.time_since_epoch().count() == 0) {
    valid_until = started + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
      std::chrono::duration<double>(options_.frame_timeout_s));
  }
  // 预检查不占额度；即使多个生产者同时通过，最终提交仍在同一把锁下消耗。
  const auto session = admitFrame(generation, valid_until);
  if (!session) {return;}
  const double age_at_encode_ms = encode_started.time_since_epoch().count() == 0 ? -1.0 :
    std::max(0.0, options_.frame_timeout_s * 1000.0 -
      std::chrono::duration<double, std::milli>(valid_until - encode_started).count());
  publishJpeg(jpeg, generation, *session, valid_until, age_at_encode_ms, encode_ms, source_stamp_ns);
}

// ---- 只读运行统计；不能代替客户端接收或端到端测量 ----
std::size_t MjpegServer::clientCount() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return clients_.size();
}

double MjpegServer::submitFps() const
{
  // 这是服务接受帧的平滑速率，既非网络完整帧接收率，也非相机／YOLO 吞吐。
  std::lock_guard<std::mutex> lock(stats_mutex_);
  if (last_submit_.time_since_epoch().count() == 0 ||
    std::chrono::duration<double>(std::chrono::steady_clock::now() - last_submit_).count() >
    options_.frame_timeout_s) {return 0.0;}
  return submit_fps_;
}

std::uint64_t MjpegServer::submittedFrames() const
{
  return submitted_.load();
}

std::uint64_t MjpegServer::acceptedClients() const
{
  return accepted_.load();
}

std::uint64_t MjpegServer::skippedForBackpressure() const
{
  return skipped_.load();
}
