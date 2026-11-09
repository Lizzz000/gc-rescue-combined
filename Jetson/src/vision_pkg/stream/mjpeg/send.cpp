// 有界非阻塞发送和慢连接清理。接入、快照和帧准入在 mjpeg.cpp。

#include "stream/mjpeg/mjpeg.hpp"

#include <linux/sockios.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>
#include <algorithm>
#include <chrono>
#include <iostream>
#include <mutex>
#include <vector>

// ---- 有界非阻塞发送与慢连接清理 ----
void MjpegServer::sendLoop()
{
  const auto stall_timeout =
    std::chrono::duration<double>(std::max(1.0, options_.stall_timeout_s));

  std::unique_lock<std::mutex> lock(mutex_);
  std::uint64_t observed_revision = 0;
  while (running_.load()) {
    const bool in_flight = std::any_of(clients_.begin(), clients_.end(),
      [](const Client & client) {return client.frame || client.retired_frame_end != 0;});
    // 在途帧及时推进部分写入与确认检查；没有在途帧时由新帧/新观看端通知唤醒。
    cv_.wait_for(lock, std::chrono::milliseconds(in_flight ? 5 : 200), [&] {
      return !running_.load() || wake_revision_ != observed_revision;
    });
    observed_revision = wake_revision_;
    if (!running_.load()) {break;}

    const auto now = std::chrono::steady_clock::now();
    auto it = clients_.begin();
    while (it != clients_.end()) {
      // 读方向 EOF 只表示客户端关闭写端，仍可继续收图；发送错误才判定发送失败。
      // 无帧期间也用发送进度超时回收连接，避免断流无限保留文件描述符。
      if (now - it->last_progress > stall_timeout) {
        it = dropClient(it, it->frame ? "stalled" : "idle");
        continue;
      }
      int queued = -1;
      if (it->retired_frame_end != 0) {
        if (::ioctl(it->fd, SIOCOUTQ, &queued) != 0 || queued < 0 ||
          static_cast<std::uint64_t>(queued) > it->socket_bytes_sent)
        {
          it = dropClient(it, "cannot inspect retired frame acknowledgement");
          continue;
        }
        // ACK 水位先回收已确认帧，再检查仍未确认帧的原截止时间；新帧不能为旧帧续期。
        if (!it->advanceAcknowledgement(
            it->socket_bytes_sent - static_cast<std::uint64_t>(queued), now)) {
          it = dropClient(it, "expired unacknowledged frame");
          continue;
        }
      }
      if (it->frame && it->sent == it->frame->size()) {
        int not_sent = 0;
        if ((queued < 0 && ::ioctl(it->fd, SIOCOUTQ, &queued) != 0) ||
          ::ioctl(it->fd, SIOCOUTQNSD, &not_sent) != 0 || queued < 0 || not_sent < 0 ||
          not_sent > queued || static_cast<std::uint64_t>(queued) > it->socket_bytes_sent)
        {
          it = dropClient(it, "cannot inspect TCP backlog");
          continue;
        }
        if (queued > 0 && now > it->valid_until) {
          it = dropClient(it, "expired unacknowledged part");
          continue;
        }
        // 最多一个已交给 TCP 的待确认帧加一个当前 part。旧水位未回收时不能退休第二帧，
        // 因而不会领取第三帧；只保留全局最新候选。此处不是浏览器已收到或已显示的证明。
        it->retireTransmittedFrame(queued, not_sent);
      }
      if (!it->frame && latest_part_ && latest_sequence_ > it->sequence &&
        now <= latest_valid_until_)
      {
        // 在途帧完成后只领取缓存里的最新 part；跳过其间所有帧，不重复领取同一序号。
        it->frame = latest_part_;
        it->sequence = latest_sequence_;
        it->sent = 0;
        it->valid_until = latest_valid_until_;
      }
      if (!it->frame) {
        ++it;
        continue;
      }
      if (now > it->valid_until) {
        // 未发出任何字节可直接丢帧；半帧则断开连接，防止下一帧被客户端当成旧 JPEG 的剩余内容。
        if (it->sent > 0) {
          it = dropClient(it, "frame expired during transmission");
        } else {
          it->frame.reset();
          ++it;
        }
        continue;
      }
      const std::string & data = *it->frame;
      const std::size_t left = data.size() - it->sent;
      if (left == 0) {
        ++it;
        continue;
      }
      // 观看端已设为非阻塞；写不动就保留当前偏移，下一轮继续，不在此等待慢客户端。
      const ssize_t written = ::send(it->fd, data.data() + it->sent, left, MSG_NOSIGNAL);

      if (written > 0) {
        if (static_cast<std::uint64_t>(written) >
          std::numeric_limits<std::uint64_t>::max() - it->socket_bytes_sent) {
          it = dropClient(it, "TCP byte watermark overflow");
          continue;
        }
        it->sent += static_cast<std::size_t>(written);
        it->socket_bytes_sent += static_cast<std::uint64_t>(written);
        it->last_progress = now;
        ++it;
      } else if (written < 0 &&
        (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR))
      {
        // 暂时写不动时保留当前帧和偏移；下一次新帧只跳过该端，其余观看端继续发送。
        ++it;
      } else {
        it = dropClient(it, "closed");
      }
    }
  }
}

std::vector<MjpegServer::Client>::iterator MjpegServer::dropClient(
  std::vector<Client>::iterator it, const char * why)
{
  if (it->fd >= 0) {
    // 半帧过期/换阶段时必须撤销内核剩余字节；普通 close 仍可能继续发送旧队列。
    linger discard{1, 0};
    ::setsockopt(it->fd, SOL_SOCKET, SO_LINGER, &discard, sizeof(discard));
    it->fd.reset();
  }
  std::cout << "[mjpeg] dropped viewer " << it->peer << " (" << why << ")" << std::endl;
  return clients_.erase(it);
}
