#include "transport/socketcan/socketcan.hpp"

// ---- CAN 传输实现 ----
// 职责：SocketCAN 的打开、读写和关闭。
// Copyright 2026 li
// SPDX-License-Identifier: Apache-2.0
#include <cerrno>
#include <cstring>
#include <linux/can/raw.h>
#include <net/if.h>
#include <net/if_arp.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>
#include <stdexcept>

// 实现 CanSocket——socket 打开与 bind、接口自检（索引／类型／IFF_UP）、非阻塞读写的错误分类，以及析构关闭。
// 改打开参数或错误分类从本文件进入；重连节奏、背压重试与 dropped/tx/rx 计数由 BridgeRuntime 管理。

CanSocket::~CanSocket()
{
  close();
}

bool CanSocket::validInterfaceName(const std::string & name)
{
  // 与内核接口命名边界一致；嵌入 NUL 会让 ioctl 查到另一个截断后的接口，必须先拒绝。
  return !name.empty() && name.size() < IFNAMSIZ && name != "." && name != ".." &&
         name.find('\0') == std::string::npos &&
         name.find_first_of(" /:\t\r\n\f\v") == std::string::npos;
}

// 只配置当前 socket 的过滤和非阻塞属性；网络接口波特率、UP 状态由部署环境管理。
bool CanSocket::open(const std::string & interface_name, int & index, std::string & error)
{
  error.clear();
  // 每次 open 都先放弃旧 fd；后续失败分支只清理本次资源，不把半初始化 socket 留给节点。
  close();
  index = 0;
  interface_name_ = interface_name;
  // 非阻塞读写不会占住节点回调等待总线；CLOEXEC 避免 fd 跨 exec 泄漏到子程序。
  const int fd = ::socket(PF_CAN, SOCK_RAW | SOCK_NONBLOCK | SOCK_CLOEXEC, CAN_RAW);
  if (fd < 0) {
    interface_up_ = false;
    error = std::strerror(errno);
    return false;
  }
  fd_ = fd;
  if (!check(index, error)) {
    close();
    return false;
  }
  const int receive_own = 0;
  const can_err_mask_t errors = CAN_ERR_MASK;
  // 只放行到达、转向完成、安全区到达、重找与送达确认；旧 0x201/0x202 不接收。
  // 错误帧仍由 CAN_RAW_ERR_FILTER 单独放行；当前 socket 不接收自身发送，其他本机 socket 仍可收到本地回环。
  const auto filters = CanProtocol::commandFilters();
  if (::setsockopt(
      fd_, SOL_CAN_RAW, CAN_RAW_FILTER, filters.data(),
      static_cast<socklen_t>(filters.size() * sizeof(can_filter))) < 0 ||
    ::setsockopt(fd_, SOL_CAN_RAW, CAN_RAW_RECV_OWN_MSGS, &receive_own, sizeof(receive_own)) < 0 ||
    ::setsockopt(fd_, SOL_CAN_RAW, CAN_RAW_ERR_FILTER, &errors, sizeof(errors)) < 0)
  {
    error = std::strerror(errno);
    close();
    return false;
  }
  sockaddr_can address{};
  address.can_family = AF_CAN;
  address.can_ifindex = index;
  if (::bind(fd_, reinterpret_cast<sockaddr *>(&address), sizeof(address)) < 0) {
    error = std::strerror(errno);
    close();
    return false;
  }
  return true;
}

void CanSocket::close()
{
  // 只撤销 fd 所有权，保留最近一次接口自检结果；接口 UP 与 socket 已打开是两个状态。
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
}

// 接口自检：取 index、确认类型是 ARPHRD_CAN、再看 IFF_UP；失败写 error 并返回 false。
bool CanSocket::check(int & index, std::string & error)
{
  interface_up_ = false;
  if (fd_ < 0) {
    error = "CAN socket is not open";
    return false;
  }
  ifreq request{};
  std::memcpy(request.ifr_name, interface_name_.c_str(), interface_name_.size() + 1);
  if (::ioctl(fd_, SIOCGIFINDEX, &request) < 0) {error = std::strerror(errno); return false;}
  index = request.ifr_ifindex;
  if (::ioctl(fd_, SIOCGIFHWADDR, &request) < 0) {error = std::strerror(errno); return false;}
  if (request.ifr_hwaddr.sa_family != ARPHRD_CAN) {
    error = "configured network interface is not CAN";
    return false;
  }
  if (::ioctl(fd_, SIOCGIFFLAGS, &request) < 0) {error = std::strerror(errno); return false;}
  interface_up_ = (request.ifr_flags & IFF_UP) != 0;
  if (!interface_up_) {error = "CAN interface is down";}
  return interface_up_;
}

CanSocket::ReadResult CanSocket::read(can_frame & frame, std::string & error)
{
  error.clear();
  if (fd_ < 0) {
    error = "CAN socket is not open";
    return ReadResult::FAILED;
  }
  const auto size = ::read(fd_, &frame, sizeof(frame));
  // 只有整帧才交给上层解析；短读不能拼补为指令，暂无数据和信号中断也不是致命断线。
  if (size == CAN_MTU) {return ReadResult::FRAME;}
  if (size >= 0) {return ReadResult::SHORT;}
  if (errno == EINTR) {return ReadResult::INTERRUPTED;}
  if (errno == EAGAIN || errno == EWOULDBLOCK) {return ReadResult::EMPTY;}
  error = std::strerror(errno);
  return ReadResult::FAILED;
}

CanSocket::WriteResult CanSocket::write(const can_frame & frame, std::string & error)
{
  error.clear();
  if (fd_ < 0) {
    error = "CAN socket is not open";
    return WriteResult::FAILED;
  }
  const auto size = ::write(fd_, &frame, sizeof(frame));
  if (size == CAN_MTU) {return WriteResult::SENT;}
  if (size >= 0) {
    // CAN 按整帧提交；短写直接失败，不尝试把剩余字节作为另一条报文追加发送。
    error = "incomplete CAN frame submission";
    return WriteResult::FAILED;
  }
  // 返回分类而不内部循环；信号重试也必须占用 Runtime 每轮的八次尝试预算。
  if (errno == EINTR) {return WriteResult::INTERRUPTED;}
  // 内核对发送队列满的回答；调用方保留待发项，等下一个周期或过期丢弃。
  if (errno == EAGAIN || errno == EWOULDBLOCK || errno == ENOBUFS) {
    return WriteResult::BACKPRESSURE;
  }
  error = std::strerror(errno);
  return WriteResult::FAILED;
}
