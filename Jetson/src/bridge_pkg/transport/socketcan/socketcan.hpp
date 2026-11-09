#pragma once

#include <string>
#include <linux/can.h>

#include "transport/transport.hpp"

// SocketCAN 原始套接字的 RAII 封装（CanTransport 的生产实现）——打开与绑定、接口自检、非阻塞收发的错误分类、关闭
//       不依赖 ROS，可在无 ROS 环境单测。
// 打开参数与读写错误分类在本文件的 CAN 传输实现；业务门控交给 BridgeState，会话调度与计数交给 BridgeRuntime。

// 只管一个 fd 的生命周期与一次收发的成败；重连节奏、背压重试与状态发布由调用方决定。
// ReadResult／WriteResult 继承自 CanTransport，CanSocket::ReadResult 等写法保持可用。
class CanSocket : public CanTransport
{
public:
  CanSocket() = default;
  ~CanSocket() override;
  CanSocket(const CanSocket &) = delete;
  CanSocket & operator=(const CanSocket &) = delete;

  /// 接口名是否可用：非空且短于 ifreq.ifr_name 的定长（IFNAMSIZ）。参数校验与 open() 共用同一判据。
  static bool validInterfaceName(const std::string & name);

  /// 打开并绑定接口。socket／接口自检／setsockopt／bind 任一步失败都返回 false 并写 error，且不留下 fd；
  /// 打开失败不改变网络接口状态，调用方按自己的节奏重试。index 是SocketCAN接口索引，用于判断接口是否被替换。
  bool open(const std::string & interface_name, int & index, std::string & error) override;
  /// 关闭 fd，已关闭时什么也不做。不改变网络接口状态。
  void close() override;
  bool socketOpen() const override {return fd_ >= 0;}

  /// 接口自检：取 index、确认类型是 ARPHRD_CAN、再看 IFF_UP。失败写 error 并返回 false。
  /// 无论成败都会刷新 interfaceUp()，与 open() 共用同一份状态。
  bool check(int & index, std::string & error) override;
  /// 最近一次自检看到的 IFF_UP；与 socketOpen() 相互独立，都不代表对端 STM32 在线。
  bool interfaceUp() const override {return interface_up_;}

  /// 每次只调用一次系统收发；EINTR 交给调用方，在本周期的次数预算内重试。
  ReadResult read(can_frame & frame, std::string & error) override;
  WriteResult write(const can_frame & frame, std::string & error) override;

private:
  std::string interface_name_;
  int fd_{-1};
  bool interface_up_{false};
};
