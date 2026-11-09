// Copyright 2026 li
// SPDX-License-Identifier: Apache-2.0
#pragma once
#include "teleop_pkg/control.hpp"
#include <string>
#include <termios.h>

namespace teleop {
/**
 * @brief 在 CLI 入口一次把支持的波特率转换为 termios 常量。
 * @param[in] baud 串口 bit/s，仅接受 switch 中列出的 9600..921600 档位。
 * @return 对应 termios 速率常量；不支持时抛异常，不打开设备。
 */
speed_t speed(int baud);
// 持有独占fd，仅主线程访问；与CAN后端共用同设备的flock，锁失败前不改配置/缓冲。
// damiao按CDC波特率直通UART，不发送模块控制命令；文本与输入门控共用。
class SerialPort final : public Port {
 public:
  /**
   * @brief 使用入口已解析的速率取得串口所有权并配置 8N1。
   * @param[in] path 非空 Linux 设备路径。
   * @param[in] usb_baud speed 校验成功后的 termios 常量。
   * @param[in] adapter CLI 已校验的 raw 或 damiao。
   */
  SerialPort(const std::string& path, speed_t usb_baud, const std::string& adapter);
  ~SerialPort() override;
  SerialPort(const SerialPort&) = delete;
  SerialPort& operator=(const SerialPort&) = delete;
  int waiting() override;
  /**
   * @brief 提交 SerialOutput 生成的完整速度行或同步标记。
   * @param[in] bytes 已编码帧，长度为 1..serial_max_line；仅来自内部提交器。
   * @return 本次实际写入字节数，短写不补发整包；I/O 失败抛异常。
   */
  std::size_t write(const SerialFrame& bytes) override;
  void clear() override;
  void close() noexcept override;
 private:
  // 每次写至多20ms；只重试未提交的EINTR/EAGAIN，正数短写绝不补发整包。
  std::size_t write_bytes(const std::uint8_t* data, std::size_t size, bool cancel_on_stop);
  // 持锁限时等待退出零速离开驱动队列；不等于模块发送成功或STM32应答。
  void drain_output();
  void discard_replies();
  int fd_ = -1;
  bool damiao_ = false;
  bool exclusive_ = false;
  bool startup_cleared_ = false;
  bool drain_on_close_ = false;
  bool output_blocked_ = false;
};
}  // namespace teleop
