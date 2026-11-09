// 串口 CLI 与本地输入合并；测试库只编译输入实现。
// Copyright 2026 li
// SPDX-License-Identifier: Apache-2.0
#include "teleop_pkg/control.hpp"
#include "hidraw/beitong.hpp"
#include "termios/serial_port.hpp"
#include <memory>
#include <optional>
#include <algorithm>
#include <cerrno>
#include <cmath>
#include <cstring>
#include <fcntl.h>
#include <iostream>
#include <poll.h>
#include <stdexcept>
#include <sys/file.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>

/**
 * @brief 先完成 CLI 参数校验，再创建输入和串口资源并运行发送循环。
 * @param[in] argc 命令行参数个数，至少为 1。
 * @param[in] argv 非空参数数组，每个有效元素为非空指针。
 * @return 0 正常退出；2 参数错误且未打开设备；1 运行失败并执行收尾。
 */
int main(int argc, char** argv) {
  using namespace teleop;
  std::string device, bind, input, gamepad, adapter;
  bool preview = false;
  int baud, port; speed_t usb_baud{}; double rate, timeout;
  try {
    Arguments args(argc, argv, {{"--serial-port", ""}, {"--serial-adapter", "damiao"}, {"--baud", "115200"}, {"--bind", "127.0.0.1"},
                               {"--port", "8766"}, {"--rate", "100"}, {"--timeout", "0.25"},
                               {"--input", "udp"}, {"--gamepad", ""}}, {"--dry-run"});
    if (args.flag("--help")) {
      std::cout << "serial_control --serial-port PATH [--baud 115200] [--bind 127.0.0.1]\n"
                   "               [--port 8766] [--rate 100] [--timeout 0.25] [--serial-adapter raw|damiao]\n"
                   "serial_control --input gamepad --gamepad HIDRAW --serial-port PATH\n"
                   "               [--baud 115200] [--rate 100] [--timeout 0.25] [--serial-adapter raw|damiao]\n"
                   "serial_control --input gamepad --gamepad HIDRAW --dry-run\n"
                   "BEITONG 20bc:5159: left stick translation, right X yaw, hold LB to enable, R3 stop; x/y +/-0.5m/s, w +/-1rad/s.\n"
                   "--baud sets UART and CDC speed; damiao (tested V3) is the default adapter.\n"
                   "Damiao UART requires exclusive time-sharing with CAN; no automatic detection or fallback.\n"
                   "Independent UDP or local HID -> STM32 serial bridge, 8N1; no ROS node.\n";
      return 0;
    }
    device = args.text("--serial-port"); bind = args.text("--bind"); adapter = args.text("--serial-adapter");
    if (adapter != "raw" && adapter != "damiao")
      throw std::invalid_argument("--serial-adapter must be raw or damiao");
    input = args.text("--input"); gamepad = args.text("--gamepad"); preview = args.flag("--dry-run");
    if (input != "udp" && input != "gamepad") throw std::invalid_argument("--input must be udp or gamepad");
    if (input == "gamepad" && gamepad.empty())
      throw std::invalid_argument("--gamepad HIDRAW is required for local input");
    if (!preview && device.empty()) throw std::invalid_argument("--serial-port is required");
    // 预览只打开HID，不创建设备端口；适配器选项不影响预览。
    baud = args.integer("--baud"); usb_baud = speed(baud);
    port = args.integer("--port"); rate = args.number("--rate"); timeout = args.number("--timeout");
    if (port < 1 || port > 65535 || rate < 5 || rate > 100)
      throw std::invalid_argument("port must be 1..65535; rate must be 5..100 Hz");
    // 输入超时范围只在入口校验一次（Receiver/LocalPadInput 构造不再重复）。
    if (!std::isfinite(timeout) || timeout < .05 || timeout > 2)
      throw std::invalid_argument("command timeout must be 0.05..2 seconds");
    if (timeout <= 1 / rate)
      throw std::invalid_argument("input timeout must be longer than the serial send period (1 / rate)");
  } catch (const std::exception& e) { std::cerr << "ERROR " << e.what() << '\n'; return 2; }
  try {
    Signals signals;
    // 本地HID生命周期：source/serial持有设备，pad/output借用；逆序析构先同步停车。
    if (input == "gamepad") {
      BeitongHid source(gamepad);
      LocalPadInput pad(source, monotonic, Signals::stopped, timeout);
      std::unique_ptr<SerialPort> serial;
      std::unique_ptr<SerialOutput> output;
      if (!preview) {
        serial = std::make_unique<SerialPort>(device, usb_baud, adapter);
        output = std::make_unique<SerialOutput>(*serial, Signals::stopped);
      }
      std::cout << "BEITONG " << gamepad << (preview ? " DRY RUN" : " -> " + device)
                << (preview ? "" : "; " + adapter + ", UART " + std::to_string(baud) + " 8N1")
                << "; " << rate << " Hz, timeout " << timeout << " s; release LB and center sticks before enable\n" << std::flush;
      double next_tick = monotonic(), printed_at = 0;
      std::optional<Control> printed;
      bool connected = false, ready = false;
      // 报告采样与串口发送分开计时；积压查询后再复核输入期限，超期不追赶补发。
      while (!Signals::stopped()) {
        pad.sample();
        if (output && !output->ready()) break;
        const auto control = pad.current();
        if (output) output->write(control);
        const auto now = monotonic();
        if (!printed || pad.connected() != connected || pad.ready() != ready ||
            control.enabled != printed->enabled || (preview && control != *printed) || now - printed_at >= 1) {
          connected = pad.connected(); ready = pad.ready(); printed = control; printed_at = now;
          const auto speed = velocity(control);
          std::cout << "reports=" << pad.reports() << " connected=" << connected << " ready=" << ready
                    << " enabled=" << control.enabled << " x=" << speed[0] << " y=" << speed[1]
                    << " w=" << speed[2] << '\n' << std::flush;
        }
        next_tick = next_send(next_tick, monotonic(), 1 / rate);
        poll(nullptr, 0, static_cast<int>(std::ceil(std::max(0.0, next_tick - monotonic()) * 1000)));
      }
      return 0;  // output先停车/关闭串口，最后释放只读HID；异常出口沿用同一RAII收尾。
    }
    // UDP生命周期：bridge借用channel/serial；每轮提交与ACK受同一停止守卫约束。
    UdpSocket channel; channel.bind(UdpSocket::resolve(bind, port));
    SerialPort serial(device, usb_baud, adapter);
    SerialBridge bridge(channel, serial, monotonic, Signals::stopped, timeout);
    std::cout << "UDP " << bind << ':' << port << " -> " << device << ", " << baud
              << " UART 8N1; " << adapter << "; "
              << rate << " Hz, timeout " << timeout << " s\n" << std::flush;
    double next_tick = monotonic();
    while (!Signals::stopped()) {
      bridge.step();
      if (Signals::stopped()) break;
      // 不追赶错过的周期；poll可由信号中断，避免退出后额外休眠。
      next_tick = next_send(next_tick, monotonic(), 1 / rate);
      const auto wait = std::max(0.0, next_tick - monotonic());
      poll(nullptr, 0, static_cast<int>(std::ceil(wait * 1000)));
    }
    return 0;
  } catch (const std::exception& e) {
    std::cerr << "ERROR " << e.what() << '\n';
    return Signals::stopped() ? 0 : 1;
  }
}
