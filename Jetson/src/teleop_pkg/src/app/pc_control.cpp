// ---- PC 输入实现 ----
// 职责：读取 SDL 并发送速度，含进程入口。
// Copyright 2026 li
// SPDX-License-Identifier: Apache-2.0
#include "teleop_pkg/control.hpp"
#include "sdl/pc_input.hpp"
#include <algorithm>
#include <cmath>
#include <iostream>
#include <memory>
#include <sstream>
#include <stdexcept>

namespace teleop {
/**
 * @brief 校验 PC 调试参数后建立输入窗口和可选 UDP 会话。
 * @param[in] argc 参数个数，至少为 1。
 * @param[in] argv 非空参数数组，元素不得为空指针。
 * @return 0 正常退出；2 参数错误且未初始化 SDL；1 运行错误并清理资源。
 */
int pc_main(int argc, char** argv) {
  std::string host, input; int port; bool preview, list; PadMapping mapping;
  try {
    Arguments args(argc, argv, {{"--host", ""}, {"--port", "8766"}, {"--input", "keyboard"}, {"--joystick", "0"},
      {"--forward-axis", "1"}, {"--left-axis", "0"}, {"--yaw-axis", "2"}, {"--enable-button", "4"},
      {"--open-button", "0"}, {"--close-button", "1"}, {"--deadzone", "0.15"}},
      {"--dry-run", "--list-gamepads", "--invert-forward", "--no-invert-forward", "--invert-left", "--no-invert-left", "--invert-yaw", "--no-invert-yaw"});
    if (args.flag("--help")) {
      std::cout << "pc_control --host ADDRESS [--port 8766] [--input keyboard|gamepad]\n"
                   "           [--dry-run] [--list-gamepads] [--joystick 0]\n"
                   "           [--forward-axis 1] [--left-axis 0] [--yaw-axis 2]\n"
                   "           [--[no-]invert-forward] [--[no-]invert-left] [--[no-]invert-yaw]\n"
                   "           [--enable-button 4] [--open-button 0] [--close-button 1] [--deadzone 0.15]\n"
                   "All three axes default to inverted. Hold enable; release after loss or X stop.\n";
      return 0;
    }
    host = args.text("--host"); port = args.integer("--port"); input = args.text("--input");
    preview = args.flag("--dry-run"); list = args.flag("--list-gamepads");
    if (host.empty() && !preview && !list) throw std::invalid_argument("--host is required unless --dry-run or --list-gamepads");
    if (input != "keyboard" && input != "gamepad") throw std::invalid_argument("--input must be keyboard or gamepad");
    mapping.joystick = args.integer("--joystick");
    mapping.axes = {args.integer("--forward-axis"), args.integer("--left-axis"), args.integer("--yaw-axis")};
    mapping.buttons = {args.integer("--enable-button"), args.integer("--open-button"), args.integer("--close-button")};
    mapping.invert = {!args.flag("--no-invert-forward"), !args.flag("--no-invert-left"), !args.flag("--no-invert-yaw")};
    mapping.deadzone = args.number("--deadzone");
    if (mapping.joystick < 0 || *std::min_element(mapping.axes.begin(), mapping.axes.end()) < 0 ||
        *std::min_element(mapping.buttons.begin(), mapping.buttons.end()) < 0 || port < 1 || port > 65535)
      throw std::invalid_argument("device indexes must be non-negative; port must be 1..65535");
    if (mapping.deadzone < 0 || mapping.deadzone >= 1) throw std::invalid_argument("invalid axis or deadzone");
  } catch (const std::exception& e) { std::cerr << "ERROR " << e.what() << '\n'; return 2; }
  try {
    Signals signals; Sdl sdl;
    if (list) { PcInput::list_gamepads(); return 0; }
    PcInput ui(input == "gamepad", mapping);
    std::unique_ptr<UdpSocket> channel;
    std::unique_ptr<RemoteLink> link;
    if (!preview) {
      channel = std::make_unique<UdpSocket>();
      link = std::make_unique<RemoteLink>(*channel, UdpSocket::resolve(host, port), monotonic, random_session);
    }
    EnableGate gate; auto previous = monotonic(), next_tick = previous;
    while (!Signals::stopped()) {
      const auto frame = ui.sample();
      if (!frame.running) break;
      const auto now = monotonic(); const bool stopped = frame.stopped || now - previous >= .25; previous = now;
      if (link) link->poll();
      bool connected = !link || link->available();
      auto output = gate.apply(frame.control, connected && frame.focused, stopped);
      if (link) {
        try { link->send(output); }
        catch (const std::exception&) { output = gate.apply({}, false); connected = false; }
      }
      ui.draw(output, connected, preview);
      next_tick = next_send(next_tick, monotonic(), .01);
      const auto remaining = next_tick - monotonic();
      if (remaining > 0 && !Signals::stopped()) SDL_Delay(static_cast<Uint32>(std::ceil(remaining * 1000)));
    }
    return 0;  // link先析构，三次disabled之后才关闭设备/SDL；不从异常出口绕过停车。
  } catch (const std::exception& e) { std::cerr << "ERROR teleop stopped: " << e.what() << '\n'; return 1; }
}
}  // namespace teleop

/**
 * @brief 调用与测试共用的 PC 调试入口。
 * @param[in] argc 参数个数，至少为 1。
 * @param[in] argv 非空参数数组，元素不得为空指针。
 * @return 原样返回 pc_main 的退出状态。
 */
int main(int argc, char** argv) { return teleop::pc_main(argc, argv); }
