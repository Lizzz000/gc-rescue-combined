// ---- PC 输入声明 ----
// 职责：SDL 键盘和手柄映射，不包含进程入口。
// Copyright 2026 li
// SPDX-License-Identifier: Apache-2.0
#pragma once
#include "teleop_pkg/control.hpp"
#include <SDL.h>
#include <array>
#include <string>

namespace teleop {
struct PadMapping {
  int joystick = 0;
  std::array<int, 3> axes{1, 0, 2};
  std::array<int, 3> buttons{4, 0, 1};
  std::array<bool, 3> invert{true, true, true};
  double deadzone = .15;
};
struct InputFrame { Control control; bool focused = false, stopped = false, running = true; };
// 窗口、SDL设备及事件均由UI主线程独占；用instance ID匹配移除事件，不重开旧下标。
class PcInput {
 public:
  PcInput(bool gamepad, PadMapping mapping);
  ~PcInput();
  PcInput(const PcInput&) = delete;
  PcInput& operator=(const PcInput&) = delete;
  InputFrame sample();
  void draw(Control control, bool connected, bool preview);
  static void list_gamepads();
 private:
  Control read_pad();
  PadMapping mapping_;
  SDL_Window* window_ = nullptr;
  SDL_Renderer* renderer_ = nullptr;
  SDL_Joystick* device_ = nullptr;
  SDL_JoystickID instance_ = -1;
};
// SDL 初始化守卫：由 pc_control 入口在 UI 生命周期内持有。
struct Sdl {
  Sdl();
  ~Sdl();
};
}  // namespace teleop
