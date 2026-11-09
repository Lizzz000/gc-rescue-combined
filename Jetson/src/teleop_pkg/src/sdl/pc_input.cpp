// ---- PC 输入实现 ----
// 职责：读取 SDL 并发送速度，含进程入口。
// Copyright 2026 li
// SPDX-License-Identifier: Apache-2.0
#include "sdl/pc_input.hpp"
#include <algorithm>
#include <cmath>
#include <iostream>
#include <memory>
#include <sstream>
#include <stdexcept>

namespace teleop {
namespace {
void sdl_check(bool valid, const char* operation) {
  if (!valid) throw std::runtime_error(std::string(operation) + ": " + SDL_GetError());
}
// 小型等宽字形只画状态与操作提示，避免为调试窗口引入字体库/字体文件。
void text(SDL_Renderer* renderer, int x, int y, const std::string& value) {
  static const std::string alphabet = "ABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789+-:/.()";
  static constexpr std::uint32_t glyphs[] = {
    0x1d18fe31,0x3d1f463e,0x1d18422e,0x3d18c63e,0x3f08721f,0x3f087210,
    0x1d185e2e,0x231fc631,0x3e42109f,0x084210ae,0x232e4a31,0x2108421f,
    0x23bac631,0x239ace31,0x1d18c62e,0x3d18fa10,0x1d18d64d,0x3d18fa51,
    0x1f08383e,0x3e421084,0x2318c62e,0x2318c544,0x2318d771,0x22a22a31,
    0x22a21084,0x3e22221f,0x1d19d62e,0x08c2108e,0x1d11111f,0x3c11143e,
    0x2327c421,0x3f08783e,0x1d08762e,0x3e222108,0x1d17462e,0x1d18bc2e,
    0x004f9080,0x000f8000,0x00401080,0x02222210,0x00000084,0x04421042,0x10421088
  };
  for (char c : value) {
    const auto at = alphabet.find(c);
    if (at != std::string::npos) {
      const auto glyph = glyphs[at];
      for (int row = 0; row < 6; ++row) for (int col = 0; col < 5; ++col)
        if (glyph & (1u << (29 - row * 5 - col))) {
          const SDL_Rect pixel{x + col * 3, y + row * 3, 3, 3};
          SDL_RenderFillRect(renderer, &pixel);
        }
    }
    x += 18;
  }
}
}  // namespace
Sdl::Sdl() {
    SDL_SetMainReady();
    sdl_check(SDL_Init(SDL_INIT_VIDEO | SDL_INIT_JOYSTICK) == 0, "SDL_Init");
}
Sdl::~Sdl() { SDL_Quit(); }
PcInput::PcInput(bool gamepad_input, PadMapping mapping) : mapping_(mapping) {
  try {
    window_ = SDL_CreateWindow("GC Rescue - manual serial control", SDL_WINDOWPOS_CENTERED,
                              SDL_WINDOWPOS_CENTERED, 800, 420, SDL_WINDOW_SHOWN);
    sdl_check(window_ != nullptr, "create window");
    renderer_ = SDL_CreateRenderer(window_, -1, 0);
    sdl_check(renderer_ != nullptr, "create renderer");
    if (gamepad_input) {
      device_ = SDL_JoystickOpen(mapping_.joystick);
      sdl_check(device_ != nullptr, "open selected gamepad");
      instance_ = SDL_JoystickInstanceID(device_);
      // 负下标已在 pc_main 入口拒绝，这里只查设备能力上限。
      if (*std::max_element(mapping_.axes.begin(), mapping_.axes.end()) >= SDL_JoystickNumAxes(device_) ||
          *std::max_element(mapping_.buttons.begin(), mapping_.buttons.end()) >= SDL_JoystickNumButtons(device_))
        throw std::invalid_argument("gamepad mapping out of range; check --list-gamepads");
    }
  } catch (...) {
    if (device_) SDL_JoystickClose(device_);
    if (renderer_) SDL_DestroyRenderer(renderer_);
    if (window_) SDL_DestroyWindow(window_);
    throw;
  }
}
PcInput::~PcInput() {
  if (device_) SDL_JoystickClose(device_);
  SDL_DestroyRenderer(renderer_); SDL_DestroyWindow(window_);
}
Control PcInput::read_pad() {
  if (!SDL_JoystickGetAttached(device_)) throw std::runtime_error("selected gamepad disconnected; restart after checking its index");
  SDL_ClearError();
  std::array<double, 3> values{};
  std::array<bool, 3> buttons{};
  for (unsigned i = 0; i < 3; ++i) {
    // SDL与pygame都按32768归一化，正端32767不额外放大。
    values[i] = SDL_JoystickGetAxis(device_, mapping_.axes[i]) / 32768.0 * (mapping_.invert[i] ? -1 : 1);
    buttons[i] = SDL_JoystickGetButton(device_, mapping_.buttons[i]) != 0;
  }
  sdl_check(SDL_GetError()[0] == '\0' && SDL_JoystickGetAttached(device_), "read gamepad");
  return gamepad(values[0], values[1], values[2], buttons[0], buttons[1], buttons[2], mapping_.deadzone);
}
InputFrame PcInput::sample() {
  InputFrame frame;
  SDL_Event event;
  while (SDL_PollEvent(&event)) {
    if (event.type == SDL_QUIT || (event.type == SDL_KEYDOWN && event.key.keysym.sym == SDLK_ESCAPE)) frame.running = false;
    if ((event.type == SDL_WINDOWEVENT && event.window.event == SDL_WINDOWEVENT_FOCUS_LOST) ||
        (event.type == SDL_KEYDOWN && event.key.keysym.sym == SDLK_x)) frame.stopped = true;
    if (event.type == SDL_JOYDEVICEREMOVED && device_ && event.jdevice.which == instance_)
      throw std::runtime_error("selected gamepad removed; restart after checking its index");
  }
  if (!frame.running) return frame;
  const auto* keys = SDL_GetKeyboardState(nullptr);
  frame.stopped |= keys[SDL_SCANCODE_X] != 0;
  frame.focused = (SDL_GetWindowFlags(window_) & SDL_WINDOW_INPUT_FOCUS) != 0;
  frame.control = device_ ? read_pad() : keyboard({keys[SDL_SCANCODE_SPACE] != 0,
    keys[SDL_SCANCODE_W] != 0, keys[SDL_SCANCODE_S] != 0, keys[SDL_SCANCODE_A] != 0,
    keys[SDL_SCANCODE_D] != 0, keys[SDL_SCANCODE_Q] != 0, keys[SDL_SCANCODE_E] != 0,
    keys[SDL_SCANCODE_O] != 0, keys[SDL_SCANCODE_C] != 0});
  frame.control.claw = 0;  // 当前只发送速度，开合按键留待后续加入。
  return frame;
}
void PcInput::draw(Control control, bool connected, bool preview) {
  SDL_SetRenderDrawColor(renderer_, 19, 27, 40, 255); SDL_RenderClear(renderer_);
  SDL_SetRenderDrawColor(renderer_, 228, 235, 245, 255);
  text(renderer_, 24, 24, "GC RESCUE / MANUAL CONTROL");
  SDL_SetRenderDrawColor(renderer_, connected ? 85 : 240, connected ? 215 : 155, 160, 255);
  text(renderer_, 24, 70, preview ? "PREVIEW ONLY / NO NETWORK" : connected ? "JETSON SERIAL SUBMITTED" : "WAITING FOR JETSON");
  SDL_SetRenderDrawColor(renderer_, 228, 235, 245, 255);
  text(renderer_, 24, 116, std::string("ENABLE ") + (control.enabled ? "ON" : "OFF") + " / VELOCITY");
  const auto speed = velocity(control);
  text(renderer_, 24, 162, "X " + std::to_string(speed[0]) + " Y " + std::to_string(speed[1]) + " W " + std::to_string(speed[2]));
  text(renderer_, 24, 218, "SPACE ENABLE / WS AD QE MOVE");
  text(renderer_, 24, 260, "X STOP / ESC EXIT");
  text(renderer_, 24, 302, "AFTER LOSS: RELEASE THEN ENABLE");
  text(renderer_, 24, 356, "ACK IS NOT STM32 EXECUTION");
  SDL_RenderPresent(renderer_);
}
void PcInput::list_gamepads() {
  const auto count = SDL_NumJoysticks(); sdl_check(count >= 0, "list gamepads");
  if (!count) std::cout << "No gamepads found\n";
  for (int i = 0; i < count; ++i) {
    std::unique_ptr<SDL_Joystick, decltype(&SDL_JoystickClose)> device(SDL_JoystickOpen(i), SDL_JoystickClose);
    sdl_check(device != nullptr, "inspect gamepad");
    char guid[33]{}; SDL_JoystickGetGUIDString(SDL_JoystickGetGUID(device.get()), guid, sizeof(guid));
    const char* name = SDL_JoystickName(device.get());
    std::cout << i << ": " << (name ? name : "unknown") << ", axes=" << SDL_JoystickNumAxes(device.get())
              << ", buttons=" << SDL_JoystickNumButtons(device.get()) << ", guid=" << guid << '\n';
  }
}
}  // namespace teleop
