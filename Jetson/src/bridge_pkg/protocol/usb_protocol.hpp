#pragma once

// ---- 定点量化 ----
// 职责：CAN 载荷的中心与速度定点转换，不访问设备。
// Copyright 2026 li
// SPDX-License-Identifier: Apache-2.0
// CAN 载荷共用的定点量化：中心乘 10000、速度乘 1000，失败时不饱和到边界。
// 纯数值规则，不依赖 ROS 或 linux/can.h；业务状态层与报文编码层都引用这一份。


#include <cmath>
#include <cstdint>
#include <limits>

namespace bridge_scale
{
// 缩放、四舍五入和 int16 范围检查；不饱和到边界，以免把溢出输入冒充合法位置。
static inline bool fixedPoint(double value, double scale, std::int16_t & output)
{
  // 中心乘 10000，速度乘 1000；int16 正负端不对称。失败输出 0，调用方据此整帧置无效。
  output = 0;
  if (!std::isfinite(value) || !std::isfinite(scale) || scale <= 0.0) {return false;}
  // std::round 在正负半整数处都远离零，测试工具必须按此语义而非银行家舍入对照。
  const double rounded = std::round(value * scale);
  // 先舍入再检查完整 int16 范围；失败不会用截断或饱和值冒充有效坐标／速度。
  if (!std::isfinite(rounded) ||
    rounded < std::numeric_limits<std::int16_t>::min() ||
    rounded > std::numeric_limits<std::int16_t>::max())
  {
    return false;
  }
  output = static_cast<std::int16_t>(rounded);
  return true;
}
}  // namespace bridge_scale

// ---- USB 报文 ----
// 职责：周期报文和 STM32 指令的编解码。
// Copyright 2026 li
// SPDX-License-Identifier: Apache-2.0
// 类型号与载荷见 docs/usb-protocol.md。can_id 存放类型号。电机在 STM32 的 CAN1，这里不驱动。


#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <linux/can.h>


class CanProtocol
{
public:
  // 坐标只走类型 3 和 4。类型 3 是目标框几何中心，类型 4 是安全区透视中心。
  // 类型 1 是链路心跳，不是坐标，自动桥也不在这里下发速度。
  // 改类型号或 10000 倍缩放必须同时改 docs/usb-protocol.md。
  // 中心先 round 再检查 int16；量化越界时中心帧置无效并清零载荷，不截断或裁剪。
  static constexpr canid_t VELOCITY_ID = 1;
  static constexpr canid_t MANUAL_ID = 2;
  static constexpr canid_t TARGET_ID = 3;
  static constexpr canid_t SAFETY_ID = 4;
  // 链路超时，与 docs/usb-protocol.md 中的 250 ms 为同一来源。中心接收期限与类型 8 重授间隔都用它。
  static constexpr double LINK_TIMEOUT_S = 0.25;
  static constexpr auto LINK_TIMEOUT = std::chrono::milliseconds(250);
  static constexpr double STM32_CENTER_TIMEOUT_S = LINK_TIMEOUT_S;
  // STM32 → Jetson。未知类型不改变业务状态，也不能绕过转向和搬运确认。
  static constexpr canid_t ARRIVAL_ID = 16;
  static constexpr canid_t TURN_COMPLETED_ID = 17;
  static constexpr canid_t SAFETY_ARRIVED_ID = 18;
  static constexpr canid_t START_SEARCH_ID = 19;
  static constexpr canid_t DELIVERY_COMPLETED_ID = 20;
  static constexpr canid_t ARRIVAL_POINT_ID = 21;
  // 类型 8 只授权已提交路由的去程转向。夹取/释放到点由类型 21 通知，回执为命令 5。
  // STM32 收齐本趟类型 6 通过与类型 8 后转向，完成才发类型 17。
  static constexpr canid_t ARRIVAL_ACK_ID = 5;
  static constexpr canid_t ROI_RESULT_ID = 6;
  static constexpr canid_t COMMAND_RESULT_ID = 7;
  static constexpr canid_t TURN_REQUEST_ID = 8;
  static constexpr std::uint8_t TURN_COMPLETED = 1, SAFETY_ARRIVED = 2, START_SEARCH = 3,
    DELIVERY_COMPLETED = 4, ARRIVAL_POINT = 5;

  // NONE 表示不属于本协议的指令；MALFORMED 表示已识别 ID，但长度或请求字节不合法。
  enum class CommandKind { NONE, MALFORMED, ARRIVAL, MISSION, ARRIVAL_POINT };

  struct Command
  {
    CommandKind kind{CommandKind::NONE};
    // 仅 MISSION 使用，回显 MissionCommand 的命令值，不是识别模式。
    std::uint8_t command{0};
    // 转向完成、安全区到达与投送确认共用本轮ROI请求号低32位。
    std::uint32_t token{0};
  };

  static bool streamId(canid_t id)
  {
    return id == VELOCITY_ID || id == TARGET_ID || id == SAFETY_ID;
  }

  static bool replyId(canid_t id)
  {
    return id == ARRIVAL_ACK_ID || id == ROI_RESULT_ID ||
           id == COMMAND_RESULT_ID || id == TURN_REQUEST_ID;
  }

  static bool transmitId(canid_t id)
  {
    return streamId(id) || replyId(id);
  }

  // 缩放、四舍五入和 int16 范围检查只在共享定点头维护；编码层不再复制数值规则。
  static bool fixedPoint(double value, double scale, std::int16_t & output)
  {
    return bridge_scale::fixedPoint(value, scale, output);
  }

  // 类型 3 和 4 共用：调用方只传这两个类型（运行时由 currentFrame 分派保证）。
  static can_frame center(canid_t id, bool valid, std::int16_t x, std::int16_t y,
    std::uint32_t source_frame = 0)
  {
    // 长度 8：valid、两个小端 int16 坐标、源时间低 24 位毫秒；无效时载荷清零。
    can_frame frame{};
    frame.can_id = id;
    frame.len = 8;
    if (valid) {
      frame.data[0] = 1;
      putSigned(frame.data + 1, x);
      putSigned(frame.data + 3, y);
      frame.data[5] = static_cast<std::uint8_t>(source_frame);
      frame.data[6] = static_cast<std::uint8_t>(source_frame >> 8);
      frame.data[7] = static_cast<std::uint8_t>(source_frame >> 16);
    }
    return frame;
  }

  // 类型 1，长度 8：alive、pause、vx、vy、wz。alive=0 时速度清零。
  // 自动桥在线时 alive=1、速度为 0，pause 取当前值。失联也是这 8 字节。
  static can_frame velocity(
    bool alive, std::int16_t vx, std::int16_t vy, std::int16_t angular, std::uint8_t pause)
  {
    if (pause != 0 && pause != 1) {
      throw std::invalid_argument("pause must be 0 or 1");
    }
    can_frame frame{};
    frame.can_id = VELOCITY_ID;
    frame.len = 8;
    frame.data[0] = alive ? 1 : 0;
    frame.data[1] = pause;
    if (alive) {
      putSigned(frame.data + 2, vx);
      putSigned(frame.data + 4, vy);
      putSigned(frame.data + 6, angular);
    }
    return frame;
  }

  // 到达应答：1 表示已停发坐标并受理，0 表示当前阶段拒绝；不表示 ROI 已通过。
  static can_frame arrivalAck(bool accepted = true)
  {
    can_frame frame{};
    frame.can_id = ARRIVAL_ACK_ID;
    frame.len = 1;
    frame.data[0] = accepted ? 1 : 0;
    return frame;
  }

  // ROI 结果（长度 3）：data[0]=valid，data[1]=passed，data[2]=destination（0 未定／1 左／2 右）。
  // valid=0 时后两字节为 0。passed=0 时 destination 为 0。
  static can_frame roiResult(bool valid, bool passed, std::uint8_t destination)
  {
    // 成功必须指向左/右区；不能编码成STM32会拒绝的“通过但无目的区”。
    // 无效或规则拒绝仍按协议清零后两字节，不消费调用方遗留的目的区。
    if (valid && passed && destination != 1 && destination != 2) {
      throw std::invalid_argument("Passed ROI requires a concrete destination");
    }
    can_frame frame{};
    frame.can_id = ROI_RESULT_ID;
    frame.len = 3;
    if (valid) {
      frame.data[0] = 1;
      if (passed) {
        frame.data[1] = 1;
        frame.data[2] = destination;
      }
    }
    return frame;
  }

  // 命令 1/2/3 成功表示受理，4 成功表示已记录送达，5 表示到点受理（token 恒为 0，不是开合到位）。
  static can_frame commandResult(bool accepted, std::uint8_t command, std::uint32_t token = 0)
  {
    if (command < TURN_COMPLETED || command > ARRIVAL_POINT) {
      throw std::invalid_argument("Unknown mission command");
    }
    can_frame frame{};
    frame.can_id = COMMAND_RESULT_ID;
    frame.len = 6;
    frame.data[0] = accepted ? 1 : 0;
    frame.data[1] = command;
    // 命令 3 与命令 5 没有身份，token 写出为 0；1/2/4 回显请求里的低 32 位。
    if (command != START_SEARCH && command != ARRIVAL_POINT) {putUnsigned(frame.data + 2, token);}
    return frame;
  }

  // 到点由类型 21 通知，回执为命令 5。
  static can_frame turnRequest(std::uint8_t destination, std::uint32_t token)
  {
    if (destination != 1 && destination != 2) {
      throw std::invalid_argument("Turn requires a committed destination");
    }
    can_frame frame{};
    frame.can_id = TURN_REQUEST_ID;
    frame.len = 6;
    frame.data[0] = 1;
    frame.data[1] = destination;
    putUnsigned(frame.data + 2, token);
    return frame;
  }

  // USB 类型为单字节。错误长度或请求字节不改变业务状态。
  static Command parseCommand(const can_frame & frame)
  {
    Command command;
    if (frame.can_id > 0xffU) {
      return command;
    }
    if (frame.can_id == ARRIVAL_ID) {
      command.kind = frame.len == 1 && frame.data[0] == 1 ?
        CommandKind::ARRIVAL : CommandKind::MALFORMED;
      return command;
    }
    if (frame.can_id == TURN_COMPLETED_ID || frame.can_id == SAFETY_ARRIVED_ID ||
      frame.can_id == DELIVERY_COMPLETED_ID)
    {
      command.kind = frame.len == 5 && frame.data[0] == 1 ?
        CommandKind::MISSION : CommandKind::MALFORMED;
      command.command = frame.can_id == TURN_COMPLETED_ID ? TURN_COMPLETED :
        frame.can_id == SAFETY_ARRIVED_ID ? SAFETY_ARRIVED : DELIVERY_COMPLETED;
      if (command.kind == CommandKind::MISSION) {
        command.token = getUnsigned(frame.data + 1);
      }
      return command;
    }
    if (frame.can_id == START_SEARCH_ID)
    {
      command.kind = frame.len == 1 && frame.data[0] == 1 ?
        CommandKind::MISSION : CommandKind::MALFORMED;
      command.command = START_SEARCH;
      return command;
    }
    if (frame.can_id == ARRIVAL_POINT_ID) {
      const bool ok = frame.len == 1 && (frame.data[0] == 1 || frame.data[0] == 2);
      command.kind = ok ? CommandKind::ARRIVAL_POINT : CommandKind::MALFORMED;
      if (ok) {command.command = frame.data[0];}
      return command;
    }
    return command;
  }

private:
  // 32 位回显字段统一小端；编码与解码共用同一实现，避免两侧布局分叉。
  static void putUnsigned(std::uint8_t * destination, std::uint32_t value)
  {
    for (unsigned byte = 0; byte < 4; ++byte) {
      destination[byte] = static_cast<std::uint8_t>(value >> (8 * byte));
    }
  }

  static std::uint32_t getUnsigned(const std::uint8_t * source)
  {
    std::uint32_t value = 0;
    for (unsigned byte = 0; byte < 4; ++byte) {
      value |= static_cast<std::uint32_t>(source[byte]) << (8 * byte);
    }
    return value;
  }

  static void putSigned(std::uint8_t * destination, std::int16_t value)
  {
    // 转为无符号位模式后拆字节，负数的补码也按同一小端布局写出，不依赖主机字节序。
    const auto bits = static_cast<std::uint16_t>(value);
    destination[0] = static_cast<std::uint8_t>(bits & 0xff);
    destination[1] = static_cast<std::uint8_t>(bits >> 8);
  }
};
