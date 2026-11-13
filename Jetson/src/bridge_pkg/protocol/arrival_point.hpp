#pragma once

// 类型 21 到点会话与保证 (i) 的写出顺序。不依赖 ROS，可供主机测试单独编译。
// 协议事实见 docs/usb-protocol.md。

#include "protocol/usb_protocol.hpp"

#include <cstdint>
#include <utility>

// 在途重复只丢弃、不刷新截止；完成后的重发只再交命令 5，不再做开合。
enum class ArrivalPointDecision { Discard, Resend, Accept };

struct ArrivalPointSession
{
  enum class State { Idle, InFlight, Replied, Complete };

  ArrivalPointDecision onRequest(std::uint8_t kind)
  {
    if (kind != 1 && kind != 2) {return ArrivalPointDecision::Discard;}
    if (state_ != State::Idle && kind_ == kind) {
      if (state_ == State::InFlight) {return ArrivalPointDecision::Discard;}
      return ArrivalPointDecision::Resend;
    }
    // 夹取完成后同一轮的释放是新请求；在途或已回复未完成时不同 kind 仍丢弃。
    if (state_ != State::Idle && state_ != State::Complete) {return ArrivalPointDecision::Discard;}
    kind_ = kind;
    state_ = State::InFlight;
    return ArrivalPointDecision::Accept;
  }

  void markReplied()
  {
    if (state_ == State::InFlight) {state_ = State::Replied;}
  }

  void markComplete()
  {
    if (state_ == State::Replied || state_ == State::InFlight) {state_ = State::Complete;}
  }

  void reset()
  {
    state_ = State::Idle;
    kind_ = 0;
  }

  State state() const {return state_;}
  std::uint8_t kind() const {return kind_;}

private:
  State state_{State::Idle};
  std::uint8_t kind_{0};
};

// 保证 (i)：同一发送上下文里先提交 alive=1、pause=1、速度为 0 的类型 1，成功后才允许提交回复。
// 背压或中断时不提交回复，由调用方保留待发并在下一拍重试。
template<typename Write>
auto writePauseHeartbeat(Write && write, std::uint8_t & pause) -> decltype(write(std::declval<can_frame>()))
{
  pause = 1;
  return write(CanProtocol::velocity(true, 0, 0, 0, pause));
}
