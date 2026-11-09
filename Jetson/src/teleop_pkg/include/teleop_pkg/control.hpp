// Copyright 2026 li
// SPDX-License-Identifier: Apache-2.0
#pragma once
#include <array>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <functional>
#include <optional>
#include <utility>

namespace teleop {
// 输入与编解码：三轴为[-1000,1000]整数，disabled时所有字段必须为零。
// claw仅保留UDP兼容；串口由velocity/encode_serial换算实际速度，不发送claw。
struct Control {
  bool enabled = false;
  int forward = 0, left = 0, yaw = 0, claw = 0;
};
bool operator==(const Control& a, const Control& b);
inline bool operator!=(const Control& a, const Control& b) { return !(a == b); }
void validate(const Control& control);
struct Packet {
  std::uint64_t session;
  std::uint32_t sequence;
  Control control;
};
using UdpFrame = std::array<std::uint8_t, 24>;
using AckFrame = std::array<std::uint8_t, 16>;
// Velocity依次为x/y（m/s）、w（rad/s），各轴上限为0.5/0.5/1。
// SerialFrame自持有固定缓冲，仅发送length字节；100Hz编码不分配动态内存。
using Velocity = std::array<double, 3>;
constexpr std::size_t serial_max_line = 24;
struct SerialFrame {
  std::array<std::uint8_t, serial_max_line> bytes{};
  std::size_t length = 0;
  bool moving = false;  // 仅本机停止守卫使用，不发送到STM32。
  const std::uint8_t* data() const { return bytes.data(); }
  std::size_t size() const { return length; }
  auto begin() const { return bytes.begin(); }
  auto end() const { return bytes.begin() + static_cast<std::ptrdiff_t>(length); }
};
using Ack = std::pair<std::uint64_t, std::uint32_t>;
UdpFrame encode_udp(const Packet& packet);
Packet decode_udp(const std::uint8_t* data, std::size_t size);
AckFrame encode_ack(std::uint64_t session, std::uint32_t sequence);
Ack decode_ack(const std::uint8_t* data, std::size_t size);
Velocity velocity(const Control& control);
SerialFrame encode_serial(const Control& control);
Velocity decode_serial(const std::uint8_t* data, std::size_t size);
bool newer(std::uint64_t sequence, std::uint64_t previous, unsigned bits = 32);

// UDP租约：身份由Peer与session组成；now/timeout均为同一单调时钟的秒数。
// IPv4地址保留网络字节序，端口为主机整数；超时旧会话不能重新使能。
struct Peer { std::uint32_t address = 0; std::uint16_t port = 0; };
bool operator==(Peer a, Peer b);
inline bool operator!=(Peer a, Peer b) { return !(a == b); }
class Receiver {
 public:
  explicit Receiver(double timeout = .25);
  bool expired(double now) const;
  Control output(double now);
  /**
   * @brief 校验输入包并按租约/序号受理；非法值抛异常且不改变租约。
   * @param[in] packet session 非零，控制轴范围 [-1000,1000]。
   * @param[in] peer 输入包的 IPv4 地址和端口身份。
   * @param[in] now 同一单调时钟的当前秒数。
   * @param[in] allow_motion false 时后续运动不续租。
   * @return 成功受理返回 true；身份、序号或租约不匹配返回 false。
   */
  bool accept(const Packet& packet, Peer peer, double now, bool allow_motion = true);
 private:
  friend class SerialBridge;  // 解码入口已校验同一 Packet，内部交接不重复检查值域。
  /**
   * @brief 消费已校验包；只处理随时间变化的租约和序号条件。
   * @param[in] packet accept 或 decode_udp 校验成功后的包。
   * @param[in] peer 发送方 IPv4/端口身份。
   * @param[in] now 同一单调时钟的当前秒数。
   * @param[in] allow_motion 本批是否仍允许运动续租。
   * @return 包被受理返回 true；拒绝时不更新输入时间。
   */
  bool accept_validated(const Packet& packet, Peer peer, double now, bool allow_motion);
  /**
   * @brief 沿用当前归属，或在租约过期后以新会话停车换源；本步骤不续租。
   * @param[in] packet 已校验值域且已排除当前过期会话的包。
   * @param[in] peer 发送方 IPv4/端口身份，与 session 共同识别归属。
   * @param[in] now 同一单调时钟的当前秒数。
   * @return 可受理返回 true；换源时保留最近 64 个退役身份并重置序号。
   *         拒绝返回 false 且不改变归属、序号或退役记录。
   */
  bool accept_lease_owner(const Packet& packet, Peer peer, double now);
  double timeout_;
  std::optional<std::pair<Peer, std::uint64_t>> owner_;
  std::optional<std::uint32_t> sequence_;
  std::optional<double> received_at_;
  Control control_;
  std::deque<std::uint64_t> retired_;
};
// 输入门控：不可用或显式停车后，须先松开使能再按下。
class EnableGate {
 public:
  Control apply(Control control, bool available = true, bool stop = false);
 private:
  bool ready_ = false;
};
/**
 * @brief 校验单轴值与死区后进行饱和和取偶数映射。
 * @param[in] value 有限无单位轴值，超出 [-1,1] 时饱和。
 * @param[in] deadzone 有限无单位死区，范围 [0,1)。
 * @return [-1000,1000] 整数；非法输入抛异常，无状态变更。
 */
int axis(double value, double deadzone = .15);
struct Keys {
  bool enable = false, forward = false, back = false, left = false,
       right = false, ccw = false, cw = false, open = false, close = false;
};
Control keyboard(const Keys& keys);
/**
 * @brief 在手柄入口一次校验三轴和共用死区，未使能直接返回零控制。
 * @param[in] forward 前向有限无单位轴值，映射时饱和到 [-1,1]。
 * @param[in] left 左向有限无单位轴值，映射时饱和到 [-1,1]。
 * @param[in] yaw 逆时针有限无单位轴值，映射时饱和到 [-1,1]。
 * @param[in] enabled 是否消费本次轴值。
 * @param[in] opened 张开请求，与 closed 同值时无请求。
 * @param[in] closed 闭合请求，与 opened 同值时无请求。
 * @param[in] deadzone 有限无单位死区，范围 [0,1)。
 * @return 规范化控制量；使能时非法数值抛异常，无状态变更。
 */
Control gamepad(double forward, double left, double yaw, bool enabled,
                bool opened = false, bool closed = false, double deadzone = .15);

// 设备与时钟边界：接口按单线程访问；运行层拥有对象，桥接层借用并负责调用close。
// receive返回一份报文快照；Port::write返回本次提交字节数，不证明STM32已执行。
struct Datagram {
  std::array<std::uint8_t, 256> data{};
  std::size_t size = 0;
  Peer peer;
};
class Channel {
 public:
  virtual ~Channel() = default;
  virtual std::optional<Datagram> receive() = 0;
  virtual void send(const std::uint8_t* data, std::size_t size, Peer peer) = 0;
  virtual void close() noexcept = 0;
};
class Port {
 public:
  virtual ~Port() = default;
  virtual int waiting() = 0;
  virtual std::size_t write(const SerialFrame& frame) = 0;
  virtual void clear() = 0;
  virtual void close() noexcept = 0;
};
using Clock = std::function<double()>;  // 单调秒；客户端ACK期限以原发送时刻为准。
using Stop = std::function<bool()>;
// 串口提交与收尾：UDP和本地手柄共用；借用的Port必须比本对象存活更久。
// 构造先清缓冲并写同步标记；close先同步再尽力写零速，随后关闭Port。
class SerialOutput {
 public:
  SerialOutput(Port& port, Stop stop);
  ~SerialOutput();
  SerialOutput(const SerialOutput&) = delete;
  SerialOutput& operator=(const SerialOutput&) = delete;
  bool ready();  // 检查积压，再由调用方取最新输入，避免查询期间越过输入截止。
  void write(Control control);
  void close() noexcept;
 private:
  void resynchronize();
  Port& port_;
  Stop stop_;
  bool closed_ = false;
};
// UDP批次至多32包、一条速度行；接受停车后，本批后续运动不续租。
// Channel/Port由调用方持有；完整提交且输入仍有效后才回ACK。
class SerialBridge {
 public:
  SerialBridge(Channel& channel, Port& port, Clock clock, Stop stop, double timeout = .25);
  ~SerialBridge();
  SerialBridge(const SerialBridge&) = delete;
  SerialBridge& operator=(const SerialBridge&) = delete;
  void step();
  void close() noexcept;
 private:
  Channel& channel_;
  SerialOutput output_;
  Clock clock_;
  Stop stop_;
  Receiver receiver_;
  bool closed_ = false;
};
// 客户端ACK跟踪：借用Channel，最多保留64次待确认发送；0.5秒超时换会话。
class RemoteLink {
 public:
  RemoteLink(Channel& channel, Peer peer, Clock clock,
             std::function<std::uint64_t()> new_session);
  ~RemoteLink();
  RemoteLink(const RemoteLink&) = delete;
  RemoteLink& operator=(const RemoteLink&) = delete;
  void send(Control control);
  void poll();
  bool available() const;
  void close() noexcept;
  std::uint64_t session() const { return session_; }
  std::uint32_t sequence() const { return sequence_; }
 private:
  void start_session(double now);
  bool expired(double now) const;
  Channel& channel_;
  Peer peer_;
  Clock clock_;
  std::function<std::uint64_t()> new_session_;
  std::uint64_t session_ = 0;
  std::uint32_t sequence_ = 0;
  double started_at_ = 0;
  std::optional<double> ack_time_;
  std::optional<std::uint32_t> last_ack_;
  std::deque<std::pair<std::uint32_t, double>> pending_;
  bool closed_ = false;
};
}  // namespace teleop

// ---- 平台资源 ----
// 职责：单调时钟、发送节拍和信号，不写串口、不发送速度。
// Copyright 2026 li
// SPDX-License-Identifier: Apache-2.0
#include <csignal>
#include <map>
#include <string>
#include <vector>

namespace teleop {
double monotonic();
// 单调秒。正常保持100Hz相位，超期后等待完整一周期，不连发补齐错过的帧。
double next_send(double previous, double now, double period);
std::uint64_t random_session();
// 处理器只置 sig_atomic_t；资源清理由主循环完成，不在信号上下文写串口。
class Signals {
 public:
  Signals();
  ~Signals();
  Signals(const Signals&) = delete;
  Signals& operator=(const Signals&) = delete;
  static bool stopped();
 private:
  using Handler = void (*)(int);
  Handler old_int_, old_term_;
#ifndef _WIN32
  Handler old_hup_, old_pipe_;
#endif
};
class UdpSocket final : public Channel {
 public:
  UdpSocket();
  ~UdpSocket() override;
  UdpSocket(const UdpSocket&) = delete;
  UdpSocket& operator=(const UdpSocket&) = delete;
  static Peer resolve(const std::string& host, int port);
  void bind(Peer peer);
  Peer local() const;
  std::optional<Datagram> receive() override;
  void send(const std::uint8_t* data, std::size_t size, Peer peer) override;
  void close() noexcept override;
 private:
  std::uintptr_t handle_;
};
// 固定选项表，拒绝未知选项、缺值、非有限数和尾随垃圾；支持 --name=value。
class Arguments {
 public:
  Arguments(int argc, char** argv, std::map<std::string, std::string> values,
            const std::vector<std::string>& flags = {});
  const std::string& text(const std::string& key) const;
  int integer(const std::string& key) const;
  double number(const std::string& key) const;
  bool flag(const std::string& key) const;
 private:
  std::map<std::string, std::string> values_;
};
}  // namespace teleop
