// 控制协议与平台资源合在一个实现文件里。
// ---- 控制协议实现 ----
// 职责：速度文本、套接字和手柄映射。
// Copyright 2026 li
// SPDX-License-Identifier: Apache-2.0
#include "teleop_pkg/control.hpp"
#include <algorithm>
#include <charconv>
#include <cmath>
#include <cstring>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <tuple>

namespace teleop {
namespace {
// UDP字段按小端逐字节读写，不依赖结构体布局或未对齐访问。
void require(bool valid, const char* message) {
  if (!valid) throw std::invalid_argument(message);
}
void put(std::uint8_t* p, std::uint64_t value, unsigned bytes) {
  for (unsigned i = 0; i < bytes; ++i) p[i] = static_cast<std::uint8_t>(value >> (8 * i));
}
std::uint64_t get(const std::uint8_t* p, unsigned bytes) {
  std::uint64_t result = 0;
  for (unsigned i = 0; i < bytes; ++i) result |= std::uint64_t(p[i]) << (8 * i);
  return result;
}
int signed16(const std::uint8_t* p) {
  const auto v = int(get(p, 2));
  return v < 32768 ? v : v - 65536;
}
void put_control(std::uint8_t* p, const Control& c) {
  p[0] = c.enabled; p[1] = static_cast<std::uint8_t>(c.claw);
  put(p + 2, static_cast<std::uint16_t>(c.forward), 2);
  put(p + 4, static_cast<std::uint16_t>(c.left), 2);
  put(p + 6, static_cast<std::uint16_t>(c.yaw), 2);
}
Control get_control(const std::uint8_t* p) {
  require(p[0] <= 1, "enabled must be 0 or 1");
  Control c{p[0] != 0, signed16(p + 2), signed16(p + 4), signed16(p + 6), p[1]};
  validate(c); return c;
}
void frame(const std::uint8_t* p, std::size_t size, std::size_t expected) {
  require(p && size == expected, "invalid frame length");
}
int claw(bool opened, bool closed) { return opened == closed ? 0 : opened ? 1 : 2; }
}  // namespace

// 输入校验与兼容UDP/ACK：归一化整数和claw保留在网络侧。
bool operator==(const Control& a, const Control& b) {
  return std::tie(a.enabled, a.forward, a.left, a.yaw, a.claw) ==
         std::tie(b.enabled, b.forward, b.left, b.yaw, b.claw);
}
void validate(const Control& c) {
  require(c.forward >= -1000 && c.forward <= 1000 && c.left >= -1000 && c.left <= 1000 &&
          c.yaw >= -1000 && c.yaw <= 1000 && c.claw >= 0 && c.claw <= 2, "control out of range");
  require(c.enabled || (!c.forward && !c.left && !c.yaw && !c.claw), "disabled must be all zero");
}
UdpFrame encode_udp(const Packet& p) {
  // session 由 RemoteLink 保证非零，值域由本地映射保证，编码不再重复校验。
  UdpFrame out{}; std::memcpy(out.data(), "RMT1", 4);
  put(out.data() + 4, p.session, 8); put(out.data() + 12, p.sequence, 4);
  put_control(out.data() + 16, p.control); return out;
}
Packet decode_udp(const std::uint8_t* p, std::size_t size) {
  frame(p, size, 24); require(std::memcmp(p, "RMT1", 4) == 0, "UDP magic mismatch");
  const auto session = get(p + 4, 8); require(session != 0, "session must be nonzero");
  return {session, static_cast<std::uint32_t>(get(p + 12, 4)), get_control(p + 16)};
}
AckFrame encode_ack(std::uint64_t session, std::uint32_t sequence) {
  AckFrame out{}; std::memcpy(out.data(), "RMA1", 4);
  put(out.data() + 4, session, 8); put(out.data() + 12, sequence, 4); return out;
}
Ack decode_ack(const std::uint8_t* p, std::size_t size) {
  frame(p, size, 16); require(std::memcmp(p, "RMA1", 4) == 0, "ACK magic mismatch");
  const auto session = get(p + 4, 8); require(session != 0, "session must be nonzero");
  return {session, static_cast<std::uint32_t>(get(p + 12, 4))};
}
// 实际速度与文本codec：X/Y各±0.5m/s，W为±1rad/s；逐轴换算，不限制合成平移速度。
Velocity velocity(const Control& c) {
  return {c.forward / 2000.0, c.left / 2000.0, c.yaw / 1000.0};
}
SerialFrame encode_serial(const Control& c) {
  // 输入已在解码入口或本地映射处规范化，此处不再重复 validate。
  SerialFrame out;
  // 以万分之一速度为整数格式化：线速度每摇杆单位0.0005m/s，角速度0.001rad/s。
  // 整数运算避免浮点格式的区域设置、科学记数法和舍入噪声。
  const std::array<int, 3> ticks{c.forward * 5, c.left * 5, c.yaw * 10};
  for (unsigned i = 0; i < ticks.size(); ++i) {
    int value = ticks[i];
    if (value < 0) { out.bytes[out.length++] = '-'; value = -value; }
    char* start = reinterpret_cast<char*>(out.bytes.data() + out.length);
    const auto number = std::to_chars(
        start, reinterpret_cast<char*>(out.bytes.data() + out.bytes.size()), value / 10000);
    out.length += static_cast<std::size_t>(number.ptr - start);
    const int fraction = value % 10000;
    if (fraction) {
      out.bytes[out.length++] = '.';
      for (int divisor = 1000; divisor; divisor /= 10)
        out.bytes[out.length++] = static_cast<std::uint8_t>('0' + fraction / divisor % 10);
      while (out.bytes[out.length - 1] == '0') --out.length;
    }
    out.bytes[out.length++] = i == 2 ? '\n' : ',';
  }
  out.moving = c.forward || c.left || c.yaw;
  return out;
}
Velocity decode_serial(const std::uint8_t* data, std::size_t size) {
  require(data && size >= 6 && size <= serial_max_line && data[size - 1] == '\n',
          "invalid speed line length or terminator");
  const auto* cursor = data;
  const auto* end = data + size - 1;
  Velocity out{};
  for (unsigned i = 0; i < out.size(); ++i) {
    bool negative = cursor < end && *cursor == '-';
    if (negative) ++cursor;
    require(cursor < end && *cursor >= '0' && *cursor <= '9', "speed must be decimal");
    int whole = 0, fraction = 0, digits = 0;
    while (cursor < end && *cursor >= '0' && *cursor <= '9') {
      whole = whole * 10 + (*cursor++ - '0');
      require(whole <= 1, "speed out of range");
    }
    if (cursor < end && *cursor == '.') {
      ++cursor;
      while (cursor < end && *cursor >= '0' && *cursor <= '9') {
        require(++digits <= 4, "speed has more than four decimal places");
        fraction = fraction * 10 + (*cursor++ - '0');
      }
      require(digits != 0, "missing decimal fraction");
    }
    for (; digits < 4; ++digits) fraction *= 10;
    const int ticks = whole * 10000 + fraction;
    require(ticks <= (i == 2 ? 10000 : 5000), "speed out of range");
    out[i] = (negative ? -ticks : ticks) / 10000.0;
    if (i < 2) { require(cursor < end && *cursor == ',', "expected speed separator"); ++cursor; }
    else require(cursor == end, "trailing speed data");
  }
  return out;
}
// UDP序号按半区间判断新旧；不同session不共享序号基线。
bool newer(std::uint64_t sequence, std::uint64_t previous, unsigned bits) {
  const auto mask = bits == 64 ? UINT64_MAX : (std::uint64_t(1) << bits) - 1;
  const auto delta = (sequence - previous) & mask;
  return delta > 0 && delta < (std::uint64_t(1) << (bits - 1));
}
bool operator==(Peer a, Peer b) { return a.address == b.address && a.port == b.port; }

// UDP租约：只由接受的输入续期，时间为单调秒；超时身份不可复活。
// 换源须以新会话disabled接入；最近64个退役session保留为有界拒绝记录。
// timeout 范围只在 serial_control 入口校验一次。
Receiver::Receiver(double timeout) : timeout_(timeout) {}
bool Receiver::expired(double now) const { return !received_at_ || now - *received_at_ >= timeout_; }
Control Receiver::output(double now) { if (expired(now)) control_ = {}; return control_; }
/**
 * @brief 接受调用方构造的输入包；先校验值域，再处理当前租约。
 * @param[in] p 输入包；session 非零，控制量满足 validate 的范围约束。
 * @param[in] peer 发送方 IPv4/端口身份。
 * @param[in] now 同一单调时钟的当前秒数。
 * @param[in] allow_motion 本批停车后为 false，后续运动不得续租。
 * @return 接受并更新租约返回 true；值域非法抛异常且不改变租约。
 */
bool Receiver::accept(const Packet& p, Peer peer, double now, bool allow_motion) {
  validate(p.control);
  return accept_validated(p, peer, now, allow_motion);
}
/**
 * @brief 受理租约归属：当前身份沿用，换源须在过期后以未退役的新会话停车接入。
 * @param[in] p 已校验值域且已排除当前过期会话的输入包。
 * @param[in] peer 发送方身份，与 session 共同构成租约归属。
 * @param[in] now 同一单调时钟的当前秒数。
 * @return 归属可受理返回 true；换源时记录最近 64 个退役身份并重置序号。
 *         拒绝返回 false 且不改变归属、序号或退役记录；本步骤不续租。
 */
bool Receiver::accept_lease_owner(const Packet& p, Peer peer, double now) {
  const auto identity = std::make_pair(peer, p.session);
  if (owner_ && identity == *owner_) return true;
  if (!expired(now) || p.control.enabled ||
      std::find(retired_.begin(), retired_.end(), p.session) != retired_.end()) return false;
  if (owner_) {
    retired_.push_back(owner_->second);
    if (retired_.size() > 64) retired_.pop_front();
  }
  owner_ = identity;
  sequence_.reset();
  return true;
}
/**
 * @brief 消费已通过值域校验的包，统一处理租约和序号推进。
 * @param[in] p 仅由 accept 或 SerialBridge 的 decode_udp 成功结果传入。
 * @param[in] peer 发送方身份；租约归属按地址、端口和 session 联合判断。
 * @param[in] now 同一单调时钟的当前秒数。
 * @param[in] allow_motion false 时拒绝本批后续运动续租。
 * @return 当前会话和序号可受理返回 true；拒绝时不更新输入时间。
 */
bool Receiver::accept_validated(const Packet& p, Peer peer, double now, bool allow_motion) {
  output(now);
  if (owner_ && p.session == owner_->second && expired(now)) return false;
  if (!accept_lease_owner(p, peer, now)) return false;
  if (sequence_ && !newer(p.sequence, *sequence_)) return false;
  sequence_ = p.sequence;
  // 同批停车后的运动只记去重水位：不执行、不续期，重复包也不能在下轮复活。
  if (p.control.enabled && !allow_motion) return false;
  control_ = p.control; received_at_ = now; return true;
}
// 输入门控与轴映射：失效后先松开再使能；死区外保持原逐轴曲线。
Control EnableGate::apply(Control c, bool available, bool stop) {
  if (!available || stop) { ready_ = false; return {}; }
  if (!c.enabled) { ready_ = true; return {}; }
  return ready_ ? c : Control{};
}
namespace {
/**
 * @brief 对入口已校验的轴值执行死区和整数映射，不重复检查配置。
 * @param[in] value 有限无单位轴值；绝对值大于 1 时饱和。
 * @param[in] deadzone 有限无单位死区，范围 [0,1)。
 * @return [-1000,1000] 的归一化整数，保持半整数取偶数规则。
 */
int mapped_axis(double value, double deadzone) {
  const double magnitude = std::min(1.0, std::abs(value));
  if (magnitude <= deadzone) return 0;
  // 保持既有ties-to-even规则，显式处理恰好半值，不调用nearbyint。
  const double scaled = (magnitude - deadzone) / (1 - deadzone) * 1000;
  const auto lower = static_cast<int>(std::floor(scaled));
  const double fraction = scaled - lower;
  const int rounded = lower + (fraction > .5 || (fraction == .5 && (lower & 1)));
  return value < 0 ? -rounded : rounded;
}
/**
 * @brief 一次校验手柄三轴和共用死区，并生成已经使能的控制量。
 * @param[in] values 前向、左向、逆时针的有限无单位轴值，映射时饱和到 [-1,1]。
 * @param[in] deadzone 共用有限死区，范围 [0,1)。
 * @param[in] claw_request 调用入口根据开合按钮生成的 0..2 夹爪请求。
 * @return 规范化且已使能的控制量；非法数值抛异常，无状态变更。
 */
Control mapped_gamepad(const std::array<double, 3>& values, double deadzone, int claw_request) {
  return {true, mapped_axis(values[0], deadzone), mapped_axis(values[1], deadzone),
          mapped_axis(values[2], deadzone), claw_request};
}
}  // namespace
/**
 * @brief 在单轴入口校验数值和死区，随后进行一次映射。
 * @param[in] value 有限无单位轴值，超出 [-1,1] 时饱和。
 * @param[in] deadzone 有限无单位死区，范围 [0,1)。
 * @return [-1000,1000] 的归一化整数；非法输入抛异常，无状态变更。
 */
int axis(double value, double deadzone) {
  return mapped_axis(value, deadzone);
}
Control keyboard(const Keys& k) {
  if (!k.enable) return {};
  return {true, 1000 * (int(k.forward) - int(k.back)), 1000 * (int(k.left) - int(k.right)),
          1000 * (int(k.ccw) - int(k.cw)), claw(k.open, k.close)};
}
/**
 * @brief 在手柄入口一次校验三轴和共用死区，再生成控制量。
 * @param[in] forward 前向无单位有限轴值，映射时饱和到 [-1,1]。
 * @param[in] left 左向无单位有限轴值，映射时饱和到 [-1,1]。
 * @param[in] yaw 逆时针无单位有限轴值，映射时饱和到 [-1,1]。
 * @param[in] enabled 未使能直接返回零控制，不消费轴值。
 * @param[in] opened 张开请求；与 closed 同值时无夹爪请求。
 * @param[in] closed 闭合请求；与 opened 同值时无夹爪请求。
 * @param[in] deadzone 共用有限死区，范围 [0,1)。
 * @return 已规范化控制量；使能时数值非法抛异常，无状态变更。
 */
Control gamepad(double forward, double left, double yaw, bool enabled,
                bool opened, bool closed, double deadzone) {
  if (!enabled) return {};
  return mapped_gamepad({forward, left, yaw}, deadzone, claw(opened, closed));
}

// UDP批次与串口提交：一轮至多32包、一条速度行；完整提交且仍有效后才回ACK。
SerialBridge::SerialBridge(Channel& channel, Port& port, Clock clock, Stop stop, double timeout)
    : channel_(channel), output_(port, stop), clock_(std::move(clock)),
      stop_(std::move(stop)), receiver_(timeout) {}
SerialBridge::~SerialBridge() { close(); }
/**
 * @brief 处理一批 UDP 输入，再按当前租约提交最多一条串口速度。
 * @note 每个包仅在解码入口校验值域；停止、租约和发送结果按当前状态判定。
 */
void SerialBridge::step() {
  if (closed_ || stop_()) return;
  receiver_.output(clock_());
  std::optional<std::pair<Peer, Packet>> ack;
  bool stop_seen = false;
  for (unsigned i = 0; i < 32; ++i) {
    const auto message = channel_.receive();
    if (!message) break;
    try {
      // UDP 入口已完成长度、身份和值域校验；同一快照不在租约层重复检查。
      const auto p = decode_udp(message->data.data(), message->size);
      if (receiver_.accept_validated(p, message->peer, clock_(), !stop_seen)) {
        ack = std::make_pair(message->peer, p); stop_seen |= !p.control.enabled;
      }
    } catch (const std::invalid_argument&) {}  // 非法报文不改变租约，也不终止接收服务。
  }
  if (!output_.ready()) return;
  output_.write(receiver_.output(clock_()));
  if (ack && !stop_() && !receiver_.expired(clock_())) {
    const auto bytes = encode_ack(ack->second.session, ack->second.sequence);
    try { channel_.send(bytes.data(), bytes.size(), ack->first); }
    catch (const std::exception&) {}  // ACK失败不续租，PC仍按自己的发送截止停车。
  }
}
// 共用串口生命周期：Port由调用方持有；同步标记必须完整写入后才可追加零速。
SerialOutput::SerialOutput(Port& port, Stop stop) : port_(port), stop_(std::move(stop)) {
  port_.clear(); resynchronize();
}
void SerialOutput::resynchronize() {
  // 给旧半行追加非法字符再换行，防止短写前缀与停车行拼成新的非零速度。
  SerialFrame reset; reset.bytes[0] = '!'; reset.bytes[1] = '\n'; reset.length = 2;
  if (port_.write(reset) != reset.size())
    throw std::runtime_error("incomplete serial line reset; stopped");
}
SerialOutput::~SerialOutput() { close(); }
bool SerialOutput::ready() {
  if (closed_) return false;
  if (port_.waiting() > static_cast<int>(serial_max_line))
    throw std::runtime_error("serial output backlog; manual control stopped");
  return !stop_();
}
void SerialOutput::write(Control c) {
  if (closed_) return;
  if (stop_() && c.enabled) return;
  const auto bytes = encode_serial(c);
  if (port_.write(bytes) != bytes.size())
    throw std::runtime_error("incomplete serial write; manual control stopped");
}
void SerialOutput::close() noexcept {
  if (closed_) return;
  try { port_.clear(); }
  catch (const std::exception& e) { std::cerr << "WARNING clear output: " << e.what() << '\n'; }
  try { resynchronize(); write({}); }
  catch (const std::exception& e) { std::cerr << "WARNING final stop: " << e.what() << '\n'; }
  closed_ = true; port_.close();
}
void SerialBridge::close() noexcept {
  if (closed_) return;
  closed_ = true; output_.close(); channel_.close();
}

// 客户端ACK与会话生命周期：最多记录64次待确认发送，以原发送时刻计算0.5秒期限。
// 过期先换会话，迟到ACK不能遮盖断线；close尽力发送三次停止输入再关闭Channel。
RemoteLink::RemoteLink(Channel& channel, Peer peer, Clock clock,
                       std::function<std::uint64_t()> new_session)
    : channel_(channel), peer_(peer), clock_(std::move(clock)), new_session_(std::move(new_session)) {
  start_session(clock_());
}
RemoteLink::~RemoteLink() { close(); }
void RemoteLink::start_session(double now) {
  std::uint64_t next;
  do { next = new_session_(); } while (!next || next == session_);
  session_ = next; started_at_ = now; sequence_ = 0;
  ack_time_.reset(); last_ack_.reset(); pending_.clear();
}
bool RemoteLink::expired(double now) const { return now - ack_time_.value_or(started_at_) >= .5; }
void RemoteLink::send(Control c) {
  if (closed_) return;
  const auto bytes = encode_udp({session_, sequence_, c});
  const auto sent_at = clock_();
  channel_.send(bytes.data(), bytes.size(), peer_);
  pending_.emplace_back(sequence_++, sent_at);
  if (pending_.size() > 64) pending_.pop_front();
}
void RemoteLink::poll() {
  if (closed_) return;
  auto now = clock_();
  if (expired(now)) start_session(now);
  for (unsigned i = 0; i < 32; ++i) {
    std::optional<Datagram> message;
    try { message = channel_.receive(); } catch (const std::exception&) { break; }
    if (!message) break;
    Ack ack;
    try { ack = decode_ack(message->data.data(), message->size); }
    catch (const std::invalid_argument&) { continue; }
    now = clock_();
    if (expired(now)) { start_session(now); break; }
    if (message->peer != peer_ || ack.first != session_) continue;
    const auto sent = std::find_if(pending_.begin(), pending_.end(),
                                  [&](const auto& p) { return p.first == ack.second; });
    if (sent == pending_.end() || now - sent->second >= .5 ||
        (last_ack_ && !newer(ack.second, *last_ack_))) continue;
    last_ack_ = ack.second; ack_time_ = sent->second; pending_.erase(sent);
  }
}
bool RemoteLink::available() const { return ack_time_ && !expired(clock_()); }
void RemoteLink::close() noexcept {
  if (closed_) return;
  for (unsigned i = 0; i < 3; ++i) {
    try { send({}); } catch (const std::exception&) { break; }
  }
  closed_ = true; channel_.close();
}
}  // namespace teleop
