// 平台资源、参数入口与控制协议共用同一实现单元。
// Copyright 2026 li
// SPDX-License-Identifier: Apache-2.0
#include "teleop_pkg/control.hpp"
#ifdef _WIN32
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <winsock2.h>
#include <ws2tcpip.h>
#else
#include <arpa/inet.h>
#include <fcntl.h>
#include <netdb.h>
#include <sys/socket.h>
#include <unistd.h>
#endif
#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <random>
#include <stdexcept>

namespace teleop {
namespace {
volatile std::sig_atomic_t stop_requested = 0;
void request_stop(int) { stop_requested = 1; }
#ifdef _WIN32
using Handle = SOCKET;
constexpr auto invalid_handle = INVALID_SOCKET;
int socket_error() { return WSAGetLastError(); }
bool retryable(int error) { return error == WSAEWOULDBLOCK || error == WSAEINTR; }
void close_socket(Handle handle) { closesocket(handle); }
struct Winsock {
  Winsock() { WSADATA data{}; if (WSAStartup(MAKEWORD(2, 2), &data)) throw std::runtime_error("WSAStartup failed"); }
  ~Winsock() { WSACleanup(); }
};
#else
using Handle = int;
constexpr auto invalid_handle = -1;
int socket_error() { return errno; }
bool retryable(int error) { return error == EAGAIN || error == EWOULDBLOCK || error == EINTR; }
void close_socket(Handle handle) { ::close(handle); }
#endif
Handle native(std::uintptr_t value) { return static_cast<Handle>(value); }
void fail_socket(const char* operation) {
  throw std::runtime_error(std::string(operation) + ": socket error " + std::to_string(socket_error()));
}
sockaddr_in address(Peer peer) {
  sockaddr_in result{}; result.sin_family = AF_INET;
  result.sin_addr.s_addr = peer.address; result.sin_port = htons(peer.port); return result;
}
Peer peer_of(const sockaddr_in& addr) { return {addr.sin_addr.s_addr, ntohs(addr.sin_port)}; }
}  // namespace
double monotonic() {
  return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}
double next_send(double previous, double now, double period) {
  const auto scheduled = previous + period;
  return scheduled > now ? scheduled : now + period;
}
std::uint64_t random_session() {
  static std::random_device random;
  return (std::uint64_t(random()) << 32) ^ std::uint64_t(random());
}
Signals::Signals() {
  stop_requested = 0; old_int_ = std::signal(SIGINT, request_stop); old_term_ = std::signal(SIGTERM, request_stop);
#ifndef _WIN32
  old_hup_ = std::signal(SIGHUP, request_stop); old_pipe_ = std::signal(SIGPIPE, request_stop);
#endif
}
Signals::~Signals() {
  std::signal(SIGINT, old_int_); std::signal(SIGTERM, old_term_);
#ifndef _WIN32
  std::signal(SIGHUP, old_hup_); std::signal(SIGPIPE, old_pipe_);
#endif
}
bool Signals::stopped() { return stop_requested != 0; }
UdpSocket::UdpSocket() : handle_(static_cast<std::uintptr_t>(invalid_handle)) {
#ifdef _WIN32
  static Winsock winsock;
#endif
  const auto handle = ::socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
  if (handle == invalid_handle) fail_socket("socket");
#ifdef _WIN32
  u_long nonblocking = 1;
  const auto result = ioctlsocket(handle, FIONBIO, &nonblocking);
#else
  const auto result = fcntl(handle, F_SETFL, O_NONBLOCK);
#endif
  if (result != 0) { close_socket(handle); fail_socket("nonblocking socket"); }
  handle_ = static_cast<std::uintptr_t>(handle);
}
UdpSocket::~UdpSocket() { close(); }
Peer UdpSocket::resolve(const std::string& host, int port) {
  // 端口范围已由 serial_control/pc_control 入口校验。
  addrinfo hints{}; hints.ai_family = AF_INET; hints.ai_socktype = SOCK_DGRAM;
  addrinfo* result = nullptr;
  const int error = getaddrinfo(host.c_str(), nullptr, &hints, &result);
  if (error || !result) throw std::runtime_error("cannot resolve IPv4 host: " + host);
  std::unique_ptr<addrinfo, decltype(&freeaddrinfo)> owned(result, freeaddrinfo);
  return {reinterpret_cast<sockaddr_in*>(result->ai_addr)->sin_addr.s_addr, static_cast<std::uint16_t>(port)};
}
void UdpSocket::bind(Peer peer) {
  const auto addr = address(peer);
  if (::bind(native(handle_), reinterpret_cast<const sockaddr*>(&addr), sizeof(addr))) fail_socket("bind");
}
Peer UdpSocket::local() const {
  sockaddr_in addr{};
#ifdef _WIN32
  int length = sizeof(addr);
#else
  socklen_t length = sizeof(addr);
#endif
  if (getsockname(native(handle_), reinterpret_cast<sockaddr*>(&addr), &length)) fail_socket("getsockname");
  return peer_of(addr);
}
std::optional<Datagram> UdpSocket::receive() {
  Datagram result; sockaddr_in addr{};
#ifdef _WIN32
  int length = sizeof(addr);
#else
  socklen_t length = sizeof(addr);
#endif
  const auto count = recvfrom(native(handle_), reinterpret_cast<char*>(result.data.data()),
                             static_cast<int>(result.data.size()), 0,
                             reinterpret_cast<sockaddr*>(&addr), &length);
  if (count < 0) {
    const auto error = socket_error();
    if (retryable(error)) return std::nullopt;
#ifdef _WIN32
    // 超长UDP已消费，返回必定不合法的长度；不能截成24字节后误认作合法输入。
    if (error == WSAEMSGSIZE) { result.size = result.data.size(); return result; }
#endif
    fail_socket("recvfrom");
  }
  result.size = static_cast<std::size_t>(count); result.peer = peer_of(addr); return result;
}
void UdpSocket::send(const std::uint8_t* data, std::size_t size, Peer peer) {
  const auto addr = address(peer);
  const auto count = sendto(native(handle_), reinterpret_cast<const char*>(data), static_cast<int>(size),
                            0, reinterpret_cast<const sockaddr*>(&addr), sizeof(addr));
  if (count < 0 || static_cast<std::size_t>(count) != size) fail_socket("sendto");
}
void UdpSocket::close() noexcept {
  if (native(handle_) != invalid_handle) { close_socket(native(handle_)); handle_ = static_cast<std::uintptr_t>(invalid_handle); }
}
Arguments::Arguments(int argc, char** argv, std::map<std::string, std::string> values,
                     const std::vector<std::string>& flags) : values_(std::move(values)) {
  for (const auto& flag : flags) values_.emplace(flag, "false");
  values_.emplace("--help", "false");
  for (int i = 1; i < argc; ++i) {
    std::string key = argv[i];
    if (key == "-h") key = "--help";
    const auto equal = key.find('='); const auto name = key.substr(0, equal);
    auto found = values_.find(name);
    if (found == values_.end()) throw std::invalid_argument("unknown option: " + name);
    const bool is_flag = name == "--help" || std::find(flags.begin(), flags.end(), name) != flags.end();
    if (is_flag) {
      if (equal != std::string::npos) throw std::invalid_argument("flag takes no value: " + name);
      found->second = "true";
      // argparse BooleanOptionalAction 保留最后一次正/反选项。
      const auto opposite = name.rfind("--no-", 0) == 0 ? "--" + name.substr(5) : "--no-" + name.substr(2);
      if (values_.count(opposite)) values_[opposite] = "false";
    } else if (equal != std::string::npos) found->second = key.substr(equal + 1);
    else {
      if (++i >= argc || std::string(argv[i]).rfind("--", 0) == 0) throw std::invalid_argument("missing value: " + name);
      found->second = argv[i];
    }
  }
}
const std::string& Arguments::text(const std::string& key) const { return values_.at(key); }
int Arguments::integer(const std::string& key) const {
  std::size_t used = 0; long long value;
  try { value = std::stoll(text(key), &used); } catch (const std::exception&) { throw std::invalid_argument("invalid integer: " + key); }
  if (used != text(key).size() || value < std::numeric_limits<int>::min() || value > std::numeric_limits<int>::max())
    throw std::invalid_argument("invalid integer: " + key);
  return static_cast<int>(value);
}
double Arguments::number(const std::string& key) const {
  std::size_t used = 0; double value;
  try { value = std::stod(text(key), &used); } catch (const std::exception&) { throw std::invalid_argument("invalid number: " + key); }
  if (used != text(key).size() || !std::isfinite(value)) throw std::invalid_argument("invalid finite number: " + key);
  return value;
}
bool Arguments::flag(const std::string& key) const { return text(key) == "true"; }
}  // namespace teleop
