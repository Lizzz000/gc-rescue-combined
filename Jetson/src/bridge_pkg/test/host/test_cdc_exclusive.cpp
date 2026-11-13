// CDC 设备锁：bridge 与手动程序任一方持有 link.device 时，后打开者失败且不写字节。
// 类型 2 只给手动程序。类型 1 在线时 alive=1、速度为 0；失联是 alive=0 的 8 字节。
// pty 代替 USB 串口。虚拟回环不在这里测。
#include "check.hpp"
#include "protocol/usb_protocol.hpp"
#include "state/bridge_state.hpp"
#include "transport/cdc/cdc.hpp"
#include "transport/transport.hpp"

#include <fcntl.h>
#include <poll.h>
#include <pty.h>
#include <sys/file.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>

#include <cstring>
#include <string>
#include <type_traits>
#include <utility>

namespace {

constexpr int kDrainTimeoutMs = 100;
constexpr int kScheduleRounds = 3000;

struct Pty
{
  int master{-1}, slave{-1};
  std::string path;
  Pty()
  {
    char name[128]{};
    if (::openpty(&master, &slave, name, nullptr, nullptr) == 0) {path = name;}
    ::close(slave); slave = -1;
  }
  ~Pty() {if (master >= 0) {::close(master);}}
  int pendingBytes() const
  {
    pollfd p{master, POLLIN, 0};
    if (::poll(&p, 1, kDrainTimeoutMs) <= 0 || !(p.revents & POLLIN)) {return 0;}
    char buf[256];
    const auto n = ::read(master, buf, sizeof(buf));
    return n > 0 ? int(n) : 0;
  }
};

struct TeleopHolder
{
  int fd{-1};
  TeleopHolder(const std::string & path, bool exclusive)
  {
    fd = ::open(path.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd >= 0 && ::flock(fd, LOCK_EX | LOCK_NB) < 0) {::close(fd); fd = -1;}
    if (fd >= 0 && exclusive) {::ioctl(fd, TIOCEXCL);}
  }
  ~TeleopHolder()
  {
    if (fd >= 0) {::ioctl(fd, TIOCNXCL); ::close(fd);}
  }
};

enum class OpenOutcome { OPENED, FATAL, FAILED };

OpenOutcome bridgeOpen(CdcLink & link, const std::string & path)
{
  int index = 0;
  std::string error;
  try {
    return link.open(path, index, error) ? OpenOutcome::OPENED : OpenOutcome::FAILED;
  } catch (const CanTransport::FatalError &) {
    return OpenOutcome::FATAL;
  }
}

void testBridgeFirst()
{
  Group group("CDC bridge 先占用");
  Pty pty;
  CHECK(!pty.path.empty());
  CdcLink bridge;
  CHECK(bridgeOpen(bridge, pty.path) == OpenOutcome::OPENED);
  CdcLink second;
  CHECK(bridgeOpen(second, pty.path) == OpenOutcome::FATAL);
  CHECK(!second.socketOpen());
  TeleopHolder teleop(pty.path, true);
  CHECK(teleop.fd < 0);
  CHECK(pty.pendingBytes() == 0);
}

void testTeleopFirst(bool exclusive, const char * name)
{
  Group group(name);
  Pty pty;
  int bytes = 0;
  {
    TeleopHolder teleop(pty.path, exclusive);
    CHECK(teleop.fd >= 0);
    CdcLink bridge;
    CHECK(bridgeOpen(bridge, pty.path) == OpenOutcome::FATAL);
    CHECK(!bridge.socketOpen());
    bytes = pty.pendingBytes();
  }
  CHECK(bytes == 0);
  CdcLink bridge;
  CHECK(bridgeOpen(bridge, pty.path) == OpenOutcome::OPENED);
}

void testNeverManualType()
{
  Group group("自动桥不发类型 2");
  CHECK(!CanProtocol::transmitId(CanProtocol::MANUAL_ID));
  CHECK(!CanProtocol::streamId(CanProtocol::MANUAL_ID));
  CHECK(!CanProtocol::replyId(CanProtocol::MANUAL_ID));
  CanPeriodicSchedule schedule;
  int manual = 0, velocity = 0, foreign = 0;
  for (int i = 0; i < kScheduleRounds; ++i) {
    const canid_t id = schedule.next();
    manual += id == CanProtocol::MANUAL_ID;
    velocity += id == CanProtocol::VELOCITY_ID;
    foreign += !CanProtocol::transmitId(id);
  }
  CHECK(manual == 0);
  CHECK(foreign == 0);
  CHECK(velocity == kScheduleRounds / 3);
}

void testHeartbeatEncoding()
{
  Group group("类型 1 心跳");
  const can_frame lost = CanProtocol::velocity(false, 123, -456, 789, 1);
  CHECK(lost.can_id == CanProtocol::VELOCITY_ID && lost.len == 8);
  CHECK(lost.data[0] == 0 && lost.data[1] == 1);
  bool speeds_zero = true;
  for (int i = 2; i < 8; ++i) {speeds_zero = speeds_zero && lost.data[i] == 0;}
  CHECK(speeds_zero);
  const can_frame online = CanProtocol::velocity(true, 0, 0, 0, 0);
  CHECK(online.data[0] == 1 && online.data[1] == 0);
  bool online_zero = true;
  for (int i = 2; i < 8; ++i) {online_zero = online_zero && online.data[i] == 0;}
  CHECK(online_zero);
}

template<typename T, typename = void>
struct AcceptsVelocity : std::false_type {};
template<typename T>
struct AcceptsVelocity<T, std::void_t<decltype(std::declval<T &>().velocity(
    0.0, 0.0, 0.0, BridgeMonotonicTime{}, true))>> : std::true_type {};

void testHeartbeatOnly()
{
  Group group("自动桥没有速度输入");
  CHECK((!AcceptsVelocity<BridgeState>::value));
}

void testEmptyReadAndDisconnect()
{
  Group group("空读与断开");
  Pty pty;
  CdcLink bridge;
  CHECK(bridgeOpen(bridge, pty.path) == OpenOutcome::OPENED);
  can_frame frame{};
  std::string error;
  CHECK(bridge.read(frame, error) == CanTransport::ReadResult::EMPTY);
  ::close(pty.master);
  pty.master = -1;
  CHECK(bridge.read(frame, error) == CanTransport::ReadResult::FAILED);
}

void testBackpressurePreservesFrameIdentity()
{
  Group group("背压后发送新帧");
  Pty pty;
  CdcLink bridge;
  CHECK(bridgeOpen(bridge, pty.path) == OpenOutcome::OPENED);
  ::fcntl(pty.master, F_SETFL, ::fcntl(pty.master, F_GETFL) | O_NONBLOCK);
  can_frame old_frame{};
  old_frame.can_id = 3;
  old_frame.len = 8;
  std::string error;
  auto result = CanTransport::WriteResult::SENT;
  for (int i = 0; i < 100000 && result == CanTransport::WriteResult::SENT; ++i) {
    result = bridge.write(old_frame, error);
  }
  CHECK(result == CanTransport::WriteResult::BACKPRESSURE);
  char discarded[4096];
  while (::read(pty.master, discarded, sizeof(discarded)) > 0) {}

  can_frame new_frame{};
  new_frame.can_id = 7;
  new_frame.len = 6;
  result = bridge.write(new_frame, error);
  CHECK(result == CanTransport::WriteResult::SENT);
  pollfd ready{pty.master, POLLIN, 0};
  CHECK(::poll(&ready, 1, kDrainTimeoutMs) > 0);
  unsigned char bytes[4096]{};
  const auto count = ::read(pty.master, bytes, sizeof(bytes));
  std::uint8_t expected[link_frame::kMaxBytes]{};
  const auto size = link_frame::encode(7, new_frame.data, new_frame.len, expected);
  bool found = false;
  for (ssize_t i = 0; i + static_cast<ssize_t>(size) <= count; ++i) {
    if (std::memcmp(bytes + i, expected, size) == 0) {
      found = true;
      break;
    }
  }
  CHECK(found);
}

}  // namespace

int main()
{
  testBridgeFirst();
  testTeleopFirst(true, "手动程序先占用（flock+TIOCEXCL）");
  testTeleopFirst(false, "手动程序先占用（仅 flock）");
  testNeverManualType();
  testHeartbeatEncoding();
  testHeartbeatOnly();
  testEmptyReadAndDisconnect();
  testBackpressurePreservesFrameIdentity();
  return finishTests("bridge cdc exclusive");
}
