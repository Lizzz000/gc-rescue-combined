// CDC 设备锁：bridge 与手动程序任一方持有 link.device 时，后打开者失败且不写字节。
// 自动桥仅周期发送类型 3/4；类型 2 只给手动程序，类型 1 已删除。
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
  std::string error;
  try {
    return link.open(path, error) ? OpenOutcome::OPENED : OpenOutcome::FAILED;
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
  Group group("自动桥只周期发送坐标");
  CHECK(!CanProtocol::transmitId(CanProtocol::MANUAL_ID));
  CHECK(!CanProtocol::streamId(CanProtocol::MANUAL_ID));
  CHECK(!CanProtocol::replyId(CanProtocol::MANUAL_ID));
  CanPeriodicSchedule schedule;
  int manual = 0, removed = 0, target = 0, safety = 0, foreign = 0;
  for (int i = 0; i < kScheduleRounds; ++i) {
    const canid_t id = *schedule.next(true);
    manual += id == CanProtocol::MANUAL_ID;
    removed += id == 1;
    target += id == CanProtocol::TARGET_ID;
    safety += id == CanProtocol::SAFETY_ID;
    foreign += !CanProtocol::transmitId(id);
  }
  CHECK(manual == 0);
  CHECK(foreign == 0);
  CHECK(removed == 0);
  CHECK(target == kScheduleRounds / 2 && safety == kScheduleRounds / 2);
  CHECK(CanPeriodicSchedule::interval == std::chrono::milliseconds(5));
}

template<typename T, typename = void>
struct AcceptsVelocity : std::false_type {};
template<typename T>
struct AcceptsVelocity<T, std::void_t<decltype(std::declval<T &>().velocity(
    0.0, 0.0, 0.0, BridgeMonotonicTime{}, true))>> : std::true_type {};

void testNoVelocityInput()
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
  // 先留一个仅用于填满内核输出队列的描述符；单字节写不会制造半协议帧。
  const int filler = ::open(pty.path.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  CHECK(filler >= 0);
  CdcLink bridge;
  CHECK(bridgeOpen(bridge, pty.path) == OpenOutcome::OPENED);
  ::fcntl(pty.master, F_SETFL, ::fcntl(pty.master, F_GETFL) | O_NONBLOCK);
  can_frame old_frame{};
  old_frame.can_id = 3;
  old_frame.len = 8;
  std::string error;
  const char padding = 0;
  for (int i = 0; i < 100000 && ::write(filler, &padding, 1) == 1; ++i) {}
  auto result = bridge.write(old_frame, error);
  CHECK(result == CanTransport::WriteResult::BACKPRESSURE);
  ::close(filler);
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
  link_frame::Parser parser;
  unsigned old_frames = 0;
  for (ssize_t i = 0; i < count; ++i) {
    link_frame::Frame decoded;
    if (parser.feed(bytes[i], decoded) && decoded.type == old_frame.can_id) {++old_frames;}
  }
  CHECK(old_frames == 0);
}

}  // namespace

int main()
{
  testBridgeFirst();
  testTeleopFirst(true, "手动程序先占用（flock+TIOCEXCL）");
  testTeleopFirst(false, "手动程序先占用（仅 flock）");
  testNeverManualType();
  testNoVelocityInput();
  testEmptyReadAndDisconnect();
  testBackpressurePreservesFrameIdentity();
  return finishTests("bridge cdc exclusive");
}
