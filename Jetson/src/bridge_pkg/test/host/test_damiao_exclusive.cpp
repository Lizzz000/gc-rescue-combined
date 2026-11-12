// M5：达妙串口独占。bridge（DamiaoCan）与 teleop（flock + TIOCEXCL）任一方持有设备时，
//     后打开者必须失败（DamiaoCan 抛 FatalError，入口据此退出）且不向设备写任何字节。
// M7：bridge 不发 0x17F；0x180 只作心跳（valid=0、DLC7、载荷全零）。
// 用 pty 代替 USB-CAN；独立可执行文件，自带 main。
#include "check.hpp"
#include "protocol/can_protocol.hpp"
#include "state/bridge_state.hpp"
#include "transport/damiao/damiao.hpp"
#include "transport/transport.hpp"

#include <fcntl.h>
#include <poll.h>
#include <pty.h>
#include <sys/file.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>

#include <cstdio>
#include <string>
#include <type_traits>
#include <utility>

namespace {

constexpr canid_t kManualId = 0x17F;          // 手动帧，只允许 teleop 发送
constexpr std::int64_t kBitrate = 1000000;     // 与 rescue.yaml can_bitrate 一致，open 不使用
constexpr int kDrainTimeoutMs = 100;           // 检查主端无写入的等待时间
constexpr int kScheduleRounds = 3000;          // 周期调度检查轮数

// 一对 pty：slave 路径交给被测方打开，master 端读出被测方写入的字节。
struct Pty
{
  int master{-1}, slave{-1};
  std::string path;
  Pty()
  {
    char name[128]{};
    if (::openpty(&master, &slave, name, nullptr, nullptr) == 0) {path = name;}
    // 本测试只用 path；关闭 slave 端避免它自身占用 flock 之外的状态
    ::close(slave); slave = -1;
  }
  ~Pty() {if (master >= 0) {::close(master);}}
  // 等待片刻后主端可读的字节数（被测方写入的量）
  int pendingBytes() const
  {
    pollfd p{master, POLLIN, 0};
    if (::poll(&p, 1, kDrainTimeoutMs) <= 0 || !(p.revents & POLLIN)) {return 0;}
    char buf[256];
    const auto n = ::read(master, buf, sizeof(buf));
    return n > 0 ? int(n) : 0;
  }
};

// 模拟 teleop 的独占方式（manual/link.py 同一约定）。
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

OpenOutcome bridgeOpen(DamiaoCan & can, const std::string & path)
{
  int index = 0;
  std::string error;
  try {
    return can.open(path, index, error) ? OpenOutcome::OPENED : OpenOutcome::FAILED;
  } catch (const CanTransport::FatalError &) {
    return OpenOutcome::FATAL;
  }
}

void testBridgeFirst()
{
  Group group("M5 bridge 先占用");
  Pty pty;
  CHECK(!pty.path.empty());
  DamiaoCan bridge(kBitrate);
  CHECK(bridgeOpen(bridge, pty.path) == OpenOutcome::OPENED);
  // 第二个 bridge 实例
  DamiaoCan second(kBitrate);
  CHECK(bridgeOpen(second, pty.path) == OpenOutcome::FATAL);
  CHECK(!second.socketOpen());
  // teleop 方式打开
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
    DamiaoCan bridge(kBitrate);
    CHECK(bridgeOpen(bridge, pty.path) == OpenOutcome::FATAL);
    CHECK(!bridge.socketOpen());
    bytes = pty.pendingBytes();
  }
  CHECK(bytes == 0);
  // 占用方退出后可以正常打开，不留锁
  DamiaoCan bridge(kBitrate);
  CHECK(bridgeOpen(bridge, pty.path) == OpenOutcome::OPENED);
}

void testNeverManualId()
{
  Group group("M7 bridge 不发 0x17F");
  CHECK(!CanProtocol::transmitId(kManualId));
  CHECK(!CanProtocol::streamId(kManualId));
  CHECK(!CanProtocol::replyId(kManualId));
  CanPeriodicSchedule schedule;
  int manual = 0, velocity = 0, foreign = 0;
  for (int i = 0; i < kScheduleRounds; ++i) {
    const canid_t id = schedule.next();
    manual += id == kManualId;
    velocity += id == CanProtocol::VELOCITY_ID;
    foreign += !CanProtocol::transmitId(id);
  }
  CHECK(manual == 0);
  CHECK(foreign == 0);
  CHECK(velocity == kScheduleRounds / 3);
}

void testHeartbeatEncoding()
{
  Group group("M7 0x180 心跳编码");
  // 无论调用方传入什么分量，valid=0 时 DLC7 且载荷全零
  const can_frame frame = CanProtocol::velocity(false, 123, -456, 789);
  CHECK(frame.can_id == CanProtocol::VELOCITY_ID);
  CHECK(frame.len == 7);
  bool zero = true;
  for (int i = 0; i < 8; ++i) {zero = zero && frame.data[i] == 0;}
  CHECK(zero);
  // 经达妙封装后同样是 ID 0x180、DLC7、零载荷
  const auto packet = damiao::encode(frame);
  CHECK(packet[13] == 0x80 && packet[14] == 0x01 && packet[15] == 0 && packet[16] == 0);
  CHECK(packet[18] == 7);
  bool payload_zero = true;
  for (int i = 21; i < 28; ++i) {payload_zero = payload_zero && packet[i] == 0;}
  CHECK(payload_zero);
}

// 心跳化后 BridgeState 不再接收速度输入，bridge 不订阅 /cmd_vel。
template<typename T, typename = void>
struct AcceptsVelocity : std::false_type {};
template<typename T>
struct AcceptsVelocity<T, std::void_t<decltype(std::declval<T &>().velocity(
    0.0, 0.0, 0.0, BridgeMonotonicTime{}, true))>> : std::true_type {};

void testHeartbeatOnly()
{
  Group group("M7 0x180 恒为心跳");
  CHECK((!AcceptsVelocity<BridgeState>::value));
}

}  // namespace

int main()
{
  testBridgeFirst();
  testTeleopFirst(true, "M5 teleop 先占用（flock+TIOCEXCL）");
  testTeleopFirst(false, "M5 teleop 先占用（仅 flock）");
  testNeverManualId();
  testHeartbeatEncoding();
  testHeartbeatOnly();
  return finishTests("bridge damiao exclusive");
}
