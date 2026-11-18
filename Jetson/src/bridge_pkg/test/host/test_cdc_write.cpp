// Fault injection at POSIX calls; compile the real cdc.cpp without opening a device.
#include "check.hpp"
#include "transport/cdc/cdc.hpp"
#include <sys/stat.h>
#include <termios.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <algorithm>
#include <cerrno>
#include <cstring>
#include <vector>

namespace {
enum class WriteMode { Ready, Blocked, Interrupted, Short, Zero, Failed };
WriteMode mode = WriteMode::Ready;
bool deny_lock = false, deny_exclusive = false, deny_tty = false;
bool kernel_exclusive = false;
int exclusive_releases = 0;
std::vector<unsigned char> wire;
std::vector<unsigned char> incoming;
std::size_t read_pos = 0;
int read_calls = 0;
std::vector<unsigned> types()
{
  link_frame::Parser parser;
  std::vector<unsigned> result;
  for (auto byte : wire) {
    link_frame::Frame frame;
    if (parser.feed(byte, frame)) {result.push_back(frame.type);}
  }
  return result;
}
}

int open(const char *, int, ...) {return 42;}
int close(int) {return 0;}
int flock(int, int) {if (deny_lock) {errno = EWOULDBLOCK; return -1;} return 0;}
int ioctl(int, unsigned long request, ...)
{
  if (request == TIOCEXCL) {
    if (deny_exclusive) {errno = EIO; return -1;}
    kernel_exclusive = true;
  } else if (request == TIOCNXCL) {
    ++exclusive_releases;
    kernel_exclusive = false;
  }
  return 0;
}
int fstat(int, struct stat *out) {out->st_mode = 1; return 0;}
int tcgetattr(int, termios *out) {if (deny_tty) {errno = EIO; return -1;} *out = {}; return 0;}
void cfmakeraw(termios *) {}
int tcsetattr(int, int, const termios *) {return 0;}
ssize_t read(int, void *data, std::size_t size)
{
  ++read_calls;
  if (read_pos == incoming.size()) {errno = EAGAIN; return -1;}
  const auto count = std::min(size, incoming.size() - read_pos);
  std::memcpy(data, incoming.data() + read_pos, count);
  read_pos += count;
  return static_cast<ssize_t>(count);
}
ssize_t write(int, const void *data, std::size_t size)
{
  if (mode == WriteMode::Blocked) {errno = EAGAIN; return -1;}
  if (mode == WriteMode::Interrupted) {errno = EINTR; return -1;}
  if (mode == WriteMode::Zero) {return 0;}
  if (mode == WriteMode::Failed) {errno = EIO; return -1;}
  const auto count = mode == WriteMode::Short ? size / 2 : size;
  const auto *bytes = static_cast<const unsigned char *>(data);
  wire.insert(wire.end(), bytes, bytes + count);
  return static_cast<ssize_t>(count);
}

int main()
{
  {
    Group group("CDC failed claimant cannot release another owner's tty exclusion");
    CdcLink rejected;
    std::string error;
    // Both descriptors opened before owner A set TIOCEXCL. B now loses flock.
    kernel_exclusive = true;
    exclusive_releases = 0;
    deny_lock = true;
    bool fatal = false;
    try {rejected.open("/dev/mock", error);} catch (const CanTransport::FatalError &) {fatal = true;}
    CHECK(fatal);
    CHECK(!rejected.socketOpen());
    CHECK(kernel_exclusive);
    CHECK(exclusive_releases == 0);
    deny_lock = false;
    // A failed TIOCEXCL also gives this instance no exclusion to release.
    deny_exclusive = true;
    CHECK(!rejected.open("/dev/mock", error));
    CHECK(exclusive_releases == 0);
    deny_exclusive = false;
    // Once this instance acquired exclusion, later configuration failure releases it.
    kernel_exclusive = false;
    exclusive_releases = 0;
    deny_tty = true;
    CHECK(!rejected.open("/dev/mock", error));
    CHECK(!kernel_exclusive);
    CHECK(exclusive_releases == 1);
    rejected.close();
    CHECK(exclusive_releases == 1);
    deny_tty = false;
  }
  CdcLink link;
  std::string error;
  CHECK(link.open("/dev/mock", error));
  const auto old = CanProtocol::turnRequest(1, 17);
  const auto fresh = CanProtocol::center(CanProtocol::TARGET_ID, false, 0, 0);
  {
    Group group("split frames, buffered replies and bounded noise reads");
    // The first frame straddles a read boundary. Two more arrive in the same batch.
    std::uint8_t packet[link_frame::kMaxBytes];
    const std::uint8_t payload[]{1, 0, 0xa5};
    const auto bytes = link_frame::encode(7, payload, sizeof(payload), packet);
    incoming.assign(32 * 256 + 255, 0);
    for (int i = 0; i < 3; ++i) {
      incoming.insert(incoming.end(), packet, packet + bytes);
    }
    can_frame frame{};
    CHECK(link.read(frame, error) == CanTransport::ReadResult::EMPTY);
    CHECK(read_calls == 32);
    CHECK(link.read(frame, error) == CanTransport::ReadResult::FRAME);
    CHECK(read_calls == 34);
    CHECK(frame.can_id == 7 && frame.len == sizeof(payload));
    CHECK(std::memcmp(frame.data, payload, sizeof(payload)) == 0);
    CHECK(link.read(frame, error) == CanTransport::ReadResult::FRAME);
    CHECK(read_calls == 34);
    // A new link discards the third buffered frame from the preceding session.
    link.close();
    CHECK(link.open("/dev/mock", error));
    CHECK(link.read(frame, error) == CanTransport::ReadResult::EMPTY);
  }
  {
    Group group("expired or revoked replies cannot survive in CDC");
    for (bool expired : {false, true}) {
      wire.clear();
      CanTxQueue queue;
      const auto now = CanTxQueue::Clock::now();
      queue.queue(old.can_id, now);
      mode = WriteMode::Blocked;
      CHECK(link.write(old, error) == CanTransport::WriteResult::BACKPRESSURE);
      CHECK(wire.empty());
      if (expired) {
        std::uint64_t drops = 0;
        CHECK(!queue.front(now + std::chrono::milliseconds(100), std::chrono::milliseconds(100), drops));
        CHECK(drops == 1);
      } else {
        CHECK(queue.drop(old.can_id) == 1);
      }
      mode = WriteMode::Ready;
      CHECK(link.write(fresh, error) == CanTransport::WriteResult::SENT);
      CHECK(types() == std::vector<unsigned>{CanProtocol::TARGET_ID});
    }
  }
  {
    Group group("retry has one owner and uses current payload");
    for (auto failure : {WriteMode::Blocked, WriteMode::Interrupted}) {
      wire.clear();
      mode = failure;
      const auto result = link.write(old, error);
      CHECK(result == (failure == WriteMode::Blocked ? CanTransport::WriteResult::BACKPRESSURE :
                                                       CanTransport::WriteResult::INTERRUPTED));
      mode = WriteMode::Ready;
      CHECK(link.write(old, error) == CanTransport::WriteResult::SENT);
      CHECK(types() == std::vector<unsigned>{CanProtocol::TURN_REQUEST_ID});
    }
    wire.clear();
    mode = WriteMode::Blocked;
    CHECK(link.write(CanProtocol::center(3, true, 100, 100, 10), error) == CanTransport::WriteResult::BACKPRESSURE);
    mode = WriteMode::Ready;
    CHECK(link.write(fresh, error) == CanTransport::WriteResult::SENT);
    CHECK(types() == std::vector<unsigned>{3});
    CHECK(wire[4] == 0); // No stale valid center before the current invalid center.
  }
  {
    Group group("partial and failed writes are not queued for later");
    mode = WriteMode::Short;
    bool fatal = false;
    try {link.write(old, error);} catch (const CanTransport::FatalError &) {fatal = true;}
    CHECK(fatal);
    link.close();
    CHECK(!link.socketOpen());
    CHECK(link.open("/dev/mock", error));
    wire.clear();
    mode = WriteMode::Ready;
    CHECK(link.write(fresh, error) == CanTransport::WriteResult::SENT);
    CHECK(types() == std::vector<unsigned>{3});
    for (auto failure : {WriteMode::Zero, WriteMode::Failed}) {
      mode = failure;
      CHECK(link.write(old, error) == CanTransport::WriteResult::FAILED);
      CHECK(!error.empty());
    }
  }
  return finishTests("CDC write ownership");
}
