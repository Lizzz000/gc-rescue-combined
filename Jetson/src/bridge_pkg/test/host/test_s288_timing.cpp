// Real Session worker, exchange and Stop retry loops; only POSIX device calls are fake.
#include "check.hpp"
#include "gripper/s288_servo.hpp"
#include <sys/select.h>
#include <unistd.h>
#include <algorithm>
#include <array>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <cstring>
#include <iostream>
#include <thread>

namespace {
std::array<unsigned char, 26> feedback{};
std::atomic<int> stop_calls{0};
bool timeout_reply = false;
bool have_feedback = false;
int fault_stop_call = 7;
unsigned stop_fault = 0;
std::uint32_t crc(const unsigned char *data)
{
  std::uint32_t value = 0xffffffff;
  for (int offset = 0; offset < 20; offset += 4) {
    std::uint32_t word = 0;
    for (int i = 0; i < 4; ++i) {word |= std::uint32_t(data[offset + i]) << (8 * i);}
    value ^= word;
    for (int i = 0; i < 32; ++i) {value = (value << 1) ^ ((value & 0x80000000u) ? 0x04c11db7u : 0u);}
  }
  return value;
}
}

int open(const char *, int, ...) {return 42;}
int close(int) {return 0;}
int flock(int, int) {return 0;}
int ioctl(int, unsigned long, ...) {return 0;}
ssize_t write(int, const void *raw, std::size_t size)
{
  const auto *packet = static_cast<const unsigned char *>(raw);
  const auto mode = (packet[2] >> 4) & 7;
  const int count = mode == 0 ? ++stop_calls : 0;
  // Prepare and final Stop each time out twice before receiving feedback.
  timeout_reply = count == 2 || count == 3 || count == 5 || count == 6;
  feedback.fill(0);
  feedback[0] = 0xfc;
  feedback[1] = 0xee;
  feedback[2] = packet[2] & 0x7f;
  if (count == fault_stop_call && stop_fault == 1) {feedback[14] = 1;}
  if (count == fault_stop_call && stop_fault == 2) {feedback[2] |= 0x80;}
  const auto check = crc(feedback.data() + 2);
  for (int i = 0; i < 4; ++i) {feedback[22 + i] = static_cast<unsigned char>(check >> (8 * i));}
  have_feedback = true;
  return static_cast<ssize_t>(size);
}
ssize_t read(int, void *out, std::size_t size)
{
  if (!have_feedback) {errno = EAGAIN; return -1;}
  const auto count = std::min(size, feedback.size());
  std::memcpy(out, feedback.data(), count);
  have_feedback = false;
  return static_cast<ssize_t>(count);
}
int select(int, fd_set *, fd_set *, fd_set *, timeval *timeout)
{
  if (!timeout_reply) {return 1;}
  std::this_thread::sleep_for(std::chrono::seconds(timeout->tv_sec) +
                            std::chrono::microseconds(timeout->tv_usec));
  return 0;
}

int main()
{
  Group group("maximum S288 motion includes both Stop retry sequences");
  s288::SessionConfig config;
  config.device = "/dev/mock";
  s288::Session session(config);
  CHECK(session.available());
  const auto began = std::chrono::steady_clock::now();
  CHECK(session.start_close(s288::kMaxMoveS));
  while (session.status() == s288::MotionStatus::Busy &&
         std::chrono::steady_clock::now() - began < std::chrono::seconds(20)) {
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  const double elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - began).count();
  CHECK(session.status() == s288::MotionStatus::Succeeded);
  CHECK(stop_calls == 7);
  CHECK(elapsed >= s288::kMaxMoveS + 4 * s288::kFeedbackWaitS);
  std::cout << "successful action including feedback retries: " << elapsed << " s\n";
  session.close();

  Group failure_group("S288 final Stop reports hardware error or timeout");
  for (unsigned fault : {1U, 2U}) {
    stop_calls = 0;
    stop_fault = fault;
    s288::Session failing_session(config);
    CHECK(failing_session.available());
    CHECK(failing_session.start_close(s288::kMinMoveS));
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);
    while (failing_session.status() == s288::MotionStatus::Busy &&
           std::chrono::steady_clock::now() < deadline) {
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    CHECK(failing_session.status() == s288::MotionStatus::Failed);
    CHECK(stop_calls == 7);
    failing_session.close();
  }
  Group zero_group("S288 rejects a faulted power-on zero");
  fault_stop_call = 1;
  for (unsigned fault : {1U, 2U}) {
    stop_calls = 0;
    stop_fault = fault;
    s288::Session failing_session(config);
    CHECK(!failing_session.available());
    CHECK(!failing_session.start_open(s288::kMinMoveS));
  }
  return finishTests("S288 timing");
}
