// 必须通过 LD_PRELOAD 加载。只允许 TensorRT runtime 反序列化；任何 builder 请求立即终止。
#include <atomic>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <fcntl.h>
#include <unistd.h>

namespace {
std::atomic<unsigned> attempts{0};
void record(const char* event, unsigned count) noexcept {
  const char* path = std::getenv("DS_COMPARE_GUARD_LOG");
  if (!path) _exit(87);
  const int fd = open(path, O_WRONLY | O_CREAT | O_APPEND | O_CLOEXEC, 0600);
  if (fd < 0) _exit(87);
  char line[256];
  const int size = std::snprintf(line, sizeof(line),
      "{\"event\":\"%s\",\"pid\":%ld,\"attempts\":%u}\n",
      event, static_cast<long>(getpid()), count);
  if (write(fd, line, static_cast<size_t>(size)) != size) _exit(87);
  fsync(fd);
  close(fd);
}
__attribute__((constructor)) void loaded() { record("guard_loaded", 0); }
}

extern "C" const char* ds_compare_guard_identity() noexcept {
  return "deepstream_compare_no_engine_build_v1";
}
extern "C" unsigned ds_compare_guard_attempts() noexcept { return attempts.load(); }
// NVIDIA NvInfer.h 中的真实 C ABI；不能转调 RTLD_NEXT 或返回可继续工作的 builder。
extern "C" void* createInferBuilder_INTERNAL(void*, int32_t) noexcept {
  record("builder_blocked", ++attempts);
  _exit(86);
}
