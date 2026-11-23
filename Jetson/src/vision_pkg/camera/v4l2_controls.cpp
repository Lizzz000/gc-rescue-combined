#include "camera/v4l2_controls.hpp"

#include <cerrno>
#include <cstring>
#include <linux/videodev2.h>
#include <sys/ioctl.h>

namespace vision_camera {
bool cameraControls(int fd, const CameraControls & settings, ControlOperation operation,
  std::string & error)
{
  const bool writing = operation == ControlOperation::Apply;
  for (const auto & setting : settings) {
    v4l2_control value{};
    value.id = setting.control.id;
    value.value = setting.value;
    int result;
    do {
      result = ioctl(fd, writing ? VIDIOC_S_CTRL : VIDIOC_G_CTRL, &value);
    } while (result < 0 && errno == EINTR);
    if (result < 0) {
      const int failure = errno;
      error = std::string(writing ? "cannot set " : "cannot read ") +
        setting.control.name + ": " + std::strerror(failure);
      return false;
    }
    if (!writing && value.value != setting.value) {
      error = std::string(setting.control.name) + ": expected " + std::to_string(setting.value) +
        ", camera returned " + std::to_string(value.value);
      return false;
    }
  }
  return true;
}
}  // namespace vision_camera
