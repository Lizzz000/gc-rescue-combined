#pragma once

#include <linux/v4l2-controls.h>
#include <cstdint>
#include <string>
#include <vector>

namespace vision_camera {
struct CameraControl {
  const char * name;
  std::uint32_t id;
};

// Modes come first so manual exposure is writable when its value is applied.
inline constexpr CameraControl optional_controls[] = {
  {"auto_exposure", V4L2_CID_EXPOSURE_AUTO},
  {"white_balance_automatic", V4L2_CID_AUTO_WHITE_BALANCE},
  {"focus_automatic_continuous", V4L2_CID_FOCUS_AUTO},
  {"exposure_time_absolute", V4L2_CID_EXPOSURE_ABSOLUTE},
  {"exposure_dynamic_framerate", V4L2_CID_EXPOSURE_AUTO_PRIORITY},
  {"contrast", V4L2_CID_CONTRAST},
  {"saturation", V4L2_CID_SATURATION},
  {"hue", V4L2_CID_HUE},
  {"gamma", V4L2_CID_GAMMA},
  {"gain", V4L2_CID_GAIN},
  {"sharpness", V4L2_CID_SHARPNESS},
  {"power_line_frequency", V4L2_CID_POWER_LINE_FREQUENCY},
  {"backlight_compensation", V4L2_CID_BACKLIGHT_COMPENSATION},
  {"zoom_absolute", V4L2_CID_ZOOM_ABSOLUTE},
};

struct ControlSetting {
  CameraControl control;
  int value;
};
using CameraControls = std::vector<ControlSetting>;
enum class ControlOperation { Apply, Verify };
bool cameraControls(int fd, const CameraControls & settings, ControlOperation operation,
  std::string & error);
}  // namespace vision_camera
