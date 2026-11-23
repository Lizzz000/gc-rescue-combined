#include "camera/gstreamer/gstreamer.hpp"

#include <gst/gst.h>
#include <linux/videodev2.h>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <cstdarg>
#include <iostream>
#include <string>
#include <thread>

namespace {
constexpr int fake_fd = 31234;
int brightness = 0;
int writes = 0;
int reads = 0;
bool drift = false;

// Stand in for v4l2src, keeping its descriptor property and a live source pad.
struct ControlDevice { GstBin parent; };
struct ControlDeviceClass { GstBinClass parent; };
G_DEFINE_TYPE(ControlDevice, control_device, GST_TYPE_BIN)

void get_device_fd(GObject * object, guint id, GValue * value, GParamSpec * spec)
{
  if (id == 1) {g_value_set_int(value, fake_fd);}
  else {G_OBJECT_WARN_INVALID_PROPERTY_ID(object, id, spec);}
}

void control_device_init(ControlDevice * device)
{
  auto * source = gst_element_factory_make("videotestsrc", nullptr);
  if (!source) {return;}
  g_object_set(source, "is-live", TRUE, nullptr);
  gst_bin_add(GST_BIN(device), source);
  auto * pad = gst_element_get_static_pad(source, "src");
  gst_element_add_pad(GST_ELEMENT(device), gst_ghost_pad_new("src", pad));
  gst_object_unref(pad);
}
void control_device_class_init(ControlDeviceClass * klass)
{
  auto * object = G_OBJECT_CLASS(klass);
  object->get_property = get_device_fd;
  g_object_class_install_property(object, 1,
    g_param_spec_int("device-fd", "Device FD", "Test control descriptor", -1, G_MAXINT, -1, G_PARAM_READABLE));
  gst_element_class_set_static_metadata(GST_ELEMENT_CLASS(klass), "Test control device",
    "Generic/Bin", "Supplies a simulated V4L2 descriptor", "GC Rescue");
}
}  // namespace

// Intercepts only ioctl references linked into this executable. The shared
// GStreamer libraries retain their normal system calls.
extern "C" int __wrap_ioctl(int fd, unsigned long request, ...)
{
  if (fd != fake_fd) {errno = EBADF; return -1;}
  va_list args;
  va_start(args, request);
  auto * control = va_arg(args, v4l2_control *);
  va_end(args);
  if (control->id != V4L2_CID_BRIGHTNESS) {errno = EINVAL; return -1;}
  if (request == VIDIOC_S_CTRL) {
    if (++writes == 1) {std::this_thread::sleep_for(std::chrono::milliseconds(2200));}
    brightness = control->value;
  } else if (request == VIDIOC_G_CTRL) {
    ++reads;
    control->value = drift ? brightness - 1 : brightness;
  } else {
    errno = EINVAL;
    return -1;
  }
  return 0;
}

int main(int argc, char ** argv)
{
  gst_init(&argc, &argv);
  if (!gst_element_register(nullptr, "gccontroltestdevice", GST_RANK_NONE, control_device_get_type())) {
    std::cerr << "cannot register the test control device\n";
    return 1;
  }
  const std::string graph = "gccontroltestdevice name=rescue_camera_source "
    "! video/x-raw,format=BGR,width=32,height=24,framerate=30/1 "
    "! appsink max-buffers=1 drop=true sync=false";
  const vision_camera::CameraControls controls{{{"brightness", V4L2_CID_BRIGHTNESS}, 175}};
  const std::atomic<bool> cancelled{false};
  vision_camera::GstreamerCapture capture;
  cv::Mat frame;
  std::chrono::steady_clock::time_point at;
  std::shared_ptr<void> nvmm;
  if (!capture.open(graph, cancelled, true, controls) || reads != 1 || writes != 1 ||
    !capture.read(frame, at, cancelled, nvmm) || frame.size() != cv::Size(32, 24))
  {
    std::cerr << "slow control setup consumed the first-frame budget: " << capture.lastError() << '\n';
    return 1;
  }
  drift = true;
  if (capture.open(graph, cancelled, true, controls) || capture.isOpen() || reads != 2 ||
    writes != 2 || capture.lastError().find("brightness") == std::string::npos)
  {
    std::cerr << "a mismatched control readback must discard the primed frame\n";
    return 1;
  }
  // No data is pushed. The independent first-frame wait must still expire.
  if (capture.open("appsrc is-live=true ! appsink sync=false", cancelled) || capture.isOpen() ||
    capture.lastError().find("no frame") == std::string::npos)
  {
    std::cerr << "unexpected no-frame result: " << capture.lastError() << '\n';
    return 1;
  }
  return 0;
}
