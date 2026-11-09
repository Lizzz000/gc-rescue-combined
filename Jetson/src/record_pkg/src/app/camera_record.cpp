// ---- 板端采集入口 ----
// 职责：HTTP 和进程入口。
#include "gstreamer/camera_stream.hpp"
#include <httplib.h>
#include <nlohmann/json.hpp>
#include <algorithm>
#include <atomic>
#include <charconv>
#include <csignal>
#include <iostream>
#include <stdexcept>
#include <thread>

namespace record {
namespace {
// 信号处理只置位；HTTP 与媒体线程由 run_camera 的控制流程统一收尾。
std::atomic<bool> interrupted{false};
void signal_stop(int) { interrupted.store(true); }
int number(const std::string& s, int low, int high) {
  int n = 0;
  auto p = std::from_chars(s.data(), s.data() + s.size(), n);
  if (p.ec != std::errc{} || p.ptr != s.data() + s.size() || n < low || n > high)
    throw std::invalid_argument("numeric option outside supported range: " + s);
  return n;
}
}
// 板端只接受采集和服务参数，录像路径及会话控制留在 PC。
CameraOptions camera_arguments(int argc, char** argv) {
  CameraOptions o;
  for (int i = 1; i < argc; ++i) {
    std::string key = argv[i];
    if (i + 1 == argc) throw std::invalid_argument("missing value: " + key);
    const std::string value = argv[++i];
    if (key == "--device") o.device = value;
    else if (key == "--bind") o.bind = value;
    else if (key == "--port") o.port = number(value, 1024, 65535);
    else if (key == "--width") o.width = number(value, 16, 8192);
    else if (key == "--height") o.height = number(value, 16, 8192);
    else if (key == "--fps") o.fps = number(value, 1, 240);
    else if (key == "--idle-seconds") o.idle_seconds = number(value, 5, 3600);
    else throw std::invalid_argument("unknown option: " + key + "; video output and recording controls now belong to PC record_console");
  }
  if (o.device.empty() || o.bind.empty() || o.width % 2 || o.height % 2)
    throw std::invalid_argument("device/bind must be nonempty; dimensions must be even");
  return o;
}
// HTTP 与退出协调：连接各自维护游标，仅发送帧引用，不参与相机采集或文件保存。
int run_camera(const CameraOptions& o) {
  interrupted.store(false);
  std::signal(SIGINT, signal_stop); std::signal(SIGTERM, signal_stop); std::signal(SIGHUP, signal_stop);
  std::signal(SIGPIPE, SIG_IGN);
  CameraStream camera(o);
  httplib::Server server;
  server.new_task_queue = [] { return new httplib::ThreadPool(4, 4, 8); };
  server.set_read_timeout(2, 0).set_write_timeout(1, 0).set_payload_max_length(1024);
  std::atomic<unsigned> clients{0};
  std::atomic<std::uint64_t> touched{monotonic_ns()};
  server.Get("/api/status", [&](const httplib::Request&, httplib::Response& res) {
    const auto f = camera.latest();
    nlohmann::json j{{"source", "record_pkg"}, {"board_video_files", false}, {"clients", clients.load()},
        {"camera_format", "YUYV"}, {"jpeg_encoder", "nvjpegenc"},
        {"frames", f ? f->sequence : 0}, {"width", o.width}, {"height", o.height}, {"fps", o.fps}};
    res.set_content(j.dump(), "application/json");
  });
  server.Get("/stream.mjpg", [&](const httplib::Request&, httplib::Response& res) {
    if (clients.fetch_add(1) >= 2) { --clients; res.status = 503; return; }
    res.set_header("Cache-Control", "no-store");
    auto cursor = std::make_shared<std::uint64_t>(0);
    res.set_content_provider("multipart/x-mixed-replace; boundary=gc-frame",
      [&, cursor](std::size_t, httplib::DataSink& sink) {
        try {
          // next 返回时已释放帧环锁；慢网络只阻塞本连接，落后时由下一次 next 拒绝。
          const auto f = camera.next(*cursor);
          if (!f) return false;
          touched.store(monotonic_ns());
          const auto header = frame_header(*f);
          return sink.write(header.data(), header.size()) && sink.write(f->jpeg.data(), f->jpeg.size()) && sink.write("\r\n", 2);
        } catch (...) { return false; }
      }, [&](bool) { --clients; });
  });
  if (!server.bind_to_port(o.bind, o.port)) throw std::runtime_error("camera stream port is already in use");
  camera.start();
  std::atomic<bool> server_done{false};
  std::thread network([&] {
    try { server.listen_after_bind(); }
    catch (const std::exception& e) { std::cerr << "HTTP server failed: " << e.what() << '\n'; }
    server_done = true;
  });
  std::cout << "Camera YUYV -> hardware JPEG stream " << o.bind << ':' << o.port << "; no board video output" << std::endl;
  std::string error;
  try { while (!interrupted.load()) {
    error = camera.poll_error();
    if (!error.empty() || server_done) break;
    if (monotonic_ns() - touched.load() > std::uint64_t(o.idle_seconds) * 1000000000ULL) break;
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  } } catch (...) { server.stop(); camera.stop(); network.join(); throw; }
  if (server_done && error.empty()) error = "camera HTTP server stopped unexpectedly";
  // 停止服务并唤醒等待帧的连接，等网络线程退出后才销毁其引用的相机和服务对象。
  server.stop(); camera.stop(); network.join();
  if (!error.empty()) std::cerr << error << '\n';
  return error.empty() ? 0 : 1;
}
}  // namespace record

#include <iostream>
#include <string>
/**
 * @brief 在唯一参数入口校验采集配置，再运行只传图的板端服务。
 * @param[in] argc 参数个数，至少为 1。
 * @param[in] argv 非空参数数组，元素不得为空指针。
 * @return 0 正常退出；2 参数错误；1 运行失败并释放相机与服务资源。
 */
int main(int argc, char** argv) {
  for (int i = 1; i < argc; ++i) if (std::string(argv[i]) == "--help") {
    std::cout << "camera_record: camera stream only; videos are saved by PC record_console.\n"
      "--device /dev/video0 --width 1920 --height 1080 --fps 30\n"
      "--bind 127.0.0.1 --port 8091 --idle-seconds 30\n";
    return 0;
  }
  try {
    const auto options = record::camera_arguments(argc, argv);
    gst_init(nullptr, nullptr);
    return record::run_camera(options);
  } catch (const std::invalid_argument& e) { std::cerr << e.what() << '\n'; return 2; }
    catch (const std::exception& e) { std::cerr << e.what() << '\n'; return 1; }
}
