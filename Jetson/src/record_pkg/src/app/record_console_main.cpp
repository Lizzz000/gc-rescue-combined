// ---- PC 录制入口 ----
#include "record_console.hpp"
// HTTP/CLI 与录像控制共用实现；测试目标不编译程序入口。
#include "record_web.hpp"
#include <httplib.h>
#include <atomic>
#include <charconv>
#include <csignal>
#include <iostream>
#include <thread>
extern "C" {
#include <libavutil/log.h>
}
#ifdef _WIN32
#include <windows.h>
#endif

namespace {
std::atomic<bool> stopped{false};
void stop_signal(int) { stopped.store(true); }
#ifdef _WIN32
BOOL WINAPI stop_console(DWORD kind) {
  if (kind == CTRL_C_EVENT || kind == CTRL_BREAK_EVENT) { stopped.store(true); return TRUE; }
  return FALSE;
}
#endif
/**
 * @brief 在 CLI 解析点一次检查整数文本和该选项的最终范围。
 * @param[in] text 十进制整数文本，拒绝尾随字符。
 * @param[in] low 允许的最小整数，含边界。
 * @param[in] high 允许的最大整数，含边界且不小于 low。
 * @return 已校验整数；非法时抛异常，不创建输出或网络资源。
 */
int option_number(const std::string& text, int low, int high) {
  int n = 0; const auto p = std::from_chars(text.data(), text.data() + text.size(), n);
  if (p.ec != std::errc{} || p.ptr != text.data() + text.size() || n < low || n > high)
    throw std::invalid_argument("invalid numeric option");
  return n;
}
}
/**
 * @brief 一次解析 PC 参数，随后运行预览、录制控制和接收线程。
 * @param[in] argc 参数个数，至少为 1。
 * @param[in] argv 非空参数数组，元素不得为空指针。
 * @return 0 正常退出；2 参数错误；1 运行失败并完成线程与录像收尾。
 */
int main(int argc, char** argv) {
  try {
    record::ConsoleOptions options;
    for (int i = 1; i < argc; ++i) {
      const std::string key = argv[i];
      if (key == "--help") {
        std::cout << "record_console: PC preview and H264 MP4 recording\n"
          "--output-dir D:/A1_Workspace/GC_Rescue/MP4 --port 8090\n"
          "--camera-host 127.0.0.1 --camera-port 18091 --min-free-mib 1024 --encoder x264 (auto/nvenc accepted, mapped to libx264)\n";
        return 0;
      }
      if (i + 1 == argc) throw std::invalid_argument("missing value for " + key);
      const std::string value = argv[++i];
      if (key == "--output-dir") options.output_dir = std::filesystem::u8path(value);
      else if (key == "--encoder") options.encoder = value;
      else if (key == "--camera-host") options.camera_host = value;
      else if (key == "--camera-port") options.camera_port = option_number(value, 1, 65535);
      else if (key == "--port") options.port = option_number(value, 1024, 65535);
      else if (key == "--min-free-mib") options.min_free_mib = option_number(value, 1, 1048576);
      else throw std::invalid_argument("unknown option " + key);
    }
    // 数字已在各选项解析点按最终范围校验；这里只检查文本选项。
    if (options.camera_host.empty() || options.output_dir.empty() ||
        (options.encoder != "auto" && options.encoder != "nvenc" && options.encoder != "x264")) throw std::invalid_argument("invalid host, port or output directory");
    if (options.encoder != "x264") {
      std::cout << "--encoder " << options.encoder << ": hardware encoder ignored, using libx264\n";
      options.encoder = "x264";
    }
    av_log_set_level(AV_LOG_FATAL); // 可处理的解码错误由收据和界面报告，避免相机APP元数据刷屏。
    record::RecordConsole console(options);
    httplib::Server server;
    server.new_task_queue = [] { return new httplib::ThreadPool(4, 4, 16); };
    server.set_read_timeout(2, 0).set_write_timeout(2, 0).set_payload_max_length(4096);
    server.set_default_headers({{"Cache-Control", "no-store"}, {"X-Content-Type-Options", "nosniff"}});
    const auto output = std::filesystem::canonical(options.output_dir);
    server.Get("/", [&](const httplib::Request&, httplib::Response& res) { res.set_content(record::web_page, "text/html; charset=utf-8"); });
    server.Get("/api/status", [&](const httplib::Request&, httplib::Response& res) { res.set_content(console.status().dump(), "application/json; charset=utf-8"); });
    server.Get("/preview.jpg", [&](const httplib::Request&, httplib::Response& res) {
      const auto frame = console.preview();
      if (!frame) { res.status = 503; return; }
      res.set_header("X-Sequence", std::to_string(frame->sequence));
      res.set_content(frame->jpeg, "image/jpeg");
    });
    server.Get(R"(/files/([A-Za-z0-9_.-]+\.mp4))", [&](const httplib::Request& req, httplib::Response& res) {
      const auto name = req.matches[1].str();
      const auto path = output / name;
      if (!std::filesystem::is_regular_file(path) || std::filesystem::canonical(path).parent_path() != output) { res.status = 404; return; }
      res.set_file_content(path.u8string(), "video/mp4");
    });
    auto control = [&](bool start, const httplib::Request& req, httplib::Response& res) {
      try {
        const auto current = console.status();
        // 只绑定 127.0.0.1；保留 X-Record-Token，Host/Origin 校验已收敛删除。
        if (req.get_header_value("X-Record-Token") != current["token"]) {
          res.status = 403; res.set_content(R"({"error":"控制请求来源无效"})", "application/json; charset=utf-8"); return;
        }
        const auto body = nlohmann::json::parse(req.body);
        if (start) console.start(body.at("request_id").get<std::string>(), body.at("start_generation").get<std::uint64_t>());
        else console.stop(body.at("run_id").get<std::string>());
        res.set_content(console.status().dump(), "application/json; charset=utf-8");
      } catch (const std::exception& e) {
        res.status = 409; res.set_content(nlohmann::json{{"error", e.what()}}.dump(), "application/json; charset=utf-8");
      }
    };
    server.Post("/api/start", [&](const auto& req, auto& res) { control(true, req, res); });
    server.Post("/api/stop", [&](const auto& req, auto& res) { control(false, req, res); });
    server.Post("/api/shutdown", [&](const httplib::Request& req, httplib::Response& res) {
      if (req.get_header_value("X-Record-Token") != console.status()["token"]) { res.status = 403; return; }
      stopped.store(true); res.set_content("{}", "application/json");
    });
    if (!server.bind_to_port("127.0.0.1", options.port)) throw std::runtime_error("local console port is already in use");
    std::signal(SIGINT, stop_signal); std::signal(SIGTERM, stop_signal);
#ifdef _WIN32
    SetConsoleCtrlHandler(stop_console, TRUE);
#else
    std::signal(SIGPIPE, SIG_IGN);
#endif
    httplib::Client camera(options.camera_host, options.camera_port);
    camera.set_connection_timeout(2, 0); camera.set_read_timeout(2, 0); camera.set_write_timeout(2, 0);
    std::thread receiver, http;
    std::atomic<bool> http_done{false}, http_failed{false};
    auto cleanup = [&] {
      stopped = true; server.stop(); camera.stop();
      if (receiver.joinable()) receiver.join();
      if (http.joinable()) http.join();
      console.shutdown();
    };
    try {
    receiver = std::thread([&] {
      while (!stopped.load()) {
        std::string failure;
        try {
        auto progressed = record::monotonic_ns();
        record::FrameParser parser([&](record::FramePtr frame) { progressed = record::monotonic_ns(); console.receive(std::move(frame)); });
        const auto result = camera.Get("/stream.mjpg", [&](const httplib::Response& res) {
          return res.status == 200 && res.get_header_value("Content-Type").find("multipart/x-mixed-replace") == 0;
        }, [&](const char* bytes, std::size_t length) {
          if (stopped.load()) return false;
          if (record::monotonic_ns() - progressed > 2000000000ULL) { failure = "完整相机帧接收超时，正在重连"; return false; }
          try { parser.feed(bytes, length); return true; }
          catch (const std::exception& e) { failure = e.what(); return false; }
        });
        } catch (const std::exception& e) { failure = e.what(); }
        if (!stopped.load()) console.disconnected(failure.empty() ? "相机连接已断开，正在重连" : failure);
        for (int i = 0; i < 10 && !stopped.load(); ++i) std::this_thread::sleep_for(std::chrono::milliseconds(100));
      }
    });
    http = std::thread([&] {
      try { if (!server.listen_after_bind() || !stopped.load()) http_failed = true; }
      catch (const std::exception& e) { http_failed = true; std::cerr << "HTTP server failed: " << e.what() << '\n'; }
      http_done = true;
    });
    std::cout << "Record console: http://127.0.0.1:" << options.port << "/\nOutput: " << output.u8string() << std::endl;
    while (!stopped.load() && !http_done.load()) {
      console.tick();
      for (int i = 0; i < 10 && !stopped.load() && !http_done.load(); ++i) std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    } catch (...) { cleanup(); throw; }
    cleanup();
    return http_failed.load() ? 1 : 0;
  } catch (const std::invalid_argument& e) { std::cerr << e.what() << '\n'; return 2; }
    catch (const std::exception& e) { std::cerr << e.what() << '\n'; return 1; }
}
