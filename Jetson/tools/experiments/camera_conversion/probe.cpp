// Copyright 2026 jetson_yolo integration
// SPDX-License-Identifier: Apache-2.0
// 真实生产转换为参考；穷举与微基准不打开相机、不加载模型、不代表整链性能。
#include "camera/full_range_i420.hpp"
#include "camera/usb_camera.hpp"
#include "checks/sequence_sha256.hpp"
#include <opencv2/imgcodecs.hpp>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <numeric>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace fs = std::filesystem;
namespace candidate = vision_camera_detail::fused_i420;
using Clock = std::chrono::steady_clock;
namespace {
void require(bool condition, const std::string & reason)
{if (!condition) {throw std::runtime_error(reason);}}

std::string quote(const std::string & text)
{
  std::ostringstream out;
  out << '"';
  for (const unsigned char c : text) {
    if (c == '"' || c == '\\') {out << '\\' << static_cast<char>(c);}
    else if (c < 32) {out << "\\u" << std::hex << std::setw(4) << std::setfill('0') << int(c);}
    else {out << static_cast<char>(c);}
  }
  return out.str() + '"';
}

std::string pixelHash(const cv::Mat & image)
{
  field_sequence::Sha256 hash;
  for (int y = 0; y < image.rows; ++y) {
    hash.append(image.ptr(y), static_cast<std::size_t>(image.cols) * image.elemSize());
  }
  return hash.finish();
}

void equal(const cv::Mat & actual, const cv::Mat & expected, const std::string & label)
{
  require(actual.size() == expected.size() && actual.type() == expected.type(),
    label + ": shape/type mismatch");
  const std::size_t bytes = actual.cols * actual.elemSize();
  for (int y = 0; y < actual.rows; ++y) {
    if (std::memcmp(actual.ptr(y), expected.ptr(y), bytes) == 0) {continue;}
    for (std::size_t x = 0; x < bytes; ++x) {
      if (actual.ptr(y)[x] != expected.ptr(y)[x]) {
        throw std::runtime_error(label + ": row=" + std::to_string(y) +
          " byte=" + std::to_string(x) + " actual=" + std::to_string(actual.ptr(y)[x]) +
          " expected=" + std::to_string(expected.ptr(y)[x]));
      }
    }
  }
}

void fill(cv::Mat & image, unsigned int seed)
{
  std::uint32_t value = seed;
  for (int y = 0; y < image.rows; ++y) {
    for (int x = 0; x < image.cols; ++x) {
      value = value * 1664525u + 1013904223u;
      image.ptr<unsigned char>(y)[x] = static_cast<unsigned char>(value >> 24);
    }
  }
}

struct Options {
  fs::path output, input;
  std::string run{"all"}, source_id;
  double seconds{10}, warmup{2};
  int rounds{3}, threads{6};
};

Options parse(int argc, char ** argv)
{
  Options options;
  for (int i = 1; i < argc; ++i) {
    const std::string key = argv[i];
    require(i + 1 < argc, "missing value for " + key);
    const std::string value = argv[++i];
    if (key == "--output") {options.output = value;}
    else if (key == "--input-i420") {options.input = value;}
    else if (key == "--run") {options.run = value;}
    else if (key == "--source-id") {options.source_id = value;}
    else if (key == "--seconds") {options.seconds = std::stod(value);}
    else if (key == "--warmup-seconds") {options.warmup = std::stod(value);}
    else if (key == "--rounds") {options.rounds = std::stoi(value);}
    else if (key == "--threads") {options.threads = std::stoi(value);}
    else {throw std::runtime_error("unknown argument: " + key);}
  }
  require(!options.output.empty() && !options.source_id.empty(), "--output/--source-id required");
  require(options.run == "all" || options.run == "contract" || options.run == "bench",
    "--run must be all, contract or bench");
  require(options.run == "contract" || !options.input.empty(), "--input-i420 required for bench");
  require(std::isfinite(options.seconds) && options.seconds > 0 && options.seconds <= 600 &&
    std::isfinite(options.warmup) && options.warmup >= 0 && options.warmup <= 600 &&
    options.rounds > 0 && options.rounds <= 20 && options.threads > 0 && options.threads <= 64,
    "invalid duration/rounds/threads");
  require(!fs::exists(options.output), "output exists; evidence will not be overwritten");
  return options;
}

void contracts(std::ostream & out)
{
  const int original_threads = cv::getNumThreads();
  const auto started = Clock::now();
  std::uint64_t triples = 0, pixels = 0, boundary_cases = 0, invalid_cases = 0;
  // 一个 512x512 帧的 65536 个2x2块分别携带全部 U/V 组合，256帧遍历所有 Y。
  // 因每组色度共享四个像素，唯一颜色三元组数与实际像素比较数分别记录。
  cv::Mat packed(768, 512, CV_8UC1);
  auto * raw = packed.ptr<unsigned char>();
  for (int u = 0; u < 256; ++u) {
    for (int v = 0; v < 256; ++v) {
      const int index = u * 256 + v;
      raw[512 * 512 + index] = static_cast<unsigned char>(u);
      raw[512 * 512 + 65536 + index] = static_cast<unsigned char>(v);
    }
  }
  for (const int threads : {1, 6}) {
    cv::setNumThreads(threads);  // 只在独立工具进程验证两种环境，生产不修改线程配置。
    for (int y = 0; y < 256; ++y) {
      packed.rowRange(0, 512).setTo(cv::Scalar(y));
      const auto before = packed.clone();
      const auto reference = vision_camera::fullRangeI420ToBgr(packed);
      for (const auto mode : {candidate::Mode::Scalar, candidate::Mode::Native}) {
        const auto actual = candidate::convert(packed, mode);
        equal(actual, reference, "all YUV threads=" + std::to_string(threads) +
          " Y=" + std::to_string(y) + " mode=" +
          (mode == candidate::Mode::Scalar ? "scalar" : "native"));
        triples += 65536;
        pixels += actual.total();
      }
      equal(packed, before, "all-colour input unchanged");
    }
  }

  for (const int threads : {1, 6}) {
    cv::setNumThreads(threads);
    for (const int width : {2, 4, 14, 16, 18, 30, 32, 34, 62, 64, 66, 126, 128, 130,
        254, 256, 258, 1278, 1280, 1282, 1918, 1920, 1922}) {
      for (const int height : {2, 4, 6, 10, 14, 18}) {
        cv::Mat storage(height * 3 / 2 + 4, width + 13, CV_8UC1);
        fill(storage, static_cast<unsigned int>(width * 65537 + height));
        const auto before = storage.clone();
        const auto input = storage(cv::Rect(3, 1, width, height * 3 / 2));
        const auto reference = vision_camera::fullRangeI420ToBgr(input);
        for (const auto mode : {candidate::Mode::Scalar, candidate::Mode::Native}) {
          equal(candidate::convert(input, mode), reference, "strided boundary");
          ++boundary_cases;
        }
        equal(storage, before, "input plus padding unchanged");
      }
    }
    for (const auto size : {cv::Size(1280, 720), cv::Size(1282, 722),
        cv::Size(1920, 1080), cv::Size(1922, 1082)}) {
      cv::Mat storage(size.height * 3 / 2 + 2, size.width + 17, CV_8UC1);
      fill(storage, static_cast<unsigned int>(size.width + size.height));
      const auto before = storage.clone();
      const auto input = storage(cv::Rect(1, 1, size.width, size.height * 3 / 2));
      const auto reference = vision_camera::fullRangeI420ToBgr(input);
      auto first = candidate::convert(input);
      auto second = candidate::convert(input);
      equal(first, reference, "parallel full frame");
      equal(second, reference, "second independent frame");
      require(first.data != second.data && first.u && first.u->refcount == 1 &&
        second.u && second.u->refcount == 1, "output is not independently owned");
      first.setTo(cv::Scalar(5, 7, 11));
      equal(second, reference, "mutating first output changes retained second");
      equal(storage, before, "full-frame source/padding changed");
      ++boundary_cases;
    }
  }
  for (const auto & invalid : {cv::Mat{}, cv::Mat(3, 3, CV_8UC1),
      cv::Mat(4, 2, CV_8UC1), cv::Mat(3, 2, CV_8UC3), cv::Mat(3, 2, CV_16UC1)}) {
    bool original_rejected = false, scalar_rejected = false, native_rejected = false;
    try {(void)vision_camera::fullRangeI420ToBgr(invalid);} catch (const cv::Exception & e) {
      original_rejected = e.code == cv::Error::StsBadArg;
    }
    try {(void)candidate::convert(invalid, candidate::Mode::Scalar);} catch (const cv::Exception & e) {
      scalar_rejected = e.code == cv::Error::StsBadArg;
    }
    try {(void)candidate::convert(invalid);} catch (const cv::Exception & e) {
      native_rejected = e.code == cv::Error::StsBadArg;
    }
    require(original_rejected && scalar_rejected && native_rejected, "invalid-input behavior differs");
    ++invalid_cases;
  }
  cv::setNumThreads(original_threads);
  out << "{\"status\":\"executed\",\"unique_colour_triples_per_mode_and_thread_count\":16777216,"
    "\"colour_triplet_comparisons\":" << triples << ",\"pixel_comparisons\":" << pixels <<
    ",\"modes\":[\"scalar\",\"native\"],\"thread_counts\":[1,6],\"boundary_cases\":" <<
    boundary_cases << ",\"invalid_cases\":" << invalid_cases <<
    ",\"elapsed_seconds\":" << std::chrono::duration<double>(Clock::now() - started).count() << '}';
}

void benchmarks(std::ostream & out, const Options & options)
{
  const auto input = cv::imread(options.input.string(), cv::IMREAD_UNCHANGED);
  require(!input.empty(), "cannot load fixed I420 PNG");
  const auto input_hash = pixelHash(input);
  const auto reference = vision_camera::fullRangeI420ToBgr(input);
  equal(candidate::convert(input), reference, "fixed input preflight");
  cv::setNumThreads(options.threads);
  out << "{\"input\":" << quote(fs::absolute(options.input).string()) <<
    ",\"i420_pixel_sha256\":" << quote(input_hash) <<
    ",\"bgr_pixel_sha256\":" << quote(pixelHash(reference)) <<
    ",\"width\":" << reference.cols << ",\"height\":" << reference.rows <<
    ",\"threads\":" << cv::getNumThreads() << ",\"windows\":[";
  bool first_window = true;
  for (int round = 0; round < options.rounds; ++round) {
    // 交错先后顺序，两个路径每窗都单独预热，结果只表示独立转换函数耗时。
    const std::array<bool, 2> order = round % 2 == 0 ?
      std::array<bool, 2>{false, true} : std::array<bool, 2>{true, false};
    for (bool native : order) {
      const std::string mode = native ? "fused-native" : "production";
      const auto operation = [&] {
          return native ? candidate::convert(input) : vision_camera::fullRangeI420ToBgr(input);
        };
      cv::Mat result;
      auto started = Clock::now();
      while (std::chrono::duration<double>(Clock::now() - started).count() < options.warmup) {
        result = operation();
      }
      std::vector<double> samples;
      samples.reserve(static_cast<std::size_t>(options.seconds * 20000));
      started = Clock::now();
      while (std::chrono::duration<double>(Clock::now() - started).count() < options.seconds) {
        const auto begin = Clock::now();
        result = operation();
        samples.push_back(std::chrono::duration<double, std::milli>(Clock::now() - begin).count());
      }
      const auto elapsed = std::chrono::duration<double>(Clock::now() - started).count();
      require(!samples.empty(), "empty benchmark window");
      equal(result, reference, "benchmark output");
      require(input_hash == pixelHash(input), "benchmark changed input");
      const std::string csv_name = "round-" + std::to_string(round + 1) + "-" + mode + ".csv";
      std::ofstream csv(options.output / csv_name);
      csv << "sample,conversion_ms\n" << std::setprecision(10);
      for (std::size_t i = 0; i < samples.size(); ++i) {csv << i << ',' << samples[i] << '\n';}
      require(static_cast<bool>(csv), "cannot write benchmark samples");
      // 最后一段可能仍在流缓冲中；刷新和关闭都成功后，才登记该窗口证据。
      csv.flush();
      require(static_cast<bool>(csv), "cannot flush benchmark samples");
      csv.close();
      require(static_cast<bool>(csv), "cannot close benchmark samples");
      const auto mean = std::accumulate(samples.begin(), samples.end(), 0.0) / samples.size();
      std::sort(samples.begin(), samples.end());
      const auto p95 = samples[static_cast<std::size_t>(std::ceil(samples.size() * .95)) - 1];
      if (!first_window) {out << ',';} first_window = false;
      out << "{\"round\":" << round + 1 << ",\"mode\":" << quote(mode) <<
        ",\"warmup_seconds\":" << options.warmup << ",\"elapsed_seconds\":" << elapsed <<
        ",\"samples\":" << samples.size() << ",\"mean_ms\":" << mean <<
        ",\"p95_ms\":" << p95 << ",\"max_ms\":" << samples.back() <<
        ",\"samples_csv\":" << quote(csv_name) << '}';
      std::cout << "round " << round + 1 << ' ' << mode <<
        " mean_ms=" << mean << " p95_ms=" << p95 << '\n' << std::flush;
    }
  }
  out << "]}";
}
}  // namespace

int main(int argc, char ** argv)
{
  fs::path output;
  try {
#if !defined(__linux__)
    throw std::runtime_error("project execution is restricted to the Jetson/Linux board");
#endif
    const auto options = parse(argc, argv);
    output = options.output;
    fs::create_directories(output);
    std::ostringstream report;
    report << std::setprecision(10) <<
      "{\"schema\":\"fused-i420-experiment-v1\",\"source_id\":" << quote(options.source_id) <<
      ",\"opencv_version\":" << quote(CV_VERSION) <<
      ",\"reference\":\"linked production fullRangeI420ToBgr\","
      "\"arithmetic_lut_matches_original\":" << (candidate::tables().arithmetic_equivalent ? "true" : "false") <<
      ",\"native_neon_enabled\":" << (candidate::nativeNeonEnabled() ? "true" : "false") <<
      ",\"whole_pipeline_measurement\":false,\"model_execution\":false";
#if defined(__aarch64__)
    require(candidate::nativeNeonEnabled(), "AArch64 experiment did not enable its NEON path");
#endif
    if (options.run != "bench") {
      report << ",\"contract\":";
      contracts(report);
      std::cout << "exhaustive and boundary contracts completed\n" << std::flush;
    }
    if (options.run != "contract") {
      report << ",\"benchmark\":";
      benchmarks(report, options);
    }
    report << ",\"status\":\"executed\"}\n";
    std::ofstream stream(output / "result.json");
    stream << report.str();
    require(static_cast<bool>(stream), "cannot write result.json");
    // 不依赖析构时的静默刷新；延迟写入失败必须使本次工具运行返回非零。
    stream.flush();
    require(static_cast<bool>(stream), "cannot flush result.json");
    stream.close();
    require(static_cast<bool>(stream), "cannot close result.json");
    return 0;
  } catch (const std::exception & error) {
    std::cerr << "camera_fused_probe: " << error.what() << '\n';
    if (!output.empty() && fs::exists(output)) {
      std::ofstream failure(output / "failure.json");
      failure << "{\"status\":\"failed\",\"error\":" << quote(error.what()) << "}\n";
    }
    return 1;
  }
}
