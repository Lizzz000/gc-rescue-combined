#include "profile_hooks.hpp"
#include "yolo/yolo.hpp"
#include <opencv2/imgcodecs.hpp>
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace {
std::string quote(const std::string & value)
{
  std::ostringstream out;
  out << '"';
  for (unsigned char c : value) {
    if (c == '"' || c == '\\') {out << '\\' << c;}
    else if (c < 32) {out << "\\u" << std::hex << std::setw(4) << std::setfill('0') << int(c);}
    else {out << c;}
  }
  return out.str() + '"';
}
std::string required(const std::map<std::string, std::string> & args, const char * name)
{
  auto found = args.find(name);
  if (found == args.end()) {throw std::runtime_error(std::string("missing --") + name);}
  return found->second;
}
int number(const std::map<std::string, std::string> & args, const char * name, int fallback)
{
  auto found = args.find(name);
  if (found == args.end()) {return fallback;}
  std::size_t used = 0;
  const int value = std::stoi(found->second, &used);
  if (used != found->second.size()) {throw std::runtime_error("invalid integer");}
  return value;
}
double real(const std::map<std::string, std::string> & args, const char * name, double fallback)
{
  auto found = args.find(name);
  if (found == args.end()) {return fallback;}
  std::size_t used = 0;
  const double value = std::stod(found->second, &used);
  if (used != found->second.size() || !std::isfinite(value)) {
    throw std::runtime_error("invalid finite number");
  }
  return value;
}
void line(FILE * output, const std::string & text)
{
  if (std::fwrite(text.data(), 1, text.size(), output) != text.size() ||
    std::fputc('\n', output) == EOF || std::fflush(output) != 0)
  {
    throw std::runtime_error("cannot write diagnostic output");
  }
}
}  // namespace

int main(int argc, char ** argv)
{
  FILE * output = nullptr;
  try {
    std::map<std::string, std::string> args;
    const std::vector<std::string> accepted = {"engine", "labels", "images", "output",
      "warmup", "iterations", "mask", "strided", "confidence", "iou", "max-detections",
      "mask-scale", "mask-offset", "max-masks"};
    for (int i = 1; i < argc; i += 2) {
      if (i + 1 >= argc || std::string(argv[i]).rfind("--", 0) != 0) {
        throw std::runtime_error("arguments require --name value pairs");
      }
      const std::string key = std::string(argv[i]).substr(2);
      if (std::find(accepted.begin(), accepted.end(), key) == accepted.end() ||
        !args.emplace(key, argv[i + 1]).second)
      {
        throw std::runtime_error("unknown or duplicate option: " + key);
      }
    }
    const int warmup = number(args, "warmup", 20);
    const int iterations = number(args, "iterations", 100);
    const int mask = number(args, "mask", 0);
    const int strided = number(args, "strided", 0);
    if (warmup < 0 || iterations < 1 || mask < 0 || mask > 1 || strided < 0 || strided > 1) {
      throw std::runtime_error("invalid run counts or binary option");
    }
    std::ifstream paths(required(args, "images"));
    if (!paths) {throw std::runtime_error("cannot read image list");}
    std::vector<std::string> names;
    std::vector<cv::Mat> images;
    for (std::string name; std::getline(paths, name); ) {
      if (!name.empty() && name.back() == '\r') {name.pop_back();}
      if (name.empty()) {continue;}
      cv::Mat image = cv::imread(name, cv::IMREAD_COLOR);
      if (image.empty()) {throw std::runtime_error("cannot read input " + name);}
      if (strided) {
        cv::Mat padded(image.rows + 2, image.cols + 13, CV_8UC3, cv::Scalar(213, 37, 81));
        cv::Mat view = padded(cv::Rect(5, 1, image.cols, image.rows));
        image.copyTo(view);
        image = view;
        if (image.isContinuous()) {throw std::runtime_error("strided fixture is continuous");}
      }
      names.push_back(name);
      images.push_back(image);
    }
    if (images.empty()) {throw std::runtime_error("empty image list");}
    output = std::fopen(required(args, "output").c_str(), "wx");
    if (!output) {throw std::runtime_error("output exists or cannot be created");}
    SegOptions options;
    options.conf_threshold = static_cast<float>(real(args, "confidence", 0.75));
    options.iou_threshold = static_cast<float>(real(args, "iou", 0.7));
    options.max_detections = number(args, "max-detections", 300);
    options.mask_scale = number(args, "mask-scale", 2);
    options.mask_label_offset = number(args, "mask-offset", 1);
    options.max_masks = number(args, "max-masks", 32);
    options.want_label_mask = mask != 0;
    options.want_instance_masks = false;
    FastSegmentor detector(required(args, "engine"), required(args, "labels"));
    if (!detector.isEndToEnd()) {throw std::runtime_error("probe requires deployed end-to-end model");}
    {
      std::ostringstream meta;
      meta << std::setprecision(17)
           << "{\"kind\":\"metadata\",\"engine\":" << quote(required(args, "engine"))
           << ",\"labels\":" << quote(required(args, "labels"))
           << ",\"image_list\":" << quote(required(args, "images"))
           << ",\"warmup\":" << warmup << ",\"iterations_per_image\":" << iterations
           << ",\"mask\":" << mask << ",\"strided\":" << strided
           << ",\"confidence\":" << options.conf_threshold << ",\"iou\":" << options.iou_threshold
           << ",\"max_detections\":" << options.max_detections
           << ",\"mask_scale\":" << options.mask_scale << ",\"mask_offset\":" << options.mask_label_offset
           << ",\"max_masks\":" << options.max_masks
           << ",\"engine_build\":false,\"timing_only_source_instrumentation\":true"
           << ",\"sync_ms_overlaps_device_stages\":true}";
      line(output, meta.str());
    }
    cv::Mat label;
    for (int i = 0; i < warmup; ++i) {
      detector.segmentFast(images[static_cast<std::size_t>(i) % images.size()], options, label);
    }
    rescue_yolo_profile::Trace trace;
    for (int iteration = 0; iteration < iterations; ++iteration) {
      for (std::size_t image_index = 0; image_index < images.size(); ++image_index) {
        trace.reset();
        SegTiming timing;
        rescue_yolo_profile::active = &trace;
        const auto started = rescue_yolo_profile::Clock::now();
        auto detections = detector.segmentFast(images[image_index], options, label, &timing);
        const double total_ms = rescue_yolo_profile::milliseconds(
          rescue_yolo_profile::Clock::now() - started);
        rescue_yolo_profile::active = nullptr;
        std::ostringstream record;
        record << std::setprecision(17)
               << "{\"kind\":\"frame\",\"iteration\":" << iteration
               << ",\"image\":" << quote(names[image_index])
               << ",\"width\":" << images[image_index].cols << ",\"height\":" << images[image_index].rows
               << ",\"continuous\":" << (images[image_index].isContinuous() ? "true" : "false")
               << ",\"detections\":" << detections.size()
               << ",\"graph\":" << (trace.graph ? "true" : "false")
               << ",\"host_pack_ms\":" << trace.pack_ms << ",\"host_staging_ms\":" << trace.pin_ms
               << ",\"h2d_ms\":" << trace.elapsed(0, 1) << ",\"letterbox_ms\":" << trace.elapsed(1, 2)
               << ",\"trt_ms\":" << trace.elapsed(2, 3) << ",\"d2h_ms\":" << trace.elapsed(3, 4)
               << ",\"gpu_stream_span_ms\":" << trace.elapsed(0, 4)
               << ",\"sync_wait_ms\":" << trace.sync_ms << ",\"infer_host_wall_ms\":" << timing.gpu_ms
               << ",\"cpu_post_ms\":" << timing.post_ms << ",\"call_wall_ms\":" << total_ms
               << ",\"h2d_bytes\":" << trace.h2d_bytes << ",\"d2h_bytes\":" << trace.d2h_bytes
               << ",\"d2h_copies\":" << trace.d2h_copies << "}";
        line(output, record.str());
      }
    }
    if (std::fclose(output) != 0) {output = nullptr; throw std::runtime_error("cannot close output");}
    output = nullptr;
    return 0;
  } catch (const std::exception & error) {
    rescue_yolo_profile::active = nullptr;
    if (output) {std::fclose(output);}
    std::cerr << "yolo transfer probe failed: " << error.what() << '\n';
    return 1;
  }
}
