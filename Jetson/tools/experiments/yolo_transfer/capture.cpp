#include "yolo/yolo.hpp"
#include <opencv2/imgcodecs.hpp>
#include <array>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <map>
#include <stdexcept>
#include <string>
#include <vector>

namespace {
class Binary
{
public:
  explicit Binary(const std::filesystem::path & path) : file_(path, std::ios::binary)
  {
    if (!file_) {throw std::runtime_error("cannot create capture file");}
  }
  void bytes(const void * data, std::size_t size)
  {
    if (size) {file_.write(static_cast<const char *>(data), static_cast<std::streamsize>(size));}
    if (!file_) {throw std::runtime_error("capture write failed");}
  }
  void integer(std::int32_t value) {bytes(&value, sizeof(value));}
  void text(const std::string & value)
  {
    integer(static_cast<std::int32_t>(value.size()));
    bytes(value.data(), value.size());
  }
  void image(const cv::Mat & image)
  {
    integer(image.rows); integer(image.cols); integer(image.type());
    for (int row = 0; row < image.rows; ++row) {
      bytes(image.ptr(row), static_cast<std::size_t>(image.cols) * image.elemSize());
    }
  }
  void finish()
  {
    file_.flush();
    if (!file_) {throw std::runtime_error("capture flush failed");}
    file_.close();
    if (!file_) {throw std::runtime_error("capture close failed");}
  }
private:
  std::ofstream file_;
};
std::string required(const std::map<std::string, std::string> & args, const std::string & key)
{
  auto found = args.find(key);
  if (found == args.end()) {throw std::runtime_error("missing --" + key);}
  return found->second;
}
bool samePixels(const cv::Mat & a, const cv::Mat & b)
{
  if (a.size() != b.size() || a.type() != b.type()) {return false;}
  for (int row = 0; row < a.rows; ++row) {
    if (std::memcmp(a.ptr(row), b.ptr(row),
      static_cast<std::size_t>(a.cols) * a.elemSize()) != 0) {return false;}
  }
  return true;
}
}  // namespace

// The same tool is compiled separately against each actual source version.
// No reference decoder is reimplemented: binary captures contain exact public
// FastSegmentor results, including every box/confidence/class and mask byte.
int main(int argc, char ** argv)
{
  try {
    std::map<std::string, std::string> args;
    for (int i = 1; i < argc; i += 2) {
      if (i + 1 >= argc || std::string(argv[i]).rfind("--", 0) != 0) {
        throw std::runtime_error("arguments require --name value pairs");
      }
      const std::string key = std::string(argv[i]).substr(2);
      if ((key != "engine" && key != "labels" && key != "images" && key != "output") ||
        !args.emplace(key, argv[i + 1]).second) {throw std::runtime_error("unknown/duplicate option");}
    }
    const std::filesystem::path output(required(args, "output"));
    if (!std::filesystem::create_directory(output)) {
      throw std::runtime_error("capture output exists or cannot be created");
    }
    std::ifstream paths(required(args, "images"));
    if (!paths) {throw std::runtime_error("cannot read input list");}
    std::vector<std::string> names;
    std::vector<cv::Mat> inputs;
    for (std::string name; std::getline(paths, name); ) {
      if (!name.empty() && name.back() == '\r') {name.pop_back();}
      if (name.empty()) {continue;}
      cv::Mat image = cv::imread(name, cv::IMREAD_COLOR);
      if (image.empty()) {throw std::runtime_error("cannot read input " + name);}
      names.push_back(name);
      inputs.push_back(image);
    }
    if (inputs.size() < 3) {throw std::runtime_error("use at least three real fixed images");}
    FastSegmentor detector(required(args, "engine"), required(args, "labels"));
    if (!detector.isEndToEnd()) {throw std::runtime_error("requires deployed end-to-end model");}
    SegOptions options;
    options.conf_threshold = 0.75f;
    options.iou_threshold = 0.7f;
    options.max_detections = 300;
    options.mask_scale = 2;
    options.mask_label_offset = 1;
    std::ofstream manifest(output / "cases.tsv");
    if (!manifest) {throw std::runtime_error("cannot write manifest");}
    manifest << "case\tinput\tstrided\tmask_budget\tlabel\tinstance\tdetections\n";
    constexpr std::array<std::array<bool, 2>, 7> consumers{{
      {{false, false}}, {{true, false}}, {{false, false}}, {{false, true}},
      {{false, false}}, {{true, true}}, {{false, false}}
    }};
    std::size_t cases = 0, foreground = 0, instance_masks = 0;
    for (std::size_t i = 0; i < inputs.size(); ++i) {
      for (bool strided : {false, true}) {
        cv::Mat storage = strided ?
          cv::Mat(inputs[i].rows + 2, inputs[i].cols + 13, CV_8UC3, cv::Scalar(23, 151, 237)) :
          inputs[i].clone();
        cv::Mat input = strided ?
          storage(cv::Rect(5, 1, inputs[i].cols, inputs[i].rows)) : storage;
        if (strided) {
          inputs[i].copyTo(input);
          if (input.isContinuous()) {throw std::runtime_error("strided fixture is continuous");}
        }
        const cv::Mat initial_storage = storage.clone();
        for (int mask_budget : {32, 0}) {
          options.max_masks = mask_budget;
          for (const auto & flags : consumers) {
            options.want_label_mask = flags[0];
            options.want_instance_masks = flags[1];
            cv::Mat labels;
            const auto results = detector.segmentFast(input, options, labels);
            if (!samePixels(storage, initial_storage)) {throw std::runtime_error("input mutated");}
            const std::string filename = "case-" + std::to_string(cases) + ".bin";
            Binary record(output / filename);
            record.text("FastSegmentor-public-output-v1");
            record.integer(static_cast<std::int32_t>(i));
            record.integer(strided);
            record.integer(options.max_masks);
            record.integer(flags[0]); record.integer(flags[1]);
            record.integer(static_cast<std::int32_t>(results.size()));
            for (const auto & result : results) {
              record.integer(result.bbox.x); record.integer(result.bbox.y);
              record.integer(result.bbox.width); record.integer(result.bbox.height);
              record.bytes(&result.confidence, sizeof(result.confidence));
              record.integer(result.class_id); record.text(result.class_name);
              record.image(result.mask);
              instance_masks += !result.mask.empty();
            }
            record.image(labels);
            record.finish();
            foreground += results.size();
            manifest << filename << '\t' << names[i] << '\t' << strided << '\t'
                     << mask_budget << '\t' << flags[0] << '\t' << flags[1] << '\t'
                     << results.size() << '\n';
            if (!manifest) {throw std::runtime_error("manifest write failed");}
            ++cases;
          }
        }
      }
    }
    manifest.flush();
    if (!manifest) {throw std::runtime_error("manifest flush failed");}
    if (foreground == 0 || instance_masks == 0) {
      throw std::runtime_error("fixtures did not exercise foreground and real instance masks");
    }
    std::ofstream summary(output / "summary.json");
    summary << "{\"complete\":true,\"engine_builds\":0,\"cases\":" << cases
            << ",\"detections\":" << foreground << ",\"instance_masks\":" << instance_masks
            << ",\"confidence\":0.75,\"iou\":0.7,\"max_detections\":300,"
            << "\"mask_scale\":2,\"mask_label_offset\":1}\n";
    summary.flush();
    if (!summary) {throw std::runtime_error("summary write failed");}
    return 0;
  } catch (const std::exception & error) {
    std::cerr << "YOLO output capture failed: " << error.what() << '\n';
    return 1;
  }
}
