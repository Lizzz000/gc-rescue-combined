#include "yolo/postprocess.hpp"
#include "node/detector.hpp"  // A DTO consumer must compile without ROS/TRT headers.
#include "reference.hpp"
#include <cmath>
#include <iostream>
#include <limits>
#include <random>

namespace {
int passed = 0, failed = 0;
#define CHECK(test) do { if (test) {++passed;} else {++failed; std::cerr << "line " << __LINE__ << ": " #test "\n";} } while (false)

bool equal(const cv::Mat & a, const cv::Mat & b)
{
  return a.empty() && b.empty() ? true : a.size() == b.size() && a.type() == b.type() &&
    !a.empty() && !b.empty() && cv::norm(a, b, cv::NORM_INF) == 0;
}

struct Frame {
  SegOutputView view;
  std::vector<float> detection, prototype;
  std::vector<std::string> names{"red", "blue", "green"};
  Frame(bool e2e, std::size_t rows, cv::Size image = {97, 59}, cv::Size input = {64, 64}, int prototypes = 3)
  {
    view.end_to_end = e2e; view.rows = rows;
    view.prototype_count = prototypes; view.prototype_size = {16, 16};
    view.features = 4 + (e2e ? 2 : names.size()) + prototypes;
    view.image_size = image; view.input_size = input;
    const auto lb = letterboxFor(image, input);
    view.gain = lb.scale; view.pad_x = float(lb.pad_left); view.pad_y = float(lb.pad_top);
    detection.resize(rows * view.features);
    prototype.resize(prototypes * view.prototype_size.area());
    for (std::size_t i = 0; i < prototype.size(); ++i) {
      prototype[i] = float(std::sin(i * .137) * 1.9 + std::cos(i * .093) * .4);
    }
  }
  void row(std::size_t r, float x, float y, float w, float h, float score, int cls)
  {
    // Input coordinates are deliberately letterbox coordinates, before clipping.
    if (view.end_to_end) {
      float * p = detection.data() + r * view.features;
      p[0] = x; p[1] = y; p[2] = x + w; p[3] = y + h; p[4] = score; p[5] = float(cls);
      for (int k = 0; k < view.prototype_count; ++k) {p[6 + k] = float(std::sin(r + k * 1.3));}
    } else {
      detection[r] = x + w / 2; detection[view.rows + r] = y + h / 2;
      detection[2 * view.rows + r] = w; detection[3 * view.rows + r] = h;
      for (std::size_t k = 0; k < names.size(); ++k) {
        detection[(4 + k) * view.rows + r] = int(k) == cls ? score : 0;
      }
      for (int k = 0; k < view.prototype_count; ++k) {
        detection[(4 + names.size() + k) * view.rows + r] = float(std::sin(r + k * 1.3));
      }
    }
  }
  SegOutputView input(bool masks = true)
  {
    view.detections = detection.data();
    view.prototypes = masks ? prototype.data() : nullptr;
    return view;
  }
};

std::vector<SegDetection> compare(SegPostprocessor & processor, Frame & frame, const SegOptions & options)
{
  cv::Mat actual_label, expected_label;
  const auto input = frame.input(options.needsMasks());
  auto actual = processor.run(input, options, frame.names, actual_label);
  const auto expected = reference::run(input, options, frame.names, expected_label);
  CHECK(actual.size() == expected.size());
  CHECK(equal(actual_label, expected_label));
  for (std::size_t i = 0; i < std::min(actual.size(), expected.size()); ++i) {
    const auto & a = actual[i]; const auto & b = expected[i];
    CHECK(a.bbox.x == b.bbox.x && a.bbox.y == b.bbox.y &&
      a.bbox.width == b.bbox.width && a.bbox.height == b.bbox.height);
    CHECK(a.class_id == b.class_id && a.confidence == b.confidence && a.class_name == b.class_name);
    CHECK(equal(a.mask, b.mask));
  }
  return actual;
}

void geometryAndNms()
{
  SegPostprocessor processor;
  for (bool e2e : {false, true}) {
    Frame f(e2e, 8, {100, 100}, {100, 100}, e2e ? 3 : 32);
    f.row(0, 10, 10, 20, 20, .8f, 0);
    f.row(1, 11, 10, 20, 20, .8f, 0); // same-score suppression keeps row 0
    f.row(2, 10, 10, 20, 20, .8f, 1); // other class survives
    f.row(3, 50, 10, 20, 20, .8f, 0); // same score keeps source order
    f.row(4, -3, 85, 18, 30, .9f, 2); // partial out-of-frame, output clipped
    f.row(5, -20, -20, 4, 4, 1.f, 0); // entirely outside, no edge ghost
    f.row(6, 20, 20, 0, 5, 1.f, 0);
    f.row(7, 40, 40, 10, 10, .39f, 0);
    SegOptions opt; opt.want_label_mask = false; opt.want_instance_masks = false;
    auto out = compare(processor, f, opt);
    CHECK(out.size() == 4);
    if (out.size() == 4) {
      CHECK(out[0].class_id == 2 && out[1].bbox.x == 10 && out[2].class_id == 1 && out[3].bbox.x == 50);
      CHECK(out[0].bbox.x == 0 && out[0].bbox.y == 85 && out[0].bbox.width == 15 && out[0].bbox.height == 15);
    }
    for (int budget : {-1, 0, 1, 2, 3, 100}) {opt.max_detections = budget; compare(processor, f, opt);}
    for (float threshold : {0.f, .33333334f, .45f, 1.f}) {opt.max_detections = 100; opt.iou_threshold = threshold; compare(processor, f, opt);}
    if (e2e) {
      f.detection[5] = .5f; // class indices must be integers
      f.detection[f.view.features + 4] = std::numeric_limits<float>::quiet_NaN();
      f.detection[2 * f.view.features + 4] = 1.01f;
      f.detection[3 * f.view.features] = std::numeric_limits<float>::infinity();
    } else {
      f.detection[4 * f.view.rows] = std::numeric_limits<float>::quiet_NaN();
      f.detection[5 * f.view.rows + 1] = 1.01f;
      f.detection[2] = std::numeric_limits<float>::infinity();
      // Class ties follow the original first-class-wins rule.
      f.detection[4 * f.view.rows + 3] = f.detection[5 * f.view.rows + 3] = .8f;
    }
    compare(processor, f, opt);
  }
}

void masksAndOwnership()
{
  SegPostprocessor processor;
  for (bool e2e : {false, true}) {
    for (const cv::Size image : {cv::Size(97, 59), cv::Size(59, 97), cv::Size(101, 101), cv::Size(1, 1)}) {
      Frame f(e2e, 4, image, {64, 64}, e2e ? 3 : 32);
      f.row(0, 0, 0, 60, 60, .9f, 0);
      f.row(1, 10, 11, 30, 22, .8f, 1);
      f.row(2, 45, 30, 24, 40, .7f, 2);
      f.row(3, 20, 25, 2, 2, .6f, 2);
      for (int scale : {1, 2, 3, 8}) {
        for (int budget : {0, 1, 2, 10}) {
          for (int mode = 0; mode != 4; ++mode) {
            SegOptions opt; opt.iou_threshold = 1; opt.mask_scale = scale; opt.max_masks = budget;
            opt.want_label_mask = mode & 1; opt.want_instance_masks = mode & 2;
            opt.mask_label_offset = budget == 1 ? -20 : (budget == 2 ? 1000 : 1);
            compare(processor, f, opt);
          }
        }
      }
      SegOptions opt; opt.want_instance_masks = true;
      cv::Mat labels;
      auto old = processor.run(f.input(), opt, f.names, labels);
      const cv::Mat saved_labels = labels.clone();
      std::vector<cv::Mat> saved;
      for (const auto & detection : old) {saved.push_back(detection.mask.clone());}
      std::fill(f.prototype.begin(), f.prototype.end(), 0.f);
      compare(processor, f, opt); // overwrite all scratch, including 1x1 branches
      CHECK(equal(labels, saved_labels));
      for (std::size_t i = 0; i < old.size(); ++i) {CHECK(equal(old[i].mask, saved[i]));}
      if (e2e) {f.detection[6] = std::numeric_limits<float>::quiet_NaN();}
      else {f.detection[(4 + f.names.size()) * f.view.rows] = std::numeric_limits<float>::quiet_NaN();}
      compare(processor, f, opt); // unusable coefficients preserve detection, skip mask only
      f.view.gain = 0;
      compare(processor, f, opt);
    }
  }
}

void emptyAndUnknownClass()
{
  SegPostprocessor processor;
  SegOptions opt;
  opt.want_instance_masks = true;
  for (bool e2e : {false, true}) {
    Frame empty(e2e, 0);
    CHECK(compare(processor, empty, opt).empty());
  }
  Frame unknown(true, 1, {9, 7}, {16, 16}, 1);
  unknown.row(0, 0, 0, 16, 16, 1, 16777216);
  unknown.detection[6] = 1;
  std::fill(unknown.prototype.begin(), unknown.prototype.end(), 1.f);
  auto detections = compare(processor, unknown, opt);
  CHECK(detections.size() == 1 && detections[0].class_id == 16777216 && detections[0].class_name.empty());
  cv::Mat labels;
  processor.run(unknown.input(), opt, unknown.names, labels);
  double low, high;
  cv::minMaxLoc(labels, &low, &high);
  CHECK(high == 255);  // saturation, not a wrap through mono8 background
}

void randomizedEquivalence()
{
  SegPostprocessor processor;
  std::mt19937 rng(0x20261115);
  for (bool e2e : {false, true}) {
    for (int iteration = 0; iteration < 120; ++iteration) {
      const int count = iteration % 2 ? 300 : 64;
      Frame f(e2e, count, {71 + int(rng() % 70), 51 + int(rng() % 60)}, {64, 64}, e2e ? 5 : 32);
      for (int i = 0; i < count; ++i) {
        // A small discrete confidence alphabet makes most rows tie; dense boxes
        // exercise suppression, rather than testing only a sort in isolation.
        f.row(i, float(int(rng() % 80) - 8), float(int(rng() % 80) - 8),
          float(1 + rng() % 45), float(1 + rng() % 45), float(rng() % 6) / 5, int(rng() % 3));
      }
      SegOptions opt;
      opt.max_detections = int(rng() % 40); opt.max_masks = int(rng() % 8);
      opt.mask_scale = int(1 + rng() % 4); opt.want_instance_masks = iteration & 1;
      opt.want_label_mask = iteration % 3; opt.iou_threshold = float(rng() % 6) / 5;
      compare(processor, f, opt);
    }
  }
}
}  // namespace

int main()
{
  cv::setNumThreads(1);
  geometryAndNms();
  masksAndOwnership();
  emptyAndUnknownClass();
  randomizedEquivalence();
  std::cout << "postprocess: " << passed << " passed, " << failed << " failed\n";
  return failed ? 1 : 0;
}
