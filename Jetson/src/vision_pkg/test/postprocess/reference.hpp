#pragma once
// Independent reference for the pre-extraction contract: materialize all logits,
// stable-sort decoded rows, then class-aware greedy NMS. Deliberately no production
// geometry or mask helpers; shared types describe inputs/outputs only.
#include "yolo/postprocess.hpp"
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <opencv2/imgproc.hpp>

namespace reference {
struct Item {
  double left, top, width, height;
  float score;
  int cls;
  BBox2i box;
  std::vector<float> coefficients;
};

inline std::vector<SegDetection> run(
  const SegOutputView & in, const SegOptions & opt,
  const std::vector<std::string> & names, cv::Mat & labels)
{
  labels.release();
  if (!std::isfinite(in.gain) || in.gain <= 0) {return {};}
  const int scale = std::max(1, opt.mask_scale);
  const cv::Size small(std::max(1, (in.image_size.width + scale - 1) / scale),
    std::max(1, (in.image_size.height + scale - 1) / scale));
  if (opt.want_label_mask) {labels = cv::Mat::zeros(small, CV_8U);}
  std::vector<Item> items;
  for (std::size_t r = 0; r < in.rows; ++r) {
    Item item{};
    double x0, y0, x1, y1;
    std::size_t first_coefficient;
    if (in.end_to_end) {
      const float * row = in.detections + r * in.features;
      if (!std::isfinite(row[4]) || row[4] < opt.conf_threshold || row[4] < 0 || row[4] > 1 ||
        !std::isfinite(row[5]) || row[5] < 0 || double(row[5]) > std::numeric_limits<int>::max() ||
        std::floor(row[5]) != row[5]) {continue;}
      item.score = row[4]; item.cls = int(row[5]);
      x0 = row[0]; y0 = row[1]; x1 = row[2]; y1 = row[3];
      first_coefficient = r * in.features + 6;
    } else {
      const int classes = int(in.features) - 4 - in.prototype_count;
      bool valid = classes > 0;
      for (int cls = 0; cls < classes; ++cls) {
        const float score = in.detections[(4 + cls) * in.rows + r];
        if (!std::isfinite(score) || score < 0 || score > 1) {valid = false; break;}
        if (cls == 0 || score > item.score) {item.score = score; item.cls = cls;}
      }
      if (!valid || item.score < opt.conf_threshold) {continue;}
      const double w = in.detections[2 * in.rows + r], h = in.detections[3 * in.rows + r];
      x0 = in.detections[r] - w / 2; y0 = in.detections[in.rows + r] - h / 2;
      x1 = x0 + w; y1 = y0 + h;
      first_coefficient = (4 + classes) * in.rows + r;
    }
    if (!std::isfinite(x0) || !std::isfinite(y0) || !std::isfinite(x1) || !std::isfinite(y1) ||
      x1 <= x0 || y1 <= y0) {continue;}
    item.left = x0; item.top = y0; item.width = x1 - x0; item.height = y1 - y0;
    x0 = (x0 - in.pad_x) / in.gain; y0 = (y0 - in.pad_y) / in.gain;
    x1 = (x1 - in.pad_x) / in.gain; y1 = (y1 - in.pad_y) / in.gain;
    if (!std::isfinite(x0) || !std::isfinite(y0) || !std::isfinite(x1) || !std::isfinite(y1) ||
      x1 <= 0 || y1 <= 0 || x0 >= in.image_size.width || y0 >= in.image_size.height) {continue;}
    const int left = std::min(in.image_size.width - 1,
      int(std::lround(std::clamp(x0, 0.0, double(in.image_size.width)))));
    const int top = std::min(in.image_size.height - 1,
      int(std::lround(std::clamp(y0, 0.0, double(in.image_size.height)))));
    const int right = int(std::lround(std::clamp(x1, 0.0, double(in.image_size.width))));
    const int bottom = int(std::lround(std::clamp(y1, 0.0, double(in.image_size.height))));
    if (right <= left || bottom <= top) {continue;}
    item.box = {left, top, right - left, bottom - top};
    for (int p = 0; p < in.prototype_count; ++p) {
      item.coefficients.push_back(in.detections[first_coefficient + p * (in.end_to_end ? 1 : in.rows)]);
    }
    items.push_back(std::move(item));
  }
  std::stable_sort(items.begin(), items.end(), [](const Item & a, const Item & b) {return a.score > b.score;});
  std::vector<Item> kept;
  for (const auto & item : items) {
    bool suppress = false;
    for (const auto & other : kept) {
      if (item.cls != other.cls) {continue;}
      const double w = std::min(item.left + item.width, other.left + other.width) - std::max(item.left, other.left);
      const double h = std::min(item.top + item.height, other.top + other.height) - std::max(item.top, other.top);
      if (w > 0 && h > 0 && w * h / (item.width * item.height + other.width * other.height - w * h) > opt.iou_threshold) {
        suppress = true; break;
      }
    }
    if (!suppress) {kept.push_back(item);}
    if (kept.size() >= std::size_t(std::max(1, opt.max_detections))) {break;}
  }
  std::vector<SegDetection> results;
  for (const auto & item : kept) {
    results.push_back({item.box, item.score, item.cls,
      std::size_t(item.cls) < names.size() ? names[item.cls] : "", {}});
  }
  if (opt.max_masks <= 0 || (!opt.want_label_mask && !opt.want_instance_masks) || kept.empty()) {return results;}
  const float gain = std::min(float(in.input_size.height) / in.image_size.height,
    float(in.input_size.width) / in.image_size.width);
  const float px = (in.input_size.width - int(std::round(in.image_size.width * gain))) / 2.f;
  const float py = (in.input_size.height - int(std::round(in.image_size.height * gain))) / 2.f;
  const int pad_left = int(std::round(px - .1f)), pad_top = int(std::round(py - .1f));
  const cv::Mat prototypes(in.prototype_count, in.prototype_size.area(), CV_32F, const_cast<float *>(in.prototypes));
  for (int i = std::min<int>(opt.max_masks, int(kept.size())) - 1; i >= 0; --i) {
    const auto & item = kept[i];
    if (std::any_of(item.coefficients.begin(), item.coefficients.end(), [](float x) {return !std::isfinite(x);})) {continue;}
    cv::Mat weights(1, in.prototype_count, CV_32F, const_cast<float *>(item.coefficients.data()));
    cv::Mat logits = weights * prototypes;
    logits = logits.reshape(1, in.prototype_size.height);
    cv::Mat probability;
    cv::exp(-logits, probability);
    probability += 1.f;
    cv::divide(1.0, probability, probability);
    cv::Mat binary;
    cv::Rect destination;
    if (in.end_to_end) {
      const auto mx = [&](double x) {return (x * gain + pad_left) * in.prototype_size.width / in.input_size.width;};
      const auto my = [&](double y) {return (y * gain + pad_top) * in.prototype_size.height / in.input_size.height;};
      const int x0 = int(std::floor(std::clamp(mx(item.box.x), 0.0, double(in.prototype_size.width))));
      const int y0 = int(std::floor(std::clamp(my(item.box.y), 0.0, double(in.prototype_size.height))));
      const int x1 = int(std::ceil(std::clamp(mx(item.box.x + item.box.width), 0.0, double(in.prototype_size.width))));
      const int y1 = int(std::ceil(std::clamp(my(item.box.y + item.box.height), 0.0, double(in.prototype_size.height))));
      if (x1 <= x0 || y1 <= y0) {continue;}
      destination = {item.box.x / scale, item.box.y / scale, std::max(1, item.box.width / scale), std::max(1, item.box.height / scale)};
      cv::Mat resized;
      cv::resize(probability(cv::Rect(x0, y0, x1 - x0, y1 - y0)), resized, destination.size(), 0, 0, cv::INTER_LINEAR);
      binary = resized > .5f;
    } else {
      const float sx = float(in.prototype_size.width) / in.input_size.width;
      const float sy = float(in.prototype_size.height) / in.input_size.height;
      const int x0 = std::clamp(int(std::round((px - .1f) * sx)), 0, in.prototype_size.width - 1);
      const int y0 = std::clamp(int(std::round((py - .1f) * sy)), 0, in.prototype_size.height - 1);
      const int x1 = std::clamp(int(std::round((in.input_size.width - px + .1f) * sx)), x0, in.prototype_size.width);
      const int y1 = std::clamp(int(std::round((in.input_size.height - py + .1f) * sy)), y0, in.prototype_size.height);
      cv::Mat resized;
      cv::resize(probability(cv::Rect(x0, y0, x1 - x0, y1 - y0)), resized, in.image_size, 0, 0, cv::INTER_LINEAR);
      cv::Mat full = resized > .5f;
      cv::Mat clipped = cv::Mat::zeros(in.image_size, CV_8U);
      const cv::Rect box(item.box.x, item.box.y, item.box.width, item.box.height);
      full(box).copyTo(clipped(box));
      cv::resize(clipped, binary, small, 0, 0, cv::INTER_NEAREST);
      destination = cv::Rect(0, 0, small.width, small.height);
    }
    const cv::Rect clip = destination & cv::Rect(0, 0, small.width, small.height);
    const cv::Mat patch = binary(cv::Rect(clip.x - destination.x, clip.y - destination.y, clip.width, clip.height));
    if (opt.want_label_mask) {labels(clip).setTo(std::clamp<std::int64_t>(std::int64_t(item.cls) + opt.mask_label_offset, 1, 255), patch);}
    if (opt.want_instance_masks) {
      results[i].mask = cv::Mat::zeros(small, CV_8U);
      patch.copyTo(results[i].mask(clip));
    }
  }
  return results;
}
}  // namespace reference
