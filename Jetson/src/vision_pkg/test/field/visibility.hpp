// 测试侧的补充真值：内沿几何（地面高度内边线）与中线可见性分级。
// 不修改 field_scene.hpp 的角点真值；内沿由本文件按同一相机模型独立计算。
#pragma once

#include "field_scene.hpp"
#include "scene_params.hpp"

#include <opencv2/imgproc.hpp>

#include <array>

namespace field_test {

enum class Judge { JUDGEABLE, BORDERLINE, UNJUDGEABLE };

inline const char * judgeName(Judge j)
{
  return j == Judge::JUDGEABLE ? "可判" : j == Judge::UNJUDGEABLE ? "不可判" : "边界";
}

// 内沿：内部地面四角（TL/TR/BR/BL，与算法角点同序）与左右半区中心的图像点。
struct InnerTruth
{
  std::array<cv::Point2f, 4> corners;
  cv::Point2f left_center, right_center;
};

inline InnerTruth innerTruth(const field_scene::Scene & s, cv::Size size)
{
  using namespace field_scene;
  const Camera cam(s, size);
  const double x0 = -kW / 2 + kWall, x1 = kW / 2 - kWall, y0 = kWall, y1 = kH - kWall;
  const double ym = 0.5 * (y0 + y1), half_band = kBand / 2;
  InnerTruth t;
  t.corners = {cam.project({x0, y1, 0}), cam.project({x1, y1, 0}), cam.project({x1, y0, 0}), cam.project({x0, y0, 0})};
  t.left_center = cam.project({0.5 * (x0 - half_band), ym, 0});
  t.right_center = cam.project({0.5 * (x1 + half_band), ym, 0});
  return t;
}

struct Visibility
{
  double geometric = 0.0;             // 干净场景下可见采样点 / 全部采样点：围边与画框造成的固有遮挡
  double total = 0.0;                 // 实际场景可见 / 干净场景可见：干扰与物块造成的损失
  double thirds[3] = {0.0, 0.0, 0.0}; // 干净可见范围按米三等分（远/中/近）后各段的实际可见比例
  Judge judge = Judge::UNJUDGEABLE;
};

// 同一场景渲染有/无中线两张图（关掉随机噪声，保留遮挡、光照、模糊、压缩），差分得到看得见的中线像素。
inline cv::Mat dividerVisibleMask(field_scene::Scene s, cv::Size size)
{
  s.noise = 0.0;
  field_scene::Truth unused;
  s.divider = true;
  const cv::Mat with = field_scene::render(s, size, unused);
  s.divider = false;
  const cv::Mat without = field_scene::render(s, size, unused);
  cv::Mat diff;
  cv::absdiff(with, without, diff);
  std::vector<cv::Mat> ch;
  cv::split(diff, ch);
  cv::Mat peak = cv::max(cv::max(ch[0], ch[1]), ch[2]);
  return peak >= kVisDiffMin;
}

// 只保留相机几何与围边：去掉物块和全部光照/成像干扰，中线恢复默认黑色。
inline field_scene::Scene cleanScene(const field_scene::Scene & s)
{
  field_scene::Scene c;
  c.cam_h = s.cam_h; c.cam_d = s.cam_d; c.cam_x = s.cam_x; c.yaw_deg = s.yaw_deg;
  c.floor = s.floor; c.seed = s.seed; c.jpeg = kCleanJpegQuality;
  return c;
}

inline Judge classify(const Visibility & v)
{
  const double ends = std::min(v.thirds[0], v.thirds[2]);
  if (v.geometric < kUnjudgeableGeometricMax) {return Judge::UNJUDGEABLE;}
  if (v.total < kUnjudgeableMaxRatio || ends < kUnjudgeableEndMaxRatio) {return Judge::UNJUDGEABLE;}
  if (v.geometric >= kJudgeableGeometricMin && v.total >= kJudgeableMinRatio && ends >= kJudgeableEndMinRatio) {
    return Judge::JUDGEABLE;
  }
  return Judge::BORDERLINE;
}

// 中线地面足迹按米均匀采样（与算法鸟瞰图同一度量），逐点查干净/实际两张可见掩码。
// 三等分按干净可见的纵向范围（米）划分；画外采样点视为不可见。
inline Visibility dividerVisibility(const field_scene::Scene & s, cv::Size size)
{
  using namespace field_scene;
  Visibility v;
  if (!s.tray || !s.divider || !s.interior) {return v;}
  const cv::Mat clean = dividerVisibleMask(cleanScene(s), size);
  const cv::Mat seen = dividerVisibleMask(s, size);
  const Camera cam(s, size);
  const double y0 = kWall, y1 = kH - kWall, b = kBand / 2;
  struct Sample {double y; bool clean, seen;};
  std::vector<Sample> samples;
  for (double y = y0; y <= y1; y += kVisSampleStepM) {
    for (int i = 0; i < kVisSamplesAcross; ++i) {
      const double x = -b + kBand * (i + 0.5) / kVisSamplesAcross;
      const cv::Point p = cam.project({x, y, 0});
      const bool inside = p.x >= 0 && p.y >= 0 && p.x < size.width && p.y < size.height;
      samples.push_back({y, inside && clean.at<uchar>(p) != 0, inside && seen.at<uchar>(p) != 0});
    }
  }
  double lo = y1, hi = y0;
  int clean_n = 0, seen_n = 0;
  for (const auto & q : samples) {
    if (!q.clean) {continue;}
    ++clean_n; seen_n += q.seen;
    lo = std::min(lo, q.y); hi = std::max(hi, q.y);
  }
  v.geometric = samples.empty() ? 0.0 : double(clean_n) / samples.size();
  if (clean_n == 0) {return v;}
  // 三段编号：0=远端，2=近端（y 越大越远）
  double clean_k[3] = {0, 0, 0}, seen_k[3] = {0, 0, 0};
  const double span = std::max(hi - lo, kVisSampleStepM);
  for (const auto & q : samples) {
    if (!q.clean) {continue;}
    const int k = std::clamp(int((hi - q.y) * 3.0 / span), 0, 2);
    clean_k[k] += 1; seen_k[k] += q.seen;
  }
  for (int k = 0; k < 3; ++k) {v.thirds[k] = clean_k[k] > 0 ? seen_k[k] / clean_k[k] : 0.0;}
  v.total = double(seen_n) / clean_n;
  v.judge = classify(v);
  return v;
}

}  // namespace field_test
