// 安全区管线环境鲁棒性回归：每个场景生成一小段序列跑完整 FieldLocator，
// 统计外框提交率、分区有效率、角点与中线误差；负例必须全程不出有效结果。
#include "check.hpp"
#include "field_scene.hpp"
#include "production_config.hpp"
#include "field/field.hpp"

#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <map>
#include <string>
#include <vector>

namespace {

using field_scene::Block;
using field_scene::Floor;
using field_scene::Scene;

struct Outcome
{
  int frames = 0, committed = 0, split = 0;
  double corner_err = 0, split_err = 0, centre_err = 0, ms = 0;
  int corners = 0, measured = 0;
};

constexpr int kFrames = 8;      // 每场景帧数；前几帧用于定位确认
constexpr int kScored = 4;      // 只统计最后几帧

double lineXAt(const cv::Point2f & a, const cv::Point2f & b, double y)
{
  return a.x + (b.x - a.x) * (y - a.y) / (b.y - a.y);
}

cv::Point2f mean4(const cv::Point2f & a, const cv::Point2f & b, const cv::Point2f & c, const cv::Point2f & d)
{
  return (a + b + c + d) * 0.25f;
}

// 左右区域中心（四顶点均值，与 safety_node 的 0x182 中心同口径）相对真值内部地面两半的平均误差，处理图像素。
double regionCentreError(const FieldResult & r, const field_scene::Truth & truth)
{
  const float w = r.bev.cols - 1.f, h = r.bev.rows - 1.f;
  const float xt = float(r.split_top_x), xb = float(r.split_bottom_x);
  std::vector<cv::Point2f> bev{{0, 0}, {xt, 0}, {w, 0}, {0, h}, {xb, h}, {w, h}}, p;
  cv::perspectiveTransform(bev, p, r.homography_inv);
  const auto & in = truth.inner;
  const double left = cv::norm(mean4(p[0], p[1], p[4], p[3]) - mean4(in[0], truth.split_far, truth.split_near, in[3]));
  const double right = cv::norm(mean4(p[1], p[2], p[5], p[4]) - mean4(truth.split_far, in[1], in[2], truth.split_near));
  return 0.5 * (left + right);
}

Outcome runScene(Scene scene, const FieldConfig & config)
{
  FieldLocator locator(config);
  Outcome out;
  const cv::Size size(config.proc_width, config.proc_height);
  for (int i = 0; i < kFrames; ++i) {
    scene.seed = scene.seed * 31 + i;            // 每帧独立噪声
    field_scene::Truth truth;
    const cv::Mat image = field_scene::render(scene, size, truth);
    const auto t0 = std::chrono::steady_clock::now();
    const FieldResult r = locator.update(image);
    const double ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
    if (i < kFrames - kScored) {continue;}
    ++out.frames; out.ms += ms;
    const bool committed = r.ok && r.committed && r.corners.size() == 4;
    if (!committed) {continue;}
    ++out.committed;
    if (truth.present) {
      // corners 是内部地面四角：只考核画内角点；画外角点由区域中心误差考核。
      for (int k = 0; k < 4; ++k) {
        const auto & c = truth.inner[k];
        if (c.x < 0 || c.y < 0 || c.x > size.width - 1 || c.y > size.height - 1) {continue;}
        out.corner_err += cv::norm(r.corners[k] - c);
        ++out.corners;
      }
    }
    if (!r.split_valid) {continue;}
    ++out.split;
    if (truth.present) {
      // 分界线（BEV 上下端点投回处理图）与真值中线在内部中段同一行上的横向距离
      std::vector<cv::Point2f> bev{{float(r.split_top_x), 0.f}, {float(r.split_bottom_x), float(r.bev.rows - 1)}}, img;
      cv::perspectiveTransform(bev, img, r.homography_inv);
      const double y = 0.5 * (truth.split_far.y + truth.split_near.y);
      out.split_err += std::abs(lineXAt(img[0], img[1], y) - lineXAt(truth.split_far, truth.split_near, y));
      out.centre_err += regionCentreError(r, truth);
      ++out.measured;
    }
  }
  return out;
}

// POSITIVE：须完整有效且中线准确；OCCLUDED：中线在可见内部被物块完全盖住，可以不输出，
// 但输出时必须准确；NEGATIVE：任何一帧都不能提交外框；
// OUT_OF_FRAME：有一整条边在画外，几何不完整，任何一帧都不能提交。
enum class Expect {POSITIVE, OCCLUDED, NEGATIVE, OUT_OF_FRAME};

struct Case {std::string group, name; Scene scene; Expect expect;};

std::vector<Case> matrix()
{
  std::vector<Case> cases;
  const cv::Scalar black(28, 28, 30), white(235, 235, 232), blue(170, 80, 30), yellow(40, 200, 220), green(60, 160, 60);
  auto add = [&](const std::string & g, const std::string & n, Scene s, Expect expect = Expect::POSITIVE) {
      for (unsigned seed = 1; seed <= 3; ++seed) {
        Scene v = s; v.seed = seed * 7919 + unsigned(cases.size());
        v.cam_x += 0.03 * (int(seed) - 2); v.cam_d += 0.04 * (int(seed) - 2);
        cases.push_back({g, n, v, expect});
      }
    };
  Scene base;
  add("基线", "白地面", base);
  // 光照强弱与色温
  for (double e : {0.45, 0.7, 1.35}) {Scene s = base; s.exposure = e; add("曝光", "增益" + std::to_string(e).substr(0, 4), s);}
  {Scene s = base; s.tint = {0.7, 0.95, 1.3}; add("色温", "暖光", s);}
  {Scene s = base; s.tint = {1.3, 1.0, 0.8}; add("色温", "冷光", s);}
  {Scene s = base; s.tint = {0.9, 1.12, 0.92}; add("色温", "荧光偏绿", s);}
  {Scene s = base; s.tint = {0.75, 0.95, 1.2}; s.exposure = 0.6; add("色温", "暖光+暗", s);}
  // 反光、阴影、梯度
  {Scene s = base; s.glare = 2; add("反光", "高光斑", s);}
  {Scene s = base; s.glare = 1; s.glare_on_divider = true; add("反光", "高光压中线", s);}
  {Scene s = base; s.shadows = 1; add("阴影", "单块阴影", s);}
  {Scene s = base; s.shadows = 2; s.gradient = 0.35; add("阴影", "阴影+梯度", s);}
  // 地面
  {Scene s = base; s.floor = Floor::GRAY; add("地面", "灰", s);}
  {Scene s = base; s.floor = Floor::WOOD; add("地面", "木纹", s);}
  {Scene s = base; s.floor = Floor::RUBBER; add("地面", "绿地胶", s);}
  {Scene s = base; s.floor = Floor::TEXTURE; add("地面", "强纹理", s);}
  // 杂物
  {Scene s = base; s.clutter = 4; add("杂物", "红紫杂物", s);}
  // 物块
  // 相机高 0.22 m 时 5 cm 立方体压在中线上，可见内部范围内中线整段被挡住。
  {Scene s = base; s.blocks = {{0.0, 0.2, 0.05, 0.05, black}}; add("物块", "黑块压线", s, Expect::OCCLUDED);}
  {Scene s = base; s.blocks = {{0.0, 0.15, 0.05, 0.05, white}}; add("物块", "白块压线", s, Expect::OCCLUDED);}
  {Scene s = base; s.blocks = {{0.045, 0.2, 0.05, 0.05, blue}}; add("物块", "蓝块贴线", s);}
  {Scene s = base; s.blocks = {{-0.15, 0.12, 0.05, 0.05, black}, {0.12, 0.25, 0.05, 0.05, yellow},
     {0.02, 0.08, 0.04, 0.04, green}}; add("物块", "多块", s, Expect::OCCLUDED);}
  // 距离/视角/出画
  {Scene s = base; s.cam_d = 1.1; s.cam_h = 0.3; add("视角", "远", s);}
  {Scene s = base; s.cam_d = 0.3; s.cam_h = 0.18; add("视角", "近", s);}
  {Scene s = base; s.yaw_deg = 12; add("视角", "偏航12°", s);}
  {Scene s = base; s.cam_x = 0.25; s.yaw_deg = 22; add("视角", "整侧出画", s, Expect::OUT_OF_FRAME);}
  // 成像质量
  {Scene s = base; s.motion_blur = 11; add("成像", "运动模糊", s);}
  {Scene s = base; s.contrast = 0.55; add("成像", "低对比", s);}
  {Scene s = base; s.jpeg = 30; s.noise = 6; add("成像", "JPEG30+噪声", s);}
  // 组合
  {Scene s = base; s.tint = {0.75, 0.95, 1.25}; s.glare = 1; s.shadows = 1; s.floor = Floor::WOOD;
   s.blocks = {{0.0, 0.18, 0.05, 0.05, black}}; add("组合", "暖光+反光+阴影+木纹+物块", s);}
  {Scene s = base; s.exposure = 0.6; s.contrast = 0.7; s.jpeg = 40; s.motion_blur = 7; s.clutter = 3;
   s.blocks = {{0.08, 0.22, 0.05, 0.05, white}}; add("组合", "暗+低对比+模糊+杂物+物块", s);}
  // 负例
  {Scene s = base; s.tray = false; s.clutter = 6; add("负例", "无场地+红紫杂物", s, Expect::NEGATIVE);}
  {Scene s = base; s.interior = false; add("负例", "只有紫框", s, Expect::NEGATIVE);}
  {Scene s = base; s.purple_frame = false; add("负例", "只有红板", s, Expect::NEGATIVE);}
  // 机位低于围边视线，内部完全被围边挡住：看不到安全区就不能提交。
  {Scene s = base; s.cam_h = 0.12; add("负例", "低机位内部不可见", s, Expect::NEGATIVE);}
  return cases;
}

struct Totals
{
  Expect expect = Expect::POSITIVE;
  int scenes = 0, frames = 0, committed = 0, split = 0, measured = 0, corners = 0;
  double corner = 0, split_err = 0, centre = 0, ms = 0;
};

std::map<std::string, Totals> evaluate(bool report)
{
  const FieldConfig config = productionConfig();
  std::map<std::string, Totals> by_name;
  std::string last;
  for (const auto & c : matrix()) {
    const Outcome o = runScene(c.scene, config);
    auto & t = by_name[c.group + "/" + c.name];
    t.expect = c.expect;
    ++t.scenes; t.frames += o.frames; t.committed += o.committed; t.split += o.split;
    t.corner += o.corner_err; t.corners += o.corners; t.split_err += o.split_err;
    t.centre += o.centre_err; t.measured += o.measured; t.ms += o.ms;
  }
  if (report) {
    std::printf("%-34s %8s %8s %12s %10s %12s %7s\n", "场景", "外框提交", "分区有效", "画内角点误差", "中线误差", "区域中心误差", "ms/帧");
    for (const auto & [name, t] : by_name) {
      std::printf("%-34s %4d/%-3d %4d/%-3d %12.2f %10.2f %12.2f %7.2f\n", name.c_str(), t.committed, t.frames, t.split, t.frames,
        t.corners ? t.corner / t.corners : 0.0, t.measured ? t.split_err / t.measured : 0.0,
        t.measured ? t.centre / t.measured : 0.0, t.ms / t.frames);
    }
  }
  return by_name;
}

}  // namespace

void testFieldRobustness(bool report)
{
  Group group("field robustness");
  const auto results = evaluate(report);
  for (const auto & [name, t] : results) {
    if (t.expect == Expect::NEGATIVE || t.expect == Expect::OUT_OF_FRAME) {
      // 负例与整侧出画：任何一帧提交外框都是误检或几何不完整
      CHECK(t.committed == 0);
      continue;
    }
    // 中线误差均值不超过 4 像素（约 1 cm）；正例另要求最后几帧的完整有效率不低于 3/4。
    const bool accurate = t.measured == 0 || t.split_err / t.measured <= 4.0;
    const bool complete = t.expect == Expect::OCCLUDED || t.split * 4 >= t.frames * 3;
    const bool ok = accurate && complete;
    if (!ok) {std::fprintf(stderr, "场景未达标: %s split=%d/%d\n", name.c_str(), t.split, t.frames);}
    CHECK(ok);
  }
}
