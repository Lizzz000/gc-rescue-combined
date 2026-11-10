#pragma once
// 安全区合成场景生成器：针孔相机看地面上的托盘（紫色围边、红色内部、黑色中线），
// 叠加光照/色温/曝光、反光、阴影、地面纹理、杂物、物块、运动模糊、低对比和 JPEG 噪声。
// 只给 host 测试使用，不进入生产库；真值是外框四角与中线在处理图中的位置。

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <random>
#include <string>
#include <vector>

namespace field_scene {

enum class Floor { WHITE, GRAY, WOOD, RUBBER, TEXTURE, NEAR_RED };  // NEAR_RED：与内部红色相近的浅红/橙地面

struct Block { double x, y, size, height; cv::Scalar bgr; };       // 场地坐标，米

struct Scene
{
  bool tray = true;               // false：负例，没有托盘
  bool interior = true;           // false：只有紫框没有红色内部（负例）
  bool purple_frame = true;       // false：只有红色板没有紫框（负例）
  bool divider = true;
  // 相机：高度、到托盘近边距离、横向偏移、偏航（度）
  double cam_h = 0.22, cam_d = 0.75, cam_x = 0.0, yaw_deg = 0.0;
  Floor floor = Floor::WHITE;
  double exposure = 1.0;          // 整体增益
  cv::Vec3d tint{1, 1, 1};        // BGR 色温增益
  double gradient = 0.0;          // 横向光照梯度幅度
  int glare = 0;                  // 镜面高光斑数
  bool glare_on_divider = false;  // 高光正压中线
  int shadows = 0;                // 人/车体阴影数
  int clutter = 0;                // 场外杂物数（含红/紫色物体）
  std::vector<Block> blocks;
  std::vector<Block> backdrop;     // 场外固定物体（近色背景），先于托盘绘制
  cv::Scalar divider_bgr{32, 32, 35};  // 中线颜色；真机强光下实测约 (82,95,109)
  double shadow_strength = 0.55;   // 阴影中心的亮度衰减比例
  double awb_compensation = 0.0;   // 相机自动白平衡对 tint 的补偿比例，0=不补偿，1=完全抵消
  int motion_blur = 0;            // 水平运动模糊核长，像素
  double contrast = 1.0;          // 对比度压缩
  int jpeg = 90;
  double noise = 2.0;
  unsigned seed = 1;
};

struct Truth
{
  bool present = false;
  std::array<cv::Point2f, 4> corners;   // TL/TR/BR/BL：远端顶外沿、近端底外沿
  cv::Point2f split_far, split_near;     // 中线中心在内部远/近两端的图像点
};

// 托盘尺寸与 rescue.yaml 一致：外沿 0.66×0.36 m；围边厚 0.015、高 0.04；中线宽 0.022。
constexpr double kW = 0.66, kH = 0.36, kWall = 0.015, kWallH = 0.04, kBand = 0.022;

class Camera
{
public:
  Camera(const Scene & s, cv::Size size)
  : size_(size)
  {
    f_ = 0.62 * size.width;           // 约 78° 水平视场的广角相机
    eye_ = cv::Vec3d(s.cam_x, -s.cam_d, s.cam_h);
    const cv::Vec3d target(0.0, kH * 0.5, 0.0);
    const double yaw = s.yaw_deg * CV_PI / 180.0;
    cv::Vec3d forward = target - eye_;
    forward /= cv::norm(forward);
    // 偏航绕竖直轴旋转视线
    forward = cv::Vec3d(forward[0] * std::cos(yaw) - forward[1] * std::sin(yaw),
      forward[0] * std::sin(yaw) + forward[1] * std::cos(yaw), forward[2]);
    right_ = forward.cross(cv::Vec3d(0, 0, 1)); right_ /= cv::norm(right_);
    down_ = forward.cross(right_); down_ /= cv::norm(down_);
    forward_ = forward;
  }
  cv::Point2f project(const cv::Vec3d & p) const
  {
    const cv::Vec3d d = p - eye_;
    const double z = std::max(1e-4, d.dot(forward_));
    return cv::Point2f(float(size_.width * 0.5 + f_ * d.dot(right_) / z),
      float(size_.height * 0.5 + f_ * d.dot(down_) / z));
  }
  std::vector<cv::Point> poly(const std::vector<cv::Vec3d> & pts) const
  {
    std::vector<cv::Point> out;
    for (const auto & p : pts) {const auto q = project(p); out.emplace_back(cvRound(q.x * 8), cvRound(q.y * 8));}
    return out;
  }
  cv::Vec3d eye() const {return eye_;}

private:
  cv::Size size_;
  double f_;
  cv::Vec3d eye_, right_, down_, forward_;
};

inline void fill(cv::Mat & img, const Camera & cam, const std::vector<cv::Vec3d> & pts, const cv::Scalar & c)
{
  const auto p = cam.poly(pts);
  cv::fillConvexPoly(img, p, c, cv::LINE_AA, 3);
}

// 轴对齐长方体：先画远离相机的面，近面后画。
inline void box(cv::Mat & img, const Camera & cam, double x0, double x1, double y0, double y1,
  double z0, double z1, const cv::Scalar & c)
{
  auto shade = [&](double k) {return cv::Scalar(c[0] * k, c[1] * k, c[2] * k);};
  const cv::Vec3d e = cam.eye();
  if (e[1] > y1) {fill(img, cam, {{x0, y1, z0}, {x1, y1, z0}, {x1, y1, z1}, {x0, y1, z1}}, shade(0.85));}
  if (e[0] < x0) {fill(img, cam, {{x0, y0, z0}, {x0, y1, z0}, {x0, y1, z1}, {x0, y0, z1}}, shade(0.75));}
  if (e[0] > x1) {fill(img, cam, {{x1, y0, z0}, {x1, y1, z0}, {x1, y1, z1}, {x1, y0, z1}}, shade(0.75));}
  if (e[1] < y0) {fill(img, cam, {{x0, y0, z0}, {x1, y0, z0}, {x1, y0, z1}, {x0, y0, z1}}, shade(0.9));}
  fill(img, cam, {{x0, y0, z1}, {x1, y0, z1}, {x1, y1, z1}, {x0, y1, z1}}, shade(1.0));
}

inline cv::Mat floorTexture(Floor floor, cv::Size size)
{
  cv::Mat img(size, CV_8UC3);
  switch (floor) {
    case Floor::WHITE: img.setTo(cv::Scalar(222, 224, 226)); break;
    case Floor::GRAY: img.setTo(cv::Scalar(128, 130, 132)); break;
    case Floor::RUBBER: img.setTo(cv::Scalar(70, 92, 60)); break;   // 绿色地胶
    case Floor::WOOD: {
      for (int y = 0; y < size.height; ++y) {
        const double k = 0.85 + 0.15 * std::sin(y * 0.11 + 3 * std::sin(y * 0.013));
        img.row(y).setTo(cv::Scalar(70 * k, 110 * k, 160 * k));     // 偏红棕的木纹
      }
      break;
    }
    case Floor::NEAR_RED: img.setTo(cv::Scalar(120, 140, 205)); break;  // 浅橙红地面
    case Floor::TEXTURE: {             // 低饱和的斑驳水泥/拼接地垫
      cv::Mat noise(size / 8, CV_8U);
      cv::randu(noise, cv::Scalar(70), cv::Scalar(210));
      cv::resize(noise, noise, size, 0, 0, cv::INTER_NEAREST);
      cv::Mat tinted[] = {noise * 0.95, noise, noise * 1.05};
      cv::merge(tinted, 3, img);
      break;
    }
  }
  return img;
}

// 生成处理分辨率的 BGR 图像与真值。
inline cv::Mat render(const Scene & s, cv::Size size, Truth & truth)
{
  std::mt19937 rng(s.seed);
  cv::theRNG().state = 0x9E3779B97F4A7C15ULL ^ s.seed;  // randu/randn 也按种子复现
  std::uniform_real_distribution<double> u(0, 1);
  const Camera cam(s, size);
  cv::Mat img = floorTexture(s.floor, size);
  const cv::Scalar purple(190, 95, 120), red(70, 72, 160), black = s.divider_bgr;
  for (const auto & b : s.backdrop) {
    box(img, cam, b.x - b.size / 2, b.x + b.size / 2, b.y - b.size / 2, b.y + b.size / 2, 0, b.height, b.bgr);
  }
  // 场外杂物：随机颜色的小盒子，含紫色/红色物体，位置在托盘外
  for (int i = 0; i < s.clutter; ++i) {
    const cv::Scalar colours[] = {purple, red, cv::Scalar(30, 30, 30), cv::Scalar(200, 120, 40),
      cv::Scalar(60, 180, 220)};
    const auto c = colours[i % 5];
    double x = (u(rng) < 0.5 ? -1 : 1) * (kW / 2 + 0.05 + 0.25 * u(rng)), y = -0.1 + 0.7 * u(rng);
    const double w = 0.03 + 0.08 * u(rng);
    box(img, cam, x - w / 2, x + w / 2, y, y + w * (0.5 + u(rng)), 0, 0.02 + 0.06 * u(rng), c);
  }
  truth = Truth();
  if (s.tray) {
    const double x0 = -kW / 2, x1 = kW / 2, y0 = 0, y1 = kH, t = kWall, h = kWallH;
    const cv::Scalar inside = s.interior ? red : cv::Scalar(205, 205, 205);
    fill(img, cam, {{x0 + t, y0 + t, 0}, {x1 - t, y0 + t, 0}, {x1 - t, y1 - t, 0}, {x0 + t, y1 - t, 0}}, inside);
    if (s.divider) {
      fill(img, cam, {{-kBand / 2, y0 + t, 0}, {kBand / 2, y0 + t, 0}, {kBand / 2, y1 - t, 0}, {-kBand / 2, y1 - t, 0}}, black);
    }
    const cv::Scalar wall = s.purple_frame ? purple : inside;
    box(img, cam, x0, x1, y1 - t, y1, 0, h, wall);                   // 远端
    if (s.divider) {                                                  // 中线沿远端内立面上延
      fill(img, cam, {{-kBand / 2, y1 - t, 0}, {kBand / 2, y1 - t, 0}, {kBand / 2, y1 - t, h}, {-kBand / 2, y1 - t, h}}, black);
    }
    box(img, cam, x0, x0 + t, y0, y1, 0, h, wall);                   // 左
    box(img, cam, x1 - t, x1, y0, y1, 0, h, wall);                   // 右
    std::vector<Block> blocks = s.blocks;
    std::sort(blocks.begin(), blocks.end(), [](const Block & a, const Block & b) {return a.y > b.y;});
    for (const auto & b : blocks) {
      box(img, cam, b.x - b.size / 2, b.x + b.size / 2, b.y - b.size / 2, b.y + b.size / 2, 0, b.height, b.bgr);
    }
    box(img, cam, x0, x1, y0, y0 + t, 0, h, wall);                   // 近端
    truth.present = s.purple_frame && s.interior;
    truth.corners = {cam.project({x0, y1, h}), cam.project({x1, y1, h}), cam.project({x1, y0, 0}), cam.project({x0, y0, 0})};
    truth.split_far = cam.project({0, y1 - t, 0});
    truth.split_near = cam.project({0, y0 + t, 0});
  }
  cv::Mat f;
  img.convertTo(f, CV_32FC3, 1.0 / 255);
  // 阴影：大块半透明暗多边形（人或车体），覆盖托盘一部分
  for (int i = 0; i < s.shadows; ++i) {
    cv::Mat m = cv::Mat::zeros(size, CV_32F);
    const double cx = (0.2 + 0.6 * u(rng)) * size.width, cy = (0.3 + 0.5 * u(rng)) * size.height;
    cv::ellipse(m, cv::Point(cvRound(cx), cvRound(cy)), cv::Size(cvRound(size.width * (0.08 + 0.12 * u(rng))),
      cvRound(size.height * (0.15 + 0.2 * u(rng)))), 360 * u(rng), 0, 360, cv::Scalar(1), cv::FILLED);
    cv::GaussianBlur(m, m, cv::Size(0, 0), 12);
    std::vector<cv::Mat> ch; cv::split(f, ch);
    for (auto & c : ch) {c = c.mul(1.0 - s.shadow_strength * m);}
    cv::merge(ch, f);
  }
  if (s.gradient != 0) {
    for (int x = 0; x < size.width; ++x) {
      const float k = float(1.0 + s.gradient * (2.0 * x / size.width - 1.0));
      f.col(x) *= k;
    }
  }
  // 色温与曝光
  std::vector<cv::Mat> ch; cv::split(f, ch);
  for (int c = 0; c < 3; ++c) {ch[c] *= std::pow(s.tint[c], 1.0 - s.awb_compensation) * s.exposure;}
  cv::merge(ch, f);
  // 镜面高光：向白色混合的椭圆斑，可压在中线上
  for (int i = 0; i < s.glare; ++i) {
    cv::Point2f c(float((0.3 + 0.4 * u(rng)) * size.width), float((0.35 + 0.3 * u(rng)) * size.height));
    if (s.glare_on_divider && truth.present) {c = 0.5f * (truth.split_far + truth.split_near);}
    cv::Mat m = cv::Mat::zeros(size, CV_32F);
    cv::ellipse(m, c, cv::Size(cvRound(size.width * (0.03 + 0.05 * u(rng))), cvRound(size.height * (0.02 + 0.03 * u(rng)))),
      0, 0, 360, cv::Scalar(0.75), cv::FILLED);
    cv::GaussianBlur(m, m, cv::Size(0, 0), 6);
    cv::Mat m3; cv::merge(std::vector<cv::Mat>{m, m, m}, m3);
    f = f.mul(cv::Scalar::all(1.0) - m3) + m3 * 0.97;
  }
  if (s.contrast != 1.0) {f = (f - cv::Scalar::all(0.5)) * s.contrast + cv::Scalar::all(0.5);}
  cv::Mat out;
  f.convertTo(out, CV_8UC3, 255);
  if (s.motion_blur > 1) {
    cv::Mat k = cv::Mat::zeros(1, s.motion_blur, CV_32F); k.setTo(1.0 / s.motion_blur);
    cv::filter2D(out, out, -1, k);
  }
  if (s.noise > 0) {
    cv::Mat n(size, CV_16SC3);
    cv::randn(n, 0, s.noise);
    cv::Mat o16; out.convertTo(o16, CV_16SC3); o16 += n; o16.convertTo(out, CV_8UC3);
  }
  std::vector<uchar> buf;
  cv::imencode(".jpg", out, buf, {cv::IMWRITE_JPEG_QUALITY, s.jpeg});
  return cv::imdecode(buf, cv::IMREAD_COLOR);
}

}  // namespace field_scene
