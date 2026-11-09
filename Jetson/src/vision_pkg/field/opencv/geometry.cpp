#include "field/opencv/algorithm.hpp"
#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>
#if defined(__aarch64__) && defined(__ARM_NEON)
#include <arm_neon.h>
#endif

// 缩放、直线拟合和四边形。评分在 score.cpp，形态学在 morphology.cpp。

namespace field_detail {

// ---- 图像缩放与几何拟合 ----
// 严格3:2面积缩小把每个3×3源块变成2×2目标块；四组权重均为整数/9。
// 内核只读输入，返回新分配的独占输出，不复用调用方缓冲，避免ROI或外部数据别名被覆盖。
namespace {
#if defined(__aarch64__) && defined(__ARM_NEON)
inline uint8x8_t divide9(uint16x8_t sum) {
  // 加权和范围0..2295；分母9没有半整数，整数舍入与面积采样一致。乘法倒数在此范围精确。
  sum = vaddq_u16(sum, vdupq_n_u16(4));
  const auto lo = vmull_n_u16(vget_low_u16(sum), 7282);
  const auto hi = vmull_n_u16(vget_high_u16(sum), 7282);
  return vmovn_u16(vcombine_u16(vshrn_n_u32(lo, 16), vshrn_n_u32(hi, 16)));
}
inline uint8x16_t weighted(uint8x16_t a, uint8x16_t b, uint8x16_t c, uint8x16_t d,
                          int wa, int wb, int wc, int wd) {
  auto low = vmulq_n_u16(vmovl_u8(vget_low_u8(a)), wa);
  low = vmlaq_n_u16(low, vmovl_u8(vget_low_u8(b)), wb);
  low = vmlaq_n_u16(low, vmovl_u8(vget_low_u8(c)), wc);
  low = vmlaq_n_u16(low, vmovl_u8(vget_low_u8(d)), wd);
  auto high = vmulq_n_u16(vmovl_u8(vget_high_u8(a)), wa);
  high = vmlaq_n_u16(high, vmovl_u8(vget_high_u8(b)), wb);
  high = vmlaq_n_u16(high, vmovl_u8(vget_high_u8(c)), wc);
  high = vmlaq_n_u16(high, vmovl_u8(vget_high_u8(d)), wd);
  return vcombine_u8(divide9(low), divide9(high));
}
struct Phases { uint8x16_t v[3][3]; };
inline Phases phases(const uchar * row) {
  const auto a = vld3q_u8(row), b = vld3q_u8(row + 48), c = vld3q_u8(row + 96);
  static const uint8_t indices[16] = {0,3,6,9,12,15,18,21,24,27,30,33,36,39,42,45};
  const auto first = vld1q_u8(indices);
  Phases result;
  for (int channel = 0; channel < 3; ++channel) {
    uint8x16x3_t table{{a.val[channel], b.val[channel], c.val[channel]}};
    for (int phase = 0; phase < 3; ++phase) {
      result.v[channel][phase] = vqtbl3q_u8(table, vaddq_u8(first, vdupq_n_u8(phase)));
    }
  }
  return result;
}
#endif
}  // namespace

cv::Mat resizeForField(const cv::Mat & source, cv::Size target)
{
  cv::Mat output;
  // 用64位比例判定避免尺寸乘法溢出；其他类型/比例和异常参数沿用原OpenCV处理及异常。
  if (source.empty() || source.dims != 2 || source.type() != CV_8UC3 || target.width <= 0 || target.height <= 0 ||
    std::int64_t(source.cols) * 2 != std::int64_t(target.width) * 3 ||
    std::int64_t(source.rows) * 2 != std::int64_t(target.height) * 3 ||
    source.cols % 3 != 0 || source.rows % 3 != 0) {
    cv::resize(source, output, target, 0, 0, cv::INTER_AREA);
    return output;
  }
  output.create(target, CV_8UC3);
  cv::parallel_for_(cv::Range(0, source.rows / 3), [&](const cv::Range & range) {
    for (int block_y = range.start; block_y < range.end; ++block_y) {
      const auto * r0 = source.ptr<uchar>(block_y * 3);
      const auto * r1 = source.ptr<uchar>(block_y * 3 + 1);
      const auto * r2 = source.ptr<uchar>(block_y * 3 + 2);
      auto * top = output.ptr<uchar>(block_y * 2);
      auto * bottom = output.ptr<uchar>(block_y * 2 + 1);
      int block_x = 0;
#if defined(__aarch64__) && defined(__ARM_NEON)
      for (; block_x + 16 <= source.cols / 3; block_x += 16) {
        const auto a = phases(r0 + std::size_t(block_x) * 9), b = phases(r1 + std::size_t(block_x) * 9), c = phases(r2 + std::size_t(block_x) * 9);
        uint8x16x3_t top0{}, top1{}, bottom0{}, bottom1{};
        for (int channel = 0; channel < 3; ++channel) {
          const auto tl = weighted(a.v[channel][0], a.v[channel][1], b.v[channel][0], b.v[channel][1], 4,2,2,1);
          const auto tr = weighted(a.v[channel][1], a.v[channel][2], b.v[channel][1], b.v[channel][2], 2,4,1,2);
          const auto bl = weighted(b.v[channel][0], b.v[channel][1], c.v[channel][0], c.v[channel][1], 2,1,4,2);
          const auto br = weighted(b.v[channel][1], b.v[channel][2], c.v[channel][1], c.v[channel][2], 1,2,2,4);
          const auto t = vzipq_u8(tl, tr), d = vzipq_u8(bl, br);
          top0.val[channel] = t.val[0]; top1.val[channel] = t.val[1];
          bottom0.val[channel] = d.val[0]; bottom1.val[channel] = d.val[1];
        }
        vst3q_u8(top + std::size_t(block_x) * 6, top0); vst3q_u8(top + std::size_t(block_x) * 6 + 48, top1);
        vst3q_u8(bottom + std::size_t(block_x) * 6, bottom0); vst3q_u8(bottom + std::size_t(block_x) * 6 + 48, bottom1);
      }
#endif
      for (; block_x < source.cols / 3; ++block_x) {
        for (int channel = 0; channel < 3; ++channel) {
          const std::size_t x = std::size_t(block_x) * 9 + channel, out = std::size_t(block_x) * 6 + channel;
          top[out] = (4*r0[x]+2*r0[x+3]+2*r1[x]+r1[x+3]+4)/9;
          top[out+3] = (2*r0[x+3]+4*r0[x+6]+r1[x+3]+2*r1[x+6]+4)/9;
          bottom[out] = (2*r1[x]+r1[x+3]+4*r2[x]+2*r2[x+3]+4)/9;
          bottom[out+3] = (r1[x+3]+2*r1[x+6]+2*r2[x+3]+4*r2[x+6]+4)/9;
        }
      }
    }
  }, std::max(1.0, double(output.total()) / 65536.0));
  return output;
}

struct Line
{
  double a = 1, b = 0, c = 0;     // ax + by + c = 0, a^2+b^2 = 1
};

Line normalizeLine(double a, double b, double c)
{
  Line l;
  const double n = std::sqrt(a * a + b * b);
  if (n < 1e-9) {return Line{1, 0, 0};}
  l.a = a / n; l.b = b / n; l.c = c / n;
  return l;
}

Line lineThrough(const cv::Point2f & p, const cv::Point2f & q)
{
  const double dx = q.x - p.x, dy = q.y - p.y;
  return normalizeLine(-dy, dx, dy * p.x - dx * p.y);
}

double pointLineDistance(const Line & l, const cv::Point2f & p)
{
  // 法线已归一化，所以残差可直接与 fit_outlier_px 比较，单位是当前处理图像素。
  return std::abs(l.a * p.x + l.b * p.y + l.c);
}

// 总最小二乘／主成分直线拟合：用较小特征值方向作法线，逐轮剔除残差离群点。
bool fitLineRobust(
  const std::vector<cv::Point2f> & pts, const FieldConfig & cfg, Line & out,
  int & inliers)
{
  inliers = 0;
  if (static_cast<int>(pts.size()) < cfg.min_side_points) {return false;}
  std::vector<cv::Point2f> P = pts;
  Line line;
  for (int it = 0; it < std::max(1, cfg.fit_iterations); ++it) {
    cv::Scalar mean = cv::mean(P);
    cv::Mat cov = cv::Mat::zeros(2, 2, CV_64F);
    for (const auto & p : P) {
      const double dx = p.x - mean[0], dy = p.y - mean[1];
      cov.at<double>(0, 0) += dx * dx;
      cov.at<double>(0, 1) += dx * dy;
      cov.at<double>(1, 0) += dx * dy;
      cov.at<double>(1, 1) += dy * dy;
    }
    cov /= std::max<size_t>(P.size() - 1, 1);
    cv::Mat evals, evecs;
    cv::eigen(cov, evals, evecs);                        // 特征值降序
    const double nx = evecs.at<double>(1, 0), ny = evecs.at<double>(1, 1);      // 最小特征值 -> 法线
    line = normalizeLine(nx, ny, -(nx * mean[0] + ny * mean[1]));
    std::vector<double> res;
    res.reserve(P.size());
    for (const auto & p : P) {
      res.push_back(pointLineDistance(line, p));
    }
    const double med = upperMedian(res);
    // 用残差中位数自适应裁掉离群点；fit_outlier_px 是下限，保留胶带厚度造成的小偏差。
    const double thr = std::max(cfg.fit_outlier_px, 2.5 * med);
    std::vector<cv::Point2f> keep;
    for (size_t i = 0; i < P.size(); ++i) {
      if (res[i] <= thr) {keep.push_back(P[i]);}}
    if (static_cast<int>(keep.size()) < cfg.min_side_points || keep.size() == P.size()) {break;}
    P = keep;
  }
  out = line;
  // 最后回到原始点集统计内点，避免迭代删点后把稀疏边误记成拥有足够实测证据。
  for (const auto & p : pts) {
    if (pointLineDistance(line, p) <= std::max(cfg.fit_outlier_px, 3.0)) {++inliers;}}
  return true;
}

cv::Point2f intersectLines(const Line & l1, const Line & l2, bool & ok)
{
  const double det = l1.a * l2.b - l2.a * l1.b;
  if (std::abs(det) < 1e-9) {ok = false; return {0, 0};}
  ok = true;
  return cv::Point2f(static_cast<float>((l1.b * l2.c - l2.b * l1.c) / det),
                       static_cast<float>((l2.a * l1.c - l1.a * l2.c) / det));
}

double polyArea(const std::vector<cv::Point2f> & q)
{
  double s = 0;
  for (size_t i = 0; i < q.size(); ++i) {
    const cv::Point2f & p = q[i];
    const cv::Point2f & r = q[(i + 1) % q.size()];
    s += p.x * r.y - r.x * p.y;
  }
  return std::abs(s) * 0.5;
}

double dist(const cv::Point2f & p, const cv::Point2f & q) {return cv::norm(p - q);}

// 排成 TL,TR,BR,BL (图像坐标, y 向下)
std::vector<cv::Point2f> orderCorners(std::vector<cv::Point2f> P)
{
  cv::Point2f c(0, 0);
  for (const auto & p : P) {
    c += p;
  }
  c *= 1.0f / static_cast<float>(P.size());
  std::sort(P.begin(), P.end(), [&](const cv::Point2f & u, const cv::Point2f & v) {
      return std::atan2(u.y - c.y, u.x - c.x) < std::atan2(v.y - c.y, v.x - c.x);
    });
  double s = 0;
  for (size_t i = 0; i < P.size(); ++i) {
    const cv::Point2f & p = P[i];
    const cv::Point2f & r = P[(i + 1) % P.size()];
    s += p.x * r.y - r.x * p.y;
  }
  if (s < 0) {std::reverse(P.begin(), P.end());}
  // 先选较水平的一对对边，再选上边；用最小 x+y 在透视梯形中可能选错角点，使 BEV 旋转。
  const double horizontal02 = std::abs(P[1].x - P[0].x) + std::abs(P[3].x - P[2].x);
  const double horizontal13 = std::abs(P[2].x - P[1].x) + std::abs(P[0].x - P[3].x);
  size_t start = horizontal02 >= horizontal13 ? 0 : 1;
  const size_t opposite = (start + 2) % 4;
  if (P[start].y + P[(start + 1) % 4].y > P[opposite].y + P[(opposite + 1) % 4].y) {
    start = opposite;
  }
  std::rotate(P.begin(), P.begin() + start, P.end());
  return P;
}

bool isConvexQuad(const std::vector<cv::Point2f> & q)
{
  bool pos = true, neg = true;
  for (size_t i = 0; i < q.size(); ++i) {
    const cv::Point2f & p = q[i];
    const cv::Point2f & r = q[(i + 1) % q.size()];
    const cv::Point2f & n = q[(i + 2) % q.size()];
    const double cross = (r.x - p.x) * (n.y - r.y) - (r.y - p.y) * (n.x - r.x);
    if (cross <= 1e-6) {pos = false;}
    if (cross >= -1e-6) {neg = false;}
  }
  return pos || neg;
}

std::pair<double, double> quadSize(const std::vector<cv::Point2f> & q)
{
  const double w = 0.5 * (dist(q[0], q[1]) + dist(q[3], q[2]));
  const double h = 0.5 * (dist(q[1], q[2]) + dist(q[0], q[3]));
  return {std::max(w, h), std::min(w, h)};
}


// 透视先验由凸包简化得到，不强行约束直角，以保留斜视场地的形状。
std::vector<cv::Point2f> priorQuad(const std::vector<cv::Point2f> & pts)
{
  std::vector<cv::Point> pts_i;
  pts_i.reserve(pts.size());
  for (const auto & p : pts) {
    pts_i.emplace_back(cvRound(p.x), cvRound(p.y));
  }
  std::vector<cv::Point> hull;
  cv::convexHull(pts_i, hull);
  const double peri = cv::arcLength(hull, true);
  std::vector<cv::Point> approx;
  cv::approxPolyDP(hull, approx, 0.02 * peri, true);
  std::vector<cv::Point2f> quad;
  if (approx.size() == 4) {
    for (const auto & p : approx) {
      quad.emplace_back(static_cast<float>(p.x), static_cast<float>(p.y));
    }
  } else {
    // 逐次删除面积贡献最小的凸包顶点；最小外接矩形会破坏斜视下的短斜边。
    while (hull.size() > 4) {
      size_t remove = 0;
      double smallest = 1e30;
      for (size_t i = 0; i < hull.size(); ++i) {
        const cv::Point2d a = hull[(i + hull.size() - 1) % hull.size()];
        const cv::Point2d b = hull[i];
        const cv::Point2d c = hull[(i + 1) % hull.size()];
        const double loss = std::abs((b - a).cross(c - a));
        if (loss < smallest) {smallest = loss; remove = i;}
      }
      hull.erase(hull.begin() + remove);
    }
    if (hull.size() != 4) {
      cv::Point2f box[4];
      cv::minAreaRect(pts_i).points(box);
      quad.assign(box, box + 4);
    } else {
      for (const auto & p : hull) {quad.emplace_back(float(p.x), float(p.y));}
    }
  }
  return orderCorners(quad);
}

// 阶段: 四边形拟合 —— 分边取最外侧点 -> 鲁棒直线 -> 相邻边求交；缺边用先验/上一帧预测补，角点离先验太远则整体退回先验框。
// 把紫色像素分配到四条边, 并只保留每条边的"最外侧"点。
//
// 为什么不用"按位置比例分组" (上 35%/下 35%/左 30%/右 30%):
//   1) 强透视下比例位置和实际边的对应会错, 角点附近的点会串边;
//   2) 紫框有厚度, 内边界点会把直线往场地里面拉 (实测污染 15-25px);
//   3) 内部/旁边的其它紫色物体会被误分组进来。
//
// 做法: 以先验四边形为准, 每条边用 (沿边参数 t, 有符号外法向距离 d) 描述,
// 丢掉 d 超出 [-0.05L, +edge_band_frac*L] 的点, 沿边分 edge_bins 个箱子,
// 每箱只留 d 最大的点 (即胶带外边缘), 再用这些点做鲁棒直线拟合。
std::vector<std::vector<cv::Point2f>> edgePoints(
  const std::vector<cv::Point2f> & pts,
  const std::vector<cv::Point2f> & prior,
  const FieldConfig & cfg)
{
  const int edges[4][2] = {{0, 1}, {1, 2}, {2, 3}, {3, 0}};
  cv::Point2f center(0, 0);
  for (const auto & p : prior) {
    center += p;
  }
  center *= 1.0f / static_cast<float>(prior.size());
  const int nbins = std::max(4, cfg.edge_bins);
  std::vector<std::vector<cv::Point2f>> out(4);
  for (int i = 0; i < 4; ++i) {
    const cv::Point2f a = prior[edges[i][0]], b = prior[edges[i][1]];
    const cv::Point2f ab = b - a;
    const double L = cv::norm(ab);
    if (L < 4) {continue;}
    const cv::Point2f u = ab * static_cast<float>(1.0 / L);
    cv::Point2f n(-u.y, u.x);
    const cv::Point2f mid((a.x + b.x) * 0.5f, (a.y + b.y) * 0.5f);
    if ((mid - center).dot(n) < 0) {n = -n;}            // n 指向场地外侧
    std::vector<double> best_d(nbins, -1e18);
    std::vector<cv::Point2f> best_p(nbins);
    std::vector<char> has(nbins, 0);
    for (const auto & p : pts) {
      const cv::Point2f v = p - a;
      const double t = v.dot(u) / L;
      const double d = v.dot(n);
      if (t < cfg.corner_exclude_frac || t > 1.0 - cfg.corner_exclude_frac) {continue;}
      if (d < -0.05 * L || d > cfg.edge_band_frac * L) {continue;}
      int bi = int(std::min(std::max(t, 0.0), 1.0) * (nbins - 1));
      if (!has[bi] || d > best_d[bi]) {has[bi] = 1; best_d[bi] = d; best_p[bi] = p;}
    }
    for (int k = 0; k < nbins; ++k) {
      if (has[k]) {
        out[i].push_back(best_p[k]);
      }
    }
  }
  return out;
}

// 两个四边形的"跳变程度": 平均角点距离 / 对角线长度
double quadJumpRatio(const std::vector<cv::Point2f> & a, const std::vector<cv::Point2f> & b)
{
  if (a.size() != 4 || b.size() != 4) {return 1e9;}
  double d = 0;
  for (int i = 0; i < 4; ++i) {
    d += cv::norm(a[i] - b[i]);
  }
  d /= 4.0;
  // 归一化基准取第一个参数的对角线，因此交换 a、b 后数值未必完全相同。
  const double diag = cv::norm(a[0] - a[2]);
  return diag > 1e-6 ? d / diag : 1e9;
}

// 拟合候选: assign_prior 只用来给紫色像素分边 (应该用候选自己的形状);
// fallback_prior 只在该边没测到时补线 (TRACK 时=上一帧预测, 否则=候选自己的先验)。
FitOut fitQuad(
  const std::vector<cv::Point2f> & pts, const FieldConfig & cfg,
  const std::vector<cv::Point2f> & assign_prior,
  const std::vector<cv::Point2f> & fallback_prior)
{
  FitOut out;
  // 分边先验与补线先验分开：前者描述当前候选轮廓，后者只在缺边时提供几何，不增加实测边数。
  out.prior = assign_prior.empty() ? priorQuad(pts) : assign_prior;
  const std::vector<cv::Point2f> & prior = out.prior;
  const std::vector<cv::Point2f> & fb = fallback_prior.empty() ? out.prior : fallback_prior;
  const int edges[4][2] = {{0, 1}, {1, 2}, {2, 3}, {3, 0}};
  const char * names[4] = {"top", "right", "bottom", "left"};

  const std::vector<std::vector<cv::Point2f>> sides = edgePoints(pts, prior, cfg);

  std::vector<Line> lines(4);
  std::vector<std::string> degraded;
  for (int i = 0; i < 4; ++i) {
    Line l; int n = 0;
    const bool fitted = fitLineRobust(sides[i], cfg, l, n);
    if (!fitted || n < cfg.min_side_inliers) {
            // 这条边这一帧没测到 -> 用上一帧预测(或候选自己的先验)那条边补
      lines[i] = lineThrough(fb[edges[i][0]], fb[edges[i][1]]);
      out.edge_state[i] = EdgeState::PREDICTED;
      degraded.push_back(names[i]);
    } else {
      lines[i] = l;
      out.edge_state[i] = EdgeState::MEASURED;
      ++out.measured_edges;
    }
  }
  if (out.measured_edges == 0) {
        // 四条边都没测到: 不产生几何, 交给上层用预测保持
    for (int i = 0; i < 4; ++i) {
      out.edge_state[i] = EdgeState::LOST;
    }
    out.reason = "四条边都没有足够像素";
    return out;
  }
  bool ok = true;
  const cv::Point2f p1 = intersectLines(lines[0], lines[3], ok);     // top ∩ left
  if (!ok) {out.reason = "对边近似平行, 交点退化"; return out;}
  const cv::Point2f p2 = intersectLines(lines[0], lines[1], ok);
  if (!ok) {out.reason = "对边近似平行, 交点退化"; return out;}
  const cv::Point2f p3 = intersectLines(lines[2], lines[1], ok);
  if (!ok) {out.reason = "对边近似平行, 交点退化"; return out;}
  const cv::Point2f p4 = intersectLines(lines[2], lines[3], ok);
  if (!ok) {out.reason = "对边近似平行, 交点退化"; return out;}

  std::vector<cv::Point2f> quad = orderCorners({p1, p2, p3, p4});
  // 角点一致性：拟合角点不能偏离先验框太远，避免少量噪声点带偏直线；
  // 短边较远时尤其容易出现这种退化，超出容差就退回先验框。
  const double tol = cfg.prior_corner_tol_frac * std::max(dist(prior[0], prior[2]), 1.0);
  double worst = 0;
  for (const auto & c : quad) {
    double m = 1e18;
    for (const auto & pc : prior) {
      m = std::min(m, dist(c, pc));
    }
    worst = std::max(worst, m);
  }
  if (worst > tol) {
        // 距离门槛按先验对角线缩放；退回先验只是产生备选几何，仍需调用方检查颜色和边证据。
    out.ok = true;
    out.quad = prior;
    out.used_prior_fallback = true;
    out.reason = "quad_prior_fallback";
    return out;
  }
  out.ok = true;
  out.quad = quad;
  if (!degraded.empty()) {
    out.reason = "degraded:";
    for (size_t i = 0; i < degraded.size(); ++i) {
      out.reason += (i ? "," : "") + degraded[i];
    }
  }
  return out;
}

bool checkQuad(
  const std::vector<cv::Point2f> & q, const cv::Size & sz, const FieldConfig & cfg,
  std::string & why)
{
  // sz 是当前搜索区域尺寸；面积占比随全图/ROI 改变，边长下限则始终是处理图像素。
  const double area = polyArea(q);
  if (area < cfg.quad_min_area_frac * sz.width * sz.height) {why = "面积过小"; return false;}
  if (!isConvexQuad(q)) {why = "非凸/自交"; return false;}
  const double top = dist(q[0], q[1]), bottom = dist(q[3], q[2]);
  const double left = dist(q[0], q[3]), right = dist(q[1], q[2]);
  if (std::min(top, bottom) < 8 || std::min(left, right) < 5) {why = "边长退化"; return false;}
  const double r1 = std::max(top, bottom) / std::max(std::min(top, bottom), 1e-6);
  const double r2 = std::max(left, right) / std::max(std::min(left, right), 1e-6);
  if (r1 < cfg.quad_side_ratio_lo || r1 > cfg.quad_side_ratio_hi) {why = "上下边长度比异常"; return false;}
  if (r2 < cfg.quad_side_ratio_lo || r2 > cfg.quad_side_ratio_hi) {why = "左右边长度比异常"; return false;}
  if (cfg.use_aspect_check) {
    // 这里直接比较图像中的平均边长比与米制宽高比，不消除透视，强斜视通常不启用此项。
    const double est = 0.5 * (top + bottom) / std::max(0.5 * (left + right), 1e-6);
    const double want = cfg.field_w_m / cfg.field_h_m;
    if (std::abs(est - want) / want > cfg.quad_max_aspect_err) {why = "长宽比偏差大"; return false;}
  }
  return true;
}

// 阶段: 透视与鸟瞰 —— 米制尺寸乘 px/m 定默认 BEV 大小；显式宽、高可分别覆盖，生成 BEV 还需已有角点。
// 经 validateFieldConfig 的配置保证有效尺寸在 16..4096；这里再取 16 下限，只为兜底直接调用未校验配置的场合。
cv::Size bevSize(const FieldConfig & cfg)
{
  const int w = cfg.bev_width >
    0 ? cfg.bev_width : static_cast<int>(std::lround(cfg.field_w_m * cfg.bev_px_per_m));
  const int h = cfg.bev_height >
    0 ? cfg.bev_height : static_cast<int>(std::lround(cfg.field_h_m * cfg.bev_px_per_m));
  return {std::max(16, w), std::max(16, h)};
}

}  // namespace field_detail
