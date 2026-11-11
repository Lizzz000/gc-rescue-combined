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

// 内点计数带宽下限（像素）：胶带边缘的亚像素抖动与 JPEG 振铃约 ±1.5 px。
constexpr double kMinInlierBandPx = 3.0;

// 一致性初值：按 1/4、1/3、1/2 边长间距取点对连线，取 fit_outlier_px 带内点数最多（同数取残差和最小）的直线。
// 残差中位数剔除对集中在一端的污染（凸包截角处的斜点、贴边物块）没有抵抗力，会把整条线带歪；
// 一致性初值让拟合跟随最长的共线段。点按分箱顺序排列，点对数随点数线性增长。
std::vector<cv::Point2f> consensusInliers(const std::vector<cv::Point2f> & pts, double band)
{
  const size_t n = pts.size();
  size_t best_count = 0;
  double best_cost = 0;
  Line best;
  for (size_t gap : {std::max<size_t>(1, n / 4), std::max<size_t>(1, n / 3), std::max<size_t>(1, n / 2)}) {
    for (size_t i = 0; i + gap < n; ++i) {
      if (dist(pts[i], pts[i + gap]) < 1.0) {continue;}
      const Line line = lineThrough(pts[i], pts[i + gap]);
      size_t count = 0;
      double cost = 0;
      for (const auto & p : pts) {
        const double d = pointLineDistance(line, p);
        if (d <= band) {++count; cost += d;}
      }
      if (count > best_count || (count == best_count && cost < best_cost)) {
        best_count = count; best_cost = cost; best = line;
      }
    }
  }
  if (best_count == 0) {return pts;}
  std::vector<cv::Point2f> inliers;
  for (const auto & p : pts) {
    if (pointLineDistance(best, p) <= band) {inliers.push_back(p);}
  }
  return inliers;
}

// 一致性初值 + 总最小二乘／主成分直线拟合：用较小特征值方向作法线，逐轮剔除残差离群点。
bool fitLineRobust(
  const std::vector<cv::Point2f> & pts, const FieldConfig & cfg, Line & out,
  int & inliers)
{
  inliers = 0;
  if (static_cast<int>(pts.size()) < cfg.min_side_points) {return false;}
  std::vector<cv::Point2f> P = consensusInliers(pts, cfg.fit_outlier_px);
  if (static_cast<int>(P.size()) < cfg.min_side_inliers) {return false;}
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
    if (pointLineDistance(line, p) <= std::max(cfg.fit_outlier_px, kMinInlierBandPx)) {++inliers;}}
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

// 近端角点：近边实测的是近墙前立面底边（z=0），两侧实测的是侧墙顶外沿（z=h），外轮廓在近角处是六边形，
// 侧线与近边的交点总落在真实近角外侧。近边实测点在画内结束时，前立面底边的端点就是近角，取交点与端点中靠内者；
// 近边支持延伸到画面边界（有贴边点落在近边带内）时端点不是角点，保留交点。
void trimNearCorners(
  const Line & near_line, const QuadEvidence & evidence, const FieldConfig & cfg,
  std::vector<cv::Point2f> & quad)
{
  const double band = std::max(cfg.fit_outlier_px, kMinInlierBandPx);
  const cv::Point2f u(static_cast<float>(near_line.b), static_cast<float>(-near_line.a));
  const cv::Point2f origin = quad[2];
  auto along = [&](const cv::Point2f & p) {return double((p - origin).dot(u));};
  double lo = 1e18, hi = -1e18;
  for (const auto & p : evidence.pts) {
    if (pointLineDistance(near_line, p) > band) {continue;}
    lo = std::min(lo, along(p)); hi = std::max(hi, along(p));
  }
  if (lo > hi) {return;}
  bool clipped_lo = false, clipped_hi = false;
  for (const auto & p : evidence.clipped) {
    if (pointLineDistance(near_line, p) > band) {continue;}
    (along(p) < 0.5 * (lo + hi) ? clipped_lo : clipped_hi) = true;
  }
  auto pointAt = [&](double s) {
    const double base = -near_line.c;   // 线上离原点最近点 = base·n
    const cv::Point2f foot(static_cast<float>(near_line.a * base), static_cast<float>(near_line.b * base));
    return foot + u * static_cast<float>(s - double((foot - origin).dot(u)));
  };
  // quad 为 top∩left、top∩right、bottom∩right、bottom∩left；后两个在近边上；按沿线坐标分别夹到实测端点以内。
  for (int k : {2, 3}) {
    const double s = along(quad[k]);
    const bool low_end = s < 0.5 * (along(quad[2]) + along(quad[3]));
    if (low_end && !clipped_lo && s < lo) {quad[k] = pointAt(lo);}
    if (!low_end && !clipped_hi && s > hi) {quad[k] = pointAt(hi);}
  }
}

// 拟合候选：分边先验取全部证据（含贴边点）的凸包简化，用来给像素分边，并在该边没测到时补线。
// 只用本帧证据，不借历史预测。
// 贴边点不参与直线拟合；某条边只有贴边点时整侧出画，几何不完整，不产生候选。
FitOut fitQuad(const QuadEvidence & evidence, const FieldConfig & cfg)
{
  FitOut out;
  std::vector<cv::Point2f> all = evidence.pts;
  all.insert(all.end(), evidence.clipped.begin(), evidence.clipped.end());
  if (evidence.pts.size() < 4) {out.reason = "四条边都没有足够像素"; return out;}
  out.prior = priorQuad(all);
  const std::vector<cv::Point2f> & prior = out.prior;
  const int edges[4][2] = {{0, 1}, {1, 2}, {2, 3}, {3, 0}};
  const char * names[4] = {"top", "right", "bottom", "left"};

  const std::vector<std::vector<cv::Point2f>> sides = edgePoints(evidence.pts, prior, cfg);
  const std::vector<std::vector<cv::Point2f>> clipped = edgePoints(evidence.clipped, prior, cfg);

  std::vector<Line> lines(4);
  std::vector<std::string> degraded;
  for (int i = 0; i < 4; ++i) {
    Line l; int n = 0;
    const bool fitted = fitLineRobust(sides[i], cfg, l, n);
    if (!fitted || n < cfg.min_side_inliers) {
      if (static_cast<int>(clipped[i].size()) >= cfg.min_side_points) {
        out.out_of_frame = true;
        out.reason = std::string(names[i]) + " 边整侧出画, 几何不完整";
        return out;
      }
            // 这条边这一帧没测到 -> 用候选自己的凸包先验那条边补
      lines[i] = lineThrough(prior[edges[i][0]], prior[edges[i][1]]);
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

  std::vector<cv::Point2f> corners{p1, p2, p3, p4};
  if (out.edge_state[2] == EdgeState::MEASURED) {trimNearCorners(lines[2], evidence, cfg, corners);}
  std::vector<cv::Point2f> quad = orderCorners(corners);
  // 角点一致性（单侧）：先验凸包来自实测点，真实外沿只会在缺证据或出画处越出凸包；
  // 角点落进先验内部且离最近先验角超过容差，才说明直线被带偏，此时退回先验框。画外角点必在凸包外，不受此约束。
  const double tol = cfg.prior_corner_tol_frac * std::max(dist(prior[0], prior[2]), 1.0);
  bool inward = false;
  for (const auto & c : quad) {
    double m = 1e18;
    for (const auto & pc : prior) {
      m = std::min(m, dist(c, pc));
    }
    if (m > tol && cv::pointPolygonTest(prior, c, false) > 0) {inward = true;}
  }
  if (inward) {
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

// 阶段: 透视与鸟瞰 —— BEV 对应内沿四边形（内部地面）：米制内部尺寸 (field - 2·wall_thickness) 乘 px/m 定默认大小；
// 显式宽、高可分别覆盖。经 validateFieldConfig 的配置保证有效尺寸在 16..4096；这里再取 16 下限，只为兜底直接调用未校验配置的场合。
cv::Size bevSize(const FieldConfig & cfg)
{
  const double inner_w = cfg.field_w_m - 2.0 * cfg.wall_thickness_m;
  const double inner_h = cfg.field_h_m - 2.0 * cfg.wall_thickness_m;
  const int w = cfg.bev_width > 0 ? cfg.bev_width : static_cast<int>(std::lround(inner_w * cfg.bev_px_per_m));
  const int h = cfg.bev_height > 0 ? cfg.bev_height : static_cast<int>(std::lround(inner_h * cfg.bev_px_per_m));
  return {std::max(16, w), std::max(16, h)};
}

namespace {
// 内沿扫描：连续多少个内部像素才算过渡（滤掉单像素混色）。
constexpr int kInteriorRunPx = 2;
// 内沿扫描只看外沿鸟瞰的外侧一半，越过中线的命中不属于该边。
constexpr double kInnerScanDepth = 0.5;

// 外沿鸟瞰中沿一条边向内扫描：每个采样位置取第一个连续内部像素。
// along=true 时按列扫描（上/下边），from_end=true 从底/右端开始。
std::vector<cv::Point2f> innerTransitions(
  const cv::Mat & mask, bool along, bool from_end, const FieldConfig & cfg)
{
  const int length = along ? mask.cols : mask.rows;
  const int depth = static_cast<int>((along ? mask.rows : mask.cols) * kInnerScanDepth);
  const int samples = std::max(4, cfg.edge_bins);
  std::vector<cv::Point2f> points;
  for (int k = 0; k < samples; ++k) {
    const double t = cfg.corner_exclude_frac + (1.0 - 2.0 * cfg.corner_exclude_frac) * (k + 0.5) / samples;
    const int pos = std::clamp(static_cast<int>(t * (length - 1)), 0, length - 1);
    int run = 0;
    for (int step = 0; step < depth; ++step) {
      const int inward = from_end ? (along ? mask.rows : mask.cols) - 1 - step : step;
      const bool hit = along ? mask.at<uchar>(inward, pos) != 0 : mask.at<uchar>(pos, inward) != 0;
      run = hit ? run + 1 : 0;
      if (run == kInteriorRunPx) {
        const int first = from_end ? inward + kInteriorRunPx - 1 : inward - kInteriorRunPx + 1;
        points.emplace_back(along ? cv::Point2f(float(pos), float(first)) : cv::Point2f(float(first), float(pos)));
        break;
      }
    }
  }
  return points;
}
}  // namespace

InnerQuad fitInnerQuad(
  const cv::Mat & interior, const std::vector<cv::Point2f> & outer, const FieldConfig & cfg)
{
  // 扫描用外沿鸟瞰（外沿四边形到米制外沿矩形的单应，像素密度与输出 BEV 相同）；只是图像的射影重参数化，
  // 直线仍是直线。外沿近边是近墙前立面底边（z=0），其余外沿边在墙顶（z=h），因此只借用近边。
  const cv::Size inner = bevSize(cfg);
  const double sx = inner.width / (cfg.field_w_m - 2.0 * cfg.wall_thickness_m);
  const double sy = inner.height / (cfg.field_h_m - 2.0 * cfg.wall_thickness_m);
  const cv::Size size(static_cast<int>(std::lround(cfg.field_w_m * sx)),
    static_cast<int>(std::lround(cfg.field_h_m * sy)));
  const float w = size.width - 1.f, h = size.height - 1.f;
  const std::vector<cv::Point2f> rect{{0, 0}, {w, 0}, {w, h}, {0, h}};
  const cv::Mat forward = cv::getPerspectiveTransform(outer, rect);
  cv::Mat mask;
  cv::warpPerspective(interior, mask, forward, size, cv::INTER_NEAREST, cv::BORDER_CONSTANT, cv::Scalar(0));
  const float tx = static_cast<float>(cfg.wall_thickness_m * sx);
  const float ty = static_cast<float>(cfg.wall_thickness_m * sy);
  // 远端与两侧内沿都在地面（远墙、侧墙内立面底边）；近端内沿被近墙挡住，不扫描。
  // 先验（没测到时）：外沿内缩围边厚度。
  struct Side {std::vector<cv::Point2f> scan; cv::Point2f a, b;};
  const Side sides[3] = {
    {innerTransitions(mask, true, false, cfg), {tx, ty}, {w - tx, ty}},       // 远
    {innerTransitions(mask, false, true, cfg), {w - tx, ty}, {w - tx, h}},    // 右
    {innerTransitions(mask, false, false, cfg), {tx, ty}, {tx, h}}};          // 左
  InnerQuad out;
  Line lines[3];
  for (int i = 0; i < 3; ++i) {
    Line line; int inliers = 0;
    bool measured = fitLineRobust(sides[i].scan, cfg, line, inliers) && inliers >= cfg.min_side_inliers;
    if (measured && i > 0) {
      // 边缘被遮挡时，内部物块的竖边可能被误认为侧内沿；侧线不得穿入对侧区域。
      if (std::abs(line.a) < 1e-6) {
        measured = false;
      } else {
        const double far_x = -(line.b * ty + line.c) / line.a;
        const double near_x = -(line.b * h + line.c) / line.a;
        const double boundary = (i == 1 ? 0.75 : 0.25) * w;
        measured = std::isfinite(far_x) && std::isfinite(near_x) &&
          (i == 1 ? (far_x >= boundary && near_x >= boundary) :
          (far_x <= boundary && near_x <= boundary));
      }
    }
    if (measured) {
      lines[i] = line;
      ++out.measured_edges;
    } else {
      lines[i] = lineThrough(sides[i].a, sides[i].b);
    }
  }
  const Line near_outer = lineThrough({0, h}, {w, h});
  // 地面单应：内部地面 (0,0)/(W,0) 是远内沿与两侧内沿的交点，(W,H+t)/(0,H+t) 是两侧内沿与外沿近边（z=0）的交点。
  // 由它求被近墙挡住的近端内沿角 (0,H)/(W,H)，四角都在地面。
  bool ok = true;
  std::vector<cv::Point2f> image(4);
  image[0] = intersectLines(lines[0], lines[2], ok);
  if (ok) {image[1] = intersectLines(lines[0], lines[1], ok);}
  if (ok) {image[2] = intersectLines(near_outer, lines[1], ok);}
  if (ok) {image[3] = intersectLines(near_outer, lines[2], ok);}
  bool inside = ok && isConvexQuad(image);
  for (const auto & c : image) {inside = inside && c.x >= -1 && c.y >= -1 && c.x <= w + 1 && c.y <= h + 1;}
  // 交点退化或不再是包在外沿内的凸四边形时，本帧内沿不可信，整体用内缩先验。
  if (!inside) {
    image = {{tx, ty}, {w - tx, ty}, {w - tx, h}, {tx, h}};
    out.measured_edges = 0;
  }
  const float iw = inner.width - 1.f, ih = inner.height - 1.f;
  const float near_y = static_cast<float>(ih + cfg.wall_thickness_m * sy);
  const cv::Mat ground = cv::getPerspectiveTransform(
    std::vector<cv::Point2f>{{0, 0}, {iw, 0}, {iw, near_y}, {0, near_y}}, image);
  std::vector<cv::Point2f> corners;
  cv::perspectiveTransform(std::vector<cv::Point2f>{{0, 0}, {iw, 0}, {iw, ih}, {0, ih}}, corners, ground);
  cv::perspectiveTransform(corners, out.quad, forward.inv());
  return out;
}

}  // namespace field_detail
