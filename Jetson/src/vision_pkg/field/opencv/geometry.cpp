#include "field/opencv/algorithm.hpp"
#include "field/quad_order.hpp"
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
  const FieldConfig & cfg, int only_edge = -1)
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
    if (only_edge >= 0 && i != only_edge) {continue;}
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

// 近边大断口时，用左右两端的实际近墙底缘跨过缺口。两端都要有独立内点。
bool fitBrokenNearEdge(
  const QuadEvidence & evidence, const std::vector<cv::Point2f> & prior, const FieldConfig & cfg,
  Line & line)
{
  FieldConfig dense = cfg;
  dense.corner_exclude_frac = std::min(0.015, cfg.corner_exclude_frac);
  dense.edge_bins = std::max(96, cfg.edge_bins);
  const auto samples = edgePoints(evidence.pts, prior, dense, 2);
  const auto & points = samples[2];
  Line candidate;
  int inliers = 0;
  if (!fitLineRobust(points, cfg, candidate, inliers) || inliers < cfg.min_side_inliers) {
    return false;
  }
  const auto edge = prior[3] - prior[2];
  const double length_sq = edge.dot(edge);
  if (length_sq < 16.0) {return false;}
  int ends[2]{};
  const double band = std::max(cfg.fit_outlier_px, kMinInlierBandPx);
  for (const auto & p : points) {
    if (pointLineDistance(candidate, p) > band) {continue;}
    const double along = (p - prior[2]).dot(edge) / length_sq;
    if (along < 0.25) {++ends[0];}
    if (along > 0.75) {++ends[1];}
  }
  const int support_per_end = std::max(3, (cfg.min_side_inliers + 3) / 4);
  if (ends[0] < support_per_end || ends[1] < support_per_end) {return false;}
  line = candidate;
  return true;
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
// 拟合带之外只沿与近边相连的地面侧轮廓找端点，避免用墙顶或孤立紫点补角。
// 近边支持延伸到画面边界时端点不是角点，保留直线的画外交点。
void trimNearCorners(
  const Line & near_line, const QuadEvidence & evidence, const FieldConfig & cfg,
  std::vector<cv::Point2f> & quad, bool near_rebuilt)
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
  const double core_lo = lo, core_hi = hi;
  auto signed_distance = [&](const cv::Point2f & p) {
      return near_line.a * p.x + near_line.b * p.y + near_line.c;
    };
  const double ground_side = signed_distance((quad[0] + quad[1]) * 0.5f) < 0 ? 1.0 : -1.0;
  const double continuation_band = 4.0 * band;
  auto on_near_contour = [&](const cv::Point & p) {
      const double ground_distance = ground_side * signed_distance(p);
      return ground_distance >= -band && ground_distance <= continuation_band;
    };
  for (const auto & path : evidence.contours) {
    if (path.empty()) {continue;}
    // 从条带外开始绕一圈，跨轮廓起点的片段也能合并。
    const auto cut = std::find_if_not(path.begin(), path.end(), on_near_contour);
    const std::size_t first = std::size_t(cut - path.begin()) % path.size();
    double segment_lo = 1e18, segment_hi = -1e18;
    bool touches_lo = false, touches_hi = false;
    auto finish_segment = [&] {
        if (touches_lo) {lo = std::min(lo, segment_lo);}
        if (touches_hi) {hi = std::max(hi, segment_hi);}
        segment_lo = 1e18; segment_hi = -1e18;
        touches_lo = false; touches_hi = false;
      };
    for (std::size_t index = 0; index < path.size(); ++index) {
      const auto & p = path[(first + index) % path.size()];
      if (!on_near_contour(p)) {
        finish_segment();
        continue;
      }
      const double s = along(p);
      segment_lo = std::min(segment_lo, s);
      segment_hi = std::max(segment_hi, s);
      if (pointLineDistance(near_line, p) <= band) {
        touches_lo |= std::abs(s - core_lo) <= band;
        touches_hi |= std::abs(s - core_hi) <= band;
      }
    }
    finish_segment();
  }
  bool clipped_lo = false, clipped_hi = false;
  for (const auto & p : evidence.clipped) {
    if (pointLineDistance(near_line, p) > continuation_band) {continue;}
    const double s = along(p);
    if (s <= core_lo + band) {clipped_lo = true;}
    if (s >= core_hi - band) {clipped_hi = true;}
  }
  auto pointAt = [&](double s) {
    const double base = -near_line.c;   // 线上离原点最近点 = base·n
    const cv::Point2f foot(static_cast<float>(near_line.a * base), static_cast<float>(near_line.b * base));
    return foot + u * static_cast<float>(s - double((foot - origin).dot(u)));
  };
  // quad 为 top∩left、top∩right、bottom∩right、bottom∩left；后两个在近边上；按沿线坐标分别夹到实测端点以内。
  const double mid = 0.5 * (along(quad[2]) + along(quad[3]));
  for (int k : {2, 3}) {
    const double s = along(quad[k]);
    const bool low_end = s < mid;
    // 断口较大时凸包侧线也可能内收，直接用两端独立支持的底边端点。
    if (low_end && !clipped_lo && (near_rebuilt || s < lo)) {quad[k] = pointAt(lo);}
    if (!low_end && !clipped_hi && (near_rebuilt || s > hi)) {quad[k] = pointAt(hi);}
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
  bool near_rebuilt = false;
  for (int i = 0; i < 4; ++i) {
    Line l; int n = 0;
    const bool fitted = fitLineRobust(sides[i], cfg, l, n) && n >= cfg.min_side_inliers;
    const bool recovered_near = i == 2 && !fitted &&
      fitBrokenNearEdge(evidence, prior, cfg, l);
    if (recovered_near) {near_rebuilt = true;}
    if (!fitted && !recovered_near) {
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
  if (out.edge_state[2] == EdgeState::MEASURED) {
    trimNearCorners(lines[2], evidence, cfg, corners, near_rebuilt);
  }
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
  // 与内沿地面近锚使用同一约定：上边在远端，下边在近端；地面不能向近端显著收窄。
  // 紫盒叠红盒也能形成很强的颜色包围，但其立面/顶面轮廓会出现相反的透视方向。
  // 允许两倍现有拟合内点带宽的差值，避免等宽或近似正视的边被像素误差拒绝。
  const double perspective_tolerance = 2.0 * std::max(cfg.fit_outlier_px, kMinInlierBandPx);
  if (top > bottom + perspective_tolerance) {why = "近边短于远边, 地面透视方向不符"; return false;}
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

double groundOutsideImage(
  const std::vector<cv::Point2f> & ground, cv::Size image_size, cv::Size bev_size,
  const cv::Mat & image_to_bev)
{
  const float right = image_size.width - 1.f, bottom = image_size.height - 1.f;
  const auto in_frame = [&](const cv::Point2f & p) {
      return p.x >= 0 && p.x <= right && p.y >= 0 && p.y <= bottom;
    };
  if (std::all_of(ground.begin(), ground.end(), in_frame)) {return 0.0;}

  const std::vector<cv::Point2f> frame{{0, 0}, {right, 0}, {right, bottom}, {0, bottom}};
  std::vector<cv::Point2f> visible;
  if (cv::intersectConvexConvex(ground, frame, visible) <= 0 || visible.size() < 3) {return 1.0;}
  cv::perspectiveTransform(visible, visible, image_to_bev);
  const double full_area = double(bev_size.width - 1) * (bev_size.height - 1);
  const double ratio = cv::contourArea(visible) / full_area;
  return std::isfinite(ratio) ? 1.0 - std::clamp(ratio, 0.0, 1.0) : 1.0;
}

namespace {
// 内沿扫描：连续多少个内部像素才算过渡（滤掉单像素混色）。
constexpr int kInteriorRunPx = 2;
// 侧内沿扫描只看候选的外侧一半，越过中线的命中不属于该边。
constexpr double kInnerScanDepth = 0.5;

// 候选范围内沿一条边向内扫描：每个独立行/列取第一个连续内部像素。
// along=true 时按列扫描（上/下边），from_end=true 从底/右端开始。
std::vector<cv::Point2f> innerTransitions(
  const cv::Mat & mask, bool along, bool from_end, const FieldConfig & cfg)
{
  // 候选包含围边立面和近墙遮挡；采样槽分配到实际出现内部颜色过渡的行/列。
  // 否则低机位只露出一条地面时，大部分槽落在墙面，侧边会因点数不足退回先验。
  const cv::Rect support = cv::boundingRect(mask);
  const int start = along ? support.x : support.y;
  const int length = along ? support.width : support.height;
  // 远边上方可能连着同色背景；内部地面并不保证在外框高度的前半。
  // 两侧仍限制到各自半区，避免把对侧内部当成本侧边界。
  const int depth = along ? support.y + support.height :
    static_cast<int>(mask.cols * kInnerScanDepth);
  std::vector<cv::Point2f> candidates;
  for (int pos = start; pos < start + length; ++pos) {
    int run = 0;
    for (int step = 0; step < depth; ++step) {
      const int inward = from_end ? (along ? mask.rows : mask.cols) - 1 - step : step;
      const bool hit = along ? mask.at<uchar>(inward, pos) != 0 : mask.at<uchar>(pos, inward) != 0;
      run = hit ? run + 1 : 0;
      if (run == kInteriorRunPx) {
        const int first = from_end ? inward + kInteriorRunPx - 1 : inward - kInteriorRunPx + 1;
        candidates.emplace_back(along ? cv::Point2f(float(pos), float(first)) : cv::Point2f(float(first), float(pos)));
        break;
      }
    }
  }
  const int samples = std::max(4, cfg.edge_bins);
  std::vector<cv::Point2f> points;
  int previous = -1;
  for (int k = 0; k < samples && !candidates.empty(); ++k) {
    const double t = cfg.corner_exclude_frac + (1.0 - 2.0 * cfg.corner_exclude_frac) * (k + 0.5) / samples;
    const int index = static_cast<int>(t * candidates.size());
    if (index == previous) {continue;}
    previous = index;  // 窄可见区的一行只算一次，不能靠重复采样满足实测内点门槛。
    points.push_back(candidates[index]);
  }
  return points;
}

bool fitAnchoredSide(const std::vector<cv::Point2f> & points, const cv::Point2f & anchor,
  const FieldConfig & cfg, Line & line)
{
  if (int(points.size()) < cfg.min_side_points) {return false;}
  std::vector<cv::Point2f> best;
  for (const auto & point : points) {
    const auto candidate = lineThrough(anchor, point);
    std::vector<cv::Point2f> inliers;
    for (const auto & p : points) {
      if (pointLineDistance(candidate, p) <= cfg.fit_outlier_px) {inliers.push_back(p);}
    }
    if (inliers.size() > best.size()) {best = std::move(inliers);}
  }
  if (int(best.size()) < cfg.min_side_inliers) {return false;}
  cv::Matx22d covariance = cv::Matx22d::zeros();
  for (const auto & p : best) {
    const cv::Vec2d d(p.x - anchor.x, p.y - anchor.y);
    covariance += d * d.t();
  }
  cv::Mat values, vectors;
  cv::eigen(cv::Mat(covariance), values, vectors);
  const double a = vectors.at<double>(1, 0), b = vectors.at<double>(1, 1);
  line = normalizeLine(a, b, -a * anchor.x - b * anchor.y);
  int inliers = 0;
  for (const auto & p : points) {
    if (pointLineDistance(line, p) <= std::max(cfg.fit_outlier_px, kMinInlierBandPx)) {++inliers;}
  }
  return inliers >= cfg.min_side_inliers;
}
}  // namespace

InnerQuad fitInnerQuad(
  const cv::Mat & interior, const std::vector<cv::Point2f> & outer, const FieldConfig & cfg,
  const cv::Mat & lab, const cv::Mat & purple, int missing_purple_side)
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
  const cv::Mat inverse = forward.inv();
  // 在处理图上保留独立像素证据，避免外框连着背景时，固定大小的鸟瞰把可见地面压成不足几行。
  auto scan_quad = outer;
  if (missing_purple_side == 1 || missing_purple_side == 3) {
    // 凸包的猜测边可能切进地面。仅在该侧放宽取证范围，仍要求真实的颜色终点。
    const int top = missing_purple_side == 1 ? 1 : 0;
    const int bottom = missing_purple_side == 1 ? 2 : 3;
    const float margin = static_cast<float>(cfg.expand_frac);
    scan_quad[top] += (outer[top] - outer[1 - top]) * margin;
    scan_quad[bottom] += (outer[bottom] - outer[5 - bottom]) * margin;
  }
  const cv::Rect bounds = cv::boundingRect(scan_quad) & cv::Rect(0, 0, interior.cols, interior.rows);
  cv::Mat mask = cv::Mat::zeros(bounds.size(), CV_8U);
  std::vector<cv::Point> polygon;
  for (const auto & p : scan_quad) {polygon.emplace_back(cvRound(p.x) - bounds.x, cvRound(p.y) - bounds.y);}
  cv::fillConvexPoly(mask, polygon, cv::Scalar(255));
  cv::bitwise_and(mask, interior(bounds), mask);
  const double purple_hue = (cfg.purple_hue_lo_deg + cfg.purple_hue_hi_deg) * CV_PI / 360;
  const double inner_hue = (cfg.use_blue_interior ? cfg.blue_hue_lo_deg + cfg.blue_hue_hi_deg :
    cfg.red_hue_lo_deg + cfg.red_hue_hi_deg) * CV_PI / 360;
  const cv::Point2d contrast(std::cos(inner_hue) - std::cos(purple_hue),
    std::sin(inner_hue) - std::sin(purple_hue));
  const auto transitions = [&](bool along, bool from_end) {
      auto points = innerTransitions(mask, along, from_end, cfg);
      for (auto & p : points) {p += cv::Point2f(float(bounds.x), float(bounds.y));}
      if (!lab.empty()) {
        // 严格颜色判据负责确认内部身份；几何位置取附近的材质色度跃迁，
        // 不把混色、JPEG或开运算导致的阈值内缩当作物理边界。
        const cv::Point direction = along ? cv::Point(0, 1) : cv::Point(from_end ? -1 : 1, 0);
        const int radius = cfg.edge_probe_radius + cfg.interior_open_ksize;
        for (auto & p : points) {
          const cv::Point seed(cvRound(p.x), cvRound(p.y));
          double best = 0;
          for (int step = -radius; step <= 1; ++step) {
            const auto b = seed + direction * step, a = b - direction;
            if (a.x < 0 || a.y < 0 || a.x >= lab.cols || a.y >= lab.rows ||
              b.x < 0 || b.y < 0 || b.x >= lab.cols || b.y >= lab.rows) {continue;}
            const auto before = lab.at<cv::Vec3b>(a), after = lab.at<cv::Vec3b>(b);
            const double gradient = contrast.x * (int(after[1]) - before[1]) +
              contrast.y * (int(after[2]) - before[2]);
            if (gradient > best) {best = gradient; p = 0.5f * cv::Point2f(a + b);}
          }
        }
      }
      if (!points.empty()) {cv::perspectiveTransform(points, points, forward);}
      return points;
    };
  const float tx = static_cast<float>(w * cfg.wall_thickness_m / cfg.field_w_m);
  const float ty = static_cast<float>(h * cfg.wall_thickness_m / cfg.field_h_m);
  // 远端与两侧内沿都在地面（远墙、侧墙内立面底边）；近端内沿被近墙挡住，不扫描。
  // 先验（没测到时）：外沿内缩围边厚度。
  struct Side {std::vector<cv::Point2f> scan; cv::Point2f a, b;};
  Side sides[3] = {
    {transitions(true, false), {tx, ty}, {w - tx, ty}},       // 远
    {transitions(false, true), {w - tx, ty}, {w - tx, h}},    // 右
    {transitions(false, false), {tx, ty}, {tx, h}}};          // 左
  InnerQuad out;
  Line lines[3];
  const cv::Rect2f frame(0.f, 0.f, float(interior.cols), float(interior.rows));
  const bool observed_near = frame.contains(outer[2]) && frame.contains(outer[3]);
  bool far_measured = false;
  int recovered_side = -1;
  for (int i = 0; i < 3; ++i) {
    if (i > 0 && far_measured) {
      // 远内沿以上是远墙或凸起物块，不能把那里的内部颜色竖边当作地面侧内沿。
      auto & points = sides[i].scan;
      points.erase(std::remove_if(points.begin(), points.end(), [&](const cv::Point2f & p) {
        return (lines[0].a * p.x + lines[0].b * p.y + lines[0].c) * lines[0].b < 0;
      }), points.end());
    }
    Line line; int inliers = 0;
    const bool recover_side = i > 0 && (i == 1 ? 1 : 3) == missing_purple_side;
    const bool anchored = i > 0 && observed_near && !lab.empty() && !recover_side &&
      fitAnchoredSide(sides[i].scan, {i == 1 ? w - tx : tx, h}, cfg, line);
    bool measured = anchored ||
      (fitLineRobust(sides[i].scan, cfg, line, inliers) && inliers >= cfg.min_side_inliers);
    if (measured && i > 0 && !anchored) {
      // 边缘被遮挡时，内部物块的竖边可能被误认为侧内沿；侧线不得穿入对侧区域。
      if (std::abs(line.a) < 1e-6) {
        measured = false;
      } else {
        bool intersected = false;
        const auto far = intersectLines(lines[0], line, intersected);
        const double far_x = far.x;
        const double near_x = -(line.b * h + line.c) / line.a;
        const double boundary = (i == 1 ? 0.75 : 0.25) * w;
        measured = intersected && std::isfinite(far_x) && std::isfinite(near_x) &&
          (i == 1 ? (far_x >= boundary && near_x >= boundary) :
          (far_x <= boundary && near_x <= boundary));
      }
    }
    if (measured && i > 0 && !purple.empty()) {
      // 高光/物块边界也有色度跃迁。实测侧线的内点必须有足量紫墙支持，
      // 不要求每个受模糊/JPEG影响的采样点都通过；沿用拟合的真实内点下限。
      std::vector<cv::Point2f> source;
      cv::perspectiveTransform(sides[i].scan, source, inverse);
      int supported = 0;
      int ground_transitions = 0;
      const int radius = cfg.edge_probe_radius + cfg.interior_open_ksize;
      for (size_t k = 0; k < source.size(); ++k) {
        if (pointLineDistance(line, sides[i].scan[k]) > std::max(cfg.fit_outlier_px, kMinInlierBandPx)) {continue;}
        const int y = cvRound(source[k].y);
        const int x_center = cvRound(source[k].x);
        const int outward = i == 1 ? 1 : -1;
        for (int step = 1; step <= radius; ++step) {
          const int x = x_center + outward * step;
          if (x >= 0 && y >= 0 && x < purple.cols && y < purple.rows && purple.at<uchar>(y, x)) {++supported; break;}
        }
        if (!recover_side || y < 0 || y >= interior.rows) {continue;}
        const int in = x_center - outward * radius, out = x_center + outward * radius;
        if (in >= 0 && in < interior.cols && out >= 0 && out < interior.cols &&
          interior.at<uchar>(y, in) && !interior.at<uchar>(y, out) &&
          !interior.at<uchar>(y, out - outward)) {++ground_transitions;}
      }
      measured = supported >= cfg.min_side_inliers;
      // 仅在其余三条外边已有紫色和几何约束时，缺紫边的一侧才可用独立的地面颜色边界。
      if (!measured && ground_transitions >= cfg.min_side_points) {
        measured = true;
        recovered_side = i;
      }
    }
    if (measured) {
      lines[i] = line;
      ++out.measured_edges;
      out.measured[i] = true;
    } else {
      lines[i] = lineThrough(sides[i].a, sides[i].b);
    }
    if (i == 0) {far_measured = measured;}
  }
  // 地面单应：内部地面 (0,0)/(W,0) 是远内沿与两侧内沿的交点。
  // 近边前立面底边位于地面，其端点与已知墙厚直接确定侧内沿在该边上的位置；
  // 不把仅露出很短一段的侧线外推到近边，否则亚像素角度误差会放大为明显偏移。
  // 由它求被近墙挡住的近端内沿角 (0,H)/(W,H)，四角都在地面。
  bool ok = true;
  std::vector<cv::Point2f> image(4);
  image[0] = intersectLines(lines[0], lines[2], ok);
  if (ok) {image[1] = intersectLines(lines[0], lines[1], ok);}
  image[2] = {w - tx, h};
  image[3] = {tx, h};
  // 缺少实测内沿时保留原交点一致性检查，不能用固定近锚掩盖不闭合的侧线。
  // 近角在画外时，墙顶侧线外推的端点也不能作为地面锚点；近视角可用较长的实测地面侧线。
  if (out.measured_edges != 3 || !frame.contains(outer[2]) || !frame.contains(outer[3])) {
    const Line near_outer = lineThrough(rect[3], rect[2]);
    if (ok) {image[2] = intersectLines(near_outer, lines[1], ok);}
    if (ok) {image[3] = intersectLines(near_outer, lines[2], ok);}
  } else if (ok && recovered_side > 0) {
    // 缺紫一侧的外框角不作近锚。用实测内色侧线与近墙底边的相交位置重建该侧。
    image[recovered_side + 1] = intersectLines(lineThrough(rect[3], rect[2]), lines[recovered_side], ok);
  }
  bool inside = ok && isConvexQuad(image);
  const double scan_margin = w * cfg.expand_frac;
  const double min_x = recovered_side == 2 ? -scan_margin - 1 : -1;
  const double max_x = recovered_side == 1 ? w + scan_margin + 1 : w + 1;
  for (const auto & c : image) {
    inside = inside && c.x >= min_x && c.y >= -1 && c.x <= max_x && c.y <= h + 1;
  }
  // 交点退化或不再是包在外沿内的凸四边形时，本帧内沿不可信，整体用内缩先验。
  if (!inside) {
    image = {{tx, ty}, {w - tx, ty}, {w - tx, h}, {tx, h}};
    out.measured_edges = 0;
    std::fill(std::begin(out.measured), std::end(out.measured), false);
  }
  const float iw = inner.width - 1.f, ih = inner.height - 1.f;
  const float near_y = static_cast<float>(ih *
    (1.0 + cfg.wall_thickness_m / (cfg.field_h_m - 2.0 * cfg.wall_thickness_m)));
  const cv::Mat ground = cv::getPerspectiveTransform(
    std::vector<cv::Point2f>{{0, 0}, {iw, 0}, {iw, near_y}, {0, near_y}}, image);
  std::vector<cv::Point2f> corners;
  cv::perspectiveTransform(std::vector<cv::Point2f>{{0, 0}, {iw, 0}, {iw, ih}, {0, ih}}, corners, ground);
  cv::perspectiveTransform(corners, out.quad, inverse);
  return out;
}

}  // namespace field_detail
