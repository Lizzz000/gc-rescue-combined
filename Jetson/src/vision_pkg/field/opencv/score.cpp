#include "field/opencv/algorithm.hpp"
#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>

// 候选评分、准入和鸟瞰黑带。几何拟合在 geometry.cpp。

// ---- 候选证据、评分与准入 ----
// 边支持度、内部网格、外侧风险、几何与时序共同形成 CandidateScore；
// 硬门槛先决定资格，加权总分再用于排序。搜索、回退与切换共用准入判定。

namespace field_detail {

// 阶段: 证据与评分 —— 分项硬门槛决定能否准入，total 负责加权排序并另有总分下限。
// 多特征评分同时保留边、颜色、几何和时序证据。

// 沿一条边均匀采样, 统计有多少采样点附近是紫色。
// 采样点在"边法线方向"上还会左右各偏移 ±r, ±r/2, 取命中即算 (等价于一个带状探测),
// 因为拟合出来的线是胶带外边缘, 胶带像素其实落在线的内侧 1~3px。
double supportAlongEdge(
  const cv::Mat & mask, const cv::Point2f & a, const cv::Point2f & b,
  int samples, int radius, int & hits)
{
  hits = 0;
  if (mask.empty() || samples <= 0) {return 0.0;}
  cv::Point2f d = b - a;
  const float len = std::sqrt(d.dot(d));
  if (len < 1e-6f) {return 0.0;}
  d *= 1.0f / len;
  const cv::Point2f n(-d.y, d.x);
  const float offs[5] = {0.f, -0.5f, 0.5f, -1.f, 1.f};
  for (int i = 0; i < samples; ++i) {
    const float t = samples == 1 ? 0.5f : static_cast<float>(i) / (samples - 1);
    bool hit = false;
    for (int k = 0; k < 5 && !hit; ++k) {
      const cv::Point2f p = a + (b - a) * t + n * (offs[k] * radius);
      const int x = cvRound(p.x), y = cvRound(p.y);
      if (x < 0 || y < 0 || x >= mask.cols || y >= mask.rows) {continue;}
      if (mask.at<uchar>(y, x) > 0) {hit = true;}
    }
    if (hit) {++hits;}
  }
  // 分母保留全部采样点，落在图像外的点视为无证据，不能靠裁剪缩小分母抬高边支持度。
  return static_cast<double>(hits) / samples;
}

// 四条边的紫色支持度 -> score.edge_support / score.purple / score.edges_ok
//
// 关键: 不能因为"某条边这一帧看不到"就把紫框证据拉到 0 —— 那正是要允许的情况。
// purple = 0.6 * 最强两条边的平均 + 0.4 * 四条边的平均
// (既奖励"确实测到的强边", 也保留"整体像不像场地"的信息)
void scoreEdges(
  const cv::Mat & purple_probe, const std::vector<cv::Point2f> & quad,
  const FieldConfig & cfg, CandidateScore & score)
{
  const int pairs[4][2] = {{0, 1}, {1, 2}, {2, 3}, {3, 0}};     // top,right,bottom,left
  const int r = std::max(1, cfg.edge_probe_radius);
  for (int i = 0; i < 4; ++i) {
    int hits = 0;
    score.edge_support[i] = supportAlongEdge(purple_probe, quad[pairs[i][0]], quad[pairs[i][1]],
                                                 cfg.edge_samples, r, hits);
  }
  score.edges_ok = 0;
  double sum = 0;
  for (int i = 0; i < 4; ++i) {
    if (score.edge_support[i] >= cfg.edge_support_min) {++score.edges_ok;}
    sum += score.edge_support[i];
  }
  double s[4];
  std::copy(score.edge_support, score.edge_support + 4, s);
  std::sort(s, s + 4, std::greater<double>());
  const double top2 = 0.5 * (s[0] + s[1]);
  score.top2_min = s[1];
  score.purple = 0.6 * top2 + 0.4 * (sum / 4.0);
}

// 内部颜色：默认红色（蓝色模式沿用 red 名称），配置网格（默认4×2）及总体采样占比。
// 总体比例衡量颜色覆盖率，cells_red 另约束分布；只在一角出现大片颜色不能替代多个合格格子。
double redInsideGrid(
  const cv::Mat & red, const std::vector<cv::Point2f> & quad,
  const FieldConfig & cfg, int & cells_red)
{
  cells_red = 0;
  const int cols = std::max(1, cfg.red_grid_cols), rows = std::max(1, cfg.red_grid_rows);
  const double inset = 0.10;       // 往里收 10%, 避开紫框本身
  int total = 0, hit = 0;
  for (int gy = 0; gy < rows; ++gy) {
    for (int gx = 0; gx < cols; ++gx) {
      int cell_total = 0, cell_hit = 0;
      const int sub = 4;         // 每格 4x4 个采样点
      for (int sy = 0; sy < sub; ++sy) {
        for (int sx = 0; sx < sub; ++sx) {
          const float u = static_cast<float>(inset + (1 - 2 * inset) *
            ((gx + (sx + 0.5f) / sub) / cols));
          const float v = static_cast<float>(inset + (1 - 2 * inset) *
            ((gy + (sy + 0.5f) / sub) / rows));
                    // 双线性插值定位 (四边形参数坐标 -> 图像坐标)
          const cv::Point2f top = quad[0] + (quad[1] - quad[0]) * u;
          const cv::Point2f bot = quad[3] + (quad[2] - quad[3]) * u;
          const cv::Point2f p = top + (bot - top) * v;
          const int x = cvRound(p.x), y = cvRound(p.y);
          if (x < 0 || y < 0 || x >= red.cols || y >= red.rows) {continue;}
          ++cell_total; ++total;
          if (red.at<uchar>(y, x) > 0) {++cell_hit; ++hit;}
        }
      }
      if (cell_total > 0 && static_cast<double>(cell_hit) / cell_total >= cfg.red_cell_min) {
        ++cells_red;
      }
    }
  }
  return total > 0 ? static_cast<double>(hit) / total : 0.0;
}

// 外部颜色: 按相对中心的比例外扩，在框线与外扩线的中线采样；不是固定像素宽的膨胀环。
double redOutsideRing(
  const cv::Mat & red, const std::vector<cv::Point2f> & quad,
  const FieldConfig & cfg)
{
  cv::Point2f c(0, 0);
  for (const auto & p : quad) {
    c += p;
  }
  c *= 0.25f;
  const double k = 1.0 + std::max(0.0, cfg.expand_frac);
  const int per_edge = std::max(4, cfg.edge_samples / 4);
  int total = 0, hit = 0;
  double worst_edge = 0;
  for (int e = 0; e < 4; ++e) {
    int edge_total = 0, edge_hit = 0;
    const cv::Point2f a = quad[e], b = quad[(e + 1) % 4];
    const cv::Point2f a2 = c + (a - c) * static_cast<float>(k);
    const cv::Point2f b2 = c + (b - c) * static_cast<float>(k);
    for (int i = 0; i < per_edge; ++i) {
      const float t = per_edge == 1 ? 0.5f : static_cast<float>(i) / (per_edge - 1);
            // 取框线与外扩线之间的中线
      const cv::Point2f pin = a + (b - a) * t;
      const cv::Point2f pout = a2 + (b2 - a2) * t;
      const cv::Point2f p = (pin + pout) * 0.5f;
      const int x = cvRound(p.x), y = cvRound(p.y);
      if (x < 0 || y < 0 || x >= red.cols || y >= red.rows) {continue;}
      ++total; ++edge_total;
      if (red.at<uchar>(y, x) > 0) {++hit; ++edge_hit;}
    }
    if (edge_total >= per_edge / 2) {
      worst_edge = std::max(worst_edge, double(edge_hit) / edge_total);
    }
  }
  // 单条假边穿过内部颜色也须被拒绝，不能被其余三条干净边的平均分掩盖。
  return total > 0 ? std::max(static_cast<double>(hit) / total, worst_edge / 3.0) : 0.0;
}

// 无紫色支持的边段累积明确的内外同色反证；遮挡不制造反证，也不抹掉段内已经出现的反证。
bool crossesInterior(
  const cv::Mat & interior, const cv::Mat & purple_probe,
  const std::vector<cv::Point2f> & quad, const FieldConfig & cfg)
{
  if (interior.empty() || interior.type() != CV_8U || quad.size() != 4 ||
    purple_probe.size() != interior.size() || purple_probe.type() != CV_8U) {return false;}
  const int samples = std::max(8, cfg.edge_samples);
  cv::Point2f center(0, 0);
  for (const auto & point : quad) {center += point;}
  center *= 0.25f;
  for (int edge = 0; edge < 4; ++edge) {
    int counterevidence = 0;
    const auto & a = quad[edge];
    const auto & b = quad[(edge + 1) % 4];
    const cv::Point2f direction = b - a;
    const double length = cv::norm(direction);
    if (length < 1e-6) {continue;}
    const cv::Point2f normal(-direction.y / length, direction.x / length);
    const float radius = float(std::max(1, cfg.edge_probe_radius));
    const cv::Rect bounds(0, 0, interior.cols, interior.rows);
    for (int i = 0; i < samples; ++i) {
      const float t = float(cfg.corner_exclude_frac +
        (1.0 - 2.0 * cfg.corner_exclude_frac) * (i + 0.5) / samples);
      const cv::Point2f boundary = a + t * (b - a);
      const cv::Point2f delta = (boundary - center) * float(std::max(0.02, cfg.expand_frac) * 0.5);
      const cv::Point pin = boundary - delta, pout = boundary + delta;
      // 紫色支持按本处采样点检查，不能让一端的胶带替中段切进场内的假边作证。
      bool purple = false;
      for (float offset : {0.f, -0.5f, 0.5f, -1.f, 1.f}) {
        const cv::Point sample = boundary + normal * (offset * radius);
        if (bounds.contains(sample) && purple_probe.at<uchar>(sample) != 0) {purple = true; break;}
      }
      // 本处紫边或画外端点终止该段；块体遮挡只是未知，不能把两侧已见的切红证据清零。
      if (purple || !bounds.contains(pin) || !bounds.contains(pout)) {
        counterevidence = 0;
      } else if (interior.at<uchar>(pin) != 0 && interior.at<uchar>(pout) != 0) {
        ++counterevidence;
        if (counterevidence > cfg.red_outside_max * samples) {return true;}
      }
    }
  }
  return false;
}

// 形状合理性 (0..1): 凸性 + 对边比例 + 面积
double geometryScore(
  const std::vector<cv::Point2f> & q, const cv::Size & sz,
  const FieldConfig & cfg)
{
  if (!isConvexQuad(q)) {return 0.0;}
  const double top = dist(q[0], q[1]), bottom = dist(q[3], q[2]);
  const double left = dist(q[0], q[3]), right = dist(q[1], q[2]);
  const double r1 = std::max(top, bottom) / std::max(std::min(top, bottom), 1e-6);
  const double r2 = std::max(left, right) / std::max(std::min(left, right), 1e-6);
  double s = 1.0;
  auto penalty = [&](double r) {
      if (r > cfg.quad_side_ratio_hi) {return 0.0;}
      if (r > cfg.quad_side_ratio_hi * 0.6) {return 0.5;}
      return 1.0;
    };
  s *= penalty(r1);
  s *= penalty(r2);
  const double area = polyArea(q);
  if (area < cfg.quad_min_area_frac * sz.width * sz.height) {s *= 0.3;}
  return s;
}

// 综合评分: total = Σ w_i * score_i
// 权重不做归一化；外侧颜色是风险，因此这里用 1-red_outside 将“越低越好”转成奖励。
void finalizeScore(CandidateScore & sc, const FieldConfig & cfg)
{
  sc.total = cfg.w_purple * sc.purple +
    cfg.w_temporal * sc.temporal +
    cfg.w_red_inside * sc.red_inside +
    cfg.w_red_outside * (1.0 - sc.red_outside) +
    cfg.w_geometry * sc.geometry;
}

// 候选排序、ROI 回退和位置切换共用同一套准入门槛。
// 每项都是独立否决项，较高总分不能抵消内部颜色不足或实际测边过少；返回第一条拒绝理由供诊断。
std::string rejectCandidate(
  const CandidateScore & sc, int measured, const FieldConfig & cfg, bool tracking)
{
  if (sc.crosses_interior) {return "缺少紫边证据的外框切过内部颜色";}
  if (sc.purple < cfg.min_purple_score) {return "紫框边证据不足";}
  if (sc.red_inside < cfg.red_inside_min) {return "内部红色不足";}
  if (sc.cells_red < cfg.red_cells_min) {return "内部红色格数不足";}
  if (sc.red_outside > cfg.red_outside_max) {return "四边形外部也是红色 (框到场地内部了)";}
  if (sc.edges_ok < 2) {return "紫色边支持度合格的边少于 2 条";}
  if (sc.edges_ok == 2 && sc.top2_min < cfg.edge_support_strong &&
    !(tracking && sc.temporal >= 0.9)) {return "只有 2 条边有证据, 且强度/时序一致性不足";}
  if (sc.total < cfg.min_total_score) {return "总分过低";}
  if (measured <= 1) {return "只有 1 条边测到";}
  return std::string();
}

}  // namespace field_detail

// ---- 鸟瞰黑带证据与左右分区 ----
// 黑带按“相对两侧内部颜色的局部色度下陷”测量：反光或进光会把黑带提亮到接近内部亮度，
// 但不会给它红/蓝色度，因此判据只和同一行紧邻两侧比较，不依赖全局灰度阈值或绝对亮度。
// 至少一侧紧邻是内部颜色才有资格成为证据，紫色围边立面和场外自然排除；物块贴在一侧时用另一侧作参照。
// 宽于带宽上限的暗区（暗物块）按完整无色区测宽后丢弃。两侧都是内部颜色且本列也是内部颜色的位置，
// 才算“应见黑带却见红色”的反证；被物块盖住的行只缩小分母。跨行用投票 + 稳健直线拟合，
// 允许局部反光/遮挡行缺失，但覆盖率、首末分段、残差和选优歧义仍须满足。
namespace field_detail {
namespace {

// 黑带测量的算法常量集中在此；可调的物理量（深度、覆盖率、带宽、亮度抬升）在 FieldConfig。
// 参照段中内部颜色占比下限：允许参照段边缘一两列被噪声或混色占掉。
constexpr double kReferenceShare = 0.8;
// 放宽参照资格：与本帧内部平均色相夹角 ≤ 约 35°（cos 0.82），相对色度达到内部下限的 1/4。
constexpr double kReferenceHueCos = 0.82;
constexpr double kReferenceChromaShare = 0.25;
// 本帧内部颜色像素少于窗口 2% 时没有可靠参照色相。
constexpr double kMinReferencePixels = 0.02;
// 一行内部颜色达到搜索窗宽 1/4 才算“内部行”，用于置信度中的遮挡比例。
constexpr double kInteriorRowShare = 0.25;
// 验收所需的最少行数：BEV 高度的 12%，且不少于 8 行。
constexpr double kMinRowsFrac = 0.12;
constexpr int kMinRowsFloor = 8;
// 投票斜率网格：BEV 已按外框矫正，黑带斜率在 ±0.25 内；21 格步长 0.025，在内部行内约 1 px。
constexpr int kSlopeBins = 21;
constexpr double kMaxSlope = 0.25;
// 段中心离候选线的容差，覆盖斜率步长带来的约 1 px 偏差。
constexpr double kLineTolerancePx = 2.0;
// 最多依次验收的投票峰数；次优达到最优该比例时两条都像分界，判歧义。
constexpr int kPeakAttempts = 4;
constexpr double kAmbiguityRatio = 0.8;
// 残差验收：90% 分位不超过 max(1.5 px, 带宽中位数的一半)。
constexpr double kResidualQuantile = 0.9;
constexpr double kResidualFloorPx = 1.5;
constexpr double kResidualWidthShare = 0.5;
// 沿线支持段的宽度须在中位宽度的 [1/1.5, 1.5] 倍内。
constexpr double kWidthSpread = 1.5;
// 无色区两端外侧在该像素数内须出现内部颜色：紧贴物块的黑带被盖住一部分，中心有偏，按未知处理。
constexpr int kBandEdgeProbePx = 2;
// 八位 Lab 的 L8 = L×2.55，a、b 偏移 128。
constexpr double kLightTo8bit = 2.55;
constexpr int kLabOffset = 128;

/** @brief 单行内一段连续色度下陷，坐标是搜索窗内列号。 */
struct SplitRun {int y; float x, width, depth;};

/** @brief 候选直线 x = top + slope * y（搜索窗内坐标）的验收结果。 */
struct SplitLine {bool valid = false; double top = 0, slope = 0, confidence = 0;};

/** @brief 搜索窗与带参照余量的上下文窗（BEV 坐标），half 是带宽上限的一半。 */
struct SplitWindow {cv::Rect window, context; int half = 0, pad_lo = 0;};

/** @brief 上下文窗内的逐像素证据：纵向平滑后的色度与亮度、放宽后的内部颜色（参照资格）。 */
struct SplitEvidence {cv::Mat chroma, light, reference;};

bool splitWindow(const cv::Mat & bev, const FieldConfig & cfg, SplitWindow & out)
{
  const int bw = bev.cols, bh = bev.rows;
  const int x0 = int(cfg.black_search_lo * bw), x1 = int(cfg.black_search_hi * bw);
  const int y0 = int(cfg.black_row_lo * bh), y1 = int(cfg.black_row_hi * bh);
  // 带宽上限的一半同时决定两侧参照段的偏移与长度：超宽暗区的参照段落在暗区内，不形成下陷。
  out.half = std::max(2, cvRound(0.5 * cfg.black_max_width_frac * bw));
  if (bev.empty() || bev.type() != CV_8UC3 || x1 - x0 < 4 * out.half + 8 || y1 - y0 < 8) {return false;}
  out.window = cv::Rect(x0, y0, x1 - x0, y1 - y0);
  // 参照段可伸出搜索窗，但不超出 BEV。
  out.pad_lo = std::min(x0, 2 * out.half);
  const int pad_hi = std::min(bw - x1, 2 * out.half);
  out.context = cv::Rect(x0 - out.pad_lo, y0, out.window.width + out.pad_lo + pad_hi, out.window.height);
  return true;
}

// 本帧严格内部颜色像素的平均 (a,b) 方向作参照色相，随整体光照偏色自适应。
bool referenceHue(const cv::Mat & lab, const cv::Mat & interior, cv::Vec2d & hue)
{
  double sum_a = 0, sum_b = 0;
  int count = 0;
  for (int y = 0; y < lab.rows; ++y) {
    const auto * pixel = lab.ptr<uchar>(y);
    const auto * in = interior.ptr<uchar>(y);
    for (int x = 0; x < lab.cols; ++x, pixel += 3) {
      if (in[x] != 0) {sum_a += pixel[1] - kLabOffset; sum_b += pixel[2] - kLabOffset; ++count;}
    }
  }
  const double norm = std::hypot(sum_a, sum_b);
  if (count < kMinReferencePixels * lab.total() || norm < 1e-6) {return false;}
  hue = cv::Vec2d(sum_a / norm, sum_b / norm);
  return true;
}

// 参照资格：严格内部颜色，或非紫色、相对色度达到内部下限的 1/4 且色相接近本帧参照色相。
// 局部反光/阴影使红色略出扇区时仍可作参照；紫色围边、低色度地面与黑带、其他颜色物块不能。
bool splitEvidence(const cv::Mat & bgr, const FieldConfig & cfg, SplitEvidence & out)
{
  cv::Mat lab, purple;
  cv::cvtColor(bgr, lab, cv::COLOR_BGR2Lab);
  FieldLocator::segment(bgr, cfg, purple, out.reference);
  cv::Vec2d hue;
  if (!referenceHue(lab, out.reference, hue)) {return false;}
  const double chroma_per_light = kReferenceChromaShare / kLightTo8bit *
    (cfg.use_blue_interior ? cfg.blue_chroma_ratio_min : cfg.red_chroma_ratio_min);
  out.chroma.create(lab.size(), CV_32F);
  out.light.create(lab.size(), CV_32F);
  for (int y = 0; y < lab.rows; ++y) {
    const auto * pixel = lab.ptr<uchar>(y);
    const auto * violet = purple.ptr<uchar>(y);
    auto * reference = out.reference.ptr<uchar>(y);
    auto * c = out.chroma.ptr<float>(y);
    auto * l = out.light.ptr<float>(y);
    for (int x = 0; x < lab.cols; ++x, pixel += 3) {
      const int a = pixel[1] - kLabOffset, b = pixel[2] - kLabOffset;
      l[x] = pixel[0];
      c[x] = float(std::abs(a) + std::abs(b));
      if (violet[x] == 0 && c[x] >= chroma_per_light * pixel[0] &&
        a * hue[0] + b * hue[1] >= kReferenceHueCos * std::hypot(a, b)) {reference[x] = 255;}
    }
  }
  // 只沿黑带方向（纵向）平滑压缩噪声，横向不扩散，带宽保持实测。
  cv::blur(out.chroma, out.chroma, cv::Size(1, 3));
  cv::blur(out.light, out.light, cv::Size(1, 3));
  return true;
}

/** @brief 一行的前缀和，参照段均值 O(1)。 */
struct RowSums
{
  RowSums(const float * chroma, const float * light, const uchar * reference, int width)
  : c(width + 1), l(width + 1), r(width + 1)
  {
    for (int x = 0; x < width; ++x) {
      c[x + 1] = c[x] + chroma[x]; l[x + 1] = l[x] + light[x]; r[x + 1] = r[x] + (reference[x] != 0);
    }
  }
  static double mean(const std::vector<double> & sum, int a, int b) {return (sum[b] - sum[a]) / (b - a);}
  std::vector<double> c, l, r;
};

/** @brief 单列相对参照的色度下陷；two_sided 表示两侧参照都合格。 */
struct ColumnDip {double depth = 0, reference = 0; bool two_sided = false;};

// 参照段 [cx-2h, cx-h) 与 (cx+h, cx+2h]，各长 h 像素。合格的一侧或两侧给出参照色度（两侧取小）与亮度上限；
// 高光条带也会失去色度，但亮度明显高于参照，超过 (1+black_max_lift) 倍不算下陷。
ColumnDip columnDip(
  const RowSums & sums, const float * chroma, const float * light, int cx, int half,
  const FieldConfig & cfg)
{
  ColumnDip dip;
  const int la = cx - 2 * half, lb = cx - half, ra = cx + half + 1, rb = cx + 2 * half + 1;
  const bool left = RowSums::mean(sums.r, la, lb) >= kReferenceShare;
  const bool right = RowSums::mean(sums.r, ra, rb) >= kReferenceShare;
  if (!left && !right) {return dip;}
  dip.two_sided = left && right;
  const double cl = RowSums::mean(sums.c, la, lb), cr = RowSums::mean(sums.c, ra, rb);
  const double ll = RowSums::mean(sums.l, la, lb), lr = RowSums::mean(sums.l, ra, rb);
  const double reference = dip.two_sided ? std::min(cl, cr) : (left ? cl : cr);
  const double brightest = dip.two_sided ? std::max(ll, lr) : (left ? ll : lr);
  if (reference > 1.0 && light[cx] <= brightest * (1.0 + cfg.black_max_lift)) {
    dip.depth = 1.0 - chroma[cx] / reference;
    dip.reference = reference;
  }
  return dip;
}

// 无色区 [left,right) 两端外侧 kBandEdgeProbePx 内都有内部颜色资格像素。
bool bandEdgesVisible(const uchar * reference, int left, int right, int context_width)
{
  auto any = [&](int from, int step) {
      for (int k = 0; k < kBandEdgeProbePx; ++k) {
        const int x = from + step * k;
        if (x >= 0 && x < context_width && reference[x] != 0) {return true;}
      }
      return false;
    };
  return any(left - 1, -1) && any(right, 1);
}

// 连续下陷段 [start,end)（上下文列号）收尾：宽度按同一色度门槛向两侧延伸的完整无色区测量，
// 不受参照资格截断，宽暗物块中心恰有资格的一两列不能冒充窄黑带；两端须紧邻内部颜色，
// 被物块盖住一侧的黑带中心有偏，不作证据；中心按无色区内的下陷加权。
bool measureRun(
  const float * chroma, const uchar * reference_mask, int width_limit, int context_width,
  int start, int end, double reference, double depth, const FieldConfig & cfg, int y, int pad_lo,
  SplitRun & run)
{
  const double limit = (1.0 - cfg.black_min_depth) * reference;
  int left = start, right = end;
  while (left > 0 && chroma[left - 1] <= limit && right - left <= width_limit) {--left;}
  while (right < context_width && chroma[right] <= limit && right - left <= width_limit) {++right;}
  if (right - left > width_limit || !bandEdgesVisible(reference_mask, left, right, context_width)) {
    return false;
  }
  double weight = 0, moment = 0;
  for (int x = left; x < right; ++x) {
    const double w = std::max(0.0, reference - chroma[x]);
    weight += w; moment += w * x;
  }
  if (weight <= 0) {return false;}
  run = {y, float(moment / weight - pad_lo), float(right - left), float(depth)};
  return true;
}

// 一行内的下陷段写入 runs；两侧参照都合格且本列也有内部颜色资格的位置记入 eligible（可作反证）。
void rowRuns(
  const SplitEvidence & evidence, const SplitWindow & window, const FieldConfig & cfg, int y,
  std::vector<SplitRun> & runs, uchar * eligible)
{
  const auto * chroma = evidence.chroma.ptr<float>(y);
  const auto * light = evidence.light.ptr<float>(y);
  const auto * reference = evidence.reference.ptr<uchar>(y);
  const int width = window.context.width, half = window.half, pad_lo = window.pad_lo;
  const RowSums sums(chroma, light, reference, width);
  int start = -1;
  double depth_sum = 0, reference_sum = 0;
  auto close = [&](int end) {
      SplitRun run;
      const int count = end - start;
      if (measureRun(chroma, reference, 2 * half, width, start + pad_lo, end + pad_lo,
        reference_sum / count, depth_sum / count, cfg, y, pad_lo, run)) {runs.push_back(run);}
      start = -1; depth_sum = reference_sum = 0;
    };
  for (int x = 0; x < window.window.width; ++x) {
    const int cx = x + pad_lo;
    ColumnDip dip;
    if (cx - 2 * half >= 0 && cx + 2 * half + 1 <= width) {
      dip = columnDip(sums, chroma, light, cx, half, cfg);
      if (dip.two_sided && reference[cx] != 0) {eligible[x] = 255;}
    }
    if (dip.depth >= cfg.black_min_depth) {
      if (start < 0) {start = x;}
      depth_sum += dip.depth; reference_sum += dip.reference;
    } else if (start >= 0) {
      close(x);
    }
  }
  if (start >= 0) {close(window.window.width);}
}

int interiorRows(const cv::Mat & interior, const SplitWindow & window)
{
  const int cols = window.window.width;
  int rows = 0;
  for (int y = 0; y < interior.rows; ++y) {
    if (cv::countNonZero(interior.row(y).colRange(window.pad_lo, window.pad_lo + cols)) >=
      kInteriorRowShare * cols) {++rows;}
  }
  return rows;
}

/** @brief 统计直线沿线的资格行与支持行，检查覆盖、分段、残差，并用 Huber 拟合精修。 */
SplitLine acceptLine(
  const std::vector<SplitRun> & runs, const cv::Mat & eligible, double top, double slope,
  const FieldConfig & cfg, int min_rows, int interior_rows)
{
  SplitLine result;
  const int rows = eligible.rows, cols = eligible.cols;
  // 每行最多取一个离线最近的段；同一行多段只算一次支持。
  std::vector<int> best(rows, -1);
  std::vector<double> best_distance(rows, kLineTolerancePx);
  for (int i = 0; i < static_cast<int>(runs.size()); ++i) {
    const auto & run = runs[i];
    const double distance = std::abs(run.x - (top + slope * run.y));
    if (distance <= best_distance[run.y]) {
      best[run.y] = i; best_distance[run.y] = distance;
    }
  }
  std::vector<int> eligible_rows;
  for (int y = 0; y < rows; ++y) {
    const int x = cvRound(top + slope * y);
    if (best[y] >= 0 || (x >= 0 && x < cols && eligible.at<uchar>(y, x) != 0)) {eligible_rows.push_back(y);}
  }
  if (static_cast<int>(eligible_rows.size()) < min_rows) {return result;}
  // 黑带在 BEV 中宽度恒定：宽度偏离沿线中位数超过 kWidthSpread 倍的段是贴线物块或噪声，不算支持。
  std::vector<float> line_widths;
  for (int y : eligible_rows) {
    if (best[y] >= 0) {line_widths.push_back(runs[best[y]].width);}
  }
  if (line_widths.empty()) {return result;}
  const double typical_width = upperMedian(line_widths);
  auto consistent = [&](int y) {
      if (best[y] < 0) {return false;}
      const double width = runs[best[y]].width;
      return width <= kWidthSpread * typical_width && width * kWidthSpread >= typical_width;
    };
  std::vector<cv::Point2f> centers;
  std::vector<double> widths;
  double depth_sum = 0;
  const int first = eligible_rows.front(), span = eligible_rows.back() - first + 1;
  int thirds_total[3] = {}, thirds_hit[3] = {};
  for (int y : eligible_rows) {
    const int third = std::min(2, (y - first) * 3 / span);
    ++thirds_total[third];
    if (!consistent(y)) {continue;}
    ++thirds_hit[third];
    const auto & run = runs[best[y]];
    centers.emplace_back(run.x, float(y)); widths.push_back(run.width); depth_sum += run.depth;
  }
  const double coverage = double(centers.size()) / eligible_rows.size();
  if (coverage < cfg.black_score_frac) {return result;}
  // 支持行的首末跨度也须达到内部行的同一比例：物块盖住一端时，剩下的短暗条与场内暗物体无法区分。
  const double support_span = centers.back().y - centers.front().y + 1;
  if (support_span < cfg.black_score_frac * interior_rows) {return result;}
  // 首末两段必须有支持，中段被物块整段遮住时可以没有资格行；有资格行的分段都要达到覆盖率的一半。
  // 局部反光或物块可以缺行，但只在一端出现的短暗物体不能替代完整分界。
  if (thirds_hit[0] == 0 || thirds_hit[2] == 0) {return result;}
  for (int i = 0; i < 3; ++i) {
    if (thirds_hit[i] < 0.5 * cfg.black_score_frac * thirds_total[i]) {return result;}
  }
  cv::Vec4f line;
  cv::fitLine(centers, line, cv::DIST_HUBER, 0, 0.01, 0.01);
  if (!std::isfinite(line[1]) || std::abs(line[1]) < 1e-6) {return result;}
  result.slope = double(line[0]) / line[1];
  result.top = line[2] - result.slope * line[3];
  std::vector<double> residuals;
  residuals.reserve(centers.size());
  for (const auto & point : centers) {residuals.push_back(std::abs(point.x - result.top - result.slope * point.y));}
  const auto tail = residuals.begin() + static_cast<long>(residuals.size() * kResidualQuantile);
  std::nth_element(residuals.begin(), tail, residuals.end());
  if (*tail > std::max(kResidualFloorPx, upperMedian(widths) * kResidualWidthShare)) {return result;}
  const double depth = depth_sum / centers.size();
  result.valid = true;
  // 置信度 = 下陷强度 × 资格行覆盖率 × 资格行占内部行比例：被遮挡越多、证据越短，置信度越低，
  // 选优时完整分界线优先于只在物块高度内成立的短暗边。
  result.confidence = std::min(1.0, depth / cfg.black_conf_depth) * coverage *
    std::min(1.0, double(eligible_rows.size()) / std::max(1, interior_rows));
  return result;
}

// 投票：每段对 (中点列, 斜率) 网格投一票；依次取峰值验收，已验收线附近 2h 内的峰被抑制。
std::vector<SplitLine> candidateLines(
  const std::vector<SplitRun> & runs, const cv::Mat & eligible, int half, const FieldConfig & cfg,
  int min_rows, int interior_rows)
{
  const int rows = eligible.rows, cols = eligible.cols;
  const double middle = 0.5 * (rows - 1);
  auto slopeOf = [](int bin) {return kMaxSlope * (2.0 * bin / (kSlopeBins - 1) - 1.0);};
  cv::Mat votes = cv::Mat::zeros(kSlopeBins, cols, CV_32F);
  for (const auto & run : runs) {
    for (int k = 0; k < kSlopeBins; ++k) {
      const int x = cvRound(run.x - slopeOf(k) * (run.y - middle));
      if (x >= 0 && x < cols) {votes.at<float>(k, x) += 1.0f;}
    }
  }
  cv::blur(votes, votes, cv::Size(3, 1), cv::Point(-1, -1), cv::BORDER_CONSTANT);
  std::vector<SplitLine> accepted;
  for (int attempt = 0; attempt < kPeakAttempts; ++attempt) {
    cv::Point peak;
    double peak_votes = 0;
    cv::minMaxLoc(votes, nullptr, &peak_votes, nullptr, &peak);
    // 峰值票数低于“三段之一达到覆盖率”所需的行数时，后面的峰只会更弱。
    if (peak_votes * 3.0 < min_rows * cfg.black_score_frac) {break;}
    const double slope = slopeOf(peak.y);
    const auto line = acceptLine(runs, eligible, peak.x - slope * middle, slope, cfg, min_rows,
      interior_rows);
    const double center = line.valid ? line.top + line.slope * middle : double(peak.x);
    votes.colRange(std::max(0, cvRound(center) - 2 * half),
      std::min(cols, cvRound(center) + 2 * half + 1)).setTo(0);
    if (line.valid) {accepted.push_back(line);}
  }
  return accepted;
}

}  // namespace

SplitMeasurement detectSplitBlack(const cv::Mat & bev, const FieldConfig & cfg)
{
  SplitWindow window;
  if (!splitWindow(bev, cfg, window)) {return {};}
  SplitEvidence evidence;
  if (!splitEvidence(bev(window.context), cfg, evidence)) {return {};}
  cv::Mat eligible = cv::Mat::zeros(window.window.size(), CV_8U);
  std::vector<SplitRun> runs;
  for (int y = 0; y < window.window.height; ++y) {
    rowRuns(evidence, window, cfg, y, runs, eligible.ptr<uchar>(y));
  }
  const int min_rows = std::max(kMinRowsFloor, static_cast<int>(std::ceil(kMinRowsFrac * bev.rows)));
  if (static_cast<int>(runs.size()) < min_rows) {return {};}
  auto lines = candidateLines(runs, eligible, window.half, cfg, min_rows,
    interiorRows(evidence.reference, window));
  if (lines.empty()) {return {};}
  std::sort(lines.begin(), lines.end(),
    [](const SplitLine & a, const SplitLine & b) {return a.confidence > b.confidence;});
  if (lines.size() > 1 && lines[1].confidence >= kAmbiguityRatio * lines[0].confidence) {return {};}
  const auto & line = lines.front();
  // 窗内直线外推到 BEV 顶/底边，端点须落在鸟瞰图内。
  const int x0 = window.window.x, y0 = window.window.y;
  SplitMeasurement measurement;
  measurement.top_x = x0 + line.top - line.slope * y0;
  measurement.bottom_x = measurement.top_x + line.slope * (bev.rows - 1);
  if (!std::isfinite(measurement.top_x) || !std::isfinite(measurement.bottom_x) ||
    measurement.top_x < 0 || measurement.bottom_x < 0 ||
    measurement.top_x > bev.cols - 1 || measurement.bottom_x > bev.cols - 1) {return {};}
  measurement.valid = true;
  measurement.confidence = line.confidence;
  return measurement;
}

}  // namespace field_detail
