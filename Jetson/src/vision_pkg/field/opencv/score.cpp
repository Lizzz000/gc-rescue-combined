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
// 只有两侧紧邻都是内部颜色的像素才有资格成为证据，紫色围边立面和场外自然排除；
// 物块遮住黑带或贴在一侧时，该行两侧不全是内部颜色，不计入资格行，只缩小分母而不制造反证；
// 宽于带宽上限的暗物块内部没有两侧参照，不形成下陷。跨行用投票+稳健直线拟合，
// 允许局部反光/遮挡行缺失，但覆盖率、首末分段、残差和选优歧义仍须满足。
namespace field_detail {
namespace {

/** @brief 单行内一段连续色度下陷，坐标是搜索窗内列号。 */
struct SplitRun {int y; float x, width, depth;};

/** @brief 候选直线 x = top + slope * y（搜索窗内坐标）的验收结果。 */
struct SplitLine {bool valid = false; double top = 0, slope = 0, confidence = 0;};

/** @brief 统计直线沿线的资格行与支持行，检查覆盖、分段、残差，并用 Huber 拟合精修。 */
SplitLine acceptLine(
  const std::vector<SplitRun> & runs, const cv::Mat & eligible, double top, double slope,
  double tolerance, const FieldConfig & cfg, int min_rows, int interior_rows)
{
  SplitLine result;
  const int rows = eligible.rows, cols = eligible.cols;
  // 每行最多取一个离线最近的段；同一行多段只算一次支持。
  std::vector<int> best(rows, -1);
  std::vector<double> best_distance(rows, tolerance);
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
  std::vector<cv::Point2f> centers;
  std::vector<double> widths;
  double depth_sum = 0;
  const int first = eligible_rows.front(), span = eligible_rows.back() - first + 1;
  int thirds_total[3] = {}, thirds_hit[3] = {};
  for (int y : eligible_rows) {
    const int third = std::min(2, (y - first) * 3 / span);
    ++thirds_total[third];
    if (best[y] < 0) {continue;}
    ++thirds_hit[third];
    const auto & run = runs[best[y]];
    centers.emplace_back(run.x, float(y)); widths.push_back(run.width); depth_sum += run.depth;
  }
  const double coverage = double(centers.size()) / eligible_rows.size();
  if (coverage < cfg.black_score_frac) {return result;}
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
  const auto tail = residuals.begin() + residuals.size() * 9 / 10;
  std::nth_element(residuals.begin(), tail, residuals.end());
  if (*tail > std::max(1.5, upperMedian(widths) * 0.5)) {return result;}
  const double depth = depth_sum / centers.size();
  result.valid = true;
  // 置信度 = 下陷强度 × 资格行覆盖率 × 资格行占内部行比例：被遮挡越多、证据越短，置信度越低，
  // 选优时完整分界线优先于只在物块高度内成立的短暗边。
  result.confidence = std::min(1.0, depth / cfg.black_conf_depth) * coverage *
    std::min(1.0, double(eligible_rows.size()) / std::max(1, interior_rows));
  return result;
}

}  // namespace

SplitMeasurement detectSplitBlack(const cv::Mat & bev, const FieldConfig & cfg)
{
  const int bw = bev.cols, bh = bev.rows;
  // 纯核心调用也须拒绝无效采样范围，不能把非法浮点转换为 ROI 整数。
  if (!std::isfinite(cfg.black_search_lo) || !std::isfinite(cfg.black_search_hi) ||
    !std::isfinite(cfg.black_row_lo) || !std::isfinite(cfg.black_row_hi) ||
    cfg.black_search_lo < 0 || cfg.black_search_hi > 1 ||
    cfg.black_row_lo < 0 || cfg.black_row_hi > 1 ||
    cfg.black_search_lo >= cfg.black_search_hi || cfg.black_row_lo >= cfg.black_row_hi) {return {};}
  const int x0 = int(cfg.black_search_lo * bw), x1 = int(cfg.black_search_hi * bw);
  const int y0 = int(cfg.black_row_lo * bh), y1 = int(cfg.black_row_hi * bh);
  // 带宽上限的一半同时决定两侧参照段的偏移与长度：超宽暗区的参照段落在暗区内，不形成下陷。
  const int half = std::max(2, cvRound(0.5 * cfg.black_max_width_frac * bw));
  if (bev.empty() || bev.type() != CV_8UC3 || x1 - x0 < 4 * half + 8 || y1 - y0 < 8) {return {};}
  const cv::Rect window(x0, y0, x1 - x0, y1 - y0);
  const int rows = window.height, cols = window.width;
  // 两侧参照段须属于内部颜色；参照段可伸出搜索窗，但不超出 BEV。
  const int pad_lo = std::min(x0, 2 * half), pad_hi = std::min(bw - x1, 2 * half);
  const cv::Rect context(x0 - pad_lo, y0, cols + pad_lo + pad_hi, rows);
  cv::Mat lab;
  cv::cvtColor(bev(context), lab, cv::COLOR_BGR2Lab);
  cv::Mat purple, interior;
  FieldLocator::segment(bev(context), cfg, purple, interior);
  // 本帧内部颜色的平均色相 (a,b) 作参照：严格阈值内的像素决定场地本色，随整体光照偏色自适应。
  double ref_a = 0, ref_b = 0, ref_n = 0;
  for (int y = 0; y < rows; ++y) {
    const auto * pixel = lab.ptr<uchar>(y);
    const auto * in = interior.ptr<uchar>(y);
    for (int x = 0; x < context.width; ++x, pixel += 3) {
      if (in[x] != 0) {ref_a += pixel[1] - 128; ref_b += pixel[2] - 128; ++ref_n;}
    }
  }
  if (ref_n < 0.02 * context.area()) {return {};}
  const double ref_norm = std::hypot(ref_a, ref_b);
  if (ref_norm < 1e-6) {return {};}
  ref_a /= ref_norm; ref_b /= ref_norm;
  // 参照段资格：严格内部颜色，或非紫色、相对色度达到内部下限的 1/4 且色相与本色夹角在约 35° 内。
  // 局部反光/阴影使红色略出严格阈值时仍可作参照；紫色围边、低色度地面与黑带、其他颜色物块不能。
  // 八位 L 换回 0..100 后乘相对色度下限。
  const double chroma_per_light = 0.25 / 2.55 *
    (cfg.use_blue_interior ? cfg.blue_chroma_ratio_min : cfg.red_chroma_ratio_min);
  constexpr double hue_cos = 0.82;
  cv::Mat chroma(context.size(), CV_32F), light(context.size(), CV_32F);
  for (int y = 0; y < rows; ++y) {
    const auto * pixel = lab.ptr<uchar>(y);
    const auto * violet = purple.ptr<uchar>(y);
    auto * in = interior.ptr<uchar>(y);
    auto * c = chroma.ptr<float>(y);
    auto * l = light.ptr<float>(y);
    for (int x = 0; x < context.width; ++x, pixel += 3) {
      l[x] = pixel[0];
      c[x] = float(std::abs(pixel[1] - 128) + std::abs(pixel[2] - 128));
      const int a = pixel[1] - 128, b = pixel[2] - 128;
      if (violet[x] == 0 && c[x] >= chroma_per_light * pixel[0] && a * ref_a + b * ref_b >= hue_cos * std::hypot(a, b)) {in[x] = 255;}
    }
  }
  // 只沿黑带方向（纵向）平滑压缩噪声，横向不扩散，带宽保持实测。
  cv::blur(chroma, chroma, cv::Size(1, 3));
  cv::blur(light, light, cv::Size(1, 3));
  cv::Mat eligible = cv::Mat::zeros(rows, cols, CV_8U);
  std::vector<SplitRun> runs;
  const int n = context.width + 1;
  std::vector<double> sum_c(n), sum_l(n), sum_i(n);
  for (int y = 0; y < rows; ++y) {
    const auto * c = chroma.ptr<float>(y);
    const auto * l = light.ptr<float>(y);
    const auto * in = interior.ptr<uchar>(y);
    for (int x = 0; x < context.width; ++x) {
      sum_c[x + 1] = sum_c[x] + c[x]; sum_l[x + 1] = sum_l[x] + l[x]; sum_i[x + 1] = sum_i[x] + (in[x] != 0);
    }
    // 参照段 [x-2h, x-h) 与 (x+h, x+2h]，各长 h 像素。
    auto mean = [&](const std::vector<double> & sum, int a, int b) {return (sum[b] - sum[a]) / (b - a);};
    auto * ok = eligible.ptr<uchar>(y);
    int run_start = -1;
    double depth_sum = 0, reference_sum = 0;
    auto close = [&](int end) {
        // 宽度按同一色度门槛向两侧延伸的完整无色区测量，不受参照资格截断：
        // 宽暗物块中心恰有资格的一两列不能冒充窄黑带。
        const double limit = (1.0 - cfg.black_min_depth) * reference_sum / (end - run_start);
        int left = run_start + pad_lo, right = end + pad_lo;
        while (left > 0 && c[left - 1] <= limit && right - left <= 2 * half) {--left;}
        while (right < context.width && c[right] <= limit && right - left <= 2 * half) {++right;}
        const int width = right - left;
        if (width <= 2 * half) {
          // 中心按完整无色区内的色度下陷加权：参照资格被旁边物块截断时，只用资格像素会偏向一侧。
          const double reference = reference_sum / (end - run_start);
          double weight = 0, moment = 0;
          for (int x = left; x < right; ++x) {
            const double w = std::max(0.0, reference - c[x]);
            weight += w; moment += w * x;
          }
          if (weight > 0) {
            runs.push_back({y, float(moment / weight - pad_lo), float(width), float(depth_sum / (end - run_start))});
          }
        }
        run_start = -1; depth_sum = reference_sum = 0;
      };
    for (int x = 0; x < cols; ++x) {
      const int cx = x + pad_lo;
      double depth = 0, current_reference = 0;
      if (cx - 2 * half >= 0 && cx + 2 * half + 1 <= context.width) {
        const int la = cx - 2 * half, lb = cx - half, ra = cx + half + 1, rb = cx + 2 * half + 1;
        // 两侧参照段须绝大部分是内部颜色；物块压线或贴边时该行不计入资格行，只缩小分母。
        const double cl = mean(sum_c, la, lb), cr = mean(sum_c, ra, rb);
        if (mean(sum_i, la, lb) >= 0.8 && mean(sum_i, ra, rb) >= 0.8) {
          // 本列本身也是内部颜色才算“应见黑带却没见到”的反证；被非内部色物块盖住只是未知。
          if (in[cx] != 0) {ok[x] = 255;}
          const double reference = std::min(cl, cr);
          const double lift = std::max(mean(sum_l, la, lb), mean(sum_l, ra, rb)) * (1.0 + cfg.black_max_lift);
          // 高光条带也会失去色度，但亮度明显高于两侧；黑带被提亮也不应超过两侧太多。
          if (reference > 1.0 && l[cx] <= lift) {depth = 1.0 - c[cx] / reference; current_reference = reference;}
        }
      }
      if (depth >= cfg.black_min_depth) {
        if (run_start < 0) {run_start = x;}
        depth_sum += depth; reference_sum += current_reference;
      } else if (run_start >= 0) {
        close(x);
      }
    }
    if (run_start >= 0) {close(cols);}
  }
  const int min_rows = std::max(8, static_cast<int>(std::ceil(0.12 * bh)));
  int interior_rows = 0;
  for (int y = 0; y < rows; ++y) {
    if (cv::countNonZero(interior.row(y).colRange(pad_lo, pad_lo + cols)) >= 0.25 * cols) {++interior_rows;}
  }
  if (static_cast<int>(runs.size()) < min_rows) {return {};}
  // 投票：每段对 (中点列, 斜率) 网格投一票，斜率限制在 ±0.25 以内（BEV 已按外框矫正）。
  constexpr int slope_bins = 21;
  constexpr double max_slope = 0.25;
  const double middle = 0.5 * (rows - 1);
  cv::Mat votes = cv::Mat::zeros(slope_bins, cols, CV_32F);
  for (const auto & run : runs) {
    for (int k = 0; k < slope_bins; ++k) {
      const double slope = max_slope * (2.0 * k / (slope_bins - 1) - 1.0);
      const int x = cvRound(run.x - slope * (run.y - middle));
      if (x >= 0 && x < cols) {votes.at<float>(k, x) += 1.0f;}
    }
  }
  cv::blur(votes, votes, cv::Size(3, 1), cv::Point(-1, -1), cv::BORDER_CONSTANT);
  // 段中心离直线的容差：投票斜率步长 0.025 在内部行内最多带来约 1px 偏差。
  const double tolerance = 2.0;
  // 依次取峰值验收，已验收线附近 2h 内的峰被抑制；按置信度选优。
  // 次优线置信度达到最优的 0.8 倍时两条都像分界，视为歧义拒绝；明显弱的暗物体边缘不影响选优。
  std::vector<SplitLine> accepted;
  for (int attempt = 0; attempt < 4; ++attempt) {
    cv::Point peak;
    double peak_votes = 0;
    cv::minMaxLoc(votes, nullptr, &peak_votes, nullptr, &peak);
    if (peak_votes * 3.0 < min_rows * cfg.black_score_frac) {break;}
    const double slope = max_slope * (2.0 * peak.y / (slope_bins - 1) - 1.0);
    const double top = peak.x - slope * middle;
    const auto line = acceptLine(runs, eligible, top, slope, tolerance, cfg, min_rows, interior_rows);
    const double center = line.valid ? line.top + line.slope * middle : double(peak.x);
    cv::Mat suppressed = votes.colRange(std::max(0, cvRound(center) - 2 * half),
      std::min(cols, cvRound(center) + 2 * half + 1));
    suppressed.setTo(0);
    if (line.valid) {accepted.push_back(line);}
  }
  if (accepted.empty()) {return {};}
  std::sort(accepted.begin(), accepted.end(),
    [](const SplitLine & a, const SplitLine & b) {return a.confidence > b.confidence;});
  if (accepted.size() > 1 && accepted[1].confidence >= 0.8 * accepted[0].confidence) {return {};}
  const auto & line = accepted.front();
  // 窗内直线外推到 BEV 顶/底边，端点须落在鸟瞰图内。
  SplitMeasurement measurement;
  measurement.top_x = x0 + line.top - line.slope * y0;
  measurement.bottom_x = measurement.top_x + line.slope * (bh - 1);
  if (!std::isfinite(measurement.top_x) || !std::isfinite(measurement.bottom_x) ||
    measurement.top_x < 0 || measurement.bottom_x < 0 ||
    measurement.top_x > bw - 1 || measurement.bottom_x > bw - 1) {return {};}
  measurement.valid = true;
  measurement.confidence = line.confidence;
  return measurement;
}

}  // namespace field_detail
