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
// quad 是内沿地面四边形；网格按地面平面射影均分，远处的格子不会因透视被压扁。
// 总体比例衡量颜色覆盖率，cells_red 另约束分布；只在一角出现大片颜色不能替代多个合格格子。
double redInsideGrid(
  const cv::Mat & red, const cv::Mat & purple, const std::vector<cv::Point2f> & quad,
  const FieldConfig & cfg, int & cells_red, const cv::Mat & lab)
{
  cells_red = 0;
  const int cols = std::max(1, cfg.red_grid_cols), rows = std::max(1, cfg.red_grid_rows);
  constexpr double kInset = 0.10;  // 往里收 10%, 避开内沿混色
  constexpr int kSub = 8;          // 每格 8×8，远景可见地带较窄，避免少数采样行决定总覆盖率。
  const std::vector<cv::Point2f> square{{0, 0}, {1, 0}, {1, 1}, {0, 1}};
  const cv::Mat to_image = cv::getPerspectiveTransform(square, quad);
  // 内沿近半区被近墙挡住时，格子应覆盖可见地面。仅承认从近边连续相连的紫色墙；
  // 每列只扣本列可证实的连续紫墙，高光或缺色所在列仍按完整地面取样。
  constexpr int kWallSteps = 64;
  std::vector<cv::Point2f> wall_points;
  for (int x = 0; x < cols * kSub; ++x) {
    const float u = float(kInset + (1 - 2 * kInset) * (x + 0.5) / (cols * kSub));
    for (int y = kWallSteps; y >= 0; --y) {wall_points.emplace_back(u, float(y) / kWallSteps);}
  }
  cv::perspectiveTransform(wall_points, wall_points, to_image);
  std::vector<float> visible_end(cols * kSub);
  bool occluded = false;
  for (int x = 0; x < cols * kSub; ++x) {
    int step = 0;
    for (; step <= kWallSteps; ++step) {
      const auto & p = wall_points[x * (kWallSteps + 1) + step];
      const int px = cvRound(p.x), py = cvRound(p.y);
      if (px < 0 || py < 0 || px >= purple.cols || py >= purple.rows || !purple.at<uchar>(py, px)) {break;}
    }
    visible_end[x] = std::min(1.0f, float(kWallSteps - step + 1) / kWallSteps);
    occluded = occluded || visible_end[x] < 1.0f;
  }
  // 近墙遮挡补偿需要斜视证据：两侧向远处收敛，地面纵深也比按实物比例正视时短。
  // 近似正对的紫面板也可能带红条，此时仍从完整地面的中间 80% 实测总面积。
  const double near_width = cv::norm(quad[2] - quad[3]);
  const double far_width = cv::norm(quad[1] - quad[0]);
  const double width = (near_width + far_width) * 0.5;
  const double front_depth = width * (cfg.field_h_m - 2 * cfg.wall_thickness_m) /
    (cfg.field_w_m - 2 * cfg.wall_thickness_m);
  const double fit_uncertainty = 2 * std::max(1, cfg.edge_probe_radius);
  const bool converges = near_width > far_width + fit_uncertainty;
  const bool foreshortened = std::abs(cv::contourArea(quad)) < width * (front_depth - fit_uncertainty);
  const bool use_full_ground = occluded && !(converges && foreshortened);
  std::vector<cv::Point2f> full_ground;
  std::vector<cv::Point2f> unit;
  unit.reserve(static_cast<size_t>(cols * rows * kSub * kSub));
  if (use_full_ground) {full_ground.reserve(unit.capacity());}
  for (int gy = 0; gy < rows; ++gy) {
    for (int gx = 0; gx < cols; ++gx) {
      for (int sy = 0; sy < kSub; ++sy) {
        for (int sx = 0; sx < kSub; ++sx) {
          unit.emplace_back(
            static_cast<float>(kInset + (1 - 2 * kInset) * ((gx + (sx + 0.5) / kSub) / cols)),
            static_cast<float>(kInset + (1 - 2 * kInset) * ((gy + (sy + 0.5) / kSub) / rows)));
          if (use_full_ground) {full_ground.push_back(unit.back());}
          unit.back().y *= visible_end[gx * kSub + sx];
        }
      }
    }
  }
  std::vector<cv::Point2f> image;
  cv::perspectiveTransform(unit, image, to_image);
  if (use_full_ground) {cv::perspectiveTransform(full_ground, full_ground, to_image);}
  // 边缘留白相对于实际可见段。每列红色按可见段占整列地面的比例加权：
  // 近墙遮住大半时，既不裁掉剩余地面的大部分，也不把红色细带归一化成 100%。
  int total = 0;
  double hit = 0;
  const auto interior = HueSector::interior(cfg);
  const auto adjacent_to_seed = [&](int x, int y) {
    for (int row = std::max(0, y - 1); row <= std::min(red.rows - 1, y + 1); ++row) {
      const uchar * line = red.ptr<uchar>(row);
      for (int col = std::max(0, x - 1); col <= std::min(red.cols - 1, x + 1); ++col) {
        if (line[col]) {return true;}
      }
    }
    return false;
  };
  const auto original_colour = [&](int x, int y) {
    return !purple.at<uchar>(y, x) && interior.classify(lab.ptr<uchar>(y) + 3 * x);
  };
  // 3×3 开运算会切去低对比地面的锯齿边界。只沿原图中同一严格色相恢复两步八连通像素。
  const auto supported_colour = [&](int x, int y) {
    if (lab.empty() || !original_colour(x, y)) {return false;}
    if (adjacent_to_seed(x, y)) {return true;}
    for (int row = std::max(0, y - 1); row <= std::min(red.rows - 1, y + 1); ++row) {
      for (int col = std::max(0, x - 1); col <= std::min(red.cols - 1, x + 1); ++col) {
        if ((col != x || row != y) && original_colour(col, row) && adjacent_to_seed(col, row)) {return true;}
      }
    }
    return false;
  };
  for (size_t cell = 0; cell < image.size(); cell += kSub * kSub) {
    const size_t first_column = (cell / (kSub * kSub) % cols) * kSub;
    int cell_total = 0, cell_hit = 0;
    for (size_t k = cell; k < cell + kSub * kSub; ++k) {
      const int x = cvRound(image[k].x), y = cvRound(image[k].y);
      if (x < 0 || y < 0 || x >= red.cols || y >= red.rows) {continue;}
      ++cell_total;
      if (red.at<uchar>(y, x) || supported_colour(x, y)) {
        ++cell_hit;
        hit += visible_end[first_column + (k - cell) % kSub];
      }
    }
    total += cell_total;
    if (cell_total > 0 && static_cast<double>(cell_hit) / cell_total >= cfg.red_cell_min) {++cells_red;}
  }
  if (use_full_ground) {
    total = 0;
    hit = 0.0;
    for (const auto & point : full_ground) {
      const int x = cvRound(point.x), y = cvRound(point.y);
      if (x < 0 || y < 0 || x >= red.cols || y >= red.rows) {continue;}
      ++total;
      if (red.at<uchar>(y, x) || supported_colour(x, y)) {++hit;}
    }
  }
  return total > 0 ? hit / total : 0.0;
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
  // 与红色地面相邻时，四边均有强紫边实测可证明边界，不以外侧同色直接否决。
  if (sc.red_outside > cfg.red_outside_max && !(measured == 4 && sc.edges_ok == 4 && sc.purple >= 0.95)) {
    return "四边形外部也是红色 (框到场地内部了)";
  }
  if (sc.edges_ok < 2) {return "紫色边支持度合格的边少于 2 条";}
  if (sc.edges_ok == 2 && sc.top2_min < cfg.edge_support_strong &&
    !(tracking && sc.temporal >= 0.9)) {return "只有 2 条边有证据, 且强度/时序一致性不足";}
  if (sc.total < cfg.min_total_score) {return "总分过低";}
  // 本帧实测边少于 3 条时几何主要来自先验，不能当测量提交。
  if (measured < 3) {return "实测边少于 3 条";}
  return std::string();
}

}  // namespace field_detail

// ---- 鸟瞰黑带证据与左右分区 ----
// 黑带按“相对两侧内部颜色的局部色度下陷”测量：反光或进光会把黑带提亮到接近内部亮度，
// 但不会给它红/蓝色度，因此判据只和同一行紧邻两侧比较，不依赖全局灰度阈值或绝对亮度。
// 至少一侧紧邻是内部颜色才有资格成为证据，紫色围边立面和场外自然排除；物块贴在一侧时用另一侧作参照。
// 宽于带宽上限的暗区（暗物块）按完整无色区测宽后丢弃；单侧贴线物块用同帧双边实测宽度校验未遮挡沿。
// 两侧都是内部颜色且本列也是内部颜色的位置，
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
// 完整带宽测量的两端外侧在该像素数内须出现内部颜色；单侧可见沿另行按同帧带宽校验。
constexpr int kBandEdgeProbePx = 2;
// 八位 Lab 的 L8 = L×2.55，a、b 偏移 128。
constexpr double kLightTo8bit = 2.55;
constexpr int kLabOffset = 128;

double bandWidthTolerance(double calibrated_width)
{
  return std::max(2.0, 0.35 * calibrated_width);
}

/** @brief 单行内一段连续色度下陷，坐标是搜索窗内列号。 */
struct SplitRun {
  int y; float x, width, depth;
  float left = 0, right = 0;
  int reference_side = 0; // 0=两侧地面；-1/+1=只有左/右侧为地面。
  bool weak_boundary = false;
  float peak_width = 0; // 两侧地面之间实测的相反色度梯度峰间距（像素）。
  float core_width = 0; // 两侧材质色度阶跃之间的连续中性芯宽度（亚像素）。
  float material_peak_width = 0;
  bool sharp_left = false, sharp_right = false;
  float light_center = -1; // 两侧地面均比带芯更亮时，在亮度原分辨率上测到的中心。
};
// 一侧仍可见的色度边；side=-1 为左沿，+1 为右沿。只有同帧双边段给出宽度后才能用。
struct SplitEdge {int y; float x, depth; int side;};

/** @brief 候选直线 x = top + slope * y（搜索窗内坐标）的验收结果。 */
struct SplitLine {
  bool valid = false, measured_core = false, ground_edges_spanning = false;
  double top = 0, slope = 0, confidence = 0, width = 0;
  double edge_top = 0, edge_slope = 0;
};

/** @brief 搜索窗与带参照余量的上下文窗（BEV 坐标），half 是带宽上限的一半。 */
struct SplitWindow {cv::Rect window, context; int half = 0, pad_lo = 0;};

/** @brief 上下文窗内的逐像素证据：纵向平滑后的色度与亮度、放宽后的内部颜色（参照资格）。 */
struct SplitEvidence {cv::Mat chroma, light, reference, opponent, difference;};

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
bool referenceHue(
  const cv::Mat & lab, const cv::Mat & interior, cv::Vec2d & hue,
  double & mean_chroma, double & mean_light)
{
  double sum_a = 0, sum_b = 0, sum_chroma = 0, sum_light = 0;
  int count = 0;
  for (int y = 0; y < lab.rows; ++y) {
    const auto * pixel = lab.ptr<uchar>(y);
    const auto * in = interior.ptr<uchar>(y);
    for (int x = 0; x < lab.cols; ++x, pixel += 3) {
      if (in[x] != 0) {
        const int a = pixel[1] - kLabOffset, b = pixel[2] - kLabOffset;
        sum_a += a; sum_b += b; sum_chroma += std::abs(a) + std::abs(b); sum_light += pixel[0]; ++count;
      }
    }
  }
  const double norm = std::hypot(sum_a, sum_b);
  if (count < kMinReferencePixels * lab.total() || norm < 1e-6) {return false;}
  hue = cv::Vec2d(sum_a / norm, sum_b / norm);
  mean_chroma = sum_chroma / count;
  mean_light = sum_light / count;
  return true;
}

// 参照资格：严格内部颜色，或非紫色、相对色度达到内部下限的 1/4 且色相接近本帧参照色相。
// 局部反光/阴影使红色略出扇区时仍可作参照；紫色围边、低色度地面与黑带、其他颜色物块不能。
bool splitEvidence(const cv::Mat & bgr, const FieldConfig & cfg, SplitEvidence & out)
{
  cv::Mat lab, purple;
  cv::cvtColor(bgr, lab, cv::COLOR_BGR2Lab);
  segmentLab(lab, cfg, purple, out.reference);
  cv::Vec2d hue;
  double mean_chroma, mean_light;
  if (!referenceHue(lab, out.reference, hue, mean_chroma, mean_light)) {return false;}
  const double light_min = kLightTo8bit * (cfg.use_blue_interior ? cfg.blue_L_min : cfg.red_L_min);
  const double chroma_per_light = kReferenceChromaShare / kLightTo8bit *
    (cfg.use_blue_interior ? cfg.blue_chroma_ratio_min : cfg.red_chroma_ratio_min);
  out.chroma.create(lab.size(), CV_32F);
  out.light.create(lab.size(), CV_32F);
  out.opponent.create(lab.size(), CV_32F);
  out.difference.create(lab.size(), CV_32F);
  const auto floor_bgr = cv::mean(bgr, out.reference);
  const int primary = floor_bgr[2] >= floor_bgr[0] ? 2 : 0;
  for (int y = 0; y < lab.rows; ++y) {
    const auto * pixel = lab.ptr<uchar>(y);
    const auto * violet = purple.ptr<uchar>(y);
    auto * reference = out.reference.ptr<uchar>(y);
    auto * opponent = out.opponent.ptr<float>(y);
    auto * difference = out.difference.ptr<float>(y);
    const auto * rgb = bgr.ptr<cv::Vec3b>(y);
    auto * c = out.chroma.ptr<float>(y);
    auto * l = out.light.ptr<float>(y);
    for (int x = 0; x < lab.cols; ++x, pixel += 3) {
      const int a = pixel[1] - kLabOffset, b = pixel[2] - kLabOffset;
      l[x] = pixel[0];
      c[x] = float(std::abs(a) + std::abs(b));
      // 仅有 C/L 在暗处会把暖色黑块的 2–3 级量化色度认作红色参照。
      // 低于本帧地面亮度时仍须有真实色度；高光变亮后允许色度降低。
      // 两者沿用同一个放宽比例，不让被反光稍微提亮的暖色黑块给自身窄边作证。
      const bool floor_colour = pixel[0] >= mean_light || c[x] >= kReferenceChromaShare * mean_chroma;
      // 主色相对绿色的归一化色差保留正负号；变暗的红、蓝或紫材质仍保留自身色度。
      // 仅在已经有两侧材质参照的行使用，避免把亮度阴影计入无色隔断。
      const int p = rgb[x][primary], g = rgb[x][1];
      difference[x] = float(p - g);
      opponent[x] = difference[x] / std::max(1, p + g);
      const bool matches_hue = a * hue[0] + b * hue[1] >= kReferenceHueCos * std::hypot(a, b);
      // 阴影缩放 L 与色度，高光则把 L 推向白点并压低色度：分别用到黑/白点的距离归一化。
      // 仍须匹配本帧地面色相；纯白/纯灰没有色度，不能充当参照。
      const double light_range = std::max(1, std::min(int(pixel[0]), 255 - int(pixel[0])));
      if (violet[x] == 0 && pixel[0] >= light_min && floor_colour && c[x] >= chroma_per_light * light_range &&
        matches_hue) {reference[x] = 255;}
    }
  }
  // 只沿黑带方向（纵向）平滑压缩噪声，横向不扩散，带宽保持实测。
  cv::blur(out.chroma, out.chroma, cv::Size(1, 3));
  cv::blur(out.light, out.light, cv::Size(1, 3));
  cv::blur(out.difference, out.difference, cv::Size(1, 3));
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
struct ColumnDip {double depth = 0, reference = 0, max_light = 0; bool two_sided = false;};

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
  double reference = dip.two_sided ? std::min(cl, cr) : (left ? cl : cr);
  const double brightest = dip.two_sided ? std::max(ll, lr) : (left ? ll : lr);
  // 中性反光越亮，地面色度也按到白点的距离衰减；先消除宽缓高光坡的这部分色度变化。
  // 黑带仍须比该局部地面参照再下降 black_min_depth，浅灰带可比地面略亮。
  if (light[cx] > brightest) {
    reference *= (255.0 - light[cx]) / std::max(1.0, 255.0 - brightest);
  }
  if (reference > 1.0 && light[cx] <= brightest * (1.0 + cfg.black_max_lift)) {
    dip.depth = 1.0 - chroma[cx] / reference;
    dip.reference = reference;
    dip.max_light = brightest * (1.0 + cfg.black_max_lift);
  }
  return dip;
}

// 两侧边界必须由像素变化测出，至少一侧紧邻内部颜色；另一侧可以是有明确边界的异色物块。
double bandReferenceChroma(
  const float * chroma, const float * light, double max_light, const uchar * reference,
  int left, int right, int context_width, int & reference_side)
{
  auto mean = [&](int from, int step) {
      double sum = 0; int count = 0;
      for (int k = 0; k < kBandEdgeProbePx; ++k) {
        const int x = from + step * k;
        if (x >= 0 && x < context_width && reference[x] != 0) {sum += chroma[x]; ++count;}
      }
      return count ? sum / count : -1.0;
    };
  const double l = mean(left - 1, -1), r = mean(right, 1);
  reference_side = l < 0 ? 1 : r < 0 ? -1 : 0;
  if ((l < 0) != (r < 0)) {
    // 非地面的一侧必须确实测到不同材质边界：异色块色度回升，白块亮度超出黑带上限。
    // 灰黑物块内部的小波动不算另一条边，不能凭一侧地面把物块误分成窄带。
    const int from = l < 0 ? left - 1 : right, step = l < 0 ? -1 : 1;
    const double ground = std::max(l, r);
    bool material_edge = false;
    for (int i = 0; i < kBandEdgeProbePx; ++i) {
      const int x = from + i * step;
      if (x >= 0 && x < context_width && (chroma[x] >= ground || light[x] > max_light)) {
        material_edge = true;
      }
    }
    if (!material_edge) {return -1;}
  }
  return l < 0 ? r : r < 0 ? l : std::min(l, r);
}

// 色度阈值只定位边缘邻域；用邻域内实际色度梯度的重心测量亚像素边界，
// 避免参照亮度变化使二值阈值边缘跳动一个像素。
double measuredEdge(const float * chroma, int boundary, int side, int width)
{
  double weight = 0, moment = 0;
  for (int x = std::max(1, boundary - kBandEdgeProbePx);
    x <= std::min(width - 1, boundary + kBandEdgeProbePx); ++x) {
    const double w = std::max(0.0, side * double(chroma[x] - chroma[x - 1]));
    weight += w; moment += w * (x - 0.5);
  }
  return weight > 0 ? moment / weight : boundary - 0.5;
}

// JPEG 色度子采样会把两条色度边一起移开，亮度仍有完整分辨率。仅在两侧
// 实测地面比暗芯更亮、亮差超出各自局部噪声时，在半高度交点间定位。
double darkBandCenter(const float * light, const uchar * floor, int cols, int mid, int half)
{
  if (mid - 2 * half < 0 || mid + 2 * half >= cols) {return -1;}
  const float core = *std::min_element(light + mid - half / 3, light + mid + half / 3 + 1);
  double crossing[2];
  for (int side = 0; side < 2; ++side) {
    const int direction = 2 * side - 1;
    double sum = 0, noise = 0; int count = 0;
    for (int d = half; d < 2 * half; ++d) {
      const int x = mid + direction * d;
      if (!floor[x]) {continue;}
      sum += light[x]; noise += std::abs(light[x] - light[x + direction]); ++count;
    }
    if (count < kReferenceShare * half || sum / count - core <= 3 * std::max(1.0, noise / count)) {return -1;}
    const double level = 0.5 * (sum / count + core);
    if (light[mid] >= level) {return -1;}
    int at = mid;
    while (std::abs(at - mid) < half && light[at + direction] < level) {at += direction;}
    if (std::abs(at - mid) >= half) {return -1;}
    crossing[side] = at + direction * (level - light[at]) / (light[at + direction] - light[at]);
  }
  return 0.5 * (crossing[0] + crossing[1]);
}

// 混色会让边界像素落入下陷段。找紧邻实测边界的同向色度梯度峰，
// 对峰顶及相邻两个梯度作抛物线插值；单调坡或没有阶跃时不给出边界。
double edgePeak(const float * chroma, int boundary, int side, int width)
{
  auto gradient = [&](int x) {return side * (chroma[x] - chroma[x - 1]);};
  int best = -1;
  float peak = 0;
  for (int x = std::max(2, boundary - kBandEdgeProbePx);
    x <= std::min(width - 2, boundary + kBandEdgeProbePx); ++x) {
    const float g = gradient(x);
    if (g > peak) {best = x; peak = g;}
  }
  if (best < 0) {return -1;}
  const float left = gradient(best - 1), right = gradient(best + 1);
  if (left > peak || right > peak || (left == peak && right == peak)) {return -1;}
  return best - 0.5 + (right - left) / (2.0 * (2.0 * peak - left - right));
}

// 同行两侧实际材质给出色度参照。用阈值交点插值，避免把混色像素整列算入实体。
void measureMaterialCore(const float * opponent, const float * difference, const uchar * reference,
  int width, double center, int half, SplitRun & run)
{
  const int x = cvRound(center);
  if (x - 2 * half < 0 || x + 2 * half >= width) {return;}
  auto side = [&](int direction) {
      double sum = 0; int count = 0;
      for (int i = half + 1; i <= 2 * half; ++i) {
        const int at = x + direction * i;
        if (reference[at]) {sum += opponent[at]; ++count;}
      }
      return count >= kReferenceShare * half ? sum / count : 0.0;
    };
  const double floor = std::min(side(-1), side(1));
  if (floor <= 0) {return;}
  auto edges = [&](double level, double & left, double & right) {
      const double threshold = level * floor;
      if (opponent[x] >= threshold) {return false;}
      int lo = x, hi = x;
      while (lo > 1 && opponent[lo - 1] < threshold) {--lo;}
      while (hi + 2 < width && opponent[hi + 1] < threshold) {++hi;}
      if (lo < 2 || hi + 2 >= width || opponent[lo - 2] < threshold || opponent[hi + 2] < threshold) {return false;}
      left = lo - (threshold - opponent[lo]) / (opponent[lo - 1] - opponent[lo]);
      right = hi + (threshold - opponent[hi]) / (opponent[hi + 1] - opponent[hi]);
      return true;
    };
  double left, right;
  if (!edges(kReferenceChromaShare, left, right) ||
    std::abs(0.5 * (left + right) - center) > kLineTolerancePx) {return;}
  run.core_width = float(right - left);
  // 对称失焦能缩窄中性芯，线性通道差的梯度峰仍能测量其真实材质边距。
  const double left_peak = edgePeak(difference, int(std::ceil(left)), -1, width);
  const double right_peak = edgePeak(difference, int(std::ceil(right)), 1, width);
  if (left_peak >= 0 && right_peak > left_peak) {run.material_peak_width = float(right_peak - left_peak);}
  // 单侧地面阴影只扩展一侧的色度过渡。另一侧边界跨多个色度层的位置变动
  // 应在一个采样间隔内；远中近多行还须在同一侧保有这种可定位的材质边。
  double inner_l, inner_r, outer_l, outer_r;
  if (edges(0.5 * kReferenceChromaShare, inner_l, inner_r) &&
    edges(2.0 * kReferenceChromaShare, outer_l, outer_r)) {
    const double l = inner_l - outer_l, r = outer_r - inner_r;
    run.sharp_left = l <= 1.0;
    run.sharp_right = r <= 1.0;
  }
}

// 连续下陷段 [start,end)（上下文列号）收尾：宽度按同一色度门槛向两侧延伸的完整无色区测量，
// 不受参照资格截断，宽暗物块中心恰有资格的一两列不能冒充窄黑带。
// 至少一端须紧邻地面、另一端须有真实材质边界；中心按无色区内的下陷加权。
bool measureRun(
  const float * chroma, const float * light, const uchar * reference_mask,
  const float * opponent, const float * difference,
  int width_limit, int context_width,
  int start, int end, double reference, double max_light, double depth, const FieldConfig & cfg, int y, int pad_lo,
  SplitRun & run)
{
  const double limit = (1.0 - cfg.black_min_depth) * reference;
  int left = start, right = end;
  while (left > 0 && chroma[left - 1] <= limit && light[left - 1] <= max_light &&
    right - left <= width_limit) {--left;}
  while (right < context_width && chroma[right] <= limit && light[right] <= max_light &&
    right - left <= width_limit) {++right;}
  if (right - left > width_limit) {return false;}
  // 远处参照用于发现下陷，真正的窄带还须比紧邻地面更少色度。
  // 阴影的缓坡或物块边缘量化噪声不能借远处较亮红色把 1 像素小波动放大成深谷。
  int reference_side;
  const double local_reference = bandReferenceChroma(
    chroma, light, max_light, reference_mask, left, right, context_width, reference_side);
  if (local_reference <= 0) {return false;}
  double weight = 0, moment = 0, sum_chroma = 0;
  for (int x = left; x < right; ++x) {
    const double w = std::max(0.0, reference - chroma[x]);
    weight += w; moment += w * x; sum_chroma += chroma[x];
  }
  if (sum_chroma / (right - left) > (1.0 - cfg.black_min_depth) * local_reference) {return false;}
  if (weight <= 0) {return false;}
  run = {y, float(moment / weight - pad_lo), float(right - left), float(depth),
    float(measuredEdge(chroma, start, -1, context_width) - pad_lo),
    float(measuredEdge(chroma, end, 1, context_width) - pad_lo), reference_side};
  if (reference_side == 0) {
    const double center = darkBandCenter(light, reference_mask, context_width,
      cvRound(run.x + pad_lo), width_limit / 2);
    if (center >= 0 && std::abs(center - pad_lo - run.x) <= kLineTolerancePx) {
      run.light_center = float(center - pad_lo);
    }
    const double left_peak = edgePeak(chroma, left, -1, context_width);
    const double right_peak = edgePeak(chroma, right, 1, context_width);
    if (left_peak >= 0 && right_peak > left_peak) {run.peak_width = float(right_peak - left_peak);}
    measureMaterialCore(opponent, difference, reference_mask, context_width, run.x + pad_lo, width_limit / 2, run);
  }
  return true;
}

// 一行内的下陷段写入 runs；两侧参照都合格且本列也有内部颜色资格的位置记入 eligible（可作反证）。
void rowRuns(
  const SplitEvidence & evidence, const SplitWindow & window, const FieldConfig & cfg, int y,
  std::vector<SplitRun> & runs, uchar * eligible, std::vector<SplitEdge> & edges)
{
  const auto * chroma = evidence.chroma.ptr<float>(y);
  const auto * light = evidence.light.ptr<float>(y);
  const auto * reference = evidence.reference.ptr<uchar>(y);
  const int width = window.context.width, half = window.half, pad_lo = window.pad_lo;
  const RowSums sums(chroma, light, reference, width);
  int start = -1;
  double depth_sum = 0, reference_sum = 0, light_limit_sum = 0;
  auto close = [&](int end) {
      SplitRun run;
      const int count = end - start;
      if (measureRun(chroma, light, reference, evidence.opponent.ptr<float>(y), evidence.difference.ptr<float>(y),
        2 * half, width, start + pad_lo, end + pad_lo,
        reference_sum / count, light_limit_sum / count, depth_sum / count, cfg, y, pad_lo, run)) {runs.push_back(run);}
      if (start + pad_lo > 0 && reference[start + pad_lo - 1] != 0) {
        edges.push_back({y, float(measuredEdge(chroma, start + pad_lo, -1, width) - pad_lo),
          float(depth_sum / count), -1});
      }
      if (end + pad_lo < width && reference[end + pad_lo] != 0) {
        edges.push_back({y, float(measuredEdge(chroma, end + pad_lo, 1, width) - pad_lo),
          float(depth_sum / count), 1});
      }
      start = -1; depth_sum = reference_sum = light_limit_sum = 0;
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
      depth_sum += dip.depth; reference_sum += dip.reference; light_limit_sum += dip.max_light;
    } else if (start >= 0) {
      close(x);
    }
  }
  if (start >= 0) {close(window.window.width);}
}

// 深色物块贴住黑带时，色度只能给出地面侧强边；另一边仍可能有微弱但真实的亮度阶跃。
// 20 mm 只限定搜索邻域，实际边界须明显超过两侧局部噪声，再按原覆盖率累积为直线。
void weakBoundaryRuns(
  const SplitEvidence & evidence, const SplitWindow & window, const FieldConfig & cfg,
  double calibrated_width, const std::vector<SplitEdge> & edges, std::vector<SplitRun> & runs)
{
  // 单侧地面段的另一侧可以靠异色物料的色度回升定位。若只有亮度越界，
  // 宽缓反光也会截出一个似乎带宽正确的短段，须由下方的亮度阶跃重新实测。
  runs.erase(std::remove_if(runs.begin(), runs.end(), [&](const SplitRun & run) {
      if (run.reference_side == 0) {return false;}
      const int side = run.reference_side;
      const double ground_edge = side < 0 ? run.left : run.right;
      const double other_edge = side < 0 ? run.right : run.left;
      const int ground = cvRound(ground_edge + side * 2) + window.pad_lo;
      const int other = cvRound(other_edge - side * 2) + window.pad_lo;
      if (ground < 0 || other < 0 || ground >= evidence.chroma.cols || other >= evidence.chroma.cols) {
        return true;
      }
      const auto * chroma = evidence.chroma.ptr<float>(run.y);
      return chroma[other] < (1.0 - cfg.black_min_depth) * chroma[ground];
    }), runs.end());
  for (const auto & edge : edges) {
    const auto * light = evidence.light.ptr<float>(edge.y);
    const auto * chroma = evidence.chroma.ptr<float>(edge.y);
    const int ground = cvRound(edge.x + edge.side * 2 + window.pad_lo);
    if (ground < 0 || ground >= evidence.chroma.cols) {continue;}
    const double chroma_limit = (1.0 - cfg.black_min_depth) * chroma[ground];
    const double light_limit = (1.0 + cfg.black_max_lift) * light[ground];
    const double expected = edge.x - edge.side * calibrated_width + window.pad_lo;
    const int first = std::max(4, int(std::floor(expected - kLineTolerancePx)));
    const int last = std::min(evidence.light.cols - 6, int(std::ceil(expected + kLineTolerancePx)));
    double best_signal = 0, boundary = 0;
    for (int x = first; x <= last; ++x) {
      // x 是阶跃左侧最后一个像素。跳过混色的边界像素，从两侧平台分别估计信号和噪声。
      const double left = 0.5 * (light[x - 2] + light[x - 1]);
      const double right = 0.5 * (light[x + 2] + light[x + 3]);
      const double difference = std::abs(right - left);
      const double noise = std::hypot(light[x - 1] - light[x - 2], light[x + 3] - light[x + 2]);
      // 平台均值之差不能代替 x 与 x+1 之间的真实边界。暗块上的光照坡和 JPEG
      // 阶梯可在两侧给出低噪声平台，中间却平坦或反向；必须把边定位在同向局部梯度峰。
      // 相邻峰相等仍保留，允许透视插值把一条真实弱边分摊到两个像素。
      const double step = light[x + 1] - light[x];
      if (step * (right - left) <= 0 || std::abs(step) <
        std::max(std::abs(light[x] - light[x - 1]), std::abs(light[x + 2] - light[x + 1]))) {continue;}
      // 两侧各四个未混色像素必须形成两块分离的平台：净间隔和中心阶跃
      // 都要超过任一平台的亮度摆幅，排除高光坡上偶然出现的局部梯度峰。
      const auto left_range = std::minmax_element(light + x - 4, light + x);
      const auto right_range = std::minmax_element(light + x + 2, light + x + 6);
      const double gap = right > left ? *right_range.first - *left_range.second :
        *left_range.first - *right_range.second;
      const double variation = std::max(*left_range.second - *left_range.first,
        *right_range.second - *right_range.first);
      if (gap <= variation || std::abs(step) <= variation) {continue;}
      // 局部平台均值的差须超过三倍噪声。若单点抖动抬高这个估计，四点
      // 平台之间仍有超过三个 Lab 量化级的净间隔也足以直接分辨材质。
      if (difference <= 3.0 * std::max(1.0, noise) && gap <= 3.0) {continue;}
      // 与常规完整带相同：整段必须比紧邻地面缺色且不明显更亮。
      // 仅检查边界两点会把阴影红地面和远处黑块边拼成一条假黑带。
      const double strong = edge.x + window.pad_lo, other = x + 0.5;
      const int from = int(std::ceil(std::min(strong, other)));
      const int to = int(std::floor(std::max(strong, other)));
      double band_chroma = 0, band_light = 0;
      for (int pixel = from; pixel <= to; ++pixel) {
        band_chroma += chroma[pixel]; band_light += light[pixel];
      }
      const int count = to - from + 1;
      if (count <= 0 || band_chroma > chroma_limit * count || band_light > light_limit * count) {continue;}
      if (difference > best_signal) {
        best_signal = difference;
        boundary = x + 0.5;
      }
    }
    if (best_signal == 0) {continue;}
    const double other = boundary - window.pad_lo;
    const double left = edge.side < 0 ? edge.x : other;
    const double right = edge.side < 0 ? other : edge.x;
    if (std::any_of(runs.begin(), runs.end(), [&](const SplitRun & run) {
        return run.y == edge.y && std::abs(run.x - 0.5 * (left + right)) <= kLineTolerancePx;
      })) {continue;}
    runs.push_back({edge.y, float(0.5 * (left + right)), float(right - left + 1.0), edge.depth,
      float(left), float(right), edge.side, true});
  }
}

struct InteriorRows {int count = 0, first = -1, last = -1;};

InteriorRows interiorRows(const cv::Mat & interior, const SplitWindow & window)
{
  const int cols = window.window.width;
  InteriorRows rows;
  for (int y = 0; y < interior.rows; ++y) {
    if (cv::countNonZero(interior.row(y).colRange(window.pad_lo, window.pad_lo + cols)) >=
      kInteriorRowShare * cols) {
      if (rows.first < 0) {rows.first = y;}
      rows.last = y;
      ++rows.count;
    }
  }
  return rows;
}

/** @brief 统计直线沿线的资格行与支持行，检查覆盖、分段、残差，并用 Huber 拟合精修。 */
SplitLine acceptLine(
  const std::vector<SplitRun> & runs, const cv::Mat & eligible, double top, double slope,
  const FieldConfig & cfg, int min_rows, const InteriorRows & interior_rows,
  const std::vector<SplitEdge> & edges, double calibrated_width, double top_scale)
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
  // 黑带在 BEV 中宽度恒定：宽度偏离沿线中位数超过 kWidthSpread 倍的段是贴线物块或噪声，不算支持。
  std::vector<float> line_widths;
  std::vector<float> anchor_widths;
  bool have_strong_boundary = false;
  for (int y = 0; y < rows; ++y) {
    if (best[y] >= 0) {
      const auto & run = runs[best[y]];
      line_widths.push_back(run.width);
      if (!run.weak_boundary && run.reference_side == 0) {anchor_widths.push_back(run.width);}
    }
  }
  if (line_widths.empty()) {return result;}
  const double measured_width = upperMedian(anchor_widths.empty() ? line_widths : anchor_widths);
  // 完整双边行不足时，物理宽度只补“宽度”，不能补一条根本未测到的分界。
  // 仍须至少一个宽度相符的真实双边段来区分黑带与贴邻物块的外沿。
  // 弱边和只有一侧地面的段不能充当完整双边锚点；数量再多也不能解锁实测带宽。
  const bool calibrated = anchor_widths.size() < kMinRowsFloor;
  // 段宽按像素个数计，物理宽度是两边坐标间距，因此对应个数为该间距 + 1。
  const double typical_width = calibrated ? calibrated_width + 1.0 : measured_width;
  if (calibrated && (measured_width > kWidthSpread * typical_width ||
    measured_width * kWidthSpread < typical_width)) {return result;}
  std::vector<SplitRun> continued(rows);
  std::vector<bool> have_continued(rows, false);
  std::vector<bool> visible_edge(rows, false);
  // 双边充分时宽度和方向均实测；少量双边时标定宽度约束身份，方向由整段可见沿拟合。
  std::vector<cv::Point2f> anchors;
  std::vector<double> left_offsets, right_offsets;
  for (int y = 0; y < rows; ++y) {
    if (best[y] < 0) {continue;}
    const auto & run = runs[best[y]];
    if (run.width > kWidthSpread * typical_width || run.width * kWidthSpread < typical_width) {continue;}
    // 少量双地面段靠标定宽度辨认身份，续接前须核对真实边距；阈值段的像素
    // 宽度会受光照扩散影响，1.5 倍粗筛不能代替与弱边搜索相同的物理宽度容差。
    const bool physical_identity = !calibrated ||
      std::abs((run.right - run.left) - calibrated_width) <= kLineTolerancePx;
    have_strong_boundary |= !run.weak_boundary && run.reference_side == 0 && physical_identity;
    if (run.reference_side == 0) {
      anchors.emplace_back(run.x, float(y));
      left_offsets.push_back(run.x - run.left);
      right_offsets.push_back(run.right - run.x);
    }
  }
  // 单边的位置由色度梯度测量；用同帧锚点测量“该边到中心”的偏移，
  // 不能把完整扩展段的像素个数直接除二，否则混色边会产生半像素台阶并在外推时放大。
  const double left_offset = anchors.size() >= kMinRowsFloor ? upperMedian(left_offsets) : calibrated_width * 0.5;
  const double right_offset = anchors.size() >= kMinRowsFloor ? upperMedian(right_offsets) : calibrated_width * 0.5;
  {
    // Hough 网格斜率只用于找候选；续接必须服从实际双边锚点拟合的方向。
    // 否则短直线的斜投票峰会斜接到遮挡物外沿，伪造一条完整分界。
    double anchor_slope = slope, anchor_top = top;
    if (anchors.size() >= kMinRowsFloor) {
      cv::Vec4f anchor_line;
      cv::fitLine(anchors, anchor_line, cv::DIST_HUBER, 0, 0.01, 0.01);
      anchor_slope = double(anchor_line[0]) / anchor_line[1];
      anchor_top = anchor_line[2] - anchor_slope * anchor_line[3];
      // 精修方向后重新收齐该线的双边段，不能把初始斜投票漏掉的真测量降成单边续接。
      std::fill(best.begin(), best.end(), -1);
      std::fill(best_distance.begin(), best_distance.end(), kLineTolerancePx);
      for (int i = 0; i < static_cast<int>(runs.size()); ++i) {
        const auto & run = runs[i];
        const double distance = std::abs(run.x - (anchor_top + anchor_slope * run.y));
        if (distance <= best_distance[run.y]) {best[run.y] = i; best_distance[run.y] = distance;}
      }
    }
    top = anchor_top; slope = anchor_slope;
    for (const auto & edge : edges) {
      const double center = edge.side < 0 ? edge.x + left_offset : edge.x - right_offset;
      const double distance = std::abs(center - (anchor_top + anchor_slope * edge.y));
      if (distance <= kLineTolerancePx) {visible_edge[edge.y] = true;}
      // 弱边须沿行累积真实支持；可见强边旁未测到阶跃的行是反证，不能自动续接。
      if (!have_strong_boundary || best[edge.y] >= 0) {continue;}
      if (distance <= best_distance[edge.y]) {
        best_distance[edge.y] = distance;
        continued[edge.y] = {edge.y, float(center), float(typical_width), edge.depth};
        have_continued[edge.y] = true;
      }
    }
  }
  std::vector<int> eligible_rows;
  for (int y = 0; y < rows; ++y) {
    const int x = cvRound(top + slope * y);
    if (best[y] >= 0 || have_continued[y] || visible_edge[y] ||
      (x >= 0 && x < cols && eligible.at<uchar>(y, x) != 0)) {eligible_rows.push_back(y);}
  }
  if (static_cast<int>(eligible_rows.size()) < min_rows) {return result;}
  auto consistent = [&](int y) {
      if (have_continued[y]) {return true;}
      if (best[y] < 0) {return false;}
      const double width = runs[best[y]].width;
      return width <= kWidthSpread * typical_width && width * kWidthSpread >= typical_width;
    };
  std::vector<cv::Point2f> centers;
  std::vector<double> widths;
  std::vector<double> peak_widths;
  int first_peak_row = -1, last_peak_row = -1;
  double depth_sum = 0;
  const int first = eligible_rows.front(), span = eligible_rows.back() - first + 1;
  int thirds_total[3] = {}, thirds_hit[3] = {};
  int core_hits[2][3] = {};
  for (int y : eligible_rows) {
    const int third = std::min(2, (y - first) * 3 / span);
    ++thirds_total[third];
    if (!consistent(y)) {continue;}
    ++thirds_hit[third];
    auto run = have_continued[y] ? continued[y] : runs[best[y]];
    if (run.core_width > 0 &&
      std::abs(run.core_width / top_scale - calibrated_width) <= bandWidthTolerance(calibrated_width)) {
      const bool narrow = run.material_peak_width > 0 &&
        std::abs(run.material_peak_width / top_scale - calibrated_width) <= bandWidthTolerance(calibrated_width);
      if (narrow || run.sharp_left) {++core_hits[0][third];}
      if (narrow || run.sharp_right) {++core_hits[1][third];}
    }
    if (run.peak_width > 0) {
      peak_widths.push_back(run.peak_width);
      if (first_peak_row < 0) {first_peak_row = y;}
      last_peak_row = y;
    }
    if (run.reference_side < 0) {run.x = float(run.left + left_offset);}
    if (run.reference_side > 0) {run.x = float(run.right - right_offset);}
    centers.emplace_back(run.x, float(y)); widths.push_back(run.width); depth_sum += run.depth;
  }
  const double coverage = double(centers.size()) / eligible_rows.size();
  if (centers.empty() || coverage < cfg.black_score_frac) {return result;}
  // 支持行的首末跨度也须达到内部行的同一比例：物块盖住一端时，剩下的短暗条与场内暗物体无法区分。
  const double support_span = centers.back().y - centers.front().y + 1;
  if (support_span < cfg.black_score_frac * interior_rows.count) {return result;}
  // 分段必须覆盖实际内部区域的远、近端；不能只把残存短线自身分成三段。
  const double interior_span = interior_rows.last - interior_rows.first + 1.0;
  if (interior_rows.count == 0 || centers.front().y >= interior_rows.first + interior_span / 3.0 ||
    centers.back().y < interior_rows.first + 2.0 * interior_span / 3.0) {return result;}
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
  result.width = measured_width;
  // 整像素宽度刚超过上限时，用同帧双地面行的实测双峰距离复核。
  // 两侧各只容许一个混色像素；贯通远近的多数行须实测，
  // 单边续接、弱边、整个像素都属于黑带的宽暗条不能借此缩窄。
  const double max_width = calibrated_width + 1.0 + bandWidthTolerance(calibrated_width);
  const int required_peak_rows = std::max(min_rows, int(std::ceil(cfg.black_score_frac * interior_rows.count)));
  for (const auto & hits : core_hits) {
    bool sufficient = hits[0] + hits[1] + hits[2] >= required_peak_rows;
    for (int i = 0; i < 3; ++i) {
      sufficient &= hits[i] >= std::max(1.0, 0.5 * cfg.black_score_frac * thirds_total[i]);
    }
    result.measured_core |= sufficient;
  }
  if (measured_width > max_width && measured_width <= max_width + 2.0 &&
    int(peak_widths.size()) >= required_peak_rows &&
    first_peak_row >= 0 && first_peak_row < interior_rows.first + interior_span / 3.0 &&
    last_peak_row >= interior_rows.first + 2.0 * interior_span / 3.0) {
    result.width = upperMedian(peak_widths);
  }
  // 阈值色度质心适合验收。短促可见的黑带受水平模糊时，质心会随混色像素
  // 进出阈值跳变，外推到近端会放大误差。两侧地面均实测且跨越远近时，
  // 用成对亚像素材质边缘的中点精修位置；物块旁的单边续接仍只用于验收。
  std::vector<cv::Point2f> edge_centers;
  for (int y : eligible_rows) {
    if (best[y] < 0 || !consistent(y)) {continue;}
    const auto & run = runs[best[y]];
    const float center = run.light_center >= 0 ? run.light_center : 0.5f * (run.left + run.right);
    if (run.reference_side == 0 && !run.weak_boundary && std::abs(center - run.x) <= kLineTolerancePx) {
      edge_centers.emplace_back(center, float(y));
    }
  }
  if (measured_width <= max_width && int(edge_centers.size()) >= required_peak_rows &&
    edge_centers.front().y < interior_rows.first + interior_span / 3.0 &&
    edge_centers.back().y >= interior_rows.first + 2.0 * interior_span / 3.0) {
    cv::fitLine(edge_centers, line, cv::DIST_HUBER, 0, 0.01, 0.01);
    if (std::abs(line[1]) > 1e-6f) {
      result.ground_edges_spanning = true;
      result.edge_slope = double(line[0]) / line[1];
      result.edge_top = line[2] - result.edge_slope * line[3];
    }
  }
  // 置信度 = 下陷强度 × 资格行覆盖率 × 资格行占内部行比例：被遮挡越多、证据越短，置信度越低，
  // 选优时完整分界线优先于只在物块高度内成立的短暗边。
  result.confidence = std::min(1.0, depth / cfg.black_conf_depth) * coverage *
    std::min(1.0, double(eligible_rows.size()) / std::max(1, interior_rows.count));
  return result;
}

// 投票：每段对 (中点列, 斜率) 网格投一票；只合并测量容差内的重复峰，保留邻近分界的歧义。
std::vector<SplitLine> candidateLines(
  const std::vector<SplitRun> & runs, const cv::Mat & eligible, const FieldConfig & cfg,
  int min_rows, const InteriorRows & interior_rows, const std::vector<SplitEdge> & edges,
  double calibrated_width, double top_scale)
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
  // 少量双边片段无法独自形成投票峰时，让已标定宽度的真实单边参与找线。
  // acceptLine 仍核对同帧双边片段、完整行数、跨度、反证和歧义；无双边片段不进入此路径。
  if (runs.size() < static_cast<std::size_t>(min_rows)) {
    for (const auto & edge : edges) {
      for (int k = 0; k < kSlopeBins; ++k) {
        const int x = cvRound(edge.x - edge.side * calibrated_width * 0.5 - slopeOf(k) * (edge.y - middle));
        if (x >= 0 && x < cols) {votes.at<float>(k, x) += 1.0f;}
      }
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
      interior_rows, edges, calibrated_width, top_scale);
    // 只抑制实际内部首末位置都相近的直线，不能整列清空不同斜率：
    // 两条交叉黑带可以有相同中点，却是两种同样成立的分界解释。
    auto suppress = [&](double line_top, double line_slope) {
        for (int k = 0; k < kSlopeBins; ++k) {
          const double candidate_slope = slopeOf(k);
          auto * row = votes.ptr<float>(k);
          for (int x = 0; x < cols; ++x) {
            const double candidate_top = x - candidate_slope * middle;
            auto distance = [&](int y) {
                return std::abs(candidate_top - line_top + (candidate_slope - line_slope) * y);
              };
            if (distance(interior_rows.first) <= kLineTolerancePx &&
              distance(interior_rows.last) <= kLineTolerancePx) {row[x] = 0;}
          }
        }
      };
    suppress(peak.x - slope * middle, slope);
    if (line.valid) {suppress(line.top, line.slope);}
    if (line.valid) {
      // 不同斜率峰可被同一批真实锚点精修到同一条线；这不是第二条物理分界。
      auto same = std::find_if(accepted.begin(), accepted.end(), [&](const SplitLine & prior) {
          auto distance = [&](int y) {return std::abs(prior.top + prior.slope * y - line.top - line.slope * y);};
          return distance(interior_rows.first) <= kLineTolerancePx &&
            distance(interior_rows.last) <= kLineTolerancePx;
        });
      if (same == accepted.end()) {accepted.push_back(line);}
      else if (line.confidence > same->confidence) {*same = line;}
    }
  }
  return accepted;
}

// 隔断与远墙同高。地面两侧都有连续的窄材质芯，而阴影/侧面使暗带看起来偏宽时，
// 再在远墙紫色立面独立查找同一窄黑带。两种材质上的观测都必须来自本帧。
struct WallProjection {double scale = 1, far_y = 0;};

SplitMeasurement measureSplit(const cv::Mat & bev, const FieldConfig & cfg, const cv::Mat & source,
  const FieldResult * field, bool on_wall, bool require_ground_edges = false);

WallProjection wallProjection(const FieldResult & field, const FieldConfig & cfg)
{
  if (field.outline.size() != 4 || field.inner_measured_edges < 2 ||
    field.edge_state[0] != EdgeState::MEASURED || field.edge_state[1] != EdgeState::MEASURED ||
    field.edge_state[3] != EdgeState::MEASURED) {return {};}
  const int w = field.bev.cols, h = field.bev.rows;
  std::vector<cv::Point2f> far{field.outline[0], field.outline[1]};
  cv::perspectiveTransform(far, far, field.homography);
  const double inner_w = cfg.field_w_m - 2.0 * cfg.wall_thickness_m;
  const double scale = (far[1].x - far[0].x) * inner_w / ((w - 1) * cfg.field_w_m);
  if (!std::isfinite(scale) || scale <= 1) {return {};}
  // 外沿顶角换算到内沿顶面；地面单应中 y=0 是远墙脚。裁掉上沿混色和下沿红底。
  const double far_y = 0.5 * (far[0].y + far[1].y) + scale * cfg.wall_thickness_m * (h - 1) /
    (cfg.field_h_m - 2.0 * cfg.wall_thickness_m);
  if (!std::isfinite(far_y) || far_y < -h || far_y >= 0) {return {};}
  return {scale, far_y};
}

bool confirmedOnFarWall(const cv::Mat & source, const FieldResult & field, const FieldConfig & cfg,
  const SplitMeasurement & ground, double calibrated_width, const WallProjection & projection)
{
  if (source.empty() || projection.scale <= 1) {return false;}
  const int w = field.bev.cols, h = field.bev.rows;
  // 仅用立面中央的一半，两端分别与墙顶和地面混色；投影高度仍来自本帧实测墙角。
  const int y0 = cvRound(0.75 * projection.far_y), y1 = cvRound(0.25 * projection.far_y);
  if (y1 - y0 < kMinRowsFloor) {return false;}
  cv::Mat translation = cv::Mat::eye(3, 3, CV_64F);
  translation.at<double>(1, 2) = -y0;
  cv::Mat wall;
  cv::warpPerspective(source, wall, translation * field.homography, {w, y1 - y0});
  auto wall_cfg = cfg;
  wall_cfg.use_blue_interior = false;
  wall_cfg.red_hue_lo_deg = cfg.purple_hue_lo_deg;
  wall_cfg.red_hue_hi_deg = cfg.purple_hue_hi_deg;
  wall_cfg.red_chroma_ratio_min = cfg.purple_chroma_ratio_min;
  wall_cfg.red_L_min = cfg.purple_L_min;
  // 红底对应 z=0，隔断顶面与远墙顶边处在同一高度；按本帧顶边放大率比较实物宽度。
  wall_cfg.black_width_m *= projection.scale;
  wall_cfg.black_row_lo = 0;
  wall_cfg.black_row_hi = 1;
  const auto mark = measureSplit(wall, wall_cfg, cv::Mat(), nullptr, true);
  if (!mark.valid) {return false;}
  auto aligned = [&](double x, int y) {
      const double ground_x = ground.top_x + (ground.bottom_x - ground.top_x) * y / (h - 1);
      return std::abs(x - ground_x) <= 0.5 * calibrated_width;
    };
  return aligned(mark.top_x, y0) && aligned(mark.bottom_x, y1 - 1);
}

// 从鸟瞰图近边向远查找连续的紫墙；没有这种跨画宽的实测遮挡，不改变取样范围。
int visibleFloorEnd(const cv::Mat & bev, const SplitWindow & window, const FieldConfig & cfg)
{
  const auto strip = bev.colRange(window.window.x, window.window.x + window.window.width);
  cv::Mat lab, purple, interior;
  cv::cvtColor(strip, lab, cv::COLOR_BGR2Lab);
  segmentLab(lab, cfg, purple, interior);
  int end = purple.rows;
  while (end > 0 && cv::countNonZero(purple.row(end - 1)) >= kReferenceShare * purple.cols) {--end;}
  return end;
}

SplitMeasurement measureSplit(const cv::Mat & bev, const FieldConfig & cfg,
  const cv::Mat & source, const FieldResult * field, bool on_wall, bool require_ground_edges)
{
  SplitWindow window;
  if (!splitWindow(bev, cfg, window)) {return {};}
  SplitEvidence evidence;
  if (!splitEvidence(bev(window.context), cfg, evidence)) {return {};}
  cv::Mat eligible = cv::Mat::zeros(window.window.size(), CV_8U);
  std::vector<SplitRun> runs;
  std::vector<SplitEdge> edges;
  for (int y = 0; y < window.window.height; ++y) {
    rowRuns(evidence, window, cfg, y, runs, eligible.ptr<uchar>(y), edges);
  }
  const int min_rows = std::max(kMinRowsFloor, static_cast<int>(std::ceil(kMinRowsFrac * bev.rows)));
  const double calibrated_width = cfg.black_width_m * (bev.cols - 1) /
    (cfg.field_w_m - 2.0 * cfg.wall_thickness_m);
  weakBoundaryRuns(evidence, window, cfg, calibrated_width, edges, runs);
  // 地面与异色物块混色会形成 1–2 像素的低色度接缝，它也有两条材质边，
  // 但不是已标定宽度的黑带。只有一侧地面的段必须满足该物理宽度，
  // 两侧均为地面的完整带仍可用实测宽度处理浅灰/模糊后的边缘扩散。
  const double material_width = calibrated_width + 1.0;
  runs.erase(std::remove_if(runs.begin(), runs.end(), [&](const SplitRun & run) {
      return run.reference_side != 0 &&
        (run.width > kWidthSpread * material_width || run.width * kWidthSpread < material_width);
    }), runs.end());
  if (runs.empty()) {return {};}
  const WallProjection projection = field ? wallProjection(*field, cfg) : WallProjection{};
  const auto interior = interiorRows(evidence.reference, window);
  auto lines = candidateLines(runs, eligible, cfg, min_rows, interior, edges, calibrated_width, projection.scale);
  if (lines.empty() && !require_ground_edges && !on_wall &&
    interior.count * cfg.black_score_frac < min_rows) {
    const int floor_end = visibleFloorEnd(bev, window, cfg);
    // 近墙挡住大半地面时，按可见地面深度保留远端边距。仍须达到原有绝对行数，
    // 并且远近均有两侧地面的完整分界；单边、暗块与远墙自身不能利用扩窗。
    if (floor_end < bev.rows && (floor_end - window.window.y) * cfg.black_score_frac < min_rows) {
      auto visible = cfg;
      visible.black_row_lo *= double(floor_end) / bev.rows;
      if (int(visible.black_row_lo * bev.rows) < window.window.y) {
        return measureSplit(bev, visible, source, field, false, true);
      }
    }
  }
  if (lines.empty()) {return {};}
  // 中线应位于场地中部。过滤偏心暗条后，再在仍可能是真的分界之间判断歧义。
  const double bev_center = 0.5 * (bev.cols - 1);
  const double search_middle_y = 0.5 * (bev.rows - 1) - window.window.y;
  const double max_offset = cfg.black_max_offset_frac * bev.cols;
  lines.erase(std::remove_if(lines.begin(), lines.end(), [&](const SplitLine & line) {
      const double center = window.window.x + line.top + line.slope * search_middle_y;
      return std::abs(center - bev_center) > max_offset;
    }), lines.end());
  if (lines.empty()) {
    SplitMeasurement result;
    result.suspect = true;
    return result;
  }
  std::sort(lines.begin(), lines.end(),
    [](const SplitLine & a, const SplitLine & b) {return a.confidence > b.confidence;});
  if (lines.size() > 1 && lines[1].confidence >= kAmbiguityRatio * lines[0].confidence) {return {};}
  const auto & line = lines.front();
  if (require_ground_edges && !line.ground_edges_spanning) {return {};}
  // 先识别歧义，再用物理带宽排除细缝和宽暗条。计数包含两端像素，故标定宽度加一。
  const double width_tolerance = bandWidthTolerance(calibrated_width);
  // 窗内直线外推到 BEV 顶/底边，端点须落在鸟瞰图内。
  const int x0 = window.window.x, y0 = window.window.y;
  SplitMeasurement measurement;
  const double top = line.ground_edges_spanning ? line.edge_top : line.top;
  const double slope = line.ground_edges_spanning ? line.edge_slope : line.slope;
  if (std::abs(x0 + top + slope * search_middle_y - bev_center) > max_offset) {return {};}
  measurement.top_x = x0 + top - slope * y0;
  measurement.bottom_x = measurement.top_x + slope * (bev.rows - 1);
  if (!std::isfinite(measurement.top_x) || !std::isfinite(measurement.bottom_x) ||
    measurement.top_x < 0 || measurement.bottom_x < 0 ||
    measurement.top_x > bev.cols - 1 || measurement.bottom_x > bev.cols - 1) {return {};}
  if (std::abs(line.width - (calibrated_width + 1.0)) > width_tolerance) {
    if (!line.measured_core || line.width < calibrated_width + 1.0) {return {};}
    if (!on_wall && (!field ||
      !confirmedOnFarWall(source, *field, cfg, measurement, calibrated_width, projection))) {return {};}
  }
  measurement.valid = true;
  measurement.confidence = line.confidence;
  return measurement;
}

}  // namespace

SplitMeasurement detectSplitBlack(const cv::Mat & bev, const FieldConfig & cfg,
  const cv::Mat & source, const FieldResult * field)
{
  return measureSplit(bev, cfg, source, field, false);
}

}  // namespace field_detail
