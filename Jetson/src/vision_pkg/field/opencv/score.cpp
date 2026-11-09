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
// 黑带以配置的内部行区间为证据范围，不要求延伸到紫边或整幅 BEV 顶底。
// 连通暗带须贯穿该区间、宽度受限且中心近似直线；不完整、歧义或大片暗区不产生分区。
namespace field_detail {
namespace {

/** @brief 在给定灰度窗内测量黑带，并返回合格连通域数量以区分缺证据与歧义。 */
std::pair<SplitMeasurement, int> measureSplitBlack(
  const cv::Mat & gray, int x0, int y0, int bw, int bh, const FieldConfig & cfg)
{
  const int rows = gray.rows, cols = gray.cols;
  size_t histogram[256] = {};
  for (int y = 0; y < rows; ++y) {
    const auto * row = gray.ptr<uchar>(y);
    for (int x = 0; x < cols; ++x) {++histogram[row[x]];}
  }
  size_t count = 0;
  int median = 0;
  for (; median < 255; ++median) {
    count += histogram[median];
    if (count > gray.total() / 2) {break;}
  }
  if (median < 20) {return {};}
  // 相对下陷是主判据，不能把旧备用的强暗像素门槛当成所有黑带的必需条件。
  // 中灰黑带仍须满足唯一、覆盖、宽度和直线残差；主判据的多候选歧义禁止由备用掩盖。
  auto measure = [&](double threshold) -> std::pair<SplitMeasurement, int> {
    SplitMeasurement measurement;
    cv::Mat dark, labels, stats, centroids;
    cv::compare(gray, threshold, dark, cv::CMP_LT);
    const int components = cv::connectedComponentsWithStats(dark, labels, stats, centroids, 8, CV_32S);
    struct RowSpan {int left, right = -1, count = 0; double gray_sum = 0;};
    struct Band {std::vector<RowSpan> spans;};
    std::vector<int> selected(components, -1);
    std::vector<Band> bands;
    for (int label = 1; label < components; ++label) {
      const int top = stats.at<int>(label, cv::CC_STAT_TOP);
      const int height = stats.at<int>(label, cv::CC_STAT_HEIGHT);
      const int left = stats.at<int>(label, cv::CC_STAT_LEFT);
      const int width = stats.at<int>(label, cv::CC_STAT_WIDTH);
      // 只要求跨内部区间的上/下六分之一；横向裁断的暗块不能证明其真实宽度。
      if (top > rows / 6 || top + height < rows - rows / 6 || left == 0 || left + width == cols) {continue;}
      selected[label] = static_cast<int>(bands.size());
      bands.push_back({std::vector<RowSpan>(rows, RowSpan{cols})});
    }
    if (bands.empty()) {return {measurement, 0};}
    // 一次扫描分配行证据，避免逐个暗连通域反复扫描整幅图；总工作量与 BEV 像素数成正比。
    for (int y = 0; y < rows; ++y) {
      const auto * ids = labels.ptr<int>(y);
      const auto * pixels = gray.ptr<uchar>(y);
      for (int x = 0; x < cols; ++x) {
        const int index = selected[ids[x]];
        if (index < 0) {continue;}
        auto & span = bands[index].spans[y];
        span.left = std::min(span.left, x); span.right = std::max(span.right, x);
        ++span.count; span.gray_sum += pixels[x];
      }
    }
    int accepted = 0;
    for (const auto & band : bands) {
      std::vector<cv::Point2f> centers;
      std::vector<double> widths;
      double gray_sum = 0, pixel_count = 0;
      int thirds[3] = {};
      for (int y = 0; y < rows; ++y) {
        const auto & span = band.spans[y];
        const int width = span.right - span.left + 1;
        if (span.count == 0 || width > cfg.black_max_width_frac * bw) {continue;}
        // 同行有大空洞说明连接了多个物体，不能用两端的均值虚构黑带中心。
        if (span.count < 0.8 * width) {continue;}
        centers.emplace_back(float(x0 + (span.left + span.right) * 0.5), float(y0 + y));
        widths.push_back(width); gray_sum += span.gray_sum; pixel_count += span.count;
        ++thirds[std::min(2, y * 3 / rows)];
      }
      // 三段均须有支持，总行覆盖至少 3/4；短黑物体或局部阴影不能替代完整分界。
      if (centers.size() < std::max(0.75, cfg.black_score_frac) * rows ||
        *std::min_element(thirds, thirds + 3) < rows / 6) {continue;}
      cv::Vec4f line;
      cv::fitLine(centers, line, cv::DIST_HUBER, 0, 0.01, 0.01);
      if (!std::isfinite(line[1]) || std::abs(line[1]) < 1e-6) {continue;}
      const double slope = double(line[0]) / line[1];
      const double top = line[2] - slope * line[3];
      const double bottom = top + slope * (bh - 1);
      if (!std::isfinite(top) || !std::isfinite(bottom) || top < 0 || bottom < 0 || top > bw - 1 || bottom > bw - 1) {continue;}
      std::vector<double> residuals;
      residuals.reserve(centers.size());
      for (const auto & point : centers) {residuals.push_back(std::abs(point.x - top - slope * point.y));}
      const auto tail = residuals.begin() + residuals.size() * 9 / 10;
      std::nth_element(residuals.begin(), tail, residuals.end());
      if (*tail > std::max(1.5, upperMedian(widths) * 0.5)) {continue;}
      const double depth = (median - gray_sum / pixel_count) / median;
      if (depth < cfg.black_min_depth) {continue;}
      measurement.valid = true; measurement.top_x = top; measurement.bottom_x = bottom;
      measurement.confidence = std::min(1.0, depth / cfg.black_conf_depth) * double(centers.size()) / rows;
      ++accepted;
    }
    if (accepted != 1) {measurement = {};}
    return {measurement, accepted};
  };
  const double contrast_threshold = std::min(cfg.black_abs_gray,
    std::max(20.0, median * (1.0 - cfg.black_min_depth)));
  const auto contrast = measure(contrast_threshold);
  if (contrast.second != 0) {return contrast;}
  const double dark_threshold = std::min(cfg.black_abs_gray,
    std::max(20.0, median * cfg.black_rel_gray));
  if (dark_threshold == contrast_threshold) {return {};}
  return measure(dark_threshold);
}

/** @brief 从配置搜索窗内取唯一足够高的双侧内部色连续行段，返回窗内行号。 */
cv::Range interiorRowRange(const cv::Mat & search, int bev_height, const FieldConfig & cfg)
{
  constexpr double min_side_fraction = 0.25;
  constexpr double min_height_fraction = 0.12;
  const int min_rows = std::max(8, static_cast<int>(std::ceil(min_height_fraction * bev_height)));
  const cv::Mat interior = FieldLocator::redMask(search, cfg);
  const int middle = interior.cols / 2;
  const auto has_color = [](uchar value) {return value != 0;};
  int start = 0;
  cv::Range selected;
  for (int y = 0; y <= interior.rows; ++y) {
    if (y < interior.rows) {
      const auto * row = interior.ptr<uchar>(y);
      const auto left = std::count_if(row, row + middle, has_color);
      const auto right = std::count_if(row + middle, row + interior.cols, has_color);
      if (left >= min_side_fraction * middle &&
        right >= min_side_fraction * (interior.cols - middle)) {continue;}
    }
    // 收去颜色边缘的不稳定行，再检查剩余高度是否足以把拟合线外推到鸟瞰图顶底。
    const int length = y - start;
    const int margin = length / 10;
    if (length - 2 * margin >= min_rows) {
      // 第二个足够高的行段意味着采样区域有歧义；满窗也没有新的颜色边界。
      if (!selected.empty() || length == interior.rows) {return {};}
      selected = cv::Range(start + margin, y - margin);
    }
    start = y + 1;
  }
  return selected;
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
  if (bev.empty() || bev.type() != CV_8UC3 || x1 - x0 < 8 || y1 - y0 < 8) {return {};}
  const cv::Mat search = bev(cv::Rect(x0, y0, x1 - x0, y1 - y0));
  cv::Mat gray;
  cv::cvtColor(search, gray, cv::COLOR_BGR2GRAY);
  const auto full = measureSplitBlack(gray, x0, y0, bw, bh, cfg);
  // 宽窗已确认结果或发现歧义时直接返回。只有两种灰度阈值都缺证据才尝试颜色窗。
  if (full.second != 0) {return full.first;}
  const auto range = interiorRowRange(search, bh, cfg);
  if (range.empty()) {return {};}
  return measureSplitBlack(gray.rowRange(range), x0, y0 + range.start, bw, bh, cfg).first;
}

}  // namespace field_detail
