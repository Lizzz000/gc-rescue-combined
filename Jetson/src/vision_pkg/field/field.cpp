// 阅读：场地搜索、跟踪和每帧 update。
// 颜色分割在 color.cpp，调试图在 debug.cpp，几何和评分在 opencv/。
// 不发布 ROS。消息转换在 node/ros_convert.hpp。
#include "field/field.hpp"
#include "field/opencv/algorithm.hpp"

// ---- 场地实现 ----
// 职责：安全区拟合、投影和跟踪。调试图在 debug.cpp。
// Copyright 2026 jetson_yolo integration
// SPDX-License-Identifier: Apache-2.0
// FieldLocator 的状态机实现：SEARCH/LOCATE/TRACK 迁移、候选搜索与评分调度、大跳变确认、证据不足保留历史几何。
// 搜索不提交历史；updateTracking() 决定提交、保持与丢失，buildOutput() 生成鸟瞰与本帧黑带实测。


#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <numeric>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <cstdint>
#include <functional>
#include <vector>
#if defined(__aarch64__) && defined(__ARM_NEON)
#include <arm_neon.h>
#endif

using namespace field_detail;

namespace {
// 距处理图边界不超过该像素数的轮廓点视为画面裁断点（与候选贴边计数同一口径）。
constexpr int kBorderPx = 1;
}  // namespace

void FieldLocator::reset()
{
  // 同时清掉角点、运动历史、待确认跳变与分界线，避免新会话沿用旧相机的坐标。
  state_ = "SEARCH";
  hits_ = lost_ = frame_idx_ = hold_frames_ = 0;
  has_corners_ = false;
  corners_.clear(); inner_.clear(); inner_measured_ = 0;
  last_good_.clear();
  prev_good_.clear();
  pending_.clear();
  pending_hits_ = 0;
}

// quad 已平移回处理图坐标；评分与实测边数仍来自该搜索区域，不能把偏移再加一次。
struct FieldLocator::SearchResult
{
  std::vector<cv::Point2f> quad;
  std::string reason, quality = "none";
  std::pair<double, double> field_size_px{0, 0};  // first=长边、second=短边，处理图像素。
  CandidateScore score;
  EdgeState edges[4] = {EdgeState::LOST, EdgeState::LOST, EdgeState::LOST, EdgeState::LOST};
  int measured = 0;
  // 与 quad 同一搜索图拟合的内沿（地面）四边形，处理图坐标；内部颜色也在它上面计。
  std::vector<cv::Point2f> inner;
  int inner_measured = 0;
};

struct FieldLocator::FrameEvidence
{
  const cv::Mat & small;
  const std::vector<cv::Point2f> & predicted;
  bool tracking;
  int64 started;
  double ms_resize, ms_detect;
  cv::Mat full_purple, full_interior;

  // ROI 必须独立分割；只有需要全图回退、跳变检查或越界诊断时才生成全图双掩膜，单帧复用。
  const cv::Mat & fullPurple(const FieldConfig & config)
  {
    if (full_purple.empty()) {FieldLocator::segmentEvidence(small, config, full_purple, full_interior);}
    return full_purple;
  }
};

FieldLocator::SearchResult FieldLocator::searchRegion(
  FrameEvidence & evidence, const cv::Mat & img, cv::Point2f offset) const
{
  const auto & small = evidence.small;
  const auto & predicted = evidence.predicted;
  const bool tracking = evidence.tracking;
  SearchResult search_result;
  cv::Mat purple_raw, interior_mask;
  if (img.data == small.data && img.size() == small.size()) {
    purple_raw = evidence.fullPurple(cfg_);
    interior_mask = evidence.full_interior;
  } else {
    segmentEvidence(img, cfg_, purple_raw, interior_mask);
  }
  // 全零证据经过既定闭运算和膨胀仍全零，省去形态学及轮廓扫描；上层仍照常处理丢失和历史输出。
  // 纯核心 cfg() 允许直接改值而不校验：仅对支持范围启用快路，其余仍交给原 OpenCV 调用处理。
  const bool supported_morphology =
    cfg_.morph_close_ksize >= 1 && cfg_.morph_close_ksize <= 63 &&
    cfg_.morph_dilate_ksize >= 1 && cfg_.morph_dilate_ksize <= 63 &&
    cfg_.morph_close_iter >= 0 && cfg_.morph_close_iter <= 20 &&
    cfg_.morph_dilate_iter >= 0 && cfg_.morph_dilate_iter <= 20;
  if (supported_morphology && cv::countNonZero(purple_raw) == 0) {
    search_result.reason = "没有通过过滤的紫色候选";
    return search_result;
  }
  cv::Mat mask = morph(purple_raw, cfg_);
  // 阶段: 轮廓与候选 —— 取面积最大的若干连通域（含碎片并集凸包），按面积占比/长宽比/贴边数/填充率过滤后才进入拟合。
  std::vector<std::vector<cv::Point>> contours;
    // 保留整条轮廓采样；仅保留直线端点会使鲁棒边拟合缺少内点。
  cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);
  const double img_area = double(img.cols) * img.rows;
  // 反光胶带可能断成多个连通域；并集凸包只增加一个候选，仍须通过同样的颜色/边证据门槛。
  std::vector<size_t> component_order(contours.size());
  std::vector<double> component_areas;
  component_areas.reserve(contours.size() + 1);
  for (const auto & contour : contours) {component_areas.push_back(cv::contourArea(contour));}
  std::iota(component_order.begin(), component_order.end(), 0);
  std::sort(component_order.begin(), component_order.end(), [&](size_t a, size_t b) {
    return component_areas[a] > component_areas[b];
  });
  std::vector<cv::Point> joined;
  int joined_count = 0;
  for (size_t index : component_order) {
    if (joined_count >= 4 || component_areas[index] < cfg_.min_area_frac * img_area) {break;}
    joined.insert(joined.end(), contours[index].begin(), contours[index].end());
    ++joined_count;
  }
  if (joined_count > 1) {
    std::vector<cv::Point> hull;
    cv::convexHull(joined, hull);
    component_areas.push_back(cv::contourArea(hull));
    contours.push_back(std::move(hull));
  }
  struct Cand { double area; QuadEvidence evidence; };
  // 贴处理图边界的轮廓点是画面裁断边，不是场地实测边，只用于分边先验与整侧出画判断。
  auto clippedPoint = [&](const cv::Point & p) {
      const int x = p.x + int(offset.x), y = p.y + int(offset.y);
      return x <= kBorderPx || y <= kBorderPx || x >= small.cols - 1 - kBorderPx ||
             y >= small.rows - 1 - kBorderPx;
    };
  std::vector<Cand> cands;
  for (size_t contour_index = 0; contour_index < contours.size(); ++contour_index) {
    const auto & c = contours[contour_index];
    const double area = component_areas[contour_index];
    if (area < cfg_.min_area_frac * img_area) {continue;}
    cv::RotatedRect rr = cv::minAreaRect(c);
    const double rw = rr.size.width, rh = rr.size.height;
    if (rw < 2 || rh < 2) {continue;}
    const double ratio = std::max(rw, rh) / std::max(std::min(rw, rh), 1e-6);
    if (ratio < cfg_.min_rect_ratio_lo || ratio > cfg_.min_rect_ratio_hi) {continue;}
    const cv::Rect bb = cv::boundingRect(c);
    // 人工 ROI 裁剪边不是场地实测边；候选被裁断时放弃 ROI 假设，退回全图搜索。
    if ((bb.x <= kBorderPx && offset.x > 0) || (bb.y <= kBorderPx && offset.y > 0) ||
      (bb.x + bb.width >= img.cols - kBorderPx && offset.x + img.cols < small.cols) ||
      (bb.y + bb.height >= img.rows - kBorderPx && offset.y + img.rows < small.rows)) {continue;}
    int touch = (bb.x <= kBorderPx) + (bb.y <= kBorderPx) + (bb.x + bb.width >=
      img.cols - kBorderPx) + (bb.y + bb.height >= img.rows - kBorderPx);
    if (touch > cfg_.max_border_touch) {continue;}
        // 凸包填充率: 紫框断裂成 C 形时"轮廓填充率"只有 0.28-0.34 (会把真场地筛掉),
        // 凸包仍然包住整个场地, 所以用凸包。
    std::vector<cv::Point> hull;
    cv::convexHull(c, hull);
    const double fill = cv::contourArea(hull) / std::max(rw * rh, 1e-6);
    if (fill < cfg_.min_fill_ratio) {continue;}
    Cand candidate; candidate.area = area;
    // 形态学只负责连通胶带；拟合回到连通域内原始紫色证据，不能把膨胀边当实测边。
    cv::Mat component = cv::Mat::zeros(bb.size(), CV_8U);
    std::vector<std::vector<cv::Point>> one{c};
    cv::drawContours(component, one, 0, cv::Scalar(255), cv::FILLED, cv::LINE_8,
      cv::noArray(), 0, -bb.tl());
    cv::bitwise_and(component, purple_raw(bb), component);
    std::vector<std::vector<cv::Point>> raw_contours;
    cv::findContours(component, raw_contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);
    for (const auto & raw : raw_contours) {
      for (const auto & p : raw) {
        const cv::Point q = p + bb.tl();
        (clippedPoint(q) ? candidate.evidence.clipped : candidate.evidence.pts).emplace_back(float(q.x), float(q.y));
      }
    }
    if (candidate.evidence.pts.size() < 4) {continue;}
    cands.push_back(std::move(candidate));
  }
  std::sort(cands.begin(), cands.end(), [](const Cand & u, const Cand & v) {
      return u.area > v.area;
                                                                                                   });

  if (cands.empty()) {
        // 区分"场地不在画面里"和"离得太近, 紫框已经贴满画面边缘":
        // 后者紫框方法本来就无法定位 (需要靠状态机/里程计兜底), 报出来更好排查。
    search_result.reason = "没有通过过滤的紫色候选";
    search_result.quality = "none";
    // 仅无候选诊断需要总紫像素与包围框；直接读取二值图，避免生成全部非零点。
    const int purple_px = cv::countNonZero(mask);
    if (purple_px > 0.02 * img_area) {
      const cv::Rect nb = cv::boundingRect(mask);
      const int touch = (nb.x <= kBorderPx) + (nb.y <= kBorderPx) +
        (nb.x + nb.width >= img.cols - kBorderPx) + (nb.y + nb.height >= img.rows - kBorderPx);
      if (touch >= 3) {
        search_result.reason = "紫框贴满画面边缘, 场地可能溢出画面 (此状态不做定位)";
        search_result.quality = "overflow";
      }
    }
    return search_result;
  }

    // 候选拟合：分边用候选自己的形状，缺边才用预测补；实现见 field_algorithms.cpp::fitQuad。
  std::string reasons;
  double best_total = -1.0;
  bool best_accepted = false;
  cv::Mat purple_probe;
  const int n_try = std::min<int>(std::max(1, cfg_.max_candidates_try), cands.size());
  for (int i = 0; i < n_try; ++i) {
    // 只用本帧证据拟合：缺边由候选自己的凸包先验补，不借历史预测（预测只决定搜索窗与跳变检查）。
    FitOut fit_result = fitQuad(cands[i].evidence, cfg_);
    if (!fit_result.ok) {if (!reasons.empty()) {reasons += "; ";} reasons += fit_result.reason; continue;}
    std::vector<cv::Point2f> q = fit_result.quad;
    for (auto & p : q) {
      p += offset;
    }
    std::string why;
    if (!checkQuad(q, img.size(), cfg_, why)) {
      if (!reasons.empty()) {reasons += "; ";}
      reasons += "候选被合理性检查拒绝: " + why;
      continue;
    }
        // 加候选来源或评分项：新候选要接进 cands 并走同一套排序与准入，新评分项要补 CandidateScore 字段、w_* 权重与门限。
        // ---- 多特征评分 (在这一层, 而不是"谁面积大选谁") ----
    // 首个真正进入评分的候选才需要探测图；无候选或拟合全部失败时省去整图膨胀。
    // 轻度膨胀补偿胶带厚度与拟合外缘的偏差，同一搜索区域内的后续候选共用此图。
    if (purple_probe.empty()) {
      const cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, {3, 3});
      cv::dilate(purple_raw, purple_probe, kernel, cv::Point(-1, -1), 2);
    }
    CandidateScore score;
    scoreEdges(purple_probe, fit_result.quad, cfg_, score);
    // 内部颜色只在内沿地面四边形里计：外沿四边形还含围边立面，远/低视角时围边占去大半。
    const InnerQuad inner = fitInnerQuad(interior_mask, fit_result.quad, cfg_);
    if (inner.measured_edges == 0) {
      if (!reasons.empty()) {reasons += "; ";}
      reasons += "没有可测的内沿";
      continue;
    }
    score.red_inside = redInsideGrid(interior_mask, inner.quad, cfg_, score.cells_red);
    score.red_outside = redOutsideRing(interior_mask, fit_result.quad, cfg_);
    score.crosses_interior = crossesInterior(interior_mask, purple_probe, fit_result.quad, cfg_);
    score.geometry = geometryScore(fit_result.quad, img.size(), cfg_);
    if (tracking) {
      const std::vector<cv::Point2f> pred_local = [&] {
          std::vector<cv::Point2f> v = predicted;
          for (auto & p : v) {
            p -= offset;
          }
          return v;
        }();
      const double jump = quadJumpRatio(fit_result.quad, pred_local);
      score.temporal = 1.0 - std::min(1.0, jump / std::max(1e-6, cfg_.max_jump_frac));
    } else {
      score.temporal = 0.5;         // 没有历史: 给中性分, 不奖励也不惩罚
    }
    finalizeScore(score, cfg_);
    // 通过硬门槛的候选始终优先；同为通过或同为拒绝时才比较总分，拒绝者可留下供诊断。
    const bool accepted = rejectCandidate(score, fit_result.measured_edges, cfg_, tracking).empty();
    if ((accepted && !best_accepted) || (accepted == best_accepted && score.total > best_total)) {
      best_accepted = accepted;
      best_total = score.total;
      search_result.quad = q;
      search_result.inner = inner.quad;
      for (auto & p : search_result.inner) {p += offset;}
      search_result.inner_measured = inner.measured_edges;
      search_result.score = score;
      search_result.measured = fit_result.measured_edges;
      for (int k = 0; k < 4; ++k) {
        search_result.edges[k] = fit_result.edge_state[k];
      }
      search_result.quality = fit_result.used_prior_fallback ? "prior_fallback" :
        (fit_result.measured_edges == 4 ? "good" : "degraded");
    }
  }
  if (search_result.quad.empty()) {search_result.reason = reasons.empty() ? "拟合失败" : reasons;}
  search_result.field_size_px = search_result.quad.empty() ? std::make_pair(0.0, 0.0) : quadSize(search_result.quad);
  return search_result;

}

FieldLocator::SearchResult FieldLocator::findCandidate(
  FrameEvidence & evidence, bool force_full) const
{
  const auto & small = evidence.small;
  const auto & predicted = evidence.predicted;
  const bool tracking = evidence.tracking;
  std::optional<SearchResult> full_search;
  std::vector<cv::Point2f> quad;
  std::string reason, quality = "none";
  std::pair<double, double> field_size_px{0, 0};  // first=长边、second=短边，处理图像素。
  CandidateScore best_score;
  EdgeState best_edges[4] = {EdgeState::LOST, EdgeState::LOST, EdgeState::LOST, EdgeState::LOST};
  int best_measured = 0;
  std::vector<cv::Point2f> inner;
  int inner_measured = 0;

  auto window = [&](const std::vector<cv::Point2f> & ref, double scale) {
      float x0 = 1e9f, y0 = 1e9f, x1 = -1e9f, y1 = -1e9f;
      for (const auto & p : ref) {
        x0 = std::min(x0, p.x); y0 = std::min(y0, p.y);
        x1 = std::max(x1, p.x); y1 = std::max(y1, p.y);
      }
      const float ex = (scale - 1.f) * 0.5f * (x1 - x0), ey = (scale - 1.f) * 0.5f * (y1 - y0);
      // 预测可能整体移出画面；两端都裁到图内，避免倒置端点被 Rect 重排成越界正面积。
      // 无交集得到空窗口，后续照常走全图回退；有交集时保留原外扩及整数截断结果。
      return cv::Rect(cv::Point(std::clamp(int(x0 - ex), 0, small.cols),
                               std::clamp(int(y0 - ey), 0, small.rows)),
                      cv::Point(std::clamp(int(x1 + ex), 0, small.cols),
                                std::clamp(int(y1 + ey), 0, small.rows)));
    };

  // 平时围绕预测搜索；定期全图重拟合，或 ROI 无可接受候选时退回全图，避免裁剪边缘成为假边界。
  if (tracking && !force_full) {
    cv::Rect win = window(predicted, cfg_.track_window_scale);
    if (win.width > 8 && win.height > 8) {
      SearchResult search_result = searchRegion(evidence, small(win), cv::Point2f(float(win.x), float(win.y)));
      // 跟踪窗口恰好覆盖全图时，这次搜索已是全图搜索；拒绝后复用它，避免重复拟合与评分。
      if (win == cv::Rect(0, 0, small.cols, small.rows)) {full_search = search_result;}
      quad = search_result.quad; reason = search_result.reason; quality = search_result.quality;
      inner = search_result.inner; inner_measured = search_result.inner_measured;
      field_size_px = search_result.field_size_px;
      best_score = search_result.score; best_measured = search_result.measured;
      for (int k = 0; k < 4; ++k) {
        best_edges[k] = search_result.edges[k];
      }
    }
  }
  // 回退按字段合并：全图有几何才替换 ROI 候选及评分；无几何只补原因／质量，尺寸仍取全图结果。
  // 因而全图失败并不等于把 ROI 结果整体清空，不能用一次结构赋值替代下面的分支。
  if (quad.empty() || !rejectCandidate(best_score, best_measured, cfg_, tracking).empty()) {
    SearchResult search_result = full_search ? std::move(*full_search) :
      searchRegion(evidence, small, cv::Point2f(0, 0));
    if (!search_result.quad.empty()) {
      quad = search_result.quad; reason.clear(); quality = search_result.quality;
      inner = search_result.inner; inner_measured = search_result.inner_measured;
      best_score = search_result.score; best_measured = search_result.measured;
      for (int k = 0; k < 4; ++k) {
        best_edges[k] = search_result.edges[k];
      }
    } else if (reason.empty()) {
      reason = search_result.reason;
    }
    if (quality == "none" && !search_result.quality.empty()) {quality = search_result.quality;}
    field_size_px = search_result.field_size_px;
  }

  SearchResult result;
  result.quad = std::move(quad);
  result.reason = std::move(reason);
  result.quality = std::move(quality);
  result.field_size_px = field_size_px;
  result.score = best_score;
  result.measured = best_measured;
  result.inner = std::move(inner);
  result.inner_measured = inner_measured;
  for (int edge = 0; edge < 4; ++edge) {result.edges[edge] = best_edges[edge];}
  return result;
}

// 帧准备、状态更新与诊断输出分别处理；update() 保留整帧调用顺序。
bool FieldLocator::prepareFrame(
  const cv::Mat & frame, cv::Mat & small, FieldResult & res)
{
  if (frame.empty()) {res.reason = "空帧"; return false;}
  if (frame.type() != CV_8UC3) {res.reason = "Expected BGR8 frame"; return false;}

  const int pw = cfg_.proc_width;
  const int ph = cfg_.proc_height > 0 ? cfg_.proc_height :
    static_cast<int>(std::lround(frame.rows * double(pw) / frame.cols));
  if (pw < 16 || pw > 4096 || ph < 16 || ph > 4096) {
    res.reason = "Processing dimensions outside 16..4096"; return false;
  }
  // 后续角点、预测与搜索窗口统一在 small 坐标。尺寸相同时只借用 frame 的只读视图，不改输入像素。
  if (frame.cols == pw && frame.rows == ph) {small = frame;} else {
    small = resizeForField(frame, {pw, ph});
  }
  ++frame_idx_;
  return true;
}

void FieldLocator::buildOutput(
  FrameEvidence & evidence, SearchResult & candidate, FieldResult & res,
  bool ok, const std::string & quality, const std::string & why)
{
  const auto & small = evidence.small;
  auto & reason = candidate.reason;
  const auto & field_size_px = candidate.field_size_px;
  const auto t0 = evidence.started;
  const double ms_resize = evidence.ms_resize;
  const double ms_detect = evidence.ms_detect;
  res.ok = ok;
  res.quality = quality;
  if (!why.empty()) {reason += (reason.empty() ? "" : "; ") + why;}
  res.reason = reason;
  res.state = state_;
  if (has_corners_) {
    res.corners = inner_;
    res.outline = corners_;
    res.quad_area = polyArea(inner_);
  }
    // 未提交时保留上次接受的几何生成诊断 BEV；预测只帮助搜索，不在这里替换历史角点。
    // BEV 用内沿四边形：鸟瞰里只有内部地面，没有围边立面。
  if (has_corners_) {
    res.inner_measured_edges = inner_measured_;
    const cv::Size bs = bevSize(cfg_);
    std::vector<cv::Point2f> dst{cv::Point2f(0, 0), cv::Point2f(bs.width - 1.f, 0),
      cv::Point2f(bs.width - 1.f, bs.height - 1.f), cv::Point2f(0, bs.height - 1.f)};
    res.homography = cv::getPerspectiveTransform(inner_, dst);
    cv::warpPerspective(small, res.bev, res.homography, bs);
    cv::invert(res.homography, res.homography_inv);
    res.ms_warp = (cv::getTickCount() - t0) * 1000.0 / cv::getTickFrequency() - ms_resize -
      ms_detect;
    res.red_ratio = redRatioInBev(res.bev, cfg_);
    const auto split = detectSplitBlack(res.bev, cfg_);
    res.split_valid = split.valid;
    res.split_conf = split.confidence;
    if (split.valid) {
      res.split_measured = (split.top_x + split.bottom_x) * 0.5;
      res.split_suspect = std::abs(res.split_measured - res.bev.cols / 2.0) >
        cfg_.black_max_offset_frac * res.bev.cols;
      // 逐帧实测，不做跨帧平滑：跨帧保持在桥接层按时间处理。
      res.split_top_x = split.top_x; res.split_bottom_x = split.bottom_x;
      res.split_x = (split.top_x + split.bottom_x) * 0.5;
    } else if (ok && res.committed) {
      // 没有完整黑带时不能把中线先验冒充本帧左右区域。
      res.reason += (res.reason.empty() ? "" : "; ") + std::string("黑带证据不足, 不输出左右分区");
    }
    res.approx_field_h_px = field_size_px.second;
  }
  res.ms_resize = ms_resize;
  res.ms_detect = ms_detect;
  const double ms_now = (cv::getTickCount() - t0) * 1000.0 / cv::getTickFrequency();
  res.ms_total = ms_now;
  res.ms_split = std::max(0.0, ms_now - ms_resize - ms_detect - res.ms_warp);

}

void FieldLocator::updateTracking(
  FrameEvidence & evidence, SearchResult & candidate, FieldResult & res)
{
  auto & quad = candidate.quad;
  auto & reason = candidate.reason;
  auto & quality = candidate.quality;
  const auto & best_score = candidate.score;
  const int best_measured = candidate.measured;
  const auto & predicted = evidence.predicted;
  const bool tracking = evidence.tracking;
  const auto & small = evidence.small;
  res.score = candidate.score;
  res.measured_edges = candidate.measured;
  for (int edge = 0; edge < 4; ++edge) {res.edge_state[edge] = candidate.edges[edge];}
  if (quad.empty()) {
    // 缺少候选即中断连续确认；保留历史几何不代表本帧提供了跳变证据。
    pending_.clear(); pending_hits_ = 0;
    // 没有候选按 lost_ 老化；阈值前可输出历史几何但 ok=false，到阈值后清空历史并回 SEARCH。
    // 阈值前保留 hits_：历史仍有效时 TRACK 不应因一次丢失退回 LOCATE；清历史时才一起归零。
    ++lost_;
    if (lost_ >= cfg_.lost_frames_to_search) {
      state_ = "SEARCH"; has_corners_ = false; corners_.clear(); inner_.clear(); inner_measured_ = 0;
      last_good_.clear(); prev_good_.clear();
      lost_ = 0; hits_ = 0; hold_frames_ = 0;
    } else {
      // 只有达到定位命中门槛才报 TRACK；否则保持 LOCATE，避免绕过 locate_hits_needed。
      state_ = has_corners_ ? (hits_ >= cfg_.locate_hits_needed ? "TRACK" : "LOCATE") : "SEARCH";
    }
    res.committed = false;
    buildOutput(evidence, candidate, res, false, quality, reason.empty() ? "未检测到紫框" : "");
    return;
  }

    // 与上一帧预测的跳变
  if (tracking) {res.jump_ratio = quadJumpRatio(quad, predicted);}

    // (a) 大跳变: 不立刻切换, 先挂起, 连续 N 帧确认再切 (误检通常只持续一帧)
  const std::string fail = rejectCandidate(best_score, best_measured, cfg_, tracking);
  if (fail.empty() && tracking && res.jump_ratio > cfg_.max_jump_frac) {
        // 例外: 当前跟踪的几何已经没有紫框证据 (说明跟丢了), 而新候选非常干净
        // -> 允许立即切换, 否则会一直抱着一个假四边形不放。
    CandidateScore cur;
    {
            // 需要全画面的紫掩膜来评估"当前跟踪的几何" (ROI 得到的掩膜带偏移, 不能用)
      scoreEdges(evidence.fullPurple(cfg_), corners_, cfg_, cur);
    }
    const bool cur_is_bad = !has_corners_ || cur.purple < cfg_.purple_lost_threshold;
    const bool new_is_clean = best_score.purple >= cfg_.fast_switch_purple &&
      best_score.red_inside >= cfg_.fast_switch_red_inside &&
      best_score.red_outside <= cfg_.fast_switch_red_outside;
    const bool same_as_pending = (pending_.size() == 4 && quadJumpRatio(quad,
      pending_) < cfg_.jump_accept_frac);
    pending_ = quad;
    // 确认比较的是连续候选之间的距离；候选换位置就重新计数，不累计互不相干的跳变。
    pending_hits_ = same_as_pending ? pending_hits_ + 1 : 1;
    if (cur_is_bad && new_is_clean) {
      pending_hits_ = std::max(pending_hits_, cfg_.switch_confirm_frames);
    }
    if (pending_hits_ >= std::max(2, cfg_.switch_confirm_frames)) {
            // 正式切换后清掉旧速度和分界线，避免下一帧从旧位置继续外推或沿用旧 BEV 分区。
      prev_good_.clear();
      commitGeometry(candidate);
      last_good_ = corners_;
      lost_ = 0; hold_frames_ = 0; hits_ = std::max(hits_, cfg_.locate_hits_needed);
      state_ = "TRACK";
      pending_.clear(); pending_hits_ = 0;
      res.committed = true; res.switched = true;
      buildOutput(evidence, candidate, res, true, quality, "大跳变已连续确认, 切换到新位置");
      return;
    }
    // 跳变待确认不消耗普通 hold_frames_／lost_ 预算，也不把候选写入运动历史。
    res.committed = false;
    buildOutput(evidence, candidate, res, false, "hold", "候选与预测差太多, 等待连续确认");
    return;
  } else {
    pending_.clear();
    pending_hits_ = 0;
  }

    // (b) 多特征硬门槛: 紫框证据 / 内部红色 / 外部不是红色
  if (!fail.empty()) {
    res.candidate_corners = quad;          // 调试: 把被拒的最佳候选也带出去
    res.hold_reason = fail;
        // 场地太近导致紫框贴满画面时, 直接报 overflow (这是"超出方法适用范围",
        // 不是算法失败), 交给上层用里程计/保持上次几何处理。
    {
      const cv::Mat & pm = evidence.fullPurple(cfg_);
      const int px = cv::countNonZero(pm);
      if (px > 0.02 * small.cols * small.rows) {
        const cv::Rect nb = cv::boundingRect(pm);
        const int touch = (nb.x <= kBorderPx) + (nb.y <= kBorderPx) +
          (nb.x + nb.width >= small.cols - kBorderPx) + (nb.y + nb.height >= small.rows - kBorderPx);
        if (touch >= 3) {
          res.committed = false;
          buildOutput(evidence, candidate, res, false, "overflow", "紫框贴满画面边缘, 场地可能溢出画面 (此状态不做定位)");
          return;
        }
      }
    }
        // 本次最佳候选的紫框证据极弱时立即使历史几何失效，不再进入普通的保持计数。
    if (has_corners_ && best_score.purple < cfg_.purple_lost_threshold) {
      state_ = "SEARCH";
      has_corners_ = false;
      corners_.clear(); inner_.clear(); inner_measured_ = 0;
      last_good_.clear();
      prev_good_.clear();
      lost_ = 0; hits_ = 0; hold_frames_ = 0;
      res.committed = false;
      buildOutput(evidence, candidate, res, false, "none", fail + " (当前几何已无紫框证据, 回到 SEARCH)");
      return;
    }
    // 有拟合候选但未过硬门槛按 hold_frames_ 计数；与“没有候选”的 lost_ 路径分开。
    // buildOutput 输出的是已接受的 corners_，不沿用本帧被拒候选，也不使用预测角点。
    ++hold_frames_;
    res.committed = false;
    if (hold_frames_ > cfg_.max_hold_frames) {
      state_ = "SEARCH";
      has_corners_ = false;
      corners_.clear(); inner_.clear(); inner_measured_ = 0;
      last_good_.clear();
      prev_good_.clear();
      lost_ = 0; hits_ = 0; hold_frames_ = 0;
      buildOutput(evidence, candidate, res, false, "none", fail + " 且保持超时, 回到 SEARCH");
    } else {
      // 普通拒绝不推进命中数；TRACK 仍要求已越过定位命中门槛，LOCATE 不能提前升级。
      state_ = has_corners_ ?
        (hits_ >= cfg_.locate_hits_needed ? "TRACK" : "LOCATE") : state_;
      buildOutput(evidence, candidate, res, false, "hold", fail + " (保持已接受几何)");
    }
    return;
  }

    // (c) 提交本帧实测角点（不做跨帧平滑，避免叠加 STM32 滤波的滞后），再写入成功历史；命中次数决定 LOCATE/TRACK。
  commitGeometry(candidate);
  // 只有正常提交走到这里；上面的保持/等待分支均提前返回，不能污染运动预测历史。
  prev_good_ = last_good_;
  last_good_ = corners_;
  ++hits_; lost_ = 0; hold_frames_ = 0;
  state_ = hits_ >= cfg_.locate_hits_needed ? "TRACK" : "LOCATE";
  res.committed = true;
  res.ok = true;
  res.quality = quality == "prior_fallback" ? quality : (best_measured == 4 ? "good" : "degraded");
  buildOutput(evidence, candidate, res, true, res.quality, reason);
  return;

}

void FieldLocator::commitGeometry(const SearchResult & candidate)
{
  corners_ = candidate.quad;
  inner_ = candidate.inner;
  inner_measured_ = candidate.inner_measured;
  has_corners_ = true;
}

FieldResult FieldLocator::update(const cv::Mat & frame)
{
  const int64 t0 = cv::getTickCount();
  FieldResult res;
  res.state = state_;
  cv::Mat small;
  if (!prepareFrame(frame, small, res)) {return res;}
  const double ms_resize = (cv::getTickCount() - t0) * 1000.0 / cv::getTickFrequency();
  // 运动预测仅使用已接受的几何；失败帧不能写入成功历史。
  const auto predicted = predictQuad();
  const bool tracking = has_corners_ && predicted.size() == 4;
  FrameEvidence evidence{small, predicted, tracking, t0, ms_resize, 0.0, {}, {}};
  // refit_every 按通过帧准备的累计帧数计，包括之后未通过候选门槛的帧，不按成功提交次数计。
  auto candidate = findCandidate(evidence, frame_idx_ % std::max(1, cfg_.refit_every) == 0);
  evidence.ms_detect = (cv::getTickCount() - t0) * 1000.0 / cv::getTickFrequency() - ms_resize;
  updateTracking(evidence, candidate, res);
  return res;
}

// 上一帧运动趋势预测: last + gain*(last - prev)
std::vector<cv::Point2f> FieldLocator::predictQuad() const
{
  if (last_good_.size() != 4) {return corners_;}
  if (prev_good_.size() != 4) {return last_good_;}
  const float g = static_cast<float>(cfg_.pred_velocity_gain);
  std::vector<cv::Point2f> pred(4);
  for (int i = 0; i < 4; ++i) {
    pred[i] = last_good_[i] + (last_good_[i] - prev_good_[i]) * g;
  }
  return pred;
}


// 调试图在 debug.cpp。紫框和内部颜色分割在 color.cpp。
