// 调试图和配置摘要。搜索状态机在 field.cpp，颜色分割在 color.cpp。

#include "field/field.hpp"
#include "field/opencv/algorithm.hpp"

// Copyright 2026 jetson_yolo integration
// SPDX-License-Identifier: Apache-2.0


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

// 调试图输出：drawDebug() 叠加四边形、被拒候选与状态文字；asciiReason() 把中文原因压成 putText 能画的 ASCII 短标签。
// 新增叠加元素或调试图尺寸 -> drawDebug()；新增拒绝原因 -> field_algorithms.cpp 的 rejectCandidate() 文案与 asciiReason() 映射表。

namespace field_detail {

// 中文原因 -> 短 ASCII 标签。cv::putText 画不了中文 (会变成方块/问号),
// 所以调试图里统一用 ASCII; 控制台输出仍然保留中文。
std::string asciiReason(const std::string & r)
{
  struct Map { const char * cn; const char * en; };
  static const Map kMap[] = {
    {"紫框边证据不足", "purple-weak"},
    {"内部红色不足", "red-inside-low"},
    {"内部红色格数不足", "red-cells-low"},
    {"四边形外部也是红色", "red-outside-high"},
    {"紫色边支持度合格的边少于 2 条", "edges<2"},
    {"只有 2 条边有证据", "only-2-edges"},
    {"只有 1 条边测到", "only-1-edge"},
    {"总分过低", "score-low"},
    {"紫框贴满画面边缘", "overflow"},
    {"没有通过过滤的紫色候选", "no-candidate"},
    {"四条边都没有足够像素", "no-edge-pixels"},
    {"未检测到紫框", "no-field"},
    {"黑带位置离中心太远", "black-off-center"},
  };
  if (r.empty()) {return "";}
  for (const auto & m : kMap) {
    if (r.find(m.cn) != std::string::npos) {
      std::string out = m.en;
      if (r.find("保持") != std::string::npos) {out += "+hold";}
      if (r.find("连续确认") != std::string::npos) {out += "+pending";}
      if (r.find("超时") != std::string::npos) {out += "+timeout";}
      return out;
    }
  }
  std::string out;                         // 兜底: 丢掉所有非 ASCII 字符
  for (unsigned char c : r) {
    if (c < 128) {
      out.push_back(static_cast<char>(c));
    }
  }
  while (!out.empty() && out.back() == ' ') {out.pop_back();}
  return out.empty() ? std::string("rejected") : out;
}

}  // namespace field_detail

using namespace field_detail;

// 阶段: 调试输出 —— 上联原图叠加四边形与被拒候选，下联鸟瞰图叠加分区线；中文原因转 ASCII 短标签（putText 画不了中文）。
cv::Mat drawDebug(const cv::Mat & small, const FieldResult & res)
{
  cv::Mat vis = small.clone();
    // 被拒绝的候选用橙色细线画出来 (排查"为什么这帧没输出"最直观)
  if (!res.committed && res.candidate_corners.size() == 4) {
    std::vector<cv::Point> poly;
    for (const auto & p : res.candidate_corners) {
      poly.emplace_back(cvRound(p.x), cvRound(p.y));
    }
    const cv::Point * pts[1] = {poly.data()};
    int n = 4;
    cv::polylines(vis, pts, &n, 1, true, cv::Scalar(0, 140, 255), 1);
    char tag[128];
    std::snprintf(tag, sizeof(tag), "rejected: meas=%d p=%.2f ri=%.2f ro=%.2f",
                          res.measured_edges, res.score.purple, res.score.red_inside,
      res.score.red_outside);
    cv::putText(vis, tag, {6, 44}, cv::FONT_HERSHEY_SIMPLEX, 0.4,
                        cv::Scalar(0, 140, 255), 1, cv::LINE_AA);
  }
  if (res.corners.size() == 4) {
    std::vector<cv::Point> poly;
    for (const auto & p : res.corners) {
      poly.emplace_back(cvRound(p.x), cvRound(p.y));
    }
    const cv::Point * pts[1] = {poly.data()};
    int n = 4;
    const cv::Scalar color = res.ok && res.committed ? cv::Scalar(0, 255, 0) : cv::Scalar(150, 150, 150);
    cv::polylines(vis, pts, &n, 1, true, color, 2);
    for (size_t i = 0; i < poly.size(); ++i) {
      cv::circle(vis, poly[i], 3, cv::Scalar(0, 200, 255), -1);
      cv::putText(vis, "P" + std::to_string(i + 1), poly[i] + cv::Point(5, -5),
                        cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(0, 200, 255), 1, cv::LINE_AA);
    }
  }
  char buf[256];
  std::snprintf(buf, sizeof(buf), "%s ok=%d q=%s split=%.0f conf=%.2f red=%.2f h=%.0fpx",
                  res.state.c_str(), int(res.ok), res.quality.c_str(), res.split_x,
                  res.split_conf, res.red_ratio, res.approx_field_h_px);
  cv::putText(vis, buf, {6, 16}, cv::FONT_HERSHEY_SIMPLEX, 0.42, cv::Scalar(255, 255, 255), 1,
    cv::LINE_AA);
  const std::string why = asciiReason(res.reason);
  if (!why.empty()) {
    cv::putText(vis, why.substr(0, 60), {6, 30}, cv::FONT_HERSHEY_SIMPLEX, 0.38,
                    cv::Scalar(0, 0, 255), 1, cv::LINE_AA);
  }
  if (!res.bev.empty()) {
    cv::Mat bev = res.bev.clone();
    const int x = cvRound(res.split_top_x);
    const int bottom_x = cvRound(res.split_bottom_x);
    if (res.split_valid) {
      cv::line(bev, {x, 0}, {bottom_x, bev.rows - 1}, cv::Scalar(0, 255, 255), 1);
      cv::putText(bev, "LEFT", {6, 18}, cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 1,
        cv::LINE_AA);
      cv::putText(bev, "RIGHT", {x + 6, 18}, cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 165, 255),
        1, cv::LINE_AA);
    }
    cv::resize(vis, vis, bev.size());
    cv::Mat out(vis.rows * 2, vis.cols, CV_8UC3);
    vis.copyTo(out(cv::Rect(0, 0, vis.cols, vis.rows)));
    bev.copyTo(out(cv::Rect(0, vis.rows, bev.cols, bev.rows)));
    return out;
  }
  return vis;
}

#include <cmath>
#include <cstdio>
#include <stdexcept>

// 只读诊断摘要；FieldConfig 值由统一装配注入，不从此文本恢复配置。
std::string configToString(const FieldConfig & cfg)
{
  char buf[2048];
  std::snprintf(buf, sizeof(buf),
                  "proc=%dx%d field=%.2fx%.2fm bev_ppm=%.0f "
                  "purple[H=%.0f..%.0f C/L>=%.2f L=%.0f..%.0f] "
                  "fill>=%.2f ratio=%.2f..%.2f corner_tol=%.2f black[depth>=%.2f suspect_offset>%.2f width<=%.2f lift<=%.2f] "
                  "interior[%s H=%.0f..%.0f C/L>=%.2f L>=%.0f]",
                  cfg.proc_width, cfg.proc_height, cfg.field_w_m, cfg.field_h_m, cfg.bev_px_per_m,
                  cfg.purple_hue_lo_deg, cfg.purple_hue_hi_deg, cfg.purple_chroma_ratio_min,
                  cfg.purple_L_min, cfg.purple_L_max,
                  cfg.min_fill_ratio, cfg.min_rect_ratio_lo, cfg.min_rect_ratio_hi,
                  cfg.prior_corner_tol_frac, cfg.black_min_depth, cfg.black_max_offset_frac,
                  cfg.black_max_width_frac, cfg.black_max_lift,
                  cfg.use_blue_interior ? "blue" : "red",
                  cfg.use_blue_interior ? cfg.blue_hue_lo_deg : cfg.red_hue_lo_deg,
                  cfg.use_blue_interior ? cfg.blue_hue_hi_deg : cfg.red_hue_hi_deg,
                  cfg.use_blue_interior ? cfg.blue_chroma_ratio_min : cfg.red_chroma_ratio_min,
                  cfg.use_blue_interior ? cfg.blue_L_min : cfg.red_L_min);
  return std::string(buf);
}

// 参数范围校验：越界或非有限值一律抛 std::invalid_argument，调用方在启动阶段整包拒绝。
void validateFieldConfig(const FieldConfig & c)
{
  auto range = [](double value, double lo, double hi, const char * name) {
      if (!std::isfinite(value) || value < lo || value > hi) {
        throw std::invalid_argument(std::string("Invalid field config: ") + name);
      }
    };
  // 扇区端点在 [-180,180]，宽度在 (0,180)，HueSector 的叉积判据只对半平面内扇区成立。
  auto sector = [&range](double lo, double hi, const char * name) {
      range(lo, -180, 180, name);
      range(hi, -180, 180, name);
      if (!(hi > lo && hi - lo < 180)) {
        throw std::invalid_argument(std::string("Invalid field config: ") + name);
      }
    };
  range(c.proc_width, 16, 4096, "proc_width");
  range(c.proc_height, 0, 4096, "proc_height");
  if (c.proc_height != 0) {range(c.proc_height, 16, 4096, "proc_height");}
  range(c.field_w_m, 0.001, 1000, "field_w_m");
  range(c.field_h_m, 0.001, 1000, "field_h_m");
  range(c.bev_px_per_m, 0.001, 100000, "bev_px_per_m");
  range(c.bev_width, 0, 4096, "bev_width");
  range(c.bev_height, 0, 4096, "bev_height");
  // 显式尺寸逐轴覆盖；任一轴为 0 时使用米制尺寸乘像素密度，并检查最终有效尺寸。
  range(c.bev_width > 0 ? c.bev_width : c.field_w_m * c.bev_px_per_m, 16, 4096,
    "effective bev width");
  range(c.bev_height > 0 ? c.bev_height : c.field_h_m * c.bev_px_per_m, 16, 4096,
    "effective bev height");
  range(c.morph_close_ksize, 1, 63, "morph_close_ksize");
  range(c.morph_dilate_ksize, 1, 63, "morph_dilate_ksize");
  range(c.morph_close_iter, 0, 20, "morph_close_iter");
  range(c.morph_dilate_iter, 0, 20, "morph_dilate_iter");
  range(c.edge_probe_radius, 0, 63, "edge_probe_radius");
  range(c.edge_bins, 2, 4096, "edge_bins");
  range(c.edge_samples, 2, 4096, "edge_samples");
  range(c.max_candidates_try, 1, 100, "max_candidates_try");
  range(c.fit_iterations, 1, 100, "fit_iterations");
  range(c.min_side_points, 2, 100000, "min_side_points");
  range(c.min_side_inliers, 2, 100000, "min_side_inliers");
  range(c.red_grid_cols, 1, 64, "red_grid_cols");
  range(c.red_grid_rows, 1, 64, "red_grid_rows");
  range(c.red_cells_min, 0, c.red_grid_cols * c.red_grid_rows, "red_cells_min");
  range(c.locate_hits_needed, 1, 100000, "locate_hits_needed");
  range(c.lost_frames_to_search, 1, 100000, "lost_frames_to_search");
  range(c.refit_every, 1, 100000, "refit_every");
  range(c.switch_confirm_frames, 2, 100000, "switch_confirm_frames");
  range(c.max_hold_frames, 0, 100000, "max_hold_frames");
  range(c.track_window_scale, 1, 10, "track_window_scale");
  range(c.fit_outlier_px, 0.001, 4096, "fit_outlier_px");
  range(c.min_rect_ratio_lo, 0.001, 1000, "min_rect_ratio_lo");
  range(c.min_rect_ratio_hi, c.min_rect_ratio_lo, 1000, "min_rect_ratio_hi");
  range(c.quad_side_ratio_lo, 0.001, 1000, "quad_side_ratio_lo");
  range(c.quad_side_ratio_hi, c.quad_side_ratio_lo, 1000, "quad_side_ratio_hi");
  range(c.quad_max_aspect_err, 0.001, 1000, "quad_max_aspect_err");
  range(c.black_conf_depth, 0.001, 1, "black_conf_depth");
  range(c.black_search_lo, 0, 1, "black_search_lo");
  range(c.black_search_hi, c.black_search_lo + 0.001, 1, "black_search_hi");
  range(c.black_row_lo, 0, 1, "black_row_lo");
  range(c.black_row_hi, c.black_row_lo + 0.001, 1, "black_row_hi");
  range(c.bev_border_margin_frac, 0, 0.49, "bev_border_margin_frac");
  range(c.corner_exclude_frac, 0, 0.49, "corner_exclude_frac");
// 此组参数是无量纲比例；颜色明度、饱和度与色相采用各自的单位，不能套用本范围。
#define FRACTION(name) range(c.name, 0, 1, #name)
  FRACTION(min_area_frac); FRACTION(min_fill_ratio); FRACTION(edge_band_frac);
  FRACTION(prior_corner_tol_frac); FRACTION(quad_min_area_frac);
  FRACTION(black_min_depth); FRACTION(black_score_frac); FRACTION(black_max_width_frac);
  FRACTION(black_max_offset_frac); FRACTION(corner_ema_alpha);
  FRACTION(edge_support_min); FRACTION(edge_support_strong); FRACTION(min_total_score);
  FRACTION(min_purple_score); FRACTION(purple_lost_threshold); FRACTION(red_inside_min);
  FRACTION(red_outside_max); FRACTION(expand_frac); FRACTION(red_cell_min);
  FRACTION(max_jump_frac); FRACTION(jump_accept_frac); FRACTION(fast_switch_purple);
  FRACTION(fast_switch_red_inside); FRACTION(fast_switch_red_outside);
#undef FRACTION
  sector(c.purple_hue_lo_deg, c.purple_hue_hi_deg, "purple hue sector");
  range(c.purple_chroma_ratio_min, 0, field_detail::kMaxChromaRatio, "purple_chroma_ratio_min");
  range(c.purple_L_min, 0, 100, "purple_L_min");
  range(c.purple_L_max, c.purple_L_min, 100, "purple_L_max");
  sector(c.red_hue_lo_deg, c.red_hue_hi_deg, "red hue sector");
  range(c.red_chroma_ratio_min, 0, field_detail::kMaxChromaRatio, "red_chroma_ratio_min");
  range(c.red_L_min, 0, 100, "red_L_min");
  sector(c.blue_hue_lo_deg, c.blue_hue_hi_deg, "blue hue sector");
  range(c.blue_chroma_ratio_min, 0, field_detail::kMaxChromaRatio, "blue_chroma_ratio_min");
  range(c.blue_L_min, 0, 100, "blue_L_min");
  range(c.interior_open_ksize, 1, 15, "interior_open_ksize");
  if (c.interior_open_ksize % 2 == 0) {
    throw std::invalid_argument("Invalid field config: interior_open_ksize must be odd");
  }
  // 所选内部扇区与紫色扇区不能重叠，否则同一像素同时是边框和内部证据。
  const double interior_lo = c.use_blue_interior ? c.blue_hue_lo_deg : c.red_hue_lo_deg;
  const double interior_hi = c.use_blue_interior ? c.blue_hue_hi_deg : c.red_hue_hi_deg;
  if (interior_lo < c.purple_hue_hi_deg && c.purple_hue_lo_deg < interior_hi) {
    throw std::invalid_argument("Invalid field config: interior hue sector overlaps purple");
  }
  range(c.black_max_lift, 0, 4, "black_max_lift");
  range(c.max_border_touch, 0, 4, "max_border_touch");
  range(c.pred_velocity_gain, 0, 10, "pred_velocity_gain");
  for (double weight : {c.w_purple, c.w_temporal, c.w_red_inside, c.w_red_outside, c.w_geometry}) {
    range(weight, 0, 100, "score weight");
  }
}
