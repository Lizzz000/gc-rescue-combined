#include "field/opencv/algorithm.hpp"

// 形态学只改变调度。核、迭代和边界沿用原来的 OpenCV 调用。
// 公开入口 morph() 也放在这里，几何拟合不再直接碰条带实现。

// ---- 场地形态学 ----
// 职责：条带调度，核与迭代沿用原操作。
// Copyright 2026 jetson_yolo integration
// SPDX-License-Identifier: Apache-2.0
// 形态学私有实现：条带只改变调度，核、迭代、默认边界和像素所有权均沿用原操作。

#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <cstdint>

namespace field_detail::morphology
{

inline cv::Mat serial(const cv::Mat & mask, const FieldConfig & cfg)
{
  // 范围外的开发调用仍交给 OpenCV；不能提前创建第二个核或自行替换异常。
  cv::Mat out;
  cv::Mat k1 = cv::getStructuringElement(cv::MORPH_ELLIPSE,
      {cfg.morph_close_ksize, cfg.morph_close_ksize});
  cv::morphologyEx(mask, out, cv::MORPH_CLOSE, k1, cv::Point(-1, -1), cfg.morph_close_iter);
  cv::Mat k2 = cv::getStructuringElement(cv::MORPH_ELLIPSE,
      {cfg.morph_dilate_ksize, cfg.morph_dilate_ksize});
  cv::dilate(out, out, k2, cv::Point(-1, -1), cfg.morph_dilate_iter);
  return out;
}

inline bool supportsStripes(const cv::Mat & mask, const FieldConfig & cfg)
{
  return !mask.empty() && mask.dims == 2 && mask.type() == CV_8UC1 &&
         cfg.morph_close_ksize >= 1 && cfg.morph_close_ksize <= 63 &&
         cfg.morph_dilate_ksize >= 1 && cfg.morph_dilate_ksize <= 63 &&
         cfg.morph_close_iter >= 0 && cfg.morph_close_iter <= 20 &&
         cfg.morph_dilate_iter >= 0 && cfg.morph_dilate_iter <= 20;
}

struct Halo
{
  int above;
  int below;
};

// 仅对 supportsStripes() 的合法范围求值。默认 anchor=floor(k/2)，偶数核上下不对称。
// CLOSE 的 n 次膨胀、n 次腐蚀，再加 m 次膨胀，按依赖方向逐项累加保守边界。
inline Halo dependencyHalo(const FieldConfig & cfg)
{
  const int close_anchor = cfg.morph_close_ksize / 2;
  const int dilate_anchor = cfg.morph_dilate_ksize / 2;
  return {
    2 * cfg.morph_close_iter * close_anchor + cfg.morph_dilate_iter * dilate_anchor,
    2 * cfg.morph_close_iter * (cfg.morph_close_ksize - 1 - close_anchor) +
      cfg.morph_dilate_iter * (cfg.morph_dilate_ksize - 1 - dilate_anchor),
  };
}

// 私有调度入口也供合同测试强制划分边界；不新增生产参数、不改变 OpenCV 全局线程数。
inline cv::Mat withStripes(
  const cv::Mat & mask, const FieldConfig & cfg, int requested_stripes)
{
  if (!supportsStripes(mask, cfg) || requested_stripes <= 1 || mask.rows < 2) {
    return serial(mask, cfg);
  }
  const int count = std::min(requested_stripes, mask.rows);
  const auto halo = dependencyHalo(cfg);
  const cv::Mat close_kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE,
      {cfg.morph_close_ksize, cfg.morph_close_ksize});
  cv::Mat output(mask.size(), mask.type());
  cv::parallel_for_(cv::Range(0, count), [&](const cv::Range & stripe_range) {
      for (int stripe = stripe_range.start; stripe < stripe_range.end; ++stripe) {
        const int first = static_cast<int>(std::int64_t(mask.rows) * stripe / count);
        const int last = static_cast<int>(std::int64_t(mask.rows) * (stripe + 1) / count);
        const int expanded_first = std::max(0, first - halo.above);
        const int expanded_last = static_cast<int>(
          std::min<std::int64_t>(mask.rows, std::int64_t(last) + halo.below));

        // rowRange 保留原父图信息：首轮膨胀仍可读取 ROI 外的原像素，不能 clone 或隔离边界。
        // 后续操作在独立 local 内进行；内部条带边界距最终保留区域至少为累计依赖半径。
        const cv::Mat source = mask.rowRange(expanded_first, expanded_last);
        cv::Mat local;
        cv::morphologyEx(source, local, cv::MORPH_CLOSE, close_kernel,
          cv::Point(-1, -1), cfg.morph_close_iter);
        // 与串行一样，完成 CLOSE 后再创建第二核。非法核和特殊 Mat 已由串行路径处理。
        const cv::Mat dilate_kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE,
            {cfg.morph_dilate_ksize, cfg.morph_dilate_ksize});
        cv::dilate(local, local, dilate_kernel, cv::Point(-1, -1), cfg.morph_dilate_iter);

        // 每个任务独占中心行；输出不与输入或条带临时量共享存储。
        cv::Mat destination = output.rowRange(first, last);
        local.rowRange(first - expanded_first, last - expanded_first).copyTo(destination);
      }
    }, count);
  return output;
}

inline cv::Mat apply(const cv::Mat & mask, const FieldConfig & cfg)
{
  if (!supportsStripes(mask, cfg)) {return serial(mask, cfg);}
  const auto halo = dependencyHalo(cfg);
  const int total_halo = halo.above + halo.below;
  if (total_halo == 0) {return serial(mask, cfg);}

  // 小 ROI 或大核/多迭代时减少重复边缘计算；复用已有 OpenCV 线程池，不引入新工作线程。
  const int minimum_rows = std::max(64, total_halo * 2);
  const int count = std::min(std::max(1, cv::getNumThreads()), mask.rows / minimum_rows);
  return withStripes(mask, cfg, count);
}

}  // namespace field_detail::morphology

namespace field_detail {
// 阶段: 形态学 —— 闭运算/膨胀用于连通轮廓；拟合点取原始紫色，边支持度另用轻度膨胀的 purple_probe。
cv::Mat morph(const cv::Mat & mask, const FieldConfig & cfg)
{
  return morphology::apply(mask, cfg);
}
}  // namespace field_detail
