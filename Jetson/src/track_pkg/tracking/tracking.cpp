// 分割跟踪：按类别把检测送进 OC-SORT。选择算法在 selector.cpp。

#include "tracking/tracking.hpp"

// ---- 分割跟踪实现 ----
// 职责：检测与轨迹关联。
// Copyright 2026 li
// SPDX-License-Identifier: Apache-2.0
// 把纯值检测按模型类别送入 OC-SORT，再恢复框、轨迹 ID 和类别；不处理 ROS 时间或分割掩膜。
// 阶段和断流重置时机由 TrackNode 控制；这里的轨迹老化按 update() 次数推进，不按实际经过的秒数。
// 适配行为优先在本文件维护；OC-SORT 的现有实现、已记录的本地修补与许可证位于 third_party，不随适配层整理改动。

#include <interface_pkg/app/core.hpp>
#include <cmath>
#include <stdexcept>

namespace
{

/**
 * @brief 在窄化前校验跟踪帧数及其相互约束。
 * @param[in] max_age 未命中帧数上限，范围[1,10000]。
 * @param[in] min_hits 连续命中帧数，范围[1,max_age]。
 * @param[in] delta_t 速度回看帧数，范围[1,max_age]。
 * @throws std::invalid_argument 参数越界时抛出，不创建或改变跟踪状态。
 */
void validateFrameCounts(std::int64_t max_age, std::int64_t min_hits, std::int64_t delta_t)
{
  if (max_age < 1 || max_age > 10000 || min_hits < 1 || min_hits > max_age ||
    delta_t < 1 || delta_t > max_age)
  {
    throw std::invalid_argument("invalid OC-SORT parameters");
  }
}

}  // namespace

// ---- SegTracker：按类别跟踪 ----
/**
 * @brief 唯一参数入口：校验实际float阈值及原int64帧数，再转换成核心参数。
 * @param[in] threshold 置信度，无量纲，转float后须在[0,1)。
 * @param[in] max_age 未命中帧数上限，[1,10000]。
 * @param[in] min_hits 连续命中帧数，[1,max_age]。
 * @param[in] iou IoU阈值，无量纲，转float后须在(0,1]。
 * @param[in] delta_t 回看帧数，[1,max_age]。
 * @param[in] inertia 方向惯性，无量纲，转float后须在[0,1]。
 * @param[in] use_byte 是否启用BYTE弱检测关联。
 * @return 已校验的参数；非法值抛invalid_argument，不创建或改变跟踪状态。
 */
TrackerParameters TrackerParameters::validated(
  double threshold, std::int64_t max_age, std::int64_t min_hits, double iou,
  std::int64_t delta_t,
  double inertia, bool use_byte)
{
  TrackerParameters parameters;
  parameters.detection_threshold = static_cast<float>(threshold);
  parameters.iou_threshold = static_cast<float>(iou);
  parameters.inertia = static_cast<float>(inertia);
  parameters.use_byte = use_byte;
  validateFrameCounts(max_age, min_hits, delta_t);
  // 判据只看窄化后的 float：double 下的余量在这里已经不存在了。
  if (!std::isfinite(parameters.detection_threshold) ||
    parameters.detection_threshold < 0.0f || parameters.detection_threshold >= 1.0f ||
    !std::isfinite(parameters.iou_threshold) ||
    parameters.iou_threshold <= 0.0f || parameters.iou_threshold > 1.0f ||
    !std::isfinite(parameters.inertia) ||
    parameters.inertia < 0.0f || parameters.inertia > 1.0f)
  {
    throw std::invalid_argument("invalid OC-SORT parameters");
  }
  parameters.max_age = static_cast<int>(max_age);
  parameters.min_hits = static_cast<int>(min_hits);
  parameters.delta_t = static_cast<int>(delta_t);
  return parameters;
}

SegTracker::SegTracker(const TrackerParameters & parameters)
: detection_threshold_(parameters.detection_threshold),
  iou_threshold_(parameters.iou_threshold), inertia_(parameters.inertia),
  max_age_(parameters.max_age), min_hits_(parameters.min_hits),
  delta_t_(parameters.delta_t), use_byte_(parameters.use_byte) {}

void SegTracker::reset() {trackers_.clear();}

/**
 * @brief 在检测输入入口校验一次几何与假设，避免规范化类别时重复扫描。
 * @param[in] detection 输入检测，框采用像素；幅度与尺寸和上限为1e7。
 * @return 合法时返回规范数字类别；拒绝时返回空串，输入和跟踪状态不变。
 */
std::string SegTracker::validatedClass(const RecognitionDetection & detection)
{
  if (!detection.hypothesis) {return {};}
  const auto & box = detection.box;
  if (!RecognitionValueValidation::axisAlignedBox(box) ||
    std::abs(box.center_x) + box.width > 1e7 ||
    std::abs(box.center_y) + box.height > 1e7) {return {};}
  return RecognitionValueValidation::classId(*detection.hypothesis);
}

RecognitionDetection SegTracker::toDetection(
  const Eigen::RowVectorXf & track, const std::string & class_id)
{
  // OC-SORT 输出以像素角点表示框；第 5 列是轨迹 ID，第 7 列是置信度。
  // 输出 id 属于跟踪器，不能沿用输入检测编号；模型类别由当前类别实例的键恢复，标签名称不参与身份。
  // 上游以 float 携带 ID，超过 2^24 后相邻整数可能重号；这里转回整数不能恢复已丢失的精度。
  RecognitionDetection detection;
  detection.id = std::to_string(static_cast<long long>(track(4)));
  detection.box.center_x = (track(0) + track(2)) / 2.0;
  detection.box.center_y = (track(1) + track(3)) / 2.0;
  detection.box.width = track(2) - track(0);
  detection.box.height = track(3) - track(1);
  detection.hypothesis = RecognitionHypothesis{class_id, track(6)};
  return detection;
}

/**
 * @brief 接收单个检测并建立其类别输入，非法项不改变类别状态。
 * @param[in] detection 原始像素检测；非法项或转float后尺寸坍缩的框忽略。
 * @details 仅接受后才创建类别会话并追加本帧行，不推进帧龄或改写原检测。
 */
void SegTracker::acceptDetection(const RecognitionDetection & detection)
{
  const auto class_id = validatedClass(detection);
  if (class_id.empty()) {return;}
  const auto & hypothesis = *detection.hypothesis;
  const auto & box = detection.box;
  const double x = box.center_x, y = box.center_y;
  const double w = box.width, h = box.height, score = hypothesis.score;
  Eigen::Matrix<float, 1, 6> row;
  // 输入行依次为左上、右下、置信度和类别占位；真正的类别隔离由 trackers_ 的键完成。
  row << x - w / 2, y - h / 2, x + w / 2, y + h / 2, score, 0;
  // double 下正尺寸可能在 float 中坍缩为零；提前拒绝，避免 Kalman 状态除零。
  if (row(2) <= row(0) || row(3) <= row(1)) {return;}
  auto & state = trackers_[class_id];
  // 类别串规范化后建实例，因此 "02" 与 "2" 共享同一类别的跟踪状态。
  if (!state.tracker) {
    state.tracker = std::make_unique<OCSort>(
      detection_threshold_, max_age_, min_hits_, iou_threshold_, delta_t_, "iou",
      inertia_, use_byte_);
  }
  state.detections.push_back(row);
}

/**
 * @brief 每次输入只校验和规范化一次检测，再按类别推进OC-SORT帧龄。
 * @param[in] input 只读像素检测数组，允许空帧；非法单项不进入关联。
 * @return 核心生成的轨迹；更新类别历史，不表示任何ROS消息已发布。
 */
RecognitionDetections SegTracker::update(const RecognitionDetections & input)
{
  // 复用每个活跃类别的输入缓冲；没有本帧检测的类别仍须用空矩阵推进老化。
  for (auto & entry : trackers_) {
    entry.second.detections.clear();
  }
  for (const auto & detection : input) {
    acceptDetection(detection);
  }
  // 按规范类别键遍历输出；这里的新轨迹只携带核心生成字段，原消息位置不参与跟踪身份。
  RecognitionDetections output;
  output.reserve(input.size());
  for (auto it = trackers_.begin(); it != trackers_.end(); ) {
    auto & state = it->second;
    const std::size_t count = state.detections.size();
    state.input.resize(count, 6);
    for (std::size_t i = 0; i < count; ++i) {
      state.input.row(i) = state.detections[i];
    }
    // 空帧也调用 update，推进预测和 time_since_update；超过 max_age 才删除未命中轨迹。
    for (const auto & row : state.tracker->update(state.input)) {
      if (row.size() != 7 || !row.allFinite()) {continue;}
      output.push_back(toDetection(row, it->first));
    }
    // 最后一条轨迹过期后释放该类别会话；下次出现会重新执行 OC-SORT 的启动预热规则。
    if (state.tracker->trackers.empty()) {it = trackers_.erase(it);} else {++it;}
  }
  return output;
}
