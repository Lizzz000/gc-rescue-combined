#pragma once
// 跟踪运行时合在一个头里，按节区分选择、跟踪和节点。OC-SORT 仍用第三方库。

// ---- 目标选择 ----
// 职责：按归一化画面代价选择原输入索引。
// Copyright 2026 li
// SPDX-License-Identifier: Apache-2.0
// 纯值目标选择核心，返回原输入索引，供 SelectNode 的 ROS 适配层原样转发。


#include <cstdint>
#include <optional>
#include <string>
#include <vector>
#include <interface_pkg/app/core.hpp>

// 生产 SelectNode 和未显式传策略的纯值调用共用以下默认值。
// 底边参考点、代价、切换余量和拥挤判据只在这里维护，单位为归一化画面比例。
struct SelectionReference
{
  // 坐标按图像宽、高分别归一化；默认点是画面底边中央，不是像素坐标。
  double x{0.5};
  double y{1.0};
};

struct SelectionPolicy
{
  // 下列代价和切换余量共用无量纲尺度；余量不是像素距离或框间隙阈值。
  SelectionReference reference{};
  // 代价 = (参考点 y - 底边 y) + lateral_weight * |底边 x - 参考点 x|。
  double lateral_weight{2.0};
  // 已选目标还在时，新目标必须好过这个余量才更换。
  double switch_margin{0.05};
  // 首趟：归一化底边中点的欧氏距离小于该值时视为拥挤，并加一次 isolation_penalty。
  double isolation_radius{0.15};
  double isolation_penalty{1.0};
};

// 维护上一帧目标 ID 以抑制轻微评分波动；输入只读、无线程，调用方决定阶段和断流重置。
class TargetSelector
{
public:
  /**
   * @brief 在构造入口一次校验画幅、类别与策略；实例只接收已建立的配置。
   * @param[in] width 图像宽度，像素，范围[1,INT_MAX]。
   * @param[in] height 图像高度，像素，范围[1,INT_MAX]。
   * @param[in] blue_min_distance_ratio 避蓝框间隙，无量纲，范围[0,1]。
   * @param[in] candidate_class_ids 非空非负类别列表，不得包含蓝色类别。
   * @param[in] blue_class_id 非负蓝色类别编号。
   * @param[in] policy 有限的归一化参考与非负策略系数。
   * @throws std::invalid_argument 非法配置时构造失败，不创建选择历史。
   */
  TargetSelector(
    std::int64_t width, std::int64_t height, double blue_min_distance_ratio,
    const std::vector<std::int64_t> & candidate_class_ids, std::int64_t blue_class_id,
    SelectionPolicy policy = {});
  /**
   * @brief 在输入入口筛选一次检测，再执行避蓝、首趟资格与目标择优。
   * @param[in] input 原始像素检测，保持原索引；非法单项被忽略。
   * @return 原输入索引或空值；更新上一目标ID。
   */
  std::optional<std::size_t> select(const RecognitionDetections & input);
  // 首次运输只保留 first_class；置 true 后恢复全部候选。默认 true，节点启动时改为 false。
  void setLaterTransports(bool later);
  // 首趟类别由节点启动校验后传入，须为候选中的规范数字类别串。
  void setFirstClass(const std::string & class_id);
  // 清空上一帧选中 ID（并列取舍用），不改变参数。
  // 上一帧 ID 还用于切换余量判断；重置后按当前安全候选重新择优。
  void reset();

private:
  /**
   * @brief 校验单个输入并判断是否属于候选或蓝色，供公共入口复用。
   * @param[in] detection 原始检测，像素框、数字类别和[0,1]置信度。
   * @return 可参与时返回规范类别；否则为空，输入和选择历史不变。
   */
  std::string acceptedClass(const RecognitionDetection & detection) const;
  // 本轮已接受的货物与蓝色框；定义随实现保留，不公开内部几何表示。
  struct SelectionInput;
  /**
   * @brief 接收单个检测，将可计算的框归入候选集合。
   * @param[in] detection 原始像素检测，非法项忽略，输入不变。
   * @param[in,out] accepted 本轮已接受的货物和蓝色框，按输入顺序追加。
   */
  void acceptDetection(
    const RecognitionDetection & detection, SelectionInput & accepted) const;
  double image_width_, image_height_, blue_distance_;
  std::vector<std::string> candidate_classes_;
  std::string blue_class_, first_class_{"0"}, previous_id_;
  SelectionPolicy policy_{};
  bool later_transports_{true};
};

// ---- 分割跟踪 ----
// 职责：按类别维护 OC-SORT，不创建线程。
// Copyright 2026 li
// SPDX-License-Identifier: Apache-2.0
// 声明按类别维护 OC-SORT 的纯值核心与窄化后校验的跟踪参数。


#include <map>
#include <memory>
#include <string>
#include <vector>
#include <interface_pkg/app/core.hpp>
#include "OCSort.hpp"

// OC-SORT 参数在窄化到 float 之后校验：double 合法但 float 非法的值必须在启动期失败。
// 例如 iou_threshold=1e-300 下溢为 0，会让无交集框命中同一轨迹；det_threshold=0.9999999999 舍入为 1，会丢弃全部检测。
struct TrackerParameters
{
  // 阈值与惯性均无量纲；max_age/delta_t以跟踪帧为单位，min_hits约束连续匹配次数。
  float detection_threshold;
  float iou_threshold;
  float inertia;
  int max_age;
  int min_hits;
  int delta_t;
  bool use_byte;

  /**
   * @brief 唯一参数入口：浮点按实际float精度校验，整数在窄化前校验。
   * @param[in] threshold 检测置信度，无量纲，转float后须在[0,1)。
   * @param[in] max_age 未命中帧数上限，范围[1,10000]。
   * @param[in] min_hits 连续命中帧数，范围[1,max_age]。
   * @param[in] iou IoU阈值，无量纲，转float后须在(0,1]。
   * @param[in] delta_t 速度回看帧数，范围[1,max_age]。
   * @param[in] inertia 方向惯性，无量纲，转float后须在[0,1]。
   * @param[in] use_byte 是否启用BYTE弱检测关联。
   * @return 已校验的参数值；非法输入抛invalid_argument，不创建跟踪状态。
   */
  static TrackerParameters validated(
    double threshold, std::int64_t max_age, std::int64_t min_hits, double iou,
    std::int64_t delta_t,
    double inertia, bool use_byte);

private:
  TrackerParameters() = default;
};

// 每个模型类别独立维护 OC-SORT，只跟踪框和轨迹身份，不跟踪分割掩膜。
// 只接受已校验的参数，构造出来即合法。
class SegTracker
{
public:
  explicit SegTracker(const TrackerParameters & parameters);
  // min_hits 预热按每类实例 update 次数计，空帧也计入；预热后新轨迹需要再关联 min_hits 次才输出。
  // max_age 按已接受帧推进。未命中的轨迹不输出；超过配置帧数才删除，恰好等于时仍保留。
  RecognitionDetections update(const RecognitionDetections & detections);
  // 清空全部类别跟踪器，不复位上游进程级 ID 计数器；新轨迹继续取号。
  void reset();

private:
  /**
   * @brief 在检测输入入口校验框、幅度和假设，并返回规范类别。
   * @param[in] detection 输入检测；像素幅度与尺寸和不得超过1e7。
   * @return 合法类别字符串；空串表示拒绝，输入不变。
   */
  static std::string validatedClass(const RecognitionDetection & detection);
  /**
   * @brief 接收单个检测，校验并转成float角点后追加到对应类别的本帧缓冲。
   * @param[in] detection 原始像素检测；非法项和转float后坍缩的框忽略。
   * @details 仅接受后才创建类别会话，不推进跟踪帧龄或改写原检测。
   */
  void acceptDetection(const RecognitionDetection & detection);
  static RecognitionDetection toDetection(
    const Eigen::RowVectorXf & track, const std::string & class_id);

  float detection_threshold_, iou_threshold_, inertia_;
  int max_age_, min_hits_, delta_t_;
  bool use_byte_;
  struct ClassTracker
  {
    // 每个模型类别独立保有跟踪状态和输入缓冲，避免跨类别关联同一个轨迹。
    std::unique_ptr<OCSort> tracker;
    std::vector<Eigen::Matrix<float, 1, 6>> detections;
    Eigen::MatrixXf input;
  };
  // 键是去前导零后的数字类别串，与轨迹 id 和显示用标签名称无关。
  std::map<std::string, ClassTracker> trackers_;
};
