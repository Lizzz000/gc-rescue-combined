// 目标选择：代价、切换余量和拥挤判据。不改写输入检测。
// 跟踪在 tracking.cpp。本文件不订阅 ROS。

#include "tracking/tracking.hpp"

// ---- 目标选择实现 ----
// 职责：代价、切换余量和拥挤判据。
// Copyright 2026 li
// SPDX-License-Identifier: Apache-2.0
// 实现不依赖 ROS 的候选筛选、避蓝、归一化择优及上帧 ID 并列规则，不改写输入检测。

#include <interface_pkg/app/core.hpp>
#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace
{

/**
 * @brief 在构造入口校验选择策略的参考点及非负系数。
 * @param[in] policy 归一化参考点必须有限；代价、切换和拥挤系数必须有限且非负。
 * @throws std::invalid_argument 策略非法时抛出，不创建选择历史。
 */
void validateSelectionPolicy(const SelectionPolicy & policy)
{
  if (!std::isfinite(policy.reference.x) || !std::isfinite(policy.reference.y)) {
    throw std::invalid_argument("invalid selection geometry or classes");
  }
  for (double coefficient : {policy.lateral_weight, policy.switch_margin,
    policy.isolation_radius, policy.isolation_penalty})
  {
    if (!std::isfinite(coefficient) || coefficient < 0) {
      throw std::invalid_argument("invalid selection geometry or classes");
    }
  }
}

}  // namespace

// ---- TargetSelector：选择算法 ----
// 比较点是框底边中点；生产节点采用 SelectionPolicy 的默认值。
// 亮蓝按归一化框间隙挡路：两个方向的间隙都小于阈值才淘汰，等于阈值仍安全。
// 底边只决定选中哪一个框。类型 3 使用框几何中心，换算在桥接包的定点量化节。
// 已选 ID 还在安全候选里时，新目标必须好过 switch_margin 才更换。
// 没有旧目标时，代价相同则取数值较小的 ID（"2" 先于 "10"）。
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
TargetSelector::TargetSelector(
  std::int64_t width, std::int64_t height, double blue_min_distance_ratio,
  const std::vector<std::int64_t> & candidate_class_ids, std::int64_t blue_class_id,
  SelectionPolicy policy)
: image_width_(width), image_height_(height),
  blue_distance_(blue_min_distance_ratio), blue_class_(std::to_string(blue_class_id)),
  policy_(policy)
{
  if (width <= 0 || height <= 0 || width > std::numeric_limits<int>::max() ||
    height > std::numeric_limits<int>::max() || !std::isfinite(blue_min_distance_ratio) ||
    blue_min_distance_ratio < 0 || blue_min_distance_ratio > 1 ||
    candidate_class_ids.empty() || blue_class_id < 0)
  {
    throw std::invalid_argument("invalid selection dimensions, geometry or classes");
  }
  validateSelectionPolicy(policy);
  for (auto id : candidate_class_ids) {
    if (id < 0 || id == blue_class_id) {
      throw std::invalid_argument("candidate class must be nonnegative and non-blue");
    }
    candidate_classes_.push_back(std::to_string(id));
  }
}

/**
 * @brief 在输入入口校验一次几何和假设，返回可参与的规范类别。
 * @param[in] detection 只读原检测；ID非空、框合法、类别属于候选或蓝色。
 * @return 规范类别或空串；不改变输入与历史。
 */
std::string TargetSelector::acceptedClass(const RecognitionDetection & detection) const
{
  if (detection.id.empty() || !detection.hypothesis ||
    !RecognitionValueValidation::axisAlignedBox(detection.box)) {return {};}
  auto class_id = RecognitionValueValidation::classId(*detection.hypothesis);
  if (class_id == blue_class_ ||
    std::find(candidate_classes_.begin(), candidate_classes_.end(), class_id) !=
    candidate_classes_.end()) {return class_id;}
  return {};
}

void TargetSelector::reset() {previous_id_.clear();}

void TargetSelector::setLaterTransports(bool later) {later_transports_ = later;}

void TargetSelector::setFirstClass(const std::string & class_id) {first_class_ = class_id;}

namespace
{

// 指针始终指向只读原输入记录；相同轨迹 ID 的两条记录仍有独立几何与原始位置。
struct NormBox
{
  const RecognitionDetection * detection;
  double left, right, top, bottom, x, y;
  bool selectable;
};

/**
 * @brief 将已校验的像素框换成归一化边界，拒绝换算结果溢出的记录。
 * @param[in] detection 合法的原始检测；返回值中的指针仅借用该输入。
 * @param[in] image_width 已校验的图像宽度，像素，正数。
 * @param[in] image_height 已校验的图像高度，像素，正数。
 * @param[in] selectable 本轮是否具备货物选择资格，蓝色框不使用该标记。
 * @return 保留有限画外坐标的框；换算溢出时为空，不改变输入。
 */
std::optional<NormBox> normalizedBox(
  const RecognitionDetection & detection, double image_width, double image_height,
  bool selectable)
{
  const auto & box = detection.box;
  const double x = box.center_x / image_width;
  const double bottom = (box.center_y + box.height / 2.0) / image_height;
  const double top = (box.center_y - box.height / 2.0) / image_height;
  const double half_width = box.width / image_width / 2.0;
  const double left = x - half_width, right = x + half_width;
  // 输入分量有限不保证计算结果有限；画外有限边界仍保留，避免改变避蓝和评分。
  if (!std::isfinite(x) || !std::isfinite(bottom) || !std::isfinite(top) ||
    !std::isfinite(left) || !std::isfinite(right)) {return std::nullopt;}
  return NormBox{&detection, left, right, top, bottom, x, bottom, selectable};
}

double axisGap(double a0, double a1, double b0, double b1)
{
  // 两个闭区间重叠或触边时间隙为零；分别计算横、纵间隙，不计算中心点距离。
  if (a1 < b0) {return b0 - a1;}
  if (b1 < a0) {return a0 - b1;}
  return 0;
}

bool blockedByBlue(
  const NormBox & candidate, const std::vector<NormBox> & blues, double blue_distance)
{
  for (const auto & blue : blues) {
    const double dx = axisGap(candidate.left, candidate.right, blue.left, blue.right);
    const double dy = axisGap(candidate.top, candidate.bottom, blue.top, blue.bottom);
    // EPS 仅补偿间隙比较的浮点误差；切换余量的严格比较另行保留，不共用此容差。
    if (dx + 1e-12 < blue_distance && dy + 1e-12 < blue_distance) {return true;}
  }
  return false;
}

bool crowdedForFirstTrip(
  const NormBox & candidate, const std::vector<NormBox> & cargo,
  bool later_transports, const SelectionPolicy & policy)
{
  if (later_transports) {return false;}
  // 首趟未获选择资格的黑、橙货物也参与邻近判定；相同 ID 不代表同一条输入。
  for (const auto & other : cargo) {
    if (other.detection == candidate.detection) {continue;}
    const bool nearby = std::hypot(candidate.x - other.x, candidate.y - other.y) + 1e-12 <
      policy.isolation_radius;
    if (nearby) {return true;}
  }
  return false;
}

double selectionCost(const NormBox & candidate, bool crowded, const SelectionPolicy & policy)
{
  const double depth = policy.reference.y - candidate.y;
  // 纵深是带符号差值，横向偏离取绝对值；不计算欧氏距离或裁剪有限画外底边。
  const double lateral = std::abs(candidate.x - policy.reference.x);
  double cost = depth + policy.lateral_weight * lateral;
  if (crowded) {cost += policy.isolation_penalty;}
  return cost;
}

bool smallerTrackId(const std::string & left, const std::string & right)
{
  // 纯数字轨迹 ID 去掉前导零后按长度和数值文本比较，无须窄化成固定宽度整数。
  // 非数字或数值等价的文本仍保留确定性的后备比较。
  // numericClass 在此只复用纯数字串判据，轨迹 ID 不解释为模型类别。
  if (RecognitionValueValidation::numericClass(left) &&
    RecognitionValueValidation::numericClass(right))
  {
    const auto canonical = [](const std::string & id) {
        const auto first = id.find_first_not_of('0');
        return first == std::string::npos ? std::string("0") : id.substr(first);
      };
    const auto a = canonical(left), b = canonical(right);
    if (a.size() != b.size()) {return a.size() < b.size();}
    if (a != b) {return a < b;}
  }
  return left < right;
}

// 本轮评分状态：相同ID仍按独立记录处理，previous保留最后一个安全匹配。
class SelectionRanking
{
public:
  /**
   * @brief 建立本轮择优状态并借用上一目标身份。
   * @param[in] previous_id 上一目标ID，可为空；其引用须覆盖本轮评分过程。
   */
  explicit SelectionRanking(const std::string & previous_id) : previous_id_(previous_id) {}

  /**
   * @brief 接收一个安全候选，按代价和轨迹ID更新最优与旧目标记录。
   * @param[in] detection 原检测，其指针须保持到本轮结果取出。
   * @param[in] cost 已校验为有限的无量纲代价，越小越优。
   */
  void consider(const RecognitionDetection * detection, double cost)
  {
    const Scored scored{detection, cost};
    if (detection->id == previous_id_) {previous_ = scored;}
    const bool better = best_.detection && scored.cost + 1e-12 < best_.cost;
    const bool tie = best_.detection && std::abs(scored.cost - best_.cost) <= 1e-12 &&
      smallerTrackId(scored.detection->id, best_.detection->id);
    if (!best_.detection || better || tie) {best_ = scored;}
  }

  /**
   * @brief 应用切换余量后返回本轮目标，改善恰等于余量时保留旧目标。
   * @param[in] switch_margin 已校验为有限非负的无量纲切换余量。
   * @return 借用的原检测指针；没有安全候选时为空，不改写检测或历史。
   */
  const RecognitionDetection * result(double switch_margin) const
  {
    // previous只会在best已建立时存在；两者按记录指针区分，不合并重复ID。
    if (previous_.detection && best_.detection != previous_.detection &&
      !(best_.cost + switch_margin < previous_.cost))
    {
      return previous_.detection;
    }
    return best_.detection;
  }

private:
  struct Scored
  {
    // 借用原输入记录，cost为其有限无量纲代价；仅由consider更新。
    const RecognitionDetection * detection{nullptr};
    double cost{0.0};
  };
  const std::string & previous_id_;
  Scored best_, previous_;
};

}  // namespace

// 每次select调用独占该输入集合；框指针只在本次只读输入有效期内使用。
struct TargetSelector::SelectionInput
{
  std::vector<NormBox> cargo;
  std::vector<NormBox> blues;
};

/**
 * @brief 完成单框接收，登记可计算的候选或蓝色框。
 * @param[in] detection 原始像素检测，非法项忽略，输入不变。
 * @param[in,out] accepted 本轮已接受集合，按原输入顺序追加。
 */
void TargetSelector::acceptDetection(
  const RecognitionDetection & detection, SelectionInput & accepted) const
{
  const auto class_id = acceptedClass(detection);
  if (class_id.empty()) {return;}
  const auto entry = normalizedBox(
    detection, image_width_, image_height_, later_transports_ || class_id == first_class_);
  if (!entry) {return;}
  if (class_id == blue_class_) {accepted.blues.push_back(*entry);}
  else {accepted.cargo.push_back(*entry);}
}

/**
 * @brief 在本次输入入口完成单项校验和规范化，再按已有几何规则择优。
 * @param[in] input 原始像素检测，非法单项被忽略，输入不变。
 * @return 原输入位置或空值；更新上一目标ID。
 */
std::optional<std::size_t> TargetSelector::select(const RecognitionDetections & input)
{
  SelectionInput accepted;
  for (const auto & detection : input) {
    acceptDetection(detection, accepted);
  }
  SelectionRanking ranking(previous_id_);
  // 两个方向的框间隙都小于阈值才算被亮蓝挡住。首趟再给旁边有其他货物的候选加罚分。
  for (const auto & candidate : accepted.cargo) {
    if (!candidate.selectable) {continue;}
    if (blockedByBlue(candidate, accepted.blues, blue_distance_)) {continue;}
    const bool crowded = crowdedForFirstTrip(candidate, accepted.cargo, later_transports_, policy_);
    const double cost = selectionCost(candidate, crowded, policy_);
    if (!std::isfinite(cost)) {continue;}
    ranking.consider(candidate.detection, cost);
  }
  const auto * best = ranking.result(policy_.switch_margin);
  if (best) {
    previous_id_ = best->id;
    // 返回原消息的位置，适配层据此完整复制原检测。
    return best->input_index;
  }
  reset();
  return std::nullopt;
}
