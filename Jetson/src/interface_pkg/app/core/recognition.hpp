#pragma once
// 框、类别和画幅的纯值。不依赖 ROS。

// ---- 识别纯值 ----
// 职责：框、类别和画幅诊断，不依赖 ROS。
// Copyright 2026 li
// SPDX-License-Identifier: Apache-2.0

#include <cstddef>
#include <optional>
#include <string>
#include <vector>
#include <cmath>
#include <cstdint>

// 识别核心算法共用的纯值类型，不依赖 ROS；保留非法条目与原输入索引。

struct PixelBox
{
  // 中心和宽高使用原图像素，theta 使用弧度；值类型本身不裁剪或校验。
  double center_x{0}, center_y{0}, width{0}, height{0}, theta{0};
};

struct RecognitionHypothesis
{
  // class_id 是模型的数字类别编号，不是标签名称。
  std::string class_id;
  double score{0};  // 置信度无量纲，合法闭区间为 [0,1]，不由值类型自动修正。
};

// 一条输入对应一条纯值记录；算法筛选后仍可通过 input_index 找回完整消息。
struct RecognitionDetection
{
  // id 是检测／轨迹身份，与 hypothesis 中的类别编号是两个不同概念。
  std::string id;
  PixelBox box;
  // 缺失假设原样表示，交给具体算法决定拒绝单框还是整帧，转换层不提前丢弃它。
  std::optional<RecognitionHypothesis> hypothesis;
  // 指向转换前数组中的原始位置，用于选择输出保留全部 ROS 字段。
  std::size_t input_index{0};
};

using RecognitionDetections = std::vector<RecognitionDetection>;

// 像素框与归一化 ROI 使用不同类型，避免隐含坐标单位转换。
struct NormalizedRoi
{
  // x、y 为左上角，宽高为相对图像尺寸的比例；归一化基准由调用节点提供。
  double x{0}, y{0}, width{0}, height{0};
};

// 纯值框、置信度与数字类别校验；ROS 边界和算法使用同一判据。
struct RecognitionValueValidation
{
  static bool axisAlignedBox(const PixelBox & box)
  {
    // 只检查有限值和正尺寸；中心或边界落在画外不由这层拒绝。
    return std::isfinite(box.center_x) && std::isfinite(box.center_y) &&
           std::isfinite(box.width) && std::isfinite(box.height) &&
           box.width > 0 && box.height > 0;
  }

  static bool numericClass(const std::string & value)
  {
    // 只接受非空十进制数字串，不接受正负号、空白或模型标签名称。
    return !value.empty() && value.find_first_not_of("0123456789") == std::string::npos;
  }

  static std::string classId(const std::string & value)
  {
    // 保持字符串表示以避免整数溢出；全零串统一为 "0"，非法串返回空值。
    if (!numericClass(value)) {return {};}
    const auto first = value.find_first_not_of('0');
    return first == std::string::npos ? "0" : value.substr(first);
  }

  /**
   * @brief 在识别输入入口同时校验置信度并规范化数字类别，避免重复扫描类别。
   * @param[in] value 输入假设；类别为非空数字串，置信度无量纲且须在[0,1]。
   * @return 有效时返回去前导零的类别；无效时返回空串，输入不变。
   */
  static std::string classId(const RecognitionHypothesis & value)
  {
    if (!std::isfinite(value.score) || value.score < 0 || value.score > 1) {return {};}
    return classId(value.class_id);
  }

  static bool hypothesis(const std::string & class_id, double score)
  {
    // 直接借用调用方的类别串，ROS 接收端无需为校验临时复制每条检测的类别。
    return numericClass(class_id) && std::isfinite(score) && score >= 0 && score <= 1;
  }

  static bool hypothesis(const RecognitionHypothesis & value)
  {
    return hypothesis(value.class_id, value.score);
  }

  // Mission 提交与类型 6 共用目的地映射，编号与 MissionStatus 一致：
  // 恰好一件橙色返回 2（DEST_RIGHT），否则返回 1（DEST_LEFT）。
  static constexpr std::uint8_t cargoSide(std::uint32_t orange_count)
  {
    // 只映射左右区，不负责验证货物计数组合；调用方须先完成其业务准入检查。
    return orange_count == 1 ? 2 : 1;
  }
};
