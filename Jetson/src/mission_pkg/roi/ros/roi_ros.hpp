#pragma once
// ---- 节点内部 ROI 消息搬运；业务规则只在核心入口判定 ----

#include <vision_msgs/msg/detection2_d_array.hpp>
#include <interface_pkg/msg/roi_check.hpp>
#include "roi/roi.hpp"

struct RoiRosConversion
{
  /**
   * @brief 搬运 ROI 证据到纯值规则，不重新判定消息结论。
   * @param[in] result 当前服务回执；header 与 request_id 由节点入口处理。
   * @return 独立的规则输入副本，不改变消息或任务状态。
   */
  static RoiRuleResult rule(const interface_pkg::msg::RoiCheck & result)
  {
    RoiRuleResult rule;
    rule.valid = result.valid;
    rule.passed = result.passed;
    rule.reason = result.reason;
    rule.roi = {result.roi_x, result.roi_y, result.roi_width, result.roi_height};
    rule.green_count = result.green_count;
    rule.blue_count = result.blue_count;
    rule.black_count = result.black_count;
    rule.orange_count = result.orange_count;
    rule.other_count = result.other_count;
    return rule;
  }

  /**
   * @brief 将纯值 ROI 结果组装为消息，沿用调用者选择的源身份。
   * @param[in] result 已完成规则判断的结果。
   * @param[in] header 源帧或本机失败诊断的消息头；不代表请求已执行。
   * @return 新消息；request_id 留给节点填写，不改变输入。
   */
  static interface_pkg::msg::RoiCheck message(
    const RoiRuleResult & result, const std_msgs::msg::Header & header)
  {
    interface_pkg::msg::RoiCheck output;
    output.header = header;
    output.valid = result.valid;
    output.passed = result.passed;
    output.reason = result.reason;
    output.roi_x = result.roi.x;
    output.roi_y = result.roi.y;
    output.roi_width = result.roi.width;
    output.roi_height = result.roi.height;
    output.green_count = result.green_count;
    output.blue_count = result.blue_count;
    output.black_count = result.black_count;
    output.orange_count = result.orange_count;
    output.other_count = result.other_count;
    return output;
  }

  /**
   * @brief 将完整检测数组送入 ROI 入口一次判断，保留非法项与源消息头。
   * @param[in] validator 已验证启动配置的 ROI 核心。
   * @param[in] input 本帧原始检测；不按轨迹编号去重，也不使用单个选中目标代替。
   * @return 本帧判定消息；无效输入产生无效结果，不沿用旧结论。
   */
  static interface_pkg::msg::RoiCheck evaluate(
    const RoiValidator & validator, const vision_msgs::msg::Detection2DArray & input)
  {
    return message(validator.evaluate(RecognitionRosConversion::detections(input)), input.header);
  }
};
