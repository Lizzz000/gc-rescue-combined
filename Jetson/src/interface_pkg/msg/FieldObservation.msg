# Copyright 2026 li
# SPDX-License-Identifier: Apache-2.0
# detect_node 对单帧原图的 field（场地定位）识别结果，是 field 链路唯一的几何数据源。
# detect_node → /detect_node/field/results（每帧至多一条，含无效观测）；订阅：stream_node（叠加）、safety_node（校验后产出 SafetyTarget）。
# 增删几何字段 → 先改本文件，再同步 safety_node 的 z==0/有限性校验与 annotate 的绘制、重建 interface_pkg

# 每张源图像最多一条观测；未产生可发布正式几何时也发布无效观测以清空旧结果。
# corners/左右区域/split_line/candidate_corners 为原图像素，z=0；BEV 尺寸和 split_x/measured 为鸟瞰像素。
# 置信度、占比、评分及 jump_ratio 为无量纲诊断，不能把所有 split_* 字段都当作像素。
uint8 EDGE_MEASURED=0
uint8 EDGE_PREDICTED=1
uint8 EDGE_LOST=2
std_msgs/Header header
# header 沿用源图像时间；下方几何属于该帧，不是状态发布时刻的新观测。
# 无效观测不能用于几何计算，且 committed／switched 必须为 false。
bool valid
# state 表示 SEARCH／LOCATE／TRACK；quality 另描述保持、越界或回退。
string state
string quality
string reason
# 消费安全区几何须同时检查 valid 与 committed；候选和 HOLD 诊断不能代替正式结果。
bool committed
bool switched
string hold_reason
# 正式区域按左上、右上、右下、左下排列；无效时下列正式几何数组为空，候选诊断可单独保留。
geometry_msgs/Point32[<=4] corners
geometry_msgs/Point32[<=4] left_region
geometry_msgs/Point32[<=4] right_region
# 分界线的上、下端点投影回原图像素坐标。
geometry_msgs/Point32[<=2] split_line
# candidate_corners 仅供调试显示，不参与左右区中心或 CAN 目标计算。
geometry_msgs/Point32[<=4] candidate_corners
# 边缘顺序为上/右/下/左，取值同 EDGE_*。
uint8[4] edge_states [2, 2, 2, 2]
# 预测边包含候选先验补全，不仅是上一帧运动外推；此字段统计实测边数量。
uint8 measured_edges
# 鸟瞰宽高和下方分界位置使用 BEV 像素，不能与原图像素混算。
uint32 bev_width
uint32 bev_height
# split_x 是采用的鸟瞰分界位置；split_measured 是测量值，二者可能不同。
float64 split_x
float64 split_measured
bool split_suspect
# split_confidence 与 red_ratio 取值 [0,1]；score 是加权启发式得分，不是概率。
float64 split_confidence
float64 red_ratio
float64 score
float64 jump_ratio
