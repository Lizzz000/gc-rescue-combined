# Copyright 2026 li
# SPDX-License-Identifier: Apache-2.0
# safety_node 输出的安全区目标点，只表达被选中一侧的区域中心。
# safety_node → /safety_node/target（keep_last(1)，每帧至多一条）；阶段、路由或超时也会主动发布撤销。
# 订阅：bridge_node（中心门控与 CAN 下发）。

std_msgs/Header header
# 有效目标的时间对应源场地观测，不能用收到本消息的时刻替代其新鲜度判定。
# false 表示本帧没有可用安全区，此时 x、y 无意义，bridge_node 不应使用。
bool valid
# valid=false 时仍可能携带最近目的区编号；不能只检查 side 就生成有效运动目标。
uint8 side
# side 使用 MissionStatus 的左／右目的地编号，不是模型颜色类别 ID。
# x、y 是所选区域四顶点平均值除以 image_width/image_height，不裁剪，有限画外值可以落在 [0,1] 之外。
# 仅 valid=true 时有效；无效时 x、y 为 0。
float64 x
float64 y
