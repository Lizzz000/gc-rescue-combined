# Copyright 2026 li
# SPDX-License-Identifier: Apache-2.0
# 一次 ROI 服务请求的检查结果（有效性、是否通过、ROI 位置、各类颜色计数）。
# mission_node 与 target_node 的 roi_result 话题用于诊断；同一结果通过 CheckRoi 服务返回给调用方。

# 单次请求只检查一张符合条件的源帧，通过结果仅代表该帧。
std_msgs/Header header
# 检查源帧时沿用其 header；无帧拒绝或超时结果不能当作一份源图像几何。
# target_node 进程内请求编号；mission 传输失败时为零。
# 请求号要和 mission_node 透传的值、MissionStatus.roi_request_id 是同一个数。
uint64 request_id
# 请求号用于关联这次检查，不是轨迹 ID，也不保证跨进程重启全局唯一。
# valid 表示已检查新鲜且格式正确的帧，passed 表示满足本服务的 ROI／运输规则。
# valid=true 不保证 passed=true；空 ROI 或违反颜色组合时仍可以是一帧有效检查。
bool valid
bool passed
string reason
# ROI 四个量均为相对图像宽高的比例，不是像素或物理距离。
float64 roi_x
float64 roi_y
float64 roi_width
float64 roi_height
# 每条相交记录分别计数，重复检测/轨迹 ID 不合并；非法帧的计数可能只覆盖扫描前缀。
uint32 green_count
uint32 blue_count
uint32 black_count
uint32 orange_count
uint32 other_count
# 计数按与 ROI 相交或触边的检测框累计，不按被覆盖面积折算。
