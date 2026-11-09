# Copyright 2026 li
# SPDX-License-Identifier: Apache-2.0
# 任务状态：目的地、异步操作、运输轮次与 STM32 握手阶段。不是运动速度命令。
# mission_node 发布到 /mission_node/status。safety_node 用目的地选左右区，bridge_node 用它做中心门控。
# 目的地和结果的取值要与 safety_node、bridge_node 用的是同一套，不能另编。

# header.stamp 是本机状态发布时间；同一个时刻可先后出现 PENDING/SUCCESS，消费者不能按帧时间去重。
std_msgs/Header header
# 目的地枚举：0 未定 / 1 左区 / 2 右区；由 mission 提交，切换失败时也可能撤销。
uint8 DEST_NONE=0
uint8 DEST_LEFT=1
uint8 DEST_RIGHT=2
# 操作阶段枚举：0 空闲 / 1 ROI 检查中 / 2 切换中。
uint8 OP_IDLE=0
uint8 OP_CHECKING=1
uint8 OP_SWITCHING=2
# 结果枚举：0 无 / 1 进行中 / 2 成功 / 3 被拒绝 / 4 超时 / 5 ROI 未通过 / 6 ROI 图像无效。
uint8 RESULT_NONE=0
uint8 RESULT_PENDING=1
uint8 RESULT_SUCCESS=2
uint8 RESULT_REJECTED=3
uint8 RESULT_TIMEOUT=4
uint8 RESULT_ROI_REJECTED=5
uint8 RESULT_ROI_INVALID=6
# 运输阶段：0 首绿尚未确认送达，只准恰好一个绿色；1 首绿已送达，绿黑合计≤3，橙色单独。
# 只有匹配本趟的 DELIVERY_COMPLETED 推进到 1；ROI 准入、到达和重找均不推进。
uint8 TRANSPORT_FIRST=0
uint8 TRANSPORT_LATER=1
# 目的地取值必须与 SafetyTarget.side 同一套。safety_node 的合法性校验和 bridge_node 的中心门控认的就是这组数。
# 业务握手与识别模式分开：TaskState 仍只报告实际 seg/field。
# 只有 SEARCHING_TARGET / NAVIGATING_SAFETY 允许对应坐标流。
uint8 CYCLE_SEARCHING_TARGET=0
uint8 CYCLE_CHECKING_ROI=1
# 闭合及后退后的 ROI 已提交；等待 STM32 授权后的去程转向完成，仍 seg 且停坐标。
uint8 CYCLE_WAITING_TURN=2
uint8 CYCLE_SWITCHING_FIELD=3
uint8 CYCLE_NAVIGATING_SAFETY=4
# 等待 STM32 盲推、释放与回撤；记录送达后仍等本地返程转向及显式重找。
# 本状态本身不代表已送达，运输进度仍看 transport_phase。
uint8 CYCLE_WAITING_TRANSFER=5
uint8 CYCLE_SWITCHING_SEG=6
uint8 CYCLE_ROI_FAILED=7
uint8 CYCLE_FAULT=8
uint8 cycle_phase
# 每次受理模式切换命令递增；即使瞬时中间状态被 QoS 合并，消费者仍识别新的坐标授权。
# 这是进程内代次，不是 CAN 事务号；不能防止跨进程重启或跨轮总线重放。
uint64 cycle_generation
uint8 destination
uint8 operation
uint8 outcome
uint8 transport_phase
# 与 RoiCheck.request_id 对应；mission 侧传输失败时为 0。
uint64 roi_request_id
string reason
