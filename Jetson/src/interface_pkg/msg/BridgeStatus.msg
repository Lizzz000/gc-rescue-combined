# Copyright 2026 li
# SPDX-License-Identifier: Apache-2.0
# bridge_node 的链路诊断快照（端点、本机句柄与链路就绪状态、收发与错误计数）。
# bridge_node → /bridge_node/status（QoS(1)）；无运行期订阅者，仅测试订阅。

std_msgs/Header header
# CDC 的 link.device。
string can_interface
# socket_open 表示本地传输句柄已打开；interface_up 保留兼容字段，与 socket_open 一致。
# 均不能据此认定下位机应用已就绪。
bool socket_open
bool interface_up
# 计数为进程内累计值；tx 只表示本机提交了完整一帧，不保证下位机收到或执行。
uint64 tx_frames
# rx 统计 CDC 收到的完整帧，业务校验拒绝另计 dropped_frames。
uint64 rx_frames
# 兼容旧消息的保留字段，CDC 固定为 0；传输故障写在 last_error。
uint64 bus_errors
# 混合统计队列撤销/过期、畸形输入和部分请求拒绝等，不是单一网络丢包率。
uint64 dropped_frames
string last_error
