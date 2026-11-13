# Copyright 2026 li
# SPDX-License-Identifier: Apache-2.0
# bridge_node 的链路诊断快照（端点、本机句柄与链路就绪状态、收发与错误计数）。
# bridge_node → /bridge_node/status（QoS(1)）；无运行期订阅者，仅测试订阅。

std_msgs/Header header
# CDC 的 link.device，或虚拟回环的网口名。
string can_interface
# socket_open 表示本地传输句柄已打开；interface_up 为 CDC 会话就绪或虚拟网口 UP。
# 均不能据此认定下位机应用已就绪。
bool socket_open
bool interface_up
# 计数为进程内累计值；tx 只表示本机提交了完整一帧，不保证下位机收到或执行。
uint64 tx_frames
# rx 在虚拟回环上统计错误帧，与 bus_errors 同步递增，不含普通下位机报文。
uint64 rx_frames
# 虚拟回环的错误帧数量。CDC 故障写在 last_error，不伪造错误帧。
uint64 bus_errors
# 混合统计队列撤销/过期、畸形输入和部分请求拒绝等，不是单一网络丢包率。
uint64 dropped_frames
string last_error
