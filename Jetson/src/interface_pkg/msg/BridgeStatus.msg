# Copyright 2026 li
# SPDX-License-Identifier: Apache-2.0
# bridge_node 的 CAN 链路诊断快照（端点、本机句柄与链路就绪状态、收发与错误计数）。
# bridge_node → /bridge_node/status（QoS(1)）；无运行期订阅者，仅测试订阅。

std_msgs/Header header
# SocketCAN接口名或达妙设备路径；字段名为兼容既有读取者保留。
string can_interface
# socket_open表示本地传输句柄已打开；interface_up为网口UP或达妙CAN初始化完成。
# 均不能据此认定下位机应用已就绪。
bool socket_open
bool interface_up
# 计数为进程内累计值；tx只表示内核接收整帧/完整串口封装，不保证下位机收到或执行。
uint64 tx_frames
# rx当前只统计完整CAN错误帧，与bus_errors同步递增，不含普通下位机控制指令。
uint64 rx_frames
# 收到的SocketCAN错误报告数量；达妙故障在last_error报告，不伪造内核错误帧。
uint64 bus_errors
# 混合统计队列撤销/过期、畸形输入和部分请求拒绝等，不是单一网络丢包率。
uint64 dropped_frames
string last_error
