# Copyright 2026 li
# SPDX-License-Identifier: Apache-2.0
# detect_node 的进程级诊断快照（帧率、推理耗时、队列丢弃计数、活动相位）。
# detect_node → /detect_node/status（1 Hz）；仓库内无运行期订阅者，仅诊断与录包。

# header.stamp 是诊断采样时刻，各帧率字段单位为 FPS。
std_msgs/Header header
float64 capture_fps
float64 seg_fps
float64 field_fps
float64 raw_fps
# 推理耗时以 alpha=0.1 指数平滑，单位为毫秒。
float64 seg_inference_ms
float64 field_inference_ms
# GPU／后处理字段取最近一次完成的样本，单位为毫秒。
float64 seg_gpu_ms
float64 seg_post_ms
# 场地后端运行在 CPU，GPU 耗时恒为零。
float64 field_gpu_ms
float64 field_post_ms
# 原图订阅者数是采样时的连接数量，不是已成功接收每一帧的确认数。
uint64 raw_subscribers
# 以下计数自节点启动起累计，不是每秒增量。
uint64 inference_replaced
uint64 raw_replaced
uint64 camera_reopens
uint64 seg_skipped
uint64 field_skipped
bool seg_active
bool field_active
# 当前实现只有一个推理工作线程，因此串行推理标志恒为 true。
bool serial_inference
