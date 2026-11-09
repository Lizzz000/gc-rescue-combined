# Copyright 2026 li
# SPDX-License-Identifier: Apache-2.0
# 单帧推理各阶段耗时明细（毫秒），供性能剖析与调参。
# detect_node → /detect_node/{seg,field}/timing（受 publish_timing 控制）；无运行期订阅者。

# header 沿用源图像，各耗时字段的单位均为毫秒。
std_msgs/Header header
float64 gpu_ms
float64 post_ms
float64 inference_ms
# 包含本机消息准备、像素复制、发布锁等待及结果发布，不含此条timing消息自身的发布。
# 不代表远端订阅者已收到或 HTTP 客户端完成解码。
float64 publish_ms
# total_ms 包含推理与结果发布，不包含采集、排队或网络传输。
float64 total_ms
uint64 instances
