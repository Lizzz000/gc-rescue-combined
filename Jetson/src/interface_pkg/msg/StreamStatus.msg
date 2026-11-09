# Copyright 2026 li
# SPDX-License-Identifier: Apache-2.0
# stream_node 的图像合成与推流诊断快照（帧率、合成耗时、观看人数、覆盖帧数）。
# stream_node → /stream_node/status（1 Hz，受 publish_status 控制）；无运行期订阅者，仅测试订阅。

# header.stamp 是诊断采样时刻，各帧率字段单位为 FPS。
std_msgs/Header header
float64 image_fps
float64 compose_fps
# 合成耗时以 alpha=0.1 指数平滑，单位为毫秒。
float64 compose_ms
# 图传 FPS 为成功提交间隔的指数平滑值（alpha=0.1），超期断流后为0，不代表客户端接收/显示率。
float64 stream_fps
uint64 viewers
# 最近取得同帧图层快照时的检测数：seg框数加有效field观测数，不保证该帧最终已发布；切阶段清0。
uint64 instances
# 三帧等待队列满时淘汰最旧待处理帧的累计数，不包括正在合成的帧或其他失效原因。
uint64 replaced_frames
