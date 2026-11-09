# Copyright 2026 li
# SPDX-License-Identifier: Apache-2.0
# 声明 detect_node 当前的检测相位（seg / field）与相位代次 epoch，供各节点做任务门控。
# detect_node → /detect_node/active_task（transient_local），mission_node 原样转发；订阅：track、select、stream、target、safety、bridge 六节点。

# 源端阶段切换时间，与图像 header 使用同一 ROS 时钟。
# 当前活动阶段使用 TASK_SEG/TASK_FIELD；NONE/BOTH 是保留常量，不是可执行的活动模式。
string TASK_NONE=none
string TASK_SEG=seg
string TASK_FIELD=field
string TASK_BOTH=both
std_msgs/Header header
# header.stamp 是源端阶段生效时间，供消费者建立旧帧截止，不是订阅端收到时间。
# 当前活动阶段只取 seg 或 field，其他取值使消费者停止有效输出。
string task
# 同一检测进程内每次切阶段递增，仅用于相等性判定，不能区分进程重启前后的会话。
# 消费者联合 task/epoch/阶段源时间判会话，不能仅因 epoch 较小就拒绝重启后的新源时间。
uint64 epoch
# epoch 不能替代源时间；当前检测和场地结果并不携带这个代次字段。
