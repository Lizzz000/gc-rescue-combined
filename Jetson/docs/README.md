# GC Rescue（Jetson）

在 Jetson 上完成货物分割、跟踪选择、运输阶段和安全区中心，再经 CAN 交给下位机。图传与底盘控制分开。

## 包

| 包 | 职责 |
| --- | --- |
| interface_pkg | 共用消息和服务：检测、场地、任务、ROI、安全区、桥状态 |
| vision_pkg | USB 相机、按阶段切换的分割和场地推理、MJPEG 图传 |
| track_pkg | OC-SORT 跟踪，以及避开蓝色的目标选择 |
| mission_pkg | 运输阶段、ROI 检查、安全区中心 |
| bridge_pkg | 中心与速度的 CAN 桥，以及和 STM32 的运输握手 |
| bringup_pkg | 唯一启动入口，用同一份配置拉起各角色 |
| record_pkg | 独立工具：相机流和 PC 预览 / MP4 录制 |
| teleop_pkg | 独立工具：通过 USB-CAN 发送定时速度和停车命令 |

`record_pkg` 和 `teleop_pkg` 不在完整部署的角色表里。

## 进程、节点与角色

完整部署是 7 个进程、8 个节点。`vision` 在同一个 `vision_runtime` 里跑检测和图传。角色表在 `src/bringup_pkg/runtime/core.py` 第 19–31 行。

| 角色 | 可执行文件 | 节点 |
| --- | --- | --- |
| vision | vision_runtime | detect_node、stream_node |
| track | tracking_runtime | track_node |
| select | tracking_runtime | select_node |
| mission | mission_runtime | mission_node |
| target | mission_runtime | target_node |
| safety | mission_runtime | safety_node |
| bridge | bridge_runtime | bridge_node |

`detect` 和 `stream` 是只含一个节点的角色。`run:=all` 使用上表这 7 个进程，不再额外启动它们。

## 一次运输

相机帧进入 `detect_node`。seg 阶段发布检测，`track_node` 维持轨迹，`select_node` 选出一个目标。到达后 `mission_node` 调用 `target_node` 做 ROI。首趟只接受一个绿色；通过后目的地为左或右，并停在等待转向。命令 1 切到 field，服务成功且任务状态也变成 field 之后进入安全区导航。`safety_node` 取对应侧四个顶点的均值，再除以画幅宽高，得到中心。`bridge_node` 把目标中心、安全区中心和握手报文送到 CAN。命令 4 把 `transport_phase` 记为 Later，阶段仍是等待送达。命令 3 切回 seg 并重新寻找，此时黑色和橙色也可以被选中。

## 阅读顺序

1. `src/bringup_pkg/launch/rescue.launch.py`：启动入口，把实现交给 runtime。
2. `src/bringup_pkg/runtime/core.py`：唯一角色表。
3. `src/bringup_pkg/runtime/ros/launch.py`：校验配置、写出参数文件、按角色启动。
4. `src/bringup_pkg/config/rescue.yaml` 与 `rescue_advanced.yaml`：日常项和其余参数。
5. `src/vision_pkg/app/vision_runtime.cpp`，然后 `node/detect_node.cpp`、`camera/camera.cpp`、`yolo/cuda/nvmm_preprocess.hpp`、`stream/mjpeg/mjpeg.cpp`。
6. `src/track_pkg/node/track_node.cpp` 与 `select_node.cpp`。
7. `src/mission_pkg/control/controller.hpp`，然后 `node/mission_node.cpp`、`target_node.cpp`、`safety_node.cpp`。
8. `src/bridge_pkg/protocol/can_protocol.hpp`，然后 `node/bridge_io.cpp`。
9. `src/interface_pkg/msg/` 与 `src/interface_pkg/srv/`。
10. `src/record_pkg/` 与 `src/teleop_pkg/` 各自阅读。

## 运行

板端工作区 `/home/nvidia/WorkSpace/GC_Rescue`：

```bash
source /opt/ros/humble/setup.bash
source /home/nvidia/WorkSpace/GC_Rescue/install/setup.bash
export PYTHONNOUSERSITE=1
export PATH=/usr/local/cuda/bin:$PATH
ros2 launch bringup_pkg rescue.launch.py run:=vision
ros2 launch bringup_pkg rescue.launch.py run:=all
```

默认请求相机以 1920×1080 MJPG、120 FPS 采集；分割、场地识别、原图发布和 HTTP 图传的上限为 100 FPS，CAN 三种周期帧错开约 3.33 ms 轮发，每种帧以 10 ms 为目标周期。实际帧率与总线负载需在 Jetson 和相机上实测。

`run:=vision` 只启动检测和图传。`run:=all` 启动上一节的 7 个进程。图传地址和端口来自 `rescue.yaml` 的 `stream_address`、`stream_port`，当前是 `0.0.0.0:8080`。

- `http://<板端>:8080/healthz`
- `http://<板端>:8080/view`
- `http://<板端>:8080/snapshot.jpg`
- `http://<板端>:8080/stream.mjpg`（同时最多 8 路，见 `mjpeg.cpp` 的 `kMaxStreamClients`）

手动定时运动由 `teleop_pkg/move_timed.py` 直接使用达妙 USB-CAN，不启动 ROS 桥接或自动搬运程序。独立手动帧使用 `0x17F`，自动固件忽略该帧，不把它视为任务心跳；必须同步更新 STM32 与脚本。STM32 须运行手动联调固件，USB-CAN 须空闲，不能与桥接同时占用。

```bash
# 不带参数进入中文交互菜单
python3 /home/nvidia/WorkSpace/GC_Rescue/Jetson/src/teleop_pkg/move_timed.py

# 也可直接发送一次动作
python3 /home/nvidia/WorkSpace/GC_Rescue/Jetson/src/teleop_pkg/move_timed.py forward 0.08 1.2
python3 /home/nvidia/WorkSpace/GC_Rescue/Jetson/src/teleop_pkg/move_timed.py stop
```

终端中使用方向键选择平移、`Q/E` 左右转、`P` 设置速度和时间、`V` 输入组合速度、`X` 退出。每次按键触发一次定时动作，动作期间的其他方向键被丢弃；按 Ctrl+C 可请求停车。屏幕显示预计剩余时间，不代表实际电机反馈。非交互终端和不支持的终端沿用数字菜单。命令行方向可用 `forward`、`backward`、`left`、`right`、`ccw`（逆时针）或 `cw`（顺时针）；后面依次填写速度幅值和秒数。也可用 `velocity vx vy wz 秒数` 指定组合速度，使用 `--device 设备路径` 指定其他达妙适配器。平移单位 m/s，车体转向单位 rad/s。脚本先发 0.5 秒零速准备，再以 100 Hz 发送速度；时间到、执行 `stop` 或按 Ctrl+C 后发送零速并撤权。指定时间包含加速，速度上限由 STM32 配置决定。此脚本不会驱动自动搬运流程。

常用话题：`/detect_node/status`、`/detect_node/seg/detections`、`/detect_node/field/results`、`/detect_node/active_task`、`/track_node/seg/tracks`、`/select_node/target`、`/mission_node/status`、`/safety_node/target`、`/bridge_node/status`。`/detect_node/active_task` 和 `/mission_node/status` 是 `transient_local`。

服务：`/detect_node/set_field`、`/target_node/check_roi`、`/mission_node/arrived`、`/mission_node/command`。

## 术语

- ROI：归一化矩形。到达后统计与它相交的检测。当前左上角 `(0.25, 0.5)`，宽和高都是 `0.5`。
- CyclePhase：运输握手阶段 0–8，定义在 `controller.hpp`。它和识别模式 `seg` / `field` 是两套状态。
- transport_phase：`0` 是首趟，选择器只要一个绿色；`1` 是 Later，本趟送达确认之后，黑色和橙色也可以入选。
- epoch：`TaskState` 里的代次。同一次 `detect_node` 进程每次切换 seg/field 都会加一。
- cycle_generation：`mission_node` 每次受理模式切换时加一，下游用它丢掉旧的坐标授权。
- active_task：当前识别模式，活动值只有 `seg` 或 `field`。
- NVMM：留在 Jetson 显存里的视频缓冲。分割前处理直接读这块缓冲。
- BEV：场地鸟瞰图的像素宽高，字段是 `FieldObservation.bev_width` 和 `bev_height`。左右区角点是原图像素。
- destination：`0` 未定，`1` 左区，`2` 右区。`SafetyTarget.side` 使用同一套取值。
- FrameGate：按源时间丢掉过期、回退或不属于当前阶段的帧。无新帧时的撤销期限是 `reset_timeout_s`，当前为 1 秒；100 毫秒只是轮询周期。
- OC-SORT：按类别关联检测框。切阶段会清空轨迹输出；进程内的 ID 计数不回到 0。
- letterbox：把原图等比缩进模型输入，空边填固定值，再除以 255 写成 NCHW。
- 命令 1–4：转向完成、安全区到达、重新寻找、送达确认。1、2、4 携带本趟 `roi_request_id` 的低 32 位。
- vcan：网络命名空间里的虚拟 CAN。隔离验证只向它发帧。

## 流程图

`flowcharts/system.svg` 画出 7 个进程、8 个节点，以及一次运输用到的话题和服务。实线是话题，虚线是服务。节点划分见 `runtime/core.py` 第 19–31 行；话题名见各节点的 `createRosInterfaces` 和 `rescue_advanced.yaml`。

`flowcharts/vision.svg` 是一帧的两条出路：NVMM 上做分割或场地，同时把处理帧送给图传。采集管线在 `camera/camera.cpp` 第 101–129 行，GPU 前处理在 `yolo/cuda/nvmm_preprocess.hpp` 第 13–36 行，发布在 `detect_node.cpp` 第 298–362 行，HTTP 在 `stream/mjpeg/mjpeg.cpp`。

`flowcharts/mission.svg` 是 CyclePhase 0–8，以及命令 1–4 怎样换阶段。枚举在 `controller.hpp` 第 34–45 行，服务在 `mission_node.cpp` 第 99–120 行。

`flowcharts/can.svg` 是 v3 握手顺序和三条周期帧。标识与载荷在 `can_protocol.hpp` 第 58–72 行、第 166–227 行；到达分支在 `bridge_io.cpp` 的 `handleArrival`（约第 219 行）。
