# GC Rescue（Jetson）

在 Jetson 上完成货物分割、跟踪选择、运输阶段和安全区中心，再经 USB 交给下位机。图传与底盘控制分开。板间链路是 STM32 USB OTG FS 设备，Jetson 打开 `link.device`；电机仍在 CAN1。

## 包

| 包 | 职责 |
| --- | --- |
| interface_pkg | 共用消息和服务：检测、场地、任务、ROI、安全区、桥状态 |
| vision_pkg | USB 相机、按阶段切换的分割和场地推理、MJPEG 图传 |
| track_pkg | OC-SORT 跟踪，以及避开蓝色的目标选择 |
| mission_pkg | 运输阶段、ROI 检查、安全区中心 |
| bridge_pkg | 中心数据的 USB 桥，以及和 STM32 的运输握手、夹爪动作结果 |
| bringup_pkg | 唯一启动入口，用同一份配置拉起各角色 |
| record_pkg | 独立工具：相机流和 PC 预览 / MP4 录制 |
| teleop_pkg | 独立工具：经 USB 发送类型 2 手动速度和停车命令 |

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

相机帧进入 `detect_node`。seg 阶段发布检测，`track_node` 维持轨迹，`select_node` 选出一个目标。到达后 `mission_node` 调用 `target_node` 做 ROI。首趟只接受一个绿色；通过后目的地为左或右，并停在等待转向。命令 1 切到 field，服务成功且任务状态也变成 field 之后进入安全区导航。`safety_node` 计算对应区域对角线的交点，再除以画幅宽高，得到透视中心。`bridge_node` 把目标中心、安全区中心和握手报文送到 USB。命令 4 把 `transport_phase` 记为 Later，阶段仍是等待送达。命令 3 切回 seg 并重新寻找，此时黑色和橙色也可以被选中。

## 阅读顺序

1. `src/bringup_pkg/launch/rescue.launch.py`：启动入口，把实现交给 runtime。
2. `src/bringup_pkg/runtime/core.py`：唯一角色表。
3. `src/bringup_pkg/runtime/ros/launch.py`：校验配置、写出参数文件、按角色启动。
4. `src/bringup_pkg/config/rescue.yaml` 与 `rescue_advanced.yaml`：日常项和其余参数。
5. `src/vision_pkg/app/vision_runtime.cpp`，然后 `node/detect_node.cpp`、`camera/camera.cpp`、`yolo/cuda/nvmm_preprocess.hpp`、`stream/mjpeg/mjpeg.cpp`。
6. `src/track_pkg/node/track_node.cpp` 与 `select_node.cpp`。
7. `src/mission_pkg/control/controller.hpp`，然后 `node/mission_node.cpp`、`target_node.cpp`、`safety_node.cpp`。
8. `src/bridge_pkg/protocol/usb_protocol.hpp`，然后 `node/bridge_io.cpp`。
9. `src/interface_pkg/msg/` 与 `src/interface_pkg/srv/`。
10. `src/record_pkg/` 与 `src/teleop_pkg/` 各自阅读。

CPU 分割后处理在 `src/vision_pkg/yolo/postprocess.cpp`，GPU 提交在 `yolo.cpp`，ROS 消息转换在 `node/seg_ros.cpp`。纯后处理测试入口为 `src/vision_pkg/test/postprocess`，只需 OpenCV；跟踪及选择器测试入口为 `src/track_pkg/test/host`，只需 Eigen 和 GoogleTest。两者也接入各包的 `BUILD_TESTING`。跟踪 ID 全程使用整数，避免大编号经 float 矩阵传递时丢失精度；关联及运动模型保持。

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

默认请求相机以 1920×1080 MJPG、120 FPS 采集；分割、场地识别、原图发布和 HTTP 图传的上限为 100 FPS。类型 3、4 两种中心帧每 5 ms 交替发送，每种以 10 ms 为目标周期；夹爪动作以类型 9 独立回执，不再发送类型 1。实际帧率需在 Jetson 和相机上实测。板间链路是全速 12 Mbit/s，不是 480 Mbit/s。

自动控制的通信保护复用类型 3/4，不增加心跳或改变载荷。正常丢目标时继续发送 `valid=0`，STM32 继续搜索；夹爪、转向和等待阶段也持续发送。MissionStatus 超过既有 500 ms 有效期时 bridge 停发周期流。STM32 在 USB 离线、会话改变或周期流断开达到 250 ms 后触发制动，确认停稳或制动达到 500 ms 后请求失能；这些是检测和制动期限，不保证 250 ms 内物理停稳。故障锁存至显式重新初始化，恢复连接不会自动续跑。启动前必须收到当前 USB 会话中的周期帧，详细时序见 [板间帧说明](usb-protocol.md)。

自动 GStreamer 相机管线每次打开设备时应用 `rescue_advanced.yaml` 中的 `vision.detect.camera_brightness`；当前标定值为 150，设为 -1 可不修改设备亮度。该值只适用于自动 GStreamer 管线，其他后端或自定义管线需自行配置亮度；不同相机的支持范围和其他光照下的效果需实测。

当前现场固定光照下，亮度 150 在远景和近景的短时采样均保持有效，无安全区画面的采样没有误报；这是识别有效率，不代表位置已按真实坐标标定。使用旧版完整配置快照时，需要补充 `vision.detect.camera_brightness` 字段。

此前搬运实测中，车辆横向对准安全区后连续 796 帧未得到有效场地观测。当前 STM32 流程已改为：视觉接近并对准后，锁定当时航向，按配置距离直行，此段不再依赖后续视觉；通信与底盘故障制动仍生效。`JETSON_AUTO_SAFETY_STRAIGHT_M` 保持为 0，需实车标定为正值后才能启用自动流程。距离来自命令速度积分，不是轮速里程计；本轮尚未复测车辆运动。

手动网页通过 `--config` 读取顶层 `link.device` 与 `gripper.*`。`link.device` 是 STM32 的 USB OTG FS 设备，`gripper.device` 仍是 S288。`gripper.move_s` 仍是待标定空值，不要填数字。相对转角动作结束后舵机进入 Stop 模式，不再主动保持夹持力；该手动功能未接入自动搬运状态机。实际夹持、松爪位置和力矩须在现场确认。

`run:=vision` 只启动检测和图传。`run:=all` 启动上一节的 7 个进程。图传地址和端口来自 `rescue.yaml` 的 `stream_address`、`stream_port`，当前是 `0.0.0.0:8080`。

- `http://<板端>:8080/healthz`
- `http://<板端>:8080/view`
- `http://<板端>:8080/snapshot.jpg`
- `http://<板端>:8080/stream.mjpg`（同时最多 8 路，见 `mjpeg.cpp` 的 `kMaxStreamClients`）

手动控制由 `teleop_pkg/manual_control.py` 经 USB 发送类型 2，不启动 ROS 桥接。自动固件忽略该帧。STM32 须运行手动固件。bridge 与手动程序用同一把设备锁，不能同时打开 `link.device`。

板间通信固定使用 USB CDC，不再提供其他传输后端。连接后用 `ls -l /dev/serial/by-id/` 核对 `link.device`，与 S288 夹爪串口分别配置。当前 STM32 的 USB 序列号为 `0028003D3034470E31353233`，更换板卡后需要更新路径。STM32 到电机仍使用 CAN1。

```bash
python3 /home/nvidia/WorkSpace/GC_Rescue/Jetson/src/teleop_pkg/manual_control.py --config /home/nvidia/WorkSpace/GC_Rescue/Jetson/src/bringup_pkg/config/rescue.yaml
```

默认先使能零速 0.1 s，再按网页指令以 100 Hz 发送。速度按向零截断量化，不是四舍五入。此程序不会驱动自动搬运。

常用话题：`/detect_node/status`、`/detect_node/seg/detections`、`/detect_node/field/results`、`/detect_node/active_task`、`/track_node/seg/tracks`、`/select_node/target`、`/mission_node/status`、`/safety_node/target`、`/bridge_node/status`。`/detect_node/active_task` 和 `/mission_node/status` 是 `transient_local`。

服务：`/detect_node/set_field`、`/target_node/check_roi`、`/mission_node/arrived`、`/mission_node/command`。

### 安全区识别验证状态

安全区识别仍需实车校准。左右区域沿黑带中心线分割，安全区目标为各区域四边形对角线的交点；对矩形区域，这对应矩形中心经过透视投影后的位置。仿真采用已确认的 20 mm 黑带宽度，中心真值与消息中的分区定义一致。

2026-11-15 的 Windows/OpenCV 主机验证中，完整矩阵 1162 条断言通过，原先 9 条失败已修复；连续帧、颜色、几何、黑带、角点与真值专项也通过。贴线黑块、高光和强阴影继续使用原验收门槛，黑带完全遮挡仍必须无效。详情见 [安全区测试](../src/vision_pkg/test/field/README.md)。尚无带标注的实车回放数据，NEON、ROS/TensorRT 集成及实车成功率、位置精度、车辆运动仍待板端验收。

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
- 伪终端：软件验收使用临时创建的 USB 字节流替身，不访问实车设备。

## 流程图

`flowcharts/system.svg` 画出 7 个进程、8 个节点，以及一次运输用到的话题和服务。实线是话题，虚线是服务。节点划分见 `runtime/core.py` 第 19–31 行；话题名见各节点的 `createRosInterfaces` 和 `rescue_advanced.yaml`。

`flowcharts/vision.svg` 是一帧的两条出路：NVMM 上做分割或场地，同时把处理帧送给图传。采集管线在 `camera/camera.cpp` 第 101–129 行，GPU 前处理在 `yolo/cuda/nvmm_preprocess.hpp` 第 13–36 行，发布在 `detect_node.cpp` 第 298–362 行，HTTP 在 `stream/mjpeg/mjpeg.cpp`。

`flowcharts/mission.svg` 是 CyclePhase 0–8，以及命令 1–4 怎样换阶段。枚举在 `controller.hpp` 第 34–45 行，服务在 `mission_node.cpp` 第 99–120 行。

`flowcharts/usb.svg` 是 USB 握手顺序和两条中心周期帧。类型、载荷与失联制动规则见 [usb-protocol.md](usb-protocol.md)。到达分支在 `bridge_io.cpp` 的 `handleArrival`。
