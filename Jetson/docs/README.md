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

板端 ROS 工作区 `/home/nvidia/WorkSpace/GC_Rescue/Jetson`：

```bash
source /opt/ros/humble/setup.bash
source /home/nvidia/WorkSpace/GC_Rescue/Jetson/install/setup.bash
export PYTHONNOUSERSITE=1
export PATH=/usr/local/cuda/bin:$PATH
ros2 launch bringup_pkg rescue.launch.py run:=vision
ros2 launch bringup_pkg rescue.launch.py run:=all
```

默认请求相机以 1920×1080 MJPG、120 FPS 采集；分割、场地识别、原图发布和 HTTP 图传的上限为 100 FPS。类型 3、4 两种中心帧每 5 ms 交替发送，每种以 10 ms 为目标周期；夹爪动作以类型 9 独立回执，不再发送类型 1。实际帧率需在 Jetson 和相机上实测。板间链路是全速 12 Mbit/s，不是 480 Mbit/s。

自动控制的通信保护复用类型 3/4，不增加心跳或改变载荷。正常丢目标时继续发送 `valid=0`，STM32 继续搜索；夹爪、转向和等待阶段也持续发送。MissionStatus 超过既有 500 ms 有效期时 bridge 停发周期流。STM32 在 USB 离线、会话改变或周期流断开达到 250 ms 后触发制动，确认停稳或制动达到 500 ms 后请求失能；这些是检测和制动期限，不保证 250 ms 内物理停稳。故障锁存至显式重新初始化，恢复连接不会自动续跑。启动前必须收到当前 USB 会话中的周期帧，详细时序见 [板间帧说明](usb-protocol.md)。

自动 GStreamer 相机管线每次打开设备时应用 `rescue_advanced.yaml` 中的 `vision.detect.camera_brightness`；当前标定值为 150，设为 -1 可不修改设备亮度。该值只适用于自动 GStreamer 管线，其他后端或自定义管线需自行配置亮度；不同相机的支持范围和其他光照下的效果需实测。

当前现场固定光照下，亮度 150 在远景和近景的短时采样均保持有效，无安全区画面的采样没有误报；这是识别有效率，不代表位置已按真实坐标标定。使用旧版完整配置快照时，需要补充 `vision.detect.camera_brightness` 字段。

此前搬运实测中，车辆横向对准安全区后连续 796 帧未得到有效场地观测。当前 STM32 流程已改为：视觉接近并对准后，锁定当时航向，按配置距离直行，此段不再依赖后续视觉；通信与底盘故障制动仍生效。2026-11-17 起定距由三轮反馈与 IMU 航向估计位移，按本段目标方向的有符号投影闭环控制。`JETSON_AUTO_SAFETY_STRAIGHT_M` 保持为 0，需实车标定到至少 10 mm 后才能启用自动流程。位姿留在 STM32 本地，USB 类型和载荷保持原协议；里程计失效、无进展或超时不能作为到达放行。轮滑误差仍需尺量复核，本轮尚未烧录或复测车辆运动。详见 [里程计与定距控制](../../STM32/docs/ODOMETRY.md)。

手动网页通过 `--config` 读取顶层 `link.device` 与 `gripper.*`。`link.device` 是 STM32 的 USB OTG FS 设备，`gripper.device` 仍是 S288。`gripper.move_s` 仍是待标定空值，不要填数字。相对转角动作结束后舵机进入 Stop 模式，不再主动保持夹持力；该手动功能未接入自动搬运状态机。实际夹持、松爪位置和力矩须在现场确认。

`run:=vision` 只启动检测和图传。`run:=all` 启动上一节的 7 个进程。图传地址和端口来自 `rescue.yaml` 的 `stream_address`、`stream_port`，当前是 `0.0.0.0:8080`。

- `http://<板端>:8080/healthz`
- `http://<板端>:8080/view`
- `http://<板端>:8080/snapshot.jpg`
- `http://<板端>:8080/stream.mjpg`（同时最多 8 路，见 `mjpeg.cpp` 的 `kMaxStreamClients`）

手动控制由 `teleop_pkg/manual_control.py` 经 USB 发送类型 2，不启动 ROS 桥接。自动固件忽略该帧。STM32 须运行手动固件。bridge 与手动程序用同一把设备锁，不能同时打开 `link.device`。

板间通信固定使用 USB CDC，不再提供其他传输后端。连接后用 `ls -l /dev/serial/by-id/` 核对 `link.device`，与 S288 夹爪串口分别配置。当前 STM32 的 USB 序列号为 `004300353034510530343537`，更换板卡后需要更新路径。STM32 到电机仍使用 CAN1。

```bash
python3 /home/nvidia/WorkSpace/GC_Rescue/Jetson/src/teleop_pkg/manual_control.py --config /home/nvidia/WorkSpace/GC_Rescue/Jetson/src/bringup_pkg/config/rescue.yaml
```

上场地前的低速手动控制可追加 `--max-linear-mps 0.25 --max-angular-radps 0.5 --max-timed-s 1.5`。这些限制由后端同时作用于按住和定时接口，网页从 `/api/state` 读取有效范围；平移按合速度限幅，超长定时请求会被拒绝。不传这些参数时，平移合速度和转向上限为 0.50 m/s、4.0 rad/s。使用扩展范围前，STM32 USB 手动固件需同步更新。配置中的 USB 路径须与实际设备一致；不使用夹爪时可在现场配置副本中将 `gripper.device` 设为空字符串。

手动运行默认 0.40 m/s、3.0 rad/s。服务启动时等待接管；首次按键先发 0.1 s 有效零速，随后按 100 Hz 发送，速度朝零量化。松键和定时结束都保持有效零速，再次按键可直接起步。显式停车、按住期间丢失续租或活动链路中断会触发撤权保护。

手动页面的空格停车在输入框聚焦时仍有效；停车后仍按着方向键的自动重复不会重新起步，须松键后重新按下。页面为底盘请求附带 `client_id` 和递增 `sequence`，后端在同一锁内按序执行，迟到及重复请求不覆盖新动作、不延长租约。直接调用 HTTP API 时也可提供这两个字段；省略时保持旧调用方式，但没有跨连接的乱序保护。序号按页面独立记录，不能将其当作多个操作者之间的全局急停锁。

任务侧保留本进程已见 ROI 编号高水位；Target 单独重启造成重复或倒退编号时，新检查会返回无效、编号 0，并清除本次路线。旧送达记录仍只读确认，不会因此建立新路线。出现该诊断时应在停车后协调重启任务链，并核对运输阶段。Mission 纯核心回归已纳入 `colcon test --packages-select mission_pkg`。串口打开失败的实例只释放自己成功获取的独占标记，避免争抢设备时误清除已有进程的锁。

常用话题：`/detect_node/status`、`/detect_node/seg/detections`、`/detect_node/field/results`、`/detect_node/active_task`、`/track_node/seg/tracks`、`/select_node/target`、`/mission_node/status`、`/safety_node/target`、`/bridge_node/status`。`/detect_node/active_task` 和 `/mission_node/status` 是 `transient_local`。

服务：`/detect_node/set_field`、`/target_node/check_roi`、`/mission_node/arrived`、`/mission_node/command`。

### 安全区识别验证状态

安全区识别仍需实车校准。左右区域沿黑带中心线分割，安全区目标为各区域四边形对角线的交点；对矩形区域，这对应矩形中心经过透视投影后的位置。仿真采用已确认的 20 mm 黑带宽度，中心真值与消息中的分区定义一致。

红色和蓝色安全区共用标准几何。标准配置采用 660 × 360 mm 外框和 600 × 300 mm 内框，平面内缩每侧 30 mm。紫围栏高 20 mm，中央黑带长 300 mm、宽 20 mm、高 20 mm。标准鸟瞰图为 240 × 120 像素。红色或蓝色底面由 `vision.field.use_blue_interior` 选择。11 月 16 日的 660 × 640 mm 试验场地使用该次记录中的独立配置。

| 当前画面 | C++ 识别结果 |
| --- | --- |
| 近侧紫边中段断开，两端底缘仍清晰 | 用本帧两端的底缘重新拟合近边 |
| 一条侧边缺紫色，其余三边可靠且红色或蓝色内沿仍可测 | 用本帧地面色差补测缺失的一侧 |
| 单角出画，仍能测到相邻边 | 延伸实测边求交点，并在诊断中给出按地面度量的画外面积 |
| 红色或蓝色底面被画面裁断，候选整边贴住图像边缘 | 当前帧缺少完整分区的几何证据，正式观测为无效 |
| 黑带不可靠，或只有明显偏离中心的暗条 | 正式观测为无效，偏心暗条会标记 `split_suspect` |

八向裁切使用完整场地图像进行配准，目前是离线原型。正式接入前需标定实际相机内参和安装姿态，回放带标注的平移、转向和遮挡画面。逐方向误差、已知回归失败与运行命令见[安全区测试](../src/vision_pkg/test/field/README.md#八方向裁切模拟)。

2026-11-16 的 Jetson Orin Nano / OpenCV 4.10 板端验证中，完整矩阵 1162 条断言、中线专项 385 条及 ROS 图像适配 33 条全部通过。修复了杂物反向透视外框、伪弱边及稀疏锚点授权补线的问题，保留原覆盖率和误差门槛。图像适配直接持有八位 ROS 消息，移除了引入另一版本 OpenCV 的 cv_bridge 依赖；支持 BGR8/RGB8/BGRA8/RGBA8/MONO8，类别掩膜只接受 MONO8/8UC1。详情见 [安全区测试](../src/vision_pkg/test/field/README.md)。尚无带标注的实车回放数据，该项测试会明确跳过；真实场地识别成功率和位置精度仍需现场校准。

同日完成安全区性能优化：内色阴影掩膜按候选需要计算，ARM NEON 加速阴影候选像素扫描。分辨率、颜色和几何阈值、覆盖率门槛及 JPEG 质量均保持。130 张冻结输入组成 13 组序列，前后各运行 432 次，非计时结果及鸟瞰图像素全部一致；含 SEARCH、LOCATE、TRACK、遮挡保持、切换及动态配置。新增 144 条阴影掩膜像素等价断言通过，优化后的完整视觉回归无失败（标注回放因缺少数据跳过）。

现有桌面画面没有真实安全区，前后配对的 30 秒实测均正确保持 SEARCH/INVALID：识别诊断均值从 50.4 提升至 71.5 FPS，电脑经 SSH 隧道收到的完整 JPEG 从 50.5 提升至 69.6 FPS，均无相机重开或图传超时。固定桌面图的处理耗时中位数从 15.39 降至 8.39 ms；有效跟踪场景的收益较小，不能把搜索场景增益推广到所有场景，也不能据此认定真实场地精度已完成验证。记录位于 `outputs/preflight_20261116/field_live_comparison.json` 和 `field_equivalence_comparison.json`。

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
