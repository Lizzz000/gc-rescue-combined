# GC Rescue（Jetson）

在 Jetson 上完成货物分割、跟踪选择、运输阶段和安全区中心，再经 USB 交给下位机。图传与底盘控制分开。板间链路是 STM32 USB OTG FS 设备，Jetson 打开 `link.device`；电机仍在 CAN1。

## 包

| 包 | 职责 |
| --- | --- |
| interface_pkg | 共用消息和服务：检测、场地、任务、ROI、安全区、桥状态 |
| vision_pkg | USB 相机、按阶段切换的分割和场地推理、MJPEG 图传 |
| track_pkg | OC-SORT 跟踪，以及避开蓝色的目标选择 |
| mission_pkg | 运输阶段、投送区登记、整区中心、独立 ROI 诊断 |
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

相机帧进入 `detect_node`。seg 阶段发布检测，`track_node` 维持轨迹，`select_node` 选择目标。首次运动先经类型 22 切到整区识别，STM32 自动对准停稳后记录起点，转 180° 并经类型 19 切回 seg。Bridge 在受理夹取时锁存真实选中类别，成功后由 `mission_node` 登记目的区。首趟接受绿色，后续绿色和黑色送左区，橙色送右区。正式流程无需 ROI 检查。STM32 返航停稳后发送类型 17，命令 1 切到 field，服务成功且任务状态确认后进入投送阶段。STM32 按目的区定时横移，再盲推、松爪并退出。送达后，命令 4 将 `transport_phase` 记为 Later，任务进入 `LocalizingStart`。`safety_node` 取整块安全区内框的对角线交点并归一化，`bridge_node` 通过 USB 发送新中心，保留本趟目的区和 token。STM32 复核并记录新起点后通过命令 3 切回 seg，后续也可选择黑色和橙色目标。

## 阅读顺序

1. `src/bringup_pkg/launch/rescue.launch.py`：启动入口，把实现交给 runtime。
2. `src/bringup_pkg/runtime/core.py`：唯一角色表。
3. `src/bringup_pkg/runtime/ros/launch.py`：校验配置、写出参数文件、按角色启动。
4. `src/bringup_pkg/config/rescue.yaml` 与 `rescue_advanced.yaml`：日常项和其余参数。
5. `src/vision_pkg/app/vision_runtime.cpp`，然后 `runtime/detect_runtime.cpp`（采集、推理、帧发布和停机）和 `runtime/stream_runtime.cpp`（同帧配对、编码与图传）。ROS 接口在 `node/detect_node.cpp` 和 `node/stream_node.cpp`。
6. `src/track_pkg/node/track_node.cpp` 与 `select_node.cpp`。
7. `src/mission_pkg/control/controller.hpp`，然后 `node/mission_node.cpp`、`target_node.cpp`、`safety_node.cpp`。
8. `src/bridge_pkg/protocol/usb_protocol.hpp`，然后 `state/bridge_transactions.cpp`（投送区登记、独立到达诊断与命令回执）和 `node/bridge_io.cpp`（USB 轮询与 ROS 服务）。
9. `src/interface_pkg/msg/` 与 `src/interface_pkg/srv/`。
10. `src/record_pkg/` 与 `src/teleop_pkg/` 各自阅读。

CPU 分割后处理在 `src/vision_pkg/yolo/postprocess.cpp`，GPU 提交在 `yolo.cpp`，ROS 消息转换在 `node/seg_ros.cpp`。跟踪 ID 全程使用整数，避免大编号经 float 矩阵传递时丢失精度；关联及运动模型保持。

## 分层验收

### 主机独立回归

以下入口可在各包内独立运行，无需启动 ROS、使用 GPU 或连接实车设备。

| 回归 | 入口与依赖 |
| --- | --- |
| 分割 CPU 后处理 | [postprocess](../src/vision_pkg/test/postprocess/README.md)，C++17、OpenCV |
| NVMM 像素与映射顺序 | [nvmm](../src/vision_pkg/test/nvmm/README.md)，C++17、CMake，使用 SDK 桩 |
| 安全区 | [test/field](../src/vision_pkg/test/field/README.md#运行)，C++17、OpenCV、Python/PyYAML |
| 跟踪与选择 | [track host](../src/track_pkg/test/host/README.md)，Eigen、GoogleTest |
| Bridge 协议与握手 | [Linux run.sh](../src/bridge_pkg/test/host/run.sh) 或 [Windows run.ps1](../src/bridge_pkg/test/host/run.ps1)，主机 G++ |
| Mission 状态机与几何 | [Windows run.ps1](../src/mission_pkg/test/host/run.ps1)，主机 G++ |
| 装配与验收工具 | 在 `Jetson/` 下运行 `python -m unittest discover -s tools/tests`，Python/PyYAML |

安全区在 Windows、OpenCV 4.13.0 的主机基线有 7 条失败断言（压力矩阵 4 条、黑带专项 2 条、困难光照几何 1 条）。回归时用相同配置和环境对比具体断言。没有标注时，实拍回放按跳过记录。按上表入口运行安全区回归。 [已知基线](../src/vision_pkg/test/field/README.md#2026-11-17-验证结果)

### Linux ROS 软件合同

在仓库的 `Jetson/` 目录运行。先加载 ROS 2 Humble，安装 Jetson 生产构建所需的 OpenCV、GStreamer、CUDA、TensorRT 和多媒体 SDK。`validate.py build` 会按 Bridge 包声明一起构建 S288 测试依赖 `teleop_pkg`。使用能通过 `run:=all` 校验的现场配置，包括已标定的夹爪参数。`contracts` 使用本轮构建和安装的身份记录，运行前需加载同一份安装。

```bash
validation_dir="$(mktemp -d)"
config=/path/to/rescue.yaml
python3 tools/validate.py build --config "$config" --output "$validation_dir/build-report" \
  --build "$validation_dir/build" --install "$validation_dir/install"
source "$validation_dir/install/setup.bash"
python3 tools/validate.py contracts --config "$config" --output "$validation_dir/contracts" \
  --build "$validation_dir/build" --install "$validation_dir/install"
```

登记了 `vision_contract_yolo_transfer` 的构建还需要与配置一致的引擎、标签和 `--inputs` 指定的 3 张冻结图。合同阶段以软件配置覆盖实车 USB 路径，检查五个测试包的必需 CTest 登记和本轮产生的 XML。`interface_pkg` 参与构建和安装核验。用 `contract-status.json` 检查缺失、过期、跳过和失败项，各包的结果汇总在 `contract-results/`。安全区实拍回放可通过 `FIELD_REPLAY_DIR` 提供标注数据，跳过和失败都会使 ROS 合同验收无法通过。

### Jetson 冻结图整链

在 Jetson 上加载 ROS 2 Humble 和上一步的验收安装。输入目录放 3 张不同的 JPG/PNG，配置须有本机能够加载的 TensorRT 引擎和类别表，并通过 `run:=all` 的配置校验。现有冻结图 `capture` 使用独立诊断的 `16/01` 报文核对 ROI 兼容路径。要覆盖该诊断的成功结果，至少一张图须同时满足诊断 ROI 规则和目的区识别条件。正式无 ROI 目的区登记有独立的 Mission 和 Bridge 回归。

```bash
python3 tools/validate.py capture --config "$config" --inputs /path/to/frozen-images \
  --output "$validation_dir/capture" --build "$validation_dir/build" \
  --install "$validation_dir/install"
```

回放在生产推理入口前为本轮冻结图上传 BGRx NVMM，每次打开只上传一次。整链的 STM32 和 S288 各走本轮自建伪终端，报告保留 USB 帧和 S288 反馈证据。送达命令确认到 `LocalizingStart` 后，测试端再模拟重找。该场景验证真实 TensorRT 分割、ROS 话题与服务、图传及串口软件握手，实际相机、底盘运动和夹爪力学另在实车验收。

### 实车整链

使用正式 `rescue.launch.py` 和匹配的相机、自动 GStreamer/NVMM 管线及 TensorRT 引擎。

1. 核对 `all` 为 7 个进程、8 个节点，`vision` 在同一进程运行检测和图传。让 Bridge 退出后，核对正式 launch 关闭本次全栈进程。
2. 让 `MissionStatus` 超过配置的有效期，确认 Bridge 停止发送类型 3/4 周期帧。
3. 分别切换 `seg → field` 和 `field → seg`，确认阶段切换后旧帧不再发布，图传继续按同一时间戳和 `frame_id` 配对。

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

11 月 22 日的实车场地配置采用 15 mm 单边内缩。STM32 正式搬运的整区复定位参考为 `(0.5430, 0.3298)`，单绿物块夹取到达点为 `(0.5721, 0.8681)`。相机、夹爪、板间 USB 设备和图传设置集中在[现场配置](../src/bringup_pkg/config/field_20261122.yaml)。在已加载安装环境的 Jetson 上使用下列命令启动视觉。

```bash
ros2 launch bringup_pkg rescue.launch.py run:=vision config:="$(ros2 pkg prefix --share bringup_pkg)/config/field_20261122.yaml"
```

单独运行视觉时，可在另一终端执行 `ros2 service call /detect_node/set_field std_srvs/srv/SetBool "{data: true}"` 切换到安全区识别。正式图纸配置 `rescue.yaml` 的单边内缩为 30 mm。改用图纸值前，应在同一停位重新核对归一化中心。现场配置和图纸配置都默认关闭 HTTP 图传。

相机的现场标定保存在 `rescue_advanced.yaml` 的 `vision.detect.camera_brightness` 和 `vision.detect.camera_controls`。当前标定为亮度 175、对比度 68、饱和度 90、色调 200、伽马 138、增益 48、手动曝光 150，并开启自动白平衡和连续自动对焦。这些数值是当前相机的 V4L2 控件值。使用完整配置快照启动时，快照中的参数优先，现场改动应同步到实际启动配置。

在 Jetson 上用正式入口启动 `vision`、`detect` 或 `all` 时，自动 GStreamer 管线会在创建节点前通过 `v4l2-ctl`（来自 `v4l-utils`）先设置曝光等自动模式，再写手动参数，并回读全部目标值。缺少工具、设备写入失败或回读不符都会中止本次启动。文件回放、自定义管线和单独的 `stream` 角色跳过这一步。检查正在出图的相机时可直接读取驱动值：

```bash
v4l2-ctl -d /dev/video0 --get-ctrl=brightness,contrast,saturation,hue,gamma,gain,auto_exposure,exposure_time_absolute,white_balance_automatic,focus_automatic_continuous
```

下一次带货联调使用 [STM32 实车启动顺序](../../STM32/docs/JETSON_USB_TEST.md#下一次现场单绿验收)。此前搬运实测中，车辆横向对准安全区后连续 796 帧未得到有效场地观测。当前 STM32 正式流程的横移、盲推和直线退出使用锁航向的定时动作，各段停稳后才交接。退出并确认送达后，STM32 按本趟实测横移量反向粗复位并恢复安全区航向，再用整区中心精调。粗复位和视觉共用 10 s，停稳后两张新帧入窗才记起点。返航采用 20 s 总时限，停稳后复核位姿。正式搬运本场的 `JETSON_AUTO_ALIGN_MAX_M` 为 0.25 m。`JETSON_AUTO_SAFETY_STRAIGHT_M` 只在单次底盘联调中标定。位姿留在 STM32 本地，正式路由用 `16/02`，独立诊断用 `16/01`。通信与底盘故障制动继续生效。轮滑和实际行程须在现场复核。详见 [里程计与定距控制](../../STM32/docs/ODOMETRY.md)。

手动网页通过 `--config` 读取顶层 `link.device` 与 `gripper.*`。`link.device` 是 STM32 的 USB OTG FS 设备，`gripper.device` 是 S288 串口。启动自动桥或手动网页前将夹爪摆到机械张开位置，会话用当时的输出轴编码器读数作零点。此夹爪本次机械张开位的读数为 6873 格，编码器每圈 8192 格。卸力后的回弹位置会偏离机械张开位，重连前应复核零点。

- S288 闭合目标 −50°，张开目标 0°。配置为 `Kp=2.0`、`Kd=0.10`、基本动作时间 1.0 s，前 0.8 s 对目标角平滑插值。闭合与张开的电机指令补偿分别为 +0.35°、+5.0°。
- 闭合后发送 Stop，记录输出轴角差，夹住物块而提前受阻时仍按原动作协议交接。张开时用独立输出轴编码器确认连续三次低速反馈均在 ±2°，随后持续轻力保位。空载闭合也按 ±2° 验收。保位的位置比例项力矩上限为 0.14 N·m。手动 Stop、接续下一个动作、退出或故障会结束保位。
- 2026-11-22 空载标定中，闭合 Stop 后为 −50.19° 至 −50.01°。张开保位读数为 +0.48° 至 +0.70°，最长一轮测到动作开始后第 20 秒。最终自动代码以 6868 格建立会话，两轮闭合实际均为 −49.26°，张开保位读数在 ±1° 内。手动端用卸力后 6843 格的读数另建会话时，闭合为 −47.90° 和 −47.86°。带货夹持和释放仍需实测。

默认配置下，`run:=vision` 和 `run:=all` 均关闭 HTTP 图传，8080 端口不监听。相机采集、物块分割和安全区识别按当前任务运行。

调试时在启动命令末尾加 `stream:=true`，例如：

```bash
ros2 launch bringup_pkg rescue.launch.py run:=vision stream:=true
```

省略 `stream` 时沿用 `rescue_advanced.yaml` 的 `vision.stream.stream_enabled`；`stream:=false` 可显式关闭。开关在启动时生效。启用后，地址和端口由 `rescue.yaml` 的 `stream_address`、`stream_port` 决定，默认监听 `0.0.0.0:8080`。

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

独立 ROI 诊断保留任务进程已见的路由编号高水位；Target 单独重启造成重复或倒退编号时，新检查会返回无效、编号 0，并清除本次路线。旧送达记录仍只读确认，不会因此建立新路线。出现该诊断时应在停车后协调重启任务链，并核对运输阶段。Mission 纯核心回归已纳入 `colcon test --packages-select mission_pkg`。串口打开失败的实例只释放自己成功获取的独占标记，避免争抢设备时误清除已有进程的锁。

常用话题：`/detect_node/status`、`/detect_node/seg/detections`、`/detect_node/field/results`、`/detect_node/active_task`、`/track_node/seg/tracks`、`/select_node/target`、`/mission_node/status`、`/safety_node/target`、`/bridge_node/status`。`/detect_node/active_task` 和 `/mission_node/status` 是 `transient_local`。

正式服务包括 `/detect_node/set_field`、`/mission_node/register_pickup`、`/mission_node/command`。独立诊断使用 `/target_node/check_roi` 和 `/mission_node/arrived`。

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

- 独立诊断 ROI：归一化矩形，统计与它相交的检测。当前左上角 `(0.25, 0.5)`，宽和高都是 `0.5`。
- CyclePhase：运输握手阶段 0–10，定义在 `controller.hpp`。它和识别模式 `seg` / `field` 是两套状态。
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
- 命令 1、2、3、4、5、6 分别为返航后阶段确认、安全区到达、重新寻找、送达确认、夹爪受理和首次整区定位。1、2、4 携带本趟路由编号，正式流程中等于成功的夹取请求号。
- 伪终端：软件验收使用临时创建的 USB 字节流替身，不访问实车设备。

## 流程图

- [进程与设备](flowcharts/system.svg)展示 7 个进程、8 个节点和两条独立 USB 设备链路。
- [视觉与图传](flowcharts/vision.svg)展示 TensorRT 分割、OpenCV 安全区识别及按需开启的 MJPEG 图传。
- [任务阶段](flowcharts/mission.svg)展示 CyclePhase 0–10、首次整区对准、正式投送与独立诊断。
- [USB 握手](flowcharts/usb.svg)展示动作与回复编号、正式 `16/02` 路由以及两次整区建点。载荷与失联制动规则见 [USB 协议](usb-protocol.md)。
