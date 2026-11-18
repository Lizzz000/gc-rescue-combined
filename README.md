# GC Rescue

面向三轮全向底盘的搬运项目。Jetson 负责视觉、目标选择、运输任务和夹爪；STM32 C 板负责姿态估计、底盘控制和电机驱动。两端通过 **USB CDC** 通信，电机仍接 STM32 的 **CAN1**。

仓库只维护 `main` 一条分支。最新双端算法、通信修复和回归测试已归入主线；早期试验分支的处理方式见 [主线整理与验证记录](docs/validation-2026-11-15.md)。

## 目录与职责

| 目录 | 内容 |
| --- | --- |
| [STM32](STM32/README.md) | STM32F407 固件、底盘与姿态算法、USB 协议、主机回归和硬件联调说明 |
| [Jetson](Jetson/docs/README.md) | ROS 2 视觉、跟踪、任务、USB 桥接、S288 夹爪及独立手动控制工具 |
| [docs](docs/validation-2026-11-15.md) | 主线整理、离线验证结果和实车验收边界 |

Jetson 的完整自动部署是 7 个进程、8 个节点；独立手动控制和录像工具不随自动流程启动。节点与包的对应关系见 [Jetson 说明](Jetson/docs/README.md)。

## 一次运输如何工作

1. 首次把车停在左侧安全区前并对准。底盘使能就绪、里程计有效后保存起点，原地转 180°。转向到位后接收新目标中心，丢目标时停车等待。
2. 靠近后执行夹取和 ROI 核验。首趟只接受一个绿色目标，后续允许黑色、橙色目标参与选择。
3. 按里程计返回本趟起点和航向。跨区投送时用目的区中心做横向对准；同侧投送在返航位姿复核通过后直接准备盲推。
4. 锁航向限时推进，到时发零速并请求松爪，松爪成功后限时退出。送达后用目的区的两帧新中心复核位置，把当前里程计位姿保存为下一趟起点。
5. 留在本次投送侧，原地转 180° 并重开目标识别，开始下一趟。

安全区以 **20 mm 黑带的中心线**划分左右区域，目标点取对应四边形对角线交点。类型 1 心跳已删除，夹爪完成使用独立的类型 9 回执。正常丢目标时周期中心帧仍发送 `valid=0`；通信断流与底盘故障沿用现有停车处理。详细载荷和时序见 [USB 协议](Jetson/docs/usb-protocol.md)。

## 构建与测试入口

以下各组命令从仓库根目录开始；工具链和依赖须先安装。Python 示例使用 `python`，Linux 上可按安装情况替换为 `python3`。

### STM32

需要 ARM GNU 工具链、CMake（3.22 及以上）和 Ninja。

```sh
cd STM32/project
cmake --preset Debug
cmake --build --preset Debug
```

默认 `Debug` 为六轴 USB 手动版，`Debug-UART6-Manual` 用于 COM7 网页遥控，`Debug-Auto` 用于 Jetson 视觉搬运。每个预设只链接对应控制器，产物位于 `STM32/project/build/<预设>/`。正式手动和自动固件在姿态就绪后申请零速使能，正常运行限速为 0.40 m/s、3.0 rad/s。自动搬运出发前还需完成场地标定并收到视觉会话和夹爪授权。九轴手动版使用 `Debug-9axis`。[手动测试入口](STM32/manual_test/README.md) · [STM32 联调说明](STM32/docs/JETSON_USB_TEST.md)

主机回归需要 Python 3 与主机 GCC/G++，从仓库根目录运行：

```sh
python STM32/tests/run_native.py
python STM32/tests/check_jetson_codec.py --jetson-root Jetson
python STM32/tests/check_gripper_budget.py
```

磁标定回归另运行 `python STM32/tests/test_mag_calibration.py`，需要 NumPy/SciPy。

### Jetson

完整工作区需要 ROS 2 Humble、CUDA/TensorRT、OpenCV 和相应 Jetson 多媒体 SDK；模型引擎需按实际设备准备。进入 `Jetson/` 后，在已经加载 ROS 环境的终端构建：

```sh
cd Jetson
source /opt/ros/humble/setup.bash
colcon build --base-paths src --packages-select \
  interface_pkg vision_pkg track_pkg mission_pkg bridge_pkg bringup_pkg \
  --cmake-args -DBUILD_TESTING=ON
source install/setup.bash
ros2 launch bringup_pkg rescue.launch.py run:=vision
```

`run:=vision` 用于检测与图传，也需要匹配的 TensorRT 引擎；仓库未附带该引擎。完成夹爪及运动参数标定后，还须在 STM32 配置中显式启用 `JETSON_AUTO_ENABLED`、重新编译并烧录，再使用 `run:=all` 启动完整运输流程。两种启动方式二选一。配置入口为 [rescue.yaml](Jetson/src/bringup_pkg/config/rescue.yaml) 和 [rescue_advanced.yaml](Jetson/src/bringup_pkg/config/rescue_advanced.yaml)。完整部署细节见 [Jetson 运行说明](Jetson/docs/README.md)。

不依赖 ROS/GPU 的主机测试入口：

| 测试 | 入口与依赖 |
| --- | --- |
| 安全区识别 | [field](Jetson/src/vision_pkg/test/field/README.md)，C++17、OpenCV、Python/PyYAML |
| 分割 CPU 后处理 | [postprocess](Jetson/src/vision_pkg/test/postprocess/README.md)，C++17、OpenCV |
| 跟踪与选择 | [track host](Jetson/src/track_pkg/test/host/README.md)，Eigen、GoogleTest |
| Bridge 协议与握手 | [run.sh](Jetson/src/bridge_pkg/test/host/run.sh) / [run.ps1](Jetson/src/bridge_pkg/test/host/run.ps1)，主机 G++ |
| Mission 状态机与几何 | [run.ps1](Jetson/src/mission_pkg/test/host/run.ps1)，主机 G++ |
| 录像协议 | [record host](Jetson/src/record_pkg/test/host/README.md)，C++17 |

源码结构检查从仓库根目录运行 `python Jetson/tools/source/check_source.py`。

## 当前验证状态与待标定项

2026-11-17 增加 STM32 轮速反馈与 IMU 航向里程计，现有八向定距改为按实测位移投影闭环控制。接口、反馈有效期、超时处理和实车复核方法见 [里程计与定距控制](STM32/docs/ODOMETRY.md)。本轮未烧录或执行实车运动，自动模式和待标定配置保持原值。

2026-11-17 更新了标准安全区尺寸、紫框局部缺失时的几何拟合，以及画面裁切和偏心暗带的误判。八方向裁切原型已完成离线模拟，Jetson 运行程序仍使用当前帧的几何证据。主机紫框缺色专项 74 项通过，围栏和黑带压力回归有 7 条断言未过，实拍验收待完成。[尺寸和运行判定](Jetson/docs/README.md#安全区识别验证状态) · [八方向结果与复现](Jetson/src/vision_pkg/test/field/README.md#八方向裁切模拟)

2026-11-16 已保存双端联调修改、1.8 kg 底盘参数和场地相机配置；断电前核验了板端固件、USB 收包、IMU 与夹爪通信。安全区识别约 68 FPS；单次搬运尚未启动，三轮零速使能后的反馈和松爪效果仍待现场验收。最新状态见 [现场链路检查记录](Jetson/outputs/field_trial_20261116/COMMUNICATION_CHECK_END.txt)，参数见 [场地参数说明](Jetson/outputs/field_trial_20261116/FIELD_PARAMETERS.txt)。`outputs` 中仅选定的参数、工具和记录纳入版本控制，原始图像、大型逐帧数据、临时实验及编译产物保留在本地。

2026-11-15 已完成两端主机回归、协议互验、连续两趟运输模拟和四种 ARM 配置编译；安全区完整矩阵 **1162 项通过、0 项失败**。本轮没有执行真实机器人运输，实拍回放因缺少标注数据跳过。详细结果见 [验证记录](docs/validation-2026-11-15.md)。

自动运行前仍需实车标定：

- Jetson `gripper.move_s`：当前留空，`all/bridge` 会拒绝未标定的配置。
- STM32 的返航/对准范围、左右视觉窗口、ROI/恢复退距和盲推进退参数需按场地标定。返航与对准范围默认为 0。
- 相机、光照、夹爪位置与力矩、车辆速度和停车距离，以及真实 ROS/GPU/USB/S288 负载下的完整流程。

定距采用**轮速反馈与 IMU 航向里程计闭环**；最短请求距离为 10 mm，3 mm 的软件到位容差不代表真实地面精度。轮胎打滑、轮径误差和航向漂移仍需通过实际行程测量复核。两端应使用同一主线版本配套部署。
