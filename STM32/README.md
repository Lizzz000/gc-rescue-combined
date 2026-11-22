# STM32 C 板固件

`project/` 是 STM32F407 CubeMX/CMake 工程，`tests/` 包含主机回归，`docs/` 包含硬件配置、板间 USB 和电机 CAN1 联调说明，`LICENSES/` 保留第三方许可。

在具备 ARM GNU 工具链、CMake 和 Ninja 的环境中，进入 `project/` 后运行 `cmake --preset Debug` 和 `cmake --build --preset Debug`。可用预设还有 `Debug-9axis`、`Debug-Auto`、`Debug-Auto-Chassis-Test`（无夹爪的底盘联调）、`Debug-UART6-Manual`、`Debug-Tuning`（SWD 显式调参）和 `Release`。主机测试在本目录运行 `python tests/run_native.py`；可用 `--suite` 筛选套件、`--list-suites` 列出名称，`--python` 在 native 之后追加偏航 ADRC 初测复算、协议互验和夹爪总时间预算检查。也可单独运行 `python tests/check_jetson_codec.py --jetson-root ../Jetson` 和 `python tests/check_gripper_budget.py`（使用主机 g++）。

默认使用六轴姿态且自动搬运关闭。启用自动模式前需按 [联调说明](docs/JETSON_USB_TEST.md) 完成 Jetson 夹爪联调、运动参数标定和板端验证。

与参考项目 `C_project` 的主要功能差异如下；参考项目仅用于对比。

| 功能 | `C_project` | 当前 `STM32` |
|---|---|---|
| 板间通信 | Jetson 与电机共用 CAN1 | USB 传输板间帧，CAN1 只连接电机 |
| 手动控制 | Jetson 速度帧联调 | 独立类型 2 手动控制，零速准备后接收运动指令，断流触发制动 |
| 夹爪 | STM32 的 TIM1 舵机驱动 | Jetson 执行夹爪动作，STM32 发送到点请求并等待放行；旧舵机业务已移除 |
| 视觉靠近 | 固定参数图像跟踪滤波 | 自适应卡尔曼滤波及近窗修正，仍需实车标定 |
| 底盘保护 | 各控制模块检查反馈 | 共用电机就绪检查，邮箱忙时跳拍，姿态异常先制动，斜坡步长最多 50 ms |
| 转向上限 | 车体 1.0 rad/s，纯转各轮约 3.09 rad/s | 连续转向上限 4.0 rad/s，定角转向仍用正常值 3.0 rad/s |
| 姿态与默认模式 | 六轴默认、九轴可选，自动关闭 | 保留相同传感器、温控、VOFA 和默认模式，自动参数完成标定后再启用 |

此次整理精简电机单组过滤器配置，移除 CAN BSP 中的 USB 上行包装，由自动流程直接编码并调用 USB；顶层 CMake 只启用 C/ASM 并删除空配置，同时修正遗留 CAN/舵机说明。CAN2、TIM1 的 CubeMX 初始化配置仍保留，当前没有对应业务启动。

进一步删除 BMI088/IST8310 的四个空初始化接口、BMI088 未使用的 `status/time` 字段和 PID 增量式遗留历史缓存，以及无引用的测试桩和宏。传感器外设仍由 CubeMX 初始化；手动、自动、六轴/九轴、温控和保护逻辑保留，USB 调试记录与旧滤波对照测试保留。

正式运行使用 `CHASSIS_RUN_SPEED_MPS`（0.40 m/s）与 `CHASSIS_RUN_YAW_RATE_RAD_S`（3.0 rad/s）。手动和自动固件各自管理上电零速使能、接管与故障。手动硬上限为 0.50 m/s 合速度和 4.0 rad/s。连续速度按实测周期限制加减速并经 40 ms 平滑，再换算为三轮 CAN1 目标。自动视觉靠近时随距离减速，安全区精调使用独立的低速参数。正式视觉搬运使用 `Debug-Auto` 预设；单次底盘联调用 `Debug-Auto-Chassis-Test`。

底盘的周期、超时、限速和控制增益集中在 [chassis_config.h](project/Core/Control/Chassis/chassis_config.h)，各轮 ID、方向、齿比和默认电机参数集中在 [chassis_config.c](project/Core/Control/Chassis/chassis_config.c)。自动流程的场地和视觉参数由 [jetson_auto_config.h](project/Core/Control/Jetson/jetson_auto_config.h) 管理。修改固件参数后需重新编译。常用速度配置如下。

- `CHASSIS_STRAIGHT_MAX_SPEED_MPS`、`CHASSIS_DISTANCE_MAX_SPEED_MPS`：平移与定距速度上限。
- `CHASSIS_VECTOR_MAX_WZ_RAD_S`、`CHASSIS_TURN_OUTPUT_LIMIT`：连续转向硬上限和定角转向正常上限。
- `JETSON_AUTO_APPROACH_SPEED_MPS`、`JETSON_AUTO_DISTANCE_SPEED_MPS`、`JETSON_AUTO_PUSH_SPEED_MPS`：视觉靠近、定距及盲推/后退的目标速度。
- `JETSON_AUTO_RETURN_SPEED_MPS`、`JETSON_AUTO_ALIGN_SPEED_MPS` 用于返航和安全区视觉对准。

目标速度不得超过对应上限；换算和加减速在底盘控制层执行。类型 3、4 是中心帧，须与 Jetson 同时部署。MissionStatus 过期时 bridge 停发周期流并撤销业务准入，因此独立运行 bridge 也需要配套 Mission 节点。板间类型 1 已删除，夹爪完成通过类型 9 回执。`gripper.move_s` 仍是待标定空值，不要填数字。

自动流程不增加心跳，复用合法类型 3/4 的接收时效：USB 离线、会话改变或周期流断开达到 250 ms 后触发制动，确认停稳或制动达到 500 ms 后请求失能；故障锁存到显式重新初始化，恢复连接不会自动续跑。启动前须先收到当前 USB 会话的周期帧。正常丢目标仍发送 `valid=0` ，底盘停车等待，夹爪、转向和等待期间也持续发送。250 ms 是断流判定期限，实际停车仍受控制节拍和机械减速影响；Mission 掉线还先经过 Jetson 的 500 ms 状态有效期。

自动靠近使用 [Algorithm/Vision](project/Core/Algorithm/Vision/README.md) 的自适应图像坐标滤波；参数和实际跟踪效果仍需实车验证与标定。

自动搬运首次从左侧起点出发，先原地转 180°。夹取和 ROI 通过后按里程计返回起点；异侧才使用类型 4 横向对准。盲推进退使用锁航向的限时速度。退出并确认送达后，车在本次目的区视觉复核并保存下一趟起点。单次底盘联调仍使用 `JETSON_AUTO_SAFETY_STRAIGHT_M`。详见 [动作与握手](docs/JETSON_USB_TEST.md#机械动作与握手) 和 [里程计与定距控制](docs/ODOMETRY.md)。

2026-11-17 里程计采用纯 C 模块 `chassis_odometry`，沿用现有三轮几何、轮序、方向、齿比和航向 PID。参考 ros2_controllers 的平面圆弧积分思路，正式入口使用三轮位置增量、平面 ESKF 航向和 SE(2) 积分，不依赖 ROS/Eigen。`Chassis_Get_Odometry` 用于读取估计状态，`Chassis_Reset_Odometry` 仅允许失能或已确认停车时调用。反馈丢失期间不补积分，恢复首组仅重建基线；10 mm 定距交接容差是软件判据，轮滑和航向漂移仍需实车测量。

2026-11-15 算法整理：温控 PID 按加热器实际的 `[0,4500]` 输出范围抗积分饱和；航向 PID 与 Mahony 去掉非必要双精度运算，定距轨迹仍保留原精度。BMI088 连续寄存器读取合并 HAL 调用，线上字节和片选时序不变。USB CRC 使用 16 项表递推，协议不变。离线磁标定改用解析 Jacobian，模型、求解容差和现有标定系数保持。

`tests/run_native.py` 覆盖六/九轴、温控、SPI 读错恢复、PID、底盘、自动流程、CRC 与通信；其中滤波回归含 15/30/50/100 Hz、丢帧及离群场景。新增里程计及定距反馈行为也通过该入口复查，具体行为与验证边界见 [里程计说明](docs/ODOMETRY.md)。磁标定另运行 `python tests/test_mag_calibration.py`，需要 NumPy/SciPy。主机回归和 ARM 编译不代替实车运动、温控及传感器时序验证。

2026-11-16 按整车1.8 kg完成0.25 m/s前后及0.5 rad/s双向旋转对照。三电机Kt均为0（驱动辨识值），保留Kd `0.20/0.25/0.20`；默认加入平移惯性前馈和两前轮平滑摩擦补偿，纯旋转与停车前馈为零。最终配置两组复测完成，用户确认平顺。实时调参、版本核对和日志分析见 [现场调参工具](manual_test/live_tuning/README.md)；专用测试模式不在正常构建中启用。

## 手动测试

USB 手动固件、USART6 桌面与网页遥控、SWD 联调工具的入口见 [手动测试说明](manual_test/README.md)。
