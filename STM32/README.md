# STM32 C 板固件

`project/` 是 STM32F407 CubeMX/CMake 工程，`tests/` 包含主机回归，`docs/` 包含硬件配置、板间 USB 和电机 CAN1 联调说明，`LICENSES/` 保留第三方许可。

在具备 ARM GNU 工具链、CMake 和 Ninja 的环境中，进入 `project/` 后运行 `cmake --preset Debug` 和 `cmake --build --preset Debug`。主机测试在本目录运行 `python tests/run_native.py`；协议互验运行 `python tests/check_jetson_codec.py --jetson-root ../Jetson`；夹爪总时间预算互验运行 `python tests/check_gripper_budget.py`（使用主机 g++）。

默认使用六轴姿态且自动搬运关闭。启用自动模式前需按 [联调说明](docs/JETSON_USB_TEST.md) 完成 Jetson 夹爪联调、运动参数标定和板端验证。

与参考项目 `C_project` 的主要功能差异如下；参考项目仅用于对比。

| 功能 | `C_project` | 当前 `STM32` |
|---|---|---|
| 板间通信 | Jetson 与电机共用 CAN1 | USB 传输板间帧，CAN1 只连接电机 |
| 手动控制 | Jetson 速度帧联调 | 独立类型 2 手动控制，保留零速准备、停稳及断流保护 |
| 夹爪 | STM32 的 TIM1 舵机驱动 | Jetson 执行夹爪动作，STM32 发送到点请求并等待放行；旧舵机业务已移除 |
| 视觉靠近 | 固定参数图像跟踪滤波 | 自适应卡尔曼滤波及近窗修正，仍需实车标定 |
| 底盘保护 | 各控制模块检查反馈 | 共用电机就绪检查，邮箱忙时跳拍，姿态异常先制动，斜坡步长最多 50 ms |
| 转向上限 | 车体 1.0 rad/s，纯转各轮约 3.09 rad/s | 车体 2.0 rad/s，纯转各轮约 6.17 rad/s |
| 姿态与默认模式 | 六轴默认、九轴可选，自动关闭 | 保留相同传感器、温控、VOFA 和默认模式，自动参数完成标定后再启用 |

此次整理精简电机单组过滤器配置，移除 CAN BSP 中的 USB 上行包装，由自动流程直接编码并调用 USB；顶层 CMake 只启用 C/ASM 并删除空配置，同时修正遗留 CAN/舵机说明。CAN2、TIM1 的 CubeMX 初始化配置仍保留，当前没有对应业务启动。

进一步删除 BMI088/IST8310 的四个空初始化接口、BMI088 未使用的 `status/time` 字段和 PID 增量式遗留历史缓存，以及无引用的测试桩和宏。传感器外设仍由 CubeMX 初始化；手动、自动、六轴/九轴、温控和保护逻辑保留，USB 调试记录与旧滤波对照测试保留。

速度由 project/Core/Control/Chassis/chassis.h 中的上限宏和 project/Core/Control/Jetson/jetson_auto_config.h 中的自动动作速度宏配置。默认车体平移上限和盲推目标为 0.50 m/s，车体转向上限为 2.0 rad/s；近窗最低修正速度为 0.005 m/s，需按实车标定。板间速度保持 m/s，三轮运动学在底盘层换算为轮速 rad/s。电机在 CAN1，板间帧不走 CAN。

速度配置保留在现有头文件中，修改后需要重新编译：

- `CHASSIS_STRAIGHT_MAX_SPEED_MPS`、`CHASSIS_DISTANCE_MAX_SPEED_MPS`：平移与定距速度上限。
- `CHASSIS_VECTOR_MAX_WZ_RAD_S`、`CHASSIS_TURN_OUTPUT_LIMIT`：连续转向与定角转向的车体角速度上限。
- `JETSON_AUTO_APPROACH_SPEED_MPS`、`JETSON_AUTO_DISTANCE_SPEED_MPS`、`JETSON_AUTO_PUSH_SPEED_MPS`：视觉靠近、定距及盲推/后退的目标速度。
- `JETSON_AUTO_SEARCH_WZ_RAD_S`、`JETSON_AUTO_APPROACH_MIN_SPEED_MPS`：搜索角速度与窗外最低修正速度。

目标速度不得超过对应上限；换算和加减速在底盘控制层执行。类型 3、4 是中心帧，须与 Jetson 同时部署。MissionStatus 过期时 bridge 停发周期流并撤销业务准入，因此独立运行 bridge 也需要配套 Mission 节点。板间类型 1 已删除，夹爪完成通过类型 9 回执。`gripper.move_s` 仍是待标定空值，不要填数字。

自动流程不增加心跳，复用合法类型 3/4 的接收时效：USB 离线、会话改变或周期流断开达到 250 ms 后触发制动，确认停稳或制动达到 500 ms 后请求失能；故障锁存到显式重新初始化，恢复连接不会自动续跑。启动前须先收到当前 USB 会话的周期帧。正常丢目标仍发送 `valid=0` 并继续搜索，夹爪、转向和等待期间也持续发送。250 ms 是断流判定期限，实际停车仍受控制节拍和机械减速影响；Mission 掉线还先经过 Jetson 的 500 ms 状态有效期。

自动靠近使用 [Algorithm/Kalman](project/Core/Algorithm/Kalman/README.md) 的自适应图像坐标滤波；参数和实际跟踪效果仍需实车验证与标定。

安全区导航先按现有类型 4 的窗口靠近并对准中心，停稳核验后按 `JETSON_AUTO_SAFETY_STRAIGHT_M` 锁航向直行；此后不再使用识别结果，定距完成且三轮停稳才发送类型 18。该距离以米配置，按要求留零待实车标定；单趟底盘测试也需要填写。定距通过三轮反馈正解与 IMU 航向估计位移，按本段目标方向的有符号投影闭环控制；仅有效里程计状态允许起步。反馈失效、无进展或总时限到期均进入故障制动，不能当作正常完成。通信断流、USB 会话变化及底盘故障仍沿用已有处理。详见 [里程计与定距控制](docs/ODOMETRY.md)。

2026-11-17 里程计采用纯 C 模块 `chassis_odometry`，沿用现有三轮几何、轮序、方向、齿比和航向 PID。参考 ros2_controllers 的平面圆弧积分思路，平移使用相邻有效轮速的梯形积分，航向使用 IMU，不依赖 ROS/Eigen。`Chassis_Get_Odometry` 用于读取估计状态，`Chassis_Reset_Odometry` 仅允许失能或已确认停车时调用。反馈丢失期间不补积分，恢复首组仅重建基线；3 mm 到位容差是软件判据，轮滑和航向漂移仍需实车测量。

2026-11-15 算法整理：温控 PID 按加热器实际的 `[0,4500]` 输出范围抗积分饱和；航向 PID 与 Mahony 去掉非必要双精度运算，定距轨迹仍保留原精度。BMI088 连续寄存器读取合并 HAL 调用，线上字节和片选时序不变。USB CRC 使用 16 项表递推，协议不变。离线磁标定改用解析 Jacobian，模型、求解容差和现有标定系数保持。

`tests/run_native.py` 覆盖六/九轴、温控、SPI 读错恢复、PID、底盘、自动流程、CRC 与通信；其中滤波回归含 15/30/50/100 Hz、丢帧及离群场景。新增里程计及定距反馈行为也通过该入口复查，具体行为与验证边界见 [里程计说明](docs/ODOMETRY.md)。磁标定另运行 `python tests/test_mag_calibration.py`，需要 NumPy/SciPy。主机回归和 ARM 编译不代替实车运动、温控及传感器时序验证。

2026-11-16 按整车1.8 kg完成0.25 m/s前后及0.5 rad/s双向旋转对照。三电机Kt均为0（驱动辨识值），保留Kd `0.20/0.25/0.20`；默认加入平移惯性前馈和两后轮平滑摩擦补偿，纯旋转与停车前馈为零。最终配置两组复测完成，用户确认平顺。实时调参、版本核对和日志分析见 [现场调参工具](tools/live_tuning/README.md)；专用测试模式不在正常构建中启用。
