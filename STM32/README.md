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
- `JETSON_AUTO_RETURN_SPEED_MPS`、`JETSON_AUTO_ALIGN_SPEED_MPS` 用于返航和安全区视觉对准。

目标速度不得超过对应上限；换算和加减速在底盘控制层执行。类型 3、4 是中心帧，须与 Jetson 同时部署。MissionStatus 过期时 bridge 停发周期流并撤销业务准入，因此独立运行 bridge 也需要配套 Mission 节点。板间类型 1 已删除，夹爪完成通过类型 9 回执。`gripper.move_s` 仍是待标定空值，不要填数字。

自动流程不增加心跳，复用合法类型 3/4 的接收时效：USB 离线、会话改变或周期流断开达到 250 ms 后触发制动，确认停稳或制动达到 500 ms 后请求失能；故障锁存到显式重新初始化，恢复连接不会自动续跑。启动前须先收到当前 USB 会话的周期帧。正常丢目标仍发送 `valid=0` ，底盘停车等待，夹爪、转向和等待期间也持续发送。250 ms 是断流判定期限，实际停车仍受控制节拍和机械减速影响；Mission 掉线还先经过 Jetson 的 500 ms 状态有效期。

自动靠近使用 [Algorithm/Vision](project/Core/Algorithm/Vision/README.md) 的自适应图像坐标滤波；参数和实际跟踪效果仍需实车验证与标定。

自动搬运首次从左侧起点出发，先原地转 180°。夹取和 ROI 通过后按里程计返回起点；异侧才使用类型 4 横向对准。盲推进退使用锁航向的限时速度。退出并确认送达后，车在本次目的区视觉复核并保存下一趟起点。单次底盘联调仍使用 `JETSON_AUTO_SAFETY_STRAIGHT_M`。详见 [动作与握手](docs/JETSON_USB_TEST.md#机械动作与握手) 和 [里程计与定距控制](docs/ODOMETRY.md)。

2026-11-17 里程计采用纯 C 模块 `chassis_odometry`，沿用现有三轮几何、轮序、方向、齿比和航向 PID。参考 ros2_controllers 的平面圆弧积分思路，正式入口使用三轮位置增量、平面 ESKF 航向和 SE(2) 积分，不依赖 ROS/Eigen。`Chassis_Get_Odometry` 用于读取估计状态，`Chassis_Reset_Odometry` 仅允许失能或已确认停车时调用。反馈丢失期间不补积分，恢复首组仅重建基线；3 mm 到位容差是软件判据，轮滑和航向漂移仍需实车测量。

2026-11-15 算法整理：温控 PID 按加热器实际的 `[0,4500]` 输出范围抗积分饱和；航向 PID 与 Mahony 去掉非必要双精度运算，定距轨迹仍保留原精度。BMI088 连续寄存器读取合并 HAL 调用，线上字节和片选时序不变。USB CRC 使用 16 项表递推，协议不变。离线磁标定改用解析 Jacobian，模型、求解容差和现有标定系数保持。

`tests/run_native.py` 覆盖六/九轴、温控、SPI 读错恢复、PID、底盘、自动流程、CRC 与通信；其中滤波回归含 15/30/50/100 Hz、丢帧及离群场景。新增里程计及定距反馈行为也通过该入口复查，具体行为与验证边界见 [里程计说明](docs/ODOMETRY.md)。磁标定另运行 `python tests/test_mag_calibration.py`，需要 NumPy/SciPy。主机回归和 ARM 编译不代替实车运动、温控及传感器时序验证。

2026-11-16 按整车1.8 kg完成0.25 m/s前后及0.5 rad/s双向旋转对照。三电机Kt均为0（驱动辨识值），保留Kd `0.20/0.25/0.20`；默认加入平移惯性前馈和两后轮平滑摩擦补偿，纯旋转与停车前馈为零。最终配置两组复测完成，用户确认平顺。实时调参、版本核对和日志分析见 [现场调参工具](tools/live_tuning/README.md)；专用测试模式不在正常构建中启用。

## USART6 底盘遥控

在 `STM32/project` 下运行 `cmake --preset Debug-UART6-Manual` 和 `cmake --build --preset Debug-UART6-Manual`，产物在 `build/Debug-UART6-Manual`。这个预设用 USART6 独占控制和状态回传。普通 USB 手动、自动搬运和 SWD 联调继续使用各自的构建模式。

J29 使用 3.3 V USB–TTL，共地。模块 TX 接 PG9/RX，模块 RX 接 PG14/TX。串口是 115200、8N1。电脑安装 Python 的 `pyserial` 后，从仓库根目录运行 `python STM32/tools/uart6_manual.py --port COM7`，将 COM7 换成实际端口。工具使用 Python Tk 窗口。

| 操作 | 条件与行为 |
| --- | --- |
| Enter | 发送显式使能，等三轮停稳、界面显示就绪后再移动。 |
| W/S、A/D、Q/E | 前后、左右、逆/顺时针，松键发零速。默认平移 0.10 m/s、转向 0.50 rad/s。 |
| O | 就绪且里程计有效时保存当前位姿为原点，界面显示原点坐标系中的相对 X、Y 和航向。再次按 O 可重建原点。 |
| R | 停稳且原点有效时沿直线返航，同时恢复原朝向。限速 0.10 m/s、0.50 rad/s，起步距离最多 1.5 m，超时 20 s。停稳后用里程计复核 2 cm/10° 到位窗口。 |
| 空格、退出或窗口失焦 | 请求停车并撤销使能。返航期间同样有效。 |

电脑以 20 Hz 发速度帧或返航保活。250 ms 无有效控制帧进入制动，停稳或制动达到 500 ms 后失能。返航期间里程计失效、触及范围边界或动作超时也会停车。失能后原点清空，重新使能、停稳并按 O 后才能再次返航。返航直线须保持空旷；到位窗口以里程计估计核验。

串口复用 A5/版本 01/类型/长度/载荷/CRC-16 的分帧。类型 2 沿用手动速度格式；USART6 专用帧如下。整数按小端传输。

| 类型 | 载荷 |
| --- | --- |
| 0x40 / 0x42 | 使能、建原点，各带非零 `uint16` 请求号。 |
| 0x43 | 返航。长度 2 只有请求号，限速为 0.10 m/s、0.50 rad/s。长度 6 再带两个 `uint16`，单位 0.001 m/s 和 0.001 rad/s，须大于 0 且不超过 0.50 m/s、2.0 rad/s。 |
| 0x41 / 0x44 | 停车、返航保活，空载荷。 |
| 0x80 | 状态、标志、请求号（uint16）、命令、结果、原因。标志 bit0 为原点已建，bit1 为里程计有效。 |
| 0x81 | 相对 X、Y、航向，三个 `int16`，单位依次为 mm、mm、0.01°。 |

板端首先架空验证方向、松键停车、失焦和拔线制动；再在空旷地面低速平移并转向，核对原点显示和返航实走终点。

2026-11-18 使用 DAP-Link（HID/SWD，480 kHz）烧录并校验串口遥控固件。架空时通过 COM9 验证前后、左右、双向转轮、松键停车和串口断流失能。里程计返航从相对坐标 (17, 20) mm 到 (4, 3) mm；这是车轮架空条件下的编码器闭环结果，实车地面位移和车身回转尚待验证。

### 网页遥控

使用同一份 `Debug-UART6-Manual` 固件。此预设由 USART6 独占底盘指令；Jetson 自动搬运使用单独的 USB 固件模式。关闭占用串口的桌面遥控或 VOFA，在仓库根目录运行：

```powershell
python STM32/tools/uart6_web.py
```

网页默认打开 `http://127.0.0.1:8765`，只在电脑本机提供。选择串口（本机实测为 COM9），连接并使能后可以定时运行，也可以按键遥控。

定时运行先选方向，再设平移或转向速度和秒数。到时发送零速，确认三轮停稳后保持使能。按键遥控用 W/S、A/D、Q/E，或按住页面上的按钮；松开后发零速，三轮停稳后保持使能，再接受新动作。斜向同时按时，合速度缩回设定的平移速度。返航平移和转向在页面上单独填写，经长度 6 的 `0x43` 发给板端。运行和返航都受底盘现有加减速限制。空格或「停车并失能」可中止动作，页面失焦、关闭或通讯断流也会请求停车。按住超过 0.25 秒没有新的按键心跳时，网页改发零速，不撤销使能。

网页和桌面工具共用 `pyserial`。若当前 Python 缺少该包，运行 `python -m pip install pyserial`；本地已经随架空测试保存了依赖副本，可以先设置 `$env:PYTHONPATH = (Resolve-Path 'STM32/project/build/Debug-UART6-Manual/serial-test-deps').Path` 再启动。
