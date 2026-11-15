# STM32 C 板固件

`project/` 是 STM32F407 CubeMX/CMake 工程，`tests/` 包含主机回归，`docs/` 包含硬件配置、板间 USB 和电机 CAN1 联调说明，`LICENSES/` 保留第三方许可。

在具备 ARM GNU 工具链、CMake 和 Ninja 的环境中，进入 `project/` 后运行 `cmake --preset Debug` 和 `cmake --build --preset Debug`。主机测试在本目录运行 `python tests/run_native.py`；协议互验运行 `python tests/check_jetson_codec.py --jetson-root ../Jetson`。

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

目标速度不得超过对应上限；换算和加减速在底盘控制层执行。类型 3、4 是中心帧，须与 Jetson 同时部署。Mission 心跳缺失时 bridge 停止运动保活，因此独立运行 bridge 也需要配套 Mission 节点。`gripper.move_s` 仍是待标定空值，不要填数字。

自动靠近使用 [Algorithm/Kalman](project/Core/Algorithm/Kalman/README.md) 的自适应图像坐标滤波；参数和实际跟踪效果仍需实车验证与标定。
