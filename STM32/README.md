# STM32 C 板固件

`project/` 是 STM32F407 CubeMX/CMake 工程，`tests/` 包含主机回归，`docs/` 包含硬件配置、板间 USB 和电机 CAN1 联调说明，`LICENSES/` 保留第三方许可。

在具备 ARM GNU 工具链、CMake 和 Ninja 的环境中，进入 `project/` 后运行 `cmake --preset Debug` 和 `cmake --build --preset Debug`。主机测试在本目录运行 `python tests/run_native.py`；协议互验运行 `python tests/check_jetson_codec.py --jetson-root ../Jetson`。

默认使用六轴姿态且自动搬运关闭。启用自动模式前需按 [联调说明](docs/JETSON_USB_TEST.md) 完成舵机、运动参数和板端验证。

速度由 project/Core/Control/Chassis/chassis.h 中的上限宏和 project/Core/Control/Jetson/jetson_auto_config.h 中的自动动作速度宏配置。默认车体平移上限和盲推目标为 0.50 m/s，车体转向上限为 2.0 rad/s；近窗最低修正速度为 0.005 m/s，需按实车标定。板间速度保持 m/s，三轮运动学在底盘层换算为轮速 rad/s。电机在 CAN1，板间帧不走 CAN。

速度配置保留在现有头文件中，修改后需要重新编译：

- `CHASSIS_STRAIGHT_MAX_SPEED_MPS`、`CHASSIS_DISTANCE_MAX_SPEED_MPS`：平移与定距速度上限。
- `CHASSIS_VECTOR_MAX_WZ_RAD_S`、`CHASSIS_TURN_OUTPUT_LIMIT`：连续转向与定角转向的车体角速度上限。
- `JETSON_AUTO_APPROACH_SPEED_MPS`、`JETSON_AUTO_DISTANCE_SPEED_MPS`、`JETSON_AUTO_PUSH_SPEED_MPS`：视觉靠近、定距及盲推/后退的目标速度。
- `JETSON_AUTO_SEARCH_WZ_RAD_S`、`JETSON_AUTO_APPROACH_MIN_SPEED_MPS`：搜索角速度与窗外最低修正速度。

目标速度不得超过对应上限；换算和加减速在底盘控制层执行。类型 3、4 是中心帧，须与 Jetson 同时部署。Mission 心跳缺失时 bridge 停止运动保活，因此独立运行 bridge 也需要配套 Mission 节点。`gripper.move_s` 仍是待标定空值，不要填数字。

自动靠近使用 [Algorithm/Kalman](project/Core/Algorithm/Kalman/README.md) 的自适应图像坐标滤波；参数和实际跟踪效果仍需实车验证与标定。
