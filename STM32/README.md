# STM32 C 板固件

`project/` 是 STM32F407 CubeMX/CMake 工程，`tests/` 包含主机回归，`docs/` 包含硬件配置和 Jetson CAN 联调说明，`LICENSES/` 保留第三方许可。

在具备 ARM GNU 工具链、CMake 和 Ninja 的环境中，进入 `project/` 后运行 `cmake --preset Debug` 和 `cmake --build --preset Debug`。主机测试在本目录运行 `python tests/run_native.py`；协议互验运行 `python tests/check_jetson_codec.py --jetson-root ../Jetson`。

默认使用六轴姿态且自动搬运关闭。启用自动模式前需按 [联调说明](docs/JETSON_CAN_TEST.md) 完成舵机、运动参数和板端验证。
