# GC Rescue

- `STM32/`：C 型开发板固件、主机测试和硬件说明。
- `Jetson/`：视觉、任务和 CAN 桥接 ROS 2 工作区。

构建与硬件条件分别见 [STM32 说明](STM32/README.md)和 [Jetson 说明](Jetson/docs/README.md)。模型和板端 SDK 需按实际设备准备；频率及自动搬运参数仍须板端验证和标定。

CAN 协议、配置、验收步骤与离线测试见 [CAN 联调指南](STM32/docs/JETSON_CAN_TEST.md)，旧版设备结果见[历史联调记录](STM32/docs/JETSON_CAN_HISTORY.md)。底盘速度上限在 `STM32/project/Core/Control/Chassis/chassis.h` 配置，自动搬运速度和视觉对准参数在 `STM32/project/Core/Control/Jetson/jetson_auto_config.h` 配置。

本分支尚未进行真实设备验证。自动搬运默认关闭，机械与舵机参数须实测标定；部署时应同步匹配 Jetson 与 STM32 协议版本，并按联调指南验证 CAN 负载、反馈、方向和停车。
