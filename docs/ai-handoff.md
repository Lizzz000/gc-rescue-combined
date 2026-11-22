# 自动搬运交接

更新于 2026-11-22。开工先看 [AGENTS.md](../AGENTS.md) 和 [项目说明](../README.md)，核对当前工作区、已烧录固件和实际板端连接。调试与正式自动搬运使用不同 STM32 预设。

## 正式配置与启动

- STM32 使用 `Debug-Auto`。底盘参数在 [chassis_config.h](../STM32/project/Core/Control/Chassis/chassis_config.h)，视觉、整区窗口、投送和回撤时序在 [jetson_auto_config.h](../STM32/project/Core/Control/Jetson/jetson_auto_config.h)。
- Jetson 生产环境为 ROS 2 Humble，使用 [field_20261122.yaml](../Jetson/src/bringup_pkg/config/field_20261122.yaml)。相机控件在启动检测前写入并回读，图传默认关。STM32 OTG 与 S288 各有独立的 USB 设备，使用 `/dev/serial/by-id/` 路径。
- 同一工作树配套构建并更新两端。现场接线、单绿摆位、启动命令和记录要求见 [下一次现场单绿验收](../STM32/docs/JETSON_USB_TEST.md#下一次现场单绿验收)。

## 运行顺序

1. 开爪成功后请求整区模式，自动对准整区中心。停稳后两张新源图入窗，保存里程计起点。原地转 180°，停稳后再切到目标识别。
2. 近处先收横向误差，再进入夹取窗。停稳后发送夹取请求，桥锁存新鲜目标类别。夹取成功后直接发 `16/02` 登记目的区；首趟绿色送左区，后续绿色、黑色送左区，橙色送右区。授权编号与本次夹取编号一致才返航。
3. 使用里程计回到保存的起点和首次安全区朝向，然后向目的侧横移 0.30 m/s × 0.70 s。停稳后盲推 0.40 m/s × 1.20 s，松爪后回退 0.40 m/s × 1.00 s。
4. 按实测横移量反平移，限速 0.225 m/s；停稳后用整区精调再建点。复定位的位移上限 0.25 m，阶段共用 10 s。整区精调先停稳再依据新图补正。记录新起点后再转 180°，切回搜索。

正式流程不检查 ROI、不执行检查退距或 ROI 拒收恢复。独立底盘诊断仍通过 `16/01` 验证旧到达接口。通信故障进入制动失能，重连后需重新初始化。完整编号、期限和重复请求规则见 [USB 协议](../Jetson/docs/usb-protocol.md)。

## 常用入口

| 范围 | 入口 |
| --- | --- |
| STM32 自动状态机、快照与里程计 | [jetson_auto.c](../STM32/project/Core/Control/Jetson/jetson_auto.c)、[chassis_odometry.c](../STM32/project/Core/Control/Chassis/chassis_odometry.c)、[主机回归](../STM32/tests/run_native.py) |
| Jetson 配置、任务与串口 | [Jetson 运行说明](../Jetson/docs/README.md)、[Mission 控制](../Jetson/src/mission_pkg/control/controller.cpp)、[Bridge 事务](../Jetson/src/bridge_pkg/state/bridge_transactions.cpp) |
| 整区识别与标定 | [安全区离线回放](../Jetson/src/vision_pkg/test/field/README.md)、[里程计和现场实测](../STM32/docs/ODOMETRY.md) |
| 当前离线验证 | [11 月 22 日主机、ARM 与 ROS 回归](validation-2026-11-22.md) |
| 历史验证 | [11 月 15 日验证](validation-2026-11-15.md)、[11 月 17 日优化](code-optimization-2026-11-17.md)、[11 月 19 日优化](optimization-changes-2026-11-19.md) |

11 月 22 日的现场记录已确认初次对准、记点、转身与夹取；旧流程的检查退距曾受机械死区影响。最终无 ROI 版本仍要在两板接通后验证带货返航、实际放货、回撤到重新建点的耗时。热点失联需单独复验。
