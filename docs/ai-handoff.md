# 后续 AI 交接

更新于 2026-11-17。先读根目录的 [AGENTS.md](../AGENTS.md) 和[项目说明](../README.md)，再根据本轮任务核对 Git 状态和实际板端连接。

上一轮在云端 `62086f9` 基础上优化安全区内沿扫描与 STM32 USB 接收缓冲区；变更范围、主机检查和板端验证边界见 [代码优化与验证](code-optimization-2026-11-17.md)。本轮在该基础上接入 STM32 轮速反馈与 IMU 航向里程计，八向定距改为实测位移投影闭环，见 [里程计与定距控制](../STM32/docs/ODOMETRY.md)。历史验证文档中的指令积分描述仅对应当时版本。

## 工程入口

- `Jetson/src/vision_pkg/field` 负责安全区识别。候选搜索和跟踪在 [field.cpp](../Jetson/src/vision_pkg/field/field.cpp)，边线、内沿与投影在 [geometry.cpp](../Jetson/src/vision_pkg/field/opencv/geometry.cpp)，黑带测量在 [score.cpp](../Jetson/src/vision_pkg/field/opencv/score.cpp)。正式观测由 [ros_convert.hpp](../Jetson/src/vision_pkg/node/ros_convert.hpp) 检查后发布。
- 常规启动读取 [rescue.yaml](../Jetson/src/bringup_pkg/config/rescue.yaml) 和 [rescue_advanced.yaml](../Jetson/src/bringup_pkg/config/rescue_advanced.yaml)。标准场地外框 660 × 360 mm，内框 600 × 300 mm，平面内缩每侧 30 mm。紫围栏高 20 mm，黑带长 300 mm、宽 20 mm、高 20 mm。11 月 16 日实测的 660 × 640 mm 场地使用当时的[独立配置](../Jetson/outputs/field_trial_20261116/FIELD_PARAMETERS.txt)。
- 板间通信使用 USB CDC。STM32 通过 CAN1 控制电机。载荷和时序以 [USB 协议](../Jetson/docs/usb-protocol.md) 为准，实际双端联调从对应日期的[现场记录](../Jetson/outputs/field_trial_20261116/COMMUNICATION_CHECK_END.txt)接续。
- 里程计纯计算在 `STM32/project/Core/Algorithm/Chassis/chassis_odometry.*`，反馈快照、定距闭环与故障停车在 [chassis.c](../STM32/project/Core/Control/Chassis/chassis.c)。本轮未烧录或实车运行；自动模式仍关闭，自动距离仍为 0，Jetson `gripper.move_s` 仍留空。先完成轮径、方向、IMU 航向和实际定距复核再启用自动搬运。
- 本轮里程计主机回归 27 套、双端协议互验和四种 ARM 配置编译通过，含慢跟随、堵转、到位后滑行与 CAN 拥塞反例；详细结果见[里程计验证](../STM32/docs/ODOMETRY.md#2026-11-17-主机验证结果)。这些结果不代替板端里程和停车效果验收。

## 安全区当前行为和待验证项

- C++ 运行路径依赖本帧证据。两端底缘可见时可补测近边断口，侧面缺紫色时可用内部地面色差补测。单角出画且边线可测时，诊断会给出估计的画外地面面积。画面边缘直接裁断地面且缺少完整几何证据时，该帧的正式分区无效。
- [八方向原型](../Jetson/src/vision_pkg/test/field/simulate_translation.py)只用于离线模拟，以完整场地图、已知相机标定和固定朝向为前提。约 8% 出画时 8/8，约 20% 出画时 6/8。后左、后右在较大裁切下被拒绝。接入前要用实际相机内参与安装姿态回放，重点检查平移与转向叠加、黑带遮挡和两个后斜方向。
- 2026-11-17 在 Windows / OpenCV 4.13.0 重编安全区测试。紫框缺色专项 74 项通过。40 mm 围栏压力矩阵有 4 项未达到原有效率门槛，黑带专项有 2 项运动模糊误差超限，几何专项有 1 项困难光照有效率不足。实拍回放缺少标注数据。逐方向真值、误差和复现方式见[安全区测试说明](../Jetson/src/vision_pkg/test/field/README.md)。

## 本机复查

从仓库根目录运行：

```bash
python3 Jetson/tools/source/check_source.py
cmake -S Jetson/src/vision_pkg/test/field -B /tmp/gc-rescue-field
cmake --build /tmp/gc-rescue-field
ctest --test-dir /tmp/gc-rescue-field --output-on-failure
python3 Jetson/src/vision_pkg/test/field/simulate_translation.py --missing-fraction 0.20
```

C++ 测试依赖 OpenCV、C++17 和 PyYAML，离线模拟依赖 Python OpenCV 和 NumPy。交接时更新测试结果、仍待验证的场景和相应配置，现场结论以真实采样或板端回放为准。
