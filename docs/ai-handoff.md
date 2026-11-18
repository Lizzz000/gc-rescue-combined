# 后续 AI 交接

更新于 2026-11-18。先读根目录的 [AGENTS.md](../AGENTS.md) 和[项目说明](../README.md)，再根据本轮任务核对 Git 状态和实际板端连接。

## 11 月 18 日运行速度、上电使能与视觉接入

COM7 手动固件使用 `Debug-UART6-Manual`，Jetson 自动搬运固件使用 `Debug-Auto`。两者采用 0.40 m/s 平移、3.0 rad/s 转向的正常目标。手动连续速度可显式提高到 0.50 m/s、4.0 rad/s，起停和换向按原斜坡运行。安全区精对准使用 0.10 m/s。上电取得有效姿态后申请使能，三轮提交零目标并通过反馈检查后就绪。普通松键、换向、自动等待及动作衔接保持使能。COM7 在底盘接下零速目标后即可归还控制，余下减速由板端完成。急停、已接管时断流、姿态或驱动故障进入保护。[操作与协议](../STM32/docs/JETSON_USB_TEST.md) · [COM7 操控](../STM32/manual_test/README.md#usart6-底盘遥控)

自动流程通过 `Visual_Observation_t` 接收物资或安全区中心、有效位、源帧时刻和板端接收信息。新视觉源沿用类型 3/4，重复或乱序源帧不能推进到位确认。当前场地限位仍待标定，自动版上电后可以零速待命。开始搬运还需场地参数、视觉周期流及匹配的开爪回执。[视觉协议](../Jetson/docs/usb-protocol.md) · [自动标定参数](../STM32/project/Core/Control/Jetson/jetson_auto_config.h)

- **主机验证**　七种 ARM 配置构建成功，最终 ELF 符号检查确认手动和自动各自只链接对应的控制循环。通过 37 套 STM32 原生回归、17 种模式组合、11 个算法独立编链、115 项 STM32 Python 测试、53 项 Jetson 手动端 Python 测试，以及 COM7 和 Jetson 两个网页的交互回归。双端协议互验覆盖下行 155 帧、上行 33 帧。
- **离线对照**　用同一组 10 ms 输入运行新旧底盘代码。在 0.40 m/s 前进、直接反向、正常松键序列中，右前轮起步首拍目标变化从 0.111 降到 0.022 rad/s，松键首拍从 9.897 降到 0.037 rad/s，新指令在 0.80 s 后归零。手动断流回放中，末帧位于第 100 ms，控制器在第 350 ms 进入制动。随后恢复的有效零帧未触发使能，显式撤权后再发送新零帧才恢复。
- **待验证**　此次交付时 COM7 和 DAPLink 均未连接。新版未烧录，上电零速使能、地面实际轮速、振动及停车距离待板端验证。当前 Windows 主机未运行 Jetson 的 Linux 伪串口进程测试。

## 11 月 18 日里程计对时与 STM32 模块拆分

三轮编码器按 CAN 接收时刻插值到共同时间，INS 航向按源帧时间对齐。平移积分使用轮区间起止航向，对外报告的航向对应控制拍。手动、自动分别编入各自的控制器和 USB 协议适配，公共底盘接口只公开位姿和动作。轮角对时、运动学、航向 ESKF、航向跳点检查、SE(2) 积分、视觉跟踪和视觉速度规划都可独立链接。常规主机回归也会执行模块隔离检查。[里程计实现说明](../STM32/docs/ODOMETRY.md) · [构建与模式](../STM32/docs/JETSON_USB_TEST.md) · [模块隔离检查](../STM32/tests/check_module_boundaries.py)

- 主机通过 35 套原生回归、17 种模式组合和 10 个算法模块隔离构建。手动、完整自动搬运和自动单趟底盘测试分别只使用本侧控制器运行。六种 ARM 固件配置编译通过。
- 离线对照采用带编码器量化的 10 秒解析轨迹。异步采样下，原地往复转向的假平移峰值由 2.8461 mm 降至 0.0104 mm，变速二维平移的位置误差峰值由 1.1149 mm 降至 0.0138 mm。另一组 6 秒变速换向圆弧同时加入轮反馈错相和 IMU 延迟，最大位置误差由 3.093 mm 降至 0.302 mm。位置按轮反馈共同时间对照，航向按控制拍对照。
- 已用这版里程计完成地面 35 圈测试。五档 2、3、4、3、2 rad/s 各 7 圈，板端用时 86.30 s，航向计 35.00325 圈，编码器计 35.04873 圈，里程计终点平移估计 11.34 mm。首次预检定位到 USB 在 FreeRTOS 首项任务创建之后初始化导致 HAL 延时卡住，修正后完成实测。已恢复测试前的 USART6 手动固件，COM7 网页遥控已连接并失能。真实地面转角与位移尚待外部测量。[分档实测与固件记录](../STM32/docs/ODOMETRY.md#2026-11-18-地面原地旋转-35-圈)

## 11 月 18 日新推送与底盘换段

已合并远端 `be2aa1a` 和本地 `117ef0e`。自动搬运、USB 手动和 USART6 遥控在三轮零目标提交、使能反馈和姿态有效后可接续运动。视觉入窗后，物资用一张新源帧复核，安全区用连续两张新源帧复核。夹爪仍须返回匹配的成功结果。手动建原点、调参静态验收和故障制动继续检查实际停转。[动作与握手](../STM32/docs/JETSON_USB_TEST.md#机械动作与握手)

修复了同侧投送时 ROS 消息乱序导致的安全区坐标闭锁、左右视觉窗口无法逐项覆盖，以及 USART6 跨间隔拼接半帧和桌面遥控读取缓存后错过状态超时的问题。

- 主机上通过了 STM32 原生回归 34 套、控制模式组合 17 种、Python 110 项，以及 Jetson Bridge 652 项、Mission 500 项。六种 ARM 固件配置编译通过，双端 USB 编解码及夹爪时间预算互验通过。
- 离线返航模拟通过，理想和带打滑扰动的模拟真值终点误差分别为 1.73 mm 和 5.14 mm。
- 板端已通过 DAPLink/SWD 烧录并校验 `41d6a91` 的 `Debug-UART6-Manual` 固件。三轮架空，用 COM7 以 0.08 m/s 做了两次前进、零速、反向测试。第二次运行中，板端在 tick 599345 已受理反向速度，停稳计数为 0，ID2/ID3 最近的反馈为 −2.10 / +2.02 rad/s。随后采到反向目标，至 tick 599371 两轮反馈为 +0.36 / −0.36 rad/s。两次 STOP 后均收到三台电机失能反馈。地面负载下的滑行距离、夹取和 Jetson 自动搬运仍待实测。

## 11 月 18 日先前代码审阅

以 `5971dd8` 为基线审阅 Jetson 各包、STM32 固件、主机工具及测试。HAL、CMSIS 和 FreeRTOS 按本项目实际使用的接口核查。

- **USB 和底盘**。STM32 启动时配置 48 MHz USB 时钟，删除 USB 初始化中切换系统时钟和重启 PLL 的补救流程。时钟校验或 USB 控制器启动失败时，接收任务和中断均不启动。姿态解算跳过异常采样时距时撤销底盘快照，下一次有效更新后恢复。六轴和九轴主机测试均覆盖 70 ms 采样停顿。[时钟配置](../STM32/docs/DJI_C_CUBEMX.md)
- **视觉和通信**。安全区搜索复用候选的准入结果，Mission 对外框和两个分区都检查消息约定的 `z=0`。CDC 直接从接收缓冲解析同批多帧，Bridge 共用中心缓存清理逻辑。OC-SORT 连续命中时少复制一次观测，遮挡回放仍保留两端观测。
- **配置和调试**。自动和手动程序都拒绝点号键与嵌套键指向同一参数。自动配置拒绝无穷大的夹爪增益。视觉和旋转调试各读取一次 ELF 符号表。串口按 56 或 64 通道的完整帧宽解码，拒绝超长帧中的合法长度后缀。主机收到显式 STOP 的完成确认后结束命令，丢失确认时仍执行停止清理。
- **主机验证**。STM32 原生回归 33 套、控制开关组合 9 种、Python 测试 101 项通过。五种 ARM 配置编译通过，CubeMX 从 `.ioc` 重新生成的 PLLQ 为 7。双端协议下行 155 帧、上行 33 帧互验通过。Jetson Bridge 540 项、Mission 493 项、分割 CPU 后处理 20,794 项及 NVMM 主机测试桩 58 项通过。
- **离线模拟和回放**。安全区 210 帧逐帧对照一致，完整压力与专项测试合计 3419 项通过、7 项失败，与既有记录一致。OC-SORT 的 2048 帧遮挡序列状态对照一致。理想和打滑扰动两种返航模拟均完成。两段历史串口记录共解出 2459 个完整帧，按不同块长回放结果相同。
- **待验证项**。本轮未烧录、未执行板端 USB 枚举、ROS/GPU 或实车搬运。安全区实拍回放缺少标注。参数文档生成器的 `--check` 仍因仓库缺少 `Jetson/docs/reference/parameter-catalog.json` 失败。

11 月 17 日的安全区与 USB 优化、里程计和算法变更见[代码优化与验证](code-optimization-2026-11-17.md)、[里程计与定距控制](../STM32/docs/ODOMETRY.md)和[算法升级记录](../STM32/docs/ALGORITHM_UPGRADE_20261117.md)。历史验证文档中的指令积分描述仅对应当时版本。

## 工程入口

- `Jetson/src/vision_pkg/field` 负责安全区识别。候选搜索和跟踪在 [field.cpp](../Jetson/src/vision_pkg/field/field.cpp)，边线、内沿与投影在 [geometry.cpp](../Jetson/src/vision_pkg/field/opencv/geometry.cpp)，黑带测量在 [score.cpp](../Jetson/src/vision_pkg/field/opencv/score.cpp)。正式观测由 [ros_convert.hpp](../Jetson/src/vision_pkg/node/ros_convert.hpp) 检查后发布。
- 常规启动读取 [rescue.yaml](../Jetson/src/bringup_pkg/config/rescue.yaml) 和 [rescue_advanced.yaml](../Jetson/src/bringup_pkg/config/rescue_advanced.yaml)。标准场地外框 660 × 360 mm，内框 600 × 300 mm，平面内缩每侧 30 mm。紫围栏高 20 mm，黑带长 300 mm、宽 20 mm、高 20 mm。11 月 16 日实测的 660 × 640 mm 场地使用当时的[独立配置](../Jetson/outputs/field_trial_20261116/FIELD_PARAMETERS.txt)。
- 板间通信使用 USB CDC。STM32 通过 CAN1 控制电机。载荷和时序以 [USB 协议](../Jetson/docs/usb-protocol.md) 为准，实际双端联调从对应日期的[现场记录](../Jetson/outputs/field_trial_20261116/COMMUNICATION_CHECK_END.txt)接续。
- 里程计组合入口在 [chassis_odometry.c](../STM32/project/Core/Control/Chassis/chassis_odometry.c)，反馈快照、定距闭环与故障停车在 [chassis.c](../STM32/project/Core/Control/Chassis/chassis.c)。正式配置默认关闭自动模式，自动距离为 0，Jetson `gripper.move_s` 留空。现场标定与复测历史见[里程计记录](../STM32/docs/ODOMETRY.md)。
- 里程计初次接入时，主机回归 27 套、双端协议互验和四种 ARM 配置编译通过，覆盖慢跟随、堵转、到位后滑行与 CAN 拥塞。[当时的验证结果](../STM32/docs/ODOMETRY.md#2026-11-17-主机验证结果)

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
