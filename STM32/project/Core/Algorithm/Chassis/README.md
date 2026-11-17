# 三轮里程计的来源与接入

当前正式底盘在 `Control/Chassis/chassis.c` 调用 `Odometry_Update_Position()`：三轮编码器角位置增量提供平移，已有 INS 提供航向，使用 SE(2) 指数映射累计平面位姿。实现为纯 C，无动态内存、ROS 或 Eigen 依赖。

## 开源实现与论文选择

- **WPILib v2026.2.2**，固定提交 [`7ca35e5678cf32caec6a1a866ca51d0136c4c398`](https://github.com/wpilibsuite/allwpilib/tree/7ca35e5678cf32caec6a1a866ca51d0136c4c398)：参考 [`Odometry::Update`](https://github.com/wpilibsuite/allwpilib/blob/7ca35e5678cf32caec6a1a866ca51d0136c4c398/wpimath/src/main/native/include/frc/kinematics/Odometry.h) 的“轮位置差求车体增量，IMU替代轮转角”，以及 [`Pose2d::Exp`](https://github.com/wpilibsuite/allwpilib/blob/7ca35e5678cf32caec6a1a866ca51d0136c4c398/wpimath/src/main/native/include/frc/geometry/Pose2d.h) 的平面刚体积分，按本车接口适配。BSD 许可原文保留于 [LICENSE-WPILib.txt](LICENSE-WPILib.txt)。没有把四轮或差速车的运动学套用到本车。
- **ROS 2 omni_wheel_drive_controller**，固定提交 [`62af79e4788adaf4718491968c3f8fcc632af265`](https://github.com/ros-controls/ros2_controllers/blob/62af79e4788adaf4718491968c3f8fcc632af265/omni_wheel_drive_controller/src/odometry.cpp)：该实现也提供位置差与速度入口。此前轮速积分按其圆弧公式独立实现；保留 `Odometry_Update()` 供对照测试，正式控制不会自动切回轮速积分。参考源采用 Apache-2.0，未移入上游 ROS/Eigen 代码。
- SciSpace 检索的 [三轮全向里程计系统误差标定研究](https://scispace.com/papers/systematic-odometry-error-evaluation-and-correction-in-a-29u40hfa)（[2022 原论文](https://www.mdpi.com/2076-3417/12/5/2606)）说明需要以独立轨迹观测拟合几何参数。本车尚未尺量或获得视觉真值，轮半径35mm、中心距108mm继续使用现有值，不写入未经标定的修正系数。
- [轮编码器、MEMS陀螺仪与单目视觉紧耦合研究](https://scispace.com/papers/tightly-coupled-monocular-visual-odometric-slam-using-wheels-1g432zoxxr)（[作者预印本](https://arxiv.org/abs/1804.04854)）作为后续视觉融合参考；当前没有移植优化器、实现视觉SLAM或宣称在线漂移校正。

## 本车约定

CAN ID1=后轮，ID2=右前轮，ID3=左前轮；x前、y左、航向俯视逆时针为正。控制层把电机安装方向 −1 和齿比1换算后，同时传入位置及速度；`motor_chassis.c` 负责统一正逆解，算法层不重复反向。

本车MIT位置反馈在±12.5rad间回绕，周期为25rad，并非2π。位置入口解绕相邻样本角差，再通过三轮正解得到车体 `dx/dy`；以IMU相邻航向差进行SE(2)积分。小角处使用展开式，世界x/y累计采用double。速度入口与位置入口切换时只重建基线，不跨来源积分。

位置入口只接受有限且处于协议范围内的样本；超过50ms、非法位置/速度及不可实现的角跳变使估计失效并保留最后位姿。跳变上界使用40rad/s、采样间隔、控制层30ms新鲜度窗口和两个编码器量化码的余量。控制层仍负责三轮新序号、使能状态、同一临界区快照及IMU新鲜度检查；三轮并非硬件同步。

`vx_mps/vy_mps/wheel_wz_rad_s` 继续表示同组速度反馈正解，仅供诊断；即使它们有停车尾迹，静止编码器也不会因此累加x/y。位置增量不能消除打滑、轮径误差、编码器本身误差或六轴航向漂移，实际地面精度仍须外部测量。

## 验证入口

`python STM32/tests/run_native.py` 覆盖独立接地点几何、八向、多次双向25rad回绕、计时回绕、IMU主导圆弧、断流恢复、错误跳变、输入源切换和真实控制链。`test_simulate_return.py` 编译真实C位置入口；其电机角位置独立于速度反馈偏差生成，模型真值仅作评估，不供导航使用。

板端固件版本、实测结果和未完成项目记录于 [STM32/docs/ODOMETRY.md](../../../../docs/ODOMETRY.md)，不能用离线模拟结果代替实车精度验证。
