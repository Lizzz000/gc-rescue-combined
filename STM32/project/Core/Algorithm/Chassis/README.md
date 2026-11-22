# 三轮里程计的来源与接入

正式底盘通过 `Odometry_Update_Feedback()` 更新位姿。控制层负责组合四个可独立链接的算法。轮角对时给出同一时间段的三个编码器增量，三轮运动学把它们换算为车体位移，航向 ESKF 融合陀螺、YAW 和轮角速度，SE(2) 积分器将车体位移累加到世界坐标。手动与自动控制器通过公共底盘接口读取结果。[里程计组合入口](../../Control/Chassis/chassis_odometry.c)

## 开源实现与论文选择

- **WPILib v2026.2.2**，固定提交 [`7ca35e5678cf32caec6a1a866ca51d0136c4c398`](https://github.com/wpilibsuite/allwpilib/tree/7ca35e5678cf32caec6a1a866ca51d0136c4c398)：参考 [`Odometry::Update`](https://github.com/wpilibsuite/allwpilib/blob/7ca35e5678cf32caec6a1a866ca51d0136c4c398/wpimath/src/main/native/include/frc/kinematics/Odometry.h) 的“轮位置差求车体增量，IMU替代轮转角”，以及 [`Pose2d::Exp`](https://github.com/wpilibsuite/allwpilib/blob/7ca35e5678cf32caec6a1a866ca51d0136c4c398/wpimath/src/main/native/include/frc/geometry/Pose2d.h) 的平面刚体积分，按本车接口适配。BSD 许可原文保留于 [LICENSE-WPILib.txt](LICENSE-WPILib.txt)。没有把四轮或差速车的运动学套用到本车。
- **ROS 2 omni_wheel_drive_controller**，固定提交 [`62af79e4788adaf4718491968c3f8fcc632af265`](https://github.com/ros-controls/ros2_controllers/blob/62af79e4788adaf4718491968c3f8fcc632af265/omni_wheel_drive_controller/src/odometry.cpp)：该实现也提供位置差与速度入口。此前轮速积分按其圆弧公式独立实现；保留 `Odometry_Update()` 供对照测试，正式控制不会自动切回轮速积分。参考源采用 Apache-2.0，未移入上游 ROS/Eigen 代码。
- SciSpace 检索的 [三轮全向里程计系统误差标定研究](https://scispace.com/papers/systematic-odometry-error-evaluation-and-correction-in-a-29u40hfa)（[2022 原论文](https://www.mdpi.com/2076-3417/12/5/2606)）说明需要以独立轨迹观测拟合几何参数。本车尚未尺量或获得视觉真值；轮半径35mm、名义中心距108mm保留。用户指定信任YAW后，以30圈实测作相对标定，将旋转项等效臂长设为98.2mm，不能当作实测机械尺寸或绝对精度标定。
- [轮编码器、MEMS陀螺仪与单目视觉紧耦合研究](https://scispace.com/papers/tightly-coupled-monocular-visual-odometric-slam-using-wheels-1g432zoxxr)（[作者预印本](https://arxiv.org/abs/1804.04854)）作为后续视觉融合参考；当前没有移植优化器、实现视觉SLAM或宣称在线漂移校正。

## 本车约定

CAN ID1=后轮，ID2=右前轮，ID3=左前轮；x前、y左、航向俯视逆时针为正。控制层把电机安装方向 −1 和齿比1换算后，同时传入位置及速度；`motor_chassis.c` 负责统一正逆解，算法层不重复反向。

本车MIT位置反馈在±12.5rad间回绕，周期为25rad，并非2π。位置入口解绕相邻样本角差，再通过三轮正解得到车体 `dx/dy`；以ESKF相邻航向差进行SE(2)积分。小角处使用展开式，世界x/y累计采用double。速度入口与位置入口切换时只重建基线，不跨来源积分。

位置入口只接受有限且处于协议范围内的样本；超过50ms、非法位置/速度及不可实现的角跳变使估计失效并保留最后位姿。正式入口用各轮前后帧的 CAN 接收间隔计算跳变上界，采用40rad/s、1ms时钟量化余量和两个编码器量化码；同步入口按组间隔计算。控制层仍负责三轮新序号、使能状态、同一临界区快照及IMU新鲜度检查；三轮并非硬件同步。

`vx_mps/vy_mps/wheel_wz_rad_s` 继续表示同组速度反馈正解，仅供诊断；即使它们有停车尾迹，静止编码器也不会因此累加x/y。位置增量不能消除打滑、轮径误差、编码器本身误差或六轴航向漂移，实际地面精度仍须外部测量。

## 验证入口

`python STM32/tests/run_native.py` 覆盖独立接地点几何、八向、多次双向25rad回绕、计时回绕、IMU主导圆弧、断流恢复、错误跳变、输入源切换和真实控制链。`test_simulate_return.py` 编译真实C位置入口；其电机角位置独立于速度反馈偏差生成，模型真值仅作评估，不供导航使用。

板端固件版本、实测结果和未完成项目记录于 [STM32/docs/ODOMETRY.md](../../../../docs/ODOMETRY.md)，不能用离线模拟结果代替实车精度验证。

## 定点规划

`point_plan.*` 在无障碍平面选择当前位置至世界目标的欧氏直线，角度按最短方向包裹。平移和转向可以同时进行，适用于本车全向轮；不沿去程折返、不先对准车头才直行。每拍使用有效里程计更新剩余距离，按制动距离限速，由底盘现有斜坡执行。轨迹长度是几何下界，实走长度受控制、打滑和里程误差影响。没有地图输入，不做避障，不能当成A*或有障碍最优规划。

参考 [WPILib TrapezoidProfile固定版本](https://github.com/wpilibsuite/allwpilib/blob/7ca35e5678cf32caec6a1a866ca51d0136c4c398/wpimath/src/main/native/include/frc/trajectory/TrapezoidProfile.h)的速度/加速度约束思想；本实现为反馈驱动的剩余制动距离规划，不是按时间开环跟随上游轨迹，也未实现jerk约束或时间最优证明。

世界位移继续采用double；正常航向包裹走加减一圈分支，三角函数和近段开方采用单精度，巡航段不重复开方。运动学预计算几何常量倒数，轮序/极性/35mm轮半径不变，旋转项使用98.2mm等效臂长。速度和位置里程计入口使用具名内部模式，避免来源切换混用。

实测用命令7接入最短路径，单次最多1.5m、位置围栏1.8m；主机实验另有更小的路线约束。达到1cm/4°内层范围开始停车，2cm/10°仍为返航结束核验。命令8注入图像坐标，只使用视觉算法生成速度，里程计只检查区域和终点。实验步骤和未实测项目见 [本轮记录](../../../../docs/ALGORITHM_UPGRADE_20261117.md)。

## YAW主导的平面ESKF

`heading_eskf.*` 的名义状态为航向 θ 和世界z陀螺偏置 b，误差状态为 `[δθ, δb]`。使用前后陀螺均值预测 `θ += (ω-b)dt`，协方差按 `F=[[1,-dt],[0,1]]` 与连续白噪声/偏置随机游走离散过程噪声传播。

YAW观测的 H 为 `[1,0]`，创新取最短角差；轮角增量只构造 `gyro_mean-wheel_delta/dt` 的弱偏置观测，H 为 `[0,1]`，没有累计编码器绝对角度来拉回航向。每个标量观测使用Joseph协方差更新，注入航向/偏置后误差均值归零。SO(2)交换，重置Jacobian严格为单位矩阵。最后更新YAW，使输出偏向用户选定的YAW参考。INS 对源 YAW 与陀螺预测的最短差使用5°门限；孤立跳点发布预测航向供控制，ESKF只校正新且可信的观测。源 YAW 连续三帧超限时锁存无效，重新初始化 INS 才能恢复。

默认100Hz下YAW标准差1°，gyro过程噪声密度0.02rad/√s，偏置随机游走0.0005rad/s/√s。轮角速度标准差至少0.5rad/s，并随旋转速度和残差增加；残差超过1.5rad/s只跳过轮约束。两个观测的R按`0.01/dt`缩放，避免改变采样频率时改变单位时间信息量。偏置限幅±0.05rad/s，防止把大几何误差吸收为零偏。增益由协方差计算，并非固定“90%/10%”。

正式入口在无效IMU、无效轮输入或间隔超过50ms时失效；恢复首帧重建ESKF和轮位置基线，不补算缺测运动。保留原始YAW的`Odometry_Update_Position()`及速度对照入口；切换来源不跨帧积分。`simulate_return.py`仍明确调用原始YAW对照入口，其历史模拟不代表ESKF测试；ESKF由`test_heading_eskf.c`覆盖噪声、10%轮角偏差、滑动、零偏、跨圈、协方差、变周期和实际SE(2)入口。

AHRS YAW与gyro来自同一IMU，有相关性。噪声与方差下限是工程保守设置，不代表严格独立观测后验，也不能消除六轴YAW长期漂移。当前ESKF只估计平面航向/偏置，不是完整三维惯导；后续独立视觉世界位姿需带时间戳、外参与不确定度另行融合。

参考 [Solà的ESKF推导，§5–6](https://arxiv.org/html/1711.02508v1)（[SciSpace检索记录](https://scispace.com/papers/quaternion-kinematics-for-the-error-state-kalman-filter-4h3c1yi2bu)）的误差预测、注入和复位流程；对照阅读 [OpenVINS StateHelper固定版本](https://github.com/rpng/open_vins/blob/74a63cf758f595cc280f2165e5a6576e7ef8521d/ov_msckf/src/state/StateHelper.cpp)的误差状态更新组织。按本车二维模型独立编写C实现，没有复制OpenVINS的GPL C++/Eigen源码。

旋转校准和同向30圈复测见 [YAW/ESKF板端记录](../../../../docs/SPIN_YAW_20261117.md)。

## 自动导航速度实验

自动导航的沿程速度目标仍由视觉制动包络和里程计点位规划给出。控制拍内只在需要增加平移速度时使用实验方法。三个候选共享现有合加速度上限，最后一步限制在目标速度内，反向平移先过零。

- MPC 每拍枚举 7 个加速度候选，用一阶速度响应预测 8 拍，比较跟踪误差、动作幅值和动作变化代价，仅执行首拍。
- SMC 以沿速度差方向的反馈误差为滑模变量，通过 0.025 m/s 的边界层连续地限制输出。
- ADRC 用二阶扩张状态观测器估计平移速度和总扰动，使用上一拍限幅后的指令增量，并在同一加速度包络内补偿。

方法取自有限时域滚动控制、滑模边界层和扩张状态观测器的通用结构。这里的模型及初值是本车的主机实验配置。 [OSQP 的 MPC 示例](https://osqp.org/docs/examples/mpc.html) · [移动机器人 SMC 和 ADRC 研究](https://scispace.com/papers/robust-tracking-control-of-wheeled-mobile-robot-based-on-1j8jfdp1ea)
