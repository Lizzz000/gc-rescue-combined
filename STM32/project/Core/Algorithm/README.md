# STM32 算法入口

本目录全部为固定状态的 C 算法，无堆分配、ROS、Eigen 或运行时矩阵库。调度、串口/CAN、目标有效期、使能和故障处理在 `Core/Control`、`Core/Task`。图像坐标不能直接写入世界里程计。

| 目录/入口 | 输入与输出 | 调用方 |
| --- | --- | --- |
| `AHRS/AHRS_update` | 机体系 IMU → 单位四元数；`get_angle` 输出 rad | INS任务 |
| `PID/PID_calc` | 离散温控 PID，增益按调用周期标定 | INS温控 |
| `PID/PID_Step` | 显式dt、测量微分、抗积分饱和 | 底盘锁航向/转向 |
| `Vision/AdaptiveKalman_Update` | 图像中心q → 中心q、图像速度q/s | 视觉接近 |
| `Vision/Visual_Servo_Update` | 图像误差、图像速度 → 平滑平移速度m/s | 正式自动流程、注入测试 |
| `Chassis/Motor_Chassis_*` | 车体速度 ↔ 后/右前/左前轮几何速度 | 底盘控制/里程计 |
| `Chassis/Odometry_Update_Position_ESKF` | 编码器位置差、YAW主导ESKF航向 → 世界位姿m/rad | 正式底盘 |
| `Chassis/Point_Plan_*` | 世界位姿、目标 → 最短直线与短角度速度指令 | 命令7返航 |

## 组织与参考

[视觉算法说明](Vision/README.md)、[运动学/里程计/规划说明](Chassis/README.md)、[AHRS说明](AHRS/README.md)、[PID说明](PID/README.md)分别记录单位、边界、来源与验证。现有 GPL/BSD 许可证保留，新增速度/路径规划按本车接口独立实现，没有复制上游完整控制器。

本车为可侧移的三轮全向底盘。已确认ID1后、ID2右前、ID3左前；车体x前/y左/逆时针正。空旷无障碍空间两点之间直线最短，旋转采用最短角差；当前没有地图或障碍物输入，不能宣称避障或有障碍时的全局最优。

## 验证与状态

从仓库根目录运行 `python STM32/tests/run_native.py`；33套原生回归包含独立三轮几何、32组多帧率滤波场景、24组视觉闭环模型、8向路径闭环模型和控制状态切换。执行 `python -m unittest discover -s STM32/tests -p "test_*session.py"` 验证主机会话。

`python STM32/tests/benchmark_algorithms.py --baseline HEAD^ --output STM32/outputs/benchmark.json` 编译实际C文件，与指定Git版本交替运行7轮，每轮每项100万次。该结果是PC耗时，不是STM32 WCET。具体版本、主机对照、已实测固件及未通过项目见 [本轮记录](../../../docs/ALGORITHM_UPGRADE_20261117.md)。
