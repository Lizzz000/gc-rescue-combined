# 图像滤波与视觉伺服

`adaptive_kalman.c/.h` 按源帧时间分别估计图像两轴的位置和速度。宽度和高度各为 10000 q，速度单位为 q/s，加速度标准差单位为 q/s²。`visual_servo.c/.h` 接收位置、速度和本帧有效轴掩码，可单独编译和调用。自动控制与现场调参通过 `Control/Vision/tracked_visual_servo.c` 将跟踪结果传给速度规划。

## 本轮改进

- 保留原10ms正常观测的增益规律，把噪声记忆改为0.323333s时间常数，减小帧率变化的影响。
- 采用Joseph协方差更新并约束半正定，去掉原来的融合乘加库调用；无动态矩阵分配。
- NIS超过100的孤立异常只预测，不注入速度；连续三帧同侧大创新重新捕获，避免永远拒绝真实目标变化。9至100的创新仍使用原有限测量降权，避免把较强普通噪声全部判成失效。
- 重复/倒序源时间忽略，uint32回绕正常处理；间隔达到250ms重建基线。NaN、Inf、非法初始化参数不污染已有状态。
- `accepted_mask` 表示本帧接受的轴；速度规划对拒绝轴减速，不跟随跳点突然反向。

`AdaptiveKalman_Init()` 先初始化参数，`AdaptiveKalman_Update()` 每个新源帧调用一次。正式视觉层仍按原始窗口制动、零速下发后用新原始帧核验，不把滤波坐标当外部真值。底盘不再使用视觉积分补偿：`Visual_Servo_Update()` 根据剩余图像误差、响应初值和执行延迟解制动速度上限，限制合速度与加速增量；入窗轴归零，反向先过零。物理逐拍斜坡仍在底盘层。

自动流程与现场调参共用 `Control/Vision/visual_tracking_config.h` 中的相机轴向、响应初值、延迟和滤波参数。每轴默认响应为 10000 q/m，制动余量默认取 0.12 s 加本帧间隔。上述初值有待真实相机标定。自动流程仍可用 `JETSON_AUTO_RESPONSE_X_Q_PER_M`、`JETSON_AUTO_APPROACH_LATENCY_S` 等已有配置名覆盖。

## 视觉速度规划

视觉伺服取半窗内侧作为制动目标，按 `v*t+v²/(2a)` 计算速度上限。输入是图像坐标误差和滤波估计，输出是图像两轴方向的平移速度（m/s）。控制层再把它映射为底盘前向和侧向速度。

## 来源与选择

- GitHub核对 [FilterPy固定版本](https://github.com/rlabbe/filterpy/blob/5df4e095fac07684a05c495bd72d3424f7c68c9c/filterpy/kalman/kalman_filter.py) 的Joseph形式。此处按二维位置/速度直接展开公式，未复制Python源码。
- SciSpace检索 [Robust Extended Kalman Filtering for Systems With Measurement Outliers](https://scispace.com/papers/robust-extended-kalman-filtering-for-systems-with-9i1n40sg)。参考其限制异常创新影响的方向；本实现是线性CV滤波和门限重捕获，并非论文IS-EKF，也不继承其稳定性证明。
- [全向底盘约束视觉MPC](https://scispace.com/papers/quasi-min-max-mpc-for-visual-servoing-stabilization-of-l5rvhctj)作为图像/物理约束的后续参考。当前只有图像中心，没有完整深度、标定模型和LMI求解器，因此使用可验证的标量制动上限，没有声称移植MPC。SciSpace此次返回的摘要/提取未列出完整作者局限，以上取舍是本工程判断。

## 验证范围

主机覆盖固定目标抖动、加减速、噪声突增、孤立误检、15/30/50/100fps及丢帧、时间回绕、非法输入、协方差、真实流程和执行器闭环模型。与修改前自适应版本同轨迹同种子比较，静止/加速/噪声突增的位置RMS比均约1.000，孤立误检约0.636。不能将这些比例当作真实相机精度。

一次0.6m前进、0.1m侧移的坐标注入实测已完成，用户确认平顺返航；该次属于中间固件。最终版本1.2m/0.6m视觉去程已完成，完整返航因DAP读取超时未通过；不能宣称整轮成功。固件、去程误差、测试验收修复及失败日志见 [验证记录](../../../../docs/ALGORITHM_UPGRADE_20261117.md)。
