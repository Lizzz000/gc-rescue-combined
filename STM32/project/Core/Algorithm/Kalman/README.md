# 自适应图像坐标卡尔曼滤波

`adaptive_kalman.c/.h` 是独立的二维常速度滤波模块，不依赖 RTOS 或底盘控制；自动靠近流程从此目录调用它。它估计图像中心位置和图像速度，不估计车体世界位置或轮速。状态由调用者持有，不使用动态内存。

## 调用方式

1. 创建 `AdaptiveKalman` 状态，调用 `AdaptiveKalman_Init(&filter, sigma_x, sigma_y, sigma_acceleration)`。参数必须为有限正数。
2. 对每个按时间顺序到达的有效观测调用 `AdaptiveKalman_Update(&filter, x, y, timestamp_ms)`，从 `filter.position[0/1]` 和 `filter.velocity[0/1]` 读取估计。
3. 第一帧初始化位置、速度为零；同毫秒和倒序帧被忽略，正常的 32 位毫秒计数回绕可处理。更换目标或观测长时间中断后，调用者必须重新初始化。

坐标单位为 q，图像宽或高对应 10000 q；位置噪声标准差 `sigma_x`、`sigma_y` 的单位为 q，加速度噪声标准差 `sigma_acceleration` 的单位为 q/s²，速度输出单位为 q/s。输入坐标必须为有限数值。

自动靠近的参数在 `project/Core/Control/Jetson/jetson_auto_config.h`：`JETSON_AUTO_APPROACH_SIGMA_X_Q`、`JETSON_AUTO_APPROACH_SIGMA_Y_Q` 默认分别为 15、25 q，`JETSON_AUTO_APPROACH_SIGMA_ACCEL_Q_S2` 默认 4000 q/s²。它们是初值，修改后需要重新编译。源帧重复时控制层不重复更新；帧间隔达到 `JETSON_AUTO_LINK_TIMEOUT_MS`（默认 250 ms）时重新初始化；制动阶段清空视觉状态，重新对准时从新观测开始。到位核验仍使用原始观测。

## 算法与调试

两轴分别以位置、速度为状态；每帧按实际间隔预测，再使用观测更新。创新归一化平方用于调整后续测量方差，同时对较大的当前创新额外降权；缩放范围和记忆系数在 `adaptive_kalman.c` 中。噪声估计按帧更新，因此帧率变化会改变其响应时间。较大加速度或持续偏移也可能被当作噪声，不能单凭调大降权来解决过冲。

从 `STM32/` 运行 `python tests/run_native.py` 可执行全部主机回归；`tests/native/fixtures/image_tracker.c` 只供离线对照，不编入固件。固定种子仿真覆盖匀速、加速、噪声突增和孤立误检；还检查时间回绕、重复帧、重置、协方差及自动控制流程。现有仿真中，自适应滤波在噪声突增和孤立误检时误差下降，加速跟踪略有退化，不能据此断言实车改善。

实车验证尚未完成。调试时应记录源帧时间、原始与滤波中心、速度估计、底盘指令和可验证的参考位置，比较靠近时的响应延迟、过冲、误检恢复和停车位置，再调整噪声参数。

本实现参考 [FilterPy 的创新协方差](https://github.com/rlabbe/filterpy/blob/5df4e095fac07684a05c495bd72d3424f7c68c9c/filterpy/kalman/kalman_filter.py)和 [robot_localization 的创新异常检验](https://github.com/cra-ros-pkg/robot_localization/blob/b1e373177cc8190f2a424243a73a1023c882140f/src/filter_base.cpp)，未复制其源码。
