#ifndef JETSON_AUTO_CONFIG_H
#define JETSON_AUTO_CONFIG_H

/* 标定后改为 1，上电自动搬运；0 保留 0x180 联调与舵机标定。 */
#ifndef JETSON_AUTO_ENABLED
#define JETSON_AUTO_ENABLED 0
#endif

/* 单次底盘联调：由 SWD 显式启动，物资→规则转向→安全区；不执行夹爪或定距。 */
#ifndef JETSON_AUTO_CHASSIS_TEST
#define JETSON_AUTO_CHASSIS_TEST 0
#endif
#if JETSON_AUTO_ENABLED && JETSON_AUTO_CHASSIS_TEST
#error "Select either full transport or chassis-only test"
#endif
#define JETSON_AUTO_CHASSIS_TEST_TIMEOUT_MS 90000U

/* 旧车 MG90 空载调试范围，单位 μs；开合端点按当前机构实测。 */
#define TRANSPORT_SERVO_CALIBRATION_MIN_US 1000U
#define TRANSPORT_SERVO_CALIBRATION_MAX_US 2000U

/* 到位窗口为 x、y 设定点及各轴允许误差。单位为原图宽/高的比例，0 待标定。 */
#define JETSON_AUTO_TARGET_X 0.5000f
#define JETSON_AUTO_TARGET_Y 0.7500f
#define JETSON_AUTO_TARGET_TOL_X 0.0100f
#define JETSON_AUTO_TARGET_TOL_Y 0.0100f
#define JETSON_AUTO_SAFETY_X 0.5000f
#define JETSON_AUTO_SAFETY_Y 0.7500f
#define JETSON_AUTO_SAFETY_TOL_X 0.0100f
#define JETSON_AUTO_SAFETY_TOL_Y 0.0100f

/* 符号乘 (设定点 - 实测点)，横向取本次相机安装的实测方向。 */
#define JETSON_AUTO_X_TO_LEFT (-1)
#define JETSON_AUTO_Y_TO_FORWARD 1
/* 图像误差速度增益 m/s 每单位归一化误差、平移上限 m/s、搜索车体角速度 rad/s。 */
#define JETSON_AUTO_APPROACH_GAIN 0.80f
#define JETSON_AUTO_APPROACH_SPEED_MPS 0.50f
#define JETSON_AUTO_SEARCH_WZ_RAD_S 0.0f

/* 图像卡尔曼的坐标噪声和过程加速度标准差，单位为定点单位及定点单位/s²。 */
#define JETSON_AUTO_APPROACH_SIGMA_X_Q 15.0f
#define JETSON_AUTO_APPROACH_SIGMA_Y_Q 25.0f
#define JETSON_AUTO_APPROACH_SIGMA_ACCEL_Q_S2 4000.0f
/* 0.18 s 为实测漂移推得的制动试调初值，锁航向后复标。 */
#define JETSON_AUTO_APPROACH_BRAKE_S 0.18f
/* 积分增益和单轴补偿上限为试调初值。单位分别为 m/s 每归一化误差秒和 m/s。 */
#define JETSON_AUTO_APPROACH_I_GAIN 2.0f
#define JETSON_AUTO_APPROACH_I_LIMIT_MPS 0.05f

/* 正常视觉入窗后的停车确认期限，ms；不改变故障制动期限或停稳轮速门限。 */
#define JETSON_AUTO_VISION_STOP_TIMEOUT_MS 1000U
/* 停稳后的核验等待：Jetson 源帧龄上限 1000 ms，另留 20 ms 链路余量。 */
#define JETSON_AUTO_VERIFY_WAIT_MS (1000U + 20U)

/* 检查后退与拒绝恢复仍使用定距；速度和距离留待实车标定。 */
#define JETSON_AUTO_DISTANCE_SPEED_MPS 0.0f
#define JETSON_AUTO_ROI_BACK_M 0.0f
/* 前后直行时两后轮目标约 10 rad/s；计时包含加速，停稳还需额外时间。 */
#define JETSON_AUTO_PUSH_SPEED_MPS (10.0f * 0.035f / 0.8660254f)
#define JETSON_AUTO_PUSH_DURATION_MS 1200U
#define JETSON_AUTO_RECOVERY_BACK_M 0.0f
/* 左/右去程分别左/右转；返程方向取反，角度均为 180°。 */
#define JETSON_AUTO_OUT_TURN_DEG 180.0f
#define JETSON_AUTO_RETURN_TURN_DEG 180.0f

/* TIM1_CH1 开合高电平脉宽 μs 和动作等待 ms；均留待实车标定。 */
#define TRANSPORT_SERVO_OPEN_US 0U
#define TRANSPORT_SERVO_CLOSED_US 0U
#define JETSON_AUTO_SERVO_OPEN_MS 0U
#define JETSON_AUTO_SERVO_CLOSE_MS 0U

/* 未入邮箱最多等待 3 s。首次提交后，回执期限覆盖 Jetson 3 s 服务预算及链路余量。 */
#define JETSON_AUTO_REQUEST_TIMEOUT_MS 3000U
/* 覆盖 250 ms 重报、20 ms 双向调度、100 ms 回复队列及 CAN 传输，单位 ms。 */
#define JETSON_AUTO_REQUEST_LINK_MARGIN_MS 450U
#define JETSON_AUTO_REQUEST_RETRY_MS 250U

#endif
