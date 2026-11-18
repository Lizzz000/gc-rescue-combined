#ifndef JETSON_AUTO_CONFIG_H
#define JETSON_AUTO_CONFIG_H

/* 标定后改为 1，上电自动搬运；0 为类型 2 手动遥控固件。夹爪由 Jetson 负责。 */
#ifndef JETSON_AUTO_ENABLED
#define JETSON_AUTO_ENABLED 0
#endif

/* 单次底盘联调：由 SWD 显式启动，物资→规则转向→安全区对准及直行；不执行夹爪。 */
#ifndef JETSON_AUTO_CHASSIS_TEST
#define JETSON_AUTO_CHASSIS_TEST 0
#endif
#if JETSON_AUTO_ENABLED && JETSON_AUTO_CHASSIS_TEST
#error "Select either full transport or chassis-only test"
#endif
/* 自动搬运或底盘联调固件：USB 收 Jetson 下行并运行自动流程；否则为类型 2 手动固件。 */
#define JETSON_AUTO_CAN_ENABLED (JETSON_AUTO_ENABLED || JETSON_AUTO_CHASSIS_TEST)
#if defined(CHASSIS_TUNING_ENABLED) && CHASSIS_TUNING_ENABLED && JETSON_AUTO_CAN_ENABLED
#error "Chassis tuning cannot share control with Jetson auto or chassis-only test"
#endif
#ifndef UART6_MANUAL_ENABLED
#define UART6_MANUAL_ENABLED 0
#endif
#if UART6_MANUAL_ENABLED && (JETSON_AUTO_CAN_ENABLED || (defined(CHASSIS_TUNING_ENABLED) && CHASSIS_TUNING_ENABLED))
#error "USART6 manual cannot share chassis control with Jetson or SWD tuning"
#endif
#define JETSON_AUTO_CHASSIS_TEST_TIMEOUT_MS 90000U

/* 到位窗口为 x、y 设定点及各轴允许误差。单位为原图宽/高的比例，0 待标定。 */
#define JETSON_AUTO_TARGET_X 0.5000f
#define JETSON_AUTO_TARGET_Y 0.7500f
#define JETSON_AUTO_TARGET_TOL_X 0.0100f
#define JETSON_AUTO_TARGET_TOL_Y 0.0100f
/* 安全区直行前的对准点：类型 4 为安全区透视中心，需板端标定。 */
#define JETSON_AUTO_SAFETY_X 0.5000f
#define JETSON_AUTO_SAFETY_Y 0.7500f
#define JETSON_AUTO_SAFETY_TOL_X 0.0100f
#define JETSON_AUTO_SAFETY_TOL_Y 0.0100f

/* 左右安全区的实车对准窗口；现场可以分别覆盖。 */
#ifndef JETSON_AUTO_LEFT_X
#define JETSON_AUTO_LEFT_X JETSON_AUTO_SAFETY_X
#endif
#ifndef JETSON_AUTO_LEFT_Y
#define JETSON_AUTO_LEFT_Y JETSON_AUTO_SAFETY_Y
#endif
#ifndef JETSON_AUTO_LEFT_TOL_X
#define JETSON_AUTO_LEFT_TOL_X JETSON_AUTO_SAFETY_TOL_X
#endif
#ifndef JETSON_AUTO_LEFT_TOL_Y
#define JETSON_AUTO_LEFT_TOL_Y JETSON_AUTO_SAFETY_TOL_Y
#endif
#ifndef JETSON_AUTO_RIGHT_X
#define JETSON_AUTO_RIGHT_X JETSON_AUTO_SAFETY_X
#endif
#ifndef JETSON_AUTO_RIGHT_Y
#define JETSON_AUTO_RIGHT_Y JETSON_AUTO_SAFETY_Y
#endif
#ifndef JETSON_AUTO_RIGHT_TOL_X
#define JETSON_AUTO_RIGHT_TOL_X JETSON_AUTO_SAFETY_TOL_X
#endif
#ifndef JETSON_AUTO_RIGHT_TOL_Y
#define JETSON_AUTO_RIGHT_TOL_Y JETSON_AUTO_SAFETY_TOL_Y
#endif

/* 正式搬运的场地限位，0 表示未标定；前两项均须在清空场地后实测。 */
#define JETSON_AUTO_RETURN_MAX_M 0.0f
#define JETSON_AUTO_ALIGN_MAX_M 0.0f
#define JETSON_AUTO_RETURN_SPEED_MPS 0.12f
#define JETSON_AUTO_ALIGN_SPEED_MPS 0.10f
#define JETSON_AUTO_RETURN_TIMEOUT_MS 20000U
#define JETSON_AUTO_ALIGN_TIMEOUT_MS 10000U

/* 符号乘 (设定点 - 实测点)，横向取本次相机安装的实测方向。 */
#define JETSON_AUTO_X_TO_LEFT (-1)
#define JETSON_AUTO_Y_TO_FORWARD 1
/* 视觉速度规划；图像响应是待相机实测的局部初值，不是米制坐标标定。 */
#define JETSON_AUTO_APPROACH_SPEED_MPS 0.50f
#define JETSON_AUTO_APPROACH_ACCEL_MPS2 0.40f
#define JETSON_AUTO_APPROACH_DECEL_MPS2 0.35f
#define JETSON_AUTO_APPROACH_LATENCY_S 0.12f
#define JETSON_AUTO_RESPONSE_X_Q_PER_M 10000.0f
#define JETSON_AUTO_RESPONSE_Y_Q_PER_M 10000.0f
#define JETSON_AUTO_SEARCH_WZ_RAD_S 2.0f

/* 图像位置/加速度标准差，单位 q、q/s²；1 张图宽/高为 10000q。 */
#define JETSON_AUTO_APPROACH_SIGMA_X_Q 15.0f
#define JETSON_AUTO_APPROACH_SIGMA_Y_Q 25.0f
#define JETSON_AUTO_APPROACH_SIGMA_ACCEL_Q_S2 4000.0f

/* 现有类型 3/4 周期流的断流期限，ms；合法无目标帧也续期，不新增心跳报文。 */
#define JETSON_AUTO_LINK_TIMEOUT_MS 250U
/* 有效图像中心的接收期限，独立于通信在线状态，单位 ms。 */
#define JETSON_AUTO_CENTER_TIMEOUT_MS 250U
/* 使能前姿态样本允许的最大年龄，ms。 */
#define JETSON_AUTO_IMU_MAX_AGE_MS 20U
#define JETSON_AUTO_BRAKE_TIMEOUT_MS 500U

/* 发出使能请求后，等待三轮反馈与姿态、里程计就绪的期限。 */
#define JETSON_AUTO_ARM_TIMEOUT_MS 1000U

/* 单次底盘联调用：对准安全区后锁航向直行，0 待实车标定。 */
#define JETSON_AUTO_SAFETY_STRAIGHT_M 0.0f
/* 安全区直行、检查后退与拒绝恢复共用定距速度。 */
#define JETSON_AUTO_DISTANCE_SPEED_MPS 0.50f
#define JETSON_AUTO_ROI_BACK_M 0.0f
/* 直行目标可配置为 0.50 m/s；计时包含加速，到期下发零速。 */
#define JETSON_AUTO_PUSH_SPEED_MPS 0.50f
#define JETSON_AUTO_PUSH_DURATION_MS 1200U
#define JETSON_AUTO_RECOVERY_BACK_M 0.0f
/* 单次联调按目的区选择转向；正式搬运从出发点原地转 180° 后找目标。 */
#define JETSON_AUTO_OUT_TURN_DEG 180.0f
#define JETSON_AUTO_RETURN_TURN_DEG 180.0f

/* Jetson 处理一次请求的服务预算，ms。 */
#define JETSON_AUTO_REQUEST_SERVICE_MS 3000U
/* 覆盖 USB 提交等待、250 ms 重报、20 ms 双向调度、100 ms 回复队列及传输，单位 ms。 */
#define JETSON_AUTO_REQUEST_LINK_MARGIN_MS 450U
/* 唯一请求期限：自进入请求阶段起算，含 USB 提交等待，见 Jetson/docs/usb-protocol.md。 */
#define JETSON_AUTO_REQUEST_TIMEOUT_MS (JETSON_AUTO_REQUEST_SERVICE_MS + JETSON_AUTO_REQUEST_LINK_MARGIN_MS)
#define JETSON_AUTO_REQUEST_RETRY_MS 250U
/* 10 s 运动 + prepare/Stop 各最多 3×200 ms + 末次 exchange 200 ms/sleep 20 ms
 * + 链路余量 450 ms = 11870 ms，向上取 12000 ms。启动从首个受理回执起算。 */
#define JETSON_AUTO_ACTION_TIMEOUT_MS 12000U

#endif
