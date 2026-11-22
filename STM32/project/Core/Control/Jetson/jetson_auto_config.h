#ifndef JETSON_AUTO_CONFIG_H
#define JETSON_AUTO_CONFIG_H

#include "chassis_mode.h"
#include "chassis.h"
#include "visual_tracking_config.h"

/* 场地距离与路线标定 */
/* 正式搬运的场地距离使用 m，保留 0 时流程等待现场标定。 */
/* 定时横移和整区复定位的允许距离上限。 */
#define JETSON_AUTO_ALIGN_MAX_M 0.0f

/* 夹取后的 ROI 检查退距和投送拒绝时的恢复退距。 */
#define JETSON_AUTO_ROI_BACK_M 0.0f
#define JETSON_AUTO_RECOVERY_BACK_M 0.0f

/* 单次联调按目的区选择转向；正式搬运从出发点原地转 180° 后找目标。 */
#define JETSON_AUTO_OUT_TURN_DEG 180.0f
#define JETSON_AUTO_RETURN_TURN_DEG 180.0f

/* 物资与左右安全区视觉窗口 */
/* 中心坐标和各轴容差按原图宽高归一化，现场按相机与目标位置标定。 */
#define JETSON_AUTO_TARGET_X 0.1000f
#define JETSON_AUTO_TARGET_Y 0.900f
#define JETSON_AUTO_TARGET_TOL_X 0.0100f
#define JETSON_AUTO_TARGET_TOL_Y 0.0100f
/* 单次底盘联调使用的类型 4 对准窗口。 */
#define JETSON_AUTO_SAFETY_X 0.5000f
#define JETSON_AUTO_SAFETY_Y 0.7500f
#define JETSON_AUTO_SAFETY_TOL_X 0.0100f
#define JETSON_AUTO_SAFETY_TOL_Y 0.0100f

/* 正式搬运用整块安全区透视中心复定位，按实车停位标定。 */
#ifndef JETSON_AUTO_LOCALIZE_X
#define JETSON_AUTO_LOCALIZE_X JETSON_AUTO_SAFETY_X
#endif
#ifndef JETSON_AUTO_LOCALIZE_Y
#define JETSON_AUTO_LOCALIZE_Y JETSON_AUTO_SAFETY_Y
#endif
#ifndef JETSON_AUTO_LOCALIZE_TOL_X
#define JETSON_AUTO_LOCALIZE_TOL_X JETSON_AUTO_SAFETY_TOL_X
#endif
#ifndef JETSON_AUTO_LOCALIZE_TOL_Y
#define JETSON_AUTO_LOCALIZE_TOL_Y JETSON_AUTO_SAFETY_TOL_Y
#endif

/* 正式导航与整区视觉复定位的限速由 chassis_config.h 提供。 */
#define JETSON_AUTO_RETURN_SPEED_MPS CHASSIS_NAV_TRAVEL_SPEED_MPS
#ifndef JETSON_AUTO_RELOCALIZE_SPEED_MPS
#define JETSON_AUTO_RELOCALIZE_SPEED_MPS CHASSIS_NAV_ALIGN_SPEED_MPS
#endif

#if JETSON_AUTO_ENABLED
#define JETSON_AUTO_APPROACH_SPEED_MPS CHASSIS_NAV_TRAVEL_SPEED_MPS
#else
#define JETSON_AUTO_APPROACH_SPEED_MPS CHASSIS_RUN_SPEED_MPS
#endif
#ifndef JETSON_AUTO_APPROACH_ACCEL_MPS2
#if JETSON_AUTO_ENABLED
/* 按离线验证使用的比例给视觉规划留出加速度余量。 */
#define JETSON_AUTO_APPROACH_ACCEL_MPS2 (CHASSIS_NAV_ACCELERATION_MPS2 * (8.0f / 9.0f))
#else
#define JETSON_AUTO_APPROACH_ACCEL_MPS2 VISUAL_TRACKING_ACCEL_MPS2
#endif
#endif
#ifndef JETSON_AUTO_APPROACH_DECEL_MPS2
#define JETSON_AUTO_APPROACH_DECEL_MPS2 VISUAL_TRACKING_DECEL_MPS2
#endif
#ifndef JETSON_AUTO_APPROACH_LATENCY_S
#define JETSON_AUTO_APPROACH_LATENCY_S VISUAL_TRACKING_LATENCY_S
#endif

/* 安全区直行、检查后退与拒绝恢复共用定距速度。 */
#define JETSON_AUTO_DISTANCE_SPEED_MPS CHASSIS_RUN_SPEED_MPS

/* 左右横移共用速度和时长。盲推和直线回退分别计时。
 * 速度为目标值，计时包含加速。到期发零速，随后等待停稳。 */
#define JETSON_AUTO_LATERAL_SPEED_MPS 0.40f
#define JETSON_AUTO_LATERAL_DURATION_MS 700U
#define JETSON_AUTO_PUSH_SPEED_MPS 0.40f
#define JETSON_AUTO_PUSH_DURATION_MS 1200U
#define JETSON_AUTO_RETRACT_SPEED_MPS 0.40f
#define JETSON_AUTO_RETRACT_DURATION_MS 1000U

/* 视觉轴向、图像响应与滤波 */
/* 符号乘 (设定点 - 实测点)，横向取本次相机安装的实测方向。 */
#ifndef JETSON_AUTO_X_TO_LEFT
#define JETSON_AUTO_X_TO_LEFT VISUAL_TRACKING_X_TO_LEFT
#endif
#ifndef JETSON_AUTO_Y_TO_FORWARD
#define JETSON_AUTO_Y_TO_FORWARD VISUAL_TRACKING_Y_TO_FORWARD
#endif

/* 图像响应单位为 q/m，按镜头和工作距离标定。 */
#ifndef JETSON_AUTO_RESPONSE_X_Q_PER_M
#define JETSON_AUTO_RESPONSE_X_Q_PER_M VISUAL_TRACKING_RESPONSE_X_Q_PER_M
#endif
#ifndef JETSON_AUTO_RESPONSE_Y_Q_PER_M
#define JETSON_AUTO_RESPONSE_Y_Q_PER_M VISUAL_TRACKING_RESPONSE_Y_Q_PER_M
#endif

/* 图像位置/加速度标准差，单位 q、q/s²；1 张图宽/高为 10000q。 */
#ifndef JETSON_AUTO_APPROACH_SIGMA_X_Q
#define JETSON_AUTO_APPROACH_SIGMA_X_Q VISUAL_TRACKING_SIGMA_X_Q
#endif
#ifndef JETSON_AUTO_APPROACH_SIGMA_Y_Q
#define JETSON_AUTO_APPROACH_SIGMA_Y_Q VISUAL_TRACKING_SIGMA_Y_Q
#endif
#ifndef JETSON_AUTO_APPROACH_SIGMA_ACCEL_Q_S2
#define JETSON_AUTO_APPROACH_SIGMA_ACCEL_Q_S2 VISUAL_TRACKING_SIGMA_ACCEL_Q_S2
#endif

/* 动作时限、视觉时效与停稳保护 */
/* 返航限时涵盖停车后的位姿复核，对准限时也用于反平移和重新定位。 */
#define JETSON_AUTO_RETURN_TIMEOUT_MS 20000U
#define JETSON_AUTO_ALIGN_TIMEOUT_MS 10000U

/* 现有类型 3/4 周期流的断流期限，ms；合法无目标帧也续期，不新增心跳报文。 */
#define JETSON_AUTO_LINK_TIMEOUT_MS 250U
/* 有效图像中心的接收期限，独立于通信在线状态，单位 ms。 */
#define JETSON_AUTO_CENTER_TIMEOUT_MS 250U
/* 故障零目标发出后等待停稳的期限。 */
#define JETSON_AUTO_BRAKE_TIMEOUT_MS 500U
/* 正常到位通过速度斜坡减至零，超时转故障制动。 */
#define JETSON_AUTO_HOLD_TIMEOUT_MS 1500U

/* 发出使能请求后，等待三轮反馈与姿态、里程计就绪的期限。 */
#define JETSON_AUTO_ARM_TIMEOUT_MS 1000U

/* 单次底盘联调 */
/* 单趟底盘联调的总时限，单位 ms。 */
#define JETSON_AUTO_CHASSIS_TEST_TIMEOUT_MS 90000U

/* 单次底盘联调用：对准安全区后锁航向直行，0 待实车标定。 */
#define JETSON_AUTO_SAFETY_STRAIGHT_M 0.0f

/* Jetson 请求与夹爪动作时限 */
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
