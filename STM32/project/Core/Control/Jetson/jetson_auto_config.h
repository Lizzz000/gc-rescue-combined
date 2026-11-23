#ifndef JETSON_AUTO_CONFIG_H
#define JETSON_AUTO_CONFIG_H

#include "chassis_mode.h"
#include "chassis.h"
#include "visual_tracking_config.h"

/* 正式搬运完成指定趟数后失能。0 为连续搬运。 */
#ifndef JETSON_AUTO_DELIVERY_LIMIT
#define JETSON_AUTO_DELIVERY_LIMIT 0U
#endif

/* 场地距离与路线标定 */
/* 正式搬运的场地距离使用 m，保留 0 时流程等待现场标定。 */
/* 定时横移和反平移复定位分别从各段起点限距。
 * 现场确认左右净空各至少 0.30 m，横移和复位的净位移上限取 0.25 m。 */
#define JETSON_AUTO_ALIGN_MAX_M 0.25f

/* 单次联调按目的区选择转向；正式搬运从出发点原地转 180° 后找目标。 */
#define JETSON_AUTO_OUT_TURN_DEG 180.0f
#define JETSON_AUTO_RETURN_TURN_DEG 180.0f

/* 物资与左右安全区视觉窗口 */
/* 单绿物块可直接夹取时，正式 target 节点中心的 479 帧中位值。
 * 中心与各轴容差按 1920×1080 原图归一化。 */
#define JETSON_AUTO_TARGET_X 0.5721f
#define JETSON_AUTO_TARGET_Y 0.8681f
#define JETSON_AUTO_TARGET_TOL_X 0.0100f
#define JETSON_AUTO_TARGET_TOL_Y 0.0100f
/* 单次底盘联调使用的类型 4 对准窗口。 */
#define JETSON_AUTO_SAFETY_X 0.5000f
#define JETSON_AUTO_SAFETY_Y 0.7500f
#define JETSON_AUTO_SAFETY_TOL_X 0.0100f
#define JETSON_AUTO_SAFETY_TOL_Y 0.0100f

/* 正式搬运用整块安全区透视中心复定位，按实车停位标定。 */
#ifndef JETSON_AUTO_LOCALIZE_X
#define JETSON_AUTO_LOCALIZE_X 0.5430f
#endif
#ifndef JETSON_AUTO_LOCALIZE_Y
#define JETSON_AUTO_LOCALIZE_Y 0.3298f
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
/* 整区中心容易受车身摆动影响。精调用较慢的起步和更保守的制动距离。
 * 图像响应以 q/m 估计，取值用于提前减速。搬运接近沿用独立参数。 */
#define JETSON_AUTO_LOCALIZE_ACCEL_MPS2 0.60f
#define JETSON_AUTO_LOCALIZE_LATENCY_S 0.22f
#define JETSON_AUTO_LOCALIZE_RESPONSE_X_Q_PER_M 20000.0f
#define JETSON_AUTO_LOCALIZE_RESPONSE_Y_Q_PER_M 14000.0f

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

/* 真实物块在近处的像素变化更快，按较高图像响应提前制动。
 * 横向未收敛时，进入最后 0.24 个画面高度后渐减前进速度。 */
#ifndef JETSON_AUTO_PICKUP_ACCEL_MPS2
#define JETSON_AUTO_PICKUP_ACCEL_MPS2 0.80f
#endif
#ifndef JETSON_AUTO_PICKUP_LATENCY_S
#define JETSON_AUTO_PICKUP_LATENCY_S 0.20f
#endif
#ifndef JETSON_AUTO_PICKUP_RESPONSE_X_Q_PER_M
#define JETSON_AUTO_PICKUP_RESPONSE_X_Q_PER_M 25000.0f
#endif
#ifndef JETSON_AUTO_PICKUP_RESPONSE_Y_Q_PER_M
#define JETSON_AUTO_PICKUP_RESPONSE_Y_Q_PER_M 40000.0f
#endif
#define JETSON_AUTO_PICKUP_LATERAL_FIRST_Q 2400.0f

/* 独立底盘联调的安全区定距速度。 */
#define JETSON_AUTO_DISTANCE_SPEED_MPS CHASSIS_RUN_SPEED_MPS

/* 左右横移共用速度和时长。盲推和直线回退分别计时。
 * 速度为目标值，计时包含加速。到期发零速，随后等待停稳。 */
#define JETSON_AUTO_LATERAL_SPEED_MPS 0.30f
#define JETSON_AUTO_LATERAL_DURATION_MS 700U
#define JETSON_AUTO_PUSH_SPEED_MPS 0.40f
#define JETSON_AUTO_PUSH_DURATION_MS 1200U
#define JETSON_AUTO_RETRACT_SPEED_MPS 0.40f
#define JETSON_AUTO_RETRACT_DURATION_MS 1000U
/* 直退停稳后按本趟实测横移量反平移，单位 m/s，只沿首次安全区朝向的横轴运动。 */
#define JETSON_AUTO_RECENTER_SPEED_MPS 0.225f

/* 视觉轴向、图像响应与滤波 */
/* 符号乘 (设定点 - 实测点)，横向取本次相机安装的实测方向。 */
#ifndef JETSON_AUTO_X_TO_LEFT
#if JETSON_AUTO_ENABLED
/* 正式相机实测右移会把物块推向画面左边，目标 x 偏小时应向左修正。 */
#define JETSON_AUTO_X_TO_LEFT 1
#else
#define JETSON_AUTO_X_TO_LEFT VISUAL_TRACKING_X_TO_LEFT
#endif
#endif
/* 整块安全区中心单独标定。现场左移时中心 x 增大，误差为正时向左修正。 */
#define JETSON_AUTO_LOCALIZE_X_TO_LEFT 1
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
/* 返航限时包含停稳复核。定时横移单独计时；直退后的反平移、停稳和整区视觉精调共用对准限时。 */
#define JETSON_AUTO_RETURN_TIMEOUT_MS 20000U
#define JETSON_AUTO_ALIGN_TIMEOUT_MS 10000U
/* 首次整区模式确认后，静止等待首张有效新图。收到后才开始上述对准限时。 */
#define JETSON_AUTO_INITIAL_CENTER_TIMEOUT_MS 10000U

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
/* 10 s 运动 + 500 ms 开爪捕获 + 保位交接、动作准备和最终停机各 3×200 ms
 * + 在途保位和末次反馈各 220 ms + 链路余量 450 ms = 13190 ms。启动从首个受理回执起算。 */
#define JETSON_AUTO_ACTION_TIMEOUT_MS 13500U

#endif
