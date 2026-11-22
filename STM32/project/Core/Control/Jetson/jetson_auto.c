#include "jetson_auto.h"

#if JETSON_AUTO_CAN_ENABLED
#include "chassis.h"
#include "tracked_visual_servo.h"
#if JETSON_AUTO_ENABLED
#include "point_plan.h"
#endif
#include "jetson_link.h"
#include "usb_link.h"
#include "FreeRTOS.h"
#include "task.h"
#include <math.h>

/* 流程与 Jetson 的 MissionCommand.srv 一致；只由原 test_task 推进。 */
typedef enum
{
    UNCONFIGURED, WAIT_STARTUP, WAIT_IMU, ARMING, APPROACH_TARGET, WAIT_GRASP_POINT_ACK, RESERVED_6,
    WAIT_DESTINATION, TURN_OUT, WAIT_TURN_ACK, APPROACH_SAFETY, SAFETY_STRAIGHT, WAIT_SAFETY_ACK, PUSHING,
    WAIT_RELEASE_POINT_ACK, RETRACTING, WAIT_DELIVERY_ACK, TURN_BACK, WAIT_SEARCH_ACK,
    RESERVED_19, RESERVED_20, FAULT, TEST_DONE,
    INITIAL_SEARCH_TURN, RETURN_TO_START, VERIFY_RETURN_POSE, LOCALIZE_START, WAIT_ENABLE,
    TIMED_LATERAL, RECENTER_LATERAL, VERIFY_RECENTER, WAIT_START_ALIGN, INITIAL_LOCALIZE
} Auto_Stage_t;

typedef struct
{
    float x, y;              /**< 到位点，归一化图像坐标。 */
    float tolerance_x, tolerance_y; /**< 各轴最大允许误差。 */
    uint8_t verify_pair;     /**< 发出零速后使用两个新中心核验安全区。 */
} Image_Window_t;

typedef enum { VISION_FOLLOW, VISION_VERIFY, VISION_REACQUIRE } Vision_Phase_t;

typedef struct
{
    uint32_t received_ms; /**< 最近一次该轴越界的 CAN 接收时刻，ms。 */
    int8_t side;          /**< 最近一次越界的方向 ±1，尚无样本为 0。 */
} Window_Evidence_t;

typedef struct
{
    Vision_Phase_t phase;
    AdaptiveKalman tracker;      /**< 图像定点位置和速度估计。 */
    Visual_Observation_t previous; /**< 上一帧积分或停车核验样本。 */
    Window_Evidence_t verify_outside[2]; /**< 视觉核验时 x、y 各自最近一次越界。 */
    Visual_Servo_t servo;        /**< 有限制动距离的图像速度规划。 */
    uint32_t evidence_ms;         /**< 最近一次有效新图像的接收时刻，ms。 */
#if JETSON_AUTO_ENABLED
    uint8_t settled;             /**< 已观察到停稳，后续源帧可用于复核。 */
#endif
} Vision_Approach_t;

typedef struct
{
    uint16_t id;             /**< 当前待发送或等待回复的上行类型号。 */
    uint8_t point;           /**< 类型 21 的到点类型，其余请求为 0。 */
    uint8_t submitted;       /**< 首次已进入发送邮箱。 */
    uint32_t first_rx;       /**< 首次发送前的接收序号屏障。 */
    uint32_t last_tx_ms;     /**< 最近一次入发送邮箱的 RTOS 毫秒。 */
    uint32_t request_id;     /**< 类型 19/21 的非零请求编号，重发保持不变。 */
    uint32_t accepted_ms;    /**< 启动查询首次受理后的完成期限起点。 */
    uint8_t accepted;
} Auto_Request_t;

static const Image_Window_t target_window = {
    JETSON_AUTO_TARGET_X, JETSON_AUTO_TARGET_Y,
    JETSON_AUTO_TARGET_TOL_X, JETSON_AUTO_TARGET_TOL_Y, 0U
};
#if JETSON_AUTO_CHASSIS_TEST
static const Image_Window_t safety_window = {
    JETSON_AUTO_SAFETY_X, JETSON_AUTO_SAFETY_Y,
    JETSON_AUTO_SAFETY_TOL_X, JETSON_AUTO_SAFETY_TOL_Y, 1U
};
#else
enum { SAFE_LEFT = 1U, SAFE_RIGHT = 2U };
static const Image_Window_t start_window = {
    JETSON_AUTO_LOCALIZE_X, JETSON_AUTO_LOCALIZE_Y,
    JETSON_AUTO_LOCALIZE_TOL_X, JETSON_AUTO_LOCALIZE_TOL_Y, 1U
};
static const Point_Plan_Config_t return_config = {
    .linear_limit_mps = JETSON_AUTO_RETURN_SPEED_MPS,
    .angular_limit_rad_s = CHASSIS_RUN_YAW_RATE_RAD_S,
    .linear_deceleration_mps2 = 0.30f,
    .angular_deceleration_rad_s2 = 0.80f,
    .latency_s = 0.12f,
    .position_tolerance_m = 0.01f,
    .yaw_tolerance_rad = 0.06981317f,
};
static Chassis_Odometry_t start_pose;
/* 首次起点的安全区朝向作为返航参考，避免打滑后的航向偏差传到下一趟。 */
static float return_reference_yaw_rad;
static Chassis_Odometry_t alignment_origin;
static uint32_t alignment_started_ms;
/* 首次安全区朝向的车体左轴在世界坐标中的方向，横移量保留左右符号。 */
static double left_axis_x, left_axis_y, outbound_lateral_m;
static Point_Plan_t return_plan;
static uint8_t delivery_side, return_corrected;
static uint32_t return_started_ms;
#endif
static uint8_t center_source_valid;
static uint32_t center_source_boundary;
static Auto_Stage_t stage;
static Auto_Request_t request;
static uint32_t stage_started_ms, center_boundary, center_boundary_ms, route_token;
/* 软件重新初始化不能复用旧搜索或动作请求号；整板复位需要与桥重新建立会话。 */
static uint32_t next_request_id;
static uint8_t navigating;
static uint32_t navigation_zero_ms;
#if JETSON_AUTO_ENABLED
static uint8_t timed_zero_requested;
#endif
#if JETSON_AUTO_CHASSIS_TEST
static uint32_t safety_lost_ms;
static uint8_t safety_lost;
#endif
static uint8_t link_bound;
static uint32_t link_generation;
static Vision_Approach_t vision;
#if JETSON_AUTO_CHASSIS_TEST
volatile uint8_t jetson_test_start;
static uint8_t test_started;
static uint32_t test_started_ms;
#endif

/** @brief 检查标定量是否在 (0,limit] 内。
 * @param[in] value,limit 同单位的标定值和上限。
 * @return 有限且在范围内返回 1。
 */
static uint8_t Positive_Within(float value, float limit)
{
    return value > 0.0f && value <= limit;
}

/** @brief 检查窗口中心与误差是否已配置为归一化坐标。
 * @param[in] window 非空的到位窗口，单位为原图宽高比例。
 * @return 四项均在 (0,1] 返回 1。
 */
static uint8_t Window_Configured(const Image_Window_t *window)
{
    return Positive_Within(window->x, 1.0f) && Positive_Within(window->y, 1.0f) &&
           Positive_Within(window->tolerance_x, 1.0f) && Positive_Within(window->tolerance_y, 1.0f);
}

/** @brief 检查当前模式使用的参数；安全区直行距离也须完成标定。
 * @return 所需参数已配置返回 1；待标定时禁止启动任务。
 */
static uint8_t Configuration_Ready(void)
{
#if JETSON_AUTO_CHASSIS_TEST
    if (!isfinite(JETSON_AUTO_SAFETY_STRAIGHT_M) ||
        JETSON_AUTO_SAFETY_STRAIGHT_M < CHASSIS_DISTANCE_MIN_M || !Window_Configured(&safety_window) ||
        !Positive_Within(JETSON_AUTO_DISTANCE_SPEED_MPS, CHASSIS_DISTANCE_MAX_SPEED_MPS))
        return 0U;
#else
    if (!isfinite(JETSON_AUTO_ALIGN_MAX_M) || JETSON_AUTO_ALIGN_MAX_M < CHASSIS_DISTANCE_MIN_M ||
        !Positive_Within(JETSON_AUTO_RETURN_SPEED_MPS, CHASSIS_DISTANCE_MAX_SPEED_MPS) ||
        !Positive_Within(JETSON_AUTO_RELOCALIZE_SPEED_MPS, CHASSIS_STRAIGHT_MAX_SPEED_MPS) ||
        !Positive_Within(JETSON_AUTO_LATERAL_SPEED_MPS, CHASSIS_STRAIGHT_MAX_SPEED_MPS) ||
        !Positive_Within(JETSON_AUTO_RECENTER_SPEED_MPS, CHASSIS_STRAIGHT_MAX_SPEED_MPS) ||
        (JETSON_AUTO_LOCALIZE_X_TO_LEFT != 1 && JETSON_AUTO_LOCALIZE_X_TO_LEFT != -1) ||
        !Positive_Within(JETSON_AUTO_PUSH_SPEED_MPS, CHASSIS_STRAIGHT_MAX_SPEED_MPS) ||
        !Positive_Within(JETSON_AUTO_RETRACT_SPEED_MPS, CHASSIS_STRAIGHT_MAX_SPEED_MPS) ||
        JETSON_AUTO_LATERAL_DURATION_MS == 0U || JETSON_AUTO_PUSH_DURATION_MS == 0U ||
        JETSON_AUTO_RETRACT_DURATION_MS == 0U || !Window_Configured(&start_window) ||
        !Positive_Within(JETSON_AUTO_RETURN_TURN_DEG, 360.0f))
        return 0U;
#endif
    return Window_Configured(&target_window) &&
           (JETSON_AUTO_X_TO_LEFT == 1 || JETSON_AUTO_X_TO_LEFT == -1) &&
           (JETSON_AUTO_Y_TO_FORWARD == 1 || JETSON_AUTO_Y_TO_FORWARD == -1) &&
           Positive_Within(JETSON_AUTO_APPROACH_SPEED_MPS, CHASSIS_STRAIGHT_MAX_SPEED_MPS) &&
           isfinite(JETSON_AUTO_APPROACH_SIGMA_X_Q) && JETSON_AUTO_APPROACH_SIGMA_X_Q > 0.0f &&
           isfinite(JETSON_AUTO_APPROACH_SIGMA_Y_Q) && JETSON_AUTO_APPROACH_SIGMA_Y_Q > 0.0f &&
           isfinite(JETSON_AUTO_APPROACH_SIGMA_ACCEL_Q_S2) && JETSON_AUTO_APPROACH_SIGMA_ACCEL_Q_S2 > 0.0f &&
           Positive_Within(JETSON_AUTO_APPROACH_ACCEL_MPS2, CHASSIS_NAV_ACCELERATION_MPS2) &&
           Positive_Within(JETSON_AUTO_APPROACH_DECEL_MPS2, CHASSIS_STRAIGHT_ACCELERATION_MPS2) &&
           Positive_Within(JETSON_AUTO_APPROACH_LATENCY_S, 0.25f) &&
           Positive_Within(JETSON_AUTO_RESPONSE_X_Q_PER_M, 1000000.0f) &&
           Positive_Within(JETSON_AUTO_RESPONSE_Y_Q_PER_M, 1000000.0f) &&
           Positive_Within(JETSON_AUTO_OUT_TURN_DEG, 360.0f);
}

/** @brief 切换搬运阶段并结束旧握手等待。
 * @param[in] next 下一阶段。
 * @param[in] now_ms 当前 RTOS 毫秒。
 */
static void Enter(Auto_Stage_t next, uint32_t now_ms)
{
    if (request.submitted && request.id != 0U && request.id != JETSON_CAN_ARRIVAL_ID)
        Jetson_CAN_Expect_Command_Reply(0U, 0U);
    if (request.id == JETSON_CAN_POINT_REACHED_ID)
        Jetson_CAN_Expect_Action_Result(0U, 0U);
    stage = next;
    stage_started_ms = now_ms;
    request.id = 0U;
}

/** @brief 停车并停在当前阶段，避免通信故障时自动重复运动；夹爪由 Jetson 负责。
 * @param[in] now_ms 当前 RTOS 毫秒，作为制动起点。
 */
static void Halt(uint32_t now_ms)
{
    Chassis_Stop();
    Enter(FAULT, now_ms);
}

/** @brief 用 USB 会话与既有中心周期流检查通信，正常无目标仍允许搜索。
 * @param[in] data 当前接收快照；只有格式合法的类型 3/4 更新周期流时间。
 * @param[in] link 与接收快照在同一临界区读取的 USB 会话状态。
 * @param[in] now_ms 快照之后读取的 RTOS 毫秒，允许 uint32_t 回绕。
 * @return 当前会话在线且周期流未超时返回 1；首次连通前只等待，之后失联锁存故障。
 */
static uint8_t Link_Ready(const Jetson_CAN_Data_t *data, const UsbLink_State_t *link, uint32_t now_ms)
{
    const uint8_t fresh = link->connected && data->periodic_received &&
        (uint32_t)(now_ms - data->periodic_received_ms) < JETSON_AUTO_LINK_TIMEOUT_MS;
    if (!link_bound)
    {
        if (!fresh)
            return 0U;
        link_generation = link->generation;
        link_bound = 1U;
    }
    else if (!fresh || link->generation != link_generation)
    {
        /* USB 即使在两拍之间断开又恢复，也不能继续旧动作或消费旧回执。 */
        Halt(now_ms);
        return 0U;
    }
    return 1U;
}

/** @brief 比较接收顺序，支持 uint32_t 自然回绕。
 * @param[in] sequence,boundary 帧序号和阶段接收边界。
 * @return 帧在边界之后返回 1，要求间距小于半个序号周期。
 */
static uint8_t Received_After(uint32_t sequence, uint32_t boundary)
{
    const uint32_t delta = sequence - boundary;
    return delta != 0U && delta <= INT32_MAX;
}

/** @brief 开始新的视觉阶段，屏蔽上阶段已缓存的坐标。
 * @param[in] next 物资或安全区接近阶段。
 * @param[in] data 当前非空接收快照。
 * @param[in] now_ms 当前 RTOS 毫秒。
 */
static void Begin_Approach(Auto_Stage_t next, const Jetson_CAN_Data_t *data, uint32_t now_ms)
{
    navigating = 0U;
    navigation_zero_ms = now_ms;
    vision = (Vision_Approach_t){0};
    center_boundary = data->rx_sequence;
    center_boundary_ms = now_ms;
#if JETSON_AUTO_CHASSIS_TEST
    safety_lost = 0U;
#endif
    const Visual_Observation_t *previous = next == APPROACH_TARGET ? &data->target : &data->safety;
    center_source_valid = previous->valid;
    center_source_boundary = previous->source_frame;
    Enter(next, now_ms);
}

/** @brief 正常减速只请求一次，后续轮询保持当前斜坡。 */
static void Hold_Navigation(uint32_t now_ms)
{
    if (navigating)
    {
        Chassis_Hold();
        navigating = 0U;
        navigation_zero_ms = now_ms;
    }
}

/** @brief 更新连续速度，底盘负责检查动作准入。
 * @param[in] vx,vy 前、左为正的车体线速度，m/s。
 * @param[in] wz 逆时针为正的车体角速度，rad/s。
 * @param[in] now_ms 当前 RTOS 毫秒。
 */
static void Navigate(float vx, float vy, float wz, uint32_t now_ms)
{
    if (vx == 0.0f && vy == 0.0f && wz == 0.0f)
    {
        /* 窗外换向的过零样本仍属于跟踪过程，下一帧可以直接反向。 */
        Chassis_Hold();
        navigating = 1U;
        return;
    }
    if (!Chassis_Set_Navigation_Velocity(vx, vy, wz, 0U))
        Halt(now_ms);
    else
        navigating = 1U;
}

/** @brief 清除旧坐标速度，按当前视觉阶段搜索或停车。
 * @param[in] now_ms 当前 RTOS 毫秒。
 */
static void Approach_Lost_Center(uint32_t now_ms, uint8_t holding)
{
    const uint8_t slowing = navigating || holding;
    Hold_Navigation(now_ms);
    vision = (Vision_Approach_t){.phase = slowing ? VISION_REACQUIRE : VISION_FOLLOW};
#if JETSON_AUTO_CHASSIS_TEST
    /* 单趟诊断有固定失去中心时限。正式搬运的安全区对准受独立总时限约束。 */
    if (!safety_lost)
    {
        safety_lost = 1U;
        safety_lost_ms = now_ms;
        return;
    }
    if ((uint32_t)(now_ms - safety_lost_ms) >= JETSON_AUTO_BRAKE_TIMEOUT_MS)
        Halt(now_ms);
#endif
}

/** @brief 用合法视觉中心更新卡尔曼估计，取得本次积分的实际间隔。
 * @param[in] center 非空的合法中心，接收时间使用 RTOS 毫秒。
 * @return 当前积分时长，单位秒。首包或长间隔返回 0。
 * @note 相同源帧编号的周期重发不重复参与滤波和积分。
 */
static float Update_Image(const Visual_Observation_t *center)
{
    /* 按源帧间隔更新滤波，避免通信调度抖动变成虚假的图像速度。 */
    uint32_t elapsed = vision.previous.valid ?
        (center->source_frame - vision.previous.source_frame) & 0xffffffU : 0U;
    if (!vision.tracker.initialized || elapsed >= JETSON_AUTO_CENTER_TIMEOUT_MS)
    {
        AdaptiveKalman_Init(&vision.tracker, JETSON_AUTO_APPROACH_SIGMA_X_Q,
                           JETSON_AUTO_APPROACH_SIGMA_Y_Q, JETSON_AUTO_APPROACH_SIGMA_ACCEL_Q_S2);
        vision.servo = (Visual_Servo_t){0};
        elapsed = 0U;
    }
    const uint32_t filter_time = vision.tracker.initialized ?
        vision.tracker.timestamp_ms + elapsed : center->received_ms;
    AdaptiveKalman_Update(&vision.tracker, center->x, center->y, filter_time);
    vision.previous = *center;
    return elapsed * 0.001f;
}

/** @brief 判断原始定点误差位于到位窗哪一侧。
 * @param[in] error,tolerance 定点误差和该轴容差，单位 q。
 * @return 窗内为 0，窗外为误差方向 ±1。
 */
static int8_t Window_Side(float error, float tolerance)
{
    return (int8_t)((error > tolerance) - (error < -tolerance));
}

/** @brief 更新单轴越界样本，核对最近两次是否在有效期内同向越界。
 * @param[in,out] evidence 非空的该轴最近越界记录。
 * @param[in] side 当前样本窗内为 0，窗外为误差方向 ±1。
 * @param[in] received_ms,now_ms 本帧接收时刻和当前 RTOS 时刻，ms。
 * @return 本帧和该轴最近窗外样本同向且有效返回 1。窗内样本不覆盖记录。
 */
static uint8_t Repeated_Outside(Window_Evidence_t *evidence, int8_t side,
                                uint32_t received_ms, uint32_t now_ms)
{
    if (side == 0)
        return 0U;
    const uint8_t repeated = side == evidence->side &&
        (uint32_t)(now_ms - evidence->received_ms) < JETSON_AUTO_CENTER_TIMEOUT_MS;
    *evidence = (Window_Evidence_t){.received_ms = received_ms, .side = side};
    return repeated;
}

/** @brief 按图像误差跟随，入窗时下发零速并核验后续图像。
 * @param[in] center,window 非空的接收中心和归一化到位窗口。
 * @param[in] now_ms 当前 RTOS 毫秒。
 * @return 发出零速后取得新源帧并核验到位时返回 1。
 */
static uint8_t Approach(const Visual_Observation_t *center, const Image_Window_t *window,
                        uint32_t now_ms)
{
    /* 邮箱暂忙可重试；持续未交付零目标时，即使相机还在发新图也须故障停车。 */
#if JETSON_AUTO_ENABLED
    const uint8_t holding = !navigating && !Chassis_Is_Stopped();
#else
    const uint8_t holding = !navigating && !Chassis_Motion_Ready();
#endif
    if (holding && (uint32_t)(now_ms - navigation_zero_ms) >= JETSON_AUTO_HOLD_TIMEOUT_MS)
    {
        Halt(now_ms);
        return 0U;
    }
    if (holding && vision.phase == VISION_FOLLOW)
        return 0U;
    /* USB 字节可能早已入队，解析顺序不能替代本机实际接收时间。相同 tick 继续按序号和源帧判断。 */
    const uint8_t received_after_boundary =
        (uint32_t)(center->received_ms - center_boundary_ms) <= INT32_MAX;
    const uint8_t fresh = received_after_boundary &&
        Visual_Observation_Is_New(center, center_boundary, center_source_valid,
            center_source_boundary, now_ms, JETSON_AUTO_CENTER_TIMEOUT_MS);
#if JETSON_AUTO_ENABLED
    if (vision.phase == VISION_VERIFY)
    {
        if (holding)
            vision.settled = 0U;
        else if (!vision.settled)
        {
            /* 发现停稳时的快照可能来自减速过程，下一张新图才能参与复核。 */
            vision.settled = 1U;
            center_boundary_ms = now_ms;
            vision.previous.valid = 0U;
            vision.verify_outside[0] = (Window_Evidence_t){0};
            vision.verify_outside[1] = (Window_Evidence_t){0};
            if (fresh)
            {
                center_source_valid = 1U;
                center_source_boundary = center->source_frame;
                vision.evidence_ms = center->received_ms;
                return 0U;
            }
        }
    }
#endif
    if (!fresh)
    {
        /* 保持上一帧的速度或零目标，直到新的图像到达或已有证据过期。 */
        if (center->valid && (vision.previous.valid || vision.phase == VISION_VERIFY) &&
            (uint32_t)(now_ms - vision.evidence_ms) < JETSON_AUTO_CENTER_TIMEOUT_MS)
            return 0U;
        Approach_Lost_Center(now_ms, holding);
        return 0U;
    }
#if JETSON_AUTO_CHASSIS_TEST
    safety_lost = 0U;
#endif
    center_source_valid = 1U;
    center_source_boundary = center->source_frame;
    vision.evidence_ms = center->received_ms;

    /* 在 CAN 定点单位比较窗口，确保标定的边界点按等号接收。 */
    const float goal_x = roundf(window->x * 10000.0f);
    const float goal_y = roundf(window->y * 10000.0f);
    const float x_tol = roundf(window->tolerance_x * 10000.0f);
    const float y_tol = roundf(window->tolerance_y * 10000.0f);
    const float raw_x_error = goal_x - center->x;
    const float raw_y_error = goal_y - center->y;
    const int8_t x_side = Window_Side(raw_x_error, x_tol);
    const int8_t y_side = Window_Side(raw_y_error, y_tol);
    const uint8_t inside = x_side == 0 && y_side == 0;

    if (vision.phase == VISION_VERIFY)
    {
        /* 夹取目标已入窗后，允许复核时的轻微像素抖动；安全区仍用标定窗口和双帧。 */
        const uint8_t verified_inside = inside ||
            (!window->verify_pair && fabsf(raw_x_error) <= 1.2f * x_tol && fabsf(raw_y_error) <= 1.2f * y_tol);
#if JETSON_AUTO_ENABLED
        /* 整区一旦进入减速，等稳态新帧再决定补正；车身摆动中的窗外帧不反向驱动。 */
        if (holding && window == &start_window)
            return 0U;
#endif
        if (holding && verified_inside)
        {
            /* 物资入窗的滑行帧不能确认到位；窗外新目标可以重定向斜坡。 */
            vision.previous.valid = 0U;
            return 0U;
        }
        if (window->verify_pair)
        {
            const uint8_t arrived = inside && vision.previous.valid &&
                (uint32_t)(now_ms - vision.previous.received_ms) < JETSON_AUTO_CENTER_TIMEOUT_MS &&
                Window_Side(goal_x - vision.previous.x, x_tol) == 0 &&
                Window_Side(goal_y - vision.previous.y, y_tol) == 0;
            /* 分轴保留证据；一轴回到窗内或另一轴越界都不覆盖该轴的最近越界。 */
            const uint8_t adjust_x = Repeated_Outside(&vision.verify_outside[0], x_side,
                                                     center->received_ms, now_ms);
            const uint8_t adjust_y = Repeated_Outside(&vision.verify_outside[1], y_side,
                                                     center->received_ms, now_ms);
            vision.previous = *center;
            if (!arrived && !adjust_x && !adjust_y)
                return 0U;
            /* 再对准时以当前点初始化滤波，停车核验样本不参与速度和积分。 */
            vision.previous.valid = 0U;
        }
        if (verified_inside)
            return 1U;
        vision.phase = VISION_FOLLOW;
    }

    /* 安全区边缘有抖动时先保留平滑微调。深入半窗或两帧连续入窗再制动。 */
    if (inside &&
        (!window->verify_pair ||
         (fabsf(raw_x_error) <= 0.5f * x_tol && fabsf(raw_y_error) <= 0.5f * y_tol) ||
         (vision.previous.valid &&
          (uint32_t)(now_ms - vision.previous.received_ms) < JETSON_AUTO_CENTER_TIMEOUT_MS &&
          Window_Side(goal_x - vision.previous.x, x_tol) == 0 &&
          Window_Side(goal_y - vision.previous.y, y_tol) == 0)))
    {
        Hold_Navigation(now_ms);
        vision = (Vision_Approach_t){.phase = VISION_VERIFY, .evidence_ms = center->received_ms};
#if JETSON_AUTO_ENABLED
        vision.settled = Chassis_Is_Stopped();
        if (vision.settled)
            center_boundary_ms = now_ms;
#endif
        return 0U;
    }

    const float seconds = Update_Image(center);
    float x_to_left = JETSON_AUTO_X_TO_LEFT;
    Visual_Servo_Config_t config = {
        .max_speed_mps = JETSON_AUTO_APPROACH_SPEED_MPS,
        .acceleration_mps2 = JETSON_AUTO_APPROACH_ACCEL_MPS2,
        .deceleration_mps2 = JETSON_AUTO_APPROACH_DECEL_MPS2,
        .latency_s = JETSON_AUTO_APPROACH_LATENCY_S,
        .response_q_per_m = {JETSON_AUTO_RESPONSE_X_Q_PER_M, JETSON_AUTO_RESPONSE_Y_Q_PER_M},
    };
#if JETSON_AUTO_ENABLED
    if (window == &start_window)
    {
        config.max_speed_mps = JETSON_AUTO_RELOCALIZE_SPEED_MPS;
        config.acceleration_mps2 = JETSON_AUTO_LOCALIZE_ACCEL_MPS2;
        config.latency_s = JETSON_AUTO_LOCALIZE_LATENCY_S;
        config.response_q_per_m[0] = JETSON_AUTO_LOCALIZE_RESPONSE_X_Q_PER_M;
        config.response_q_per_m[1] = JETSON_AUTO_LOCALIZE_RESPONSE_Y_Q_PER_M;
        x_to_left = JETSON_AUTO_LOCALIZE_X_TO_LEFT;
    }
    else
    {
        config.acceleration_mps2 = JETSON_AUTO_PICKUP_ACCEL_MPS2;
        config.latency_s = JETSON_AUTO_PICKUP_LATENCY_S;
        config.response_q_per_m[0] = JETSON_AUTO_PICKUP_RESPONSE_X_Q_PER_M;
        config.response_q_per_m[1] = JETSON_AUTO_PICKUP_RESPONSE_Y_Q_PER_M;
    }
#endif
    const float raw[2] = {center->x, center->y};
    const float goal[2] = {goal_x, goal_y}, tolerance[2] = {x_tol, y_tol};
    Tracked_Visual_Servo_Update(&vision.servo, &config, &vision.tracker, raw, goal, tolerance, seconds);
#if JETSON_AUTO_ENABLED
    if (window == &target_window && vision.servo.speed[1] > 0.0f &&
        raw_y_error <= JETSON_AUTO_PICKUP_LATERAL_FIRST_Q)
    {
        /* 横向误差三倍容差时不再前进，进入容差后恢复当前允许速度。 */
        const float forward_scale = (3.0f * x_tol - fabsf(raw_x_error)) / (2.0f * x_tol);
        vision.servo.speed[1] *= fminf(1.0f, fmaxf(0.0f, forward_scale));
    }
#endif
    Navigate(vision.servo.speed[1] * JETSON_AUTO_Y_TO_FORWARD,
             vision.servo.speed[0] * x_to_left, 0.0f, now_ms);
    return 0U;
}

#if JETSON_AUTO_CHASSIS_TEST
/** @brief 启动单次底盘联调的定距。
 * @param[in] direction 底盘前进或后退方向。
 * @param[in] meters 已标定的正距离，m。
 * @param[in] next 定距阶段。
 * @param[in] now_ms 当前 RTOS 毫秒。
 */
static void Start_Distance(Chassis_Translation_Direction_t direction, float meters,
                           Auto_Stage_t next, uint32_t now_ms)
{
    if (Chassis_Distance_Start(direction, meters, JETSON_AUTO_DISTANCE_SPEED_MPS))
        Enter(next, now_ms);
    else
        Halt(now_ms);
}
#endif

#if JETSON_AUTO_ENABLED
/* 三段定时平移共用底盘连续速度斜坡和锁航向控制。 */
static void Start_Timed_Move(float vx, float vy, Auto_Stage_t next, uint32_t now_ms)
{
    if (!Chassis_Set_Velocity(vx, vy, 0.0f))
    {
        Halt(now_ms);
        return;
    }
    timed_zero_requested = 0U;
    Enter(next, now_ms);
}

/* 到时发零速，三轮停稳后才能进入握手。 */
static uint8_t Timed_Move_Complete(uint32_t duration_ms, uint32_t now_ms)
{
    if (!timed_zero_requested)
    {
        if ((uint32_t)(now_ms - stage_started_ms) < duration_ms)
            return 0U;
        Chassis_Hold();
        timed_zero_requested = 1U;
        stage_started_ms = now_ms;
    }
    if (Chassis_Is_Stopped())
        return 1U;
    if ((uint32_t)(now_ms - stage_started_ms) >= JETSON_AUTO_HOLD_TIMEOUT_MS)
        Halt(now_ms);
    return 0U;
}
#endif

/** @brief 启动一次定角转向。
 * @param[in] direction +1 左转，-1 右转。
 * @param[in] angle_deg 相对转角，范围 (0,360] 度。
 * @param[in] next 转向阶段。
 * @param[in] now_ms 当前 RTOS 毫秒。
 */
static void Start_Turn(int8_t direction, float angle_deg, Auto_Stage_t next, uint32_t now_ms)
{
    if (Chassis_Turn_Start(direction, angle_deg))
        Enter(next, now_ms);
    else
        Halt(now_ms);
}

#if JETSON_AUTO_ENABLED
/* 定角控制本身限制运动时长，再给零目标和三轮停稳反馈留出有限时间。 */
static uint8_t Search_Turn_Stopped(uint32_t now_ms)
{
    if (Chassis_Is_Stopped())
        return 1U;
    if ((uint32_t)(now_ms - stage_started_ms) >= CHASSIS_TURN_TIMEOUT_MS + JETSON_AUTO_HOLD_TIMEOUT_MS)
        Halt(now_ms);
    return 0U;
}

/* 定时横移从自身起点限距；回退后的粗复位和视觉精调从同一停点限距、计时。 */
static uint8_t Alignment_Pose(Chassis_Odometry_t *pose, uint32_t now_ms)
{
    if ((uint32_t)(now_ms - alignment_started_ms) >= JETSON_AUTO_ALIGN_TIMEOUT_MS ||
        !Chassis_Get_Odometry(pose) ||
        hypot(pose->x_m - alignment_origin.x_m, pose->y_m - alignment_origin.y_m) > JETSON_AUTO_ALIGN_MAX_M)
    {
        Halt(now_ms);
        return 0U;
    }
    return 1U;
}

static double Along_Left(double dx, double dy)
{
    return dx * left_axis_x + dy * left_axis_y;
}

/* 返航回到已保存起点；坡前回退之后只撤销实际横移量，保留直退留出的纵向净空。 */
static void Start_Return(Auto_Stage_t next, uint32_t now_ms)
{
    Chassis_Odometry_t pose;
    if (!Chassis_Get_Odometry(&pose))
    {
        Halt(now_ms);
        return;
    }
    const double from[3] = {pose.x_m, pose.y_m, pose.yaw_rad};
    double goal[3] = {start_pose.x_m, start_pose.y_m, return_reference_yaw_rad};
    if (next == RECENTER_LATERAL)
    {
        alignment_origin = pose;
        alignment_started_ms = now_ms;
        goal[0] = pose.x_m - outbound_lateral_m * left_axis_x;
        goal[1] = pose.y_m - outbound_lateral_m * left_axis_y;
    }
    if (!Point_Plan_Init(&return_plan, from, goal))
    {
        Halt(now_ms);
        return;
    }
    return_started_ms = now_ms;
    return_corrected = 0U;
    Enter(next, now_ms);
}

static uint8_t Read_Return_Pose(Chassis_Odometry_t *pose, uint8_t recenter, uint32_t now_ms)
{
    if (recenter)
        return Alignment_Pose(pose, now_ms);
    if ((uint32_t)(now_ms - return_started_ms) >= JETSON_AUTO_RETURN_TIMEOUT_MS ||
        !Chassis_Get_Odometry(pose))
    {
        Halt(now_ms);
        return 0U;
    }
    return 1U;
}

static void Service_Return(uint32_t now_ms)
{
    const uint8_t recenter = stage == RECENTER_LATERAL;
    Chassis_Odometry_t pose;
    if (!Read_Return_Pose(&pose, recenter, now_ms))
        return;
    double current[3] = {pose.x_m, pose.y_m, pose.yaw_rad};
    const float measured[3] = {pose.vx_mps, pose.vy_mps, pose.yaw_rate_rad_s};
    Point_Plan_Config_t config = return_config;
    if (recenter)
    {
        /* 打滑后的纵向里程不参与到位判定，保持原安全区航向。 */
        const double lateral_error = Along_Left(pose.x_m - return_plan.goal[0], pose.y_m - return_plan.goal[1]);
        current[0] = return_plan.goal[0] + lateral_error * left_axis_x;
        current[1] = return_plan.goal[1] + lateral_error * left_axis_y;
        config.linear_limit_mps = JETSON_AUTO_RECENTER_SPEED_MPS;
    }
    Point_Plan_Output_t motion;
    if (!Point_Plan_Step(&return_plan, &config, current, measured, &motion))
    {
        Halt(now_ms);
        return;
    }
    if (motion.arrived)
    {
        Chassis_Hold();
        Enter(recenter ? VERIFY_RECENTER : VERIFY_RETURN_POSE, now_ms);
        return;
    }
    if (recenter)
    {
        /* 规划器的横向阻尼也可能含坡向分量，输出只保留初始车体左轴的速度。 */
        const double lateral_speed = Along_Left(motion.vx_mps, motion.vy_mps);
        motion.vx_mps = (float)(lateral_speed * left_axis_x);
        motion.vy_mps = (float)(lateral_speed * left_axis_y);
    }
    if (!Chassis_Set_Navigation_Velocity(motion.vx_mps, motion.vy_mps, motion.wz_rad_s, 1U))
        Halt(now_ms);
}
#endif

static uint8_t Request_Uses_Identity(uint16_t id)
{
    return id == JETSON_CAN_POINT_REACHED_ID || id == JETSON_CAN_START_SEARCH_ID ||
           id == JETSON_CAN_START_ALIGN_ID;
}

/** @brief 打开一个新握手等待；每次只允许重报同一个已完成事件。
 * @param[in] id 本阶段的上行类型号。
 * @param[in] next 回执等待阶段。
 * @param[in] now_ms 当前 RTOS 毫秒，作为总期限起点。
 */
static void Begin_Request(uint16_t id, Auto_Stage_t next, uint32_t now_ms)
{
    Enter(next, now_ms);
    request = (Auto_Request_t){.id = id};
    if (Request_Uses_Identity(id))
    {
        if (++next_request_id == 0U)
            ++next_request_id;
        request.request_id = next_request_id;
    }
}

#if JETSON_AUTO_ENABLED
static void Verify_Return(const Jetson_CAN_Data_t *data, uint32_t now_ms)
{
    const uint8_t recenter = stage == VERIFY_RECENTER;
    Chassis_Odometry_t pose;
    if (!Read_Return_Pose(&pose, recenter, now_ms))
        return;
    if ((uint32_t)(now_ms - stage_started_ms) >= JETSON_AUTO_HOLD_TIMEOUT_MS)
    {
        Halt(now_ms);
        return;
    }
    if (!Chassis_Is_Stopped())
        return;

    const double dx = pose.x_m - return_plan.goal[0], dy = pose.y_m - return_plan.goal[1];
    const double error_m = recenter ? fabs(Along_Left(dx, dy)) : hypot(dx, dy);
    const float error_rad = fabsf(remainderf(pose.yaw_rad - return_reference_yaw_rad, 6.28318530718f));
    if (error_m > CHASSIS_RETURN_CORRECTION_MAX_M || error_rad > CHASSIS_RETURN_CORRECTION_MAX_RAD)
        Halt(now_ms);
    else if (error_m <= CHASSIS_RETURN_POSITION_TOLERANCE_M && error_rad <= CHASSIS_RETURN_YAW_TOLERANCE_RAD)
    {
        if (recenter)
            Begin_Approach(LOCALIZE_START, data, now_ms);
        else
            Begin_Request(JETSON_CAN_TURN_COMPLETED_ID, WAIT_TURN_ACK, now_ms);
    }
    else if (return_corrected)
        Halt(now_ms);
    else
    {
        return_corrected = 1U;
        Enter(recenter ? RECENTER_LATERAL : RETURN_TO_START, now_ms);
    }
}
#endif

/** @brief 创建类型 21 动作请求，命令 5 只受理，类型 9 才报告完成。
 * @param[in] point 启动查询、抓取、释放或拒绝后张开。
 * @param[in] next 回执等待阶段。
 * @param[in] now_ms 当前 RTOS 毫秒，作为总期限起点。
 */
static void Begin_Point_Request(uint8_t point, Auto_Stage_t next, uint32_t now_ms)
{
    Begin_Request(JETSON_CAN_POINT_REACHED_ID, next, now_ms);
    request.point = point;
}

/** @brief 按首次入邮箱时的接收序号边界判断回执是否属于本次请求。
 * @param[in] sequence 回执的 CAN 接收序号。
 * @return 请求已提交且回执在首次提交之后到达返回 1；期限由消费回执的上层先行检查。
 */
static uint8_t Reply_In_Request(uint32_t sequence)
{
    return request.submitted && Received_After(sequence, request.first_rx);
}

/** @brief 核对发送后的命令回执；命令 3/5 回显请求号，其余沿用路线身份。
 * @param[in] reply 当前非空回执快照。
 * @param[in] command 期望命令，1 至 5。
 * @return 本次请求得到匹配的成功回执时返回 1。
 */
static uint8_t Reply_Accepted(const Jetson_CAN_Command_Reply_t *reply, uint8_t command)
{
    return Reply_In_Request(reply->sequence) &&
           reply->valid && reply->accepted && reply->command == command &&
           reply->token == (Request_Uses_Identity(request.id) ? request.request_id : route_token);
}

/** @brief 启动只在首次受理后计时；其他握手从入阶段起算，不因重发延长。 */
static uint8_t Request_Timed_Out(uint32_t now_ms)
{
    const uint8_t action = request.id == JETSON_CAN_POINT_REACHED_ID;
    const uint8_t startup = action && request.point == JETSON_CAN_POINT_STARTUP;
    const uint32_t started_ms = startup ? request.accepted_ms : stage_started_ms;
    const uint32_t timeout_ms = action ? JETSON_AUTO_ACTION_TIMEOUT_MS : JETSON_AUTO_REQUEST_TIMEOUT_MS;
    return (!startup || request.accepted) && (uint32_t)(now_ms - started_ms) >= timeout_ms;
}

/** @brief 只组装当前请求及其回执匹配参数，不提交发送、不改变请求状态。
 * @param[out] payload 至少可写 8 字节的载荷缓冲。
 * @param[out] argument 路线 token、请求号或到达通知的请求模式。
 * @return 编码长度；不支持的请求沿用编码器的零长度结果。
 */
static uint8_t Encode_Request(uint8_t payload[8], uint32_t *argument)
{
    const uint8_t action = request.id == JETSON_CAN_POINT_REACHED_ID;
    *argument = route_token;
    if (Request_Uses_Identity(request.id))
        *argument = request.request_id;
    else if (request.id == JETSON_CAN_ARRIVAL_ID)
        *argument = JETSON_AUTO_ENABLED ? JETSON_CAN_SELECTED_ROUTE : 0U;
    return action ? Jetson_CAN_Encode_Action_Request(request.point, *argument, payload) :
                    Jetson_CAN_Encode_Uplink(request.id, *argument, payload);
}

/** @brief 非阻塞提交 USB 握手；调用方已先检查本次消费期限，重试不延长期限。 */
static void Service_Request(uint32_t now_ms)
{
    if (request.submitted && (uint32_t)(now_ms - request.last_tx_ms) < JETSON_AUTO_REQUEST_RETRY_MS)
        return;

    const uint8_t action = request.id == JETSON_CAN_POINT_REACHED_ID;
    uint8_t payload[8];
    uint32_t argument;
    const uint8_t length = Encode_Request(payload, &argument);
    if (length == 0U)
    {
        Halt(now_ms);
        return;
    }
    /* 在提交 USB 前取接收边界，期间不让 USB 任务改写快照。 */
    Jetson_CAN_Data_t before_send;
    taskENTER_CRITICAL();
    Jetson_CAN_Get_Data(&before_send);
    const int result = Jetson_Link_Send((uint8_t)request.id, payload, length);
    const uint32_t sent_ms = (uint32_t)xTaskGetTickCount();
    if (result == 0 && !request.submitted && request.id != JETSON_CAN_ARRIVAL_ID)
    {
        Jetson_CAN_Expect_Command_Reply(request.id, argument);
        if (action)
            Jetson_CAN_Expect_Action_Result(request.point, request.request_id);
    }
    taskEXIT_CRITICAL();
    if (result == 0)
    {
        if (!request.submitted)
        {
            request.first_rx = before_send.rx_sequence;
            request.submitted = 1U;
        }
        request.last_tx_ms = sent_ms;
    }
    else if (result != 1)
        Halt(now_ms);
}

/** @brief 动作受理与终态可乱序；未完成持续查询，失败立即停车。 */
static uint8_t Action_Completed(const Jetson_CAN_Data_t *data, uint32_t now_ms)
{
    if (!request.accepted && Reply_Accepted(&data->command_reply, JETSON_CAN_POINT_REPLY_COMMAND))
    {
        request.accepted = 1U;
        request.accepted_ms = now_ms;
    }
    /* 动作完成期限包含等号；到期同拍的迟到终态不得放行。 */
    if (Request_Timed_Out(now_ms))
    {
        Halt(now_ms);
        return 0U;
    }
    const Jetson_CAN_Action_Result_t *result = &data->action_result;
    if (result->valid && result->kind == request.point && result->request_id == request.request_id &&
        Reply_In_Request(result->sequence))
    {
        if (!result->success)
        {
            Halt(now_ms);
            return 0U;
        }
        if (request.accepted && (stage == WAIT_STARTUP || Chassis_Motion_Ready()))
            return 1U;
    }
    Service_Request(now_ms);
    return 0U;
}

/** @brief 等待到达受理、路由登记结果和对应的目的区授权。
 * @param[in] data 当前非空接收快照。
 * @param[in] now_ms 当前 RTOS 毫秒。
 */
static void Wait_Destination(const Jetson_CAN_Data_t *data, uint32_t now_ms)
{
    const Jetson_CAN_Arrival_Ack_t *ack = &data->arrival_ack;
    const Jetson_CAN_ROI_Reply_t *route = &data->roi_reply;
    const Jetson_CAN_Turn_Request_t *turn = &data->turn_to_safety;
    if (Reply_In_Request(ack->sequence) && ack->valid)
    {
        const uint8_t current_route = Reply_In_Request(route->sequence) && route->valid;
        if (!ack->accepted || (current_route && !route->passed))
        {
            Halt(now_ms);
            return;
        }
        if (current_route && route->passed && turn->valid && turn->destination == route->destination &&
            (JETSON_AUTO_CHASSIS_TEST || turn->token == route_token) && Reply_In_Request(turn->sequence))
        {
            route_token = turn->token;
#if JETSON_AUTO_CHASSIS_TEST
            Start_Turn(turn->destination == 1U ? 1 : -1, JETSON_AUTO_OUT_TURN_DEG, TURN_OUT, now_ms);
#else
            delivery_side = turn->destination;
            Start_Return(RETURN_TO_START, now_ms);
#endif
            return;
        }
    }
    Service_Request(now_ms);
}

/** @brief 推进机械动作。定距和转向由底盘闭环到位后交接。
 * @param[in] data 当前非空接收快照。
 * @param[in] now_ms 当前 RTOS 毫秒。
 * @note 夹爪动作须成功终态；普通握手消费匹配回执。
 */
static void Service_Stage(const Jetson_CAN_Data_t *data, uint32_t now_ms)
{
    /* 普通握手以控制拍消费时刻截止；即使回执先到，到期后也不能放行动作。 */
    if (request.id != 0U && request.id != JETSON_CAN_POINT_REACHED_ID && Request_Timed_Out(now_ms))
    {
        Halt(now_ms);
        return;
    }
    const Jetson_CAN_Command_Reply_t *reply = &data->command_reply;
    switch (stage)
    {
#if JETSON_AUTO_ENABLED
    case WAIT_START_ALIGN:
    {
        if (!Reply_Accepted(reply, JETSON_CAN_START_ALIGN_COMMAND))
        {
            Service_Request(now_ms);
            break;
        }
        if (!Chassis_Is_Stopped() || !Chassis_Get_Odometry(&alignment_origin))
        {
            Halt(now_ms);
            break;
        }
        alignment_started_ms = now_ms;
        Begin_Approach(INITIAL_LOCALIZE, data, now_ms);
        break;
    }
#endif
    case APPROACH_TARGET:
#if JETSON_AUTO_CHASSIS_TEST
        /* 单次联调丢帧时停车等待；不使用未标定的主动搜索参数。 */
        if (Approach(&data->target, &target_window, now_ms))
            Begin_Request(JETSON_CAN_ARRIVAL_ID, WAIT_DESTINATION, now_ms);
#else
        if (Approach(&data->target, &target_window, now_ms))
            Begin_Point_Request(JETSON_CAN_POINT_GRASP, WAIT_GRASP_POINT_ACK, now_ms);
#endif
        break;
    case WAIT_GRASP_POINT_ACK:
        if (Action_Completed(data, now_ms))
        {
            route_token = request.request_id;
            Begin_Request(JETSON_CAN_ARRIVAL_ID, WAIT_DESTINATION, now_ms);
        }
        break;
    case WAIT_DESTINATION:
        Wait_Destination(data, now_ms);
        break;
#if JETSON_AUTO_CHASSIS_TEST
    case TURN_OUT:
        if (Chassis_Motion_Ready())
            Begin_Request(JETSON_CAN_TURN_COMPLETED_ID, WAIT_TURN_ACK, now_ms);
        break;
#else
    case RETURN_TO_START:
    case RECENTER_LATERAL:
        Service_Return(now_ms);
        break;
    case VERIFY_RETURN_POSE:
    case VERIFY_RECENTER:
        Verify_Return(data, now_ms);
        break;
#endif
    case WAIT_TURN_ACK:
        if (!Reply_Accepted(reply, 1U))
            Service_Request(now_ms);
        else
        {
#if JETSON_AUTO_CHASSIS_TEST
            Begin_Approach(APPROACH_SAFETY, data, now_ms);
#else
            if (Chassis_Get_Odometry(&alignment_origin))
            {
                left_axis_x = -sin((double)return_reference_yaw_rad);
                left_axis_y = cos((double)return_reference_yaw_rad);
                alignment_started_ms = now_ms;
                const float vy = delivery_side == SAFE_LEFT ?
                    JETSON_AUTO_LATERAL_SPEED_MPS : -JETSON_AUTO_LATERAL_SPEED_MPS;
                Start_Timed_Move(0.0f, vy, TIMED_LATERAL, now_ms);
            }
            else
                Halt(now_ms);
#endif
        }
        break;
#if JETSON_AUTO_CHASSIS_TEST
    case APPROACH_SAFETY:
        if (Approach(&data->safety, &safety_window, now_ms))
            Start_Distance(CHASSIS_TRANSLATION_FORWARD, JETSON_AUTO_SAFETY_STRAIGHT_M,
                           SAFETY_STRAIGHT, now_ms);
        break;
    case SAFETY_STRAIGHT:
        if (Chassis_Motion_Ready())
            Begin_Request(JETSON_CAN_SAFETY_ARRIVED_ID, WAIT_SAFETY_ACK, now_ms);
        break;
#else
    case TIMED_LATERAL:
    {
        Chassis_Odometry_t pose;
        if (Alignment_Pose(&pose, now_ms) && Timed_Move_Complete(JETSON_AUTO_LATERAL_DURATION_MS, now_ms))
        {
            outbound_lateral_m = Along_Left(pose.x_m - alignment_origin.x_m, pose.y_m - alignment_origin.y_m);
            Begin_Request(JETSON_CAN_SAFETY_ARRIVED_ID, WAIT_SAFETY_ACK, now_ms);
        }
        break;
    }
#endif
    case WAIT_SAFETY_ACK:
        if (!Reply_Accepted(reply, 2U))
            Service_Request(now_ms);
#if JETSON_AUTO_CHASSIS_TEST
        else if (Chassis_Motion_Ready())
#else
        else if (Chassis_Is_Stopped())
#endif
        {
#if JETSON_AUTO_CHASSIS_TEST
            Chassis_Disable();
            Enter(TEST_DONE, now_ms);
#else
            Start_Timed_Move(JETSON_AUTO_PUSH_SPEED_MPS, 0.0f, PUSHING, now_ms);
#endif
        }
        break;
#if JETSON_AUTO_ENABLED
    case PUSHING:
        if (Timed_Move_Complete(JETSON_AUTO_PUSH_DURATION_MS, now_ms))
            Begin_Point_Request(JETSON_CAN_POINT_RELEASE, WAIT_RELEASE_POINT_ACK, now_ms);
        break;
    case WAIT_RELEASE_POINT_ACK:
        if (Action_Completed(data, now_ms))
            Start_Timed_Move(-JETSON_AUTO_RETRACT_SPEED_MPS, 0.0f, RETRACTING, now_ms);
        break;
    case RETRACTING:
        if (Timed_Move_Complete(JETSON_AUTO_RETRACT_DURATION_MS, now_ms))
            Begin_Request(JETSON_CAN_DELIVERY_COMPLETED_ID, WAIT_DELIVERY_ACK, now_ms);
        break;
    case WAIT_DELIVERY_ACK:
        if (!Reply_Accepted(reply, 4U))
            Service_Request(now_ms);
        else
            Start_Return(RECENTER_LATERAL, now_ms);
        break;
    case INITIAL_LOCALIZE:
    case LOCALIZE_START:
    {
        Chassis_Odometry_t pose;
        if (Alignment_Pose(&pose, now_ms) && Approach(&data->safety, &start_window, now_ms))
        {
            const uint8_t initial = stage == INITIAL_LOCALIZE;
            start_pose = pose;
            if (initial)
                return_reference_yaw_rad = pose.yaw_rad;
            Start_Turn(1, initial ? JETSON_AUTO_OUT_TURN_DEG : JETSON_AUTO_RETURN_TURN_DEG,
                       initial ? INITIAL_SEARCH_TURN : TURN_BACK, now_ms);
        }
        break;
    }
    case INITIAL_SEARCH_TURN:
#endif
    case TURN_BACK:
#if JETSON_AUTO_ENABLED
        if (Search_Turn_Stopped(now_ms))
            Begin_Request(JETSON_CAN_START_SEARCH_ID, WAIT_SEARCH_ACK, now_ms);
        break;
#endif
    case WAIT_SEARCH_ACK:
        if (!Reply_Accepted(reply, 3U))
            Service_Request(now_ms);
        else
            Begin_Approach(APPROACH_TARGET, data, now_ms);
        break;
    default:
        break;
    }
}

/** @brief 初始化搬运状态；CAN 接收由原任务启动。 */
void Jetson_Auto_Init(void)
{
    Jetson_CAN_Expect_Command_Reply(0U, 0U);
    Jetson_CAN_Expect_Action_Result(0U, 0U);
    request = (Auto_Request_t){0};
    route_token = center_boundary = center_boundary_ms = 0U;
#if JETSON_AUTO_ENABLED
    delivery_side = 0U;
    start_pose = (Chassis_Odometry_t){0};
#endif
    center_source_valid = 0U;
    navigating = 0U;
    link_bound = 0U;
    link_generation = 0U;
#if JETSON_AUTO_CHASSIS_TEST
    jetson_test_start = test_started = 0U;
    test_started_ms = 0U;
#endif
#if JETSON_AUTO_CHASSIS_TEST
    if (Configuration_Ready())
        Begin_Point_Request(JETSON_CAN_POINT_STARTUP, WAIT_STARTUP, (uint32_t)xTaskGetTickCount());
    else
        Enter(UNCONFIGURED, 0U);
#else
    Enter(WAIT_IMU, (uint32_t)xTaskGetTickCount());
#endif
}

/** @brief 在底盘服务后推进一次，沿用现有链路、轮反馈和制动期限。 */
void Jetson_Auto_Loop(void)
{
#if JETSON_AUTO_CHASSIS_TEST
    if (stage == UNCONFIGURED || stage == TEST_DONE)
        return;
    const uint32_t test_now_ms = (uint32_t)xTaskGetTickCount();
    if (!test_started)
    {
        if (jetson_test_start != 1U)
            return;
        test_started = 1U;
        test_started_ms = test_now_ms;
    }
    if (stage != FAULT && (jetson_test_start != 1U ||
        (uint32_t)(test_now_ms - test_started_ms) >= JETSON_AUTO_CHASSIS_TEST_TIMEOUT_MS))
        Halt(test_now_ms);
#endif
    if (stage == FAULT)
    {
        const uint32_t now_ms = (uint32_t)xTaskGetTickCount();
        if (Chassis_Is_Enabled() && (Chassis_Is_Stopped() ||
            (uint32_t)(now_ms - stage_started_ms) >= JETSON_AUTO_BRAKE_TIMEOUT_MS))
            Chassis_Disable();
        return;
    }

    Jetson_CAN_Data_t data;
    UsbLink_State_t link;
    /* 把 USB 会话与协议快照一起取出，避免断开重连恰好跨越两次读取。 */
    taskENTER_CRITICAL();
    UsbLink_Get_State(&link);
    Jetson_CAN_Get_Data(&data);
    taskEXIT_CRITICAL();
    /* 先取接收快照再读时钟，避免中断跨 tick 写入更新的时间而被误判过期。 */
    const uint32_t now_ms = (uint32_t)xTaskGetTickCount();
#if JETSON_AUTO_ENABLED
    Chassis_Odometry_t pose;
#endif
#if JETSON_AUTO_CHASSIS_TEST
    /* 专用单趟测试沿用人工触发、在线夹爪握手、最后才使能的启动顺序。 */
    if (!Link_Ready(&data, &link, now_ms))
        return;
    if (stage == WAIT_STARTUP)
    {
        if (!Action_Completed(&data, now_ms))
            return;
        Enter(WAIT_IMU, now_ms);
    }
#endif
    if (stage == WAIT_IMU)
    {
        if (!Chassis_Attitude_Ready())
            return;
        Enter(WAIT_ENABLE, now_ms);
    }
    if (stage == WAIT_ENABLE)
    {
        if (!Chassis_Attitude_Ready() || now_ms - stage_started_ms >= JETSON_AUTO_ARM_TIMEOUT_MS)
        {
            Halt(now_ms);
            return;
        }
        if (!Chassis_Can_Enable())
            return;
        if (!Chassis_Enable())
        {
            Halt(now_ms);
            return;
        }
        Enter(ARMING, now_ms);
    }

    if (stage == ARMING)
    {
        if (!Chassis_Is_Enabled())
        {
            Halt(now_ms);
            return;
        }
        if (!Chassis_Motion_Ready()
#if JETSON_AUTO_ENABLED
            || !Chassis_Get_Odometry(&pose)
#endif
           )
        {
            if ((uint32_t)(now_ms - stage_started_ms) >= JETSON_AUTO_ARM_TIMEOUT_MS)
                Halt(now_ms);
            return;
        }
#if JETSON_AUTO_CHASSIS_TEST
        Begin_Approach(APPROACH_TARGET, &data, now_ms);
        return;
#else
        if (Configuration_Ready())
            Begin_Point_Request(JETSON_CAN_POINT_STARTUP, WAIT_STARTUP, now_ms);
        else
            Enter(UNCONFIGURED, now_ms);
#endif
    }
    if (!Chassis_Is_Enabled() || !Chassis_Motors_Ready())
    {
        Halt(now_ms);
        return;
    }
#if JETSON_AUTO_ENABLED
    /* 已使能的零速待机同样监视反馈；出发后覆盖定时盲推等所有阶段。 */
    if (!Chassis_Get_Odometry(&pose))
    {
        Halt(now_ms);
        return;
    }
    if (stage == UNCONFIGURED || !Link_Ready(&data, &link, now_ms))
        return;
    if (stage == WAIT_STARTUP)
    {
        if (Action_Completed(&data, now_ms) && Chassis_Is_Stopped() && Chassis_Motion_Ready())
            Begin_Request(JETSON_CAN_START_ALIGN_ID, WAIT_START_ALIGN, now_ms);
        return;
    }
#endif
    Service_Stage(&data, now_ms);
}
#endif
