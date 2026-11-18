#include "jetson_auto.h"

#if JETSON_AUTO_CAN_ENABLED
#include "chassis.h"
#include "tracked_visual_servo.h"
#if JETSON_AUTO_ENABLED
#include "point_plan.h"
#endif
#include "INS_task.h"
#include "jetson_link.h"
#include "usb_link.h"
#include "FreeRTOS.h"
#include "task.h"
#include <math.h>

/* 流程与 Jetson 的 MissionCommand.srv 一致；只由原 test_task 推进。 */
typedef enum
{
    UNCONFIGURED, WAIT_STARTUP, WAIT_IMU, ARMING, APPROACH_TARGET, WAIT_GRASP_POINT_ACK, ROI_BACK,
    WAIT_ROI, TURN_OUT, WAIT_TURN_ACK, APPROACH_SAFETY, SAFETY_STRAIGHT, WAIT_SAFETY_ACK, PUSHING,
    WAIT_RELEASE_POINT_ACK, RETRACTING, WAIT_DELIVERY_ACK, TURN_BACK, WAIT_SEARCH_ACK,
    WAIT_REJECT_OPEN, RECOVERY_BACK, FAULT, TEST_DONE,
    INITIAL_SEARCH_TURN, RETURN_TO_START, VERIFY_RETURN_POSE, LOCALIZE_START
} Auto_Stage_t;

typedef struct
{
    float x, y;              /**< 到位点，归一化图像坐标。 */
    float tolerance_x, tolerance_y; /**< 各轴最大允许误差。 */
    uint8_t verify_pair;     /**< 发出零速后使用两个新中心核验安全区。 */
} Image_Window_t;

typedef enum { VISION_FOLLOW, VISION_VERIFY } Vision_Phase_t;

typedef struct
{
    uint32_t received_ms; /**< 最近一次该轴越界的 CAN 接收时刻，ms。 */
    int8_t side;          /**< 最近一次越界的方向 ±1，尚无样本为 0。 */
} Window_Evidence_t;

typedef struct
{
    Vision_Phase_t phase;
    AdaptiveKalman tracker;      /**< 图像定点位置和速度估计。 */
    Jetson_CAN_Center_t previous;  /**< 上一帧积分或停车核验样本。 */
    Window_Evidence_t verify_outside[2]; /**< 视觉核验时 x、y 各自最近一次越界。 */
    Visual_Servo_t servo;        /**< 有限制动距离的图像速度规划。 */
    uint32_t evidence_ms;         /**< 最近一次有效新图像的接收时刻，ms。 */
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
static const Image_Window_t safety_windows[2] = {
    {JETSON_AUTO_LEFT_X, JETSON_AUTO_LEFT_Y, JETSON_AUTO_LEFT_TOL_X, JETSON_AUTO_LEFT_TOL_Y, 1U},
    {JETSON_AUTO_RIGHT_X, JETSON_AUTO_RIGHT_Y, JETSON_AUTO_RIGHT_TOL_X, JETSON_AUTO_RIGHT_TOL_Y, 1U},
};
static const Point_Plan_Config_t return_config = {
    .linear_limit_mps = JETSON_AUTO_RETURN_SPEED_MPS,
    .angular_limit_rad_s = 0.50f,
    .linear_deceleration_mps2 = 0.30f,
    .angular_deceleration_rad_s2 = 0.80f,
    .latency_s = 0.12f,
    .position_tolerance_m = 0.01f,
    .yaw_tolerance_rad = 0.06981317f,
};
static Chassis_Odometry_t start_pose;
static Chassis_Odometry_t alignment_origin;
static Point_Plan_t return_plan;
static uint8_t start_side, delivery_side;
#endif
static uint8_t center_source_valid;
static uint32_t center_source_boundary;
static Auto_Stage_t stage;
static Auto_Request_t request;
static uint32_t stage_started_ms, center_boundary, route_token;
/* 软件重新初始化不能复用旧搜索或动作请求号；整板复位需要与桥重新建立会话。 */
static uint32_t next_request_id;
static uint8_t navigating;
static uint32_t navigation_zero_ms;
static uint8_t timed_zero_requested;
static uint32_t safety_lost_ms;
static uint8_t safety_lost;
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
 * @return 所需参数已配置返回 1；待标定时保持失能。
 */
static uint8_t Configuration_Ready(void)
{
#if !JETSON_AUTO_CHASSIS_TEST
    const float distances[] = {JETSON_AUTO_ROI_BACK_M, JETSON_AUTO_RECOVERY_BACK_M};
    for (unsigned i = 0U; i < sizeof(distances) / sizeof(distances[0]); ++i)
        if (!isfinite(distances[i]) || distances[i] < CHASSIS_DISTANCE_MIN_M)
            return 0U;
#endif

#if JETSON_AUTO_CHASSIS_TEST
    if (!isfinite(JETSON_AUTO_SAFETY_STRAIGHT_M) ||
        JETSON_AUTO_SAFETY_STRAIGHT_M < CHASSIS_DISTANCE_MIN_M || !Window_Configured(&safety_window))
        return 0U;
#else
    if (!isfinite(JETSON_AUTO_RETURN_MAX_M) || JETSON_AUTO_RETURN_MAX_M < CHASSIS_DISTANCE_MIN_M ||
        !isfinite(JETSON_AUTO_ALIGN_MAX_M) || JETSON_AUTO_ALIGN_MAX_M < CHASSIS_DISTANCE_MIN_M ||
        !Positive_Within(JETSON_AUTO_RETURN_SPEED_MPS, CHASSIS_DISTANCE_MAX_SPEED_MPS) ||
        !Positive_Within(JETSON_AUTO_ALIGN_SPEED_MPS, CHASSIS_STRAIGHT_MAX_SPEED_MPS) ||
        !Window_Configured(&safety_windows[0]) || !Window_Configured(&safety_windows[1]))
        return 0U;
#endif
    return Positive_Within(JETSON_AUTO_DISTANCE_SPEED_MPS, CHASSIS_DISTANCE_MAX_SPEED_MPS) &&
           Window_Configured(&target_window) &&
           (JETSON_AUTO_X_TO_LEFT == 1 || JETSON_AUTO_X_TO_LEFT == -1) &&
           (JETSON_AUTO_Y_TO_FORWARD == 1 || JETSON_AUTO_Y_TO_FORWARD == -1) &&
           Positive_Within(JETSON_AUTO_APPROACH_SPEED_MPS, CHASSIS_STRAIGHT_MAX_SPEED_MPS) &&
           isfinite(JETSON_AUTO_APPROACH_SIGMA_X_Q) && JETSON_AUTO_APPROACH_SIGMA_X_Q > 0.0f &&
           isfinite(JETSON_AUTO_APPROACH_SIGMA_Y_Q) && JETSON_AUTO_APPROACH_SIGMA_Y_Q > 0.0f &&
           isfinite(JETSON_AUTO_APPROACH_SIGMA_ACCEL_Q_S2) && JETSON_AUTO_APPROACH_SIGMA_ACCEL_Q_S2 > 0.0f &&
           Positive_Within(JETSON_AUTO_APPROACH_ACCEL_MPS2, CHASSIS_STRAIGHT_ACCELERATION_MPS2) &&
           Positive_Within(JETSON_AUTO_APPROACH_DECEL_MPS2, CHASSIS_STRAIGHT_ACCELERATION_MPS2) &&
           Positive_Within(JETSON_AUTO_APPROACH_LATENCY_S, 0.25f) &&
           Positive_Within(JETSON_AUTO_RESPONSE_X_Q_PER_M, 1000000.0f) &&
           Positive_Within(JETSON_AUTO_RESPONSE_Y_Q_PER_M, 1000000.0f) &&
           Positive_Within(JETSON_AUTO_OUT_TURN_DEG, 360.0f)
#if !JETSON_AUTO_CHASSIS_TEST
           && Positive_Within(JETSON_AUTO_PUSH_SPEED_MPS, CHASSIS_STRAIGHT_MAX_SPEED_MPS) &&
           JETSON_AUTO_PUSH_DURATION_MS > 0U
#endif
           ;
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
    safety_lost = 0U;
    const Jetson_CAN_Center_t *previous = next == APPROACH_TARGET ? &data->target : &data->safety;
    center_source_valid = previous->valid;
    center_source_boundary = previous->source_frame;
    Enter(next, now_ms);
}

/** @brief 更新连续速度，底盘负责检查动作准入。
 * @param[in] vx,vy 前、左为正的车体线速度，m/s。
 * @param[in] wz 逆时针为正的车体角速度，rad/s。
 * @param[in] now_ms 当前 RTOS 毫秒。
 */
static void Navigate(float vx, float vy, float wz, uint32_t now_ms)
{
    if (!Chassis_Set_Velocity(vx, vy, wz))
        Halt(now_ms);
    else
    {
        navigating = vx != 0.0f || vy != 0.0f || wz != 0.0f;
        if (!navigating)
            navigation_zero_ms = now_ms;
    }
}

/** @brief 退出连续速度；后续轮询不重复清零。 */
static void Stop_Navigation(uint32_t now_ms)
{
    if (navigating)
    {
        Chassis_Stop();
        navigating = 0U;
        navigation_zero_ms = now_ms;
    }
}

/** @brief 清除旧坐标速度，按当前视觉阶段搜索或停车。
 * @param[in] now_ms 当前 RTOS 毫秒。
 */
static void Approach_Lost_Center(uint32_t now_ms)
{
    vision = (Vision_Approach_t){0};
    Stop_Navigation(now_ms);
#if JETSON_AUTO_ENABLED
    /* 转完后原地等候目标，目标丢失时同样先停下等待新图像。 */
    if (stage == APPROACH_TARGET)
        return;
#endif
    /* 安全区中心丢失后只停车等待。超过制动期限仍没有新中心就 Halt，避免一直停在这个阶段。 */
    if (!safety_lost)
    {
        safety_lost = 1U;
        safety_lost_ms = now_ms;
        return;
    }
    if ((uint32_t)(now_ms - safety_lost_ms) >= JETSON_AUTO_BRAKE_TIMEOUT_MS)
        Halt(now_ms);
}

/** @brief 用合法视觉中心更新卡尔曼估计，取得本次积分的实际间隔。
 * @param[in] center 非空的合法中心，接收时间使用 RTOS 毫秒。
 * @return 当前积分时长，单位秒。首包或长间隔返回 0。
 * @note 相同源帧编号的周期重发不重复参与滤波和积分。
 */
static float Update_Image(const Jetson_CAN_Center_t *center)
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
 * @param[in] x_only 跨区前只调整横向，屏蔽图像纵向误差。
 * @param[in] now_ms 当前 RTOS 毫秒。
 * @return 发出零速后取得新源帧并核验到位时返回 1。
 */
static uint8_t Approach(const Jetson_CAN_Center_t *center, const Image_Window_t *window,
                        uint8_t x_only, uint32_t now_ms)
{
    /* 邮箱暂忙可重试；持续未交付零目标时，即使相机还在发新图也须故障停车。 */
    if (!navigating && !Chassis_Motion_Ready())
    {
        if ((uint32_t)(now_ms - navigation_zero_ms) >= JETSON_AUTO_BRAKE_TIMEOUT_MS)
            Halt(navigation_zero_ms);
        return 0U;
    }
    const uint8_t newer = Received_After(center->sequence, center_boundary);
    const uint8_t recent = (uint32_t)(now_ms - center->received_ms) < JETSON_AUTO_CENTER_TIMEOUT_MS;
    const uint32_t source_delta = (center->source_frame - center_source_boundary) & 0xffffffU;
    const uint8_t fresh = center->valid && newer && recent &&
        (!center_source_valid || (source_delta != 0U && source_delta < 0x800000U));
    if (!fresh)
    {
        /* 保持上一帧的速度或零目标，直到新的图像到达或已有证据过期。 */
        if (center->valid && (vision.previous.valid || vision.phase == VISION_VERIFY) &&
            (uint32_t)(now_ms - vision.evidence_ms) < JETSON_AUTO_CENTER_TIMEOUT_MS)
            return 0U;
        Approach_Lost_Center(now_ms);
        return 0U;
    }
    safety_lost = 0U;
    center_source_valid = 1U;
    center_source_boundary = center->source_frame;
    vision.evidence_ms = center->received_ms;

    /* 在 CAN 定点单位比较窗口，确保标定的边界点按等号接收。 */
    const float goal_x = roundf(window->x * 10000.0f);
    const float goal_y = roundf(window->y * 10000.0f);
    const float x_tol = roundf(window->tolerance_x * 10000.0f);
    const float y_tol = roundf(window->tolerance_y * 10000.0f);
    const float raw_x_error = goal_x - center->x;
    const float raw_y_error = x_only ? 0.0f : goal_y - center->y;
    const int8_t x_side = Window_Side(raw_x_error, x_tol);
    const int8_t y_side = Window_Side(raw_y_error, y_tol);
    const uint8_t inside = x_side == 0 && y_side == 0;

    if (vision.phase == VISION_VERIFY)
    {
        if (window->verify_pair)
        {
            const uint8_t arrived = inside && vision.previous.valid &&
                (uint32_t)(now_ms - vision.previous.received_ms) < JETSON_AUTO_CENTER_TIMEOUT_MS &&
                Window_Side(goal_x - vision.previous.x, x_tol) == 0 &&
                (x_only || Window_Side(goal_y - vision.previous.y, y_tol) == 0);
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
        if (inside)
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
          (x_only || Window_Side(goal_y - vision.previous.y, y_tol) == 0))))
    {
        Stop_Navigation(now_ms);
        vision = (Vision_Approach_t){.phase = VISION_VERIFY, .evidence_ms = center->received_ms};
        return 0U;
    }

    Jetson_CAN_Center_t measurement = *center;
    if (x_only)
        measurement.y = (int16_t)goal_y;
    const float seconds = Update_Image(&measurement);
    const Visual_Servo_Config_t config = {
#if JETSON_AUTO_ENABLED
        .max_speed_mps = stage == APPROACH_TARGET ? JETSON_AUTO_APPROACH_SPEED_MPS : JETSON_AUTO_ALIGN_SPEED_MPS,
#else
        .max_speed_mps = JETSON_AUTO_APPROACH_SPEED_MPS,
#endif
        .acceleration_mps2 = JETSON_AUTO_APPROACH_ACCEL_MPS2,
        .deceleration_mps2 = JETSON_AUTO_APPROACH_DECEL_MPS2,
        .latency_s = JETSON_AUTO_APPROACH_LATENCY_S,
        .response_q_per_m = {JETSON_AUTO_RESPONSE_X_Q_PER_M, JETSON_AUTO_RESPONSE_Y_Q_PER_M},
    };
    const float raw[2] = {measurement.x, measurement.y};
    const float goal[2] = {goal_x, goal_y}, tolerance[2] = {x_tol, y_tol};
    Tracked_Visual_Servo_Update(&vision.servo, &config, &vision.tracker, raw, goal, tolerance, seconds);
    Navigate(x_only ? 0.0f : vision.servo.speed[1] * JETSON_AUTO_Y_TO_FORWARD,
             vision.servo.speed[0] * JETSON_AUTO_X_TO_LEFT, 0.0f, now_ms);
    return 0U;
}

/** @brief 启动一段前进或后退定距。
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

/** @brief 开始限时前进或后退；沿用连续速度斜坡和航向保护。
 * @param[in] speed 前进为正、后退为负，m/s。
 * @param[in] next 本次动作阶段。
 * @param[in] now_ms 命令受理时的 RTOS 毫秒。
 */
static void Start_Timed_Move(float speed, Auto_Stage_t next, uint32_t now_ms)
{
    if (!Chassis_Set_Velocity(speed, 0.0f, 0.0f))
    {
        Halt(now_ms);
        return;
    }
    timed_zero_requested = 0U;
    Enter(next, now_ms);
}

/** @brief 限时动作到期后下发零速，三轮零目标提交到 CAN 后进入握手。 */
static uint8_t Timed_Move_Complete(uint32_t now_ms)
{
    if (!timed_zero_requested)
    {
        if ((uint32_t)(now_ms - stage_started_ms) < JETSON_AUTO_PUSH_DURATION_MS)
            return 0U;
        Chassis_Stop();
        timed_zero_requested = 1U;
        stage_started_ms = now_ms;
    }
    if (Chassis_Motion_Ready())
        return 1U;
    if ((uint32_t)(now_ms - stage_started_ms) >= JETSON_AUTO_BRAKE_TIMEOUT_MS)
        Halt(now_ms);
    return 0U;
}

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
/* 返回本趟保存的绝对位姿。返航与视觉对准均须有新鲜里程计及已标定限位。 */
static void Start_Return(uint32_t now_ms)
{
    Chassis_Odometry_t pose;
    if (!Chassis_Get_Odometry(&pose))
    {
        Halt(now_ms);
        return;
    }
    const double from[3] = {pose.x_m, pose.y_m, pose.yaw_rad};
    const double goal[3] = {start_pose.x_m, start_pose.y_m, start_pose.yaw_rad};
    if (!Point_Plan_Init(&return_plan, from, goal) || return_plan.length_m > JETSON_AUTO_RETURN_MAX_M)
    {
        Halt(now_ms);
        return;
    }
    Enter(RETURN_TO_START, now_ms);
}

static void Service_Return(uint32_t now_ms)
{
    Chassis_Odometry_t pose;
    if ((uint32_t)(now_ms - stage_started_ms) >= JETSON_AUTO_RETURN_TIMEOUT_MS ||
        !Chassis_Get_Odometry(&pose) ||
        hypot(pose.x_m - return_plan.start[0], pose.y_m - return_plan.start[1]) > JETSON_AUTO_RETURN_MAX_M)
    {
        Halt(now_ms);
        return;
    }
    const double current[3] = {pose.x_m, pose.y_m, pose.yaw_rad};
    Point_Plan_Output_t motion;
    if (!Point_Plan_Step(&return_plan, &return_config, current, &motion) ||
        motion.remaining_m > JETSON_AUTO_RETURN_MAX_M)
        Halt(now_ms);
    else if (motion.arrived)
    {
        Chassis_Stop();
        Enter(VERIFY_RETURN_POSE, now_ms);
    }
    else if (!Chassis_Set_Velocity_Explicit_Yaw(motion.vx_mps, motion.vy_mps, motion.wz_rad_s))
        Halt(now_ms);
}

static void Begin_Alignment(Auto_Stage_t next, const Jetson_CAN_Data_t *data, uint32_t now_ms)
{
    if (Chassis_Get_Odometry(&alignment_origin))
        Begin_Approach(next, data, now_ms);
    else
        Halt(now_ms);
}

static uint8_t Aligned(const Jetson_CAN_Center_t *center, uint8_t x_only,
                       Chassis_Odometry_t *pose, uint32_t now_ms)
{
    if ((uint32_t)(now_ms - stage_started_ms) >= JETSON_AUTO_ALIGN_TIMEOUT_MS ||
        !Chassis_Get_Odometry(pose) ||
        hypot(pose->x_m - alignment_origin.x_m, pose->y_m - alignment_origin.y_m) > JETSON_AUTO_ALIGN_MAX_M)
    {
        Halt(now_ms);
        return 0U;
    }
    return Approach(center, &safety_windows[delivery_side - SAFE_LEFT], x_only, now_ms);
}
#endif

/** @brief 打开一个新握手等待；每次只允许重报同一个已完成事件。
 * @param[in] id 本阶段的上行类型号。
 * @param[in] next 回执等待阶段。
 * @param[in] now_ms 当前 RTOS 毫秒，作为总期限起点。
 */
static void Begin_Request(uint16_t id, Auto_Stage_t next, uint32_t now_ms)
{
    Enter(next, now_ms);
    request = (Auto_Request_t){.id = id};
    if (id == JETSON_CAN_POINT_REACHED_ID || id == JETSON_CAN_START_SEARCH_ID)
    {
        if (++next_request_id == 0U)
            ++next_request_id;
        request.request_id = next_request_id;
    }
}

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
           reply->token == ((command == 3U || command == JETSON_CAN_POINT_REPLY_COMMAND) ?
                             request.request_id : route_token);
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

/** @brief 非阻塞提交 USB 握手；调用方已先检查本次消费期限，重试不延长期限。 */
static void Service_Request(uint32_t now_ms)
{
    if (request.submitted && (uint32_t)(now_ms - request.last_tx_ms) < JETSON_AUTO_REQUEST_RETRY_MS)
        return;

    const uint8_t action = request.id == JETSON_CAN_POINT_REACHED_ID;
    uint32_t argument = route_token;
    if (action || request.id == JETSON_CAN_START_SEARCH_ID)
        argument = request.request_id;
    else if (request.id == JETSON_CAN_ARRIVAL_ID)
        argument = 0U;
    uint8_t payload[8];
    const uint8_t length = action ? Jetson_CAN_Encode_Action_Request(request.point, argument, payload) :
                                   Jetson_CAN_Encode_Uplink(request.id, argument, payload);
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

/** @brief 动作受理与终态可乱序；未完成持续查询，失败立即停车。启动时底盘仍禁用。 */
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

/** @brief 消费到达受理、ROI 结果和同侧转向授权。191/193 允许乱序；拿到结果后按通用放行规则开始运动。
 * @param[in] data 当前非空接收快照。
 * @param[in] now_ms 当前 RTOS 毫秒。
 */
static void Wait_ROI(const Jetson_CAN_Data_t *data, uint32_t now_ms)
{
    const Jetson_CAN_Arrival_Ack_t *ack = &data->arrival_ack;
    const Jetson_CAN_ROI_Reply_t *roi = &data->roi_reply;
    const Jetson_CAN_Turn_Request_t *turn = &data->turn_to_safety;
    if (Reply_In_Request(ack->sequence) && ack->valid)
    {
        const uint8_t current_roi = Reply_In_Request(roi->sequence) && roi->valid;
        if (!ack->accepted || (current_roi && !roi->passed))
        {
#if JETSON_AUTO_CHASSIS_TEST
            Halt(now_ms);
#else
            Begin_Point_Request(JETSON_CAN_POINT_REJECT_OPEN, WAIT_REJECT_OPEN, now_ms);
#endif
            return;
        }
        if (current_roi && roi->passed && turn->valid && turn->destination == roi->destination &&
            Reply_In_Request(turn->sequence))
        {
            route_token = turn->token;
#if JETSON_AUTO_CHASSIS_TEST
            Start_Turn(turn->destination == 1U ? 1 : -1, JETSON_AUTO_OUT_TURN_DEG, TURN_OUT, now_ms);
#else
            delivery_side = turn->destination;
            Start_Return(now_ms);
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
    case ARMING:
        /* 开机张爪已经完成，正式搬运在左侧停车位建立本趟起点。 */
        if (Chassis_Motion_Ready()
#if JETSON_AUTO_ENABLED
            && Chassis_Get_Odometry(&start_pose)
#endif
           )
        {
#if JETSON_AUTO_CHASSIS_TEST
            Begin_Approach(APPROACH_TARGET, data, now_ms);
#else
            start_side = SAFE_LEFT;
            Start_Turn(1, 180.0f, INITIAL_SEARCH_TURN, now_ms);
#endif
        }
        else if ((uint32_t)(now_ms - stage_started_ms) >= JETSON_AUTO_ARM_TIMEOUT_MS)
            Halt(now_ms);
        break;
#if JETSON_AUTO_ENABLED
    case INITIAL_SEARCH_TURN:
        if (Chassis_Motion_Ready())
            Begin_Approach(APPROACH_TARGET, data, now_ms);
        break;
#endif
    case APPROACH_TARGET:
#if JETSON_AUTO_CHASSIS_TEST
        /* 单次联调丢帧时停车等待；不使用未标定的主动搜索参数。 */
        if (Approach(&data->target, &target_window, 0U, now_ms))
            Begin_Request(JETSON_CAN_ARRIVAL_ID, WAIT_ROI, now_ms);
#else
        if (Approach(&data->target, &target_window, 0U, now_ms))
            Begin_Point_Request(JETSON_CAN_POINT_GRASP, WAIT_GRASP_POINT_ACK, now_ms);
#endif
        break;
    case WAIT_GRASP_POINT_ACK:
        if (Action_Completed(data, now_ms))
            Start_Distance(CHASSIS_TRANSLATION_BACKWARD, JETSON_AUTO_ROI_BACK_M, ROI_BACK, now_ms);
        break;
    case ROI_BACK:
        if (Chassis_Motion_Ready())
            Begin_Request(JETSON_CAN_ARRIVAL_ID, WAIT_ROI, now_ms);
        break;
    case WAIT_ROI:
        Wait_ROI(data, now_ms);
        break;
#if JETSON_AUTO_CHASSIS_TEST
    case TURN_OUT:
        if (Chassis_Motion_Ready())
            Begin_Request(JETSON_CAN_TURN_COMPLETED_ID, WAIT_TURN_ACK, now_ms);
        break;
#else
    case RETURN_TO_START:
        Service_Return(now_ms);
        break;
    case VERIFY_RETURN_POSE:
        if (Chassis_Motion_Ready())
        {
            Chassis_Odometry_t pose;
            if (!Chassis_Get_Odometry(&pose) || hypot(pose.x_m - start_pose.x_m, pose.y_m - start_pose.y_m) > 0.02 ||
                fabsf(remainderf(pose.yaw_rad - start_pose.yaw_rad, 6.28318530718f)) > 0.174532925f)
                Halt(now_ms);
            else
                Begin_Request(JETSON_CAN_TURN_COMPLETED_ID, WAIT_TURN_ACK, now_ms);
        }
        else if ((uint32_t)(now_ms - stage_started_ms) >= JETSON_AUTO_BRAKE_TIMEOUT_MS)
            Halt(now_ms);
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
            if (delivery_side == start_side)
                Begin_Request(JETSON_CAN_SAFETY_ARRIVED_ID, WAIT_SAFETY_ACK, now_ms);
            else
                Begin_Alignment(APPROACH_SAFETY, data, now_ms);
#endif
        }
        break;
    case APPROACH_SAFETY:
#if JETSON_AUTO_CHASSIS_TEST
        if (Approach(&data->safety, &safety_window, 0U, now_ms))
            Start_Distance(CHASSIS_TRANSLATION_FORWARD, JETSON_AUTO_SAFETY_STRAIGHT_M,
                           SAFETY_STRAIGHT, now_ms);
        break;
    case SAFETY_STRAIGHT:
        if (Chassis_Motion_Ready())
            Begin_Request(JETSON_CAN_SAFETY_ARRIVED_ID, WAIT_SAFETY_ACK, now_ms);
#else
    {
        Chassis_Odometry_t pose;
        if (Aligned(&data->safety, 1U, &pose, now_ms))
            Begin_Request(JETSON_CAN_SAFETY_ARRIVED_ID, WAIT_SAFETY_ACK, now_ms);
    }
#endif
        break;
    case WAIT_SAFETY_ACK:
        if (!Reply_Accepted(reply, 2U))
            Service_Request(now_ms);
        else if (Chassis_Motion_Ready())
        {
#if JETSON_AUTO_CHASSIS_TEST
            Chassis_Disable();
            Enter(TEST_DONE, now_ms);
#else
            Start_Timed_Move(JETSON_AUTO_PUSH_SPEED_MPS, PUSHING, now_ms);
#endif
        }
        break;
    case PUSHING:
        if (Timed_Move_Complete(now_ms))
            Begin_Point_Request(JETSON_CAN_POINT_RELEASE, WAIT_RELEASE_POINT_ACK, now_ms);
        break;
    case WAIT_RELEASE_POINT_ACK:
        if (Action_Completed(data, now_ms))
            Start_Timed_Move(-JETSON_AUTO_PUSH_SPEED_MPS, RETRACTING, now_ms);
        break;
    case RETRACTING:
        if (Timed_Move_Complete(now_ms))
            Begin_Request(JETSON_CAN_DELIVERY_COMPLETED_ID, WAIT_DELIVERY_ACK, now_ms);
        break;
    case WAIT_DELIVERY_ACK:
        if (!Reply_Accepted(reply, 4U))
            Service_Request(now_ms);
#if JETSON_AUTO_ENABLED
        else
            Begin_Alignment(LOCALIZE_START, data, now_ms);
#endif
        break;
#if JETSON_AUTO_ENABLED
    case LOCALIZE_START:
    {
        Chassis_Odometry_t pose;
        if (Aligned(&data->safety, 0U, &pose, now_ms))
        {
            start_pose = pose;
            start_side = delivery_side;
            Start_Turn(1, 180.0f, TURN_BACK, now_ms);
        }
        break;
    }
#endif
    case TURN_BACK:
    case RECOVERY_BACK:
        if (Chassis_Motion_Ready())
            Begin_Request(JETSON_CAN_START_SEARCH_ID, WAIT_SEARCH_ACK, now_ms);
        break;
    case WAIT_SEARCH_ACK:
        if (!Reply_Accepted(reply, 3U))
            Service_Request(now_ms);
        else
            Begin_Approach(APPROACH_TARGET, data, now_ms);
        break;
    case WAIT_REJECT_OPEN:
        if (Action_Completed(data, now_ms))
            Start_Distance(CHASSIS_TRANSLATION_BACKWARD, JETSON_AUTO_RECOVERY_BACK_M, RECOVERY_BACK, now_ms);
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
    route_token = center_boundary = 0U;
#if JETSON_AUTO_ENABLED
    start_side = delivery_side = 0U;
#endif
    center_source_valid = 0U;
    navigating = 0U;
    link_bound = 0U;
    link_generation = 0U;
#if JETSON_AUTO_CHASSIS_TEST
    jetson_test_start = test_started = 0U;
    test_started_ms = 0U;
#endif
    if (Configuration_Ready())
        Begin_Point_Request(JETSON_CAN_POINT_STARTUP, WAIT_STARTUP, (uint32_t)xTaskGetTickCount());
    else
        Enter(UNCONFIGURED, 0U);
}

/** @brief 在底盘服务后推进一次，沿用现有链路、轮反馈和制动期限。 */
void Jetson_Auto_Loop(void)
{
    if (stage == UNCONFIGURED || stage == TEST_DONE)
        return;
#if JETSON_AUTO_CHASSIS_TEST
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
    if (!Link_Ready(&data, &link, now_ms))
        return;
    if (stage == WAIT_STARTUP)
    {
        if (!Action_Completed(&data, now_ms))
            return;
        Enter(WAIT_IMU, now_ms);
    }
    if (stage == WAIT_IMU)
    {
        INS_control_data_t imu;
        if (INS_Get_Data(&imu) &&
            (uint32_t)((uint32_t)xTaskGetTickCount() - imu.sample_time_ms) <= JETSON_AUTO_IMU_MAX_AGE_MS && Chassis_Enable())
            Enter(ARMING, now_ms);
        return;
    }

    if (!Chassis_Is_Enabled() || (stage != ARMING && !Chassis_Motors_Ready()))
    {
        Halt(now_ms);
        return;
    }
#if JETSON_AUTO_ENABLED
    /* 出发后任何阶段失去里程计都不得继续上次动作，包括限时盲推。 */
    Chassis_Odometry_t pose;
    if (stage != ARMING && !Chassis_Get_Odometry(&pose))
    {
        Halt(now_ms);
        return;
    }
#endif
    Service_Stage(&data, now_ms);
}
#endif
