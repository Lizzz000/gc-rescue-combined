#include "jetson_auto.h"

#if JETSON_AUTO_CAN_ENABLED
#include "bsp_can.h"
#include "chassis.h"
#include "image_tracker.h"
#include "INS_task.h"
#include "jetson_can.h"
#include "FreeRTOS.h"
#include "task.h"
#include <math.h>

/* 流程与 Jetson 的 MissionCommand.srv 一致；只由原 test_task 推进。 */
typedef enum
{
    UNCONFIGURED, WAIT_LINK, ARMING, APPROACH_TARGET, WAIT_GRASP_POINT_ACK, ROI_BACK,
    WAIT_ROI, TURN_OUT, WAIT_TURN_ACK, APPROACH_SAFETY, WAIT_SAFETY_ACK, PUSHING,
    WAIT_RELEASE_POINT_ACK, RETRACTING, WAIT_DELIVERY_ACK, TURN_BACK, WAIT_SEARCH_ACK,
    RECOVERY_BACK, FAULT, TEST_DONE
} Auto_Stage_t;

typedef struct
{
    float x, y;              /**< 到位点，归一化图像坐标。 */
    float tolerance_x, tolerance_y; /**< 各轴最大允许误差。 */
    uint8_t verify_pair;     /**< 停稳后使用两个新中心核验安全区。 */
} Image_Window_t;

typedef enum { VISION_FOLLOW, VISION_BRAKE, VISION_VERIFY } Vision_Phase_t;

typedef struct
{
    uint32_t received_ms; /**< 最近一次该轴越界的 CAN 接收时刻，ms。 */
    int8_t side;          /**< 最近一次越界的方向 ±1，尚无样本为 0。 */
} Window_Evidence_t;

typedef struct
{
    Vision_Phase_t phase;
    Image_Tracker_t tracker;      /**< 图像定点位置和速度估计。 */
    Jetson_CAN_Center_t previous;  /**< 上一帧积分或停车核验样本。 */
    Window_Evidence_t verify_outside[2]; /**< 停稳核验时 x、y 各自最近一次越界。 */
    float integral_x, integral_y;  /**< 相机各轴的速度补偿，m/s。 */
    uint32_t phase_started_ms;    /**< 制动开始或已确认停稳的 RTOS 时刻。 */
} Vision_Approach_t;

typedef struct
{
    uint16_t id;             /**< 当前待发送或等待回复的标准 CAN ID。 */
    uint8_t point;           /**< 0x208 的到点类型，其余请求为 0。 */
    uint8_t submitted;       /**< 首次已进入发送邮箱。 */
    uint32_t first_rx;       /**< 首次发送前的接收序号屏障。 */
    uint32_t last_tx_ms;     /**< 最近一次入发送邮箱的 RTOS 毫秒。 */
} Auto_Request_t;

static const Image_Window_t target_window = {
    JETSON_AUTO_TARGET_X, JETSON_AUTO_TARGET_Y,
    JETSON_AUTO_TARGET_TOL_X, JETSON_AUTO_TARGET_TOL_Y, 0U
};
static const Image_Window_t safety_window = {
    JETSON_AUTO_SAFETY_X, JETSON_AUTO_SAFETY_Y,
    JETSON_AUTO_SAFETY_TOL_X, JETSON_AUTO_SAFETY_TOL_Y, 1U
};
static Auto_Stage_t stage;
static Auto_Request_t request;
static uint32_t stage_started_ms, center_boundary, route_token, mission_loss_sequence;
static int8_t outbound_direction;
static uint8_t navigating;
static uint8_t timed_move_stopping;
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

/** @brief 检查当前模式使用的参数；底盘单趟测试不使用定距。
 * @return 所需参数已配置返回 1；待标定时保持失能。
 */
static uint8_t Configuration_Ready(void)
{
#if !JETSON_AUTO_CHASSIS_TEST
    const float distances[] = {JETSON_AUTO_ROI_BACK_M, JETSON_AUTO_RECOVERY_BACK_M};
    for (unsigned i = 0U; i < sizeof(distances) / sizeof(distances[0]); ++i)
        if (!isfinite(distances[i]) || distances[i] <= 0.0f)
            return 0U;
#endif

    return Window_Configured(&target_window) && Window_Configured(&safety_window) &&
           (JETSON_AUTO_X_TO_LEFT == 1 || JETSON_AUTO_X_TO_LEFT == -1) &&
           (JETSON_AUTO_Y_TO_FORWARD == 1 || JETSON_AUTO_Y_TO_FORWARD == -1) &&
           isfinite(JETSON_AUTO_APPROACH_GAIN) && JETSON_AUTO_APPROACH_GAIN > 0.0f &&
           Positive_Within(JETSON_AUTO_APPROACH_SPEED_MPS, CHASSIS_STRAIGHT_MAX_SPEED_MPS) &&
           Positive_Within(JETSON_AUTO_APPROACH_MIN_SPEED_MPS, JETSON_AUTO_APPROACH_SPEED_MPS) &&
           isfinite(JETSON_AUTO_APPROACH_I_GAIN) && JETSON_AUTO_APPROACH_I_GAIN >= 0.0f &&
           Positive_Within(JETSON_AUTO_APPROACH_I_LIMIT_MPS, JETSON_AUTO_APPROACH_SPEED_MPS) &&
           isfinite(JETSON_AUTO_APPROACH_SIGMA_X_Q) && JETSON_AUTO_APPROACH_SIGMA_X_Q > 0.0f &&
           isfinite(JETSON_AUTO_APPROACH_SIGMA_Y_Q) && JETSON_AUTO_APPROACH_SIGMA_Y_Q > 0.0f &&
           isfinite(JETSON_AUTO_APPROACH_SIGMA_ACCEL_Q_S2) && JETSON_AUTO_APPROACH_SIGMA_ACCEL_Q_S2 > 0.0f &&
           isfinite(JETSON_AUTO_APPROACH_BRAKE_S) && JETSON_AUTO_APPROACH_BRAKE_S >= 0.0f &&
           Positive_Within(JETSON_AUTO_OUT_TURN_DEG, 360.0f)
#if !JETSON_AUTO_CHASSIS_TEST
           && Positive_Within(fabsf(JETSON_AUTO_SEARCH_WZ_RAD_S), CHASSIS_VECTOR_MAX_WZ_RAD_S) &&
           Positive_Within(JETSON_AUTO_DISTANCE_SPEED_MPS, CHASSIS_DISTANCE_MAX_SPEED_MPS) &&
           Positive_Within(JETSON_AUTO_PUSH_SPEED_MPS, CHASSIS_STRAIGHT_MAX_SPEED_MPS) &&
           JETSON_AUTO_PUSH_DURATION_MS > 0U &&
           Positive_Within(JETSON_AUTO_RETURN_TURN_DEG, 360.0f)
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
    vision = (Vision_Approach_t){0};
    center_boundary = data->rx_sequence;
    Enter(next, now_ms);
}

/** @brief 更新连续速度，首次起步使用底盘已有的三轮停车确认。
 * @param[in] vx,vy 前、左为正的车体线速度，m/s。
 * @param[in] wz 逆时针为正的车体角速度，rad/s。
 * @param[in] now_ms 当前 RTOS 毫秒。
 */
static void Navigate(float vx, float vy, float wz, uint32_t now_ms)
{
    if (!navigating && !Chassis_Is_Stopped())
        return;
    if (!Chassis_Set_Velocity(vx, vy, wz))
        Halt(now_ms);
    else
        navigating = 1U;
}

/** @brief 退出连续速度；后续轮询不重复清零。 */
static void Stop_Navigation(void)
{
    if (navigating)
    {
        Chassis_Stop();
        navigating = 0U;
    }
}

/** @brief 按原始误差限制跟随方向，用预测位移减速并平滑单轴入窗。
 * @param[in] raw_error,error 原始、滤波图像误差，单位为定点值。
 * @param[in] tolerance 到位容差，单位为定点值。
 * @param[in] rate 图像定点坐标每秒变化量。
 * @return 与原始误差同向的定点误差。入窗后输出为零，窗外按预测后的剩余误差减速。
 */
static float Predicted_Error(float raw_error, float error, float tolerance, float rate)
{
    const float direction = copysignf(1.0f, raw_error);
    const float raw_distance = fabsf(raw_error) - tolerance;
    /* 进入允许窗口的轴不再驱动；窗外只对超出窗口的误差施加速度。 */
    if (raw_distance <= 0.0f)
        return 0.0f;
    const float toward = fmaxf(0.0f, direction * rate);
    const float predicted = direction * error - tolerance - JETSON_AUTO_APPROACH_BRAKE_S * toward;
    const float width = fmaxf(1.0f, tolerance);
    /* 预测过窗时保留有界正余量。实际进窗前保留原比例微动幅度，避免过小目标停在窗外。 */
    const float floor = raw_distance * width / (raw_distance + width);
    const float remaining = fminf(raw_distance, fmaxf(floor, predicted));
    return direction * remaining;
}

/** @brief 退出导航，停稳前保持零目标并清除运动时的图像速度。
 * @param[in] now_ms 当前 RTOS 毫秒，作为本次制动起点。
 */
static void Brake_Approach(uint32_t now_ms)
{
    Stop_Navigation();
    vision = (Vision_Approach_t){.phase = VISION_BRAKE, .phase_started_ms = now_ms};
}

/** @brief 清除旧坐标速度，按当前视觉阶段搜索或停车。
 * @param[in] search 1 为目标搜索，0 为等待安全区重新出现。
 * @param[in] now_ms 当前 RTOS 毫秒。
 */
static void Approach_Lost_Center(uint8_t search, uint32_t now_ms)
{
    /* 搜索目标已交给底盘斜坡，无须每 5 ms 重设同一角速度。 */
    if (search && navigating && !vision.previous.valid)
        return;
    vision = (Vision_Approach_t){0};
    if (search)
        Navigate(0.0f, 0.0f, JETSON_AUTO_SEARCH_WZ_RAD_S, now_ms);
    else
        Stop_Navigation();
}

/** @brief 用合法 CAN 中心更新卡尔曼估计，取得本次积分的实际间隔。
 * @param[in] center 非空的合法中心，接收时间使用 RTOS 毫秒。
 * @return 当前积分时长，单位秒。重复快照、首包或长间隔返回 0。
 * @note 相同源帧编号的周期重发不重复参与滤波和积分。
 */
static float Update_Image(const Jetson_CAN_Center_t *center)
{
    if (vision.previous.valid && center->source_frame == vision.previous.source_frame)
        return 0.0f;
    /* 按源帧间隔更新滤波，避免 CAN 调度抖动变成虚假的图像速度。 */
    uint32_t elapsed = vision.previous.valid ?
        (center->source_frame - vision.previous.source_frame) & 0xffffffU : 0U;
    if (!vision.tracker.initialized || elapsed >= JETSON_AUTO_LINK_TIMEOUT_MS)
    {
        Image_Tracker_Init(&vision.tracker, JETSON_AUTO_APPROACH_SIGMA_X_Q,
                           JETSON_AUTO_APPROACH_SIGMA_Y_Q, JETSON_AUTO_APPROACH_SIGMA_ACCEL_Q_S2);
        vision.integral_x = vision.integral_y = 0.0f;
        elapsed = 0U;
    }
    const uint32_t filter_time = vision.tracker.initialized ?
        vision.tracker.received_ms + elapsed : center->received_ms;
    Image_Tracker_Update(&vision.tracker, center->x, center->y, filter_time);
    vision.previous = *center;
    return elapsed * 0.001f;
}

/** @brief 随窗外余量衰减积分输出，即使没有估计到同向速度。
 * @param[in] raw_error,remaining,tolerance 原始误差、减速后的误差、容差，单位为图像定点值。
 * @return 0 到 1 的缩放量。
 */
static float Integral_Scale(float raw_error, float remaining, float tolerance)
{
    return fabsf(remaining) /
        fmaxf(1.0f, fmaxf(tolerance, fabsf(raw_error) - tolerance));
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
        (uint32_t)(now_ms - evidence->received_ms) < JETSON_AUTO_LINK_TIMEOUT_MS;
    *evidence = (Window_Evidence_t){.received_ms = received_ms, .side = side};
    return repeated;
}

/** @brief 计算单轴限幅积分候选。
 * @param[in] integral 之前的速度补偿，m/s。
 * @param[in] error 预测后的归一化误差。
 * @param[in] seconds 本次 CAN 样本间隔，秒。
 * @return 限幅后的速度补偿，m/s。
 */
static float Integral_Candidate(float integral, float error, float seconds)
{
    const float limit = JETSON_AUTO_APPROACH_I_LIMIT_MPS;
    const float next = integral + JETSON_AUTO_APPROACH_I_GAIN * error * seconds;
    return fmaxf(-limit, fminf(limit, next));
}

/** @brief 按图像误差跟随，预测制动并在停稳后用新到达的 CAN 坐标核验。
 * @param[in] center,window 非空的接收中心和归一化到位窗口。
 * @param[in] search 跟随阶段丢失中心时，1 转向搜索，0 停车等待。
 * @param[in] now_ms 当前 RTOS 毫秒。
 * @return 停稳并经过源帧龄窗口后，核验到位返回 1，否则返回 0。
 */
static uint8_t Approach(const Jetson_CAN_Center_t *center, const Image_Window_t *window,
                        uint8_t search, uint32_t now_ms)
{
    if (vision.phase == VISION_BRAKE)
    {
        if (Chassis_Is_Stopped())
        {
            Jetson_CAN_Data_t stopped;
            Jetson_CAN_Get_Data(&stopped);
            center_boundary = stopped.rx_sequence;
            vision.phase_started_ms = (uint32_t)xTaskGetTickCount();
            vision.phase = VISION_VERIFY;
        }
        else if ((uint32_t)(now_ms - vision.phase_started_ms) >= JETSON_AUTO_SETTLE_TIMEOUT_MS)
            Halt(vision.phase_started_ms);
        return 0U;
    }

    const uint8_t newer = Received_After(center->sequence, center_boundary);
    const uint8_t recent = (uint32_t)(now_ms - center->received_ms) < JETSON_AUTO_LINK_TIMEOUT_MS;
    const uint8_t fresh = center->valid && newer && recent;
    if (vision.phase == VISION_VERIFY)
    {
        if ((uint32_t)(now_ms - vision.phase_started_ms) < JETSON_AUTO_VERIFY_WAIT_MS)
            return 0U;
        /* 用实际收帧时刻筛选；等待期间收到的旧坐标不能在截止后直接充当核验。 */
        const uint32_t since_stop = center->received_ms - vision.phase_started_ms;
        const uint8_t after_wait = since_stop >= JETSON_AUTO_VERIFY_WAIT_MS && since_stop <= INT32_MAX;
        if (!recent || (newer && after_wait && !center->valid))
        {
            Approach_Lost_Center(search, now_ms);
            return 0U;
        }
        if (!fresh || !after_wait)
            return 0U;
    }
    /* 底盘斜坡每轮照常推进，视觉只用新包计算目标。无新包时仍检查断流。 */
    if (vision.phase == VISION_FOLLOW && navigating && fresh && vision.previous.valid &&
        center->source_frame == vision.previous.source_frame)
        return 0U;

    /* 在 CAN 定点单位比较窗口，确保标定的边界点按等号接收。 */
    const float goal_x = roundf(window->x * 10000.0f);
    const float goal_y = roundf(window->y * 10000.0f);
    const float x_tol = roundf(window->tolerance_x * 10000.0f);
    const float y_tol = roundf(window->tolerance_y * 10000.0f);
    const float raw_x_error = goal_x - center->x;
    const float raw_y_error = goal_y - center->y;
    const int8_t x_side = Window_Side(raw_x_error, x_tol);
    const int8_t y_side = Window_Side(raw_y_error, y_tol);
    const uint8_t raw_x_in_window = x_side == 0;
    const uint8_t raw_y_in_window = y_side == 0;
    const uint8_t inside = raw_x_in_window && raw_y_in_window;

    if (vision.phase == VISION_VERIFY)
    {
        if (window->verify_pair)
        {
            if (vision.previous.valid && center->source_frame == vision.previous.source_frame)
                return 0U;
            const uint8_t arrived = inside && vision.previous.valid &&
                (uint32_t)(now_ms - vision.previous.received_ms) < JETSON_AUTO_LINK_TIMEOUT_MS &&
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
        if (inside)
            return 1U;
        vision.phase = VISION_FOLLOW;
    }
    if (!fresh)
    {
        Approach_Lost_Center(search, now_ms);
        return 0U;
    }

    /* 安全区边缘有抖动时先保留平滑微调。深入半窗或两帧连续入窗再制动。 */
    if (inside &&
        (!window->verify_pair ||
         (fabsf(raw_x_error) <= 0.5f * x_tol && fabsf(raw_y_error) <= 0.5f * y_tol) ||
         (vision.previous.valid && center->source_frame != vision.previous.source_frame &&
          (uint32_t)(now_ms - vision.previous.received_ms) < JETSON_AUTO_LINK_TIMEOUT_MS &&
          Window_Side(goal_x - vision.previous.x, x_tol) == 0 &&
          Window_Side(goal_y - vision.previous.y, y_tol) == 0)))
    {
        Brake_Approach(now_ms);
        return 0U;
    }

    const float seconds = Update_Image(center);
    const float x_remaining = Predicted_Error(raw_x_error, goal_x - vision.tracker.position[0],
                                             x_tol, vision.tracker.velocity[0]);
    const float y_remaining = Predicted_Error(raw_y_error, goal_y - vision.tracker.position[1],
                                             y_tol, vision.tracker.velocity[1]);

    /* 入窗或误差反向时清除该轴积分，换向不用旧补偿。 */
    if (raw_x_in_window || vision.integral_x * x_remaining <= 0.0f)
        vision.integral_x = 0.0f;
    if (raw_y_in_window || vision.integral_y * y_remaining <= 0.0f)
        vision.integral_y = 0.0f;

    const float ex = x_remaining / 10000.0f;
    const float ey = y_remaining / 10000.0f;
    const float px = JETSON_AUTO_APPROACH_GAIN * ex;
    const float py = JETSON_AUTO_APPROACH_GAIN * ey;
    const float ix_scale = Integral_Scale(raw_x_error, x_remaining, x_tol);
    const float iy_scale = Integral_Scale(raw_y_error, y_remaining, y_tol);
    const float limit = fminf(JETSON_AUTO_APPROACH_SPEED_MPS, CHASSIS_STRAIGHT_MAX_SPEED_MPS - 1e-6f);
    const float next_ix = raw_x_in_window ? 0.0f : Integral_Candidate(vision.integral_x, ex, seconds);
    const float next_iy = raw_y_in_window ? 0.0f : Integral_Candidate(vision.integral_y, ey, seconds);
    /* 合速度限幅时不累积积分，预测接近窗时按剩余误差衰减积分输出。 */
    if (hypotf(px + next_ix * ix_scale, py + next_iy * iy_scale) <= limit)
    {
        vision.integral_x = next_ix;
        vision.integral_y = next_iy;
    }
    float vy = (px + vision.integral_x * ix_scale) * JETSON_AUTO_X_TO_LEFT;
    float vx = (py + vision.integral_y * iy_scale) * JETSON_AUTO_Y_TO_FORWARD;
    /* 窗外保留可调的低速克服静摩擦；入窗后该轴输出为零。 */
    if (!raw_x_in_window && fabsf(vy) < JETSON_AUTO_APPROACH_MIN_SPEED_MPS)
        vy = copysignf(JETSON_AUTO_APPROACH_MIN_SPEED_MPS, x_remaining * JETSON_AUTO_X_TO_LEFT);
    if (!raw_y_in_window && fabsf(vx) < JETSON_AUTO_APPROACH_MIN_SPEED_MPS)
        vx = copysignf(JETSON_AUTO_APPROACH_MIN_SPEED_MPS, y_remaining * JETSON_AUTO_Y_TO_FORWARD);
    const float speed = hypotf(vx, vy);
    const float scale = speed > limit ? limit / speed : 1.0f;
    Navigate(vx * scale, vy * scale, 0.0f, now_ms);
    return 0U;
}

/** @brief 从停稳状态启动一段前进或后退定距。
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

/** @brief 从停稳状态开始限时前进或后退；沿用连续速度斜坡和航向保护。
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
    timed_move_stopping = 0U;
    Enter(next, now_ms);
}

/** @brief 限时动作到期后停车，并等待三轮反馈确认。
 * @param[in] now_ms 当前 RTOS 毫秒。
 * @return 已请求停车且确认停稳时返回 1。
 */
static uint8_t Timed_Move_Stopped(uint32_t now_ms)
{
    if (!timed_move_stopping && (uint32_t)(now_ms - stage_started_ms) >= JETSON_AUTO_PUSH_DURATION_MS)
    {
        Chassis_Stop();
        timed_move_stopping = 1U;
        stage_started_ms = now_ms;
    }
    if (!timed_move_stopping)
        return 0U;
    if (Chassis_Is_Stopped())
        return 1U;
    if ((uint32_t)(now_ms - stage_started_ms) >= JETSON_AUTO_BRAKE_TIMEOUT_MS)
        Halt(now_ms);
    return 0U;
}

/** @brief 从停稳状态启动一次定角转向，转向完成沿用底盘的停车确认。
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

/** @brief 打开一个新握手等待；每次只允许重报同一个已完成事件。
 * @param[in] id 本阶段的上行标准 CAN ID。
 * @param[in] next 回执等待阶段。
 * @param[in] now_ms 当前 RTOS 毫秒，作为总期限起点。
 */
static void Begin_Request(uint16_t id, Auto_Stage_t next, uint32_t now_ms)
{
    Enter(next, now_ms);
    request = (Auto_Request_t){.id = id};
}

/** @brief 停稳后上报 0x208 到点，等待 0x192 命令 5 受理。
 * @param[in] point JETSON_CAN_POINT_GRASP 或 JETSON_CAN_POINT_RELEASE。
 * @param[in] next 回执等待阶段。
 * @param[in] now_ms 当前 RTOS 毫秒，作为总期限起点。
 */
static void Begin_Point_Request(uint8_t point, Auto_Stage_t next, uint32_t now_ms)
{
    Begin_Request(JETSON_CAN_POINT_REACHED_ID, next, now_ms);
    request.point = point;
}

/** @brief 通用放行：Jetson 的任何回复只表示受理，不放行运动。
 * @param[in] data 当前非空接收快照。
 * @param[in] reply_sequence 放行所依据回复的接收序号。
 * @return 该回复之后已收到 pause=0 的合法心跳且底盘停稳时返回 1。
 * @note 等待期间不重发、不计请求期限；Jetson 停发心跳时由链路超时停车。
 */
static uint8_t Released_After(const Jetson_CAN_Data_t *data, uint32_t reply_sequence)
{
    return !data->heartbeat_pause && Received_After(data->heartbeat_sequence, reply_sequence) &&
           Chassis_Is_Stopped();
}

/** @brief 取两个接收序号中较晚的一个。 @param[in] a,b 接收序号。 @return 较晚者。 */
static uint32_t Later(uint32_t a, uint32_t b)
{
    return Received_After(a, b) ? a : b;
}

/** @brief 按首次入邮箱时的接收序号边界判断回执是否属于本次请求。
 * @param[in] sequence 回执的 CAN 接收序号。
 * @return 请求已提交且回执在首次提交之后到达返回 1；期限由 Service_Request 停车处理。
 */
static uint8_t Reply_In_Request(uint32_t sequence)
{
    return request.submitted && Received_After(sequence, request.first_rx);
}

/** @brief 核对发送后收到的命令回执，命令 3、5 无 token，其余沿用本趟 token。
 * @param[in] reply 当前非空回执快照。
 * @param[in] command 期望命令，1 至 5。
 * @return 本次请求得到匹配的成功回执时返回 1。
 */
static uint8_t Reply_Accepted(const Jetson_CAN_Command_Reply_t *reply, uint8_t command)
{
    return Reply_In_Request(reply->sequence) &&
           reply->valid && reply->accepted && reply->command == command &&
           reply->token == (command == 3U || command == JETSON_CAN_POINT_REPLY_COMMAND ? 0U : route_token);
}

/** @brief 在空邮箱时非阻塞提交握手；重试截止不因成功入队而延长。
 * @param[in] now_ms 当前 RTOS 毫秒。
 */
static void Service_Request(uint32_t now_ms)
{
    /* 先由各阶段消费已到达的回复，再按停稳后首次尝试提交起算的唯一期限停车。 */
    if ((uint32_t)(now_ms - stage_started_ms) >= JETSON_AUTO_REQUEST_TIMEOUT_MS)
    {
        Halt(now_ms);
        return;
    }
    if (request.submitted && (uint32_t)(now_ms - request.last_tx_ms) < JETSON_AUTO_REQUEST_RETRY_MS)
        return;

    uint32_t argument = route_token;
    if (request.id == JETSON_CAN_POINT_REACHED_ID)
        argument = request.point;
    else if (request.id == JETSON_CAN_ARRIVAL_ID || request.id == JETSON_CAN_START_SEARCH_ID)
        argument = 0U;
    /* 在入邮箱前取接收边界，期间不让 RX0 中断改写快照。 */
    Jetson_CAN_Data_t before_send;
    taskENTER_CRITICAL();
    Jetson_CAN_Get_Data(&before_send);
    const HAL_StatusTypeDef result = Jetson_CAN_Try_Send_Uplink(request.id, argument);
    const uint32_t sent_ms = (uint32_t)xTaskGetTickCount();
    if (result == HAL_OK && !request.submitted && request.id != JETSON_CAN_ARRIVAL_ID)
        Jetson_CAN_Expect_Command_Reply(request.id, argument);
    taskEXIT_CRITICAL();
    if (result == HAL_OK)
    {
        if (!request.submitted)
        {
            request.first_rx = before_send.rx_sequence;
            request.submitted = 1U;
        }
        request.last_tx_ms = sent_ms;
    }
    else if (result != HAL_BUSY)
        Halt(now_ms);
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
            if (Released_After(data, ack->accepted ? roi->sequence : ack->sequence))
                Start_Distance(CHASSIS_TRANSLATION_BACKWARD, JETSON_AUTO_RECOVERY_BACK_M,
                               RECOVERY_BACK, now_ms);
#endif
            return;
        }
        if (current_roi && roi->passed && turn->valid && turn->destination == roi->destination &&
            Reply_In_Request(turn->sequence))
        {
            if (Released_After(data, Later(Later(ack->sequence, roi->sequence), turn->sequence)))
            {
                route_token = turn->token;
                outbound_direction = turn->destination == 1U ? 1 : -1;
                Start_Turn(outbound_direction, JETSON_AUTO_OUT_TURN_DEG, TURN_OUT, now_ms);
            }
            return;
        }
    }
    Service_Request(now_ms);
}

/** @brief 推进机械动作。定距和转向的停稳结果来自现有底盘控制。
 * @param[in] data 当前非空接收快照。
 * @param[in] now_ms 当前 RTOS 毫秒。
 * @note 回复之后的转段统一由 Released_After 放行；已在运动的阶段不看 pause。
 */
static void Service_Stage(const Jetson_CAN_Data_t *data, uint32_t now_ms)
{
    const Jetson_CAN_Command_Reply_t *reply = &data->command_reply;
    switch (stage)
    {
    case ARMING:
        /* 停稳期限只约束底盘；停稳后等 Jetson 解除 pause（如张开夹爪）再接近。 */
        if (!Chassis_Is_Stopped())
        {
            if ((uint32_t)(now_ms - stage_started_ms) >= JETSON_AUTO_SETTLE_TIMEOUT_MS)
                Halt(now_ms);
        }
        else if (!data->heartbeat_pause)
            Begin_Approach(APPROACH_TARGET, data, now_ms);
        break;
    case APPROACH_TARGET:
#if JETSON_AUTO_CHASSIS_TEST
        /* 单次联调丢帧时停车等待；不使用未标定的主动搜索参数。 */
        if (Approach(&data->target, &target_window, 0U, now_ms))
            Begin_Request(JETSON_CAN_ARRIVAL_ID, WAIT_ROI, now_ms);
#else
        if (Approach(&data->target, &target_window, 1U, now_ms))
            Begin_Point_Request(JETSON_CAN_POINT_GRASP, WAIT_GRASP_POINT_ACK, now_ms);
#endif
        break;
    case WAIT_GRASP_POINT_ACK:
        if (!Reply_Accepted(reply, JETSON_CAN_POINT_REPLY_COMMAND))
            Service_Request(now_ms);
        else if (Released_After(data, reply->sequence))
            Start_Distance(CHASSIS_TRANSLATION_BACKWARD, JETSON_AUTO_ROI_BACK_M, ROI_BACK, now_ms);
        break;
    case ROI_BACK:
        if (Chassis_Is_Stopped())
            Begin_Request(JETSON_CAN_ARRIVAL_ID, WAIT_ROI, now_ms);
        break;
    case WAIT_ROI:
        Wait_ROI(data, now_ms);
        break;
    case TURN_OUT:
        if (Chassis_Is_Stopped())
            Begin_Request(JETSON_CAN_TURN_COMPLETED_ID, WAIT_TURN_ACK, now_ms);
        break;
    case WAIT_TURN_ACK:
        if (!Reply_Accepted(reply, 1U))
            Service_Request(now_ms);
        else if (Released_After(data, reply->sequence))
            Begin_Approach(APPROACH_SAFETY, data, now_ms);
        break;
    case APPROACH_SAFETY:
        if (Approach(&data->safety, &safety_window, 0U, now_ms))
            Begin_Request(JETSON_CAN_SAFETY_ARRIVED_ID, WAIT_SAFETY_ACK, now_ms);
        break;
    case WAIT_SAFETY_ACK:
        if (!Reply_Accepted(reply, 2U))
            Service_Request(now_ms);
        else if (Released_After(data, reply->sequence))
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
        if (Timed_Move_Stopped(now_ms))
            Begin_Point_Request(JETSON_CAN_POINT_RELEASE, WAIT_RELEASE_POINT_ACK, now_ms);
        break;
    case WAIT_RELEASE_POINT_ACK:
        if (!Reply_Accepted(reply, JETSON_CAN_POINT_REPLY_COMMAND))
            Service_Request(now_ms);
        else if (Released_After(data, reply->sequence))
            Start_Timed_Move(-JETSON_AUTO_PUSH_SPEED_MPS, RETRACTING, now_ms);
        break;
    case RETRACTING:
        if (Timed_Move_Stopped(now_ms))
            Begin_Request(JETSON_CAN_DELIVERY_COMPLETED_ID, WAIT_DELIVERY_ACK, now_ms);
        break;
    case WAIT_DELIVERY_ACK:
        if (!Reply_Accepted(reply, 4U))
            Service_Request(now_ms);
        else if (Released_After(data, reply->sequence))
            Start_Turn(-outbound_direction, JETSON_AUTO_RETURN_TURN_DEG, TURN_BACK, now_ms);
        break;
    case TURN_BACK:
    case RECOVERY_BACK:
        if (Chassis_Is_Stopped())
            Begin_Request(JETSON_CAN_START_SEARCH_ID, WAIT_SEARCH_ACK, now_ms);
        break;
    case WAIT_SEARCH_ACK:
        if (!Reply_Accepted(reply, 3U))
            Service_Request(now_ms);
        else if (Released_After(data, reply->sequence))
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
    request = (Auto_Request_t){0};
    route_token = center_boundary = mission_loss_sequence = 0U;
    outbound_direction = 0;
    navigating = 0U;
    timed_move_stopping = 0U;
#if JETSON_AUTO_CHASSIS_TEST
    jetson_test_start = test_started = 0U;
    test_started_ms = 0U;
#endif
    Enter(Configuration_Ready() ? WAIT_LINK : UNCONFIGURED, 0U);
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
    Jetson_CAN_Get_Data(&data);
    /* 先取接收快照再读时钟，避免中断跨 tick 写入更新的时间而被误判过期。 */
    const uint32_t now_ms = (uint32_t)xTaskGetTickCount();
    /* 自动搬运按最后一条合法格式心跳计时。坏帧不保活，速度撤权仍由手动模式处理。 */
    const uint8_t mission_lost = data.mission_loss_sequence != mission_loss_sequence;
    mission_loss_sequence = data.mission_loss_sequence;
    const uint8_t link = !mission_lost && data.heartbeat_seen &&
        (uint32_t)(now_ms - data.heartbeat_received_ms) < JETSON_AUTO_LINK_TIMEOUT_MS;
    if (stage == WAIT_LINK)
    {
        INS_control_data_t imu;
        if (link && INS_Get_Data(&imu) &&
            (uint32_t)((uint32_t)xTaskGetTickCount() - imu.sample_time_ms) <= JETSON_AUTO_IMU_MAX_AGE_MS && Chassis_Enable())
            Enter(ARMING, now_ms);
        return;
    }

    if (!link || !Chassis_Is_Enabled() || (stage != ARMING && !Chassis_Motors_Ready()))
        Halt(now_ms);
    else
        Service_Stage(&data, now_ms);
}
#endif
