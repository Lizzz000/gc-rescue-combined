/* 来源：旧车 chassis_straight/chassis_translation/chassis_turn 的动作计算与参数。
 * 设置目标、周期计算后提交 MIT 的组织方式参考 basic_framework；三轮几何沿旧车，
 * 不使用其四轮公式。本文件仅保留动作状态，不迁移输入仲裁、租期或整套故障框架。
 * F407 适配：三个 CAN 发送邮箱，三帧一组；取消异步完成后才提交失能帧。
 * 本次 F407 收发实测适配：成功帧之间让出 1 tick，留出电机反馈时间，已架空验证；
 * 此间隔不是旧车参数，10 ms 控制节拍、PID 和轮序参数保持不变。
 */
#include "chassis.h"
#include "motor_chassis.h"
#include "dm_motor.h"
#include "bsp_can.h"
#include "INS_task.h"
#include "pid.h"
#include "tim.h"
#include "FreeRTOS.h"
#include "task.h"
#include <math.h>

#define CONTROL_PERIOD_MS 10U
#define STOP_SPEED_RAD_S 0.20f
#define STOP_GROUPS 3U
#define TIMED_BRAKE_LIMIT_MS 500U
#define DEG_TO_RAD 0.01745329251994329577f
#define CHASSIS_TURN_DEG_TO_RAD 0.01745329251994329577

typedef enum { IDLE, TRANSLATION, DISTANCE, VELOCITY, TURN } Chassis_Mode_t;

/* 轮序、命令 ID、方向、反馈轴/轮轴齿比、MIT 速度阻尼均沿旧车配置。 */
static const uint16_t motor_id[3] = {1U, 2U, 3U};
static const float motor_direction[3] = {-1.0f, -1.0f, -1.0f};
static const float motor_gear_ratio[3] = {1.0f, 1.0f, 1.0f};
static const float motor_kd[3] = {0.2f, 0.25f, 0.2f};
static const float motor_vmax[3] = {40.0f, 40.0f, 40.0f};
float chassis_speed[3];
static pid_type_def yaw_pid, turn_pid;
static uint8_t enabled, pending_disable, pending_enable;
static uint8_t monitoring_stop, stop_groups, disable_after_stop;
static uint32_t stop_seq[3], last_stop_group_ms;
static Chassis_Mode_t mode;
static TickType_t last_control_tick;
static uint32_t last_control_us, step_start_us;
static Chassis_Translation_Direction_t translation_direction;
static float target_speed, ramp_speed, target_yaw_deg;
static uint32_t run_duration_ms, run_started_ms;
static uint8_t run_started;
static double profile_distance_m, profile_rise_s, profile_end_s;
static float vector_target[3], vector_ramp[3];
static float turn_requested_deg, turn_accumulated_deg, turn_last_yaw_deg;
static float turn_speed;
static uint8_t distance_turn_blend;
static int8_t turn_direction;
static uint32_t turn_started_ms, turn_settled_ms;
static uint8_t turn_settling;

static uint8_t Send_Chassis_Targets(float vx, float vy, float wz);

/** @brief 初始化发帧与斜坡时基。RTOS tick 控制节拍，TIM2 微秒用于实际步长。
 * @return 无。
 */
static void Reset_Control_Time(void)
{
    last_control_tick = xTaskGetTickCount();
    last_control_us = step_start_us = __HAL_TIM_GET_COUNTER(&htim2);
}

/** @brief 返回本任务的软件使能状态，不代替实际电机反馈。 @return 使能为 1，否则 0。 */
uint8_t Chassis_Is_Enabled(void)
{
    return enabled;
}

/** @brief 沿旧 Read_IMU 读取有效姿态，使用同源 RTOS 毫秒检查 20 ms 样本期限。
 * @param[out] imu 姿态快照，yaw 为度、世界 z 角速度为 rad/s。
 * @return 姿态有效且未超龄返回 1，否则 0。
 * @note 数值有效性由 INS 发布时检查，此处不重复检查。
 */
static uint8_t Read_IMU(INS_control_data_t *imu)
{
    return INS_Get_Data(imu) &&
           (uint32_t)((uint32_t)xTaskGetTickCount() - imu->sample_time_ms) <= 20U;
}

/** @brief 根据达妙 CAN1 的接收序号与到达时间确认三轮静止，本车 10 ms 控制适配。
 * @param[in] baseline 1 表示刚提交首组三轮零 MIT，仅记录此刻序号；0 检查后续反馈。
 * @return 无；三轮均有新帧才累计一组，单轮过快、失能或断流会撤销确认。
 */
static void Update_Stop_Feedback(uint8_t baseline)
{
    uint32_t seq[3];
    uint8_t fresh_group = 1U, quiet = 1U;
    taskENTER_CRITICAL();
    uint32_t now_ms = (uint32_t)xTaskGetTickCount();
    for (unsigned i = 0U; i < 3U; ++i)
    {
        seq[i] = dm_motor[i].feedback_seq;
        if (seq[i] == stop_seq[i])
            fresh_group = 0U;
        if (dm_motor[i].state != 1U || !(fabsf(dm_motor[i].velocity) <= STOP_SPEED_RAD_S) ||
            (uint32_t)(now_ms - dm_motor[i].feedback_time_ms) > CHASSIS_MOTOR_FEEDBACK_TIMEOUT_MS)
            quiet = 0U;
    }
    taskEXIT_CRITICAL();

    if (baseline || !quiet || (uint32_t)(now_ms - last_stop_group_ms) > CHASSIS_MOTOR_FEEDBACK_TIMEOUT_MS)
        stop_groups = 0U;
    if (baseline || !quiet || fresh_group)
        for (unsigned i = 0U; i < 3U; ++i)
            stop_seq[i] = seq[i];
    if (!baseline && quiet && fresh_group)
    {
        last_stop_group_ms = now_ms;
        if (stop_groups < STOP_GROUPS)
            ++stop_groups;
    }
}

/** @brief 检查使能待命时，零速指令之后的三轮反馈是否仍能确认停车。
 * @return 连续三组新反馈均在轮速门限内且未过期返回 1，否则返回 0。
 */
uint8_t Chassis_Is_Stopped(void)
{
    if (!enabled || mode != IDLE || !monitoring_stop)
        return 0U;
    Update_Stop_Feedback(0U);
    return stop_groups == STOP_GROUPS;
}

/** @brief 用与反馈 ISR 相同的时基核对三轮使能状态与反馈年龄；约束见头文件。 */
uint8_t Chassis_Motors_Ready(void)
{
    uint8_t ready = 1U;
    taskENTER_CRITICAL();
    uint32_t now_ms = (uint32_t)xTaskGetTickCount();
    for (unsigned i = 0U; i < 3U; ++i)
        if (dm_motor[i].id != motor_id[i] || dm_motor[i].state != 1U ||
            !isfinite(dm_motor[i].velocity) ||
            (uint32_t)(now_ms - dm_motor[i].feedback_time_ms) > CHASSIS_MOTOR_FEEDBACK_TIMEOUT_MS)
            ready = 0U;
    taskEXIT_CRITICAL();
    return ready;
}

/** @brief 沿旧 Drive_Enter_Zero 清除运动目标和 PID，保留电机使能状态。
 * @return 无；只更新本任务的软件状态。
 */
void Chassis_Stop(void)
{
    if (mode != IDLE)
        monitoring_stop = 0U;
    mode = IDLE;
    target_speed = ramp_speed = 0.0f;
    turn_speed = 0.0f;
    distance_turn_blend = 0U;
    run_duration_ms = 0U;
    run_started = turn_settling = 0U;
    for (unsigned i = 0U; i < 3U; ++i)
        vector_target[i] = vector_ramp[i] = 0.0f;
    PID_clear(&yaw_pid);
    PID_clear(&turn_pid);
}

/** @brief 沿旧初始化参数准备航向/转向 PID；默认失能，不提交电机指令。
 * @return 无。
 */
void Chassis_Init(void)
{
    const float yaw_gains[3] = {CHASSIS_STRAIGHT_YAW_KP, CHASSIS_STRAIGHT_YAW_KI,
                                CHASSIS_STRAIGHT_YAW_KD};
    const float turn_gains[3] = {CHASSIS_TURN_YAW_KP, CHASSIS_TURN_YAW_KI,
                                 CHASSIS_TURN_YAW_KD};
    PID_init(&yaw_pid, PID_POSITION, yaw_gains, CHASSIS_STRAIGHT_YAW_OUTPUT_LIMIT,
             CHASSIS_STRAIGHT_YAW_INTEGRAL_LIMIT);
    PID_init(&turn_pid, PID_POSITION, turn_gains, CHASSIS_TURN_OUTPUT_LIMIT,
             CHASSIS_TURN_INTEGRAL_LIMIT);
    enabled = pending_disable = pending_enable = 0U;
    monitoring_stop = stop_groups = disable_after_stop = 0U;
    Chassis_Stop();
    for (unsigned i = 0U; i < 3U; ++i)
        chassis_speed[i] = 0.0f;
    Reset_Control_Time();
}

/** @brief 清目标并撤销待发帧，失能帧交由 Loop 等待三个邮箱空闲后提交。
 * @return 无；沿旧停机先取消、后 FD 的顺序，适配 F407 异步取消。
 */
void Chassis_Disable(void)
{
    enabled = 0U;
    pending_enable = 0U;
    monitoring_stop = stop_groups = disable_after_stop = 0U;
    Chassis_Stop();
    pending_disable = 1U;
    HAL_CAN_AbortTxRequest(&MOTOR_CAN, CAN_TX_MAILBOX0 | CAN_TX_MAILBOX1 | CAN_TX_MAILBOX2);
}

/** @brief 沿旧 Queue_Enable_Command 先提交零 MIT，再由 Loop 分周期提交 FC/零 MIT。
 * @return 首组三帧提交成功/已经接受使能返回 1，否则 0。
 */
uint8_t Chassis_Enable(void)
{
    if (pending_disable)
        return 0U;
    if (enabled)
        return 1U;
    if (HAL_CAN_GetTxMailboxesFreeLevel(&MOTOR_CAN) != 3U)
        return 0U;
    if (!Send_Chassis_Targets(0.0f, 0.0f, 0.0f))
        return 0U;
    Chassis_Stop();
    enabled = 1U;
    pending_enable = 1U;
    Reset_Control_Time();
    return 1U;
}

/** @brief 沿旧八向分解生成等幅平移分量，调用方已校验方向。
 * @param[in] direction 八向枚举，FORWARD 至 BACKWARD_RIGHT。
 * @param[in] speed 合速度，m/s。
 * @param[out] vx 前向速度，m/s。
 * @param[out] vy 左向速度，m/s。
 * @return 无。
 */
static void Chassis_Translation_Target(Chassis_Translation_Direction_t direction,
                                       float speed, float *vx, float *vy)
{
    static const int8_t axes[8][2] = {
        {1, 0}, {-1, 0}, {0, 1}, {0, -1}, {1, 1}, {1, -1}, {-1, 1}, {-1, -1}
    };
    unsigned index = (unsigned)direction - 1U;
    float component = axes[index][0] && axes[index][1] ? speed * 0.70710678118654752440f : speed;
    *vx = axes[index][0] * component;
    *vy = axes[index][1] * component;
}

/** @brief 沿旧车 Start_Run 在待命时锁定本次平移的车体方向和起始航向。
 * @param[in] direction FORWARD 至 BACKWARD_RIGHT 的八向枚举。
 * @return 状态、方向和姿态有效时返回 1，否则返回 0。
 */
static uint8_t Prepare_Translation(Chassis_Translation_Direction_t direction)
{
    INS_control_data_t imu;
    if (disable_after_stop || !Chassis_Is_Stopped() || direction < CHASSIS_TRANSLATION_FORWARD ||
        direction > CHASSIS_TRANSLATION_BACKWARD_RIGHT || !Read_IMU(&imu))
        return 0U;
    Chassis_Stop();
    translation_direction = direction;
    target_yaw_deg = CHASSIS_STRAIGHT_YAW_SIGN * imu.yaw_deg;
    /* 保留原发帧节拍，只从接单时刻开始累计首拍斜坡。 */
    step_start_us = __HAL_TIM_GET_COUNTER(&htim2);
    return 1U;
}

/** @brief 沿旧车 Start_Run 设置定时平移，参数及返回值见 chassis.h。 */
uint8_t Chassis_Translation_Start(Chassis_Translation_Direction_t direction,
                                  float speed, uint32_t duration_ms)
{
    if (!isfinite(speed) || speed <= 0.0f || speed > CHASSIS_STRAIGHT_MAX_SPEED_MPS ||
        duration_ms > INT32_MAX || !Prepare_Translation(direction))
        return 0U;
    target_speed = speed;
    run_duration_ms = duration_ms;
    mode = TRANSLATION;
    return 1U;
}

/** @brief 按指定距离求静止到静止的限速轨迹，参数及返回值见 chassis.h。
 * @note 参考 WPILib TrapezoidProfile 的速度、加速度约束。本车对称加减速，
 *       短段峰速为 sqrt(a*d)，长段在限速下巡航。规划仅在接收动作时做一次。
 *       本车连续往返适配：转向剩余 5 度内允许接入定距，保留原转向终点航向，
 *       由原直行 PID 在行驶中修偏；角速度沿原斜坡接管，不经过停车状态。
 */
uint8_t Chassis_Distance_Start(Chassis_Translation_Direction_t direction,
                               float distance_m, float speed_limit_mps)
{
    if (!isfinite(distance_m) || distance_m <= 0.0f || !isfinite(speed_limit_mps) ||
        speed_limit_mps <= 0.0f || speed_limit_mps > CHASSIS_DISTANCE_MAX_SPEED_MPS)
        return 0U;
    const double accel = CHASSIS_DISTANCE_ACCELERATION_MPS2;
    const double peak = fmin(speed_limit_mps, sqrt(accel * distance_m));
    const double rise_s = peak / accel;
    const double end_s = distance_m / peak + rise_s;
    if (end_s * 1000.0 > INT32_MAX)
        return 0U;
    if (mode == TURN)
    {
        INS_control_data_t imu;
        if (disable_after_stop || direction < CHASSIS_TRANSLATION_FORWARD ||
            direction > CHASSIS_TRANSLATION_BACKWARD_RIGHT || !Read_IMU(&imu))
            return 0U;
        float yaw = CHASSIS_STRAIGHT_YAW_SIGN * imu.yaw_deg;
        float delta = remainderf(yaw - turn_last_yaw_deg, 360.0f);
        if (fabsf(delta) == 180.0f)
            delta = copysignf(180.0f, imu.world_z_rad_s != 0.0f ?
                             CHASSIS_STRAIGHT_YAW_SIGN * imu.world_z_rad_s : (float)turn_direction);
        float remaining = turn_requested_deg - turn_accumulated_deg - delta;
        if (fabsf(remaining) > CHASSIS_TURN_FINE_ANGLE_DEG)
            return 0U;
        float previous_speed = turn_speed;
        Chassis_Stop();
        translation_direction = direction;
        target_yaw_deg = remainderf(yaw + remaining, 360.0f);
        turn_speed = previous_speed;
        distance_turn_blend = 1U;
        /* 运行中衔接保留控制时基；首个定距目标仍在下一控制拍提交。 */
    }
    else if (!Prepare_Translation(direction))
        return 0U;
    profile_distance_m = distance_m;
    profile_rise_s = rise_s;
    profile_end_s = end_s;
    mode = DISTANCE;
    return 1U;
}

/** @brief 求定距轨迹从起点到时刻 t 的指令路程。
 * @param[in] t 自首组运动帧起的秒数，大于等于零。
 * @return 路程，m，到终点后保持目标距离。
 * @note 为本车限速梯形的分段积分，制动段从终点按剩余时间反推。
 */
static double Distance_Position(double t)
{
    const double accel = CHASSIS_DISTANCE_ACCELERATION_MPS2;
    if (t < profile_rise_s)
        return 0.5 * accel * t * t;
    if (t <= profile_end_s - profile_rise_s)
        return accel * profile_rise_s * (t - 0.5 * profile_rise_s);
    const double remaining_s = fmax(0.0, profile_end_s - t);
    return profile_distance_m - 0.5 * accel * remaining_s * remaining_s;
}

/** @brief 沿旧直行接口选择前向；参数和返回值同八向启动接口。 */
uint8_t Chassis_Straight_Start(float speed, uint32_t duration_ms)
{
    return Chassis_Translation_Start(CHASSIS_TRANSLATION_FORWARD, speed, duration_ms);
}

/** @brief 沿旧 Chassis_Turn_Start 保存相对转角起点，独立 PID 从零开始。
 * @param[in] direction +1 左转，-1 右转。
 * @param[in] angle_deg 相对转角，度，有限且 (0,360]。
 * @return 从待命接受返回 1，状态、参数或姿态不满足返回 0。
 * @note 本次定角参数适配，仅将原固定整圈改为调用者传入角度。
 */
uint8_t Chassis_Turn_Start(int8_t direction, float angle_deg)
{
    INS_control_data_t imu;
    if (disable_after_stop || !Chassis_Is_Stopped() ||
        (direction != 1 && direction != -1) || !isfinite(angle_deg) ||
        angle_deg <= 0.0f || angle_deg > 360.0f || !Read_IMU(&imu))
        return 0U;
    Chassis_Stop();
    turn_direction = direction;
    turn_requested_deg = direction * angle_deg;
    turn_accumulated_deg = 0.0f;
    turn_last_yaw_deg = CHASSIS_STRAIGHT_YAW_SIGN * imu.yaw_deg;
    turn_started_ms = (uint32_t)xTaskGetTickCount();
    mode = TURN;
    step_start_us = __HAL_TIM_GET_COUNTER(&htim2);
    return 1U;
}

/** @brief 沿旧连续向量入口保存目标，纯平移开始时记录当前航向。
 * @param[in] vx 前向速度，m/s。
 * @param[in] vy 左向速度，m/s；与 vx 合速度不超过 0.50。
 * @param[in] wz 逆时针角速度，rad/s，绝对值不超过 2.0。
 * @return 请求接受返回 1，否则 0；全零请求回零速待命。
 * @note 连续更新平移目标保留航向；显式转向结束后按当前姿态重新锁定，斜坡连续。
 */
uint8_t Chassis_Set_Velocity(float vx, float vy, float wz)
{
    INS_control_data_t imu;
    if (!enabled || !isfinite(vx) || !isfinite(vy) || !isfinite(wz) ||
        hypotf(vx, vy) > CHASSIS_STRAIGHT_MAX_SPEED_MPS ||
        fabsf(wz) > CHASSIS_VECTOR_MAX_WZ_RAD_S)
        return 0U;
    if (vx == 0.0f && vy == 0.0f && wz == 0.0f)
    {
        Chassis_Stop();
        return 1U;
    }
    if (disable_after_stop || (mode != VELOCITY && !Chassis_Is_Stopped()) || !Read_IMU(&imu))
        return 0U;
    if (mode == IDLE)
        step_start_us = __HAL_TIM_GET_COUNTER(&htim2);
    if (wz == 0.0f && (mode != VELOCITY || vector_target[2] != 0.0f))
    {
        target_yaw_deg = CHASSIS_STRAIGHT_YAW_SIGN * imu.yaw_deg;
        PID_clear(&yaw_pid);
    }
    vector_target[0] = vx;
    vector_target[1] = vy;
    vector_target[2] = wz;
    mode = VELOCITY;
    return 1U;
}

/** @brief 沿旧 Send_Chassis_Targets 整体限制轮速，加入方向/齿比后发三帧 MIT。
 * @param[in] vx 前向速度，m/s。
 * @param[in] vy 左向速度，m/s。
 * @param[in] wz 逆时针角速度，rad/s。
 * @return 整组提交成功返回 1；提交失败请求失能并返回 0。
 * @note 调用方已确认三个邮箱空闲，电机CAN发送由本任务独占。
 */
static uint8_t Send_Chassis_Targets(float vx, float vy, float wz)
{
    float wheel[3];
    Motor_Chassis_Inverse(vx, vy, wz, wheel);
    float scale = 1.0f;
    for (unsigned i = 0U; i < 3U; ++i)
    {
        float speed = fabsf(wheel[i] * motor_gear_ratio[i]);
        if (speed > motor_vmax[i])
            scale = fminf(scale, motor_vmax[i] / speed);
    }
    for (unsigned i = 0U; i < 3U; ++i)
    {
        float target = wheel[i] * scale * motor_gear_ratio[i] * motor_direction[i];
        if (ctrl_motor(&MOTOR_CAN, motor_id[i], 0.0f, target, 0.0f, motor_kd[i], 0.0f) != HAL_OK)
        {
            Chassis_Disable();
            return 0U;
        }
        if (i < 2U)
            vTaskDelay(pdMS_TO_TICKS(1U));
    }
    return 1U;
}

/** @brief 沿用定向平移的航向 PID 计算修正角速度。
 * @param[in] imu 已核验的非空姿态，航向为度，世界 z 角速度为 rad/s。
 * @param[in] dt 实际控制间隔，s，首拍同刻接单时可为 0。
 * @return 修正后的车体角速度，rad/s，逆时针为正。
 */
static float Hold_Yaw(const INS_control_data_t *imu, float dt)
{
    const float error = remainderf(target_yaw_deg - CHASSIS_STRAIGHT_YAW_SIGN * imu->yaw_deg, 360.0f);
    return PID_Step(&yaw_pid, error * DEG_TO_RAD, CHASSIS_STRAIGHT_YAW_SIGN * imu->world_z_rad_s, dt);
}

/** @brief 沿旧 Service_Vector_Run 联合限制线速度和角速度增量。
 * @param[in] target_wz 显式或锁航向生成的角速度目标，rad/s，逆时针为正。
 * @param[in] dt 实际控制间隔，s，大于 0。
 * @return 无；只更新三维斜坡，发送由 Loop 统一完成。
 */
static void Service_Vector_Run(float target_wz, float dt)
{
    float dx = vector_target[0] - vector_ramp[0];
    float dy = vector_target[1] - vector_ramp[1];
    float distance = hypotf(dx, dy);
    float linear_limit = CHASSIS_STRAIGHT_ACCELERATION_MPS2 * dt;
    float scale = distance > linear_limit ? linear_limit / distance : 1.0f;
    float angular_delta = target_wz - vector_ramp[2];
    float angular_limit = CHASSIS_VECTOR_ACCEL_RAD_S2 * dt;
    if (fabsf(angular_delta) > angular_limit)
        scale = fminf(scale, angular_limit / fabsf(angular_delta));
    vector_ramp[0] += dx * scale;
    vector_ramp[1] += dy * scale;
    vector_ramp[2] += angular_delta * scale;
}

/** @brief 沿旧 Chassis_Turn_Step 累计跨圈角度并判定连续到位。
 * @param[in] imu 已验证的姿态快照，yaw 为度、角速度为 rad/s。
 * @param[in] dt 实际控制间隔，s，首拍同刻接单时可为 0。
 * @param[in] now_ms 当前 RTOS 毫秒，用于 200 ms 连续到位计时。
 * @return 目标角速度，rad/s；到位时清动作并返回零。
 * @note 2026-11-08 转向适配：远端用 omega²=2*alpha*剩余角规划制动速度，
 *       最后 5 度沿旧 PID 慢速修偏；沿 Service_Vector_Run 的实际 dt 限增量。
 *       制动规划为本车适配，不改变 PID 增益或电机 MIT 阻尼。
 */
static float Service_Turn(const INS_control_data_t *imu, float dt, uint32_t now_ms)
{
    float yaw = CHASSIS_STRAIGHT_YAW_SIGN * imu->yaw_deg;
    float rate = CHASSIS_STRAIGHT_YAW_SIGN * imu->world_z_rad_s;
    float delta = remainderf(yaw - turn_last_yaw_deg, 360.0f);
    if (fabsf(delta) == 180.0f)
        delta = copysignf(180.0f, rate != 0.0f ? rate : (float)turn_direction);
    turn_accumulated_deg += delta;
    turn_last_yaw_deg = yaw;
    double remaining = (double)turn_requested_deg - turn_accumulated_deg;
    float wz = PID_Step(&turn_pid, (float)(remaining * CHASSIS_TURN_DEG_TO_RAD), rate, dt);
    if (fabs(remaining) > CHASSIS_TURN_FINE_ANGLE_DEG)
    {
        float braking_speed = sqrtf(2.0f * CHASSIS_TURN_ACCEL_RAD_S2 *
                                    (float)(fabs(remaining) * CHASSIS_TURN_DEG_TO_RAD));
        wz = copysignf(fminf(CHASSIS_TURN_OUTPUT_LIMIT, braking_speed), (float)remaining);
    }
    float speed_step = CHASSIS_TURN_ACCEL_RAD_S2 * dt;
    turn_speed += fmaxf(-speed_step, fminf(speed_step, wz - turn_speed));
    if (fabs(remaining) <= CHASSIS_TURN_ANGLE_TOLERANCE_DEG &&
        fabsf(rate) <= CHASSIS_TURN_RATE_TOLERANCE_RAD_S &&
        fabsf(turn_speed) <= CHASSIS_TURN_RATE_TOLERANCE_RAD_S)
    {
        if (!turn_settling)
        {
            turn_settling = 1U;
            turn_settled_ms = now_ms;
        }
        if ((uint32_t)(now_ms - turn_settled_ms) >= CHASSIS_TURN_SETTLE_TIME_MS)
        {
            Chassis_Stop();
            return 0.0f;
        }
    }
    else
        turn_settling = 0U;
    return turn_speed;
}

/** @brief 适配现有 5 ms RTOS 任务；沿旧 100 Hz 控制、实际 dt 和到期动作语义。
 * @return 无；不补发历史周期，不自动使能。
 */
void Chassis_Loop(void)
{
    const TickType_t now_tick = xTaskGetTickCount();
    const uint32_t now_ms = (uint32_t)now_tick;
    if (mode == TRANSLATION && run_started && run_duration_ms != 0U &&
        (uint32_t)(now_ms - run_started_ms) >= run_duration_ms)
    {
        Chassis_Stop();
        disable_after_stop = 1U;
        run_started_ms = now_ms;
    }
    /* 定时动作先用原 MIT 速度阻尼制动，再按旧约定失能；反馈缺失时限时释放。 */
    if ((disable_after_stop && ((uint32_t)(now_ms - run_started_ms) >= TIMED_BRAKE_LIMIT_MS ||
                                (stop_groups == STOP_GROUPS && Chassis_Is_Stopped()))) ||
        (mode == TURN && (uint32_t)(now_ms - turn_started_ms) >= CHASSIS_TURN_TIMEOUT_MS))
        Chassis_Disable();
    /* 任务按 5 ms 的 RTOS 节拍唤醒。微秒抖动不应把名义 10 ms 的控制拍推到 15 ms。 */
    if ((TickType_t)(now_tick - last_control_tick) < pdMS_TO_TICKS(CONTROL_PERIOD_MS))
        return;
    const uint32_t now_us = __HAL_TIM_GET_COUNTER(&htim2);
    if (now_us == last_control_us)
        return;
    const uint32_t elapsed_us = now_us - step_start_us;
    last_control_tick = now_tick;
    last_control_us = step_start_us = now_us;

    /* 沿旧 Fill_Planar_Telemetry 先还原几何轮速，再正解最近反馈的车体速度。 */
    float wheel_feedback[3];
    for (unsigned i = 0U; i < 3U; ++i)
        wheel_feedback[i] = dm_motor[i].velocity * motor_direction[i] / motor_gear_ratio[i];
    Motor_Chassis_Forward(wheel_feedback, chassis_speed);

    if (pending_disable)
    {
        if (HAL_CAN_GetTxMailboxesFreeLevel(&MOTOR_CAN) != 3U)
            return;
        uint8_t sent = 1U;
        for (unsigned i = 0U; i < 3U; ++i)
        {
            if (disable_motor(&MOTOR_CAN, motor_id[i]) != HAL_OK)
                sent = 0U;
            else if (i < 2U)
                vTaskDelay(pdMS_TO_TICKS(1U));
        }
        if (sent)
            pending_disable = 0U;
        return;
    }
    if (!enabled)
        return;

    if (HAL_CAN_GetTxMailboxesFreeLevel(&MOTOR_CAN) != 3U)
    {
        Chassis_Disable();
        return;
    }

    /* 沿旧零 MIT→FC→零 MIT；F407 每次只容纳三个邮箱，分组等待下一周期。 */
    if (pending_enable)
    {
        for (unsigned i = 0U; i < 3U; ++i)
        {
            if (enable_motor(&MOTOR_CAN, motor_id[i]) != HAL_OK)
            {
                Chassis_Disable();
                return;
            }
            if (i < 2U)
                vTaskDelay(pdMS_TO_TICKS(1U));
        }
        pending_enable = 0U; /* 下周期仍在 IDLE，提交首组使能后的零 MIT。 */
        return;
    }

    INS_control_data_t imu;
    if (mode != IDLE && !Read_IMU(&imu))
    {
        Chassis_Disable();
        return;
    }
    float dt = elapsed_us * 0.000001f;
    float vx = 0.0f, vy = 0.0f, wz = 0.0f;
    if (mode == TRANSLATION)
    {
        ramp_speed = (float)fmin((double)target_speed,
                                (double)ramp_speed + (double)CHASSIS_STRAIGHT_ACCELERATION_MPS2 * dt);
    }
    else if (mode == DISTANCE)
    {
        if (!run_started)
        {
            run_started_ms = now_ms;
            run_started = 1U;
        }
        const double t = (uint32_t)(now_ms - run_started_ms) * 0.001;
        if (t >= profile_end_s)
            Chassis_Stop();
        else
        {
            /* 一帧速度保持到下一控制拍。取这 10 ms 的平均指令速度，
             * 加减速交界和末尾不足一拍的路程也计入。实时时刻决定轨迹进度。 */
            const double period_s = CONTROL_PERIOD_MS * 0.001;
            ramp_speed = (float)((Distance_Position(t + period_s) - Distance_Position(t)) / period_s);
        }
    }
    if (mode == TRANSLATION || mode == DISTANCE)
    {
        Chassis_Translation_Target(translation_direction, ramp_speed, &vx, &vy);
        wz = Hold_Yaw(&imu, dt);
        if (distance_turn_blend)
        {
            float speed_step = CHASSIS_TURN_ACCEL_RAD_S2 * dt;
            float delta = wz - turn_speed;
            if (fabsf(delta) <= speed_step)
                distance_turn_blend = 0U;
            turn_speed += fmaxf(-speed_step, fminf(speed_step, delta));
            wz = turn_speed;
        }
    }
    else if (mode == VELOCITY)
    {
        const float target_wz = vector_target[2] == 0.0f ? Hold_Yaw(&imu, dt) : vector_target[2];
        Service_Vector_Run(target_wz, dt);
        vx = vector_ramp[0];
        vy = vector_ramp[1];
        wz = vector_ramp[2];
    }
    else if (mode == TURN)
        wz = Service_Turn(&imu, dt, now_ms);

    if (!Send_Chassis_Targets(vx, vy, wz))
        return;
    if (mode == IDLE)
    {
        Update_Stop_Feedback(!monitoring_stop);
        monitoring_stop = 1U;
    }
    else if (mode == TRANSLATION && !run_started && ramp_speed > 0.0f)
    {
        run_started = 1U;
        run_started_ms = (uint32_t)xTaskGetTickCount();
    }
}
