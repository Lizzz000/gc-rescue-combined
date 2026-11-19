/* 来源：旧车 chassis_straight/chassis_translation/chassis_turn 的动作计算与参数。
 * 设置目标、周期计算后提交 MIT 的组织方式参考 basic_framework；三轮几何沿旧车，
 * 不使用其四轮公式。本文件仅保留动作状态，不迁移输入仲裁、租期或整套故障框架。
 * F407 适配：三个 CAN 发送邮箱，三帧一组；取消异步完成后才提交失能帧。
 * 本次 F407 收发实测适配：成功帧之间让出 1 tick，留出电机反馈时间，已架空验证；
 * 此间隔不是旧车参数，10 ms 控制节拍、PID 和轮序参数保持不变。
 */
#include "chassis.h"
#include "chassis_odometry.h"
#include "motor_chassis.h"
#include "velocity_profile.h"
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

typedef enum { IDLE, TRANSLATION, DISTANCE, VELOCITY, TURN, HOLDING } Chassis_Mode_t;

/* ID1后轮、ID2右前轮、ID3左前轮；方向、齿比与MIT速度阻尼沿实测配置。 */
static const uint16_t motor_id[3] = {1U, 2U, 3U};
static const float motor_direction[3] = {-1.0f, -1.0f, -1.0f};
static const float motor_gear_ratio[3] = {1.0f, 1.0f, 1.0f};
static const float motor_kd_default[3] = {0.22f, 0.27f, 0.22f};
/* 2026-11-16：1.8 kg、0.25 m/s 前后与双向旋转复测。摩擦项只用于平移。 */
static const float motor_friction_default[3] = {0.0f, 0.015f, 0.015f};
static const float motor_vmax[3] = {40.0f, 40.0f, 40.0f};
float chassis_speed[3];
float chassis_tuning_kd[3];
float chassis_tuning_ff_gain;
float chassis_tuning_friction[3];
float chassis_motor_target[3], chassis_motor_ff[3];
static float last_body_target[3];
static float ff_control_interval_s;
static uint32_t ff_previous_us;
static float ff_previous_yaw_rad;
static uint8_t ff_history_valid, ff_yaw_valid;
static pid_type_def yaw_pid, turn_pid;
static uint8_t enabled, pending_disable, pending_enable;
static uint8_t waiting_for_mailboxes;
static uint32_t mailboxes_busy_since_ms;
static uint8_t awaiting_enable_feedback;
static uint32_t enable_feedback_seq[3];
static uint32_t disable_feedback_seq[3];
static uint8_t awaiting_disable_feedback, disable_confirmed;
static uint8_t monitoring_stop, stop_groups, disable_after_stop;
static uint32_t stop_seq[3], last_stop_group_ms;
static Chassis_Mode_t mode;
static TickType_t last_control_tick;
static uint32_t last_control_us, step_start_us;
static Chassis_Translation_Direction_t translation_direction;
static float target_speed, ramp_speed, target_yaw_deg;
static uint32_t run_duration_ms, run_started_ms;
static uint8_t run_started;
static Odometry_State_t odometry;
#if CHASSIS_TUNING_ENABLED
/* 只在显式测试固件中保留；停止后读取，运动中不增加 SWD 遥测。 */
volatile uint32_t chassis_odometry_last_us, chassis_odometry_max_us;
volatile float chassis_heading_debug[6];
#endif
static uint32_t odometry_seq[3];
static uint32_t odometry_imu_sample_ms;
static double distance_origin_x, distance_origin_y, distance_goal_m, distance_progress_m;
static float distance_axis_x, distance_axis_y, distance_cross_speed;
static uint32_t distance_timeout_ms, distance_progress_ms, distance_stop_ms;
static uint8_t distance_completion_pending;
static float vector_target[3], vector_smoothing_s;
static Velocity_Profile_t velocity_profile;
static uint8_t vector_hold_yaw, vector_world_frame;
static float command_yaw_rad;
static float yaw_rate_filtered;
static uint8_t yaw_rate_filter_valid;
#if CHASSIS_TUNING_ENABLED
static uint8_t vector_calibration_spin;
#endif
static float turn_requested_deg, turn_accumulated_deg, turn_last_yaw_deg;
static float turn_speed;
static uint8_t distance_turn_blend;
static int8_t turn_direction;
static uint8_t turn_in_window, turn_zero_submitted, turn_imu_quiet;
static uint32_t turn_started_ms, turn_quiet_sample_ms;

static uint8_t Send_Chassis_Targets(float vx, float vy, float wz);
static double Distance_Progress(void);
static void Begin_Fault_Stop(uint32_t now_ms);

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
 * @note INS 发布时已检查数值，此处同时保证动作入口和里程计不接受非法快照。
 */
static uint8_t Read_IMU(INS_control_data_t *imu)
{
    return INS_Get_Data(imu) && isfinite(imu->yaw_deg) && isfinite(imu->world_z_rad_s) &&
           (uint32_t)((uint32_t)xTaskGetTickCount() - imu->sample_time_ms) <= CHASSIS_ODOMETRY_MAX_IMU_AGE_MS;
}

uint8_t Chassis_Attitude_Ready(void)
{
    INS_control_data_t imu;
    return Read_IMU(&imu);
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
static uint8_t Stopped_Feedback(void)
{
    if (!enabled || mode != IDLE || !monitoring_stop)
        return 0U;
    Update_Stop_Feedback(0U);
    return stop_groups == STOP_GROUPS;
}

static uint8_t Distance_Endpoint_Valid(void)
{
    Chassis_Odometry_t data;
    return !distance_completion_pending || (Chassis_Get_Odometry(&data) &&
        fabs(distance_goal_m - Distance_Progress()) <= CHASSIS_DISTANCE_TOLERANCE_M);
}

/** @brief 正常待命可报告停稳；故障制动中的轮速确认供 Loop 决定何时失能。 */
uint8_t Chassis_Is_Stopped(void)
{
    const uint8_t stopped = !disable_after_stop && Stopped_Feedback() && Distance_Endpoint_Valid();
    if (stopped)
        distance_completion_pending = 0U;
    return stopped;
}

static uint8_t Zero_Target_Ready(INS_control_data_t *imu, uint8_t allow_holding)
{
    return enabled && !disable_after_stop &&
        ((mode == IDLE && monitoring_stop) || (allow_holding && mode == HOLDING)) &&
        Chassis_Motors_Ready() && Read_IMU(imu) && Distance_Endpoint_Valid();
}

uint8_t Chassis_Can_Release(void)
{
    INS_control_data_t imu;
    return Zero_Target_Ready(&imu, 1U);
}

/** @brief 指令已回零并成功提交，反馈与姿态可用于下一段。轮子允许仍在减速。 */
static uint8_t Motion_Ready(INS_control_data_t *imu)
{
    if (!Zero_Target_Ready(imu, 0U))
        return 0U;
    awaiting_enable_feedback = 0U;
    distance_completion_pending = 0U;
    return 1U;
}

uint8_t Chassis_Motion_Ready(void)
{
    INS_control_data_t imu;
    return Motion_Ready(&imu);
}

/** @brief 用与反馈 ISR 相同的时基核对三轮使能状态与反馈年龄；约束见头文件。 */
uint8_t Chassis_Motors_Ready(void)
{
    uint8_t ready = 1U;
    taskENTER_CRITICAL();
    uint32_t now_ms = (uint32_t)xTaskGetTickCount();
    for (unsigned i = 0U; i < 3U; ++i)
        if (dm_motor[i].id != motor_id[i] || dm_motor[i].state != 1U ||
            (awaiting_enable_feedback && dm_motor[i].feedback_seq == enable_feedback_seq[i]) ||
            !isfinite(dm_motor[i].velocity) ||
            (uint32_t)(now_ms - dm_motor[i].feedback_time_ms) > CHASSIS_MOTOR_FEEDBACK_TIMEOUT_MS)
            ready = 0U;
    taskEXIT_CRITICAL();
    return ready;
}

/** @brief 从同一控制任务复制里程计；最近样本、三轮和姿态均须有效。 */
uint8_t Chassis_Get_Odometry(Chassis_Odometry_t *data)
{
    if (data == NULL)
        return 0U;
    INS_control_data_t imu;
    *data = odometry.data;
    data->valid = data->valid &&
        (uint32_t)(__HAL_TIM_GET_COUNTER(&htim2) - data->sample_time_us) <=
            CHASSIS_MOTOR_FEEDBACK_TIMEOUT_MS * 1000U &&
        Chassis_Motors_Ready() && Read_IMU(&imu);
    data->yaw_rate_rad_s = data->valid ? CHASSIS_STRAIGHT_YAW_SIGN * imu.world_z_rad_s : 0.0f;
    return data->valid;
}

/* 只补偿控制向量的旋转时延。里程计位置和到位判定仍由实测反馈推进。 */
static float Predict_Command_Yaw(const Chassis_Odometry_t *pose, uint32_t now_us, float yaw_rate)
{
    const uint32_t age_us = now_us - pose->sample_time_us;
    if (age_us > CHASSIS_MOTOR_FEEDBACK_TIMEOUT_MS * 1000U)
        return pose->yaw_rad;
    return pose->yaw_rad + (yaw_rate - odometry.heading.bias_rad_s) * (age_us * 0.000001f);
}

/** @brief 只在没有执行动作时重置平移原点，保持现有 INS 的航向参考。 */
uint8_t Chassis_Reset_Odometry(void)
{
    if (pending_disable || (enabled && !Chassis_Is_Stopped()))
        return 0U;
    Odometry_Init(&odometry);
    distance_completion_pending = 0U;
    for (unsigned i = 0U; i < 3U; ++i)
        chassis_speed[i] = 0.0f;
    return 1U;
}

/** @brief 短临界区复制三轮一致快照；每轮都有新帧后才积分一次。
 * @note 仅复制时屏蔽 CAN 中断；正解、三角函数与积分都在临界区外。
 *       断流或长调度间隔撤销连续性，恢复首点不补算缺失路程。
 */
static void Update_Odometry(uint32_t now_us,
                            const INS_control_data_t *imu, uint8_t imu_valid)
{
    dm_motor_t feedback[3];
    taskENTER_CRITICAL();
    const uint32_t now_ms = (uint32_t)xTaskGetTickCount();
    for (unsigned i = 0U; i < 3U; ++i)
        feedback[i] = dm_motor[i];
    taskEXIT_CRITICAL();
    uint8_t ready = imu_valid, fresh_group = 1U;
    float wheel[3], position[3];
    Odometry_Feedback_Timing_t timing = {.control_time_ms = now_ms};
    for (unsigned i = 0U; i < 3U; ++i)
    {
        if (feedback[i].id != motor_id[i] || feedback[i].state != 1U ||
            !isfinite(feedback[i].position) || !isfinite(feedback[i].velocity) ||
            fabsf(feedback[i].velocity) > motor_vmax[i] ||
            (uint32_t)(now_ms - feedback[i].feedback_time_ms) > CHASSIS_MOTOR_FEEDBACK_TIMEOUT_MS)
            ready = 0U;
        if (feedback[i].feedback_seq == odometry_seq[i])
            fresh_group = 0U;
        wheel[i] = feedback[i].velocity * motor_direction[i] / motor_gear_ratio[i];
        position[i] = feedback[i].position * motor_direction[i] / motor_gear_ratio[i];
        timing.wheel_time_ms[i] = feedback[i].feedback_time_ms;
    }
    if (!ready)
    {
        Odometry_Invalidate(&odometry);
        for (unsigned i = 0U; i < 3U; ++i)
            odometry_seq[i] = feedback[i].feedback_seq;
    }
    else if (fresh_group)
    {
#if CHASSIS_TUNING_ENABLED
        const uint32_t algorithm_start_us = __HAL_TIM_GET_COUNTER(&htim2);
#endif
        const uint8_t observe_yaw = !imu->yaw_predicted &&
            (!odometry.initialized || imu->sample_time_ms != odometry_imu_sample_ms);
        timing.imu_time_ms = imu->sample_time_ms;
        if (Odometry_Update_Feedback(&odometry, position, wheel, &timing,
                CHASSIS_STRAIGHT_YAW_SIGN * imu->yaw_deg * DEG_TO_RAD,
                CHASSIS_STRAIGHT_YAW_SIGN * imu->world_z_rad_s, now_us, observe_yaw) && observe_yaw)
            odometry_imu_sample_ms = imu->sample_time_ms;
#if CHASSIS_TUNING_ENABLED
        chassis_odometry_last_us = __HAL_TIM_GET_COUNTER(&htim2) - algorithm_start_us;
        if (chassis_odometry_last_us > chassis_odometry_max_us)
            chassis_odometry_max_us = chassis_odometry_last_us;
        chassis_heading_debug[0] = odometry.heading.yaw_rad;
        chassis_heading_debug[1] = odometry.heading.bias_rad_s;
        chassis_heading_debug[2] = odometry.heading.p_yaw;
        chassis_heading_debug[3] = odometry.heading.p_bias;
        chassis_heading_debug[4] = odometry.heading.yaw_gain;
        chassis_heading_debug[5] = odometry.heading.wheel_bias_gain;
#endif
        for (unsigned i = 0U; i < 3U; ++i)
            odometry_seq[i] = feedback[i].feedback_seq;
    }
    else if ((uint32_t)(now_us - odometry.data.sample_time_us) >
             CHASSIS_MOTOR_FEEDBACK_TIMEOUT_MS * 1000U)
        Odometry_Invalidate(&odometry);
    chassis_speed[0] = odometry.data.valid ? odometry.data.vx_mps : 0.0f;
    chassis_speed[1] = odometry.data.valid ? odometry.data.vy_mps : 0.0f;
    chassis_speed[2] = odometry.data.valid ? odometry.data.wheel_wz_rad_s : 0.0f;
}

/** @brief 沿旧 Drive_Enter_Zero 清除运动目标和 PID，保留电机使能状态。
 * @return 无；只更新本任务的软件状态。
 */
void Chassis_Stop(void)
{
    if (mode != IDLE)
        monitoring_stop = 0U;
    mode = IDLE;
    target_speed = ramp_speed = distance_cross_speed = 0.0f;
    turn_speed = 0.0f;
    turn_in_window = turn_zero_submitted = turn_imu_quiet = 0U;
    distance_turn_blend = 0U;
    distance_completion_pending = 0U;
    vector_hold_yaw = vector_world_frame = 0U;
    yaw_rate_filter_valid = 0U;
#if CHASSIS_TUNING_ENABLED
    vector_calibration_spin = 0U;
#endif
    run_duration_ms = 0U;
    run_started = 0U;
    Velocity_Profile_Seed(&velocity_profile, 0.0f, 0.0f, 0.0f);
    for (unsigned i = 0U; i < 3U; ++i)
    {
        vector_target[i] = 0.0f;
        chassis_motor_ff[i] = 0.0f;
    }
    ff_history_valid = 0U;
    PID_clear(&yaw_pid);
    PID_clear(&turn_pid);
}

void Chassis_Hold(void)
{
    if (mode == IDLE || mode == HOLDING)
        return;
    float vx = last_body_target[0], vy = last_body_target[1];
    if (vector_world_frame && ff_yaw_valid)
    {
        const float cosine = cosf(ff_previous_yaw_rad), sine = sinf(ff_previous_yaw_rad);
        vx = cosine * last_body_target[0] - sine * last_body_target[1];
        vy = sine * last_body_target[0] + cosine * last_body_target[1];
    }
    Velocity_Profile_Seed(&velocity_profile, vx, vy, last_body_target[2]);
    for (unsigned i = 0; i < 3; ++i)
        vector_target[i] = 0.0f;
    vector_hold_yaw = 0U;
    vector_smoothing_s = CHASSIS_VECTOR_SMOOTHING_S;
    mode = HOLDING;
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
    enabled = pending_disable = pending_enable = awaiting_enable_feedback = waiting_for_mailboxes = 0U;
    awaiting_disable_feedback = disable_confirmed = 0U;
    monitoring_stop = stop_groups = disable_after_stop = 0U;
    Chassis_Stop();
    Odometry_Init(&odometry);
    for (unsigned i = 0U; i < 3U; ++i)
    {
        odometry_seq[i] = 0U;
        chassis_speed[i] = 0.0f;
        last_body_target[i] = 0.0f;
        chassis_tuning_kd[i] = motor_kd_default[i];
        chassis_tuning_friction[i] = motor_friction_default[i];
        chassis_motor_target[i] = chassis_motor_ff[i] = 0.0f;
    }
    chassis_tuning_ff_gain = 1.0f;
    Reset_Control_Time();
}

/** @brief 故障先用现有零 MIT 制动；停稳或 500 ms 后失能，不报告正常完成。 */
static void Begin_Fault_Stop(uint32_t now_ms)
{
    Chassis_Stop();
    if (!disable_after_stop)
    {
        disable_after_stop = 1U;
        run_started_ms = now_ms;
    }
}

/** @brief 在失能或已确认停稳时原子受理整组调参；非法输入不部分生效。 */
uint8_t Chassis_Set_Tuning(const float kd[3], float ff_gain, const float friction[3])
{
    if (kd == NULL || friction == NULL || !isfinite(ff_gain) || ff_gain < 0.0f || ff_gain > 1.0f ||
        (enabled && !Chassis_Is_Stopped()))
        return 0U;
    for (unsigned i = 0U; i < 3U; ++i)
        if (!isfinite(kd[i]) || kd[i] < 0.05f || kd[i] > 0.5f ||
            !isfinite(friction[i]) || friction[i] < 0.0f || friction[i] > 0.03f)
            return 0U;
    for (unsigned i = 0U; i < 3U; ++i)
    {
        chassis_tuning_kd[i] = kd[i];
        chassis_tuning_friction[i] = friction[i];
    }
    chassis_tuning_ff_gain = ff_gain;
    ff_history_valid = 0U;
    return 1U;
}

/** @brief 清目标并撤销待发帧，失能帧交由 Loop 等待三个邮箱空闲后提交。
 * @return 无；沿旧停机先取消、后 FD 的顺序，适配 F407 异步取消。
 */
void Chassis_Disable(void)
{
    enabled = 0U;
    waiting_for_mailboxes = 0U;
    pending_enable = 0U;
    monitoring_stop = stop_groups = disable_after_stop = 0U;
    Chassis_Stop();
    pending_disable = 1U;
    awaiting_disable_feedback = disable_confirmed = 0U;
    HAL_CAN_AbortTxRequest(&MOTOR_CAN, CAN_TX_MAILBOX0 | CAN_TX_MAILBOX1 | CAN_TX_MAILBOX2);
}

uint8_t Chassis_Disable_Confirmed(void)
{
    if (enabled)
        return 0U;

    const uint32_t now_ms = (uint32_t)xTaskGetTickCount();
    taskENTER_CRITICAL();
    if (awaiting_disable_feedback)
    {
        uint8_t confirmed = 1U;
        for (unsigned i = 0U; i < 3U; ++i)
        {
            if (dm_motor[i].id != motor_id[i] || dm_motor[i].state != 0U ||
                dm_motor[i].feedback_seq == disable_feedback_seq[i] ||
                (uint32_t)(now_ms - dm_motor[i].feedback_time_ms) > CHASSIS_MOTOR_FEEDBACK_TIMEOUT_MS)
                confirmed = 0U;
        }
        if (confirmed)
        {
            disable_confirmed = 1U;
            awaiting_disable_feedback = 0U;
        }
    }
    for (unsigned i = 0U; i < 3U; ++i)
    {
        if (dm_motor[i].id == motor_id[i] && dm_motor[i].state != 0U)
            disable_confirmed = 0U;
    }
    const uint8_t confirmed = disable_confirmed;
    taskEXIT_CRITICAL();
    return confirmed;
}

uint8_t Chassis_Can_Enable(void)
{
    return !enabled && !pending_disable && HAL_CAN_GetTxMailboxesFreeLevel(&MOTOR_CAN) == 3U;
}

/** @brief 沿旧 Queue_Enable_Command 先提交零 MIT，再由 Loop 分周期提交 FC/零 MIT。
 * @return 首组三帧提交成功/已经接受使能返回 1，否则 0。
 */
uint8_t Chassis_Enable(void)
{
    if (enabled)
        return 1U;
    if (!Chassis_Can_Enable())
        return 0U;
    if (!Send_Chassis_Targets(0.0f, 0.0f, 0.0f))
        return 0U;
    Chassis_Stop();
    enabled = 1U;
    awaiting_disable_feedback = disable_confirmed = 0U;
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
    if (direction < CHASSIS_TRANSLATION_FORWARD || direction > CHASSIS_TRANSLATION_BACKWARD_RIGHT ||
        !Motion_Ready(&imu))
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

/** @brief 以有效轮速里程计为起点接入定距，参数及返回值见 chassis.h。
 * @note 理想梯形时长只用于故障总时限，不再按时间或指令积分报告到达。
 *       本车连续往返适配：转向剩余 5 度内允许接入定距，保留原转向终点航向，
 *       由原直行 PID 在行驶中修偏；角速度沿原斜坡接管，不经过停车状态。
 */
uint8_t Chassis_Distance_Start(Chassis_Translation_Direction_t direction,
                               float distance_m, float speed_limit_mps)
{
    if (!isfinite(distance_m) || distance_m < CHASSIS_DISTANCE_MIN_M || !isfinite(speed_limit_mps) ||
        speed_limit_mps <= 0.0f || speed_limit_mps > CHASSIS_DISTANCE_MAX_SPEED_MPS)
        return 0U;
    const double accel = CHASSIS_DISTANCE_ACCELERATION_MPS2;
    const double peak = fmin(speed_limit_mps, sqrt(accel * distance_m));
    const double rise_s = peak / accel;
    const double end_s = distance_m / peak + rise_s;
    const double timeout_ms = end_s * CHASSIS_DISTANCE_TIMEOUT_FACTOR * 1000.0 +
                              CHASSIS_DISTANCE_TIMEOUT_MARGIN_MS;
    Chassis_Odometry_t origin;
    if (timeout_ms > INT32_MAX || !Chassis_Get_Odometry(&origin))
        return 0U;
    const uint32_t now_us = __HAL_TIM_GET_COUNTER(&htim2);
    float heading = Predict_Command_Yaw(&origin, now_us, origin.yaw_rate_rad_s);
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
        const float remaining = turn_requested_deg - turn_accumulated_deg - delta;
        const uint32_t imu_age_ms = (uint32_t)xTaskGetTickCount() - imu.sample_time_ms;
        if (imu_age_ms > CHASSIS_ODOMETRY_MAX_IMU_AGE_MS)
            return 0U;
        const float remaining_rad = remaining * DEG_TO_RAD -
            CHASSIS_STRAIGHT_YAW_SIGN * imu.world_z_rad_s * (imu_age_ms * 0.001f);
        if (fabsf(remaining_rad) > CHASSIS_TURN_FINE_ANGLE_DEG * DEG_TO_RAD)
            return 0U;
        float previous_speed = last_body_target[2];
        Chassis_Stop();
        translation_direction = direction;
        target_yaw_deg = remainderf(yaw + remaining, 360.0f);
        heading += remaining_rad;
        turn_speed = previous_speed;
        distance_turn_blend = 1U;
        /* 运行中衔接保留控制时基；首个定距目标仍在下一控制拍提交。 */
    }
    else if (!Prepare_Translation(direction))
        return 0U;
    distance_goal_m = distance_m;
    distance_origin_x = origin.x_m;
    distance_origin_y = origin.y_m;
    float ux, uy;
    Chassis_Translation_Target(direction, 1.0f, &ux, &uy);
    const float cosine = cosf(heading), sine = sinf(heading);
    distance_axis_x = cosine * ux - sine * uy;
    distance_axis_y = sine * ux + cosine * uy;
    target_speed = speed_limit_mps;
    distance_timeout_ms = (uint32_t)ceil(timeout_ms);
    vector_world_frame = 1U;
    mode = DISTANCE;
    return 1U;
}

/** @brief 返回实测位移在锁定方向的有符号投影；横移、绕圈不能冒充前进。 */
static double Distance_Progress(void)
{
    return (odometry.data.x_m - distance_origin_x) * distance_axis_x +
           (odometry.data.y_m - distance_origin_y) * distance_axis_y;
}

/** @brief 剩余位移决定减速与末段修正，保持原线加速度限制。
 * @note 位置到位后提交零目标，调用者可按里程终点接续或等待三轮实际停车。
 */
static void Service_Distance(float dt, uint32_t now_ms)
{
    if (!run_started)
    {
        run_started_ms = distance_progress_ms = now_ms;
        distance_progress_m = 0.0;
        run_started = 1U;
    }
    const double remaining = distance_goal_m - Distance_Progress();
    if (remaining <= CHASSIS_DISTANCE_STOP_REMAINING_M)
    {
        Chassis_Hold();
        distance_completion_pending = 1U;
        return;
    }
    const float braking_speed = sqrtf(2.0f * CHASSIS_DISTANCE_ACCELERATION_MPS2 * (float)remaining);
    const float approach_speed = fmaxf(CHASSIS_DISTANCE_POSITION_KP * (float)remaining,
                                      CHASSIS_DISTANCE_APPROACH_SPEED_MPS);
    const float desired_speed = fminf(target_speed,
        fminf(braking_speed, approach_speed));
    /* 横向修正逐步收敛，和沿程速度共用制动及合加速度额度。 */
    const double cross_track = (odometry.data.y_m - distance_origin_y) * distance_axis_x -
                               (odometry.data.x_m - distance_origin_x) * distance_axis_y;
    const float cross_limit = CHASSIS_DISTANCE_CROSS_SPEED_RATIO * desired_speed;
    const float cross_velocity = -CHASSIS_DISTANCE_CROSS_GAIN_PER_S * (float)cross_track;
    const float desired_cross = fmaxf(-cross_limit, fminf(cross_limit, cross_velocity));
    const float desired_along = sqrtf(fmaxf(0.0f, desired_speed * desired_speed - desired_cross * desired_cross));
    const float along_error = desired_along - ramp_speed;
    const float cross_error = desired_cross - distance_cross_speed;
    const float speed_error = hypotf(along_error, cross_error);
    const float step = CHASSIS_DISTANCE_ACCELERATION_MPS2 * dt;
    if (speed_error > step)
    {
        const float fraction = step / speed_error;
        ramp_speed += fraction * along_error;
        distance_cross_speed += fraction * cross_error;
    }
    else
    {
        ramp_speed = desired_along;
        distance_cross_speed = desired_cross;
    }
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
    if ((direction != 1 && direction != -1) || !isfinite(angle_deg) ||
        angle_deg <= 0.0f || angle_deg > 360.0f || !Motion_Ready(&imu))
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
 * @param[in] wz 逆时针角速度，rad/s，绝对值不超过 4.0。
 * @return 请求接受返回 1，否则 0；全零请求回零速待命。
 * @note 连续更新平移目标保留航向；显式转向结束后按当前姿态重新锁定，斜坡连续。
 */
static uint8_t Set_Velocity(float vx, float vy, float wz, uint8_t hold_yaw, uint8_t world_frame,
                            float smoothing_s)
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
    if ((mode == HOLDING && (distance_completion_pending || !Chassis_Motors_Ready())) ||
        ((mode == VELOCITY || mode == HOLDING) ? !Read_IMU(&imu) : !Motion_Ready(&imu)))
        return 0U;
    Chassis_Odometry_t pose;
    if (world_frame && !Chassis_Get_Odometry(&pose))
        return 0U;
    if (world_frame != vector_world_frame)
    {
        float x = last_body_target[0], y = last_body_target[1];
        if (world_frame)
        {
            const float yaw = ff_history_valid && ff_yaw_valid ? ff_previous_yaw_rad : pose.yaw_rad;
            const float cosine = cosf(yaw), sine = sinf(yaw);
            x = cosine * last_body_target[0] - sine * last_body_target[1];
            y = sine * last_body_target[0] + cosine * last_body_target[1];
        }
        Velocity_Profile_Seed(&velocity_profile, x, y, last_body_target[2]);
    }
    else if (mode == HOLDING && !world_frame)
        Velocity_Profile_Seed(&velocity_profile, last_body_target[0], last_body_target[1], last_body_target[2]);
    if (mode == IDLE)
        step_start_us = __HAL_TIM_GET_COUNTER(&htim2);
    if (hold_yaw && wz == 0.0f && (mode != VELOCITY || vector_target[2] != 0.0f || !vector_hold_yaw))
    {
        target_yaw_deg = CHASSIS_STRAIGHT_YAW_SIGN * imu.yaw_deg;
        PID_clear(&yaw_pid);
    }
    vector_target[0] = vx;
    vector_target[1] = vy;
    vector_target[2] = wz;
    vector_hold_yaw = hold_yaw;
    vector_world_frame = world_frame;
    vector_smoothing_s = smoothing_s;
#if CHASSIS_TUNING_ENABLED
    vector_calibration_spin = 0U;
#endif
    mode = VELOCITY;
    return 1U;
}

uint8_t Chassis_Set_Velocity(float vx, float vy, float wz)
{
    return Set_Velocity(vx, vy, wz, 1U, 0U, CHASSIS_VECTOR_SMOOTHING_S);
}

uint8_t Chassis_Set_Velocity_Explicit_Yaw(float vx, float vy, float wz)
{
    return Set_Velocity(vx, vy, wz, 0U, 0U, CHASSIS_VECTOR_SMOOTHING_S);
}

uint8_t Chassis_Set_World_Velocity(float vx, float vy, float wz)
{
    return Set_Velocity(vx, vy, wz, 0U, 1U, CHASSIS_VECTOR_SMOOTHING_S);
}

#if CHASSIS_TUNING_ENABLED
uint8_t Chassis_Set_Test_Rotation(float wz)
{
    /* 专项标定需要直接观察速度指令与实际响应。 */
    if (!Set_Velocity(0.0f, 0.0f, wz, 0U, 0U, 0.0f))
        return 0U;
    vector_calibration_spin = wz != 0.0f;
    return 1U;
}
#endif

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
    /* 只对已限幅、实际提交的车体平移目标求导。长间隔不补偿累计误差，
     * Stop/故障/使能包序始终零前馈；未标定的偏航惯性项保持零。 */
    const uint32_t ff_now_us = __HAL_TIM_GET_COUNTER(&htim2);
    const float ff_dt = ff_history_valid ? (ff_now_us - ff_previous_us) * 0.000001f : ff_control_interval_s;
    float fx = 0.0f, fy = 0.0f;
    if (mode != IDLE && ff_dt > 0.0f && ff_dt <= 0.05f)
    {
        float previous_x = last_body_target[0], previous_y = last_body_target[1];
        if (vector_world_frame && ff_history_valid && ff_yaw_valid)
        {
            /* 用同一车体系求导，车身旋转造成的分量变化不计作平移加速度。 */
            const float angle = command_yaw_rad - ff_previous_yaw_rad;
            const float cosine = cosf(angle), sine = sinf(angle);
            previous_x = cosine * last_body_target[0] + sine * last_body_target[1];
            previous_y = -sine * last_body_target[0] + cosine * last_body_target[1];
        }
        float ax = (vx * scale - previous_x) / ff_dt;
        float ay = (vy * scale - previous_y) / ff_dt;
        const float acceleration = hypotf(ax, ay);
        if (acceleration > CHASSIS_DISTANCE_ACCELERATION_MPS2)
        {
            ax *= CHASSIS_DISTANCE_ACCELERATION_MPS2 / acceleration;
            ay *= CHASSIS_DISTANCE_ACCELERATION_MPS2 / acceleration;
        }
        fx = CHASSIS_FEEDFORWARD_MASS_KG * ax * chassis_tuning_ff_gain;
        fy = CHASSIS_FEEDFORWARD_MASS_KG * ay * chassis_tuning_ff_gain;
    }
    /* 沿后/右前/左前切向分配平移力，三轮合力为 (fx,fy)，合转矩为零。 */
    const float wheel_torque[3] = {
        CHASSIS_WHEEL_RADIUS * (-2.0f * fy / 3.0f),
        CHASSIS_WHEEL_RADIUS * (fx / 1.7320508f + fy / 3.0f),
        CHASSIS_WHEEL_RADIUS * (-fx / 1.7320508f + fy / 3.0f)
    };
    for (unsigned i = 0U; i < 3U; ++i)
    {
        float target = wheel[i] * scale * motor_gear_ratio[i] * motor_direction[i];
        float ff = 0.0f;
        if (mode != IDLE)
        {
            ff = wheel_torque[i] * motor_direction[i] / motor_gear_ratio[i] +
                 ((vx != 0.0f || vy != 0.0f) ?
                  chassis_tuning_friction[i] * target / (fabsf(target) + CHASSIS_FRICTION_SPEED_RAD_S) : 0.0f);
            ff = fmaxf(-CHASSIS_FEEDFORWARD_LIMIT_NM, fminf(CHASSIS_FEEDFORWARD_LIMIT_NM, ff));
        }
        if (ctrl_motor(&MOTOR_CAN, motor_id[i], 0.0f, target, 0.0f, chassis_tuning_kd[i], ff) != HAL_OK)
        {
            Chassis_Disable();
            return 0U;
        }
        chassis_motor_target[i] = target;
        chassis_motor_ff[i] = ff;
        if (i < 2U)
            vTaskDelay(pdMS_TO_TICKS(1U));
    }
    last_body_target[0] = vx * scale;
    last_body_target[1] = vy * scale;
    last_body_target[2] = wz * scale;
    ff_previous_us = ff_now_us;
    ff_previous_yaw_rad = command_yaw_rad;
    ff_yaw_valid = odometry.data.valid;
    ff_history_valid = mode != IDLE;
    return 1U;
}

/* 用陀螺反馈收回超速量。参考方向和原斜坡限制优先，零参考仍直接归零。 */
static float Apply_Yaw_Rate_Feedback(float reference, float measured, float dt, float max_acceleration)
{
    if (!yaw_rate_filter_valid)
    {
        yaw_rate_filtered = measured;
        yaw_rate_filter_valid = 1U;
    }
    else
        yaw_rate_filtered += dt / (CHASSIS_YAW_RATE_FILTER_S + dt) * (measured - yaw_rate_filtered);

    const float direction = copysignf(1.0f, reference);
    const float requested = fabsf(reference);
    const float excess = direction * yaw_rate_filtered - requested - CHASSIS_YAW_RATE_DEADBAND_RAD_S;
    const float reduction_limit = fminf(CHASSIS_YAW_RATE_MAX_REDUCTION_RAD_S,
                                         CHASSIS_YAW_RATE_MAX_REDUCTION_RATIO * requested);
    const float reduction = fminf(reduction_limit, CHASSIS_YAW_RATE_FEEDBACK_KP * fmaxf(excess, 0.0f));
    const float previous = direction * last_body_target[2];
    const float step = max_acceleration * dt;
    const float applied = previous + fmaxf(-step, fminf(step, requested - reduction - previous));
    return direction * fmaxf(0.0f, fminf(requested, applied));
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

/** @brief 沿旧 Chassis_Turn_Step 累计跨圈角度，在到角窗口内收回转向目标。
 * @param[in] imu 已验证的姿态快照，yaw 为度、角速度为 rad/s。
 * @param[in] dt 实际控制间隔，s，首拍同刻接单时可为 0。
 * @return 目标角速度，rad/s；入窗后先平滑归零，再等待实测停稳。
 * @note 2026-11-08 转向适配：远端用 omega²=2*alpha*剩余角规划制动速度，
 *       最后 5 度沿旧 PID 慢速修偏，按控制拍的实际 dt 限制角速度增量。
 *       制动规划为本车适配，不改变 PID 增益或电机 MIT 阻尼。
 */
static float Service_Turn(const INS_control_data_t *imu, float dt)
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
    /* 入窗后用稍宽的退出门槛，防止减速时在边缘反复恢复转向。小转角仍须产生实测进展。 */
    const float window = turn_in_window ?
        fminf(CHASSIS_TURN_ANGLE_EXIT_DEG, fabsf(turn_requested_deg) * 0.75f) :
        fminf(CHASSIS_TURN_ANGLE_TOLERANCE_DEG, fabsf(turn_requested_deg) * 0.5f);
    turn_in_window = fabs(remaining) <= window;
    if (turn_in_window)
        wz = 0.0f;
    float speed_step = CHASSIS_TURN_ACCEL_RAD_S2 * dt;
    turn_speed += fmaxf(-speed_step, fminf(speed_step, wz - turn_speed));
    if (!turn_in_window || turn_speed != 0.0f)
        turn_zero_submitted = turn_imu_quiet = 0U;
    return turn_speed;
}

/* 只在已提交零速时累计新轮反馈。角度漂出窗口会重新接入转向。 */
static void Confirm_Turn_Stop(const INS_control_data_t *imu)
{
    if (!turn_in_window || turn_speed != 0.0f)
        return;
    Update_Stop_Feedback(!turn_zero_submitted);
    turn_zero_submitted = 1U;
    if (imu->yaw_predicted || fabsf(imu->world_z_rad_s) > CHASSIS_TURN_STOP_RATE_RAD_S ||
        stop_groups == 0U)
    {
        turn_imu_quiet = 0U;
        return;
    }
    const uint32_t quiet_ms = imu->sample_time_ms - turn_quiet_sample_ms;
    if (!turn_imu_quiet || quiet_ms > INT32_MAX)
    {
        turn_quiet_sample_ms = imu->sample_time_ms;
        turn_imu_quiet = 1U;
        return;
    }
    if (stop_groups == STOP_GROUPS && quiet_ms >= CHASSIS_TURN_STOP_CONFIRM_MS)
    {
        Chassis_Stop();
        monitoring_stop = 1U;
    }
}

/** @brief 适配现有 5 ms RTOS 任务；沿旧 100 Hz 控制、实际 dt 和到期动作语义。
 * @return 无；不补发历史周期，不自动使能。
 */
void Chassis_Loop(void)
{
    const TickType_t now_tick = xTaskGetTickCount();
    const uint32_t now_ms = (uint32_t)now_tick;
    if (mode == IDLE && distance_completion_pending &&
        (uint32_t)(now_ms - distance_stop_ms) >= TIMED_BRAKE_LIMIT_MS && !Chassis_Is_Stopped())
        Chassis_Disable();
    if ((mode == DISTANCE || (mode == HOLDING && distance_completion_pending)) && run_started &&
        (uint32_t)(now_ms - run_started_ms) >= distance_timeout_ms)
        Begin_Fault_Stop(now_ms);
    if (mode == TRANSLATION && run_started && run_duration_ms != 0U &&
        (uint32_t)(now_ms - run_started_ms) >= run_duration_ms)
        Chassis_Hold();
    /* 故障制动须撤销使能；正常到时减速完成后持续发送零 MIT。 */
    if ((disable_after_stop && ((uint32_t)(now_ms - run_started_ms) >= TIMED_BRAKE_LIMIT_MS ||
                                Stopped_Feedback())) ||
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

    INS_control_data_t imu;
    const uint8_t imu_valid = Read_IMU(&imu);
    Update_Odometry(now_us, &imu, imu_valid);
    if (odometry.data.valid)
        command_yaw_rad = Predict_Command_Yaw(&odometry.data, now_us,
                                             CHASSIS_STRAIGHT_YAW_SIGN * imu.world_z_rad_s);
    if ((mode == HOLDING && (!imu_valid || !Chassis_Motors_Ready())) ||
        (vector_world_frame && !odometry.data.valid))
        Begin_Fault_Stop(now_ms);
    /* 里程计与故障判断先于邮箱检查：总线忙不能把过期反馈继续当作有效位置。
     * 到位后的滑行也须留在容差内，不能仅因发送过零速就报告成功。 */
    if ((mode == DISTANCE || distance_completion_pending) &&
        (!odometry.data.valid || (distance_completion_pending &&
         fabs(distance_goal_m - Distance_Progress()) > CHASSIS_DISTANCE_TOLERANCE_M) ||
         (mode == DISTANCE && Distance_Progress() > distance_goal_m + CHASSIS_DISTANCE_TOLERANCE_M)))
        Begin_Fault_Stop(now_ms);
    if (mode == DISTANCE && run_started)
    {
        const double progress = Distance_Progress();
        /* 低速限值也允许较小的进展量，不能把合法的慢行误判为堵转。 */
        if (progress >= distance_progress_m + fmin(CHASSIS_DISTANCE_PROGRESS_MIN_M, (double)target_speed * CHASSIS_DISTANCE_PROGRESS_TIME_S))
        {
            distance_progress_m = progress;
            distance_progress_ms = now_ms;
        }
        if ((uint32_t)(now_ms - distance_progress_ms) >= CHASSIS_DISTANCE_STALL_TIMEOUT_MS)
            Begin_Fault_Stop(now_ms);
    }

    if (pending_disable)
    {
        if (HAL_CAN_GetTxMailboxesFreeLevel(&MOTOR_CAN) != 3U)
            return;
        uint8_t sent = 1U;
        for (unsigned i = 0U; i < 3U; ++i)
        {
            /* 记录发送前的序号，已缓存的失能状态不能当作本次确认。 */
            taskENTER_CRITICAL();
            disable_feedback_seq[i] = dm_motor[i].feedback_seq;
            const HAL_StatusTypeDef result = disable_motor(&MOTOR_CAN, motor_id[i]);
            taskEXIT_CRITICAL();
            if (result != HAL_OK)
                sent = 0U;
            else if (i < 2U)
                vTaskDelay(pdMS_TO_TICKS(1U));
        }
        if (sent)
        {
            awaiting_disable_feedback = 1U;
            pending_disable = 0U;
        }
        return;
    }
    if (!enabled)
    {
        if (awaiting_disable_feedback)
            Chassis_Disable_Confirmed();
        return;
    }

    /* 短时忙跳过本拍；长期不能更新目标时撤销旧帧，避免保持上一组非零速度。 */
    if (HAL_CAN_GetTxMailboxesFreeLevel(&MOTOR_CAN) != 3U)
    {
        if (!waiting_for_mailboxes)
        {
            waiting_for_mailboxes = 1U;
            mailboxes_busy_since_ms = now_ms;
        }
        else if ((uint32_t)(now_ms - mailboxes_busy_since_ms) >= CHASSIS_CAN_TX_TIMEOUT_MS)
            Chassis_Disable();
        return;
    }
    waiting_for_mailboxes = 0U;

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
        /* 上一次使能留下的反馈可能仍在 30 ms 窗口内，须见到本轮 FC 后各轮的新帧。 */
        taskENTER_CRITICAL();
        for (unsigned i = 0U; i < 3U; ++i)
            enable_feedback_seq[i] = dm_motor[i].feedback_seq;
        taskEXIT_CRITICAL();
        awaiting_enable_feedback = 1U;
        pending_enable = 0U; /* 下周期仍在 IDLE，提交首组使能后的零 MIT。 */
        return;
    }

    if (mode != IDLE && !imu_valid)
    {
        /* 姿态暂时不可用时先零速制动，停稳或制动期限到了再由循环开头失能。 */
        Begin_Fault_Stop(now_ms);
    }
    float dt = elapsed_us * 0.000001f;
    ff_control_interval_s = dt;
    /* 任务被堵住后再醒来时，不把整段空白算进一次加速。25 ms 的正常抖动仍按实测间隔。 */
    if (dt > 0.05f)
        dt = 0.05f;
    float vx = 0.0f, vy = 0.0f, wz = 0.0f;
    if (mode == TRANSLATION)
    {
        ramp_speed = (float)fmin((double)target_speed,
                                (double)ramp_speed + (double)CHASSIS_STRAIGHT_ACCELERATION_MPS2 * dt);
    }
    else if (mode == DISTANCE)
        Service_Distance(dt, now_ms);
    if (mode == TRANSLATION || mode == DISTANCE)
    {
        if (mode == DISTANCE)
        {
            vx = distance_axis_x * ramp_speed - distance_axis_y * distance_cross_speed;
            vy = distance_axis_y * ramp_speed + distance_axis_x * distance_cross_speed;
        }
        else
            Chassis_Translation_Target(translation_direction, ramp_speed, &vx, &vy);
        float heading_delta = Hold_Yaw(&imu, dt) - turn_speed;
        const float yaw_step = (distance_turn_blend ? CHASSIS_TURN_ACCEL_RAD_S2 : CHASSIS_VECTOR_ACCEL_RAD_S2) * dt;
        turn_speed += fmaxf(-yaw_step, fminf(yaw_step, heading_delta));
        if (fabsf(heading_delta) <= yaw_step)
            distance_turn_blend = 0U;
        wz = turn_speed;
    }
    else if (mode == VELOCITY || mode == HOLDING)
    {
        const uint8_t holding = mode == HOLDING;
        const float target_wz = vector_hold_yaw && vector_target[2] == 0.0f ?
            Hold_Yaw(&imu, dt) : vector_target[2];
        const float target[3] = {vector_target[0], vector_target[1], target_wz};
        Velocity_Profile_Step(&velocity_profile, target,
                              holding ? CHASSIS_DISTANCE_ACCELERATION_MPS2 : CHASSIS_STRAIGHT_ACCELERATION_MPS2,
                              holding ? CHASSIS_TURN_ACCEL_RAD_S2 : CHASSIS_VECTOR_ACCEL_RAD_S2,
                              vector_smoothing_s, dt);
        vx = velocity_profile.output[0];
        vy = velocity_profile.output[1];
        wz = velocity_profile.output[2];
        if (holding && vx == 0.0f && vy == 0.0f && wz == 0.0f)
        {
            const uint8_t completing_distance = distance_completion_pending;
            Chassis_Stop();
            distance_completion_pending = completing_distance;
            distance_stop_ms = now_ms;
        }
    }
    else if (mode == TURN)
        wz = Service_Turn(&imu, dt);

    if (vector_world_frame)
    {
        const float cosine = cosf(command_yaw_rad), sine = sinf(command_yaw_rad);
        const float world_vx = vx;
        vx = cosine * world_vx + sine * vy;
        vy = -sine * world_vx + cosine * vy;
    }

    /* 定向平移已经有陀螺阻尼；这里只修正显式旋转、定角转向及其正常收速。 */
    uint8_t use_rate_feedback = mode == TURN || mode == HOLDING ||
        (mode == VELOCITY && (!vector_hold_yaw || vector_target[2] != 0.0f));
#if CHASSIS_TUNING_ENABLED
    use_rate_feedback = use_rate_feedback && !vector_calibration_spin;
#endif
    if (use_rate_feedback && imu_valid && wz != 0.0f)
    {
        const float acceleration = mode == TURN || mode == HOLDING ?
            CHASSIS_TURN_ACCEL_RAD_S2 : CHASSIS_VECTOR_ACCEL_RAD_S2;
        wz = Apply_Yaw_Rate_Feedback(wz, CHASSIS_STRAIGHT_YAW_SIGN * imu.world_z_rad_s, dt, acceleration);
    }
    else
        yaw_rate_filter_valid = 0U;

    if (!Send_Chassis_Targets(vx, vy, wz))
        return;
    if (mode == TURN)
    {
        Confirm_Turn_Stop(&imu);
    }
    else if (mode == IDLE)
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
