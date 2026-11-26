/* 来源：旧车 chassis_straight/chassis_translation/chassis_turn 的动作计算与参数。
 * 设置目标、周期计算后提交 MIT 的组织方式参考 basic_framework；三轮几何沿旧车，
 * 不使用其四轮公式。本文件仅保留动作状态，不迁移输入仲裁、租期或整套故障框架。
 * F407 适配：三个 CAN 发送邮箱，三帧一组；取消异步完成后才提交失能帧。
 * 本次 F407 收发实测适配：成功帧之间让出 1 tick，留出电机反馈时间，已架空验证；
 * 此间隔不是旧车参数，10 ms 控制节拍、PID 和轮序参数保持不变。
 */
#include "chassis.h"
#include "chassis_odometry.h"
#include "chassis_time.h"
#include "chassis_feedback.h"
#include "motor_chassis.h"
#include "velocity_profile.h"
#include "dm_motor.h"
#include "bsp_can.h"
#include "INS_task.h"
#include "pid.h"
#include "FreeRTOS.h"
#include "task.h"
#include <math.h>
#include <stddef.h>

#define DEG_TO_RAD 0.01745329251994329577f
#define CHASSIS_TURN_DEG_TO_RAD 0.01745329251994329577

typedef enum { IDLE, TRANSLATION, DISTANCE, VELOCITY, TURN, HOLDING } Chassis_Mode_t;

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
static uint8_t enabled, pending_enable, disable_requested;
static uint8_t waiting_for_mailboxes;
static uint32_t mailboxes_busy_since_ms;
static uint8_t awaiting_enable_feedback;
static uint32_t enable_feedback_seq[3];
typedef enum { DISABLE_UNSENT, DISABLE_WAITING, DISABLE_CONFIRMED } Disable_Phase_t;
static struct
{
    Disable_Phase_t phase;
    uint32_t feedback_seq, last_attempt_ms;
} disable_wheel[3];
static uint8_t monitoring_stop, stop_groups, disable_after_stop;
static uint32_t stop_seq[3], last_stop_group_ms;
static Chassis_Mode_t mode;
static Chassis_Control_Clock_t control_clock;
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
static float distance_axis_x, distance_axis_y, distance_cross_speed, distance_position_kp;
static uint32_t distance_timeout_ms, distance_progress_ms, distance_stop_ms;
static uint8_t distance_completion_pending;
static float vector_target[3], vector_smoothing_s;
static Velocity_Profile_t velocity_profile;
static Navigation_Speed_State_t navigation_speed_state;
_Static_assert(CHASSIS_NAV_SPEED_CONTROLLER >= NAV_SPEED_BASELINE &&
               CHASSIS_NAV_SPEED_CONTROLLER <= NAV_SPEED_ADRC, "invalid navigation speed controller");
static const Velocity_Navigation_Config_t navigation_config = {
    .start_accel_mps2 = CHASSIS_NAV_START_ACCEL_MPS2,
    .linear_accel_mps2 = CHASSIS_NAV_ACCELERATION_MPS2,
    .linear_decel_mps2 = CHASSIS_NAV_DECELERATION_MPS2,
    .reversal_decel_mps2 = CHASSIS_NAV_REVERSAL_DECELERATION_MPS2,
    .full_accel_speed_mps = CHASSIS_NAV_FULL_ACCEL_SPEED_MPS,
    .angular_accel_rad_s2 = CHASSIS_VECTOR_ACCEL_RAD_S2,
    .speed_mode = CHASSIS_NAV_SPEED_CONTROLLER,
};
static uint8_t vector_hold_yaw, vector_world_frame, vector_navigation;
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
           (uint32_t)(Chassis_Time_Now_Ms() - imu->sample_time_ms) <= CHASSIS_ODOMETRY_MAX_IMU_AGE_MS;
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
    uint32_t now_ms = Chassis_Time_Now_Ms();
    for (unsigned i = 0U; i < 3U; ++i)
    {
        seq[i] = dm_motor[i].feedback_seq;
        if (seq[i] == stop_seq[i])
            fresh_group = 0U;
        if (dm_motor[i].state != 1U || !(fabsf(dm_motor[i].velocity) <= CHASSIS_STOP_SPEED_RAD_S) ||
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
        if (stop_groups < CHASSIS_STOP_GROUPS)
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
    return stop_groups == CHASSIS_STOP_GROUPS;
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

typedef struct {
    uint8_t tracking;
    uint32_t started_ms;
    float origin[3], minimum[3], maximum[3];
} Encoder_Stop_Window_t;

/* SWD：从本段首组零 MIT 起累计各项拒绝次数，accepted 1/2/3 为短窗、长窗、卸载后恢复。 */
typedef struct {
    uint32_t started_ms;
    uint16_t invalid_state, invalid_imu, gyro, invalid_wheel, speed, frame_gap;
    uint16_t normal_speed, normal_span, bias_gyro, bias_span;
    uint8_t accepted;
} Encoder_Stop_Debug_t;
static volatile Encoder_Stop_Debug_t encoder_stop_debug;

typedef enum { NEUTRAL_ARM, NEUTRAL_REQUESTED, NEUTRAL_SUBMITTED, NEUTRAL_OBSERVING,
               NEUTRAL_RESTORE, NEUTRAL_RESTORED, NEUTRAL_FAILED, NEUTRAL_FAILED_RESTORE } Neutral_Phase_t;
static Neutral_Phase_t neutral_phase;
static uint32_t neutral_sent_seq[3];
static struct {
    uint8_t tracking, overspeed;
    uint32_t last_ms[3], sequence[3], start_ms[3], last_quiet_ms[3];
    float previous[3], origin[3], minimum[3], maximum[3], speed_area[3], excess_area[3];
} neutral_window;
/* SWD 在故障和阶段交接后保留本次的接管时间、三轮实测包络和首个拒绝原因。 */
typedef struct {
    uint32_t requested_ms, submitted_ms, restored_ms;
    float span[3], travel[3], mean_abs_speed[3];
    uint8_t rejected, wheel;
} Neutral_Stop_Debug_t;
static volatile Neutral_Stop_Debug_t neutral_stop_debug;

/* 每轮全窗保留极值，以免往返或轮间抵消伪装成静止。时间由各轮最后一帧独立确认。 */
static uint8_t Encoder_Window_Quiet(Encoder_Stop_Window_t *window, const Chassis_Feedback_Snapshot_t *snapshot,
                                    uint8_t restart, uint32_t duration_ms, float span_rad, volatile uint16_t *span_rejections)
{
    if (restart || !window->tracking)
    {
        window->started_ms = snapshot->now_ms;
        window->tracking = 1U;
        for (unsigned i = 0U; i < 3U; ++i)
        {
            window->origin[i] = snapshot->wheel[i].position;
            window->minimum[i] = window->maximum[i] = 0.0f;
        }
        return 0U;
    }
    uint8_t confirmed = (uint32_t)(snapshot->now_ms - window->started_ms) >= duration_ms;
    for (unsigned i = 0U; i < 3U; ++i)
    {
        const Chassis_Wheel_Feedback_t *wheel = &snapshot->wheel[i];
        const float delta = remainderf(wheel->position - window->origin[i], CHASSIS_ODOMETRY_ENCODER_PERIOD_RAD);
        window->minimum[i] = fminf(window->minimum[i], delta);
        window->maximum[i] = fmaxf(window->maximum[i], delta);
        if (window->maximum[i] - window->minimum[i] > span_rad)
        {
            ++*span_rejections;
            window->tracking = 0U;
            return 0U;
        }
        if ((uint32_t)(wheel->feedback_time_ms - window->started_ms) < duration_ms)
            confirmed = 0U;
    }
    return confirmed;
}

uint8_t Chassis_Encoder_Stop_Confirmed(uint8_t begin)
{
    static Encoder_Stop_Window_t normal, biased;
    static uint8_t have_feedback;
    static uint32_t last_poll_ms, last_ms[3], sequence[3];
    if (begin)
        encoder_stop_debug = (Encoder_Stop_Debug_t){ .started_ms = Chassis_Time_Now_Ms() };
    INS_control_data_t imu;
    if (!enabled || disable_after_stop || mode != IDLE || !monitoring_stop || distance_completion_pending)
    {
        ++encoder_stop_debug.invalid_state;
        have_feedback = 0U;
        return 0U;
    }
    if (!Read_IMU(&imu) || imu.yaw_predicted)
    {
        ++encoder_stop_debug.invalid_imu;
        have_feedback = 0U;
        return 0U;
    }
    if (fabsf(imu.world_z_rad_s) > CHASSIS_ENCODER_STOP_GYRO_RAD_S)
    {
        ++encoder_stop_debug.gyro;
        have_feedback = 0U;
        return 0U;
    }
    Chassis_Feedback_Snapshot_t snapshot;
    Chassis_Feedback_Read(&snapshot);
    const uint8_t poll_gap = !begin && have_feedback &&
        (uint32_t)(snapshot.now_ms - last_poll_ms) > CHASSIS_MOTOR_FEEDBACK_TIMEOUT_MS;
    if (poll_gap)
        ++encoder_stop_debug.frame_gap;
    const uint8_t baseline = begin || !have_feedback || poll_gap;
    uint8_t normal_speed = 1U;
    for (unsigned i = 0U; i < 3U; ++i)
    {
        const Chassis_Wheel_Feedback_t *wheel = &snapshot.wheel[i];
        if (wheel->id != chassis_wheel_config[i].id || wheel->state != 1U ||
            !(fabsf(wheel->position) <= CHASSIS_ODOMETRY_ENCODER_PERIOD_RAD * 0.5f) ||
            (uint32_t)(snapshot.now_ms - wheel->feedback_time_ms) > CHASSIS_MOTOR_FEEDBACK_TIMEOUT_MS)
        {
            ++encoder_stop_debug.invalid_wheel;
            have_feedback = 0U;
            return 0U;
        }
        if (!(fabsf(wheel->velocity) <= CHASSIS_ENCODER_BIAS_SPEED_RAD_S))
        {
            ++encoder_stop_debug.speed;
            have_feedback = 0U;
            return 0U;
        }
        if (!baseline)
        {
            const uint32_t frames = wheel->feedback_seq - sequence[i];
            const uint32_t dt = wheel->feedback_time_ms - last_ms[i];
            /* 同一帧可反复读取，但不推进该轮时间。跳号或间断的中间位移不可验证。 */
            if (frames > 1U || (frames == 0U) != (dt == 0U) || dt > CHASSIS_MOTOR_FEEDBACK_TIMEOUT_MS)
            {
                ++encoder_stop_debug.frame_gap;
                have_feedback = 0U;
                return 0U;
            }
        }
        if (fabsf(wheel->velocity) > CHASSIS_ENCODER_STOP_SPEED_RAD_S)
            normal_speed = 0U;
        last_ms[i] = wheel->feedback_time_ms;
        sequence[i] = wheel->feedback_seq;
    }
    have_feedback = 1U;
    last_poll_ms = snapshot.now_ms;
    if (!normal_speed)
    {
        ++encoder_stop_debug.normal_speed;
        normal.tracking = 0U;
    }
    const uint8_t bias_gyro_quiet = fabsf(imu.world_z_rad_s) <= CHASSIS_ENCODER_BIAS_GYRO_RAD_S;
    if (!bias_gyro_quiet)
    {
        ++encoder_stop_debug.bias_gyro;
        biased.tracking = 0U;
    }
    const uint8_t normal_quiet = normal_speed &&
        Encoder_Window_Quiet(&normal, &snapshot, baseline, CHASSIS_ENCODER_STOP_WINDOW_MS,
                             CHASSIS_ENCODER_STOP_SPAN_RAD, &encoder_stop_debug.normal_span);
    const uint8_t bias_quiet = bias_gyro_quiet &&
        Encoder_Window_Quiet(&biased, &snapshot, baseline, CHASSIS_ENCODER_BIAS_WINDOW_MS,
                             CHASSIS_ENCODER_BIAS_SPAN_RAD, &encoder_stop_debug.bias_span);
    if (normal_quiet || bias_quiet)
        encoder_stop_debug.accepted = normal_quiet ? 1U : 2U;
    return normal_quiet || bias_quiet;
}

/* 1 状态或姿态，2 陀螺，3 电机或陈旧帧，4 序号/时间间断，5 角跨度，
 * 6 累计往返行程，7 速度读数持续或积分异常。卸载期间任何拒绝都立即恢复常规阻尼。 */
static uint8_t Neutral_Reject(uint8_t reason, unsigned wheel)
{
    neutral_stop_debug.rejected = reason;
    neutral_stop_debug.wheel = (uint8_t)wheel;
    neutral_window.tracking = neutral_window.overspeed = 0U;
    if (neutral_phase == NEUTRAL_REQUESTED)
        neutral_phase = NEUTRAL_ARM; /* 全组尚未发出，回到正常阻尼下重新验满预备窗。 */
    else if (neutral_phase == NEUTRAL_RESTORED)
        neutral_phase = NEUTRAL_FAILED;
    else if (neutral_phase != NEUTRAL_ARM)
        neutral_phase = NEUTRAL_FAILED_RESTORE;
    return 0U;
}

uint8_t Chassis_Longitudinal_Stop_Confirmed(uint8_t begin)
{
    if (begin)
    {
        neutral_phase = NEUTRAL_ARM;
        neutral_window.tracking = neutral_window.overspeed = 0U;
        neutral_stop_debug = (Neutral_Stop_Debug_t){0};
    }
    if (neutral_phase == NEUTRAL_ARM)
    {
        if (Chassis_Is_Stopped())
        {
            if (begin)
                encoder_stop_debug = (Encoder_Stop_Debug_t){ .started_ms = Chassis_Time_Now_Ms() };
            return 1U;
        }
        if (Chassis_Encoder_Stop_Confirmed(begin))
            return 1U;
    }
    if (neutral_phase == NEUTRAL_FAILED || neutral_phase == NEUTRAL_FAILED_RESTORE)
        return 0U;
    INS_control_data_t imu;
    if (!enabled || disable_after_stop || mode != IDLE || !monitoring_stop || distance_completion_pending ||
        !Read_IMU(&imu) || imu.yaw_predicted)
        return Neutral_Reject(1U, 0U);
    if (fabsf(imu.world_z_rad_s) > CHASSIS_ENCODER_BIAS_GYRO_RAD_S)
        return Neutral_Reject(2U, 0U);

    Chassis_Feedback_Snapshot_t snapshot;
    Chassis_Feedback_Read(&snapshot);
    uint8_t all_after_command = 1U;
    for (unsigned i = 0U; i < 3U; ++i)
    {
        const Chassis_Wheel_Feedback_t *w = &snapshot.wheel[i];
        if (w->id != chassis_wheel_config[i].id || w->state != 1U ||
            !(fabsf(w->position) <= CHASSIS_ODOMETRY_ENCODER_PERIOD_RAD * 0.5f) ||
            !(fabsf(w->velocity) <= chassis_wheel_config[i].max_motor_rad_s) ||
            (uint32_t)(snapshot.now_ms - w->feedback_time_ms) > CHASSIS_MOTOR_FEEDBACK_TIMEOUT_MS)
            return Neutral_Reject(3U, i + 1U);
        if (w->feedback_seq == neutral_sent_seq[i])
            all_after_command = 0U;
    }
    /* 三轮回复可能错开。逐拍检查先到轮的位置、序号和速度，最后一轮回复才开始计满观察期。 */
    const uint8_t waiting_for_group = neutral_phase == NEUTRAL_SUBMITTED;
    const uint8_t arming = neutral_phase == NEUTRAL_ARM || neutral_phase == NEUTRAL_REQUESTED;
    const uint32_t duration = arming ? CHASSIS_NEUTRAL_ARM_MS : CHASSIS_ENCODER_BIAS_WINDOW_MS;
    uint8_t position_quiet = 1U, mean_quiet = 1U;
    for (unsigned i = 0U; i < 3U; ++i)
    {
        const Chassis_Wheel_Feedback_t *w = &snapshot.wheel[i];
        const float speed = fabsf(w->velocity);
        if (!arming && (!waiting_for_group || w->feedback_seq != neutral_sent_seq[i]) &&
            speed > CHASSIS_NEUTRAL_PEAK_SPEED_RAD_S)
            return Neutral_Reject(7U, i + 1U);
        if (speed > CHASSIS_ENCODER_BIAS_SPEED_RAD_S)
            neutral_window.overspeed = 1U;
        if (!neutral_window.tracking)
        {
            neutral_window.start_ms[i] = neutral_window.last_quiet_ms[i] = w->feedback_time_ms;
            neutral_window.origin[i] = w->position;
            neutral_window.minimum[i] = neutral_window.maximum[i] = 0.0f;
            neutral_window.speed_area[i] = neutral_window.excess_area[i] = 0.0f;
            neutral_stop_debug.span[i] = neutral_stop_debug.travel[i] = neutral_stop_debug.mean_abs_speed[i] = 0.0f;
        }
        else
        {
            const uint32_t frames = w->feedback_seq - neutral_window.sequence[i];
            const uint32_t dt = w->feedback_time_ms - neutral_window.last_ms[i];
            if (frames > 1U || (frames == 0U) != (dt == 0U) || dt > CHASSIS_MOTOR_FEEDBACK_TIMEOUT_MS)
                return Neutral_Reject(4U, i + 1U);
            const float delta = remainderf(w->position - neutral_window.origin[i], CHASSIS_ODOMETRY_ENCODER_PERIOD_RAD);
            neutral_window.minimum[i] = fminf(neutral_window.minimum[i], delta);
            neutral_window.maximum[i] = fmaxf(neutral_window.maximum[i], delta);
            neutral_stop_debug.span[i] = neutral_window.maximum[i] - neutral_window.minimum[i];
            if (neutral_stop_debug.span[i] > CHASSIS_ENCODER_BIAS_SPAN_RAD)
                return Neutral_Reject(5U, i + 1U);
            const float step = remainderf(w->position - neutral_window.previous[i], CHASSIS_ODOMETRY_ENCODER_PERIOD_RAD);
            neutral_stop_debug.travel[i] += fmaxf(0.0f, fabsf(step) - CHASSIS_NEUTRAL_POSITION_NOISE_RAD);
            if (neutral_stop_debug.travel[i] > CHASSIS_NEUTRAL_TRAVEL_RAD)
                return Neutral_Reject(6U, i + 1U);
            if (!arming && !waiting_for_group)
            {
                if (speed <= CHASSIS_ENCODER_BIAS_SPEED_RAD_S)
                    neutral_window.last_quiet_ms[i] = w->feedback_time_ms;
                neutral_window.speed_area[i] += speed * (float)dt;
                neutral_window.excess_area[i] = fmaxf(0.0f, neutral_window.excess_area[i] +
                    (speed - CHASSIS_ENCODER_BIAS_SPEED_RAD_S) * (float)dt);
                if ((uint32_t)(w->feedback_time_ms - neutral_window.last_quiet_ms[i]) > CHASSIS_NEUTRAL_SPEED_BURST_MS ||
                    neutral_window.excess_area[i] > CHASSIS_ENCODER_BIAS_SPEED_RAD_S * CHASSIS_NEUTRAL_SPEED_BURST_MS)
                    return Neutral_Reject(7U, i + 1U);
            }
        }
        const uint32_t elapsed = w->feedback_time_ms - neutral_window.start_ms[i];
        if (elapsed < duration)
            position_quiet = 0U;
        if (!arming && elapsed != 0U)
        {
            neutral_stop_debug.mean_abs_speed[i] = neutral_window.speed_area[i] / (float)elapsed;
            if (neutral_stop_debug.mean_abs_speed[i] > CHASSIS_ENCODER_BIAS_SPEED_RAD_S)
                mean_quiet = 0U;
        }
        neutral_window.last_ms[i] = w->feedback_time_ms;
        neutral_window.sequence[i] = w->feedback_seq;
        neutral_window.previous[i] = w->position;
        if (waiting_for_group && all_after_command)
            neutral_window.start_ms[i] = neutral_window.last_quiet_ms[i] = w->feedback_time_ms;
    }
    neutral_window.tracking = 1U;
    if (waiting_for_group)
    {
        if (all_after_command)
            neutral_phase = NEUTRAL_OBSERVING;
        return 0U;
    }
    if (!position_quiet)
        return 0U;
    if (neutral_phase == NEUTRAL_ARM && neutral_window.overspeed)
    {
        neutral_phase = NEUTRAL_REQUESTED;
        neutral_stop_debug.rejected = neutral_stop_debug.wheel = 0U;
        neutral_stop_debug.requested_ms = snapshot.now_ms;
    }
    else if (!arming && !mean_quiet)
        return Neutral_Reject(7U, 0U);
    else if (neutral_phase == NEUTRAL_OBSERVING)
        neutral_phase = NEUTRAL_RESTORE;
    else if (neutral_phase == NEUTRAL_RESTORED && all_after_command)
    {
        neutral_phase = NEUTRAL_ARM;
        encoder_stop_debug.accepted = 3U;
        return 1U;
    }
    return 0U;
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
    uint32_t now_ms = Chassis_Time_Now_Ms();
    for (unsigned i = 0U; i < 3U; ++i)
        if (dm_motor[i].id != chassis_wheel_config[i].id || dm_motor[i].state != 1U ||
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
        (uint32_t)(Chassis_Time_Now_Us() - data->sample_time_us) <=
            CHASSIS_MOTOR_FEEDBACK_TIMEOUT_US &&
        Chassis_Motors_Ready() && Read_IMU(&imu);
    data->yaw_rate_rad_s = data->valid ? CHASSIS_STRAIGHT_YAW_SIGN * imu.world_z_rad_s : 0.0f;
    return data->valid;
}

/* 只补偿控制向量的旋转时延。里程计位置和到位判定仍由实测反馈推进。 */
static float Predict_Command_Yaw(const Chassis_Odometry_t *pose, uint32_t now_us, float yaw_rate)
{
    const uint32_t age_us = now_us - pose->sample_time_us;
    if (age_us > CHASSIS_MOTOR_FEEDBACK_TIMEOUT_US)
        return pose->yaw_rad;
    return pose->yaw_rad + (yaw_rate - odometry.heading.bias_rad_s) * Chassis_Time_Us_To_S(age_us);
}

/** @brief 只在没有执行动作时重置平移原点，保持现有 INS 的航向参考。 */
uint8_t Chassis_Reset_Odometry(void)
{
    if (enabled && !Chassis_Is_Stopped())
        return 0U;
    if (disable_requested)
        for (unsigned i = 0U; i < 3U; ++i)
            if (disable_wheel[i].phase == DISABLE_UNSENT)
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
    Chassis_Feedback_Snapshot_t snapshot;
    Chassis_Feedback_Read(&snapshot);
    const Chassis_Wheel_Feedback_t *feedback = snapshot.wheel;
    const uint32_t now_ms = snapshot.now_ms;
    uint8_t ready = imu_valid, fresh_group = 1U;
    float wheel[3], position[3];
    Odometry_Feedback_Timing_t timing = {.control_time_ms = now_ms};
    for (unsigned i = 0U; i < 3U; ++i)
    {
        if (feedback[i].id != chassis_wheel_config[i].id || feedback[i].state != 1U ||
            !isfinite(feedback[i].position) || !isfinite(feedback[i].velocity) ||
            fabsf(feedback[i].velocity) > chassis_wheel_config[i].max_motor_rad_s ||
            (uint32_t)(now_ms - feedback[i].feedback_time_ms) > CHASSIS_MOTOR_FEEDBACK_TIMEOUT_MS)
            ready = 0U;
        if (feedback[i].feedback_seq == odometry_seq[i])
            fresh_group = 0U;
        wheel[i] = Chassis_Feedback_To_Wheel_Axis(feedback[i].velocity, &chassis_wheel_config[i]);
        position[i] = Chassis_Feedback_To_Wheel_Axis(feedback[i].position, &chassis_wheel_config[i]);
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
        const uint32_t algorithm_start_us = Chassis_Time_Now_Us();
#endif
        const uint8_t observe_yaw = !imu->yaw_predicted &&
            (!odometry.initialized || imu->sample_time_ms != odometry_imu_sample_ms);
        timing.imu_time_ms = imu->sample_time_ms;
        if (Odometry_Update_Feedback(&odometry, position, wheel, &timing,
                CHASSIS_STRAIGHT_YAW_SIGN * imu->yaw_deg * DEG_TO_RAD,
                CHASSIS_STRAIGHT_YAW_SIGN * imu->world_z_rad_s, now_us, observe_yaw) && observe_yaw)
            odometry_imu_sample_ms = imu->sample_time_ms;
#if CHASSIS_TUNING_ENABLED
        chassis_odometry_last_us = Chassis_Time_Now_Us() - algorithm_start_us;
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
             CHASSIS_MOTOR_FEEDBACK_TIMEOUT_US)
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
    neutral_phase = NEUTRAL_ARM;
    if (mode != IDLE)
        monitoring_stop = 0U;
    mode = IDLE;
    target_speed = ramp_speed = distance_cross_speed = 0.0f;
    turn_speed = 0.0f;
    turn_in_window = turn_zero_submitted = turn_imu_quiet = 0U;
    distance_turn_blend = 0U;
    distance_completion_pending = 0U;
    vector_hold_yaw = vector_world_frame = vector_navigation = 0U;
    navigation_speed_state = (Navigation_Speed_State_t){0};
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
    navigation_speed_state = (Navigation_Speed_State_t){0};
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
    enabled = pending_enable = disable_requested = awaiting_enable_feedback = waiting_for_mailboxes = 0U;
    monitoring_stop = stop_groups = disable_after_stop = 0U;
    Chassis_Stop();
    Odometry_Init(&odometry);
    for (unsigned i = 0U; i < 3U; ++i)
    {
        odometry_seq[i] = 0U;
        chassis_speed[i] = 0.0f;
        last_body_target[i] = 0.0f;
        chassis_tuning_kd[i] = chassis_wheel_config[i].default_kd;
        chassis_tuning_friction[i] = chassis_wheel_config[i].default_friction_nm;
        chassis_motor_target[i] = chassis_motor_ff[i] = 0.0f;
    }
    chassis_tuning_ff_gain = 1.0f;
    Chassis_Time_Reset(&control_clock);
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
    disable_requested = 1U;
    for (unsigned i = 0U; i < 3U; ++i)
        disable_wheel[i].phase = DISABLE_UNSENT;
    HAL_CAN_AbortTxRequest(&MOTOR_CAN, CAN_TX_MAILBOX0 | CAN_TX_MAILBOX1 | CAN_TX_MAILBOX2);
}

uint8_t Chassis_Disable_Confirmed(void)
{
    if (!disable_requested)
        return 0U;

    uint8_t confirmed = 1U;
    taskENTER_CRITICAL();
    const uint32_t now_ms = Chassis_Time_Now_Ms();
    for (unsigned i = 0U; i < 3U; ++i)
    {
        const volatile dm_motor_t *motor = &dm_motor[i];
        if (motor->id == chassis_wheel_config[i].id)
        {
            if (disable_wheel[i].phase == DISABLE_WAITING && motor->state == 0U &&
                motor->feedback_seq != disable_wheel[i].feedback_seq &&
                (uint32_t)(now_ms - motor->feedback_time_ms) <= CHASSIS_MOTOR_FEEDBACK_TIMEOUT_MS)
                disable_wheel[i].phase = DISABLE_CONFIRMED;
            else if (disable_wheel[i].phase == DISABLE_CONFIRMED && motor->state != 0U)
                disable_wheel[i].phase = DISABLE_UNSENT;
        }
        confirmed &= disable_wheel[i].phase == DISABLE_CONFIRMED;
    }
    taskEXIT_CRITICAL();
    return confirmed;
}

/* FD 已进邮箱也须等回执。没有回执时按轮限频；HAL 提交未成功的轮下一拍继续尝试。 */
static void Service_Disable(void)
{
    if (!disable_requested || Chassis_Disable_Confirmed() ||
        HAL_CAN_GetTxMailboxesFreeLevel(&MOTOR_CAN) != 3U)
        return;

    uint8_t previous_sent = 0U;
    for (unsigned i = 0U; i < 3U; ++i)
    {
        const Disable_Phase_t phase = disable_wheel[i].phase;
        if (phase == DISABLE_CONFIRMED ||
            (phase == DISABLE_WAITING &&
             (uint32_t)(Chassis_Time_Now_Ms() - disable_wheel[i].last_attempt_ms) < CHASSIS_CAN_DISABLE_RETRY_MS))
            continue;
        if (previous_sent)
            vTaskDelay(pdMS_TO_TICKS(CHASSIS_CAN_INTERFRAME_DELAY_MS));

        taskENTER_CRITICAL();
        const uint32_t feedback_seq = dm_motor[i].feedback_seq;
        const HAL_StatusTypeDef result = disable_motor(&MOTOR_CAN, chassis_wheel_config[i].id);
        if (phase == DISABLE_UNSENT && result == HAL_OK)
        {
            disable_wheel[i].feedback_seq = feedback_seq;
            disable_wheel[i].phase = DISABLE_WAITING;
        }
        disable_wheel[i].last_attempt_ms = Chassis_Time_Now_Ms();
        taskEXIT_CRITICAL();
        previous_sent = result == HAL_OK;
    }
}

uint8_t Chassis_Can_Enable(void)
{
    return !enabled && (!disable_requested || Chassis_Disable_Confirmed()) &&
        HAL_CAN_GetTxMailboxesFreeLevel(&MOTOR_CAN) == 3U;
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
    disable_requested = 0U;
    pending_enable = 1U;
    Chassis_Time_Reset(&control_clock);
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
    control_clock.step_start_us = Chassis_Time_Now_Us();
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
static uint8_t Start_Distance(Chassis_Translation_Direction_t direction,
                              float distance_m, float speed_limit_mps, float position_kp)
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
    const uint32_t now_us = Chassis_Time_Now_Us();
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
        const uint32_t imu_age_ms = Chassis_Time_Now_Ms() - imu.sample_time_ms;
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
    distance_position_kp = position_kp;
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

uint8_t Chassis_Distance_Start(Chassis_Translation_Direction_t direction,
                               float distance_m, float speed_limit_mps)
{
    return Start_Distance(direction, distance_m, speed_limit_mps, CHASSIS_DISTANCE_POSITION_KP);
}

uint8_t Chassis_Distance_Start_Fast(Chassis_Translation_Direction_t direction,
                                    float distance_m, float speed_limit_mps)
{
    return Start_Distance(direction, distance_m, speed_limit_mps, CHASSIS_DISTANCE_FAST_POSITION_KP);
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
    const float approach_speed = fmaxf(distance_position_kp * (float)remaining,
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
    turn_started_ms = Chassis_Time_Now_Ms();
    mode = TURN;
    control_clock.step_start_us = Chassis_Time_Now_Us();
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
                            float smoothing_s, uint8_t navigation)
{
    INS_control_data_t imu;
    if (!enabled || !isfinite(vx) || !isfinite(vy) || !isfinite(wz) ||
        hypotf(vx, vy) > CHASSIS_STRAIGHT_MAX_SPEED_MPS ||
        fabsf(wz) > CHASSIS_VECTOR_MAX_WZ_RAD_S)
        return 0U;
    if (vx == 0.0f && vy == 0.0f && wz == 0.0f)
    {
        if (navigation)
            Chassis_Hold();
        else
            Chassis_Stop();
        return 1U;
    }
    if ((mode == HOLDING && (distance_completion_pending || !Chassis_Motors_Ready())) ||
        ((mode == VELOCITY || mode == HOLDING) ? !Read_IMU(&imu) : !Motion_Ready(&imu)))
        return 0U;
    Chassis_Odometry_t pose;
    if (world_frame && !Chassis_Get_Odometry(&pose))
        return 0U;
    if (world_frame != vector_world_frame || navigation != vector_navigation)
        navigation_speed_state = (Navigation_Speed_State_t){0};
    if (world_frame != vector_world_frame || (navigation && (mode == HOLDING || !vector_navigation)))
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
        control_clock.step_start_us = Chassis_Time_Now_Us();
    neutral_phase = NEUTRAL_ARM;
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
    vector_navigation = navigation;
    vector_smoothing_s = smoothing_s;
#if CHASSIS_TUNING_ENABLED
    vector_calibration_spin = 0U;
#endif
    mode = VELOCITY;
    return 1U;
}

uint8_t Chassis_Set_Velocity(float vx, float vy, float wz)
{
    return Set_Velocity(vx, vy, wz, 1U, 0U, CHASSIS_VECTOR_SMOOTHING_S, 0U);
}

uint8_t Chassis_Set_Velocity_Explicit_Yaw(float vx, float vy, float wz)
{
    return Set_Velocity(vx, vy, wz, 0U, 0U, CHASSIS_VECTOR_SMOOTHING_S, 0U);
}

uint8_t Chassis_Set_World_Velocity(float vx, float vy, float wz)
{
    return Set_Velocity(vx, vy, wz, 0U, 1U, CHASSIS_VECTOR_SMOOTHING_S, 0U);
}

uint8_t Chassis_Set_Navigation_Velocity(float vx, float vy, float wz, uint8_t world_frame)
{
    return Set_Velocity(vx, vy, wz, !world_frame, world_frame, CHASSIS_VECTOR_SMOOTHING_S, 1U);
}

#if CHASSIS_TUNING_ENABLED
uint8_t Chassis_Set_Test_Rotation(float wz)
{
    /* 专项标定需要直接观察速度指令与实际响应。 */
    if (!Set_Velocity(0.0f, 0.0f, wz, 0U, 0U, 0.0f, 0U))
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
    const uint8_t neutral = mode == IDLE && !disable_after_stop &&
        (neutral_phase == NEUTRAL_REQUESTED || neutral_phase == NEUTRAL_SUBMITTED || neutral_phase == NEUTRAL_OBSERVING);
    float wheel[3];
    Motor_Chassis_Inverse(vx, vy, wz, wheel);
    float scale = 1.0f;
    for (unsigned i = 0U; i < 3U; ++i)
    {
        const Chassis_Wheel_Config_t *config = &chassis_wheel_config[i];
        const float speed = fabsf(wheel[i] * config->gear_ratio);
        if (speed > config->max_motor_rad_s)
            scale = fminf(scale, config->max_motor_rad_s / speed);
    }
    /* 只对已限幅、实际提交的车体平移目标求导。长间隔不补偿累计误差，
     * Stop/故障/使能包序始终零前馈；未标定的偏航惯性项保持零。 */
    const uint32_t ff_now_us = Chassis_Time_Now_Us();
    const float ff_dt = ff_history_valid ?
        Chassis_Time_Us_To_S(ff_now_us - ff_previous_us) : ff_control_interval_s;
    float fx = 0.0f, fy = 0.0f;
    if (mode != IDLE && ff_dt > 0.0f && ff_dt <= CHASSIS_FEEDFORWARD_MAX_INTERVAL_S)
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
        const Chassis_Wheel_Config_t *config = &chassis_wheel_config[i];
        const float target = wheel[i] * scale * config->gear_ratio * config->direction;
        float ff = 0.0f;
        if (mode != IDLE)
        {
            ff = wheel_torque[i] * config->direction / config->gear_ratio +
                 ((vx != 0.0f || vy != 0.0f) ?
                  chassis_tuning_friction[i] * target / (fabsf(target) + CHASSIS_FRICTION_SPEED_RAD_S) : 0.0f);
            ff = fmaxf(-CHASSIS_FEEDFORWARD_LIMIT_NM, fminf(CHASSIS_FEEDFORWARD_LIMIT_NM, ff));
        }
        const float kd = neutral ? 0.0f : chassis_tuning_kd[i];
        if (ctrl_motor(&MOTOR_CAN, config->id, 0.0f, target, 0.0f, kd, ff) != HAL_OK)
        {
            Chassis_Disable();
            return 0U;
        }
        chassis_motor_target[i] = target;
        chassis_motor_ff[i] = ff;
        if (i < 2U)
            vTaskDelay(pdMS_TO_TICKS(CHASSIS_CAN_INTERFRAME_DELAY_MS));
    }
    if (neutral_phase == NEUTRAL_REQUESTED || neutral_phase == NEUTRAL_RESTORE ||
        neutral_phase == NEUTRAL_FAILED_RESTORE)
    {
        /* 记录三次 HAL 发送均成功之后的序号，每只电机都必须再返回一帧。 */
        Chassis_Feedback_Snapshot_t submitted;
        Chassis_Feedback_Read(&submitted);
        for (unsigned i = 0U; i < 3U; ++i)
            neutral_sent_seq[i] = submitted.wheel[i].feedback_seq;
        if (neutral)
        {
            neutral_phase = NEUTRAL_SUBMITTED;
            neutral_stop_debug.submitted_ms = submitted.now_ms;
            /* 保留末个带阻尼样本作基准，连同首个卸载回复一起检查。 */
            for (unsigned i = 0U; i < 3U; ++i)
            {
                neutral_window.origin[i] = neutral_window.previous[i];
                neutral_window.minimum[i] = neutral_window.maximum[i] = 0.0f;
                neutral_stop_debug.span[i] = neutral_stop_debug.travel[i] = 0.0f;
            }
        }
        else
        {
            neutral_phase = neutral_phase == NEUTRAL_FAILED_RESTORE ? NEUTRAL_FAILED : NEUTRAL_RESTORED;
            neutral_stop_debug.restored_ms = submitted.now_ms;
        }
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
    if (stop_groups == CHASSIS_STOP_GROUPS && quiet_ms >= CHASSIS_TURN_STOP_CONFIRM_MS)
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
        (uint32_t)(now_ms - distance_stop_ms) >= CHASSIS_TIMED_BRAKE_LIMIT_MS && !Chassis_Is_Stopped())
        Chassis_Disable();
    if ((mode == DISTANCE || (mode == HOLDING && distance_completion_pending)) && run_started &&
        (uint32_t)(now_ms - run_started_ms) >= distance_timeout_ms)
        Begin_Fault_Stop(now_ms);
    if (mode == TRANSLATION && run_started && run_duration_ms != 0U &&
        (uint32_t)(now_ms - run_started_ms) >= run_duration_ms)
        Chassis_Hold();
    /* 故障制动须撤销使能；正常到时减速完成后持续发送零 MIT。 */
    if ((disable_after_stop && ((uint32_t)(now_ms - run_started_ms) >= CHASSIS_TIMED_BRAKE_LIMIT_MS ||
                                Stopped_Feedback())) ||
        (mode == TURN && (uint32_t)(now_ms - turn_started_ms) >= CHASSIS_TURN_TIMEOUT_MS))
        Chassis_Disable();
    uint32_t now_us, elapsed_us;
    if (!Chassis_Time_Take_Control_Step(&control_clock, now_tick, &now_us, &elapsed_us))
        return;

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

    if (!enabled)
    {
        Service_Disable();
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
            if (enable_motor(&MOTOR_CAN, chassis_wheel_config[i].id) != HAL_OK)
            {
                Chassis_Disable();
                return;
            }
            if (i < 2U)
                vTaskDelay(pdMS_TO_TICKS(CHASSIS_CAN_INTERFRAME_DELAY_MS));
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
    float dt = Chassis_Time_Us_To_S(elapsed_us);
    ff_control_interval_s = dt;
    /* 任务被堵住后再醒来时，不把整段空白算进一次加速。25 ms 的正常抖动仍按实测间隔。 */
    if (dt > CHASSIS_CONTROL_MAX_STEP_S)
        dt = CHASSIS_CONTROL_MAX_STEP_S;
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
        if (!holding && vector_navigation)
        {
            const float *measured = NULL;
#if CHASSIS_NAV_SPEED_CONTROLLER != 0
            float feedback[2];
            /* 只使用本控制拍刚接受的三轮反馈组；可疑轮角时取消实验补偿。 */
            if (odometry.data.valid && odometry.data.sample_time_us == now_us && !odometry.wheel_yaw_rejected)
            {
                feedback[0] = odometry.data.vx_mps;
                feedback[1] = odometry.data.vy_mps;
                if (vector_world_frame)
                {
                    const float cosine = cosf(command_yaw_rad), sine = sinf(command_yaw_rad);
                    feedback[0] = cosine * odometry.data.vx_mps - sine * odometry.data.vy_mps;
                    feedback[1] = sine * odometry.data.vx_mps + cosine * odometry.data.vy_mps;
                }
                measured = feedback;
            }
#endif
            Velocity_Profile_Step_Navigation(&velocity_profile, target, &navigation_config,
                                             &navigation_speed_state, measured, dt);
        }
        else
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
        run_started_ms = Chassis_Time_Now_Ms();
    }
}
