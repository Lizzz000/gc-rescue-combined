/* 现场 SWD 调参入口：有限时长运动、主机断连制动、显式使能与失能回执。
 * 不写电机内部参数，不保存 Flash，不直接改底盘状态；正常构建不启用本模块。
 */
#include "live_tuning.h"
#include "chassis.h"
#include "dm_motor.h"
#include "INS_task.h"
#include "FreeRTOS.h"
#include "task.h"
#include "tim.h"
#include "vofa.h"
#include "tracked_visual_servo.h"
#include "point_plan.h"
#include "spin_observer.h"
#include "visual_tracking_config.h"
#include <math.h>

volatile uint32_t tuning_request_seq, tuning_command, tuning_heartbeat;
volatile uint32_t tuning_ack_seq, tuning_phase, tuning_error, tuning_sample_seq;
volatile float tuning_args[8], tuning_sample[TUNING_SAMPLE_COUNT];
extern volatile float battery_voltage;
volatile float tuning_visual_center[2], tuning_visual_sample[10];
volatile uint32_t tuning_visual_seq, tuning_visual_time_ms, tuning_visual_valid;
static AdaptiveKalman visual_filter;
static Visual_Servo_t visual_servo;
static uint32_t visual_seq, visual_source_ms, visual_received_ms;
static uint8_t visual_started;
volatile float tuning_spin_sample[8];
static Spin_Observer_t spin_observer;
static float spin_requested_wz;
static unsigned spin_stage;
static unsigned spin_turns;

typedef enum { INACTIVE, PREPARE, RUN, STATIC_SETTLE, RETURN_SETTLE, TIMED_SETTLE, HOLD,
               BRAKE, WAIT_DISABLED, READ_PARAMETER } Work_State;
static Work_State work;
static uint32_t last_heartbeat, heartbeat_ms, stage_ms, motion_ms, motion_duration_ms;
static uint32_t motion_elapsed_ms, sample_ms;
static uint32_t disable_sequence[3], parameter_sequence;
static uint8_t enable_requested, motion_started, read_after_disable;
static uint8_t parameter_reg, parameter_motor, parameter_waiting;
static uint8_t sample_published;
static float requested_vx, requested_wz;
static uint32_t active_command;
static uint8_t odometry_reset, endpoint_captured, endpoint_stopped;
static Chassis_Translation_Direction_t requested_direction;
static float requested_distance, requested_speed, distance_axis_x, distance_axis_y;
static double origin_x, origin_y, endpoint_progress;
static Chassis_Odometry_t endpoint;
static uint32_t endpoint_elapsed_ms;
static float return_goal[3], return_linear_limit, return_angular_limit;
static uint8_t return_velocity_started;
static Point_Plan_t return_plan;

static uint8_t Hold_Command(void)
{
    return active_command == TUNING_RETURN_HOLD || active_command == TUNING_VISUAL_HOLD ||
        active_command == TUNING_SPIN_SWEEP;
}


static uint8_t Odometry_Command(void)
{
    return active_command == TUNING_ODOM_STATIC || active_command == TUNING_ODOM_DISTANCE ||
        Hold_Command();
}

static double Return_Remaining(const Chassis_Odometry_t *data)
{
    return hypot((double)return_goal[0] - data->x_m, (double)return_goal[1] - data->y_m);
}

static uint8_t Return_Fence(const Chassis_Odometry_t *data)
{
    const double distance = hypot(data->x_m - origin_x, data->y_m - origin_y);
    return isfinite(distance) && distance <= TUNING_RETURN_FENCE_M && isfinite(data->yaw_rad);
}

static uint8_t Return_At_Goal(const Chassis_Odometry_t *data)
{
    if (active_command == TUNING_SPIN_SWEEP)
        return Return_Fence(data) && hypot(data->x_m-origin_x, data->y_m-origin_y) <= .35 &&
            fabs(spin_observer.yaw_rad - spin_turns * SPIN_RADIANS_PER_TURN) <= .2;
    const float tolerance = active_command == TUNING_VISUAL_HOLD ?
        TUNING_VISUAL_POSITION_TOLERANCE_M : TUNING_RETURN_POSITION_TOLERANCE_M;
    return Return_Fence(data) && Return_Remaining(data) <= tolerance &&
        fabsf(remainderf(return_goal[2] - data->yaw_rad, 6.2831853071795864769f)) <=
            TUNING_RETURN_YAW_TOLERANCE_RAD;
}

static double Odometry_Progress(const Chassis_Odometry_t *data)
{
    return Hold_Command() ? requested_distance - Return_Remaining(data) :
        (data->x_m - origin_x) * distance_axis_x + (data->y_m - origin_y) * distance_axis_y;
}

static uint8_t Static_Position_Valid(const Chassis_Odometry_t *data)
{
    const double drift = hypot(data->x_m - origin_x, data->y_m - origin_y);
    return isfinite(drift) && drift <= TUNING_STATIC_DRIFT_MAX_M;
}

/* 必须在 Stop 清除定距完成状态及 Disable 撤销 valid 之前保留终点证据。 */
static void Capture_Endpoint(uint32_t now_ms)
{
    if (endpoint_captured || !Odometry_Command() ||
        (work != PREPARE && work != RUN && work != STATIC_SETTLE && work != RETURN_SETTLE))
        return;
    Chassis_Get_Odometry(&endpoint);
    endpoint_stopped = Chassis_Is_Stopped();
    endpoint_elapsed_ms = motion_started ? now_ms - motion_ms : 0U;
    endpoint_progress = Odometry_Progress(&endpoint);
    endpoint_captured = 1U;
}

static void Finish(void)
{
    if (motion_started)
        motion_elapsed_ms = (uint32_t)xTaskGetTickCount() - motion_ms;
    work = INACTIVE;
    tuning_phase = tuning_error ? TUNING_FAULT : TUNING_DONE;
    sample_published = 0U; /* 同拍 Snapshot 发布完成结果，不让主机读到旧阶段。 */
}

static uint8_t Fresh_IMU(uint32_t now_ms, INS_control_data_t *imu)
{
    return INS_Get_Data(imu) && isfinite(imu->yaw_deg) && isfinite(imu->world_z_rad_s) &&
        (uint32_t)(now_ms - imu->sample_time_ms) <= 20U;
}

static uint8_t Observe_Spin(uint32_t now_ms, const INS_control_data_t *imu)
{
    float position[3];
    taskENTER_CRITICAL();
    for (unsigned i = 0; i < 3U; ++i)
        position[i] = -dm_motor[i].position; /* 本车安装方向-1、齿比1；与正式里程计相同。 */
    taskEXIT_CRITICAL();
    return Spin_Observer_Update(&spin_observer, position,
        imu->yaw_deg * .01745329251994329577f, imu->world_z_rad_s, now_ms);
}

static void Request_Disable(uint32_t now_ms)
{
    taskENTER_CRITICAL();
    for (unsigned i = 0; i < 3U; ++i)
        disable_sequence[i] = dm_motor[i].feedback_seq;
    taskEXIT_CRITICAL();
    Chassis_Disable();
    work = WAIT_DISABLED;
    tuning_phase = TUNING_BRAKING;
    stage_ms = now_ms;
}

/* 反复 STOP 不能延长当前制动/失能回执的截止时间。 */
static void Begin_Brake(uint32_t now_ms, uint32_t error)
{
    read_after_disable = 0U;
    if (error && !tuning_error)
        tuning_error = error;
    if (work == BRAKE || work == WAIT_DISABLED)
        return;
    const uint32_t brake_start_ms = (work == STATIC_SETTLE || work == RETURN_SETTLE) ? stage_ms : now_ms;
    Capture_Endpoint(now_ms);
    Chassis_Stop();
    work = BRAKE;
    tuning_phase = TUNING_BRAKING;
    stage_ms = brake_start_ms; /* 静态确认中的异常不能重新延长既有 500 ms 期限。 */
    if (!Chassis_Is_Enabled())
        Request_Disable(now_ms);
}

/* 完成判据与结束快照间仍可能更新反馈；重新核对保留证据才能宣告正常完成。 */
static void Complete_Odometry(uint32_t now_ms)
{
    Capture_Endpoint(now_ms);
    const uint8_t valid = endpoint.valid && endpoint_stopped &&
        isfinite(endpoint.x_m) && isfinite(endpoint.y_m) && isfinite(endpoint.yaw_rad) &&
        (active_command != TUNING_ODOM_STATIC || Static_Position_Valid(&endpoint)) &&
        (!Hold_Command() || Return_At_Goal(&endpoint)) &&
        isfinite(endpoint_progress) && (active_command != TUNING_ODOM_DISTANCE ||
        fabs(requested_distance - endpoint_progress) <= CHASSIS_DISTANCE_TOLERANCE_M);
    if (valid && work == RETURN_SETTLE)
    {
        work = HOLD;
        motion_elapsed_ms = now_ms - motion_ms;
        tuning_phase = TUNING_DONE;
        sample_published = 0U;
    }
    else if (valid && work == STATIC_SETTLE)
        Request_Disable(now_ms); /* 已有真实停车证据，不再开始第二个停车窗口。 */
    else
        Begin_Brake(now_ms, valid ? 0U : TUNING_ERROR_ODOMETRY);
}

static uint8_t Disabled_Feedback(uint32_t now_ms)
{
    uint8_t result = 1U;
    taskENTER_CRITICAL();
    for (unsigned i = 0; i < 3U; ++i)
        if (dm_motor[i].id != i + 1U || dm_motor[i].state != 0U ||
            dm_motor[i].feedback_seq == disable_sequence[i] ||
            (uint32_t)(now_ms - dm_motor[i].feedback_time_ms) > 500U)
            result = 0U;
    taskEXIT_CRITICAL();
    return result;
}

static uint8_t Valid_Config(const float args[8])
{
    for (unsigned i = 0; i < 3U; ++i)
        if (!isfinite(args[i]) || args[i] < 0.05f || args[i] > 0.5f ||
            !isfinite(args[4U + i]) || args[4U + i] < 0.0f || args[4U + i] > 0.03f)
            return 0U;
    return isfinite(args[3]) && args[3] >= 0.0f && args[3] <= 1.0f;
}

static uint8_t Valid_Move(const float args[8])
{
    for (unsigned i = 0; i < 4U; ++i)
        if (!isfinite(args[i]))
            return 0U;
    return fabsf(args[0]) <= 0.25f && args[1] == 0.0f && fabsf(args[2]) <= 0.5f &&
        ((args[0] != 0.0f) != (args[2] != 0.0f)) &&
        args[3] >= 100.0f && args[3] <= 1500.0f && floorf(args[3]) == args[3];
}

static uint8_t Valid_Spin(uint32_t command, const float args[8])
{
    static const float profile[7] = {2, 3, 4, 3, 2, 120000, 0};
    if (command != TUNING_SPIN_SWEEP ||
        (args[0] != SPIN_DEFAULT_TURNS && args[0] != SPIN_EXTENDED_TURNS)) return 0U;
    for (unsigned i = 0; i < 7U; ++i)
        if (args[i + 1U] != profile[i]) return 0U;
    return 1U;
}

static uint8_t Valid_Odometry(uint32_t command, const float args[8])
{
    for (unsigned i = 0U; i < 8U; ++i)
        if (!isfinite(args[i]) || (i >= 4U && args[i] != 0.0f))
            return 0U;
    if (args[3] < 100.0f || floorf(args[3]) != args[3])
        return 0U;
    if (command == TUNING_ODOM_STATIC)
        return args[0] == 0.0f && args[1] == 0.0f && args[2] == 0.0f && args[3] <= 1500.0f;
    return command == TUNING_ODOM_DISTANCE &&
        (args[0] == CHASSIS_TRANSLATION_FORWARD || args[0] == CHASSIS_TRANSLATION_BACKWARD) &&
        args[1] >= 0.01f && args[1] <= 1.0f && args[2] >= 0.02f && args[2] <= 0.10f &&
        args[3] <= 20000.0f;
}

static uint8_t Valid_Return(uint32_t command, const float args[8])
{
    if (command != TUNING_RETURN_HOLD && command != TUNING_VISUAL_HOLD)
        return 0U;
    for (unsigned i = 0U; i < 8U; ++i)
        if (!isfinite(args[i]))
            return 0U;
    return fabsf(args[2]) <= 3.14159265358979323846f &&
        args[3] >= 0.02f && args[3] <= 0.20f && args[4] >= 0.10f && args[4] <= 0.50f &&
        args[5] >= 100.0f && args[5] <= 20000.0f && floorf(args[5]) == args[5] &&
        args[6] == 0.0f && args[7] == 0.0f;
}

/* 全向连续接近；速度仍经底盘既有斜坡，进入容差才明确Stop并等待停车。 */
static void Service_Return(uint32_t now_ms, const Chassis_Odometry_t *data)
{
    if (!Return_Fence(data))
    {
        Begin_Brake(now_ms, TUNING_ERROR_ODOMETRY);
        return;
    }
    if ((uint32_t)(now_ms - motion_ms) >= motion_duration_ms)
    {
        Begin_Brake(now_ms, TUNING_ERROR_ACTION_TIMEOUT);
        return;
    }
    const Point_Plan_Config_t config = {
        .linear_limit_mps = return_linear_limit, .angular_limit_rad_s = return_angular_limit,
        .linear_deceleration_mps2 = 0.30f, .angular_deceleration_rad_s2 = 0.8f,
        .latency_s = 0.12f, .position_tolerance_m = TUNING_RETURN_STOP_POSITION_M,
        .yaw_tolerance_rad = TUNING_RETURN_STOP_YAW_RAD,
    };
    const double pose[3] = {data->x_m, data->y_m, data->yaw_rad};
    Point_Plan_Output_t output;
    if (!Point_Plan_Step(&return_plan, &config, pose, &output))
    {
        Begin_Brake(now_ms, TUNING_ERROR_ODOMETRY);
        return;
    }
    if (output.arrived)
    {
        Chassis_Hold();
        work = RETURN_SETTLE;
        tuning_phase = TUNING_BRAKING;
        stage_ms = now_ms;
        return;
    }
    if (Chassis_Set_Velocity_Explicit_Yaw(output.vx_mps, output.vy_mps, output.wz_rad_s))
        return_velocity_started = 1U;
    else if (return_velocity_started || (uint32_t)(now_ms - motion_ms) >= TUNING_RETURN_START_MS)
        Begin_Brake(now_ms, 4U);
    /* PREPARE与首个速度提交之间可能收到新的停车反馈；尚未动时允许有限等待。
     * RUN每拍仍先检查使能、三轮/IMU/里程计新鲜度、心跳与总时限。 */
}

/* 注入帧经过正式图像滤波和速度规划；里程计只做区域/终点检查。 */
static void Service_Visual(uint32_t now_ms, const Chassis_Odometry_t *data)
{
    if (!Return_Fence(data) || (uint32_t)(now_ms - motion_ms) >= motion_duration_ms)
    {
        Begin_Brake(now_ms, TUNING_ERROR_ACTION_TIMEOUT);
        return;
    }
    const uint32_t seq = tuning_visual_seq;
    if (seq != visual_seq)
    {
        const float raw[2] = {tuning_visual_center[0], tuning_visual_center[1]};
        const uint32_t stamp = tuning_visual_time_ms, valid = tuning_visual_valid;
        if (seq != tuning_visual_seq)
            return;
        visual_seq = seq;
        if (valid != 1U || !isfinite(raw[0]) || !isfinite(raw[1]) ||
            fabsf(raw[0]) > 32767.0f || fabsf(raw[1]) > 32767.0f)
        {
            Begin_Brake(now_ms, 1U);
            return;
        }
        const uint32_t elapsed = stamp - visual_source_ms;
        if (!visual_started || (elapsed > 0U && elapsed <= INT32_MAX))
        {
            const float seconds = visual_started && elapsed < 250U ? elapsed * 0.001f : 0.0f;
            visual_started = 1U;
            visual_source_ms = stamp;
            visual_received_ms = now_ms;
            AdaptiveKalman_Update(&visual_filter, raw[0], raw[1], stamp);
            const float goal[2] = {5000.0f, 7500.0f}, tolerance[2] = {100.0f, 100.0f};
            const Visual_Servo_Config_t config = {
                .max_speed_mps = return_linear_limit,
                .acceleration_mps2 = VISUAL_TRACKING_ACCEL_MPS2,
                .deceleration_mps2 = VISUAL_TRACKING_DECEL_MPS2,
                .latency_s = VISUAL_TRACKING_LATENCY_S,
                .response_q_per_m = {VISUAL_TRACKING_RESPONSE_X_Q_PER_M, VISUAL_TRACKING_RESPONSE_Y_Q_PER_M},
            };
            Tracked_Visual_Servo_Update(&visual_servo, &config, &visual_filter, raw, goal, tolerance, seconds);
            if (visual_filter.accepted_mask == 3U && fabsf(raw[0] - goal[0]) <= tolerance[0] &&
                fabsf(raw[1] - goal[1]) <= tolerance[1])
            {
                Chassis_Hold();
                work = RETURN_SETTLE;
                tuning_phase = TUNING_BRAKING;
                stage_ms = now_ms;
                return;
            }
        }
    }
    if ((uint32_t)(now_ms - (visual_started ? visual_received_ms : motion_ms)) >= 250U)
    {
        Begin_Brake(now_ms, 5U);
        return;
    }
    if (!visual_started)
        return;
    /* 视觉的两轴短暂归零后允许正常停车确认，再继续新帧；不拿尾速当故障。 */
    if (!return_velocity_started && !Chassis_Is_Stopped())
        return;
    if (Chassis_Set_Velocity(visual_servo.speed[1] * VISUAL_TRACKING_Y_TO_FORWARD,
                             visual_servo.speed[0] * VISUAL_TRACKING_X_TO_LEFT, 0.0f))
        return_velocity_started = visual_servo.speed[0] != 0.0f || visual_servo.speed[1] != 0.0f;
    else if (return_velocity_started || (uint32_t)(now_ms - motion_ms) >= TUNING_RETURN_START_MS)
        Begin_Brake(now_ms, 4U);
}

static void Service_Spin(uint32_t now_ms, const Chassis_Odometry_t *data)
{
    if ((uint32_t)(now_ms-motion_ms) >= motion_duration_ms ||
        hypot(data->x_m-origin_x, data->y_m-origin_y) > .35 ||
        spin_observer.yaw_rad < -.2 || spin_observer.yaw_rad > spin_turns * SPIN_RADIANS_PER_TURN + .2)
    {
        Begin_Brake(now_ms, TUNING_ERROR_ACTION_TIMEOUT);
        return;
    }
    spin_requested_wz = Spin_Test_Speed(spin_observer.yaw_rad, spin_turns, &spin_stage);
    if (spin_requested_wz == 0.0f)
    {
        Chassis_Hold();
        work = RETURN_SETTLE;
        tuning_phase = TUNING_BRAKING;
        stage_ms = now_ms;
    }
    else if (Chassis_Set_Test_Rotation(spin_requested_wz))
        return_velocity_started = 1U;
    else if (return_velocity_started || (uint32_t)(now_ms-motion_ms) >= TUNING_RETURN_START_MS)
        Begin_Brake(now_ms, 4U);
}

static void Accept_Command(uint32_t command, const float args[8], uint32_t now_ms)
{
    if (command == TUNING_STOP)
    {
        if (work == INACTIVE)
            tuning_error = 0U;
        Begin_Brake(now_ms, 0U);
        return;
    }
    if (work != INACTIVE && !(work == HOLD && (command == TUNING_RETURN_HOLD ||
        command == TUNING_VISUAL_HOLD || command == TUNING_SPIN_SWEEP)))
    {
        Begin_Brake(now_ms, 2U);
        return;
    }
    tuning_error = 0U;
    motion_started = 0U;
    motion_elapsed_ms = 0U;
    if (command == TUNING_CONFIG && Valid_Config(args))
    {
        if (!Chassis_Set_Tuning(args, args[3], &args[4]))
            tuning_error = 2U;
        Finish();
    }
    else if (command == TUNING_READ_PARAM && isfinite(args[0]) &&
             args[0] >= 1.0f && args[0] <= 35.0f && floorf(args[0]) == args[0])
    {
        parameter_reg = (uint8_t)args[0];
        parameter_motor = parameter_waiting = 0U;
        read_after_disable = 1U;
        Request_Disable(now_ms);
    }
    else if ((command == TUNING_MOVE && Valid_Move(args)) || Valid_Odometry(command, args) ||
             Valid_Return(command, args) || Valid_Spin(command, args))
    {
        active_command = command;
        if (command == TUNING_SPIN_SWEEP)
            spin_turns = (unsigned)args[0];
        requested_vx = args[0];
        requested_wz = args[2];
        requested_direction = command == TUNING_ODOM_DISTANCE ?
            (Chassis_Translation_Direction_t)args[0] : CHASSIS_TRANSLATION_NONE;
        requested_distance = command == TUNING_ODOM_DISTANCE ? args[1] : 0.0f;
        requested_speed = command == TUNING_ODOM_DISTANCE ? args[2] : 0.0f;
        odometry_reset = endpoint_captured = endpoint_stopped = 0U;
        return_velocity_started = 0U;
        visual_started = 0U;
        visual_seq = tuning_visual_seq;
        visual_received_ms = now_ms;
        visual_servo = (Visual_Servo_t){0};
        spin_observer = (Spin_Observer_t){0};
        spin_requested_wz = 0.0f;
        spin_stage = 0U;
        AdaptiveKalman_Init(&visual_filter, VISUAL_TRACKING_SIGMA_X_Q,
                            VISUAL_TRACKING_SIGMA_Y_Q, VISUAL_TRACKING_SIGMA_ACCEL_Q_S2);
        endpoint = (Chassis_Odometry_t){0};
        origin_x = origin_y = endpoint_progress = 0.0;
        distance_axis_x = distance_axis_y = 0.0f;
        endpoint_elapsed_ms = 0U;
        motion_duration_ms = (uint32_t)args[active_command == TUNING_SPIN_SWEEP ? 6U : Hold_Command() ? 5U : 3U];
        if (Hold_Command() && active_command != TUNING_SPIN_SWEEP)
        {
            for (unsigned i = 0U; i < 3U; ++i)
                return_goal[i] = args[i];
            return_linear_limit = args[3];
            return_angular_limit = args[4];
        }
        last_heartbeat = tuning_heartbeat;
        heartbeat_ms = stage_ms = now_ms;
        enable_requested = 0U;
        work = PREPARE;
        tuning_phase = TUNING_PREPARING;
    }
    else
        Begin_Brake(now_ms, 1U);
}

void Live_Tuning_Init(void)
{
    spin_turns = SPIN_DEFAULT_TURNS;
    spin_observer = (Spin_Observer_t){0};
    spin_requested_wz = 0.0f;
    spin_stage = 0U;
    for (unsigned i = 0U; i < 8U; ++i) tuning_spin_sample[i] = 0.0f;
    tuning_request_seq = tuning_command = tuning_heartbeat = tuning_ack_seq = 0U;
    tuning_visual_seq = tuning_visual_time_ms = tuning_visual_valid = 0U;
    visual_seq = visual_source_ms = visual_received_ms = 0U;
    visual_started = 0U;
    visual_filter = (AdaptiveKalman){0};
    visual_servo = (Visual_Servo_t){0};
    for (unsigned i = 0U; i < 2U; ++i) tuning_visual_center[i] = 0.0f;
    for (unsigned i = 0U; i < 10U; ++i) tuning_visual_sample[i] = 0.0f;
    tuning_phase = TUNING_IDLE;
    tuning_error = tuning_sample_seq = 0U;
    work = INACTIVE;
    enable_requested = motion_started = read_after_disable = 0U;
    parameter_motor = parameter_waiting = 0U;
    motion_ms = motion_duration_ms = motion_elapsed_ms = sample_ms = 0U;
    sample_published = 0U;
    active_command = 0U;
    odometry_reset = endpoint_captured = endpoint_stopped = 0U;
    endpoint = (Chassis_Odometry_t){0};
    requested_distance = requested_speed = distance_axis_x = distance_axis_y = 0.0f;
    origin_x = origin_y = endpoint_progress = 0.0;
    endpoint_elapsed_ms = 0U;
    for (unsigned i = 0U; i < 8U; ++i)
        tuning_args[i] = 0.0f;
    for (unsigned i = 0U; i < TUNING_SAMPLE_COUNT; ++i)
        tuning_sample[i] = 0.0f;
}

void Live_Tuning_Loop(void)
{
    const uint32_t now_ms = (uint32_t)xTaskGetTickCount();
    const uint32_t request = tuning_request_seq;
    if (request != tuning_ack_seq)
    {
        float args[8];
        const uint32_t command = tuning_command;
        for (unsigned i = 0; i < 8U; ++i)
            args[i] = tuning_args[i];
        if (request == tuning_request_seq)
        {
            Accept_Command(command, args, now_ms);
            tuning_ack_seq = request;
        }
    }
    if (tuning_heartbeat != last_heartbeat)
    {
        last_heartbeat = tuning_heartbeat;
        heartbeat_ms = now_ms;
    }
    if ((work == PREPARE || work == RUN || work == STATIC_SETTLE || work == RETURN_SETTLE ||
         work == TIMED_SETTLE) && (uint32_t)(now_ms - heartbeat_ms) >= 300U)
        Begin_Brake(now_ms, 5U);

    if (active_command == TUNING_SPIN_SWEEP && motion_started &&
        (work == RUN || work == RETURN_SETTLE || work == HOLD))
    {
        INS_control_data_t imu;
        if (!Fresh_IMU(now_ms, &imu) || !Observe_Spin(now_ms, &imu))
            Begin_Brake(now_ms, TUNING_ERROR_ODOMETRY);
    }

    if (work == PREPARE)
    {
        INS_control_data_t imu;
        if ((uint32_t)(now_ms - stage_ms) >= 1000U)
            Begin_Brake(now_ms, 3U);
        else if (enable_requested && !Chassis_Is_Enabled())
            Begin_Brake(now_ms, 4U);
        else if (Fresh_IMU(now_ms, &imu))
        {
            if (!enable_requested)
                enable_requested = Chassis_Enable();
            if (enable_requested && Chassis_Is_Stopped())
            {
                if (Odometry_Command() && !Hold_Command() && !odometry_reset)
                {
                    if (!Chassis_Reset_Odometry())
                        Begin_Brake(now_ms, 3U);
                    else
                        odometry_reset = 1U;
                    /* 重置撤销 valid；下次 Loop 再等控制层的新基线。 */
                }
                else
                {
                    Chassis_Odometry_t data;
                    if (!Chassis_Get_Odometry(&data))
                        return;
                    uint8_t started = 1U;
                    if (active_command == TUNING_MOVE)
                        started = Chassis_Set_Velocity(requested_vx, 0.0f, requested_wz);
                    else
                    {
                        origin_x = data.x_m;
                        origin_y = data.y_m;
                        const float heading = CHASSIS_STRAIGHT_YAW_SIGN * imu.yaw_deg *
                                              0.01745329251994329577f;
                        const float sign = requested_direction == CHASSIS_TRANSLATION_BACKWARD ? -1.0f : 1.0f;
                        distance_axis_x = sign * cosf(heading);
                        distance_axis_y = sign * sinf(heading);
                        if (active_command == TUNING_ODOM_DISTANCE)
                            started = Chassis_Distance_Start(requested_direction, requested_distance, requested_speed);
                        else if (active_command == TUNING_SPIN_SWEEP)
                        {
                            return_goal[0] = data.x_m; return_goal[1] = data.y_m; return_goal[2] = data.yaw_rad;
                            requested_distance = 0.0f;
                            started = Observe_Spin(now_ms, &imu);
                        }
                        else if (Hold_Command())
                        {
                            const double start[3] = {data.x_m, data.y_m, data.yaw_rad};
                            const double goal[3] = {return_goal[0], return_goal[1], return_goal[2]};
                            started = Point_Plan_Init(&return_plan, start, goal);
                            requested_distance = (float)Return_Remaining(&data);
                            if (!isfinite(requested_distance) || requested_distance > TUNING_RETURN_MAX_DISTANCE_M)
                                started = 0U;
                        }
                    }
                    if (!started)
                        Begin_Brake(now_ms, 3U);
                    else
                    {
                        work = RUN;
                        tuning_phase = TUNING_RUNNING;
                        stage_ms = now_ms;
                        if (Odometry_Command())
                        {
                            motion_started = 1U;
                            motion_ms = now_ms;
                        }
                    }
                }
            }
        }
    }
    else if (work == RUN)
    {
        INS_control_data_t imu;
        if (!Chassis_Is_Enabled() || !Chassis_Motors_Ready() || !Fresh_IMU(now_ms, &imu))
            Begin_Brake(now_ms, 4U);
        else if (Odometry_Command())
        {
            Chassis_Odometry_t data;
            if (!Chassis_Get_Odometry(&data))
                Begin_Brake(now_ms, TUNING_ERROR_ODOMETRY);
            else if (active_command == TUNING_SPIN_SWEEP)
                Service_Spin(now_ms, &data);
            else if (active_command == TUNING_VISUAL_HOLD)
                Service_Visual(now_ms, &data);
            else if (active_command == TUNING_RETURN_HOLD)
                Service_Return(now_ms, &data);
            else if (active_command == TUNING_ODOM_STATIC)
            {
                if (!Static_Position_Valid(&data))
                    Begin_Brake(now_ms, TUNING_ERROR_ODOMETRY);
                else if ((uint32_t)(now_ms - motion_ms) >= motion_duration_ms)
                {
                    /* 采样阶段只有零目标；短暂失去三组停车资格不等于动作故障。
                     * 期满仍须在有限窗口内重新确认真实停车，才允许成功失能。
                     */
                    Chassis_Stop();
                    work = STATIC_SETTLE;
                    tuning_phase = TUNING_BRAKING;
                    stage_ms = now_ms;
                }
            }
            else
            {
                const double progress = Odometry_Progress(&data);
                if (!isfinite(progress) || progress > requested_distance + CHASSIS_DISTANCE_TOLERANCE_M)
                    Begin_Brake(now_ms, TUNING_ERROR_ODOMETRY);
                else if ((uint32_t)(now_ms - motion_ms) >= motion_duration_ms)
                    Begin_Brake(now_ms, TUNING_ERROR_ACTION_TIMEOUT);
                else if (Chassis_Is_Stopped())
                {
                    if (fabs(requested_distance - progress) > CHASSIS_DISTANCE_TOLERANCE_M)
                        Begin_Brake(now_ms, TUNING_ERROR_ODOMETRY);
                    else
                        Complete_Odometry(now_ms);
                }
            }
        }
        else
        {
            if (!motion_started && (chassis_motor_target[0] != 0.0f ||
                chassis_motor_target[1] != 0.0f || chassis_motor_target[2] != 0.0f))
            {
                motion_started = 1U;
                motion_ms = now_ms;
            }
            if (motion_started && (uint32_t)(now_ms - motion_ms) >= motion_duration_ms)
            {
                Chassis_Hold();
                work = TIMED_SETTLE;
                tuning_phase = TUNING_BRAKING;
                stage_ms = now_ms;
            }
            else if (!motion_started && (uint32_t)(now_ms - stage_ms) >= 100U)
                Begin_Brake(now_ms, 4U);
        }
    }
    else if (work == TIMED_SETTLE)
    {
        INS_control_data_t imu;
        if (!Chassis_Is_Enabled() || !Chassis_Motors_Ready() || !Fresh_IMU(now_ms, &imu))
            Begin_Brake(now_ms, 4U);
        else if (Chassis_Is_Stopped())
            Request_Disable(now_ms);
        else if ((uint32_t)(now_ms - stage_ms) >= TUNING_RETURN_SETTLE_MS)
        {
            tuning_error = 8U;
            Request_Disable(now_ms);
        }
    }
    else if (work == STATIC_SETTLE)
    {
        INS_control_data_t imu;
        Chassis_Odometry_t data;
        if ((uint32_t)(now_ms - stage_ms) >= 500U)
        {
            tuning_error = 8U;
            Capture_Endpoint(now_ms);
            Request_Disable(now_ms);
        }
        else if (!Chassis_Is_Enabled() || !Chassis_Motors_Ready() || !Fresh_IMU(now_ms, &imu))
            Begin_Brake(now_ms, 4U);
        else if (!Chassis_Get_Odometry(&data) || !Static_Position_Valid(&data))
            Begin_Brake(now_ms, TUNING_ERROR_ODOMETRY);
        else if (Chassis_Is_Stopped())
            Complete_Odometry(now_ms);
    }
    else if (work == RETURN_SETTLE || work == HOLD)
    {
        INS_control_data_t imu;
        Chassis_Odometry_t data;
        if (work == RETURN_SETTLE && (uint32_t)(now_ms - motion_ms) >= motion_duration_ms)
            Begin_Brake(now_ms, TUNING_ERROR_ACTION_TIMEOUT);
        /* 现场零速后反馈尾迹约500ms仍略高门限，继续零目标等待，不降低停稳门限。 */
        else if (work == RETURN_SETTLE && (uint32_t)(now_ms - stage_ms) >= TUNING_RETURN_SETTLE_MS)
        {
            tuning_error = 8U;
            Capture_Endpoint(now_ms);
            Request_Disable(now_ms);
        }
        else if (!Chassis_Is_Enabled() || !Chassis_Motors_Ready() || !Fresh_IMU(now_ms, &imu))
            Begin_Brake(now_ms, 4U);
        else if (!Chassis_Get_Odometry(&data) || !Return_Fence(&data) ||
                 (active_command == TUNING_SPIN_SWEEP && hypot(data.x_m-origin_x, data.y_m-origin_y) > .35))
            Begin_Brake(now_ms, TUNING_ERROR_ODOMETRY);
        else if (work == RETURN_SETTLE && Chassis_Is_Stopped())
        {
            if (Return_At_Goal(&data))
                Complete_Odometry(now_ms);
            else
                Begin_Brake(now_ms, TUNING_ERROR_ODOMETRY);
        }
        /* HOLD始终留在零目标IDLE；不自动纠偏、不因主机结束而重新启动。 */
    }
    else if (work == BRAKE)
    {
        if (!Chassis_Is_Enabled() || Chassis_Is_Stopped())
            Request_Disable(now_ms);
        else if ((uint32_t)(now_ms - stage_ms) >= 500U)
        {
            if (!tuning_error)
                tuning_error = 8U;
            Request_Disable(now_ms);
        }
    }
    else if (work == WAIT_DISABLED)
    {
        if (Disabled_Feedback(now_ms))
        {
            if (read_after_disable && !tuning_error)
            {
                read_after_disable = 0U;
                work = READ_PARAMETER;
                tuning_phase = TUNING_PREPARING;
                stage_ms = now_ms;
            }
            else
                Finish();
        }
        else if ((uint32_t)(now_ms - stage_ms) >= 500U)
        {
            /* 缺失电机也需能读取其余电机作只读诊断。FD 已请求且软件禁用时，
             * READ 可继续查询，但不伪造三轮反馈/停稳。STOP 和 MOVE 仍须真实回执。
             * 读请求本身另检查电机使能状态，缺失回复仍按参数超时报错。
             */
            if (read_after_disable && !tuning_error && !Chassis_Is_Enabled())
            {
                read_after_disable = 0U;
                work = READ_PARAMETER;
                tuning_phase = TUNING_PREPARING;
                stage_ms = now_ms;
            }
            else
            {
                if (!tuning_error)
                    tuning_error = 8U;
                Finish();
            }
        }
    }
    else if (work == READ_PARAMETER)
    {
        if (Chassis_Is_Enabled())
        {
            Begin_Brake(now_ms, 4U);
            return;
        }
        if (!parameter_waiting)
        {
            parameter_sequence = dm_param_reply[parameter_motor].sequence;
            HAL_StatusTypeDef status = DM_Motor_ReadParam(parameter_motor + 1U, parameter_reg);
            if (status == HAL_OK)
            {
                parameter_waiting = 1U;
                stage_ms = now_ms;
            }
            else if (status != HAL_BUSY || (uint32_t)(now_ms - stage_ms) >= 200U)
            {
                tuning_error = 7U;
                Finish();
            }
        }
        else if (dm_param_reply[parameter_motor].sequence != parameter_sequence &&
                 dm_param_reply[parameter_motor].reg == parameter_reg)
        {
            parameter_waiting = 0U;
            stage_ms = now_ms;
            if (++parameter_motor == 3U)
                Finish();
        }
        else if ((uint32_t)(now_ms - stage_ms) >= 200U)
        {
            tuning_error = 6U;
            Finish();
        }
    }
}

void Live_Tuning_Snapshot(void)
{
    const uint32_t now_ms = (uint32_t)xTaskGetTickCount();
    if (sample_published && (uint32_t)(now_ms - sample_ms) <
        (active_command == TUNING_VISUAL_HOLD || active_command == TUNING_SPIN_SWEEP ? 40U : 100U))
        return;
    sample_published = 1U;
    sample_ms = now_ms;
    INS_control_data_t imu;
    const uint8_t imu_valid = INS_Get_Data(&imu);
    Chassis_Odometry_t data;
    Chassis_Get_Odometry(&data);
    ++tuning_sample_seq;
    tuning_sample[0] = (float)now_ms;
    tuning_sample[1] = (float)tuning_phase;
    tuning_sample[2] = (float)tuning_error;
    tuning_sample[3] = (float)Chassis_Is_Enabled();
    tuning_sample[4] = imu_valid ? imu.yaw_deg : NAN;
    tuning_sample[5] = imu_valid ? imu.world_z_rad_s : NAN;
    taskENTER_CRITICAL();
    for (unsigned i = 0U; i < 3U; ++i)
    {
        tuning_sample[6U + i] = chassis_speed[i];
        tuning_sample[9U + i] = dm_motor[i].position;
        tuning_sample[12U + i] = dm_motor[i].velocity;
        tuning_sample[15U + i] = dm_motor[i].torque;
        tuning_sample[18U + i] = dm_motor[i].state;
        tuning_sample[21U + i] = dm_motor[i].t_mos;
        tuning_sample[24U + i] = chassis_motor_target[i];
        tuning_sample[27U + i] = chassis_motor_ff[i];
        tuning_sample[30U + i] = chassis_tuning_kd[i];
        tuning_sample[34U + i] = chassis_tuning_friction[i];
    }
    taskEXIT_CRITICAL();
    tuning_sample[33] = chassis_tuning_ff_gain;
    tuning_sample[37] = battery_voltage;
    tuning_sample[38] = imu_valid ? (float)(uint32_t)(now_ms - imu.sample_time_ms) : -1.0f;
    tuning_sample[39] = motion_started ? (float)(work == INACTIVE || work == HOLD ? motion_elapsed_ms :
                                                (uint32_t)(now_ms - motion_ms)) : 0.0f;
    tuning_sample[40] = (float)data.x_m;
    tuning_sample[41] = (float)data.y_m;
    tuning_sample[42] = data.yaw_rad;
    tuning_sample[43] = data.vx_mps;
    tuning_sample[44] = data.vy_mps;
    tuning_sample[45] = data.wheel_wz_rad_s;
    tuning_sample[46] = (float)data.valid;
    tuning_sample[47] = (float)(uint32_t)(__HAL_TIM_GET_COUNTER(&htim2) - data.sample_time_us) * 0.001f;
    tuning_sample[48] = (float)endpoint.x_m;
    tuning_sample[49] = (float)endpoint.y_m;
    tuning_sample[50] = endpoint.yaw_rad;
    tuning_sample[51] = (float)endpoint.valid;
    tuning_sample[52] = (float)endpoint_stopped;
    tuning_sample[53] = (float)endpoint_elapsed_ms;
    tuning_sample[54] = (float)endpoint_progress;
    tuning_sample[55] = requested_distance;
    tuning_visual_sample[0] = tuning_visual_center[0];
    tuning_visual_sample[1] = tuning_visual_center[1];
    for (unsigned axis = 0; axis < 2U; ++axis)
    {
        tuning_visual_sample[2U + axis] = visual_filter.position[axis];
        tuning_visual_sample[4U + axis] = visual_filter.velocity[axis];
        tuning_visual_sample[6U + axis] = visual_servo.speed[axis];
    }
    tuning_visual_sample[8] = visual_filter.accepted_mask;
    tuning_visual_sample[9] = visual_seq;
    tuning_spin_sample[0] = (float)spin_observer.encoder_rad;
    tuning_spin_sample[1] = (float)spin_observer.yaw_rad;
    tuning_spin_sample[2] = (float)spin_observer.gyro_rad;
    tuning_spin_sample[3] = spin_requested_wz;
    tuning_spin_sample[4] = (float)spin_stage;
    tuning_spin_sample[5] = (float)(spin_turns * SPIN_RADIANS_PER_TURN);
    tuning_spin_sample[6] = (float)(spin_observer.yaw_rad - spin_observer.encoder_rad);
    tuning_spin_sample[7] = data.wheel_wz_rad_s;
    ++tuning_sample_seq;
    float channels[TUNING_SAMPLE_COUNT + 8U];
    for (unsigned i = 0U; i < TUNING_SAMPLE_COUNT; ++i)
        channels[i] = tuning_sample[i];
    if (active_command == TUNING_SPIN_SWEEP)
        for (unsigned i = 0; i < 8U; ++i) channels[TUNING_SAMPLE_COUNT+i] = tuning_spin_sample[i];
    vofa_send_channels(channels, TUNING_SAMPLE_COUNT + (active_command == TUNING_SPIN_SWEEP ? 8U : 0U));
}
