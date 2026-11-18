#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "live_tuning.h"
#include "chassis.h"
#include "dm_motor.h"
#include "INS_task.h"
#include "task.h"
#include "tim.h"
#include "vofa.h"
#include "spin_observer.h"

uint32_t test_tick;
volatile dm_motor_t dm_motor[3];
volatile dm_param_reply_t dm_param_reply[3];
volatile float battery_voltage;
float chassis_speed[3], chassis_motor_target[3], chassis_motor_ff[3];
float chassis_tuning_kd[3], chassis_tuning_ff_gain, chassis_tuning_friction[3];
static unsigned enable_calls, disable_calls, stop_calls, move_calls, read_calls, config_calls;
static unsigned critical_depth;
static uint8_t enabled, stopped, motors_ready, imu_valid, enable_ok, move_ok, emit_target;
static uint32_t imu_age;
static float imu_yaw_deg, imu_rate;
static HAL_StatusTypeDef read_status;
static uint16_t read_id;
static uint8_t read_reg;
TIM_HandleTypeDef htim2;
static Chassis_Odometry_t odom;
static unsigned reset_calls, distance_calls, serial_calls;
static unsigned odom_reads, invalidate_read_number;
static uint8_t reset_ok, distance_ok;
static uint8_t velocity_mode;
static float last_velocity[3];
static float serial_sample[TUNING_SAMPLE_COUNT];

uint32_t test_timer_counter(TIM_HandleTypeDef *timer)
{
    assert(timer == &htim2);
    return test_tick * 1000U;
}
void vofa_send_channels(const float *values, uint8_t count)
{
    assert((count == TUNING_SAMPLE_COUNT || count == TUNING_SAMPLE_COUNT+8U) && !(tuning_sample_seq & 1U));
    ++serial_calls;
    memcpy(serial_sample, values, sizeof(serial_sample));
}
uint8_t Chassis_Get_Odometry(Chassis_Odometry_t *data)
{
    *data = odom;
    data->valid = odom.valid && enabled && motors_ready && ++odom_reads != invalidate_read_number;
    return data->valid;
}
uint8_t Chassis_Reset_Odometry(void)
{
    assert(enabled && stopped);
    ++reset_calls;
    if (reset_ok)
        odom = (Chassis_Odometry_t){0};
    return reset_ok;
}
uint8_t Chassis_Distance_Start(Chassis_Translation_Direction_t direction, float distance, float speed)
{
    assert(enabled && stopped && odom.valid);
    assert(direction == CHASSIS_TRANSLATION_FORWARD || direction == CHASSIS_TRANSLATION_BACKWARD);
    assert(distance >= 0.01f && distance <= 1.0f && speed >= 0.02f && speed <= 0.10f);
    ++distance_calls;
    if (distance_ok)
        stopped = 0U;
    return distance_ok;
}

void test_enter_critical(void) { ++critical_depth; }
void test_exit_critical(void) { assert(critical_depth > 0U); --critical_depth; }
uint8_t Chassis_Is_Enabled(void) { return enabled; }
uint8_t Chassis_Is_Stopped(void) { return enabled && stopped; }
uint8_t Chassis_Motors_Ready(void) { return enabled && motors_ready; }
uint8_t Chassis_Enable(void)
{
    ++enable_calls;
    if (enable_ok)
        enabled = 1U;
    return enable_ok;
}
void Chassis_Stop(void)
{
    ++stop_calls;
    velocity_mode = 0U;
    memset(last_velocity, 0, sizeof(last_velocity));
    memset(chassis_motor_target, 0, sizeof(chassis_motor_target));
}
void Chassis_Disable(void)
{
    ++disable_calls;
    enabled = 0U;
    memset(chassis_motor_target, 0, sizeof(chassis_motor_target));
}
uint8_t Chassis_Set_Velocity(float vx, float vy, float wz)
{
    assert(enabled);
    if (!stopped && !velocity_mode)
        return 0U;
    ++move_calls;
    if (move_ok)
    {
        stopped = 0U;
        velocity_mode = 1U;
        last_velocity[0] = vx;
        last_velocity[1] = vy;
        last_velocity[2] = wz;
        if (emit_target)
        {
            chassis_motor_target[0] = wz;
            chassis_motor_target[1] = -vx + wz;
            chassis_motor_target[2] = vx + wz;
        }
    }
    return move_ok;
}
uint8_t Chassis_Set_Velocity_Explicit_Yaw(float vx, float vy, float wz)
{
    return Chassis_Set_Velocity(vx, vy, wz);
}
uint8_t Chassis_Set_Test_Rotation(float wz)
{
    assert(isfinite(wz) && fabsf(wz) <= 4.0f);
    return Chassis_Set_Velocity(0, 0, wz);
}
uint8_t Chassis_Set_Tuning(const float kd[3], float ff_gain, const float friction[3])
{
    ++config_calls;
    if (enabled && !stopped)
        return 0U;
    memcpy(chassis_tuning_kd, kd, sizeof(chassis_tuning_kd));
    memcpy(chassis_tuning_friction, friction, sizeof(chassis_tuning_friction));
    chassis_tuning_ff_gain = ff_gain;
    return 1U;
}
uint8_t INS_Get_Data(INS_control_data_t *data)
{
    if (!imu_valid)
        return 0U;
    data->yaw_deg = imu_yaw_deg;
    data->world_z_rad_s = imu_rate;
    data->sample_time_ms = test_tick - imu_age;
    return 1U;
}
HAL_StatusTypeDef DM_Motor_ReadParam(uint16_t id, uint8_t reg)
{
    assert(!enabled);
    ++read_calls;
    read_id = id;
    read_reg = reg;
    return read_status;
}

static void reset_case(void)
{
    test_tick = 10U;
    enable_calls = disable_calls = stop_calls = move_calls = read_calls = config_calls = 0U;
    critical_depth = 0U;
    enabled = 0U;
    stopped = motors_ready = imu_valid = enable_ok = move_ok = emit_target = 1U;
    imu_age = 0U;
    imu_yaw_deg = 12.5f; imu_rate = .02f;
    read_status = HAL_OK;
    read_id = read_reg = 0U;
    battery_voltage = 16.0f;
    odom = (Chassis_Odometry_t){.valid = 1U};
    reset_calls = distance_calls = serial_calls = 0U;
    odom_reads = invalidate_read_number = 0U;
    reset_ok = distance_ok = 1U;
    velocity_mode = 0U;
    memset(last_velocity, 0, sizeof(last_velocity));
    memset(serial_sample, 0, sizeof(serial_sample));
    memset((void *)dm_motor, 0, sizeof(dm_motor));
    memset((void *)dm_param_reply, 0, sizeof(dm_param_reply));
    memset(chassis_motor_target, 0, sizeof(chassis_motor_target));
    Live_Tuning_Init();
}

static void loop(uint32_t elapsed_ms, uint8_t heartbeat)
{
    test_tick += elapsed_ms;
    if (heartbeat)
        ++tuning_heartbeat;
    Live_Tuning_Loop();
    assert(critical_depth == 0U);
}

static void submit(uint32_t command)
{
    tuning_command = command;
    ++tuning_request_seq;
    loop(0U, 1U);
    assert(tuning_ack_seq == tuning_request_seq);
}

static void disabled_replies(void)
{
    for (unsigned i = 0; i < 3U; ++i)
    {
        dm_motor[i].id = (uint8_t)(i + 1U);
        dm_motor[i].state = 0U;
        ++dm_motor[i].feedback_seq;
        dm_motor[i].feedback_time_ms = test_tick;
    }
    loop(5U, 0U);
}

static void set_move(float vx, float wz, float duration)
{
    tuning_args[0] = vx;
    tuning_args[1] = 0.0f;
    tuning_args[2] = wz;
    tuning_args[3] = duration;
}

static void start_move(float vx, float wz, uint32_t duration)
{
    set_move(vx, wz, (float)duration);
    submit(TUNING_MOVE);
    assert(tuning_phase == TUNING_RUNNING && move_calls == 1U);
    loop(5U, 1U); /* 首个非零目标开始计时。 */
}

static void test_init_and_config(void)
{
    reset_case();
    loop(1000U, 0U);
    assert(tuning_phase == TUNING_IDLE && !enable_calls && !disable_calls && !move_calls);
    for (unsigned i = 0; i < 3U; ++i)
    {
        tuning_args[i] = 0.2f;
        tuning_args[4U + i] = 0.01f;
    }
    tuning_args[3] = 0.5f;
    submit(TUNING_CONFIG);
    assert(tuning_phase == TUNING_DONE && tuning_error == 0U && config_calls == 1U);
    assert(chassis_tuning_ff_gain == 0.5f && chassis_tuning_friction[2] == 0.01f);
    tuning_args[0] = NAN;
    submit(TUNING_CONFIG);
    assert(!enable_calls && config_calls == 1U && tuning_error == 1U);
    disabled_replies();
    assert(tuning_phase == TUNING_FAULT);
}

static void test_move_rejection(void)
{
    const float invalid[][4] = {
        {0.251f, 0, 0, 800}, {0, 0, 0.501f, 800}, {0.25f, 0.01f, 0, 800},
        {0.25f, 0, 0.5f, 800}, {0, 0, 0, 800}, {NAN, 0, 0, 800},
        {0.25f, 0, 0, 1501}, {0.25f, 0, 0, 99}, {0.25f, 0, 0, 800.5f}
    };
    for (unsigned c = 0; c < sizeof(invalid) / sizeof(invalid[0]); ++c)
    {
        reset_case();
        for (unsigned i = 0; i < 4U; ++i)
            tuning_args[i] = invalid[c][i];
        submit(TUNING_MOVE);
        assert(tuning_error == 1U && !enable_calls && !move_calls);
        disabled_replies();
        assert(tuning_phase == TUNING_FAULT);
    }
}

static void test_motion_and_disable_ack(void)
{
    const float actions[][2] = {{0.25f, 0}, {-0.25f, 0}, {0, 0.5f}, {0, -0.5f}};
    for (unsigned a = 0; a < 4U; ++a)
    {
        reset_case();
        start_move(actions[a][0], actions[a][1], 100U);
        loop(99U, 1U);
        assert(tuning_phase == TUNING_RUNNING);
        loop(1U, 1U);
        assert(tuning_phase == TUNING_BRAKING && stop_calls == 1U && !disable_calls);
        stopped = 1U;
        loop(5U, 0U);
        assert(disable_calls == 1U && tuning_phase == TUNING_BRAKING);
        loop(100U, 0U);
        assert(tuning_phase == TUNING_BRAKING); /* 失能请求不等于实际回执。 */
        disabled_replies();
        assert(tuning_phase == TUNING_DONE && tuning_error == 0U && !enabled);
        loop(1000U, 1U);
        assert(enable_calls == 1U && move_calls == 1U); /* 不自动重启。 */
    }
}

static void test_disconnect_and_stop_deadline(void)
{
    reset_case();
    start_move(0.25f, 0.0f, 1500U);
    loop(299U, 0U);
    assert(tuning_phase == TUNING_RUNNING);
    loop(1U, 0U);
    assert(tuning_error == 5U && tuning_phase == TUNING_BRAKING);
    loop(400U, 1U);
    submit(TUNING_STOP); /* 重复STOP不能把500ms制动期限重新计时。 */
    loop(100U, 1U);
    assert(disable_calls == 1U && !enabled);
    disabled_replies();
    assert(tuning_phase == TUNING_FAULT && tuning_error == 5U);
    loop(1000U, 1U);
    assert(enable_calls == 1U && move_calls == 1U);

    reset_case();
    start_move(0.25f, 0.0f, 100U);
    loop(100U, 1U);
    loop(500U, 1U);
    assert(tuning_error == 8U && disable_calls == 1U);
    loop(500U, 1U);
    assert(tuning_phase == TUNING_FAULT);
}

static void test_prepare_failure_and_busy(void)
{
    reset_case();
    stopped = 0U;
    set_move(0.25f, 0, 800);
    submit(TUNING_MOVE);
    assert(tuning_phase == TUNING_PREPARING && !move_calls);
    for (unsigned i = 0; i < 10U; ++i)
        loop(100U, 1U);
    assert(tuning_error == 3U && tuning_phase == TUNING_BRAKING && !move_calls);

    reset_case();
    start_move(0.25f, 0, 800U);
    submit(TUNING_MOVE);
    assert(tuning_error == 2U && tuning_phase == TUNING_BRAKING && move_calls == 1U);

    reset_case();
    emit_target = 0U;
    set_move(0.25f, 0, 800);
    submit(TUNING_MOVE);
    loop(100U, 1U);
    assert(tuning_error == 4U && tuning_phase == TUNING_BRAKING);

    reset_case();
    imu_valid = 0U;
    set_move(0.25f, 0, 800);
    submit(TUNING_MOVE);
    loop(300U, 0U);
    assert(!enable_calls && tuning_error == 5U); /* prepare同样有断连保护。 */
}

static void test_motion_faults(void)
{
    for (unsigned i = 0; i < 3U; ++i)
    {
        reset_case();
        start_move(0.25f, 0, 800U);
        if (i == 0U) motors_ready = 0U;
        if (i == 1U) imu_age = 21U;
        if (i == 2U) enabled = 0U;
        loop(5U, 1U);
        assert(tuning_error == 4U && tuning_phase == TUNING_BRAKING);
        loop(1000U, 1U);
        assert(enable_calls == 1U);
    }
}

static void test_parameter_reads(void)
{
    reset_case();
    tuning_args[0] = 1.0f;
    submit(TUNING_READ_PARAM);
    assert(disable_calls == 1U && !read_calls);
    disabled_replies();
    for (unsigned i = 0; i < 3U; ++i)
    {
        loop(5U, 0U);
        assert(read_id == i + 1U && read_reg == 1U && read_calls == i + 1U);
        dm_param_reply[i].reg = 1U;
        dm_param_reply[i].value_bits = 0U;
        ++dm_param_reply[i].sequence;
        loop(5U, 0U);
    }
    assert(tuning_phase == TUNING_DONE && tuning_error == 0U && !enable_calls);

    reset_case();
    tuning_args[0] = 23.0f;
    submit(TUNING_READ_PARAM);
    disabled_replies();
    loop(5U, 0U);
    loop(200U, 0U);
    assert(tuning_phase == TUNING_FAULT && tuning_error == 6U);

    reset_case();
    tuning_args[0] = 1.0f;
    read_status = HAL_ERROR;
    submit(TUNING_READ_PARAM);
    disabled_replies();
    loop(5U, 0U);
    assert(tuning_phase == TUNING_FAULT && tuning_error == 7U);
}

static void test_snapshot_and_tick_wrap(void)
{
    reset_case();
    Live_Tuning_Snapshot();
    assert(tuning_sample_seq == 2U && tuning_sample[4] == 12.5f && tuning_sample[37] == 16.0f);
    loop(5U, 0U);
    Live_Tuning_Snapshot();
    assert(tuning_sample_seq == 2U);
    loop(95U, 0U);
    Live_Tuning_Snapshot();
    assert(tuning_sample_seq == 4U && !(tuning_sample_seq & 1U));

    reset_case();
    test_tick = UINT32_MAX - 50U;
    start_move(-0.25f, 0, 100U);
    loop(100U, 1U);
    assert(tuning_phase == TUNING_BRAKING && tuning_error == 0U);
}

static void test_read_with_missing_disable_reply(void)
{
    reset_case();
    tuning_args[0] = 1.0f;
    submit(TUNING_READ_PARAM);
    assert(disable_calls == 1U && !read_calls && !enabled);
    for (unsigned i = 0U; i < 2U; ++i)
    {
        dm_motor[i].id = (uint8_t)(i + 1U);
        dm_motor[i].state = 0U;
        ++dm_motor[i].feedback_seq;
        dm_motor[i].feedback_time_ms = test_tick;
    }
    loop(499U, 0U);
    assert(tuning_phase == TUNING_BRAKING && !read_calls);
    loop(1U, 0U);
    assert(tuning_phase == TUNING_PREPARING && !read_calls && tuning_error == 0U);
    for (unsigned i = 0U; i < 2U; ++i)
    {
        loop(5U, 0U);
        assert(read_id == i + 1U && read_reg == 1U);
        dm_param_reply[i].reg = 1U;
        dm_param_reply[i].value_bits = 0x3dcccccdU;
        ++dm_param_reply[i].sequence;
        loop(5U, 0U);
    }
    loop(5U, 0U);
    assert(read_id == 3U && read_calls == 3U);
    loop(200U, 0U);
    assert(tuning_phase == TUNING_FAULT && tuning_error == 6U);
    assert(dm_param_reply[0].sequence == 1U && dm_param_reply[1].sequence == 1U);
    assert(dm_motor[2].id == 0U && dm_motor[2].feedback_seq == 0U);
    assert(!enable_calls && !move_calls); /* 诊断不改变运动准入、不伪造缺失反馈。 */

    submit(TUNING_STOP);
    loop(500U, 0U);
    assert(tuning_phase == TUNING_FAULT && tuning_error == 8U);
    assert(!enable_calls && !move_calls); /* STOP仍要求全部新失能回复。 */
}

static void set_odometry(uint32_t command, float direction, float distance, float speed, float duration)
{
    for (unsigned i = 0U; i < 8U; ++i)
        tuning_args[i] = 0.0f;
    tuning_args[0] = direction;
    tuning_args[1] = distance;
    tuning_args[2] = speed;
    tuning_args[3] = duration;
    submit(command);
}

static void start_odometry(uint32_t command, float direction, float distance, float speed, float duration)
{
    set_odometry(command, direction, distance, speed, duration);
    assert(tuning_phase == TUNING_PREPARING && reset_calls == 1U && !distance_calls && !move_calls);
    loop(5U, 1U);
    assert(tuning_phase == TUNING_PREPARING && !distance_calls); /* 无新基线不能启动。 */
    odom.valid = 1U;
    odom.yaw_rad = 12.5f * 0.01745329251994329577f;
    odom.sample_time_us = test_tick * 1000U;
    loop(5U, 1U);
    assert(tuning_phase == TUNING_RUNNING && !move_calls);
}

static void set_progress(float distance)
{
    const float heading = 12.5f * 0.01745329251994329577f;
    odom.x_m = distance * cosf(heading);
    odom.y_m = distance * sinf(heading);
    odom.sample_time_us = test_tick * 1000U;
}

static void test_odometry_argument_limits(void)
{
    const float invalid[][4] = {
        {0, .10f, .05f, 1000}, {3, .10f, .05f, 1000}, {1.5f, .10f, .05f, 1000},
        {1, .009f, .05f, 1000}, {1, 1.001f, .05f, 1000}, {1, .10f, .019f, 1000},
        {1, .10f, .101f, 1000}, {1, .10f, .05f, 99}, {1, .10f, .05f, 20001},
        {1, .10f, .05f, 100.5f}, {1, NAN, .05f, 1000}
    };
    for (unsigned i = 0U; i < sizeof(invalid) / sizeof(invalid[0]); ++i)
    {
        reset_case();
        set_odometry(TUNING_ODOM_DISTANCE, invalid[i][0], invalid[i][1], invalid[i][2], invalid[i][3]);
        assert(tuning_error == 1U && !enable_calls && !reset_calls && !distance_calls);
    }
    reset_case();
    set_odometry(TUNING_ODOM_STATIC, 1, 0, 0, 100);
    assert(tuning_error == 1U && !enable_calls);
    reset_case();
    set_odometry(TUNING_ODOM_STATIC, 0, 0, 0, 1501);
    assert(tuning_error == 1U && !enable_calls);
    reset_case();
    tuning_args[3] = 100.0f;
    tuning_args[7] = NAN;
    submit(TUNING_ODOM_STATIC);
    assert(tuning_error == 1U && !enable_calls);
}

static void test_static_odometry_and_endpoint(void)
{
    reset_case();
    start_odometry(TUNING_ODOM_STATIC, 0, 0, 0, 150U);
    loop(100U, 1U);
    assert(tuning_phase == TUNING_RUNNING && !distance_calls && !move_calls); /* 零目标合法。 */
    set_progress(0.0005f);
    loop(50U, 1U);
    assert(tuning_phase == TUNING_BRAKING && tuning_error == 0U);
    loop(5U, 1U);
    disabled_replies();
    assert(tuning_phase == TUNING_DONE && !enabled);
    Live_Tuning_Snapshot();
    assert(tuning_sample[46] == 0.0f && tuning_sample[51] == 1.0f && tuning_sample[52] == 1.0f);
    assert(tuning_sample[53] == 155.0f && fabsf(tuning_sample[54] - .0005f) < .00001f);
    assert(serial_calls == 1U && serial_sample[51] == 1.0f && serial_sample[55] == 0.0f);
    const float end_x = tuning_sample[48];
    submit(TUNING_STOP);
    disabled_replies();
    Live_Tuning_Snapshot();
    assert(serial_calls == 2U && tuning_sample[48] == end_x && tuning_sample[51] == 1.0f);
    assert(tuning_sample[1] == TUNING_DONE); /* Finish 立即发布，不等 100 ms。 */
}

static void test_static_sampling_and_final_qualification(void)
{
    reset_case();
    start_odometry(TUNING_ODOM_STATIC, 0, 0, 0, 300U);
    stopped = 0U; /* 尚未累计三组新停车反馈，不代表发布过运动目标。 */
    odom.x_m = odom.y_m = .006;
    loop(100U, 1U);
    loop(100U, 1U);
    assert(tuning_phase == TUNING_RUNNING && tuning_error == 0U && !move_calls);
    loop(100U, 1U);
    assert(tuning_phase == TUNING_BRAKING && tuning_error == 0U && enabled);
    loop(100U, 1U);
    assert(tuning_phase == TUNING_BRAKING && enabled);
    stopped = 1U; /* 由底盘原有三新组确认；本模块不自行降低门限。 */
    loop(5U, 1U);
    assert(!enabled && tuning_phase == TUNING_BRAKING);
    disabled_replies();
    assert(tuning_phase == TUNING_DONE && tuning_error == 0U);
    Live_Tuning_Snapshot();
    assert(tuning_sample[51] == 1.0f && tuning_sample[52] == 1.0f && tuning_sample[53] == 405.0f);
    assert(!move_calls && !distance_calls);

    reset_case();
    start_odometry(TUNING_ODOM_STATIC, 0, 0, 0, 100U);
    stopped = 0U;
    loop(100U, 1U);
    loop(499U, 1U);
    assert(enabled && tuning_error == 0U);
    loop(1U, 1U);
    assert(!enabled && tuning_phase == TUNING_BRAKING && tuning_error == 8U);
    disabled_replies();
    assert(tuning_phase == TUNING_FAULT); /* 超过固定500ms，不能伪造静态成功。 */
}

static void test_static_sampling_faults(void)
{
    for (unsigned settling = 0U; settling < 2U; ++settling)
    for (unsigned fault = 0U; fault < 4U; ++fault)
    {
        reset_case();
        start_odometry(TUNING_ODOM_STATIC, 0, 0, 0, 100U);
        if (settling)
        {
            stopped = 0U;
            loop(100U, 1U);
            assert(tuning_phase == TUNING_BRAKING && tuning_error == 0U);
        }
        if (fault == 0U) odom.valid = 0U;
        if (fault == 1U) motors_ready = 0U;
        if (fault == 2U) imu_age = 21U;
        if (fault == 3U) odom.x_m = odom.y_m = .008; /* 单轴各不足10mm，欧氏距离已超。 */
        loop(5U, 1U);
        assert(tuning_phase == TUNING_BRAKING && tuning_error != 0U);
        loop(500U, 1U);
        disabled_replies();
        assert(tuning_phase == TUNING_FAULT && !move_calls && !distance_calls);
    }

    reset_case();
    start_odometry(TUNING_ODOM_STATIC, 0, 0, 0, 100U);
    stopped = 0U;
    loop(100U, 1U);
    loop(299U, 0U);
    assert(tuning_error == 0U && enabled);
    loop(1U, 0U);
    assert(tuning_error == 5U && enabled);
    loop(200U, 1U);
    assert(!enabled); /* 最终确认中断连仍300ms制动，且不重启500ms停车计时。 */
    disabled_replies();
    assert(tuning_phase == TUNING_FAULT && tuning_error == 5U);

    reset_case();
    start_odometry(TUNING_ODOM_STATIC, 0, 0, 0, 100U);
    stopped = 0U;
    loop(100U, 1U);
    loop(200U, 1U);
    loop(200U, 1U);
    submit(TUNING_STOP);
    loop(99U, 1U);
    assert(enabled && tuning_error == 0U);
    loop(1U, 1U);
    assert(!enabled && tuning_error == 8U); /* STOP也不能重开静态确认的500ms期限。 */

    reset_case();
    start_odometry(TUNING_ODOM_STATIC, 0, 0, 0, 100U);
    loop(100U, 1U);
    invalidate_read_number = odom_reads + 2U; /* 最终初判有效，捕获终点时失效。 */
    loop(5U, 1U);
    assert(tuning_error == TUNING_ERROR_ODOMETRY && tuning_phase == TUNING_BRAKING);
}

static void test_velocity_waits_for_valid_odometry(void)
{
    reset_case();
    odom.valid = 0U;
    set_move(.10f, 0, 100U);
    submit(TUNING_MOVE);
    loop(100U, 1U);
    assert(tuning_phase == TUNING_PREPARING && !move_calls && !reset_calls);
    odom.valid = 1U;
    odom.x_m = .25;
    loop(5U, 1U);
    assert(tuning_phase == TUNING_RUNNING && move_calls == 1U && !reset_calls && odom.x_m == .25);

    reset_case();
    odom.valid = 0U;
    set_move(.10f, 0, 100U);
    submit(TUNING_MOVE);
    for (unsigned i = 0U; i < 10U; ++i)
        loop(100U, 1U);
    assert(tuning_phase == TUNING_BRAKING && tuning_error == 3U && !move_calls && !reset_calls);
}

static void test_distance_odometry_completion(void)
{
    for (unsigned backwards = 0U; backwards < 2U; ++backwards)
    {
        reset_case();
        start_odometry(TUNING_ODOM_DISTANCE, backwards ? 2.0f : 1.0f, .10f, .05f, 5000U);
        assert(distance_calls == 1U);
        set_progress(backwards ? -.04f : .04f);
        loop(100U, 1U);
        assert(tuning_phase == TUNING_RUNNING);
        set_progress(backwards ? -.10f : .10f);
        loop(100U, 1U);
        assert(tuning_phase == TUNING_RUNNING); /* 到距离但没停稳不算完成。 */
        stopped = 1U;
        loop(5U, 1U);
        assert(tuning_phase == TUNING_BRAKING && tuning_error == 0U);
        loop(5U, 1U);
        disabled_replies();
        assert(tuning_phase == TUNING_DONE && !enabled);
        Live_Tuning_Snapshot();
        assert(tuning_sample[46] == 0.0f && tuning_sample[51] == 1.0f && tuning_sample[52] == 1.0f);
        assert(fabsf(tuning_sample[54] - .10f) < .00001f && tuning_sample[55] == .10f);
        loop(1000U, 1U);
        assert(distance_calls == 1U && reset_calls == 1U);
    }
}

static void test_odometry_faults_and_deadline(void)
{
    for (unsigned fault = 0U; fault < 7U; ++fault)
    {
        reset_case();
        start_odometry(TUNING_ODOM_DISTANCE, 1, .10f, .05f, 1000U);
        if (fault == 0U) stopped = 1U; /* 提前停稳，不得冒充到位。 */
        if (fault == 1U) odom.valid = 0U;
        if (fault == 2U) set_progress(.11f);
        if (fault == 3U) enabled = 0U;
        if (fault == 4U) imu_age = 21U;
        if (fault == 5U) motors_ready = 0U;
        if (fault == 6U) odom.x_m = NAN;
        loop(5U, 1U);
        assert(tuning_phase == TUNING_BRAKING && tuning_error != 0U);
        loop(500U, 1U);
        disabled_replies();
        assert(tuning_phase == TUNING_FAULT && distance_calls == 1U);
    }
    reset_case();
    start_odometry(TUNING_ODOM_DISTANCE, 1, .20f, .02f, 100U);
    loop(99U, 1U);
    assert(tuning_phase == TUNING_RUNNING);
    loop(1U, 1U);
    assert(tuning_phase == TUNING_BRAKING && tuning_error == TUNING_ERROR_ACTION_TIMEOUT);

    reset_case();
    start_odometry(TUNING_ODOM_DISTANCE, 1, .01f, .10f, 100U);
    set_progress(.01f);
    stopped = 1U;
    loop(100U, 1U);
    assert(tuning_error == TUNING_ERROR_ACTION_TIMEOUT); /* 过硬截止后到达也不能成功。 */

    reset_case();
    start_odometry(TUNING_ODOM_DISTANCE, 1, .01f, .10f, 1000U);
    set_progress(.01f);
    stopped = 1U;
    invalidate_read_number = odom_reads + 2U; /* 初判有效，但捕获终点时反馈失效。 */
    loop(5U, 1U);
    assert(tuning_error == TUNING_ERROR_ODOMETRY && tuning_phase == TUNING_BRAKING);

    for (unsigned static_mode = 0U; static_mode < 2U; ++static_mode)
    {
        reset_case();
        if (static_mode)
            start_odometry(TUNING_ODOM_STATIC, 0, 0, 0, 1500U);
        else
            start_odometry(TUNING_ODOM_DISTANCE, 1, .20f, .02f, 10000U);
        loop(300U, 0U);
        assert(tuning_phase == TUNING_BRAKING && tuning_error == 5U);
    }
    reset_case();
    reset_ok = 0U;
    set_odometry(TUNING_ODOM_STATIC, 0, 0, 0, 100U);
    assert(tuning_phase == TUNING_BRAKING && tuning_error == 3U && !distance_calls);
    reset_case();
    distance_ok = 0U;
    set_odometry(TUNING_ODOM_DISTANCE, 1, .01f, .10f, 100U);
    odom.valid = 1U;
    loop(5U, 1U);
    assert(tuning_phase == TUNING_BRAKING && tuning_error == 3U && distance_calls == 1U);

    reset_case();
    start_odometry(TUNING_ODOM_DISTANCE, 1, .01f, .10f, 1000U);
    set_progress(.01f);
    stopped = 1U;
    loop(5U, 1U);
    loop(5U, 1U);
    loop(500U, 1U); /* 缺失失能回执，即使到位也不成功。 */
    assert(tuning_phase == TUNING_FAULT && tuning_error == 8U);
}

static void test_odometry_time_wrap_and_age(void)
{
    reset_case();
    test_tick = UINT32_MAX - 40U;
    start_odometry(TUNING_ODOM_DISTANCE, 1, .10f, .05f, 100U);
    loop(100U, 1U);
    assert(tuning_error == TUNING_ERROR_ACTION_TIMEOUT);
    odom.sample_time_us = test_tick * 1000U - 2500U;
    Live_Tuning_Snapshot();
    assert(fabsf(tuning_sample[47] - 2.5f) < .00001f);
}

/* 长距离只扩大单次距离/总时限，不放宽原心跳或停车失能确认。 */
static void test_long_distance_limits_and_heartbeat(void)
{
    for (unsigned backwards = 0U; backwards < 2U; ++backwards)
    {
        reset_case();
        start_odometry(TUNING_ODOM_DISTANCE, backwards ? 2.0f : 1.0f, 1.0f, .10f, 20000U);
        assert(distance_calls == 1U && tuning_error == 0U);
        for (unsigned i = 0U; i < 16U; ++i)
        {
            set_progress((backwards ? -1.0f : 1.0f) * .005f * (i + 1U));
            loop(100U, 1U);
            assert(tuning_phase == TUNING_RUNNING); /* 超过原1500ms仍须可持续。 */
        }
        loop(299U, 0U);
        assert(tuning_phase == TUNING_RUNNING);
        loop(1U, 0U);
        assert(tuning_phase == TUNING_BRAKING && tuning_error == 5U);
        loop(500U, 1U);
        assert(!enabled && tuning_phase == TUNING_BRAKING);
        disabled_replies();
        assert(tuning_phase == TUNING_FAULT && tuning_error == 5U && distance_calls == 1U);
    }

    reset_case();
    start_odometry(TUNING_ODOM_DISTANCE, 1, 1.0f, .10f, 20000U);
    for (unsigned i = 0U; i < 199U; ++i)
    {
        set_progress(.004f * (i + 1U));
        loop(100U, 1U);
        assert(tuning_phase == TUNING_RUNNING);
    }
    loop(99U, 1U);
    assert(tuning_phase == TUNING_RUNNING);
    loop(1U, 1U);
    assert(tuning_phase == TUNING_BRAKING && tuning_error == TUNING_ERROR_ACTION_TIMEOUT);
    stopped = 1U;
    loop(5U, 1U);
    assert(!enabled && tuning_phase == TUNING_BRAKING); /* 仍要等三轮新失能回执。 */
    disabled_replies();
    assert(tuning_phase == TUNING_FAULT && tuning_error == TUNING_ERROR_ACTION_TIMEOUT);
    Live_Tuning_Snapshot();
    assert(tuning_sample[53] == 20000.0f && tuning_sample[55] == 1.0f);
}

static void start_return(float x, float y, float yaw, float duration)
{
    const float args[8] = {x, y, yaw, .20f, .50f, duration, 0, 0};
    for (unsigned i = 0U; i < 8U; ++i)
        tuning_args[i] = args[i];
    submit(TUNING_RETURN_HOLD);
}

static void test_return_arguments_and_fence(void)
{
    const float invalid[][2] = {{0,NAN},{2,3.15f},{3,.019f},{3,.201f},
        {4,.099f},{4,.501f},{5,99},{5,20001},{5,100.5f},{6,1},{7,1}};
    for (unsigned n = 0U; n < sizeof(invalid) / sizeof(invalid[0]); ++n)
    {
        reset_case();
        const float args[8] = {.5f, 0, 0, .2f, .5f, 20000, 0, 0};
        for (unsigned i = 0U; i < 8U; ++i)
            tuning_args[i] = args[i];
        tuning_args[(unsigned)invalid[n][0]] = invalid[n][1];
        submit(TUNING_RETURN_HOLD);
        assert(tuning_error == 1U && !move_calls && !enable_calls);
    }
    reset_case();
    start_return(1.501f, 0, 0, 20000);
    assert(tuning_error == 3U && !move_calls && !reset_calls);
    reset_case();
    start_return(1.0f, 0, 0, 20000);
    assert(tuning_phase == TUNING_RUNNING && !reset_calls);
    loop(5U, 1U);
    odom.x_m = 1.801;
    loop(5U, 1U);
    assert(tuning_error == TUNING_ERROR_ODOMETRY && tuning_phase == TUNING_BRAKING);
}

static void test_return_world_control_and_hold(void)
{
    reset_case();
    odom.x_m = .25;
    odom.y_m = .50;
    odom.yaw_rad = 1.5707963267948966f;
    start_return(.55f, .50f, odom.yaw_rad, 20000);
    assert(tuning_phase == TUNING_RUNNING && !reset_calls && odom.x_m == .25);
    loop(5U, 1U);
    assert(fabsf(last_velocity[0]) < .00001f && fabsf(last_velocity[1] + .20f) < .00001f);
    assert(last_velocity[2] == 0.0f && move_calls == 1U);
    loop(5U, 1U); /* VELOCITY内连续更新，无停轮/失能分脉冲。 */
    assert(move_calls == 2U && !disable_calls);
    odom.x_m = .545;
    loop(5U, 1U);
    assert(tuning_phase == TUNING_BRAKING && enabled && !disable_calls);
    assert(last_velocity[0] == 0 && last_velocity[1] == 0 && last_velocity[2] == 0);
    stopped = 1U;
    loop(5U, 1U);
    assert(tuning_phase == TUNING_DONE && tuning_error == 0U && enabled && !disable_calls);
    Live_Tuning_Snapshot();
    assert(tuning_sample[51] == 1 && tuning_sample[52] == 1);
    assert(fabsf(tuning_sample[54] - .295f) < .00001f && fabsf(tuning_sample[55] - .30f) < .00001f);
    const unsigned moves = move_calls;
    loop(5000U, 0U);
    assert(tuning_phase == TUNING_DONE && enabled && move_calls == moves && !disable_calls);
    start_return(.60f, .50f, odom.yaw_rad, 20000);
    assert(tuning_phase == TUNING_RUNNING && !reset_calls); /* 显式新目标才重新启动。 */
    loop(5U, 1U);
    assert(move_calls > moves);
    submit(TUNING_STOP);
    stopped = 1U;
    loop(5U, 1U);
    disabled_replies();
    assert(tuning_phase == TUNING_DONE && !enabled);
}

static void test_return_yaw_limits_and_near_speed(void)
{
    reset_case();
    odom.yaw_rad = 3.10f;
    start_return(.5f, 0, -3.0f, 20000);
    loop(5U, 1U);
    assert(last_velocity[2] > 0.0f && last_velocity[2] <= .5f); /* 跨-pi/pi取最短角差，使用足额角限速。 */
    assert(fabsf(hypotf(last_velocity[0],last_velocity[1]) - .20f) < .00001f);
    odom.x_m = .46;
    loop(5U, 1U);
    assert(hypotf(last_velocity[0],last_velocity[1]) > .02f && hypotf(last_velocity[0],last_velocity[1]) < .12f);

    reset_case();
    tuning_args[0] = .04f; tuning_args[3] = .02f; tuning_args[4] = .1f; tuning_args[5] = 20000;
    submit(TUNING_RETURN_HOLD);
    loop(5U, 1U);
    assert(fabsf(last_velocity[0] - .02f) < .00001f); /* 近处下限不得超过用户限速。 */
}

static void test_return_coarse_yaw_terminal_tolerance(void)
{
    const float accepted[] = {5.005f * .01745329251994329577f,
        TUNING_RETURN_YAW_TOLERANCE_RAD, -TUNING_RETURN_YAW_TOLERANCE_RAD};
    for (unsigned i = 0U; i < sizeof(accepted) / sizeof(accepted[0]); ++i)
    {
        reset_case();
        start_return(0,0,0,20000);
        loop(5U,1U);
        assert(tuning_phase == TUNING_BRAKING);
        odom.yaw_rad = accepted[i];
        loop(5U,1U);
        assert(tuning_phase == TUNING_DONE && tuning_error == 0U && enabled);
    }
    for (int sign = -1; sign <= 1; sign += 2)
    {
        reset_case();
        start_return(0,0,0,20000);
        loop(5U,1U);
        odom.yaw_rad = sign * (TUNING_RETURN_YAW_TOLERANCE_RAD + .00001f);
        loop(5U,1U);
        assert(tuning_phase == TUNING_BRAKING && tuning_error == TUNING_ERROR_ODOMETRY);
    }
    reset_case();
    odom.yaw_rad = 4.5f * .01745329251994329577f;
    start_return(0,0,0,20000);
    loop(5U,1U);
    assert(tuning_phase == TUNING_RUNNING && last_velocity[2] < 0.0f && last_velocity[2] >= -.5f);
    odom.yaw_rad = 3.9f * .01745329251994329577f;
    loop(5U,1U);
    assert(tuning_phase == TUNING_BRAKING); /* 内层仍为4deg，不能提前按10deg停车。 */
}

static void test_return_failures_and_terminal_race(void)
{
    for (unsigned fault = 0U; fault < 5U; ++fault)
    {
        reset_case();
        start_return(.5f, 0, 0, 20000);
        loop(5U, 1U);
        if (fault == 0U) odom.valid = 0U;
        if (fault == 1U) motors_ready = 0U;
        if (fault == 2U) imu_age = 21U;
        if (fault == 3U) move_ok = 0U;
        loop(fault == 4U ? 300U : 5U, fault != 4U);
        assert(tuning_error != 0U && tuning_phase == TUNING_BRAKING);
        loop(500U, 1U);
        disabled_replies();
        assert(tuning_phase == TUNING_FAULT && !enabled);
    }
    reset_case();
    start_return(.5f, 0, 0, 100);
    loop(100U, 1U);
    assert(tuning_error == TUNING_ERROR_ACTION_TIMEOUT);

    reset_case();
    start_return(0, 0, 0, 20000);
    loop(5U, 1U);
    stopped = 0U;
    loop(TUNING_RETURN_SETTLE_MS - 1U, 1U);
    assert(enabled && tuning_error == 0U);
    loop(1U, 1U);
    assert(!enabled && tuning_error == 8U);

    reset_case();
    start_return(0, 0, 0, 20000);
    loop(5U, 1U);
    odom.x_m = .021;
    loop(5U, 1U);
    assert(tuning_error == TUNING_ERROR_ODOMETRY); /* 停车时越出到点容差不算成功。 */

    reset_case();
    start_return(0, 0, 0, 20000);
    loop(5U, 1U);
    invalidate_read_number = odom_reads + 2U;
    loop(5U, 1U);
    assert(tuning_error == TUNING_ERROR_ODOMETRY); /* 终点重读失效不进入HOLD。 */

    reset_case();
    start_return(0, 0, 0, 20000);
    loop(5U, 1U);
    loop(5U, 1U);
    assert(tuning_phase == TUNING_DONE && enabled);
    imu_age = 21U;
    loop(5U, 0U);
    assert(tuning_error == 4U && tuning_phase == TUNING_BRAKING);
    loop(5U, 0U);
    disabled_replies();
    assert(tuning_phase == TUNING_FAULT && !enabled); /* HOLD仍监测真实故障。 */
}

static void test_return_feedback_tail_and_stop_override(void)
{
    reset_case();
    start_return(0, 0, 0, 20000);
    loop(5U, 1U);
    stopped = 0U;
    loop(700U, 1U); /* 超过旧500ms，仍以零目标等待真实三组停车资格。 */
    assert(enabled && tuning_phase == TUNING_BRAKING && !tuning_error && !disable_calls);
    stopped = 1U;
    loop(5U, 1U);
    assert(tuning_phase == TUNING_DONE && enabled && !disable_calls);
    Live_Tuning_Snapshot();
    assert(tuning_sample[51] == 1 && tuning_sample[52] == 1);

    for (unsigned disconnect = 0U; disconnect < 2U; ++disconnect)
    {
        reset_case();
        start_return(0, 0, 0, 20000);
        loop(5U, 1U);
        stopped = 0U;
        loop(700U, 1U);
        if (disconnect)
            loop(300U, 0U);
        else
            submit(TUNING_STOP);
        loop(5U, 1U);
        assert(!enabled); /* STOP或失联不继承更长的正常到点等待窗口。 */
        if (disconnect)
            assert(tuning_error == 5U);
    }
}

static void test_return_start_feedback_race(void)
{
    reset_case();
    start_return(.5f,0,0,20000);
    stopped = 0U; /* PREPARE刚通过，新反馈又暂时撤销三组停车资格。 */
    loop(5U,1U);
    assert(tuning_phase == TUNING_RUNNING && !tuning_error && !move_calls);
    loop(700U,1U);
    assert(enabled && !tuning_error && !move_calls && last_velocity[0] == 0);
    stopped = 1U;
    loop(5U,1U);
    assert(tuning_phase == TUNING_RUNNING && move_calls == 1U && last_velocity[0] > 0);
    move_ok = 0U;
    loop(5U,1U);
    assert(tuning_error == 4U); /* 已接受过速度后不重试运动。 */

    reset_case();
    start_return(.5f,0,0,20000);
    stopped = 0U;
    loop(TUNING_RETURN_START_MS,1U);
    assert(tuning_error == 4U && !move_calls); /* 窗口有界，不能无限等待。 */
}

static void visual_frame(float x, float y, uint32_t source)
{
    tuning_visual_center[0] = x;
    tuning_visual_center[1] = y;
    tuning_visual_time_ms = source;
    tuning_visual_valid = 1U;
    ++tuning_visual_seq;
    loop(20U, 1U);
}

static void start_visual(float x, float y)
{
    tuning_args[0] = x; tuning_args[1] = y; tuning_args[2] = 0.0f;
    tuning_args[3] = 0.2f; tuning_args[4] = 0.5f; tuning_args[5] = 20000;
    submit(TUNING_VISUAL_HOLD);
    assert(tuning_phase == TUNING_RUNNING && enabled);
}

static void test_visual_input_and_return_handoff(void)
{
    /* 图像矩形窗口与里程计圆形容差不能混用；现场20.27mm不能误报。 */
    for (unsigned outside = 0U; outside < 2U; ++outside)
    {
        reset_case();
        start_visual(.5f, 0.0f);
        odom.x_m = .5 - (outside ? .036 : .02027);
        visual_frame(5000, 7500, 100);
        stopped = 1U;
        loop(5U, 1U);
        if (outside)
            assert(tuning_error == TUNING_ERROR_ODOMETRY);
        else
            assert(tuning_phase == TUNING_DONE && enabled && !tuning_error && !disable_calls);
    }

    reset_case();
    start_visual(.6f,.1f);
    visual_frame(6000,1500,100);
    assert(last_velocity[0] > 0 && last_velocity[1] > 0 && !reset_calls);
    visual_frame(5000,7500,120); /* 一帧突跳到位，不能作为到位证据。 */
    assert(tuning_phase == TUNING_RUNNING && !stop_calls);
    for (unsigned i = 0; i < 13; ++i)
        visual_frame(6000,1500,120); /* 只变接收序号、源帧重复，不能续期。 */
    assert(tuning_error == 5U && stop_calls);

    reset_case();
    start_visual(.005f,.005f);
    visual_frame(5000,7500,100);
    assert(tuning_phase == TUNING_BRAKING && !disable_calls);
    odom.x_m = .005; odom.y_m = .005;
    stopped = 1U;
    loop(5U,1U);
    assert(tuning_phase == TUNING_DONE && enabled && !tuning_error && !disable_calls);
    start_return(0,0,0,20000);
    loop(5U,1U);
    assert(tuning_phase == TUNING_BRAKING && enabled && !reset_calls);
    stopped = 1U;
    loop(5U,1U);
    assert(tuning_phase == TUNING_DONE && enabled && !disable_calls);
}

static void start_spin(float turns)
{
    const float args[8]={turns,2,3,4,3,2,120000,0};
    for(unsigned i=0;i<8;++i)tuning_args[i]=args[i];
    submit(TUNING_SPIN_SWEEP);
}

static void complete_spin(unsigned turns)
{
    reset_case(); start_spin((float)turns);
    assert(tuning_phase==TUNING_RUNNING && enabled);
    double angle=0, speed=0;
    for(unsigned i=0;i<25000 && tuning_phase!=TUNING_DONE;++i)
    {
        speed+=fmax(-.008,fmin(.008,last_velocity[2]-speed));
        angle+=speed*.005;
        for(unsigned wheel=0;wheel<3;++wheel)
            dm_motor[wheel].position=-remainder(angle*.0982/.035,25.0);
        imu_yaw_deg = remainder(12.5 + angle*180.0/3.14159265358979323846, 360.0);
        imu_rate = speed;
        stopped=speed<.08;
        loop(5U,1U);
        assert(!tuning_error && !disable_calls);
    }
    assert(tuning_phase==TUNING_DONE && enabled && !disable_calls);
    Live_Tuning_Snapshot();
    assert(fabs(tuning_spin_sample[0]-(turns*SPIN_RADIANS_PER_TURN))<.03);
    assert(tuning_spin_sample[4]==4 && tuning_spin_sample[3]==0);
    assert(fabs(tuning_spin_sample[1]-(turns*SPIN_RADIANS_PER_TURN))<.03); /* 以实际YAW进度完成。 */

    assert(fabs(tuning_spin_sample[5]-turns*SPIN_RADIANS_PER_TURN)<.00002);
}

static void test_spin_sweep(void)
{
    complete_spin(30U);
    complete_spin(35U);
    reset_case(); start_spin(34); assert(tuning_error==1 && !enable_calls);
    reset_case(); start_spin(35); loop(5U,1U); loop(300U,0U);
    assert(tuning_error==5U && stop_calls); /* 高速实验仍受原心跳保护。 */
    reset_case(); start_spin(30); odom.x_m=.36; loop(5U,1U);
    assert(tuning_error==TUNING_ERROR_ACTION_TIMEOUT && stop_calls);
    reset_case(); start_spin(30); submit(TUNING_STOP); loop(5U,1U);
    assert(!enabled && !tuning_error);
    reset_case();
    tuning_args[0]=30; tuning_args[1]=2; tuning_args[2]=3; tuning_args[3]=4.01f;
    tuning_args[4]=3; tuning_args[5]=2; tuning_args[6]=120000;
    submit(TUNING_SPIN_SWEEP); assert(tuning_error==1 && !enable_calls);
}

int main(void)
{
    test_spin_sweep();
    test_visual_input_and_return_handoff();
    test_init_and_config();
    test_move_rejection();
    test_motion_and_disable_ack();
    test_disconnect_and_stop_deadline();
    test_prepare_failure_and_busy();
    test_motion_faults();
    test_parameter_reads();
    test_read_with_missing_disable_reply();
    test_snapshot_and_tick_wrap();
    test_odometry_argument_limits();
    test_static_odometry_and_endpoint();
    test_static_sampling_and_final_qualification();
    test_static_sampling_faults();
    test_velocity_waits_for_valid_odometry();
    test_distance_odometry_completion();
    test_odometry_faults_and_deadline();
    test_odometry_time_wrap_and_age();
    test_long_distance_limits_and_heartbeat();
    test_return_arguments_and_fence();
    test_return_world_control_and_hold();
    test_return_yaw_limits_and_near_speed();
    test_return_coarse_yaw_terminal_tolerance();
    test_return_failures_and_terminal_race();
    test_return_feedback_tail_and_stop_override();
    test_return_start_feedback_race();
    puts("PASS live tuning: bounded moves/odometry, fresh disable ACK, heartbeat/fault stop, 56-channel snapshots");
    return 0;
}
