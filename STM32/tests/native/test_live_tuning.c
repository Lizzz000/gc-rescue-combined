#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "live_tuning.h"
#include "chassis.h"
#include "dm_motor.h"
#include "INS_task.h"
#include "task.h"

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
static HAL_StatusTypeDef read_status;
static uint16_t read_id;
static uint8_t read_reg;

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
    assert(enabled && stopped && vy == 0.0f);
    ++move_calls;
    if (move_ok)
    {
        stopped = 0U;
        if (emit_target)
        {
            chassis_motor_target[0] = wz;
            chassis_motor_target[1] = -vx + wz;
            chassis_motor_target[2] = vx + wz;
        }
    }
    return move_ok;
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
    data->yaw_deg = 12.5f;
    data->world_z_rad_s = 0.02f;
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
    read_status = HAL_OK;
    read_id = read_reg = 0U;
    battery_voltage = 16.0f;
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

int main(void)
{
    test_init_and_config();
    test_move_rejection();
    test_motion_and_disable_ack();
    test_disconnect_and_stop_deadline();
    test_prepare_failure_and_busy();
    test_motion_faults();
    test_parameter_reads();
    test_read_with_missing_disable_reply();
    test_snapshot_and_tick_wrap();
    puts("PASS live tuning: bounded moves, fresh disable ACK, heartbeat/fault stop, read-only params, snapshots");
    return 0;
}
