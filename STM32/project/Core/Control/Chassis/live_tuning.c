/* 现场 SWD 调参入口：有限时长运动、主机断连制动、显式使能与失能回执。
 * 不写电机内部参数，不保存 Flash，不直接改底盘状态；正常构建不启用本模块。
 */
#include "live_tuning.h"
#include "chassis.h"
#include "dm_motor.h"
#include "INS_task.h"
#include "FreeRTOS.h"
#include "task.h"
#include <math.h>

volatile uint32_t tuning_request_seq, tuning_command, tuning_heartbeat;
volatile uint32_t tuning_ack_seq, tuning_phase, tuning_error, tuning_sample_seq;
volatile float tuning_args[8], tuning_sample[40];
extern volatile float battery_voltage;

typedef enum { INACTIVE, PREPARE, RUN, BRAKE, WAIT_DISABLED, READ_PARAMETER } Work_State;
static Work_State work;
static uint32_t last_heartbeat, heartbeat_ms, stage_ms, motion_ms, motion_duration_ms;
static uint32_t motion_elapsed_ms, sample_ms;
static uint32_t disable_sequence[3], parameter_sequence;
static uint8_t enable_requested, motion_started, read_after_disable;
static uint8_t parameter_reg, parameter_motor, parameter_waiting;
static uint8_t sample_published;
static float requested_vx, requested_wz;

static void Finish(void)
{
    if (motion_started)
        motion_elapsed_ms = (uint32_t)xTaskGetTickCount() - motion_ms;
    work = INACTIVE;
    tuning_phase = tuning_error ? TUNING_FAULT : TUNING_DONE;
}

static uint8_t Fresh_IMU(uint32_t now_ms, INS_control_data_t *imu)
{
    return INS_Get_Data(imu) && isfinite(imu->yaw_deg) && isfinite(imu->world_z_rad_s) &&
        (uint32_t)(now_ms - imu->sample_time_ms) <= 20U;
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
    Chassis_Stop();
    work = BRAKE;
    tuning_phase = TUNING_BRAKING;
    stage_ms = now_ms;
    if (!Chassis_Is_Enabled())
        Request_Disable(now_ms);
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

static void Accept_Command(uint32_t command, const float args[8], uint32_t now_ms)
{
    if (command == TUNING_STOP)
    {
        if (work == INACTIVE)
            tuning_error = 0U;
        Begin_Brake(now_ms, 0U);
        return;
    }
    if (work != INACTIVE)
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
    else if (command == TUNING_MOVE && Valid_Move(args))
    {
        requested_vx = args[0];
        requested_wz = args[2];
        motion_duration_ms = (uint32_t)args[3];
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
    tuning_request_seq = tuning_command = tuning_heartbeat = tuning_ack_seq = 0U;
    tuning_phase = TUNING_IDLE;
    tuning_error = tuning_sample_seq = 0U;
    work = INACTIVE;
    enable_requested = motion_started = read_after_disable = 0U;
    parameter_motor = parameter_waiting = 0U;
    motion_ms = motion_duration_ms = motion_elapsed_ms = sample_ms = 0U;
    sample_published = 0U;
    for (unsigned i = 0U; i < 8U; ++i)
        tuning_args[i] = 0.0f;
    for (unsigned i = 0U; i < 40U; ++i)
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
    if ((work == PREPARE || work == RUN) && (uint32_t)(now_ms - heartbeat_ms) >= 300U)
        Begin_Brake(now_ms, 5U);

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
                if (!Chassis_Set_Velocity(requested_vx, 0.0f, requested_wz))
                    Begin_Brake(now_ms, 3U);
                else
                {
                    work = RUN;
                    tuning_phase = TUNING_RUNNING;
                    stage_ms = now_ms;
                }
            }
        }
    }
    else if (work == RUN)
    {
        INS_control_data_t imu;
        if (!Chassis_Is_Enabled() || !Chassis_Motors_Ready() || !Fresh_IMU(now_ms, &imu))
            Begin_Brake(now_ms, 4U);
        else
        {
            if (!motion_started && (chassis_motor_target[0] != 0.0f ||
                chassis_motor_target[1] != 0.0f || chassis_motor_target[2] != 0.0f))
            {
                motion_started = 1U;
                motion_ms = now_ms;
            }
            if (motion_started && (uint32_t)(now_ms - motion_ms) >= motion_duration_ms)
                Begin_Brake(now_ms, 0U);
            else if (!motion_started && (uint32_t)(now_ms - stage_ms) >= 100U)
                Begin_Brake(now_ms, 4U);
        }
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
    if (sample_published && (uint32_t)(now_ms - sample_ms) < 100U)
        return;
    sample_published = 1U;
    sample_ms = now_ms;
    INS_control_data_t imu;
    const uint8_t imu_valid = INS_Get_Data(&imu);
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
    tuning_sample[39] = motion_started ? (float)(work == INACTIVE ? motion_elapsed_ms :
                                                (uint32_t)(now_ms - motion_ms)) : 0.0f;
    ++tuning_sample_seq;
}
