/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2019 DJI / RoboMaster.
 * 来源：https://github.com/RoboMaster/Development-Board-C-Examples/blob/master/20.standard_robot/components/controller/pid.c
 * 2026-11-05：裁剪未使用的增量式分支，保留 PID_init/PID_calc/PID_clear
 * 的位置式公式与限幅顺序；fp32 使用等价 float。
 * PID_Step 参考旧车 Core/Control/PID/pid.c，保留连续时间积分和测量微分。
 */
#include "pid.h"
#include <stddef.h>
#include <math.h>

static uint8_t Parameters_Valid(const pid_type_def *pid)
{
    return pid != NULL && pid->mode == PID_POSITION && isfinite(pid->Kp) &&
        isfinite(pid->Ki) && isfinite(pid->Kd) && isfinite(pid->max_out) && pid->max_out > 0.0f &&
        isfinite(pid->min_out) && pid->min_out <= 0.0f && isfinite(pid->max_iout) &&
        pid->max_iout >= 0.0f && isfinite(pid->Iout);
}

static float Reject(pid_type_def *pid)
{
    PID_clear(pid);
    return 0.0f;
}

/* 两个入口复用限幅和抗饱和，保持离散/连续积分的含义各自独立。 */
static float Finish(pid_type_def *pid, float proportional, float derivative, float candidate)
{
    const float pd = proportional + derivative;
    if (!isfinite(proportional) || !isfinite(derivative) || !isfinite(pd) || !isfinite(candidate))
        return Reject(pid);
    const float old = pid->Iout;
    float integral = fmaxf(-pid->max_iout, fminf(pid->max_iout, candidate));
    if (!isfinite(pd + integral))
        return Reject(pid);
    if (pd + integral > pid->max_out && integral > old)
        integral = fmaxf(old, pid->max_out - pd);
    else if (pd + integral < pid->min_out && integral < old)
        integral = fminf(old, pid->min_out - pd);
    pid->Pout = proportional;
    pid->Dout = derivative;
    pid->Iout = integral;
    pid->out = fmaxf(pid->min_out, fminf(pid->max_out, pd + integral));
    return pid->out;
}

/** @brief 初始化位置式 PID；参数、单位及返回值见 pid.h。 */
void PID_init(pid_type_def *pid, uint8_t mode, const float PID[3], float max_out, float max_iout)
{
    if (pid == NULL)
        return;
    *pid = (pid_type_def){0};
    if (PID == NULL)
        return;
    pid->mode = mode;
    pid->Kp = PID[0]; pid->Ki = PID[1]; pid->Kd = PID[2];
    pid->max_out = max_out; pid->min_out = -max_out; pid->max_iout = max_iout;
    if (!Parameters_Valid(pid))
        *pid = (pid_type_def){0};
}

/** @brief 计算位置式 PID；参数、单位及返回值见 pid.h。 */
float PID_calc(pid_type_def *pid, float ref, float set)
{
    if (!Parameters_Valid(pid) || !isfinite(ref) || !isfinite(set) ||
        !isfinite(set - ref) || !isfinite(pid->error[0]))
        return Reject(pid);
    pid->error[1] = pid->error[0];
    pid->set = set;
    pid->fdb = ref;
    pid->error[0] = set - ref;
    return Finish(pid, pid->Kp * pid->error[0], pid->Kd * (pid->error[0] - pid->error[1]),
                  pid->Iout + pid->Ki * pid->error[0]);
}

/** @brief 连续时间航向 PID；非法输入返回零并清除历史，避免污染后续控制。 */
float PID_Step(pid_type_def *pid, float error, float measurement_rate, float dt)
{
    if (!Parameters_Valid(pid) || !isfinite(error) || !isfinite(measurement_rate) || !isfinite(dt) || dt <= 0.0f)
        return Reject(pid);
    pid->error[1] = pid->error[0];
    pid->error[0] = error;
    return Finish(pid, pid->Kp * error, -pid->Kd * measurement_rate,
                  pid->Iout + pid->Ki * error * dt);
}

/** @brief 清除误差历史和输出；参数及返回值见 pid.h。 */
void PID_clear(pid_type_def *pid)
{
    if (pid == NULL)
    {
        return;
    }
    pid->error[0] = pid->error[1] = 0.0f;
    pid->out = pid->Pout = pid->Iout = pid->Dout = 0.0f;
    pid->fdb = pid->set = 0.0f;
}
