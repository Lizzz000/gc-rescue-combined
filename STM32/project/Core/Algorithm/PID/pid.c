/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2019 DJI / RoboMaster.
 * 来源：https://github.com/RoboMaster/Development-Board-C-Examples/blob/master/20.standard_robot/components/controller/pid.c
 * 2026-11-05：裁剪未使用的增量式分支，保留 PID_init/PID_calc/PID_clear
 * 的位置式公式与 LimitMax 写法；fp32 使用等价 float。
 * PID_Step 参考旧车 Core/Control/PID/pid.c，保留连续时间积分和测量微分。
 */
#include "pid.h"
#include <stddef.h>
#include <math.h>

#define LimitMax(input, max) \
    {                       \
        if (input > max)    \
        {                   \
            input = max;    \
        }                   \
        else if (input < -max) \
        {                   \
            input = -max;   \
        }                   \
    }

/** @brief 初始化位置式 PID；参数、单位及返回值见 pid.h。 */
void PID_init(pid_type_def *pid, uint8_t mode, const float PID[3], float max_out, float max_iout)
{
    if (pid == NULL || PID == NULL)
    {
        return;
    }
    pid->mode = mode;
    pid->Kp = PID[0];
    pid->Ki = PID[1];
    pid->Kd = PID[2];
    pid->max_out = max_out;
    pid->max_iout = max_iout;
    pid->Dbuf[0] = pid->Dbuf[1] = pid->Dbuf[2] = 0.0f;
    pid->error[0] = pid->error[1] = pid->error[2] = pid->Pout = pid->Iout = pid->Dout = pid->out = 0.0f;
}

/** @brief 计算位置式 PID；参数、单位及返回值见 pid.h。 */
float PID_calc(pid_type_def *pid, float ref, float set)
{
    if (pid == NULL)
    {
        return 0.0f;
    }
    pid->error[2] = pid->error[1];
    pid->error[1] = pid->error[0];
    pid->set = set;
    pid->fdb = ref;
    pid->error[0] = set - ref;
    if (pid->mode == PID_POSITION)
    {
        pid->Pout = pid->Kp * pid->error[0];
        pid->Iout += pid->Ki * pid->error[0];
        pid->Dbuf[2] = pid->Dbuf[1];
        pid->Dbuf[1] = pid->Dbuf[0];
        pid->Dbuf[0] = (pid->error[0] - pid->error[1]);
        pid->Dout = pid->Kd * pid->Dbuf[0];
        LimitMax(pid->Iout, pid->max_iout);
        pid->out = pid->Pout + pid->Iout + pid->Dout;
        LimitMax(pid->out, pid->max_out);
    }
    return pid->out;
}

/** @brief 计算旧车连续时间航向 PID；参数、单位及返回值见 pid.h。 */
float PID_Step(pid_type_def *pid, float error, float measurement_rate, float dt)
{
    if (pid == NULL)
        return 0.0f;

    /* 保留旧 PID_Step 的 double 中间乘加及先对积分限幅的顺序。 */
    const double pd = (double)pid->Kp * error - (double)pid->Kd * measurement_rate;
    const double old_integral = pid->Iout;
    double integral = old_integral + (double)pid->Ki * error * dt;
    LimitMax(integral, pid->max_iout);
    if (pd + integral > pid->max_out && integral > old_integral)
        integral = fmax(old_integral, pid->max_out - pd);
    else if (pd + integral < -pid->max_out && integral < old_integral)
        integral = fmin(old_integral, -pid->max_out - pd);

    pid->error[0] = error;
    pid->Pout = pid->Kp * error;
    pid->Iout = (float)integral;
    pid->Dout = -pid->Kd * measurement_rate;
    double output = pd + pid->Iout;
    LimitMax(output, pid->max_out);
    pid->out = (float)output;
    return pid->out;
}

/** @brief 清除误差历史和输出；参数及返回值见 pid.h。 */
void PID_clear(pid_type_def *pid)
{
    if (pid == NULL)
    {
        return;
    }
    pid->error[0] = pid->error[1] = pid->error[2] = 0.0f;
    pid->Dbuf[0] = pid->Dbuf[1] = pid->Dbuf[2] = 0.0f;
    pid->out = pid->Pout = pid->Iout = pid->Dout = 0.0f;
    pid->fdb = pid->set = 0.0f;
}
