/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2016 DJI / RoboMaster.
 * 来源：20.standard_robot/components/controller/pid.h。
 * 2026-11-05：保留温控位置式 PID；fp32 使用等价 float。
 * 底盘 PID_Step 按用户授权参考旧车 Core/Control/PID/pid.c，复用本结构。
 */
#ifndef PID_H
#define PID_H

#include <stdint.h>

enum PID_MODE
{
    PID_POSITION = 0
};

typedef struct
{
    uint8_t mode;
    float Kp;
    float Ki;
    float Kd;
    float max_out;
    float max_iout;
    float set;
    float fdb;
    float out;
    float Pout;
    float Iout;
    float Dout;
    float Dbuf[3];
    float error[3];
} pid_type_def;

/**
 * @brief 沿官方教程初始化位置式 PID。
 * @param[out] pid PID 数据结构。
 * @param[in] mode 本模块仅支持 PID_POSITION。
 * @param[in] PID 依次为 Kp、Ki、Kd；PID_calc 使用每次调用的离散增益，
 *               PID_Step 使用连续时间增益，含义见其接口说明。
 * @param[in] max_out 输出绝对值上限，大于 0，单位由使用方定义。
 * @param[in] max_iout 积分输出绝对值上限，大于等于 0，与输出同单位。
 * @return 无。
 */
void PID_init(pid_type_def *pid, uint8_t mode, const float PID[3], float max_out, float max_iout);

/**
 * @brief 沿官方教程计算位置式 PID，保留积分和输出限幅。
 * @param[in,out] pid PID 数据结构。
 * @param[in] ref 测量值，与 set 使用相同单位。
 * @param[in] set 目标值，与 ref 使用相同单位。
 * @return 限幅后的控制输出；空指针返回 0。
 */
float PID_calc(pid_type_def *pid, float ref, float set);

/**
 * @brief 按旧车 PID_Step 计算连续时间积分及测量微分，供底盘航向控制。
 * @param[in,out] pid 由 PID_init 初始化的独立实例；参数及积分须为有限数。
 * @param[in] error 目标减测量误差，航向控制使用 rad，调用前处理角度跨圈。
 * @param[in] measurement_rate 测量值导数，航向控制使用 rad/s。
 * @param[in] dt 本次实际控制间隔，s，须为有限正数。
 * @return 对称限幅后的控制输出，航向控制为 rad/s；空指针返回 0。
 * @note 来源：旧车 Core/Control/PID/pid.c 的 PID_Step。
 *       保留 I+=Ki*error*dt、D=-Kd*measurement_rate、积分限幅和同向饱和抗积分。
 *       Kp 单位为输出/误差，Ki 为输出/(误差*s)，Kd 为输出*s/误差。
 *       调用方保证输入有效；本接口与 PID_calc 使用独立实例，不混用积分历史。
 */
float PID_Step(pid_type_def *pid, float error, float measurement_rate, float dt);

/**
 * @brief 沿官方教程清除误差历史和控制输出，保留 PID 参数。
 * @param[in,out] pid PID 数据结构。
 * @return 无。
 */
void PID_clear(pid_type_def *pid);

#endif
