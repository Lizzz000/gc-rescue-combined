/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2019 DJI / RoboMaster.
 * 来源：https://github.com/RoboMaster/Development-Board-C-Examples/blob/master/18.ins_task/bsp/boards/bsp_imu_pwm.c
 * 2026-11-05：使用 CubeMX 的 tim.h 与当前 HAL 的大写比较寄存器宏。
 */
#include "bsp_imu_pwm.h"
#include "tim.h"

/** @brief 写入加热 PWM；参数范围和返回值见 bsp_imu_pwm.h。 */
void imu_pwm_set(uint16_t pwm)
{
    __HAL_TIM_SET_COMPARE(&htim10, TIM_CHANNEL_1, pwm);
}
