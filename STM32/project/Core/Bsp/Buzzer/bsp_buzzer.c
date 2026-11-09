/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2019 RoboMaster.
 * 来源：https://github.com/RoboMaster/Development-Board-C-Examples/blob/master/5.buzzer/bsp/boards/bsp_buzzer.c
 * 保留 buzzer_on/off 原签名，只将旧 HAL 宏名适配为当前大写宏。
 */
#include "bsp_buzzer.h"
#include "tim.h"

/**
 * @brief 设置蜂鸣器分频与脉宽；PWM 由任务预先启动。
 * @param[in] psc 预分频寄存器值，0~65535，实际除数为 psc+1。
 * @param[in] pwm 比较计数，当前范围 0~250；83/125 参数组合为 4 kHz、50%。
 * @return 无。
 * @note 来源：教程第 5 章 bsp_buzzer.c 的同名函数，仅适配 HAL 宏名。
 */
void buzzer_on(uint16_t psc, uint16_t pwm)
{
    __HAL_TIM_SET_PRESCALER(&htim4, psc);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, pwm);
}

/** @brief 将比较计数清零，关闭蜂鸣器输出。
 * @return 无。
 * @note 来源：教程第 5 章 bsp_buzzer.c 的同名函数。
 */
void buzzer_off(void)
{
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 0U);
}
