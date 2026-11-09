/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2019 RoboMaster.
 * 来源：官方 5.buzzer/bsp/boards/bsp_buzzer.c。
 */
#ifndef BSP_BUZZER_H
#define BSP_BUZZER_H

#include <stdint.h>

/**
 * @brief 按教程设置蜂鸣器 TIM4_CH3 的分频和脉宽。
 * @param[in] psc 预分频寄存器值，范围 0~65535，实际分频为 psc+1。
 * @param[in] pwm 比较寄存器值，单位为计数；当前 ARR=249，范围 0~250。
 * @return 无。
 * @note 调用者先启动 TIM4_CH3 PWM。当前 84 MHz 定时器时钟下，
 *       psc=83、pwm=125 对应 4 kHz、50% 占空比；本函数不启动 PWM。
 */
void buzzer_on(uint16_t psc, uint16_t pwm);

/** @brief 将蜂鸣器 TIM4_CH3 比较值清零以停止发声。 @return 无。 */
void buzzer_off(void);

#endif
