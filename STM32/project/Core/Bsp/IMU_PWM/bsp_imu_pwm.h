/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2019 DJI / RoboMaster.
 * 来源：18.ins_task/bsp/boards/bsp_imu_pwm.h。
 */
#ifndef BSP_IMU_PWM_H
#define BSP_IMU_PWM_H

#include <stdint.h>

/**
 * @brief 沿教程 imu_pwm_set 写入 TIM10_CH1 加热 PWM 比较值。
 * @param[in] pwm 比较计数值，范围 0～4999；0 关闭加热。
 * @return 无。
 * @note 引脚、周期及极性由 CubeMX 配置；INS 任务启动一次 PWM。
 */
void imu_pwm_set(uint16_t pwm);

#endif
