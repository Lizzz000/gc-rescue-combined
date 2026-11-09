/* SPDX-License-Identifier: GPL-3.0-or-later
 * 来源：https://github.com/RoboMaster/Development-Board-C-Examples/blob/master/2.flash_light/Src/main.c
 * 从教程拆出 bsp_led_toggle，适配 LED_RED/GREEN/BLUE 引脚标签。
 * 原例程文件保留的声明：
 * Copyright (c) 2019 STMicroelectronics. All rights reserved.
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 * opensource.org/licenses/BSD-3-Clause
 */
#include "bsp_led.h"
#include "main.h"

/** @brief 翻转三色 LED 的普通 GPIO 输出。
 * @return 无。
 * @note 来源：教程第 2 章 main.c 的 bsp_led_toggle，仅适配引脚标签。
 */
void bsp_led_toggle(void)
{
    HAL_GPIO_TogglePin(LED_RED_GPIO_Port, LED_RED_Pin);
    HAL_GPIO_TogglePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin);
    HAL_GPIO_TogglePin(LED_BLUE_GPIO_Port, LED_BLUE_Pin);
}
