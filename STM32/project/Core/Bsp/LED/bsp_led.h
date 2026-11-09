/* SPDX-License-Identifier: GPL-3.0-or-later
 * 来源：RoboMaster 官方 2.flash_light/Src/main.c 的 bsp_led_toggle。
 */
#ifndef BSP_LED_H
#define BSP_LED_H

/**
 * @brief 同时翻转红、绿、蓝三个板载 LED。
 * @return 无。
 * @note 沿用教程 bsp_led_toggle 的三灯语义；当前为高亮低灭的普通 GPIO，
 *       只适配生成的引脚标签，不使用 RGB PWM 渐变或忙等延时。
 */
void bsp_led_toggle(void);

#endif
