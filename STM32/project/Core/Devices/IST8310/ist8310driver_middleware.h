/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2019 DJI / RoboMaster. 来源：教程第 11 章，2026-11-05 适配。
 */
#ifndef IST8310DRIVER_MIDDLEWARE_H
#define IST8310DRIVER_MIDDLEWARE_H

#include <stdint.h>

#define IST8310_IIC_ADDRESS 0x0E

/** @brief 教程 GPIO 初始化接口，本工程由 CubeMX 完成。 @return 无。 */
void ist8310_GPIO_init(void);
/** @brief 教程通信初始化接口，本工程由 CubeMX 完成。 @return 无。 */
void ist8310_com_init(void);
/** @brief 读取单寄存器。 @param[in] reg 地址。 @return 寄存器值。 */
uint8_t ist8310_IIC_read_single_reg(uint8_t reg);
/** @brief 写单寄存器。 @param[in] reg 地址。 @param[in] data 写入值。 @return 无。 */
void ist8310_IIC_write_single_reg(uint8_t reg, uint8_t data);
/** @brief 连续读寄存器。
 * @param[in] reg 起始地址。 @param[out] buf 至少 len 字节。
 * @param[in] len 字节数。 @return 本次HAL读取成功为1，失败为0；失败缓冲不能解码。
 */
uint8_t ist8310_IIC_read_muli_reg(uint8_t reg, uint8_t *buf, uint8_t len);
/** @brief RTOS 毫秒延时。 @param[in] ms 毫秒。 @return 无。 */
void ist8310_delay_ms(uint16_t ms);
/** @brief TIM2 微秒延时。 @param[in] us 0～65535 µs。 @return 无。 */
void ist8310_delay_us(uint16_t us);
/** @brief 拉低复位引脚。 @return 无。 */
void ist8310_RST_L(void);
/** @brief 释放复位引脚。 @return 无。 */
void ist8310_RST_H(void);

#endif
