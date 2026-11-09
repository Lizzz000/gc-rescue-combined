/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2019 RoboMaster. 来源：教程第 13 章，2026-11-05 适配。
 */
#ifndef BMI088MIDDLEWARE_H
#define BMI088MIDDLEWARE_H

#include <stdint.h>

/** 本次初始化/读取的 SPI 错误；入口清零，HAL 失败置位，供温控拒绝无效温度。 */
extern uint8_t BMI088_spi_error;

/** @brief 教程 GPIO 初始化接口；本工程由 CubeMX 完成。 @return 无。 */
void BMI088_GPIO_init(void);
/** @brief 教程通信初始化接口；本工程由 CubeMX 完成。 @return 无。 */
void BMI088_com_init(void);
/** @brief RTOS 毫秒延时。 @param[in] ms 毫秒。 @return 无。 */
void BMI088_delay_ms(uint16_t ms);
/** @brief TIM2 微秒延时。 @param[in] us 0～65535 µs。 @return 无。 */
void BMI088_delay_us(uint16_t us);
/** @brief 拉低加速度计片选。 @return 无。 */
void BMI088_ACCEL_NS_L(void);
/** @brief 拉高加速度计片选。 @return 无。 */
void BMI088_ACCEL_NS_H(void);
/** @brief 拉低陀螺仪片选。 @return 无。 */
void BMI088_GYRO_NS_L(void);
/** @brief 拉高陀螺仪片选。 @return 无。 */
void BMI088_GYRO_NS_H(void);
/** @brief SPI1 交换一个字节，失败后本次事务剩余字节快速返回。
 * @param[in] txdata 发送值。 @return 接收值；失败返回 0 并置 BMI088_spi_error。
 */
uint8_t BMI088_read_write_byte(uint8_t txdata);

#endif
