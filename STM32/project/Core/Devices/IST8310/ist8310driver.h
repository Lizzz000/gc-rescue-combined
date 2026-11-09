/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2019 DJI / RoboMaster.
 * 来源：11.ist8310/bsp/boards/ist8310driver.h；2026-11-05 适配。
 */
#ifndef IST8310DRIVER_H
#define IST8310DRIVER_H

#include <stdint.h>

#define IST8310_DATA_READY_BIT 2
#define IST8310_NO_ERROR 0x00
#define IST8310_NO_SENSOR 0x40

typedef struct ist8310_real_data_t
{
    uint8_t status; /**< 教程状态字，bit2 表示本次有新数据。 */
    float mag[3];   /**< 三轴磁场，µT，芯片坐标。 */
} ist8310_real_data_t;

/** @brief 初始化 IST8310，沿用教程配置和寄存器回读。
 * @return 0 成功；1～4 为配置错误；0x40 为设备 ID 错误。
 */
uint8_t ist8310_init(void);
/** @brief 解码从 STAT1 起的七字节。
 * @param[in] status_buf 从 0x02～0x08 读取的 7 字节。
 * @param[in,out] ist8310_real_data 磁场 µT 及教程状态位。
 * @return 无；无新数据时清除状态 bit2，保留磁场值。
 */
void ist8310_read_over(uint8_t *status_buf, ist8310_real_data_t *ist8310_real_data);

#endif
