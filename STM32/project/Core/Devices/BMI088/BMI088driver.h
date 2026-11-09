/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2019 RoboMaster.
 * 来源：13.spi_bmi088/component/devices/BMI088driver.h。
 * 2026-11-05：保留使用的教程接口；fp32/bool_t 使用等价的 float/uint8_t。
 */
#ifndef BMI088DRIVER_H
#define BMI088DRIVER_H

#include <stdint.h>

#define BMI088_TEMP_FACTOR 0.125f
#define BMI088_TEMP_OFFSET 23.0f
#define BMI088_WRITE_ACCEL_REG_NUM 6
#define BMI088_WRITE_GYRO_REG_NUM 6
#define BMI088_LONG_DELAY_TIME 80
#define BMI088_COM_WAIT_SENSOR_TIME 150
#define BMI088_ACCEL_3G_SEN 0.0008974358974f
#define BMI088_GYRO_2000_SEN 0.00106526443603169529841533860381f

/* 沿用教程数据结构；本阶段只更新 accel/temp/gyro，status/time 不使用。 */
typedef struct BMI088_REAL_DATA
{
    uint8_t status;
    float accel[3];
    float temp;
    float gyro[3];
    float time;
} bmi088_real_data_t;

enum
{
    BMI088_NO_ERROR = 0x00,
    BMI088_ACC_PWR_CTRL_ERROR = 0x01,
    BMI088_ACC_PWR_CONF_ERROR = 0x02,
    BMI088_ACC_CONF_ERROR = 0x03,
    BMI088_ACC_RANGE_ERROR = 0x05,
    BMI088_INT1_IO_CTRL_ERROR = 0x06,
    BMI088_INT_MAP_DATA_ERROR = 0x07,
    BMI088_GYRO_RANGE_ERROR = 0x08,
    BMI088_GYRO_BANDWIDTH_ERROR = 0x09,
    BMI088_GYRO_LPM1_ERROR = 0x0A,
    BMI088_GYRO_CTRL_ERROR = 0x0B,
    BMI088_GYRO_INT3_INT4_IO_CONF_ERROR = 0x0C,
    BMI088_GYRO_INT3_INT4_IO_MAP_ERROR = 0x0D,
    BMI088_NO_SENSOR = 0xFF
};

/** @brief 初始化 BMI088。 @return 两部分初始化错误码的按位或；0 表示成功。 */
uint8_t BMI088_init(void);
/** @brief 初始化加速度计。 @return 0 成功；非零为教程的 ID 或配置错误码。 */
uint8_t bmi088_accel_init(void);
/** @brief 初始化陀螺仪。 @return 0 成功；非零为教程的 ID 或配置错误码。 */
uint8_t bmi088_gyro_init(void);
/** @brief 按教程直接读取角速度、加速度和温度。
 * @param[out] gyro 三轴角速度，rad/s，数组长度 3。
 * @param[out] accel 三轴加速度，m/s²，数组长度 3。
 * @param[out] temperate 温度，°C。 @return 无。
 * @note 本次 SPI 失败或 accel/gyro ID 错误时 temperate 为 NAN，调用方不得使用该样本。
 */
void BMI088_read(float gyro[3], float accel[3], float *temperate);

#endif
