/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2019 RoboMaster.
 * 来源：https://github.com/RoboMaster/Development-Board-C-Examples/blob/master/13.spi_bmi088/component/devices/BMI088driver.c
 * 2026-11-05：采用教程 SPI 分支；fp32/bool_t 改用 float/uint8_t，补中文注释。
 * 温控适配：采样增加 accel ID 验证，HAL 失败或任一芯片 ID 错误返回无效温度。
 */
#include "BMI088driver.h"
#include "BMI088reg.h"
#include "BMI088Middleware.h"
#include <math.h>

float BMI088_ACCEL_SEN = BMI088_ACCEL_3G_SEN;
float BMI088_GYRO_SEN = BMI088_GYRO_2000_SEN;

#define BMI088_accel_write_single_reg(reg, data) \
    { BMI088_ACCEL_NS_L(); BMI088_write_single_reg((reg), (data)); BMI088_ACCEL_NS_H(); }
#define BMI088_accel_read_single_reg(reg, data) \
    { BMI088_ACCEL_NS_L(); BMI088_read_write_byte((reg) | 0x80); \
      BMI088_read_write_byte(0x55); (data) = BMI088_read_write_byte(0x55); BMI088_ACCEL_NS_H(); }
#define BMI088_accel_read_muli_reg(reg, data, len) \
    { BMI088_ACCEL_NS_L(); BMI088_read_write_byte((reg) | 0x80); \
      BMI088_read_muli_reg((reg), (data), (len)); BMI088_ACCEL_NS_H(); }
#define BMI088_gyro_write_single_reg(reg, data) \
    { BMI088_GYRO_NS_L(); BMI088_write_single_reg((reg), (data)); BMI088_GYRO_NS_H(); }
#define BMI088_gyro_read_single_reg(reg, data) \
    { BMI088_GYRO_NS_L(); BMI088_read_single_reg((reg), &(data)); BMI088_GYRO_NS_H(); }
#define BMI088_gyro_read_muli_reg(reg, data, len) \
    { BMI088_GYRO_NS_L(); BMI088_read_muli_reg((reg), (data), (len)); BMI088_GYRO_NS_H(); }

static void BMI088_write_single_reg(uint8_t reg, uint8_t data);
static void BMI088_read_single_reg(uint8_t reg, uint8_t *return_data);
static void BMI088_read_muli_reg(uint8_t reg, uint8_t *buf, uint8_t len);

static uint8_t write_BMI088_accel_reg_data_error[BMI088_WRITE_ACCEL_REG_NUM][3] = {
    {BMI088_ACC_PWR_CTRL, BMI088_ACC_ENABLE_ACC_ON, BMI088_ACC_PWR_CTRL_ERROR},
    {BMI088_ACC_PWR_CONF, BMI088_ACC_PWR_ACTIVE_MODE, BMI088_ACC_PWR_CONF_ERROR},
    {BMI088_ACC_CONF, BMI088_ACC_NORMAL | BMI088_ACC_800_HZ | BMI088_ACC_CONF_MUST_Set, BMI088_ACC_CONF_ERROR},
    {BMI088_ACC_RANGE, BMI088_ACC_RANGE_3G, BMI088_ACC_RANGE_ERROR},
    {BMI088_INT1_IO_CTRL, BMI088_ACC_INT1_IO_ENABLE | BMI088_ACC_INT1_GPIO_PP | BMI088_ACC_INT1_GPIO_LOW, BMI088_INT1_IO_CTRL_ERROR},
    {BMI088_INT_MAP_DATA, BMI088_ACC_INT1_DRDY_INTERRUPT, BMI088_INT_MAP_DATA_ERROR}
};

static uint8_t write_BMI088_gyro_reg_data_error[BMI088_WRITE_GYRO_REG_NUM][3] = {
    {BMI088_GYRO_RANGE, BMI088_GYRO_2000, BMI088_GYRO_RANGE_ERROR},
    {BMI088_GYRO_BANDWIDTH, BMI088_GYRO_1000_116_HZ | BMI088_GYRO_BANDWIDTH_MUST_Set, BMI088_GYRO_BANDWIDTH_ERROR},
    {BMI088_GYRO_LPM1, BMI088_GYRO_NORMAL_MODE, BMI088_GYRO_LPM1_ERROR},
    {BMI088_GYRO_CTRL, BMI088_DRDY_ON, BMI088_GYRO_CTRL_ERROR},
    {BMI088_GYRO_INT3_INT4_IO_CONF, BMI088_GYRO_INT3_GPIO_PP | BMI088_GYRO_INT3_GPIO_LOW, BMI088_GYRO_INT3_INT4_IO_CONF_ERROR},
    {BMI088_GYRO_INT3_INT4_IO_MAP, BMI088_GYRO_DRDY_IO_INT3, BMI088_GYRO_INT3_INT4_IO_MAP_ERROR}
};

/** @brief 初始化 BMI088，按教程合并两部分错误码。 @return 0 成功；非零为初始化错误。 */
uint8_t BMI088_init(void)
{
    uint8_t error = BMI088_NO_ERROR;
    BMI088_spi_error = 0;
    BMI088_GPIO_init();
    BMI088_com_init();
    error |= bmi088_accel_init();
    error |= bmi088_gyro_init();
    return error;
}

/** @brief 按教程双读 ID、复位、配置并回读加速度计。 @return 0 成功；非零为 ID 或配置错误。 */
uint8_t bmi088_accel_init(void)
{
    uint8_t res = 0;
    uint8_t write_reg_num = 0;
    BMI088_accel_read_single_reg(BMI088_ACC_CHIP_ID, res);
    BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);
    BMI088_accel_read_single_reg(BMI088_ACC_CHIP_ID, res);
    BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);

    BMI088_accel_write_single_reg(BMI088_ACC_SOFTRESET, BMI088_ACC_SOFTRESET_VALUE);
    BMI088_delay_ms(BMI088_LONG_DELAY_TIME);
    BMI088_accel_read_single_reg(BMI088_ACC_CHIP_ID, res);
    BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);
    BMI088_accel_read_single_reg(BMI088_ACC_CHIP_ID, res);
    BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);
    if (res != BMI088_ACC_CHIP_ID_VALUE)
    {
        return BMI088_NO_SENSOR;
    }
    for (write_reg_num = 0; write_reg_num < BMI088_WRITE_ACCEL_REG_NUM; write_reg_num++)
    {
        BMI088_accel_write_single_reg(write_BMI088_accel_reg_data_error[write_reg_num][0], write_BMI088_accel_reg_data_error[write_reg_num][1]);
        BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);
        BMI088_accel_read_single_reg(write_BMI088_accel_reg_data_error[write_reg_num][0], res);
        BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);
        if (res != write_BMI088_accel_reg_data_error[write_reg_num][1])
        {
            return write_BMI088_accel_reg_data_error[write_reg_num][2];
        }
    }
    return BMI088_NO_ERROR;
}

/** @brief 按教程双读 ID、复位、配置并回读陀螺仪。 @return 0 成功；非零为 ID 或配置错误。 */
uint8_t bmi088_gyro_init(void)
{
    uint8_t write_reg_num = 0;
    uint8_t res = 0;
    BMI088_gyro_read_single_reg(BMI088_GYRO_CHIP_ID, res);
    BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);
    BMI088_gyro_read_single_reg(BMI088_GYRO_CHIP_ID, res);
    BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);

    BMI088_gyro_write_single_reg(BMI088_GYRO_SOFTRESET, BMI088_GYRO_SOFTRESET_VALUE);
    BMI088_delay_ms(BMI088_LONG_DELAY_TIME);
    BMI088_gyro_read_single_reg(BMI088_GYRO_CHIP_ID, res);
    BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);
    BMI088_gyro_read_single_reg(BMI088_GYRO_CHIP_ID, res);
    BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);
    if (res != BMI088_GYRO_CHIP_ID_VALUE)
    {
        return BMI088_NO_SENSOR;
    }
    for (write_reg_num = 0; write_reg_num < BMI088_WRITE_GYRO_REG_NUM; write_reg_num++)
    {
        BMI088_gyro_write_single_reg(write_BMI088_gyro_reg_data_error[write_reg_num][0], write_BMI088_gyro_reg_data_error[write_reg_num][1]);
        BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);
        BMI088_gyro_read_single_reg(write_BMI088_gyro_reg_data_error[write_reg_num][0], res);
        BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);
        if (res != write_BMI088_gyro_reg_data_error[write_reg_num][1])
        {
            return write_BMI088_gyro_reg_data_error[write_reg_num][2];
        }
    }
    return BMI088_NO_ERROR;
}

/** @brief 直接读取并换算数据，保留教程原有 ID 判断和温度符号扩展。
 * @param[out] gyro 三轴 rad/s。 @param[out] accel 三轴 m/s²。
 * @param[out] temperate 温度 °C。 @return 无。
 * @note 温控适配：本次 SPI 失败或任一芯片 ID 不符时，温度置为 NAN；
 *       任务据此关闭加热并跳过此样本，不把通信失败解码成 23°C。
 */
void BMI088_read(float gyro[3], float accel[3], float *temperate)
{
    uint8_t buf[8] = {0, 0, 0, 0, 0, 0};
    uint8_t accel_id;
    uint8_t gyro_id_valid;
    int16_t bmi088_raw_temp;
    BMI088_spi_error = 0;
    BMI088_accel_read_muli_reg(BMI088_ACCEL_XOUT_L, buf, 6);
    bmi088_raw_temp = (int16_t)((buf[1]) << 8) | buf[0];
    accel[0] = bmi088_raw_temp * BMI088_ACCEL_SEN;
    bmi088_raw_temp = (int16_t)((buf[3]) << 8) | buf[2];
    accel[1] = bmi088_raw_temp * BMI088_ACCEL_SEN;
    bmi088_raw_temp = (int16_t)((buf[5]) << 8) | buf[4];
    accel[2] = bmi088_raw_temp * BMI088_ACCEL_SEN;
    BMI088_gyro_read_muli_reg(BMI088_GYRO_CHIP_ID, buf, 8);
    gyro_id_valid = (buf[0] == BMI088_GYRO_CHIP_ID_VALUE);
    if (gyro_id_valid)
    {
        bmi088_raw_temp = (int16_t)((buf[3]) << 8) | buf[2];
        gyro[0] = bmi088_raw_temp * BMI088_GYRO_SEN;
        bmi088_raw_temp = (int16_t)((buf[5]) << 8) | buf[4];
        gyro[1] = bmi088_raw_temp * BMI088_GYRO_SEN;
        bmi088_raw_temp = (int16_t)((buf[7]) << 8) | buf[6];
        gyro[2] = bmi088_raw_temp * BMI088_GYRO_SEN;
    }
    /* 温度属于 accel 芯片，不能仅凭 gyro ID 或 HAL_OK 判断加热测量有效。 */
    BMI088_accel_read_single_reg(BMI088_ACC_CHIP_ID, accel_id);
    BMI088_accel_read_muli_reg(BMI088_TEMP_M, buf, 2);
    if (BMI088_spi_error || accel_id != BMI088_ACC_CHIP_ID_VALUE || !gyro_id_valid)
    {
        *temperate = NAN;
        return;
    }
    bmi088_raw_temp = (int16_t)((buf[0] << 3) | (buf[1] >> 5));
    if (bmi088_raw_temp > 1023)
    {
        bmi088_raw_temp -= 2048;
    }
    *temperate = bmi088_raw_temp * BMI088_TEMP_FACTOR + BMI088_TEMP_OFFSET;
}

/** @brief 向已选中芯片写入单寄存器。
 * @param[in] reg 地址。 @param[in] data 写入值。 @return 无。
 */
static void BMI088_write_single_reg(uint8_t reg, uint8_t data)
{
    BMI088_read_write_byte(reg);
    BMI088_read_write_byte(data);
}

/** @brief 从已选中芯片读取单寄存器。
 * @param[in] reg 地址。 @param[out] return_data 接收字节。 @return 无。
 */
static void BMI088_read_single_reg(uint8_t reg, uint8_t *return_data)
{
    BMI088_read_write_byte(reg | 0x80);
    *return_data = BMI088_read_write_byte(0x55);
}

/** @brief 连续读取寄存器，片选由教程宏控制。
 * @param[in] reg 起始地址。 @param[out] buf 至少 len 字节。
 * @param[in] len 读取字节数。 @return 无。
 */
static void BMI088_read_muli_reg(uint8_t reg, uint8_t *buf, uint8_t len)
{
    BMI088_read_write_byte(reg | 0x80);
    while (len != 0)
    {
        *buf = BMI088_read_write_byte(0x55);
        buf++;
        len--;
    }
}
