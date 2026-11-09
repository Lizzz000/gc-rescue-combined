/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2019 DJI / RoboMaster.
 * 来源：https://github.com/RoboMaster/Development-Board-C-Examples/blob/master/11.ist8310/bsp/boards/ist8310driver.c
 * 2026-11-05：保留本阶段使用的 init/read_over；补中文注释。
 */
#include "ist8310driver.h"
#include "ist8310driver_middleware.h"

#define MAG_SEN 0.3f
#define IST8310_WHO_AM_I 0x00
#define IST8310_WHO_AM_I_VALUE 0x10
#define IST8310_WRITE_REG_NUM 4

static const uint8_t ist8310_write_reg_data_error[IST8310_WRITE_REG_NUM][3] = {
    {0x0B, 0x08, 0x01}, /* 开启数据就绪，低电平有效。 */
    {0x41, 0x09, 0x02}, /* 平均采样两次。 */
    {0x42, 0xC0, 0x03}, /* 教程规定值。 */
    {0x0A, 0x0B, 0x04}  /* 200 Hz 输出。 */
};

/** @brief 按教程复位、读 ID、设置并回读配置。
 * @return 0 成功；1～4 配置错误；0x40 无传感器。
 */
uint8_t ist8310_init(void)
{
    static const uint8_t wait_time = 150;
    static const uint8_t sleepTime = 50;
    uint8_t res = 0;
    uint8_t writeNum = 0;

    ist8310_GPIO_init();
    ist8310_com_init();
    ist8310_RST_L();
    ist8310_delay_ms(sleepTime);
    ist8310_RST_H();
    ist8310_delay_ms(sleepTime);
    res = ist8310_IIC_read_single_reg(IST8310_WHO_AM_I);
    if (res != IST8310_WHO_AM_I_VALUE)
    {
        return IST8310_NO_SENSOR;
    }
    for (writeNum = 0; writeNum < IST8310_WRITE_REG_NUM; writeNum++)
    {
        ist8310_IIC_write_single_reg(ist8310_write_reg_data_error[writeNum][0], ist8310_write_reg_data_error[writeNum][1]);
        ist8310_delay_us(wait_time);
        res = ist8310_IIC_read_single_reg(ist8310_write_reg_data_error[writeNum][0]);
        ist8310_delay_us(wait_time);
        if (res != ist8310_write_reg_data_error[writeNum][1])
        {
            return ist8310_write_reg_data_error[writeNum][2];
        }
    }
    return IST8310_NO_ERROR;
}

/** @brief 按教程状态位判断并逐轴换算磁场。
 * @param[in] status_buf STAT1 和 XYZ 共 7 字节。
 * @param[in,out] ist8310_real_data 教程状态和三轴磁场 µT。 @return 无。
 */
void ist8310_read_over(uint8_t *status_buf, ist8310_real_data_t *ist8310_real_data)
{
    if (status_buf[0] & 0x01)
    {
        int16_t temp_ist8310_data = 0;
        ist8310_real_data->status |= 1 << IST8310_DATA_READY_BIT;
        temp_ist8310_data = (int16_t)((status_buf[2] << 8) | status_buf[1]);
        ist8310_real_data->mag[0] = MAG_SEN * temp_ist8310_data;
        temp_ist8310_data = (int16_t)((status_buf[4] << 8) | status_buf[3]);
        ist8310_real_data->mag[1] = MAG_SEN * temp_ist8310_data;
        temp_ist8310_data = (int16_t)((status_buf[6] << 8) | status_buf[5]);
        ist8310_real_data->mag[2] = MAG_SEN * temp_ist8310_data;
    }
    else
    {
        ist8310_real_data->status &= ~(1 << IST8310_DATA_READY_BIT);
    }
}
