/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2019 DJI / RoboMaster.
 * 来源：11.ist8310/bsp/boards/ist8310driver_middleware.c。
 * 2026-11-05：适配本工程 GPIO/I2C3；毫秒延时用 RTOS，微秒延时用 TIM2。
 */
#include "ist8310driver_middleware.h"
#include "main.h"
#include "i2c.h"
#include "tim.h"
#include "cmsis_os.h"

/** @brief 保留教程空接口；GPIO 已由 CubeMX 初始化。 @return 无。 */
void ist8310_GPIO_init(void)
{
}

/** @brief 保留教程空接口；I2C3 已由 CubeMX 初始化。 @return 无。 */
void ist8310_com_init(void)
{
}

/** @brief 通过 I2C3 读取单字节。 @param[in] reg 地址。 @return 寄存器值。 */
uint8_t ist8310_IIC_read_single_reg(uint8_t reg)
{
    uint8_t res = 0;
    HAL_I2C_Mem_Read(&hi2c3, IST8310_IIC_ADDRESS << 1, reg, I2C_MEMADD_SIZE_8BIT, &res, 1, 10);
    return res;
}

/** @brief 通过 I2C3 写入单字节。
 * @param[in] reg 地址。 @param[in] data 写入值。 @return 无。
 */
void ist8310_IIC_write_single_reg(uint8_t reg, uint8_t data)
{
    HAL_I2C_Mem_Write(&hi2c3, IST8310_IIC_ADDRESS << 1, reg, I2C_MEMADD_SIZE_8BIT, &data, 1, 10);
}

/** @brief 通过 I2C3 连续读取寄存器。
 * @param[in] reg 起始地址。 @param[out] buf 至少 len 字节。
 * @param[in] len 字节数。 @return 本次HAL读取成功为1，失败为0。
 * @note 保留教程函数名；返回HAL结果是本工程适配，失败时不得解码旧缓冲。
 */
uint8_t ist8310_IIC_read_muli_reg(uint8_t reg, uint8_t *buf, uint8_t len)
{
    return HAL_I2C_Mem_Read(&hi2c3, IST8310_IIC_ADDRESS << 1, reg, I2C_MEMADD_SIZE_8BIT, buf, len, 10) == HAL_OK;
}

/** @brief 让出当前任务并等待指定毫秒。 @param[in] ms 毫秒。 @return 无。 */
void ist8310_delay_ms(uint16_t ms)
{
    osDelay(ms);
}

/** @brief 用 1 MHz TIM2 延时，调用前由 INS_task 启动计数器。
 * @param[in] us 0～65535 µs。 @return 无。
 */
void ist8310_delay_us(uint16_t us)
{
    uint32_t start = __HAL_TIM_GET_COUNTER(&htim2);
    while ((uint32_t)(__HAL_TIM_GET_COUNTER(&htim2) - start) < us)
    {
    }
}

/** @brief 拉低 IST8310 复位引脚。 @return 无。 */
void ist8310_RST_L(void)
{
    HAL_GPIO_WritePin(RSTN_IST8310_GPIO_Port, RSTN_IST8310_Pin, GPIO_PIN_RESET);
}

/** @brief 释放 IST8310 复位引脚。 @return 无。 */
void ist8310_RST_H(void)
{
    HAL_GPIO_WritePin(RSTN_IST8310_GPIO_Port, RSTN_IST8310_Pin, GPIO_PIN_SET);
}
