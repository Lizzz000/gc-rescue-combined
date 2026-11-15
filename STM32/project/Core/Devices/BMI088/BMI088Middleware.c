/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2019 RoboMaster.
 * 来源：13.spi_bmi088/component/devices/BMI088Middleware.c。
 * 2026-11-05：适配本工程 GPIO/SPI1；毫秒延时用 RTOS，微秒延时用 TIM2。
 * 开启温控后增加本次事务的 HAL 错误传递，避免失败字节被解码为有效温度。
 */
#include "BMI088Middleware.h"
#include "main.h"
#include "spi.h"
#include "tim.h"
#include "cmsis_os.h"

uint8_t BMI088_spi_error;

/** @brief 让出当前任务并等待指定毫秒。 @param[in] ms 毫秒。 @return 无。 */
void BMI088_delay_ms(uint16_t ms)
{
    osDelay(ms);
}

/** @brief 用 1 MHz TIM2 延时，调用前由 INS_task 启动计数器。
 * @param[in] us 0～65535 µs。 @return 无。
 */
void BMI088_delay_us(uint16_t us)
{
    uint32_t start = __HAL_TIM_GET_COUNTER(&htim2);
    while ((uint32_t)(__HAL_TIM_GET_COUNTER(&htim2) - start) < us)
    {
    }
}

/** @brief 拉低加速度计片选。 @return 无。 */
void BMI088_ACCEL_NS_L(void)
{
    HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, GPIO_PIN_RESET);
}

/** @brief 拉高加速度计片选。 @return 无。 */
void BMI088_ACCEL_NS_H(void)
{
    HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, GPIO_PIN_SET);
}

/** @brief 拉低陀螺仪片选。 @return 无。 */
void BMI088_GYRO_NS_L(void)
{
    HAL_GPIO_WritePin(CS1_GYRO_GPIO_Port, CS1_GYRO_Pin, GPIO_PIN_RESET);
}

/** @brief 拉高陀螺仪片选。 @return 无。 */
void BMI088_GYRO_NS_H(void)
{
    HAL_GPIO_WritePin(CS1_GYRO_GPIO_Port, CS1_GYRO_Pin, GPIO_PIN_SET);
}

/** @brief 连续 SPI 收发；失败向读取层传递，首个失败后跳过本轮剩余事务。
 * @note SPI 保持 328.125 kbit/s，最长 9 字节约 220 us，保留 2 ms HAL 超时。
 */
void BMI088_read_write_data(uint8_t *tx, uint8_t *rx, uint16_t len)
{
    if (BMI088_spi_error) return;
    if (HAL_SPI_TransmitReceive(&hspi1, tx, rx, len, 2) != HAL_OK)
        BMI088_spi_error = 1;
}

/** @brief 初始化写寄存器使用的单字节交换。 */
uint8_t BMI088_read_write_byte(uint8_t txdata)
{
    uint8_t rx_data = 0;
    BMI088_read_write_data(&txdata, &rx_data, 1);
    return BMI088_spi_error ? 0 : rx_data;
}
