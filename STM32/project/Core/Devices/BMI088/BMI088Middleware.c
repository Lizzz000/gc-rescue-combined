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

/** @brief 保留教程空接口；GPIO 已由 CubeMX 初始化。 @return 无。 */
void BMI088_GPIO_init(void)
{
}

/** @brief 保留教程空接口；SPI1 已由 CubeMX 初始化。 @return 无。 */
void BMI088_com_init(void)
{
}

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

/** @brief 使用教程的 SPI 单字节收发方式，向读取层传递本次 HAL 失败。
 * @param[in] txdata 发送字节。 @return 接收字节；失败时返回 0。
 * @note 当前 SPI 为 328.125 kbit/s，单字节约 24.4 us；超时取 2 ms，
 *       留出 HAL 毫秒计时余量。首个失败后跳过本次剩余字节，读取层马上返回
 *       无效温度，由任务关闭加热；这里不依赖温控 PWM 模块。
 */
uint8_t BMI088_read_write_byte(uint8_t txdata)
{
    uint8_t rx_data = 0;
    if (BMI088_spi_error)
    {
        return 0;
    }
    if (HAL_SPI_TransmitReceive(&hspi1, &txdata, &rx_data, 1, 2) != HAL_OK)
    {
        BMI088_spi_error = 1;
        return 0;
    }
    return rx_data;
}
