#ifndef TEST_MAIN_H
#define TEST_MAIN_H
#include "stm32f4xx_hal.h"
#define CS1_ACCEL_GPIO_Port ((void *)1)
#define CS1_ACCEL_Pin 1U
#define CS1_GYRO_GPIO_Port ((void *)2)
#define CS1_GYRO_Pin 2U
#define RSTN_IST8310_GPIO_Port ((void *)3)
#define RSTN_IST8310_Pin 3U
#define IMU_HEAT_GPIO_Port ((void *)4)
#define IMU_HEAT_Pin 4U
#define LED_RED_GPIO_Port ((void *)5)
#define LED_RED_Pin 1U
#define LED_GREEN_GPIO_Port ((void *)6)
#define LED_GREEN_Pin 2U
#define LED_BLUE_GPIO_Port ((void *)7)
#define LED_BLUE_Pin 4U
void Error_Handler(void);
#endif
