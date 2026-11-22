#ifndef CHASSIS_TIME_H
#define CHASSIS_TIME_H

#include <stdbool.h>
#include <stdint.h>
#include "chassis_config.h"
#include "FreeRTOS.h"
#include "task.h"
#include "tim.h"

typedef struct
{
    TickType_t last_control_tick;
    uint32_t last_control_us;
    uint32_t step_start_us;
} Chassis_Control_Clock_t;

/* FreeRTOS 当前每 tick 为 1 ms，与 CAN 和 INS 反馈时刻同源。 */
static inline uint32_t Chassis_Time_Now_Ms(void)
{
    return (uint32_t)xTaskGetTickCount();
}

/* TIM2 为 1 MHz 的 32 位自由运行计数器，无符号差值保留回绕语义。 */
static inline uint32_t Chassis_Time_Now_Us(void)
{
    return __HAL_TIM_GET_COUNTER(&htim2);
}

static inline float Chassis_Time_Us_To_S(uint32_t duration_us)
{
    return duration_us * 0.000001f;
}

static inline void Chassis_Time_Reset(Chassis_Control_Clock_t *clock)
{
    clock->last_control_tick = xTaskGetTickCount();
    clock->last_control_us = clock->step_start_us = Chassis_Time_Now_Us();
}

/* RTOS 决定名义发帧节拍；TIM2 未推进时让下一次任务唤醒重试本拍。 */
static inline bool Chassis_Time_Take_Control_Step(Chassis_Control_Clock_t *clock,
                                                 TickType_t now_tick,
                                                 uint32_t *now_us, uint32_t *elapsed_us)
{
    if ((TickType_t)(now_tick - clock->last_control_tick) < pdMS_TO_TICKS(CHASSIS_CONTROL_PERIOD_MS))
        return false;

    const uint32_t sampled_us = Chassis_Time_Now_Us();
    if (sampled_us == clock->last_control_us)
        return false;

    *now_us = sampled_us;
    *elapsed_us = sampled_us - clock->step_start_us;
    clock->last_control_tick = now_tick;
    clock->last_control_us = clock->step_start_us = sampled_us;
    return true;
}

#endif
