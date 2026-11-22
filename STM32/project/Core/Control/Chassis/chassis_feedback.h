#ifndef CHASSIS_FEEDBACK_H
#define CHASSIS_FEEDBACK_H

#include "chassis_time.h"
#include "dm_motor.h"

typedef struct
{
    uint8_t id;
    uint8_t state;
    float position; /* 电机轴角度，rad。 */
    float velocity; /* 电机轴角速度，rad/s。 */
    uint32_t feedback_time_ms;
    uint32_t feedback_seq;
} Chassis_Wheel_Feedback_t;

typedef struct
{
    uint32_t now_ms;
    Chassis_Wheel_Feedback_t wheel[3];
} Chassis_Feedback_Snapshot_t;

/* 与 CAN 接收中断使用同一 RTOS 时钟，只在临界区内取时和复制。 */
static inline void Chassis_Feedback_Read(Chassis_Feedback_Snapshot_t *snapshot)
{
    taskENTER_CRITICAL();
    snapshot->now_ms = Chassis_Time_Now_Ms();
    for (unsigned i = 0U; i < 3U; ++i)
    {
        const volatile dm_motor_t *motor = &dm_motor[i];
        Chassis_Wheel_Feedback_t *wheel = &snapshot->wheel[i];
        wheel->id = motor->id;
        wheel->state = motor->state;
        wheel->position = motor->position;
        wheel->velocity = motor->velocity;
        wheel->feedback_time_ms = motor->feedback_time_ms;
        wheel->feedback_seq = motor->feedback_seq;
    }
    taskEXIT_CRITICAL();
}

/* 角度和角速度从电机轴换算到同一几何轮轴。 */
static inline float Chassis_Feedback_To_Wheel_Axis(float motor_value, const Chassis_Wheel_Config_t *config)
{
    return motor_value * config->direction / config->gear_ratio;
}

#endif
