#ifndef NAVIGATION_SPEED_H
#define NAVIGATION_SPEED_H

#include <stdint.h>

typedef enum
{
    NAV_SPEED_BASELINE = 0,
    NAV_SPEED_MPC = 1,
    NAV_SPEED_SMC = 2,
    NAV_SPEED_ADRC = 3
} Navigation_Speed_Mode_t;

typedef struct
{
    float observed_velocity_mps[2];
    float disturbance_mps2[2];
    float previous_accel_mps2[2];
    uint8_t observer_ready;
} Navigation_Speed_State_t;

/* 导航加速时沿目标与当前指令的速度差求加速度幅值。速度使用同一坐标系，
 * 反馈应来自新鲜里程计，NULL 时清空记忆并沿用现行起步上限。采样间隔和上限均为正。
 * 调用方限制最后一步不越过目标；减速、换向或切换坐标系时清空本控制器状态。 */
float Navigation_Speed_Acceleration(Navigation_Speed_State_t *state, Navigation_Speed_Mode_t mode,
                                    const float command[2], const float target[2], const float measured[2],
                                    float acceleration_limit, float dt);

#endif
