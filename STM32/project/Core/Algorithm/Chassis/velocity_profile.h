#ifndef VELOCITY_PROFILE_H
#define VELOCITY_PROFILE_H

#include "navigation_speed.h"

/* Body-frame vx, vy (m/s), wz (rad/s). Each controller owns its profile. */
typedef struct
{
    float ramp[3];
    float output[3];
} Velocity_Profile_t;

void Velocity_Profile_Seed(Velocity_Profile_t *profile, float vx, float vy, float wz);
/* Positive acceleration limits, smoothing time >= 0 and elapsed time >= 0, in SI units. */
void Velocity_Profile_Step(Velocity_Profile_t *profile, const float target[3], float linear_accel,
                           float angular_accel, float smoothing_s, float dt);

typedef struct
{
    float start_accel_mps2;
    float linear_accel_mps2;
    float linear_decel_mps2;
    float reversal_decel_mps2;
    float full_accel_speed_mps;
    float angular_accel_rad_s2;
    Navigation_Speed_Mode_t speed_mode;
} Velocity_Navigation_Config_t;

/* 起步可用反馈调整，同向跟随与换向采用各自的减速度上限。反馈与指令同坐标系。 */
void Velocity_Profile_Step_Navigation(Velocity_Profile_t *profile, const float target[3],
                                      const Velocity_Navigation_Config_t *config,
                                      Navigation_Speed_State_t *speed_state, const float measured[2], float dt);

#endif
