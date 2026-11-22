#include "velocity_profile.h"
#include <math.h>

void Velocity_Profile_Seed(Velocity_Profile_t *profile, float vx, float vy, float wz)
{
    const float start[] = {vx, vy, wz};
    for (unsigned i = 0; i < 3; ++i)
        profile->output[i] = profile->ramp[i] = start[i];
}

void Velocity_Profile_Step(Velocity_Profile_t *profile, const float target[3], float linear_accel,
                           float angular_accel, float smoothing_s, float dt)
{
    if (dt <= 0.0f)
        return;
    const float linear_step = linear_accel * dt;
    const float angular_step = angular_accel * dt;
    const float linear_error = hypotf(target[0] - profile->ramp[0], target[1] - profile->ramp[1]);
    const float angular_error = fabsf(target[2] - profile->ramp[2]);
    float fraction = linear_error > linear_step ? linear_step / linear_error : 1.0f;
    if (angular_error > angular_step)
        fraction = fminf(fraction, angular_step / angular_error);
    const float gain = dt / (smoothing_s + dt);
    /* Smooth the slope changes of the bounded ramp. Snap only within a fraction of a
     * wheel velocity code, and only when this last step also meets the acceleration limits. */
    const float output_xy_error = hypotf(target[0] - profile->output[0], target[1] - profile->output[1]);
    const float output_wz_error = fabsf(target[2] - profile->output[2]);
    const int settled = fraction == 1.0f && output_xy_error <= fminf(0.0001f, linear_step) &&
                        output_wz_error <= fminf(0.001f, angular_step);
    for (unsigned i = 0; i < 3; ++i)
    {
        profile->ramp[i] = fraction == 1.0f ? target[i] :
            profile->ramp[i] + (target[i] - profile->ramp[i]) * fraction;
        profile->output[i] = settled ? target[i] :
            profile->output[i] + (profile->ramp[i] - profile->output[i]) * gain;
    }
}

void Velocity_Profile_Step_Navigation(Velocity_Profile_t *profile, const float target[3],
                                      const Velocity_Navigation_Config_t *config,
                                      Navigation_Speed_State_t *speed_state, const float measured[2], float dt)
{
    if (dt <= 0.0f)
        return;

    float vx = target[0], vy = target[1];
    const float current_vx = profile->output[0], current_vy = profile->output[1];
    const float target_speed = hypotf(vx, vy);
    float acceleration = config->linear_decel_mps2;
    uint8_t accelerating = 0U;
    if (target_speed > 0.0f)
    {
        const float along = (current_vx * vx + current_vy * vy) / target_speed;
        if (along < 0.0f)
        {
            /* 目标反向时先制动；下一拍从零向当时的目标起步。 */
            vx = vy = 0.0f;
            acceleration = config->reversal_decel_mps2;
        }
        else if (target_speed > hypotf(current_vx, current_vy))
        {
            const float fraction = fminf(1.0f, along / config->full_accel_speed_mps);
            acceleration = config->start_accel_mps2 +
                (config->linear_accel_mps2 - config->start_accel_mps2) * fraction;
            accelerating = 1U;
        }
    }
    if (config->speed_mode != NAV_SPEED_BASELINE)
    {
        if (accelerating)
            acceleration = Navigation_Speed_Acceleration(speed_state, config->speed_mode,
                                                         profile->output, target, measured, acceleration, dt);
        else
            *speed_state = (Navigation_Speed_State_t){0};
    }

    /* 合加速度受限；速度差小于本拍额度时精确到达，无固定滤波尾段。 */
    const float dx = vx - current_vx, dy = vy - current_vy;
    const float gap = hypotf(dx, dy);
    const float linear_step = acceleration * dt;
    if (gap > linear_step)
    {
        const float fraction = linear_step / gap;
        vx = current_vx + dx * fraction;
        vy = current_vy + dy * fraction;
    }

    const float yaw_step = config->angular_accel_rad_s2 * dt;
    const float yaw_gap = target[2] - profile->output[2];
    const float wz = fabsf(yaw_gap) <= yaw_step ? target[2] :
        profile->output[2] + copysignf(yaw_step, yaw_gap);
    Velocity_Profile_Seed(profile, vx, vy, wz);
}
