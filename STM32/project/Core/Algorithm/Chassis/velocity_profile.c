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
