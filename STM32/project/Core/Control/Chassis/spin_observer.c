#include "spin_observer.h"
#include "motor_chassis.h"
#include <math.h>
#include <stddef.h>

uint8_t Spin_Observer_Update(Spin_Observer_t *state, const float position[3],
                             float yaw_rad, float gyro_rad_s, uint32_t now_ms)
{
    if (state == NULL || position == NULL || !isfinite(yaw_rad) ||
        !isfinite(gyro_rad_s) || fabsf(gyro_rad_s) > 8.0f)
        return 0U;
    for (unsigned i = 0; i < 3U; ++i)
        if (!isfinite(position[i]) || fabsf(position[i]) > 12.5f)
            return 0U;
    if (state->initialized)
    {
        const uint32_t elapsed = now_ms - state->previous_ms;
        if (!elapsed) return 1U;
        if (elapsed > 50U) return 0U;
        const float dt = elapsed * .001f;
        float delta[3], body[3];
        for (unsigned i = 0; i < 3U; ++i)
        {
            delta[i] = remainderf(position[i] - state->previous_position[i], 25.0f);
            if (fabsf(delta[i]) > 40.0f * (dt + .030f) + .002f)
                return 0U;
        }
        const float yaw_delta = remainderf(yaw_rad - state->previous_yaw, 6.28318530717958647692f);
        if (fabsf(yaw_delta) > 8.0f * dt + .08f)
            return 0U;
        Motor_Chassis_Forward(delta, body);
        state->encoder_rad += body[2];
        state->yaw_rad += yaw_delta;
        state->gyro_rad += .5 * (gyro_rad_s + state->previous_rate) * dt;
    }
    for (unsigned i = 0; i < 3U; ++i)
        state->previous_position[i] = position[i];
    state->previous_yaw = yaw_rad;
    state->previous_rate = gyro_rad_s;
    state->previous_ms = now_ms;
    state->initialized = 1U;
    return 1U;
}

float Spin_Test_Speed(double yaw_rad, unsigned *stage)
{
    static const float speeds[5] = {2.0f, 3.0f, 4.0f, 3.0f, 2.0f};
    if (!isfinite(yaw_rad) || yaw_rad < -.2 || yaw_rad > SPIN_TEST_ANGLE_RAD + .2)
        return 0.0f;
    unsigned index = yaw_rad > 0.0 ? (unsigned)(yaw_rad / (6.0 * 6.28318530717958647692)) : 0U;
    if (index > 4U) index = 4U;
    if (stage) *stage = index;
    const float remaining = (float)(SPIN_TEST_ANGLE_RAD - yaw_rad);
    if (remaining <= .02f) return 0.0f;
    /* 低于现有1.6rad/s²斜坡的制动假设，并计入80ms执行余量。 */
    const float a = 1.4f, at = 1.4f * .08f;
    const float twice_ad = 2.0f * a * (remaining - .01f);
    return fminf(speeds[index], twice_ad / (sqrtf(at * at + twice_ad) + at));
}
