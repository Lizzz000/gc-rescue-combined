#include "adaptive_kalman.h"

#include <math.h>

/* 这些值是离线对照初值，必须结合实车观测标定。 */
#define INITIAL_VELOCITY_VARIANCE 1000000.0f
#define INNOVATION_MEMORY 0.97f
#define INNOVATION_LIMIT 16.0f
#define SCALE_MAX 4.0f
#define OUTLIER_NIS 9.0f

void AdaptiveKalman_Init(AdaptiveKalman *filter, float sigma_x, float sigma_y, float sigma_acceleration)
{
    *filter = (AdaptiveKalman){
        .measurement_variance = {sigma_x * sigma_x, sigma_y * sigma_y},
        .measurement_scale = {1.0f, 1.0f},
        .acceleration_variance = sigma_acceleration * sigma_acceleration,
    };
}

void AdaptiveKalman_Update(AdaptiveKalman *filter, float x, float y, uint32_t timestamp_ms)
{
    const float measurement[2] = {x, y};
    if (!filter->initialized)
    {
        for (unsigned axis = 0U; axis < 2U; ++axis)
        {
            filter->position[axis] = measurement[axis];
            filter->position_variance[axis] = filter->measurement_variance[axis];
            filter->velocity_variance[axis] = INITIAL_VELOCITY_VARIANCE;
        }
        filter->timestamp_ms = timestamp_ms;
        filter->initialized = 1U;
        return;
    }

    const uint32_t elapsed_ms = timestamp_ms - filter->timestamp_ms;
    if (elapsed_ms == 0U || elapsed_ms > INT32_MAX)
        return;
    filter->timestamp_ms = timestamp_ms;

    const float dt = (float)elapsed_ms * 0.001f;
    const float half_dt = 0.5f * dt;
    const float q_velocity = filter->acceleration_variance * dt * dt;
    const float q_cross = q_velocity * half_dt;
    const float q_position = q_cross * half_dt;
    for (unsigned axis = 0U; axis < 2U; ++axis)
    {
        const float cross = filter->position_velocity_covariance[axis];
        const float speed_variance = filter->velocity_variance[axis];
        const float predicted_position = filter->position[axis] + dt * filter->velocity[axis];
        const float predicted_position_variance = filter->position_variance[axis] +
            dt * (2.0f * cross + dt * speed_variance) + q_position;
        const float predicted_cross = cross + dt * speed_variance + q_cross;
        const float predicted_speed_variance = speed_variance + q_velocity;
        const float innovation = measurement[axis] - predicted_position;

        float variance = filter->measurement_variance[axis] * filter->measurement_scale[axis];
        const float nis = fminf(INNOVATION_LIMIT, innovation * innovation /
            (predicted_position_variance + variance));
        /* 用历史创新幅度调整后续测量方差，并对当前异常观测额外降权。 */
        filter->measurement_scale[axis] = fminf(SCALE_MAX, fmaxf(1.0f,
            INNOVATION_MEMORY * filter->measurement_scale[axis] + (1.0f - INNOVATION_MEMORY) * nis));
        if (nis > OUTLIER_NIS)
            variance *= fminf(SCALE_MAX, nis / OUTLIER_NIS);

        const float inverse = 1.0f / (predicted_position_variance + variance);
        const float position_gain = predicted_position_variance * inverse;
        const float speed_gain = predicted_cross * inverse;
        filter->position[axis] = predicted_position + position_gain * innovation;
        filter->velocity[axis] += speed_gain * innovation;
        filter->position_variance[axis] = variance * position_gain;
        filter->position_velocity_covariance[axis] = variance * speed_gain;
        const float min_speed_variance = filter->position_velocity_covariance[axis] *
            filter->position_velocity_covariance[axis] / filter->position_variance[axis];
        filter->velocity_variance[axis] = fmaxf(min_speed_variance,
            fmaf(-speed_gain, predicted_cross, predicted_speed_variance));
    }
}
