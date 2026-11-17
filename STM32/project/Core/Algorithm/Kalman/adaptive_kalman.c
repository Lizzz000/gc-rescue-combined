#include "adaptive_kalman.h"

#include <math.h>
#include <stddef.h>

#define INITIAL_VELOCITY_VARIANCE 1000000.0f
#define NOISE_MEMORY_SECONDS 0.323333333f
#define SCALE_MAX 4.0f
#define OUTLIER_NIS 100.0f
#define RESET_INTERVAL_MS 250U

static uint8_t Valid_Sigma(float value)
{
    return isfinite(value) && value >= 0.01f && value <= 1000000.0f;
}

void AdaptiveKalman_Init(AdaptiveKalman *filter, float sigma_x, float sigma_y, float sigma_acceleration)
{
    if (filter == NULL)
        return;
    *filter = (AdaptiveKalman){0};
    if (!Valid_Sigma(sigma_x) || !Valid_Sigma(sigma_y) || !Valid_Sigma(sigma_acceleration))
        return;
    filter->measurement_variance[0] = sigma_x * sigma_x;
    filter->measurement_variance[1] = sigma_y * sigma_y;
    filter->measurement_scale[0] = filter->measurement_scale[1] = 1.0f;
    filter->acceleration_variance = sigma_acceleration * sigma_acceleration;
}

static void Reset_Axis(AdaptiveKalman *filter, unsigned axis, float measurement)
{
    filter->position[axis] = measurement;
    filter->velocity[axis] = 0.0f;
    filter->position_variance[axis] = filter->measurement_variance[axis];
    filter->velocity_variance[axis] = INITIAL_VELOCITY_VARIANCE;
    filter->position_velocity_covariance[axis] = 0.0f;
    filter->measurement_scale[axis] = 1.0f;
    filter->rejected_run[axis] = 0U;
    filter->rejected_side[axis] = 0;
}

void AdaptiveKalman_Update(AdaptiveKalman *filter, float x, float y, uint32_t timestamp_ms)
{
    if (filter == NULL)
        return;
    filter->accepted_mask = 0U;
    if (!isfinite(x) || !isfinite(y) || fabsf(x) > 10000000.0f || fabsf(y) > 10000000.0f ||
        filter->measurement_variance[0] <= 0.0f || filter->measurement_variance[1] <= 0.0f)
        return;
    const float measurement[2] = {x, y};
    const uint32_t elapsed_ms = timestamp_ms - filter->timestamp_ms;
    if (filter->initialized && (elapsed_ms == 0U || elapsed_ms > INT32_MAX))
        return;
    if (!filter->initialized || elapsed_ms >= RESET_INTERVAL_MS)
    {
        for (unsigned axis = 0U; axis < 2U; ++axis)
            Reset_Axis(filter, axis, measurement[axis]);
        filter->timestamp_ms = timestamp_ms;
        filter->initialized = 1U;
        filter->accepted_mask = 3U;
        return;
    }
    filter->timestamp_ms = timestamp_ms;
    const float dt = (float)elapsed_ms * 0.001f;
    const float q_velocity = filter->acceleration_variance * dt * dt;
    const float q_cross = q_velocity * 0.5f * dt;
    const float q_position = q_cross * 0.5f * dt;
    /* 按秒更新噪声记忆，15/30/100fps 使用相同时间常数。 */
    const float alpha = dt / (NOISE_MEMORY_SECONDS + dt);
    for (unsigned axis = 0U; axis < 2U; ++axis)
    {
        const float cross = filter->position_velocity_covariance[axis];
        const float vv = filter->velocity_variance[axis];
        const float predicted = filter->position[axis] + dt * filter->velocity[axis];
        const float pp = filter->position_variance[axis] + dt * (2.0f * cross + dt * vv) + q_position;
        const float pv = cross + dt * vv + q_cross;
        const float predicted_vv = vv + q_velocity;
        const float innovation = measurement[axis] - predicted;
        const float base_r = filter->measurement_variance[axis];
        float r = base_r * filter->measurement_scale[axis];
        const float nis = innovation * innovation / (pp + r);
        if (nis > OUTLIER_NIS)
        {
            const int8_t side = innovation > 0.0f ? 1 : -1;
            filter->rejected_run[axis] = side == filter->rejected_side[axis] ?
                filter->rejected_run[axis] + 1U : 1U;
            filter->rejected_side[axis] = side;
            if (filter->rejected_run[axis] >= 3U)
            {
                Reset_Axis(filter, axis, measurement[axis]);
                filter->accepted_mask |= (uint8_t)(1U << axis);
            }
            else
            {
                filter->position[axis] = predicted;
                filter->position_variance[axis] = pp;
                filter->position_velocity_covariance[axis] = pv;
                filter->velocity_variance[axis] = predicted_vv;
            }
            continue;
        }
        filter->rejected_run[axis] = 0U;
        filter->rejected_side[axis] = 0;
        /* 保留原滤波器10ms下的正常帧增益；记忆按秒缩放。大创新仅影响本次降权。 */
        const float bounded_nis = fminf(16.0f, nis);
        filter->measurement_scale[axis] = fminf(SCALE_MAX, fmaxf(1.0f,
            (1.0f - alpha) * filter->measurement_scale[axis] + alpha * bounded_nis));
        if (bounded_nis > 9.0f)
            r *= fminf(SCALE_MAX, bounded_nis / 9.0f);
        const float kp = pp / (pp + r), kv = pv / (pp + r), a = 1.0f - kp;
        filter->position[axis] = predicted + kp * innovation;
        filter->velocity[axis] += kv * innovation;
        /* Joseph 形式，避免 P-KHP 的单精度消减；最后补偿浮点舍入保半正定。 */
        const float p00 = fmaxf(1e-6f, a * a * pp + kp * kp * r);
        const float p01 = a * (pv - kv * pp) + kp * kv * r;
        const float p11 = predicted_vv - 2.0f * kv * pv + kv * kv * (pp + r);
        filter->position_variance[axis] = p00;
        filter->position_velocity_covariance[axis] = p01;
        filter->velocity_variance[axis] = fmaxf(p01 * p01 / p00, p11);
        filter->accepted_mask |= (uint8_t)(1U << axis);
    }
}
