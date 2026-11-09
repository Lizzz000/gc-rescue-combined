#include "image_tracker.h"
#include <math.h>

/* 首帧尚无速度信息，速度先验标准差为 1000 q/s。 */
#define INITIAL_VELOCITY_VARIANCE (1000.0f * 1000.0f)

/** @brief 初始化滤波器，参数和单位见头文件。 */
void Image_Tracker_Init(Image_Tracker_t *tracker, float sigma_x, float sigma_y, float sigma_acceleration)
{
    *tracker = (Image_Tracker_t){
        .measurement_variance = {sigma_x * sigma_x, sigma_y * sigma_y},
        .acceleration_variance = sigma_acceleration * sigma_acceleration,
    };
}

/** @brief 使用实际接收间隔推进各轴常速度模型，参数和单位见头文件。 */
void Image_Tracker_Update(Image_Tracker_t *tracker, float x, float y, uint32_t received_ms)
{
    const float measurement[2] = {x, y};
    if (!tracker->initialized)
    {
        for (unsigned axis = 0U; axis < 2U; ++axis)
        {
            tracker->position[axis] = measurement[axis];
            tracker->position_variance[axis] = tracker->measurement_variance[axis];
            tracker->velocity_variance[axis] = INITIAL_VELOCITY_VARIANCE;
        }
        tracker->received_ms = received_ms;
        tracker->initialized = 1U;
        return;
    }

    const uint32_t elapsed_ms = received_ms - tracker->received_ms;
    if (elapsed_ms == 0U || elapsed_ms > INT32_MAX)
        return;
    tracker->received_ms = received_ms;

    const float dt = (float)elapsed_ms * 0.001f;
    const float half_dt = 0.5f * dt;
    /* 单步随机加速度映射到位置和速度，即 Q = Gσ²Gᵀ，G = [dt²/2, dt]。 */
    const float q_velocity = tracker->acceleration_variance * dt * dt;
    const float q_cross = q_velocity * half_dt;
    const float q_position = q_cross * half_dt;
    for (unsigned axis = 0U; axis < 2U; ++axis)
    {
        const float cross = tracker->position_velocity_covariance[axis];
        const float speed_variance = tracker->velocity_variance[axis];
        const float position = tracker->position[axis] + dt * tracker->velocity[axis];
        const float p_position = tracker->position_variance[axis] +
                                 dt * (2.0f * cross + dt * speed_variance) + q_position;
        const float p_cross = cross + dt * speed_variance + q_cross;
        const float p_speed = speed_variance + q_velocity;

        const float variance = tracker->measurement_variance[axis];
        const float inverse_innovation_variance = 1.0f / (p_position + variance);
        const float position_gain = p_position * inverse_innovation_variance;
        const float speed_gain = p_cross * inverse_innovation_variance;
        const float innovation = measurement[axis] - position;
        tracker->position[axis] = position + position_gain * innovation;
        tracker->velocity[axis] += speed_gain * innovation;

        /* H=[1,0] 时用 R·K 更新位置项，避免以 1-K 相减丢失精度。 */
        tracker->position_variance[axis] = variance * position_gain;
        tracker->position_velocity_covariance[axis] = variance * speed_gain;
        /* 浮点舍入可能使协方差行列式略小于零；保留位置与交叉项并抬高速度方差。 */
        const float min_speed_variance = tracker->position_velocity_covariance[axis] *
            tracker->position_velocity_covariance[axis] / tracker->position_variance[axis];
        tracker->velocity_variance[axis] = fmaxf(min_speed_variance, fmaf(-speed_gain, p_cross, p_speed));
    }
}
