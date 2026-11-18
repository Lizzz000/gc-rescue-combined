/* 按 Solà (2017), arXiv:1711.02508 的预测/误差注入/复位方法独立实现。
 * SO(2) 为交换群，yaw 注入采用 wrap(yaw+delta)，复位 Jacobian 恰为 I。
 * 不是完整惯导 ESKF；不估计 roll/pitch、加速度计偏置或绝对位置。
 * AHRS YAW 与 gyro 同源：保守噪声/方差下限只限制过度自信，不能消除相关性。 */
#include "heading_eskf.h"
#include <math.h>
#include <stddef.h>

#define TWO_PI 6.28318530717958647692f
#define YAW_JUMP_LIMIT_RAD 0.0872664626f
#define YAW_REJECT_LIMIT 3U
#define YAW_VARIANCE 0.00030461742f   /* 100 Hz 下的航向观测方差：(1°)^2。 */
#define GYRO_NOISE_DENSITY_SQ 0.0004f /* (0.02 rad/sqrt(s))^2 */
#define BIAS_WALK_DENSITY_SQ 0.00000025f
#define BIAS_LIMIT 0.05f /* 单位 rad/s；避免将几何误差吸收为陀螺零偏。 */

Heading_Yaw_Status_t Heading_Yaw_Guard_Update(Heading_Yaw_Guard_t *guard, float yaw_rad,
                                               float gyro_rad_s, uint32_t sample_time_ms)
{
    if (guard == NULL || guard->rejected_run >= YAW_REJECT_LIMIT ||
        !isfinite(yaw_rad) || !isfinite(gyro_rad_s) || fabsf(gyro_rad_s) > 8.0f)
        return HEADING_YAW_INVALID;

    yaw_rad = remainderf(yaw_rad, TWO_PI);
    if (guard->status == HEADING_YAW_INVALID)
    {
        guard->yaw_rad = yaw_rad;
        guard->previous_gyro_rad_s = gyro_rad_s;
        guard->sample_time_ms = sample_time_ms;
        guard->status = HEADING_YAW_OBSERVED;
        return guard->status;
    }

    const uint32_t elapsed_ms = sample_time_ms - guard->sample_time_ms;
    if (elapsed_ms == 0U || elapsed_ms > INT32_MAX)
        return guard->status;

    /* 跳点期间保留陀螺相对航向；缺测的长时间段不外推。 */
    float predicted_yaw = guard->yaw_rad;
    if (elapsed_ms <= 50U)
        predicted_yaw = remainderf(predicted_yaw + 0.0005f * (float)elapsed_ms *
                                   (guard->previous_gyro_rad_s + gyro_rad_s), TWO_PI);
    guard->previous_gyro_rad_s = gyro_rad_s;
    guard->sample_time_ms = sample_time_ms;
    if (fabsf(remainderf(yaw_rad - predicted_yaw, TWO_PI)) <= YAW_JUMP_LIMIT_RAD)
    {
        guard->yaw_rad = yaw_rad;
        guard->rejected_run = 0U;
        guard->status = HEADING_YAW_OBSERVED;
    }
    else
    {
        guard->yaw_rad = predicted_yaw;
        ++guard->rejected_run;
        guard->status = guard->rejected_run >= YAW_REJECT_LIMIT ? HEADING_YAW_INVALID : HEADING_YAW_PREDICTED;
    }
    return guard->status;
}

static uint8_t Valid_State(const Heading_ESKF_t *state)
{
    return isfinite(state->yaw_rad) && isfinite(state->bias_rad_s) && isfinite(state->previous_gyro_rad_s) &&
           isfinite(state->p_yaw) && isfinite(state->p_yaw_bias) && isfinite(state->p_bias) &&
           state->p_yaw > 0.0f && state->p_bias > 0.0f &&
           state->p_yaw * state->p_bias >= state->p_yaw_bias * state->p_yaw_bias;
}

uint8_t Heading_ESKF_Init(Heading_ESKF_t *state, float yaw_rad, float gyro_rad_s)
{
    if (state == NULL)
    {
        return 0U;
    }
    *state = (Heading_ESKF_t){0};
    if (!isfinite(yaw_rad) || !isfinite(gyro_rad_s) || fabsf(gyro_rad_s) > 8.0f)
    {
        return 0U;
    }
    state->yaw_rad = remainderf(yaw_rad, TWO_PI);
    state->previous_gyro_rad_s = gyro_rad_s;
    state->p_yaw = YAW_VARIANCE;
    state->p_bias = 0.0004f;
    state->initialized = 1U;
    return 1U;
}

/* 标量观测 H=[h0,h1] 的 Joseph 更新；立即注入后误差均值归零。
 * 显式展开2x2运算，避免通用矩阵库、求逆和堆内存。 */
static float Correct(Heading_ESKF_t *state, float h0, float h1, float residual, float measurement_variance)
{
    const float p_yaw = state->p_yaw;
    const float p_yaw_bias = state->p_yaw_bias;
    const float p_bias = state->p_bias;
    const float ph0 = p_yaw * h0 + p_yaw_bias * h1;
    const float ph1 = p_yaw_bias * h0 + p_bias * h1;
    const float inverse_innovation_variance = 1.0f / (h0 * ph0 + h1 * ph1 + measurement_variance);
    const float yaw_gain = ph0 * inverse_innovation_variance;
    const float bias_gain = ph1 * inverse_innovation_variance;
    const float m00 = 1.0f - yaw_gain * h0;
    const float m01 = -yaw_gain * h1;
    const float m10 = -bias_gain * h0;
    const float m11 = 1.0f - bias_gain * h1;
    state->p_yaw = m00 * m00 * p_yaw + 2.0f * m00 * m01 * p_yaw_bias + m01 * m01 * p_bias +
                   yaw_gain * yaw_gain * measurement_variance;
    state->p_yaw_bias = m00 * m10 * p_yaw + (m00 * m11 + m01 * m10) * p_yaw_bias + m01 * m11 * p_bias +
                        yaw_gain * bias_gain * measurement_variance;
    state->p_bias = m10 * m10 * p_yaw + 2.0f * m10 * m11 * p_yaw_bias + m11 * m11 * p_bias +
                    bias_gain * bias_gain * measurement_variance;
    state->yaw_rad = remainderf(state->yaw_rad + yaw_gain * residual, TWO_PI);
    state->bias_rad_s = fmaxf(-BIAS_LIMIT, fminf(BIAS_LIMIT, state->bias_rad_s + bias_gain * residual));
    /* 只增加对角线，维持半正定；不能当作独立绝对航向精度的置信区间。 */
    state->p_yaw = fmaxf(state->p_yaw, 0.0000030461742f); /* 航向标准差下限为 0.1°。 */
    state->p_bias = fmaxf(state->p_bias, 1e-8f);
    return h0 != 0.0f ? yaw_gain : bias_gain;
}

static uint8_t Update_Heading(Heading_ESKF_t *state, float yaw_rad, float gyro_rad_s,
                               float wheel_delta_rad, float dt, uint8_t observe_yaw, uint8_t check_jump)
{
    if (state == NULL)
    {
        return 0U;
    }
    if (!state->initialized || !Valid_State(state) || !isfinite(yaw_rad) || !isfinite(gyro_rad_s) ||
        fabsf(gyro_rad_s) > 8.0f || !isfinite(dt) || dt <= 0.0f || dt > 0.05f)
    {
        state->initialized = 0U;
        return 0U;
    }
    const float mean_gyro_rad_s = 0.5f * (state->previous_gyro_rad_s + gyro_rad_s);
    state->previous_gyro_rad_s = gyro_rad_s;
    state->yaw_rad = remainderf(state->yaw_rad + (mean_gyro_rad_s - state->bias_rad_s) * dt, TWO_PI);
    /* F=[[1,-dt],[0,1]]；传播连续陀螺白噪声与偏置随机游走。 */
    const float dt2 = dt * dt;
    state->p_yaw += -2.0f * dt * state->p_yaw_bias + dt2 * state->p_bias + GYRO_NOISE_DENSITY_SQ * dt +
                    BIAS_WALK_DENSITY_SQ * dt2 * dt / 3.0f;
    state->p_yaw_bias -= dt * state->p_bias + 0.5f * BIAS_WALK_DENSITY_SQ * dt2;
    state->p_bias += BIAS_WALK_DENSITY_SQ * dt;
    state->wheel_bias_gain = 0.0f;
    if (isfinite(wheel_delta_rad))
    {
        const float residual = mean_gyro_rad_s - wheel_delta_rad / dt - state->bias_rad_s;
        /* 轮径/安装/轮滑误差保留较大 R；不把不一致强行吸收到 gyro bias。
         * 连续衰减权重，严重不一致仅丢弃轮观测，不影响有效 YAW。 */
        const float tolerance = 0.15f + 0.10f * fabsf(mean_gyro_rad_s);
        if (isfinite(residual) && fabsf(residual) <= 1.5f)
        {
            const float sigma = 0.5f + 0.15f * fabsf(mean_gyro_rad_s);
            const float normalized = residual / tolerance;
            const float measurement_variance = sigma * sigma * (1.0f + normalized * normalized) * (.01f / dt);
            state->wheel_bias_gain = Correct(state, 0.0f, 1.0f, residual, measurement_variance);
        }
    }
    /* YAW 最后更新，主导最终航向；角差始终取最短弧，跨±pi不突跳。
     * 观测噪声按采样率缩放，避免同源重复观测在高采样率下过度加权。 */
    state->yaw_gain = 0.0f;
    if (observe_yaw)
    {
        const float innovation = remainderf(yaw_rad - state->yaw_rad, TWO_PI);
        if (!check_jump || fabsf(innovation) <= YAW_JUMP_LIMIT_RAD)
        {
            state->yaw_rejected_run = 0U;
            state->yaw_gain = Correct(state, 1.0f, 0.0f, innovation, YAW_VARIANCE * (.01f / dt));
        }
        else if (++state->yaw_rejected_run >= YAW_REJECT_LIMIT)
        {
            state->yaw_fault = 1U;
            state->initialized = 0U;
            return 0U;
        }
    }
    if (!Valid_State(state))
    {
        state->initialized = 0U;
        return 0U;
    }
    return 1U;
}

uint8_t Heading_ESKF_Update_Observed(Heading_ESKF_t *state, float yaw_rad, float gyro_rad_s,
                                     float wheel_delta_rad, float dt, uint8_t observe_yaw)
{
    return Update_Heading(state, yaw_rad, gyro_rad_s, wheel_delta_rad, dt, observe_yaw, 0U);
}

uint8_t Heading_ESKF_Update(Heading_ESKF_t *state, float yaw_rad, float gyro_rad_s, float wheel_delta_rad,
                            float dt)
{
    return Update_Heading(state, yaw_rad, gyro_rad_s, wheel_delta_rad, dt, 1U, 1U);
}
