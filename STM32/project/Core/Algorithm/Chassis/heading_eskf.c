/* 按 Solà (2017), arXiv:1711.02508 的预测/误差注入/复位方法独立实现。
 * SO(2) 为交换群，yaw 注入采用 wrap(yaw+delta)，复位 Jacobian 恰为 I。
 * 不是完整惯导 ESKF；不估计 roll/pitch、加速度计偏置或绝对位置。
 * AHRS YAW 与 gyro 同源：保守噪声/方差下限只限制过度自信，不能消除相关性。 */
#include "heading_eskf.h"
#include <math.h>
#include <stddef.h>

#define TWO_PI 6.28318530717958647692f
#define YAW_VARIANCE 0.00030461742f       /* (1 degree)^2 at 100 Hz */
#define GYRO_NOISE_DENSITY_SQ 0.0004f    /* (0.02 rad/sqrt(s))^2 */
#define BIAS_WALK_DENSITY_SQ 0.00000025f
#define BIAS_LIMIT 0.05f                 /* rad/s; geometry error is not gyro bias */

static uint8_t Valid_State(const Heading_ESKF_t *s)
{
    return isfinite(s->yaw_rad) && isfinite(s->bias_rad_s) &&
        isfinite(s->previous_gyro_rad_s) && isfinite(s->p_yaw) &&
        isfinite(s->p_yaw_bias) && isfinite(s->p_bias) &&
        s->p_yaw > 0.0f && s->p_bias > 0.0f &&
        s->p_yaw * s->p_bias >= s->p_yaw_bias * s->p_yaw_bias;
}

uint8_t Heading_ESKF_Init(Heading_ESKF_t *s, float yaw, float gyro)
{
    if (s == NULL) return 0U;
    *s = (Heading_ESKF_t){0};
    if (!isfinite(yaw) || !isfinite(gyro) || fabsf(gyro) > 8.0f) return 0U;
    s->yaw_rad = remainderf(yaw, TWO_PI);
    s->previous_gyro_rad_s = gyro;
    s->p_yaw = YAW_VARIANCE;
    s->p_bias = 0.0004f;
    s->initialized = 1U;
    return 1U;
}

/* 标量观测 H=[h0,h1] 的 Joseph 更新；立即注入后误差均值归零。
 * 显式展开2x2运算，避免通用矩阵库、求逆和堆内存。 */
static float Correct(Heading_ESKF_t *s, float h0, float h1, float residual, float r)
{
    const float a = s->p_yaw, b = s->p_yaw_bias, c = s->p_bias;
    const float ph0 = a*h0 + b*h1, ph1 = b*h0 + c*h1;
    const float inverse_s = 1.0f / (h0*ph0 + h1*ph1 + r);
    const float k0 = ph0*inverse_s, k1 = ph1*inverse_s;
    const float m00 = 1.0f-k0*h0, m01 = -k0*h1;
    const float m10 = -k1*h0, m11 = 1.0f-k1*h1;
    s->p_yaw = m00*m00*a + 2.0f*m00*m01*b + m01*m01*c + k0*k0*r;
    s->p_yaw_bias = m00*m10*a + (m00*m11+m01*m10)*b + m01*m11*c + k0*k1*r;
    s->p_bias = m10*m10*a + 2.0f*m10*m11*b + m11*m11*c + k1*k1*r;
    s->yaw_rad = remainderf(s->yaw_rad + k0*residual, TWO_PI);
    s->bias_rad_s = fmaxf(-BIAS_LIMIT, fminf(BIAS_LIMIT, s->bias_rad_s + k1*residual));
    /* 只增加对角线，维持半正定；不能当作独立绝对航向精度的置信区间。 */
    s->p_yaw = fmaxf(s->p_yaw, 0.0000030461742f); /* 0.1 degree floor */
    s->p_bias = fmaxf(s->p_bias, 1e-8f);
    return h0 != 0.0f ? k0 : k1;
}

uint8_t Heading_ESKF_Update(Heading_ESKF_t *s, float yaw, float gyro,
                            float wheel_delta, float dt)
{
    if (s == NULL) return 0U;
    if (!s->initialized || !Valid_State(s) || !isfinite(yaw) || !isfinite(gyro) ||
        fabsf(gyro) > 8.0f || !isfinite(dt) || dt <= 0.0f || dt > 0.05f)
    {
        s->initialized = 0U;
        return 0U;
    }
    const float rate = 0.5f * (s->previous_gyro_rad_s + gyro);
    s->previous_gyro_rad_s = gyro;
    s->yaw_rad = remainderf(s->yaw_rad + (rate-s->bias_rad_s)*dt, TWO_PI);
    /* F=[[1,-dt],[0,1]], continuous white gyro noise + bias random walk. */
    const float dt2 = dt*dt;
    s->p_yaw += -2.0f*dt*s->p_yaw_bias + dt2*s->p_bias +
        GYRO_NOISE_DENSITY_SQ*dt + BIAS_WALK_DENSITY_SQ*dt2*dt/3.0f;
    s->p_yaw_bias -= dt*s->p_bias + 0.5f*BIAS_WALK_DENSITY_SQ*dt2;
    s->p_bias += BIAS_WALK_DENSITY_SQ*dt;
    s->wheel_bias_gain = 0.0f;
    if (isfinite(wheel_delta))
    {
        const float residual = rate - wheel_delta/dt - s->bias_rad_s;
        /* 轮径/安装/轮滑误差保留较大 R；不把不一致强行吸收到 gyro bias。
         * 连续衰减权重，严重不一致仅丢弃轮观测，不影响有效 YAW。 */
        const float tolerance = 0.15f + 0.10f*fabsf(rate);
        if (isfinite(residual) && fabsf(residual) <= 1.5f)
        {
            const float sigma = 0.5f + 0.15f*fabsf(rate);
            const float normalized = residual/tolerance;
            const float r = sigma*sigma*(1.0f+normalized*normalized)*(.01f/dt);
            s->wheel_bias_gain = Correct(s, 0.0f, 1.0f, residual, r);
        }
    }
    /* YAW 最后更新，主导最终航向；角差始终取最短弧，跨±pi不突跳。
     * 观测噪声按采样率缩放，避免同源重复观测在高采样率下过度加权。 */
    const float innovation = remainderf(yaw-s->yaw_rad, TWO_PI);
    s->yaw_gain = Correct(s, 1.0f, 0.0f, innovation, YAW_VARIANCE*(.01f/dt));
    if (!Valid_State(s))
    {
        s->initialized = 0U;
        return 0U;
    }
    return 1U;
}
