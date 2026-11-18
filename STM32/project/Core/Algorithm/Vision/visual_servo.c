#include "visual_servo.h"
#include <math.h>
#include <stddef.h>

void Visual_Servo_Update(Visual_Servo_t *state, const Visual_Servo_Config_t *config,
                         const Visual_Servo_Estimate_t *estimate, const float raw[2], const float goal[2],
                         const float tolerance[2], float seconds)
{
    if (state == NULL)
    {
        return;
    }
    if (config == NULL || estimate == NULL || raw == NULL || goal == NULL || tolerance == NULL ||
        !isfinite(seconds) || seconds < 0.0f || seconds >= 0.25f ||
        !isfinite(config->max_speed_mps) || config->max_speed_mps <= 0.0f ||
        !isfinite(config->acceleration_mps2) || config->acceleration_mps2 <= 0.0f ||
        !isfinite(config->deceleration_mps2) || config->deceleration_mps2 <= 0.0f ||
        !isfinite(config->latency_s) || config->latency_s < 0.0f)
    {
        *state = (Visual_Servo_t){0};
        return;
    }
    /* 首帧给一个控制拍的起步额度，不把未知间隔当成满速。 */
    const float dt = seconds > 0.0f ? seconds : 0.005f;
    float desired[2] = {0};
    for (unsigned axis = 0U; axis < 2U; ++axis)
    {
        if (!isfinite(raw[axis]) || !isfinite(goal[axis]) || !isfinite(tolerance[axis]) ||
            tolerance[axis] <= 0.0f || !isfinite(config->response_q_per_m[axis]) ||
            config->response_q_per_m[axis] <= 0.0f || !isfinite(estimate->position[axis]) ||
            !isfinite(estimate->velocity[axis]) || !isfinite(state->speed[axis]))
        {
            *state = (Visual_Servo_t){0};
            return;
        }
        const float previous = state->speed[axis];
        if (!(estimate->accepted_mask & (1U << axis)))
        {
            desired[axis] =
                copysignf(fmaxf(0.0f, fabsf(previous) - config->deceleration_mps2 * dt), previous);
            continue;
        }
        const float error = goal[axis] - raw[axis];
        if (fabsf(error) <= tolerance[axis])
        {
            continue;
        }
        const float direction = copysignf(1.0f, error);
        if (previous * direction < 0.0f)
        {
            continue; /* 换向先归零，底盘逐拍斜坡负责真实制动。 */
        }
        const float filtered = direction * (goal[axis] - estimate->position[axis]);
        /* 目标取半窗内侧，避免以窗外余量为 P 输入时无限趋近窗沿。 */
        const float remaining = fmaxf(0.0f, fminf(fabsf(error), filtered) - 0.5f * tolerance[axis]);
        float response = config->response_q_per_m[axis];
        if (fabsf(previous) >= 0.03f)
        {
            /* 图像实际收敛更快时提前制动；不降低标定初值，也不把它当标定结果保存。 */
            const float observed = fmaxf(0.0f, direction * estimate->velocity[axis]) / fabsf(previous);
            response = fmaxf(response, fminf(4.0f * response, observed));
        }
        const float a = config->deceleration_mps2;
        const float delay = config->latency_s + dt;
        const float at = a * delay;
        /* 正根的有理化形式，避免近零距离时 sqrt-at 消减。 */
        const float distance = remaining / response;
        const float cap = 2.0f * a * distance / (sqrtf(at * at + 2.0f * a * distance) + at + 1e-9f);
        desired[axis] = direction * fminf(config->max_speed_mps, cap);
    }
    const float speed = hypotf(desired[0], desired[1]);
    if (speed > config->max_speed_mps)
    {
        for (unsigned axis = 0U; axis < 2U; ++axis)
        {
            desired[axis] *= config->max_speed_mps / speed;
        }
    }
    /* 两轴共同限制加速量；减速立即降低目标，避免叠加两个减速滤波的迟滞。 */
    float increase[2];
    for (unsigned axis = 0U; axis < 2U; ++axis)
    {
        increase[axis] = fmaxf(0.0f, fabsf(desired[axis]) - fabsf(state->speed[axis]));
    }
    const float norm = hypotf(increase[0], increase[1]);
    const float scale = norm > config->acceleration_mps2 * dt ? config->acceleration_mps2 * dt / norm : 1.0f;
    for (unsigned axis = 0U; axis < 2U; ++axis)
    {
        state->speed[axis] = copysignf(fabsf(desired[axis]) - increase[axis] * (1.0f - scale), desired[axis]);
    }
}
