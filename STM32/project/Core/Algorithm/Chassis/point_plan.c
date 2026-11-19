#include "point_plan.h"
#include <math.h>
#include <stddef.h>

#define TWO_PI 6.28318530717958647692
#define PI 3.14159265358979323846
#define VELOCITY_DAMPING 0.5f

/* 正常INS航向无需调用软件double余数，跨边界只加减一圈。 */
static float Wrap(double angle)
{
    if (angle > PI && angle <= 3.0 * PI)
    {
        angle -= TWO_PI;
    }
    else if (angle < -PI && angle >= -3.0 * PI)
    {
        angle += TWO_PI;
    }
    else if (angle > PI || angle < -PI)
    {
        angle = remainder(angle, TWO_PI);
    }
    return (float)angle;
}

uint8_t Point_Plan_Init(Point_Plan_t *plan, const double start[3], const double goal[3])
{
    if (plan == NULL)
    {
        return 0U;
    }
    *plan = (Point_Plan_t){0};
    if (start == NULL || goal == NULL)
    {
        return 0U;
    }
    for (unsigned i = 0U; i < 3U; ++i)
    {
        if (!isfinite(start[i]) || !isfinite(goal[i]))
        {
            return 0U;
        }
    }
    const double length = hypot(goal[0] - start[0], goal[1] - start[1]);
    if (!isfinite(length))
    {
        return 0U;
    }
    for (unsigned i = 0U; i < 2U; ++i)
    {
        plan->start[i] = start[i];
        plan->goal[i] = goal[i];
    }
    plan->length_m = length;
    plan->goal_yaw_rad = Wrap(goal[2]);
    plan->yaw_delta_rad = Wrap((double)Wrap(goal[2]) - Wrap(start[2]));
    plan->remaining_yaw_rad = plan->yaw_delta_rad;
    plan->valid = 1U;
    return 1U;
}

/* 解 v*t+v²/(2a)=d；采用有理化正根避免近零时消减。 */
static float Brake_Speed(double distance, float acceleration, float latency, float limit)
{
    const float at = acceleration * latency;
    const float cruise_distance = limit * latency + limit * (limit / acceleration) * 0.5f;
    if (distance >= cruise_distance)
    {
        return limit; /* 巡航段无需开平方。近段用M4单精度FPU，世界位移仍保留double。 */
    }
    const float twice_ad = 2.0f * acceleration * (float)distance;
    return fminf(limit, twice_ad / (sqrtf(at * at + twice_ad) + at + 1e-12f));
}

/* 速度反馈只削减已经偏快的接近速度，保留低速前进量以跨过摩擦死区。 */
static float Damp_Approach(float reference, float approaching)
{
    return fmaxf(0.25f * reference, reference - VELOCITY_DAMPING * fmaxf(0.0f, approaching - reference));
}

uint8_t Point_Plan_Step(Point_Plan_t *plan, const Point_Plan_Config_t *config, const double pose[3],
                        const float measured_body[3], Point_Plan_Output_t *output)
{
    if (output == NULL)
    {
        return 0U;
    }
    *output = (Point_Plan_Output_t){0};
    if (plan == NULL || !plan->valid || config == NULL || pose == NULL || measured_body == NULL)
    {
        return 0U;
    }
    const float positive[] = {
        config->linear_limit_mps,
        config->angular_limit_rad_s,
        config->linear_deceleration_mps2,
        config->angular_deceleration_rad_s2,
        config->position_tolerance_m,
        config->yaw_tolerance_rad
    };
    for (unsigned i = 0U; i < sizeof(positive) / sizeof(positive[0]); ++i)
    {
        if (!isfinite(positive[i]) || positive[i] <= 0.0f)
        {
            return 0U;
        }
    }
    if (!isfinite(config->latency_s) || config->latency_s < 0.0f)
    {
        return 0U;
    }
    for (unsigned i = 0U; i < 3U; ++i)
    {
        if (!isfinite(pose[i]) || !isfinite(measured_body[i]))
        {
            return 0U;
        }
    }
    const double dx = plan->goal[0] - pose[0];
    const double dy = plan->goal[1] - pose[1];
    const double distance = hypot(dx, dy);
    if (!isfinite(distance))
    {
        return 0U;
    }
    const float yaw = Wrap(pose[2]);
    const float wrapped_error = Wrap((double)plan->goal_yaw_rad - yaw);
    /* 半圈附近取与上一拍相邻的角度，避免噪声让方向在正负 180° 之间跳。 */
    float error = plan->remaining_yaw_rad + Wrap((double)wrapped_error - plan->remaining_yaw_rad);
    /* 半圈附近两侧路程几乎一样，已有角速度时沿当前方向接入。 */
    if (!plan->yaw_started && fabsf(error) > (float)PI - 0.08726646f &&
        fabsf(measured_body[2]) > 0.15f && error * measured_body[2] < 0.0f)
        error += copysignf((float)TWO_PI, measured_body[2]);
    plan->remaining_yaw_rad = error;
    plan->yaw_started = 1U;
    output->remaining_m = distance;
    output->yaw_error_rad = error;
    output->arrived = distance <= config->position_tolerance_m && fabsf(error) <= config->yaw_tolerance_rad;
    if (distance > config->position_tolerance_m)
    {
        const float speed =
            Brake_Speed(distance - 0.5 * config->position_tolerance_m, config->linear_deceleration_mps2,
                        config->latency_s, config->linear_limit_mps);
        const float cosine = cosf(yaw);
        const float sine = sinf(yaw);
        const float direction_x = (float)(dx / distance);
        const float direction_y = (float)(dy / distance);
        const float measured_x = cosine * measured_body[0] - sine * measured_body[1];
        const float measured_y = sine * measured_body[0] + cosine * measured_body[1];
        const float along = Damp_Approach(speed, direction_x * measured_x + direction_y * measured_y);
        const float cross = -VELOCITY_DAMPING * (-direction_y * measured_x + direction_x * measured_y);
        float vx = direction_x * along - direction_y * cross;
        float vy = direction_y * along + direction_x * cross;
        const float norm = hypotf(vx, vy);
        if (norm > config->linear_limit_mps)
        {
            const float scale = nextafterf(config->linear_limit_mps, 0.0f) / norm;
            vx *= scale;
            vy *= scale;
        }
        output->vx_mps = vx;
        output->vy_mps = vy;
    }
    if (fabsf(error) > config->yaw_tolerance_rad)
    {
        const float speed = Brake_Speed(fabsf(error) - 0.5f * config->yaw_tolerance_rad,
                                        config->angular_deceleration_rad_s2, config->latency_s,
                                        config->angular_limit_rad_s);
        const float direction = copysignf(1.0f, error);
        output->wz_rad_s = direction * Damp_Approach(speed, direction * measured_body[2]);
    }
    return 1U;
}
