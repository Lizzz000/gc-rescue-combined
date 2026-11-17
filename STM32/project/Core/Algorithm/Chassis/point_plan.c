#include "point_plan.h"
#include <math.h>
#include <stddef.h>

#define TWO_PI 6.28318530717958647692
#define PI 3.14159265358979323846

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

uint8_t Point_Plan_Step(const Point_Plan_t *plan, const Point_Plan_Config_t *config, const double pose[3],
                        Point_Plan_Output_t *output)
{
    if (output == NULL)
    {
        return 0U;
    }
    *output = (Point_Plan_Output_t){0};
    if (plan == NULL || !plan->valid || config == NULL || pose == NULL)
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
        if (!isfinite(pose[i]))
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
    const float error = Wrap((double)plan->goal_yaw_rad - yaw);
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
        output->vx_mps = (cosine * direction_x + sine * direction_y) * speed;
        output->vy_mps = (-sine * direction_x + cosine * direction_y) * speed;
    }
    if (fabsf(error) > config->yaw_tolerance_rad)
    {
        output->wz_rad_s = copysignf(Brake_Speed(fabsf(error) - 0.5f * config->yaw_tolerance_rad,
                                                 config->angular_deceleration_rad_s2, config->latency_s,
                                                 config->angular_limit_rad_s),
                                     error);
    }
    return 1U;
}
