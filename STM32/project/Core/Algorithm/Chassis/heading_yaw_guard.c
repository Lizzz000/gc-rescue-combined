#include "heading_yaw_guard.h"
#include <math.h>
#include <stddef.h>

#define TWO_PI 6.28318530717958647692f
#define YAW_JUMP_LIMIT_RAD 0.0872664626f
#define YAW_REJECT_LIMIT 3U

Heading_Yaw_Status_t Heading_Yaw_Guard_Update(Heading_Yaw_Guard_t *guard, float yaw_rad,
                                               float gyro_rad_s, uint32_t sample_time_ms, float dt)
{
    if (guard == NULL || guard->rejected_run >= YAW_REJECT_LIMIT ||
        !isfinite(yaw_rad) || !isfinite(gyro_rad_s) || fabsf(gyro_rad_s) > 8.0f ||
        !isfinite(dt) || dt <= 0.0f || dt > 0.05f)
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

    /* 与 AHRS 同拍使用当前陀螺和积分时长，不能把未积分的缺测间隔算进预测。 */
    const float predicted_yaw = remainderf(guard->yaw_rad + gyro_rad_s * dt, TWO_PI);
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
