/* SE(2) 指数映射适配自 WPILib v2026.2.2 的 Pose2d::Exp。
 * BSD 许可见同目录 LICENSE-WPILib.txt。 */
#include "planar_odometry.h"
#include <math.h>

uint8_t Planar_Odometry_Integrate(double *x_m, double *y_m, double dx, double dy,
                                  float start_yaw, float end_yaw)
{
    const float half_turn = 0.5f * remainderf(end_yaw - start_yaw, 6.28318530717958647692f);
    const float square = half_turn * half_turn;
    const float arc_scale = fabsf(half_turn) < 0.001f
        ? 1.0f - square / 6.0f + square * square / 120.0f : sinf(half_turn) / half_turn;
    const float middle_yaw = start_yaw + half_turn;
    const float cosine = cosf(middle_yaw);
    const float sine = sinf(middle_yaw);
    const double x = *x_m + arc_scale * (dx * cosine - dy * sine);
    const double y = *y_m + arc_scale * (dx * sine + dy * cosine);
    if (!isfinite(x) || !isfinite(y))
        return 0U;
    *x_m = x;
    *y_m = y;
    return 1U;
}
