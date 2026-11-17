/* 平面圆弧积分方法参考 ros-controls/ros2_controllers 的 omni_wheel_drive_controller：
 * https://github.com/ros-controls/ros2_controllers/blob/62af79e4788adaf4718491968c3f8fcc632af265/omni_wheel_drive_controller/src/odometry.cpp
 * 参考源采用 Apache-2.0。本文件按本车接口独立实现：复用三轮闭式正解，
 * 以 IMU 航向增量及前后轮速均值积分，不依赖 ROS、Eigen 或动态内存。
 */
#include "chassis_odometry.h"
#include "motor_chassis.h"
#include <math.h>
#include <stddef.h>

#define ODOMETRY_TWO_PI 6.28318530717958647692f

/** @brief 初始化独立里程计；不触发传感器采样或电机动作。 */
void Odometry_Init(Odometry_State_t *state)
{
    if (state != NULL)
        *state = (Odometry_State_t){0};
}

/** @brief 丢弃速度历史，防止恢复后跨过缺测区间补算位移。 */
void Odometry_Invalidate(Odometry_State_t *state)
{
    if (state == NULL)
        return;
    state->data.valid = state->initialized = 0U;
    state->data.vx_mps = state->data.vy_mps = state->data.wheel_wz_rad_s = 0.0f;
    state->previous_vx_mps = state->previous_vy_mps = state->previous_yaw_rad = 0.0f;
}

/** @brief 在确认输入有效后一次提交本帧速度、航向和积分时基。 */
static void Accept_Sample(Odometry_State_t *state, const float body[3],
                          float yaw_rad, uint32_t now_us)
{
    state->data.yaw_rad = state->previous_yaw_rad = yaw_rad;
    state->data.vx_mps = state->previous_vx_mps = body[0];
    state->data.vy_mps = state->previous_vy_mps = body[1];
    state->data.wheel_wz_rad_s = body[2];
    state->data.sample_time_us = now_us;
    state->data.valid = state->initialized = 1U;
}

/** @brief 以反馈组为步长计算轮速平移和 IMU 航向里程计，约束见头文件。 */
uint8_t Odometry_Update(Odometry_State_t *state, const float wheel_rad_s[3],
                        float yaw_rad, uint32_t now_us)
{
    if (state == NULL)
        return 0U;
    if (wheel_rad_s == NULL || !isfinite(yaw_rad))
    {
        Odometry_Invalidate(state);
        return 0U;
    }
    for (unsigned i = 0U; i < 3U; ++i)
        if (!isfinite(wheel_rad_s[i]))
        {
            Odometry_Invalidate(state);
            return 0U;
        }
    float body[3];
    Motor_Chassis_Forward(wheel_rad_s, body);
    for (unsigned i = 0U; i < 3U; ++i)
        if (!isfinite(body[i]))
        {
            Odometry_Invalidate(state);
            return 0U;
        }
    yaw_rad = remainderf(yaw_rad, ODOMETRY_TWO_PI);
    if (!state->initialized)
    {
        Accept_Sample(state, body, yaw_rad, now_us);
        return 1U;
    }

    const uint32_t elapsed_us = now_us - state->data.sample_time_us;
    if (elapsed_us == 0U)
        return 0U;
    if (elapsed_us > CHASSIS_ODOMETRY_MAX_INTERVAL_US)
    {
        Odometry_Invalidate(state);
        return 0U;
    }
    const float half_turn = 0.5f * remainderf(yaw_rad - state->previous_yaw_rad,
                                             ODOMETRY_TWO_PI);
    /* sin(x)/x 在小角处用展开式；中间航向乘此系数等价于常转率圆弧积分。 */
    const float square = half_turn * half_turn;
    const float arc_scale = fabsf(half_turn) < 0.001f ?
                            1.0f - square / 6.0f + square * square / 120.0f :
                            sinf(half_turn) / half_turn;
    const float middle_yaw = state->previous_yaw_rad + half_turn;
    const float cosine = cosf(middle_yaw), sine = sinf(middle_yaw);
    const double duration = elapsed_us * 0.000001;
    const double dx = (0.5 * state->previous_vx_mps + 0.5 * body[0]) * duration * arc_scale;
    const double dy = (0.5 * state->previous_vy_mps + 0.5 * body[1]) * duration * arc_scale;
    const double x = state->data.x_m + dx * cosine - dy * sine;
    const double y = state->data.y_m + dx * sine + dy * cosine;
    if (!isfinite(x) || !isfinite(y))
    {
        Odometry_Invalidate(state);
        return 0U;
    }
    state->data.x_m = x;
    state->data.y_m = y;
    Accept_Sample(state, body, yaw_rad, now_us);
    return 1U;
}
