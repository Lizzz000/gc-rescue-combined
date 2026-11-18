/* 平面圆弧积分方法参考 ros-controls/ros2_controllers 的 omni_wheel_drive_controller：
 * https://github.com/ros-controls/ros2_controllers/blob/62af79e4788adaf4718491968c3f8fcc632af265/omni_wheel_drive_controller/src/odometry.cpp
 * 参考源采用 Apache-2.0。本文件按本车接口独立实现：复用三轮闭式正解，
 * 以 IMU 航向增量及前后两帧速度均值积分，不依赖 ROS、Eigen 或动态内存。
 * 位置增量入口参考 WPILib v2026.2.2 Odometry::Update/Pose2d::Exp：
 * https://github.com/wpilibsuite/allwpilib/tree/7ca35e5678cf32caec6a1a866ca51d0136c4c398/wpimath
 * 按三轮MIT位置回绕与现有IMU接口适配纯C，BSD许可见同目录LICENSE-WPILib.txt。
 */
#include "chassis_odometry.h"
#include "motor_chassis.h"
#include <math.h>
#include <stddef.h>

enum
{
    ODOMETRY_INPUT_NONE,
    ODOMETRY_INPUT_VELOCITY,
    ODOMETRY_INPUT_POSITION,
    ODOMETRY_INPUT_POSITION_ESKF,
    ODOMETRY_INPUT_POSITION_ESKF_TIMED
};

#define ODOMETRY_TWO_PI 6.28318530717958647692f

/** @brief 初始化独立里程计；不触发传感器采样或电机动作。 */
void Odometry_Init(Odometry_State_t *state)
{
    if (state != NULL)
    {
        *state = (Odometry_State_t){0};
    }
}

/** @brief 丢弃速度历史，防止恢复后跨过缺测区间补算位移。 */
void Odometry_Invalidate(Odometry_State_t *state)
{
    if (state == NULL)
    {
        return;
    }
    state->input_mode = 0U;
    state->initialized = 0U;
    state->data.valid = 0U;
    state->heading.initialized = 0U;
    state->data.wheel_wz_rad_s = 0.0f;
    state->data.vy_mps = 0.0f;
    state->data.vx_mps = 0.0f;
    state->previous_yaw_rad = 0.0f;
    state->previous_vy_mps = 0.0f;
    state->previous_vx_mps = 0.0f;
}

/** @brief 在确认输入有效后一次提交本帧速度、航向和积分时基。 */
static void Accept_Sample(Odometry_State_t *state, const float body[3], float yaw_rad, uint32_t now_us)
{
    state->previous_yaw_rad = yaw_rad;
    state->data.yaw_rad = yaw_rad;
    state->previous_vx_mps = body[0];
    state->data.vx_mps = body[0];
    state->previous_vy_mps = body[1];
    state->data.vy_mps = body[1];
    state->data.wheel_wz_rad_s = body[2];
    state->data.sample_time_us = now_us;
    state->initialized = 1U;
    state->data.valid = 1U;
}

/* SE(2)指数映射的中间航向形式；dx/dy是本采样内车体位移，不是速度。 */
static uint8_t Integrate_Displacement(Odometry_State_t *state, double dx, double dy, float yaw_rad)
{
    const float half_turn = 0.5f * remainderf(yaw_rad - state->previous_yaw_rad, ODOMETRY_TWO_PI);
    const float square = half_turn * half_turn;
    float arc_scale;
    if (fabsf(half_turn) < 0.001f)
    {
        arc_scale = 1.0f - square / 6.0f + square * square / 120.0f;
    }
    else
    {
        arc_scale = sinf(half_turn) / half_turn;
    }
    const float middle_yaw = state->previous_yaw_rad + half_turn;
    const float cosine = cosf(middle_yaw);
    const float sine = sinf(middle_yaw);
    const double x = state->data.x_m + arc_scale * (dx * cosine - dy * sine);
    const double y = state->data.y_m + arc_scale * (dx * sine + dy * cosine);
    if (!isfinite(x) || !isfinite(y))
    {
        Odometry_Invalidate(state);
        return 0U;
    }
    state->data.x_m = x;
    state->data.y_m = y;
    return 1U;
}

/** @brief 以反馈组为步长计算轮速平移和 IMU 航向里程计，约束见头文件。 */
uint8_t Odometry_Update(Odometry_State_t *state, const float wheel_rad_s[3], float yaw_rad, uint32_t now_us)
{
    if (state == NULL)
    {
        return 0U;
    }
    if (wheel_rad_s == NULL || !isfinite(yaw_rad))
    {
        Odometry_Invalidate(state);
        return 0U;
    }
    for (unsigned i = 0U; i < 3U; ++i)
    {
        if (!isfinite(wheel_rad_s[i]))
        {
            Odometry_Invalidate(state);
            return 0U;
        }
    }
    float body[3];
    Motor_Chassis_Forward(wheel_rad_s, body);
    for (unsigned i = 0U; i < 3U; ++i)
    {
        if (!isfinite(body[i]))
        {
            Odometry_Invalidate(state);
            return 0U;
        }
    }
    yaw_rad = remainderf(yaw_rad, ODOMETRY_TWO_PI);
    if (state->input_mode != ODOMETRY_INPUT_VELOCITY)
    {
        Odometry_Invalidate(state);
    }
    state->input_mode = ODOMETRY_INPUT_VELOCITY;
    if (!state->initialized)
    {
        Accept_Sample(state, body, yaw_rad, now_us);
        return 1U;
    }

    const uint32_t elapsed_us = now_us - state->data.sample_time_us;
    if (elapsed_us == 0U)
    {
        return 0U;
    }
    if (elapsed_us > CHASSIS_ODOMETRY_MAX_INTERVAL_US)
    {
        Odometry_Invalidate(state);
        return 0U;
    }
    const double duration = elapsed_us * 0.000001;
    const double dx = (0.5 * state->previous_vx_mps + 0.5 * body[0]) * duration;
    const double dy = (0.5 * state->previous_vy_mps + 0.5 * body[1]) * duration;
    if (!Integrate_Displacement(state, dx, dy, yaw_rad))
    {
        return 0U;
    }
    Accept_Sample(state, body, yaw_rad, now_us);
    return 1U;
}

static uint8_t Update_Position(Odometry_State_t *state, const float wheel_position_rad[3],
                               const float wheel_rad_s[3], const uint32_t wheel_time_ms[3],
                               float yaw_rad, float gyro_rad_s, uint32_t now_us,
                               uint8_t use_eskf, uint8_t observe_yaw)
{
    if (state == NULL)
    {
        return 0U;
    }
    if (wheel_position_rad == NULL || wheel_rad_s == NULL || !isfinite(yaw_rad) ||
        (use_eskf && (state->heading.yaw_fault || !isfinite(gyro_rad_s) || fabsf(gyro_rad_s) > 8.0f)))
    {
        Odometry_Invalidate(state);
        return 0U;
    }
    for (unsigned i = 0U; i < 3U; ++i)
    {
        if (!isfinite(wheel_position_rad[i]) || !isfinite(wheel_rad_s[i]) ||
            fabsf(wheel_position_rad[i]) > CHASSIS_ODOMETRY_ENCODER_PERIOD_RAD * 0.5f ||
            fabsf(wheel_rad_s[i]) > CHASSIS_ODOMETRY_MAX_WHEEL_RAD_S)
        {
            Odometry_Invalidate(state);
            return 0U;
        }
    }
    float body[3];
    Motor_Chassis_Forward(wheel_rad_s, body);
    yaw_rad = remainderf(yaw_rad, ODOMETRY_TWO_PI);
    uint8_t input_mode = ODOMETRY_INPUT_POSITION;
    if (use_eskf)
        input_mode = wheel_time_ms ? ODOMETRY_INPUT_POSITION_ESKF_TIMED : ODOMETRY_INPUT_POSITION_ESKF;
    if (state->input_mode != input_mode)
    {
        Odometry_Invalidate(state);
    }
    state->input_mode = input_mode;
    if (state->initialized)
    {
        const uint32_t elapsed_us = now_us - state->data.sample_time_us;
        if (elapsed_us == 0U)
        {
            return 0U;
        }
        if (elapsed_us > CHASSIS_ODOMETRY_MAX_INTERVAL_US)
        {
            Odometry_Invalidate(state);
            return 0U;
        }
        float delta[3];
        float displacement[3];
        for (unsigned i = 0U; i < 3U; ++i)
        {
            float interval_s = elapsed_us * 0.000001f;
            if (wheel_time_ms != NULL)
            {
                const uint32_t wheel_elapsed_ms = wheel_time_ms[i] - state->previous_wheel_time_ms[i];
                if (wheel_elapsed_ms > CHASSIS_ODOMETRY_MAX_INTERVAL_US / 1000U)
                {
                    Odometry_Invalidate(state);
                    return 0U;
                }
                /* 每轮使用自身的接收间隔；补偿毫秒取整和两码位置量化。 */
                interval_s = (wheel_elapsed_ms + 1U) * 0.001f;
            }
            const float max_delta = CHASSIS_ODOMETRY_MAX_WHEEL_RAD_S * interval_s +
                                    2.0f * CHASSIS_ODOMETRY_ENCODER_PERIOD_RAD / 65535.0f;
            delta[i] = remainderf(wheel_position_rad[i] - state->previous_position_rad[i],
                                  CHASSIS_ODOMETRY_ENCODER_PERIOD_RAD);
            if (fabsf(delta[i]) > max_delta)
            {
                Odometry_Invalidate(state);
                return 0U;
            }
        }
        /* 正解是线性映射：输入角增量rad时，输出dx/dy为m、转角为rad。 */
        Motor_Chassis_Forward(delta, displacement);
        if (use_eskf)
        {
            const float dt = elapsed_us * 0.000001f;
            const uint8_t heading_valid = wheel_time_ms != NULL
                ? Heading_ESKF_Update_Observed(&state->heading, yaw_rad, gyro_rad_s, displacement[2], dt, observe_yaw)
                : Heading_ESKF_Update(&state->heading, yaw_rad, gyro_rad_s, displacement[2], dt);
            if (!heading_valid)
            {
                Odometry_Invalidate(state);
                return 0U;
            }
            yaw_rad = state->heading.yaw_rad;
        }
        if (!Integrate_Displacement(state, displacement[0], displacement[1], yaw_rad))
        {
            return 0U;
        }
    }
    else if (use_eskf && (!observe_yaw || !Heading_ESKF_Init(&state->heading, yaw_rad, gyro_rad_s)))
    {
        Odometry_Invalidate(state);
        return 0U;
    }
    for (unsigned i = 0U; i < 3U; ++i)
    {
        state->previous_position_rad[i] = wheel_position_rad[i];
        if (wheel_time_ms != NULL)
            state->previous_wheel_time_ms[i] = wheel_time_ms[i];
    }
    Accept_Sample(state, body, yaw_rad, now_us);
    return 1U;
}

uint8_t Odometry_Update_Position(Odometry_State_t *state, const float position[3], const float wheel[3],
                                 float yaw, uint32_t now_us)
{
    return Update_Position(state, position, wheel, NULL, yaw, 0.0f, now_us, 0U, 1U);
}

uint8_t Odometry_Update_Position_ESKF(Odometry_State_t *state, const float position[3], const float wheel[3],
                                      float yaw, float gyro, uint32_t now_us)
{
    return Update_Position(state, position, wheel, NULL, yaw, gyro, now_us, 1U, 1U);
}

uint8_t Odometry_Update_Position_ESKF_Timed(Odometry_State_t *state, const float position[3],
                                            const float wheel[3], const uint32_t wheel_time_ms[3],
                                            float yaw, float gyro, uint32_t now_us, uint8_t observe_yaw)
{
    if (wheel_time_ms == NULL)
    {
        Odometry_Invalidate(state);
        return 0U;
    }
    return Update_Position(state, position, wheel, wheel_time_ms, yaw, gyro, now_us, 1U, observe_yaw);
}
