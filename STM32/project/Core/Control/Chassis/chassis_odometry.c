/* 平面圆弧积分方法参考 ros-controls/ros2_controllers 的 omni_wheel_drive_controller：
 * https://github.com/ros-controls/ros2_controllers/blob/62af79e4788adaf4718491968c3f8fcc632af265/omni_wheel_drive_controller/src/odometry.cpp
 * 参考源采用 Apache-2.0。本文件按本车接口独立实现：复用三轮闭式正解，
 * 以 IMU 航向增量及前后两帧速度均值积分，不依赖 ROS、Eigen 或动态内存。
 * 位置增量入口参考 WPILib v2026.2.2 Odometry::Update/Pose2d::Exp：
 * https://github.com/wpilibsuite/allwpilib/tree/7ca35e5678cf32caec6a1a866ca51d0136c4c398/wpimath
 * 按三轮MIT位置回绕与现有IMU接口适配纯C，BSD许可见 Algorithm/Chassis/LICENSE-WPILib.txt。
 */
#include "chassis_odometry.h"
#include "motor_chassis.h"
#include "planar_odometry.h"
#include <math.h>
#include <stddef.h>
#include <string.h>

enum
{
    ODOMETRY_INPUT_NONE,
    ODOMETRY_INPUT_VELOCITY,
    ODOMETRY_INPUT_POSITION,
    ODOMETRY_INPUT_POSITION_ESKF,
    ODOMETRY_INPUT_POSITION_ESKF_TIMED
};

_Static_assert(CHASSIS_ODOMETRY_MAX_INTERVAL_US / 1000U < WHEEL_ALIGNMENT_HISTORY_MS,
               "alignment history must cover all valid wheel timestamps");

#define ODOMETRY_TWO_PI 6.28318530717958647692f

/** @brief 初始化独立里程计；不触发传感器采样或电机动作。 */
void Odometry_Init(Odometry_State_t *state)
{
    if (state != NULL)
    {
        memset(state, 0, sizeof(*state));
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
    if (!Planar_Odometry_Integrate(&state->data.x_m, &state->data.y_m, dx, dy, state->previous_yaw_rad, yaw_rad))
    {
        Odometry_Invalidate(state);
        return 0U;
    }
    Accept_Sample(state, body, yaw_rad, now_us);
    return 1U;
}

static uint8_t Get_Wheel_Deltas(const Odometry_State_t *state, const float position[3],
                                const Odometry_Feedback_Timing_t *timing, uint32_t elapsed_us, float delta[3])
{
    for (unsigned i = 0U; i < 3U; ++i)
    {
        float interval_s = elapsed_us * 0.000001f;
        if (timing != NULL)
        {
            const uint32_t wheel_elapsed_ms = timing->wheel_time_ms[i] - state->previous_wheel_time_ms[i];
            if (wheel_elapsed_ms > CHASSIS_ODOMETRY_MAX_INTERVAL_US / 1000U)
                return 0U;
            /* 每轮使用自身的接收间隔，并为毫秒取整留一个 tick。 */
            interval_s = (wheel_elapsed_ms + 1U) * 0.001f;
        }
        const float max_delta = CHASSIS_ODOMETRY_MAX_WHEEL_RAD_S * interval_s +
                                2.0f * CHASSIS_ODOMETRY_ENCODER_PERIOD_RAD / 65535.0f;
        delta[i] = remainderf(position[i] - state->previous_position_rad[i], CHASSIS_ODOMETRY_ENCODER_PERIOD_RAD);
        if (fabsf(delta[i]) > max_delta)
            return 0U;
    }
    return 1U;
}

static uint8_t Update_Position(Odometry_State_t *state, const float wheel_position_rad[3],
                               const float wheel_rad_s[3], const Odometry_Feedback_Timing_t *timing,
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
    uint32_t oldest_age_ms = 0U, newest_age_ms = UINT32_MAX;
    for (unsigned i = 0U; i < 3U; ++i)
    {
        if (!isfinite(wheel_position_rad[i]) || !isfinite(wheel_rad_s[i]) ||
            fabsf(wheel_position_rad[i]) > CHASSIS_ODOMETRY_ENCODER_PERIOD_RAD * 0.5f ||
            fabsf(wheel_rad_s[i]) > CHASSIS_ODOMETRY_MAX_WHEEL_RAD_S)
        {
            Odometry_Invalidate(state);
            return 0U;
        }
        if (timing != NULL)
        {
            const uint32_t age_ms = timing->control_time_ms - timing->wheel_time_ms[i];
            if (age_ms > CHASSIS_ODOMETRY_MAX_INTERVAL_US / 1000U)
            {
                Odometry_Invalidate(state);
                return 0U;
            }
            if (age_ms > oldest_age_ms) oldest_age_ms = age_ms;
            if (age_ms < newest_age_ms) newest_age_ms = age_ms;
        }
    }
    float body[3];
    Motor_Chassis_Forward(wheel_rad_s, body);
    yaw_rad = remainderf(yaw_rad, ODOMETRY_TWO_PI);
    uint8_t input_mode = ODOMETRY_INPUT_POSITION;
    if (use_eskf)
        input_mode = timing ? ODOMETRY_INPUT_POSITION_ESKF_TIMED : ODOMETRY_INPUT_POSITION_ESKF;
    if (state->input_mode != input_mode)
    {
        Odometry_Invalidate(state);
    }
    state->input_mode = input_mode;
    if (timing != NULL)
    {
        const uint32_t imu_age_ms = timing->control_time_ms - timing->imu_time_ms;
        if (imu_age_ms > CHASSIS_ODOMETRY_MAX_IMU_AGE_MS)
        {
            Odometry_Invalidate(state);
            return 0U;
        }
        const float bias = state->initialized ? state->heading.bias_rad_s : 0.0f;
        yaw_rad = remainderf(yaw_rad + (gyro_rad_s - bias) * (0.001f * imu_age_ms), ODOMETRY_TWO_PI);
    }
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
        if (!Get_Wheel_Deltas(state, wheel_position_rad, timing, elapsed_us, delta))
        {
            Odometry_Invalidate(state);
            return 0U;
        }
        const uint32_t advance_ms = timing == NULL ? 0U :
            Wheel_Alignment_Update(&state->alignment, state->previous_wheel_time_ms, timing->wheel_time_ms,
                                   wheel_position_rad, timing->control_time_ms - oldest_age_ms,
                                   CHASSIS_ODOMETRY_ENCODER_PERIOD_RAD, delta);
        /* 正解是线性映射：输入角增量rad时，输出dx/dy为m、转角为rad。 */
        Motor_Chassis_Forward(delta, displacement);
        if (use_eskf)
        {
            const float dt = elapsed_us * 0.000001f;
            /* ESKF 的预测周期属于控制拍，轮角速度属于两个公共时刻之间的区间。 */
            const float wheel_turn = advance_ms > 0 ? displacement[2] * (dt / (0.001f * advance_ms)) : NAN;
            const uint8_t heading_valid = timing != NULL
                ? Heading_ESKF_Update_Observed(&state->heading, yaw_rad, gyro_rad_s, wheel_turn, dt, observe_yaw)
                : Heading_ESKF_Update(&state->heading, yaw_rad, gyro_rad_s, displacement[2], dt);
            if (!heading_valid)
            {
                Odometry_Invalidate(state);
                return 0U;
            }
            yaw_rad = state->heading.yaw_rad;
        }
        float start_yaw = state->previous_yaw_rad, end_yaw = yaw_rad;
        if (timing != NULL)
        {
            start_yaw = state->common_yaw_rad;
            const float age_s = 0.001f * (timing->control_time_ms - state->alignment.common_time_ms);
            end_yaw = advance_ms > 0 ? yaw_rad - (gyro_rad_s - state->heading.bias_rad_s) * age_s : start_yaw;
            state->common_yaw_rad = end_yaw;
        }
        if (!Planar_Odometry_Integrate(&state->data.x_m, &state->data.y_m, displacement[0], displacement[1],
                                       start_yaw, end_yaw))
        {
            Odometry_Invalidate(state);
            return 0U;
        }
    }
    else if (use_eskf && (!observe_yaw || !Heading_ESKF_Init(&state->heading, yaw_rad, gyro_rad_s)))
    {
        Odometry_Invalidate(state);
        return 0U;
    }
    else if (timing != NULL)
    {
        /* 首组最晚的时间才是三轮都能从测量重建的起点。 */
        Wheel_Alignment_Reset(&state->alignment, wheel_position_rad, timing->wheel_time_ms,
                             timing->control_time_ms - newest_age_ms);
        state->common_yaw_rad = yaw_rad - gyro_rad_s * (0.001f * newest_age_ms);
    }
    for (unsigned i = 0U; i < 3U; ++i)
    {
        state->previous_position_rad[i] = wheel_position_rad[i];
        if (timing != NULL)
            state->previous_wheel_time_ms[i] = timing->wheel_time_ms[i];
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
    Odometry_Feedback_Timing_t timing = {0};
    timing.control_time_ms = wheel_time_ms[0];
    for (unsigned i = 0U; i < 3U; ++i)
    {
        timing.wheel_time_ms[i] = wheel_time_ms[i];
        if ((int32_t)(wheel_time_ms[i] - timing.control_time_ms) > 0)
            timing.control_time_ms = wheel_time_ms[i];
    }
    timing.imu_time_ms = timing.control_time_ms;
    return Odometry_Update_Feedback(state, position, wheel, &timing, yaw, gyro, now_us, observe_yaw);
}

uint8_t Odometry_Update_Feedback(Odometry_State_t *state, const float position[3], const float wheel[3],
                                 const Odometry_Feedback_Timing_t *timing, float yaw, float gyro,
                                 uint32_t now_us, uint8_t observe_yaw)
{
    if (timing == NULL)
    {
        Odometry_Invalidate(state);
        return 0U;
    }
    return Update_Position(state, position, wheel, timing, yaw, gyro, now_us, 1U, observe_yaw);
}
