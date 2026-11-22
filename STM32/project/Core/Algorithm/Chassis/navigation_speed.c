#include "navigation_speed.h"
#include <math.h>
#include <stddef.h>

#define MPC_HORIZON_STEPS 8U
#define MPC_ACCEL_LEVELS 6U
#define MPC_RESPONSE_S 0.08f
#define MPC_INPUT_WEIGHT_S2 0.0015f
#define MPC_INPUT_CHANGE_WEIGHT_S2 0.003f
#define MINIMUM_ACCEL_FRACTION 0.20f
#define SMC_BOUNDARY_MPS 0.025f
#define ADRC_OBSERVER_BANDWIDTH_PER_S 12.0f
#define ADRC_FEEDBACK_GAIN_PER_S 10.0f
#define ADRC_DISTURBANCE_LIMIT_MPS2 0.60f

static float Project(const float vector[2], const float direction[2])
{
    return vector[0] * direction[0] + vector[1] * direction[1];
}

/* 只预测速度误差方向的一维车体响应。用实测速度初始化预测，枚举七个恒定
 * 输入，在八个采样拍中限制指令上界并累加跟踪、输入和切换代价。仅执行首拍。 */
static float Predictive_Acceleration(float command_gap, float measured_gap, float previous_accel,
                                     float minimum_accel, float acceleration_limit, float dt)
{
    const float gain = dt / (MPC_RESPONSE_S + dt);
    const float inverse_dt = 1.0f / dt;
    float minimum_cost = HUGE_VALF;
    float best = minimum_accel;
    for (unsigned level = 0U; level <= MPC_ACCEL_LEVELS; ++level)
    {
        const float fraction = (float)level / (float)MPC_ACCEL_LEVELS;
        const float candidate = minimum_accel + (acceleration_limit - minimum_accel) * fraction;
        float request_gap = command_gap, response_gap = measured_gap;
        const float change = fminf(candidate, command_gap * inverse_dt) - previous_accel;
        float cost = MPC_INPUT_CHANGE_WEIGHT_S2 * change * change;
        for (unsigned step = 0U; step < MPC_HORIZON_STEPS; ++step)
        {
            const float applied = fminf(candidate, request_gap * inverse_dt);
            request_gap = fmaxf(0.0f, request_gap - applied * dt);
            response_gap += gain * (request_gap - response_gap);
            cost += response_gap * response_gap + MPC_INPUT_WEIGHT_S2 * applied * applied;
        }
        if (cost < minimum_cost)
        {
            minimum_cost = cost;
            best = candidate;
        }
    }
    return best;
}

/* 平移速度近似为一阶积分对象。每轴用二阶扩张状态观测器估计总扰动，
 * 使用上一拍限幅后的实际速度指令增量，首拍以反馈初始化观测值。 */
static void Observe_Velocity(Navigation_Speed_State_t *state, const float measured[2], float dt)
{
    if (!state->observer_ready)
    {
        state->observed_velocity_mps[0] = measured[0];
        state->observed_velocity_mps[1] = measured[1];
        state->observer_ready = 1U;
        return;
    }

    const float bandwidth = ADRC_OBSERVER_BANDWIDTH_PER_S;
    for (unsigned axis = 0U; axis < 2U; ++axis)
    {
        const float innovation = measured[axis] - state->observed_velocity_mps[axis];
        state->observed_velocity_mps[axis] += dt * (state->previous_accel_mps2[axis] +
            state->disturbance_mps2[axis] + 2.0f * bandwidth * innovation);
        const float disturbance = state->disturbance_mps2[axis] + dt * bandwidth * bandwidth * innovation;
        state->disturbance_mps2[axis] = fmaxf(-ADRC_DISTURBANCE_LIMIT_MPS2,
                                             fminf(ADRC_DISTURBANCE_LIMIT_MPS2, disturbance));
    }
}

float Navigation_Speed_Acceleration(Navigation_Speed_State_t *state, Navigation_Speed_Mode_t mode,
                                    const float command[2], const float target[2], const float measured[2],
                                    float acceleration_limit, float dt)
{
    if (mode == NAV_SPEED_BASELINE)
        return acceleration_limit;
    if (measured == NULL)
    {
        *state = (Navigation_Speed_State_t){0};
        return acceleration_limit;
    }

    if (mode == NAV_SPEED_ADRC)
        Observe_Velocity(state, measured, dt);

    const float delta[2] = {target[0] - command[0], target[1] - command[1]};
    const float gap = hypotf(delta[0], delta[1]);
    if (gap == 0.0f)
    {
        state->previous_accel_mps2[0] = state->previous_accel_mps2[1] = 0.0f;
        return 0.0f;
    }
    const float direction[2] = {delta[0] / gap, delta[1] / gap};
    const float measurement_error[2] = {target[0] - measured[0], target[1] - measured[1]};
    const float measured_gap = Project(measurement_error, direction);
    const float minimum_accel = MINIMUM_ACCEL_FRACTION * acceleration_limit;
    float acceleration = acceleration_limit;
    switch (mode)
    {
    case NAV_SPEED_MPC:
        acceleration = Predictive_Acceleration(gap, measured_gap,
                                               Project(state->previous_accel_mps2, direction),
                                               minimum_accel, acceleration_limit, dt);
        break;
    case NAV_SPEED_SMC:
        /* 速度差是滑模面，饱和边界层代替符号函数，减小小误差时的抖动。 */
        acceleration = acceleration_limit * (measured_gap / SMC_BOUNDARY_MPS);
        break;
    case NAV_SPEED_ADRC:
    {
        const float estimated_error[2] = {target[0] - state->observed_velocity_mps[0],
                                         target[1] - state->observed_velocity_mps[1]};
        acceleration = ADRC_FEEDBACK_GAIN_PER_S * Project(estimated_error, direction) -
            Project(state->disturbance_mps2, direction);
        break;
    }
    default:
        break;
    }

    /* 此处只整形仍需加速的速度指令。最小步长使短指令在有限时间内到达，
     * 反向与停车由底盘原路径处理。记录到达目标时的有效输入，避免观测器虚增。 */
    acceleration = fmaxf(minimum_accel, fminf(acceleration_limit, acceleration));
    const float applied = fminf(acceleration, gap / dt);
    for (unsigned axis = 0U; axis < 2U; ++axis)
        state->previous_accel_mps2[axis] = direction[axis] * applied;
    return acceleration;
}
