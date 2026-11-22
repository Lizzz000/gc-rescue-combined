#include "navigation_speed.h"
#include <assert.h>
#include <math.h>
#include <stddef.h>
#include <stdio.h>

/* 实测速度滞后时仍能到达精确指令，二维步长不超过底盘允许值。 */
static void Check_Transition(Navigation_Speed_Mode_t mode)
{
    Navigation_Speed_State_t state = {0};
    float command[2] = {0.0f, 0.0f};
    float measured[2] = {0.0f, 0.0f};
    const float target[2] = {0.16f, 0.12f};
    const float acceleration_limit = 0.45f;
    for (unsigned step = 0U; step < 500U; ++step)
    {
        const float dt = step % 2U == 0U ? 0.008f : 0.013f;
        const float acceleration = Navigation_Speed_Acceleration(&state, mode, command, target, measured,
                                                                  acceleration_limit, dt);
        assert(acceleration > 0.0f && acceleration <= acceleration_limit);
        const float dx = target[0] - command[0], dy = target[1] - command[1];
        const float gap = hypotf(dx, dy);
        const float fraction = fminf(1.0f, acceleration * dt / gap);
        const float previous[2] = {command[0], command[1]};
        for (unsigned axis = 0U; axis < 2U; ++axis)
        {
            command[axis] = fraction == 1.0f ? target[axis] : command[axis] + (target[axis] - command[axis]) * fraction;
            measured[axis] += dt / (0.08f + dt) * (0.85f * command[axis] - measured[axis]);
            assert(command[axis] >= previous[axis] && command[axis] <= target[axis]);
        }
        assert(hypotf(command[0] - previous[0], command[1] - previous[1]) <= acceleration_limit * dt + 1e-6f);
        if (command[0] == target[0] && command[1] == target[1])
            return;
    }
    assert(0 && "navigation command did not reach target in bounded time");
}

static void Check_Feedback_Response(void)
{
    Navigation_Speed_State_t state = {0};
    const float command[2] = {0.18f, 0.0f}, target[2] = {0.22f, 0.0f};
    const float slow_feedback[2] = {0.12f, 0.0f}, fast_feedback[2] = {0.24f, 0.0f};
    const float limit = 0.45f, dt = 0.01f;

    const float from_slow = Navigation_Speed_Acceleration(&state, NAV_SPEED_MPC, command, target, slow_feedback,
                                                           limit, dt);
    state = (Navigation_Speed_State_t){0};
    const float from_fast = Navigation_Speed_Acceleration(&state, NAV_SPEED_MPC, command, target, fast_feedback,
                                                           limit, dt);
    assert(from_slow > from_fast);

    const float inside_boundary[2] = {0.21f, 0.0f};
    const float smc = Navigation_Speed_Acceleration(&state, NAV_SPEED_SMC, command, target, inside_boundary,
                                                     limit, dt);
    assert(fabsf(smc - 0.18f) < 1e-5f);
}

static void Check_Disturbance_And_Stale_Feedback(void)
{
    Navigation_Speed_State_t state = {0};
    float command[2] = {0.0f, 0.0f};
    const float stationary[2] = {0.0f, 0.0f}, target[2] = {0.40f, 0.0f};
    const float limit = 0.45f, dt = 0.01f;
    for (unsigned step = 0U; step < 50U; ++step)
    {
        const float acceleration = Navigation_Speed_Acceleration(&state, NAV_SPEED_ADRC, command, target,
                                                                  stationary, limit, dt);
        command[0] += acceleration * dt;
    }
    assert(state.observer_ready && state.disturbance_mps2[0] < -0.20f);
    assert(state.disturbance_mps2[1] == 0.0f);

    const float fallback = Navigation_Speed_Acceleration(&state, NAV_SPEED_ADRC, command, target, NULL,
                                                          limit, dt);
    assert(fallback == limit);
    assert(!state.observer_ready && state.disturbance_mps2[0] == 0.0f);
    assert(state.previous_accel_mps2[0] == 0.0f && state.previous_accel_mps2[1] == 0.0f);
}

int main(void)
{
    Check_Transition(NAV_SPEED_BASELINE);
    Check_Transition(NAV_SPEED_MPC);
    Check_Transition(NAV_SPEED_SMC);
    Check_Transition(NAV_SPEED_ADRC);
    Check_Feedback_Response();
    Check_Disturbance_And_Stale_Feedback();
    puts("PASS navigation speed: bounded MPC, SMC and ADRC, finite target, disturbance and stale feedback");
    return 0;
}
