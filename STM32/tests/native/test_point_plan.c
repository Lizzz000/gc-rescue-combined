#include "point_plan.h"
#include "velocity_profile.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>

static const double pi = 3.14159265358979323846;
static const Point_Plan_Config_t config = {0.2f, 0.5f, 0.3f, 0.8f, 0.12f, 0.01f, 0.06981317f};

static void Plant(unsigned axis, uint8_t half_turn)
{
    const double direction = axis * pi / 4.0;
    const double start[3] = {0.31, -0.27, half_turn ? pi - 0.01 : 2.8};
    const double goal[3] = {start[0] + 1.2 * cos(direction), start[1] + 1.2 * sin(direction), half_turn ? 0.0 : -2.2};
    double pose[3] = {start[0], start[1], start[2]};
    Point_Plan_Config_t cfg = config;
    if (half_turn)
    {
        cfg.linear_limit_mps = 0.4f;
        cfg.angular_limit_rad_s = 3.0f;
    }
    Point_Plan_t plan;
    assert(Point_Plan_Init(&plan, start, goal));
    assert(fabs(plan.length_m - 1.2) < 1e-9 && fabs(plan.yaw_delta_rad) < pi);
    Velocity_Profile_t profile = {0};
    double traveled = 0.0, turn = 0.0, worst_cross = 0.0;
    unsigned at = 0;
    for (unsigned step = 1; step < 1600; ++step)
    {
        const double cosine = cos(pose[2]), sine = sin(pose[2]);
        const float measured[3] = {
            (float)(cosine * profile.output[0] + sine * profile.output[1]),
            (float)(-sine * profile.output[0] + cosine * profile.output[1]),
            profile.output[2]
        };
        Point_Plan_Output_t output;
        assert(Point_Plan_Step(&plan, &cfg, pose, measured, &output));
        assert(hypotf(output.vx_mps, output.vy_mps) <= cfg.linear_limit_mps + 1e-6f);
        assert(fabsf(output.wz_rad_s) <= cfg.angular_limit_rad_s + 1e-6f);
        if (output.arrived && !at)
            at = step;
        const float target[3] = {at ? 0.0f : output.vx_mps,
                                 at ? 0.0f : output.vy_mps, at ? 0.0f : output.wz_rad_s};
        const float dt = 0.01f;
        Velocity_Profile_Step(&profile, target, at ? 0.75f : 0.45f, at ? 4.0f : 1.6f, 0.04f, dt);
        pose[0] += profile.output[0] * dt;
        pose[1] += profile.output[1] * dt;
        pose[2] = remainder(pose[2] + profile.output[2] * dt, 2.0 * pi);
        traveled += hypot(profile.output[0], profile.output[1]) * dt;
        turn += fabs(profile.output[2]) * dt;
        const double cross = (pose[0] - start[0]) * sin(direction) - (pose[1] - start[1]) * cos(direction);
        worst_cross = fmax(worst_cross, fabs(cross));
        if (at && step > at + 100U)
            break;
    }
    const double error = hypot(goal[0] - pose[0], goal[1] - pose[1]);
    printf("axis=%u half_turn=%u path=%.4fm lateral=%.4fm error=%.4fm time=%.3fs turn=%.3frad\n",
           axis, half_turn, traveled, worst_cross, error, at * 0.01, turn);
    assert(at && at * 0.01 < 9.0 && error < 0.02 && worst_cross < 0.005);
    assert(traveled >= 1.18 && traveled <= 1.22 && turn <= fabs(plan.yaw_delta_rad) + 0.08);
}

static void Test_Damping(void)
{
    double pose[3] = {0.0, 0.0, pi / 2};
    const double goal[3] = {0.03, 0.0, pi / 2 + 0.12};
    Point_Plan_t plan;
    assert(Point_Plan_Init(&plan, pose, goal));
    Point_Plan_Output_t base, damped;
    const float resting[3] = {0}, moving[3] = {0.15f, -0.24f, 0.8f};
    assert(Point_Plan_Step(&plan, &config, pose, resting, &base));
    assert(Point_Plan_Step(&plan, &config, pose, moving, &damped));
    /* 车头朝世界 +y，车体侧向速度为 -y 时正朝世界 +x 接近。 */
    assert(damped.vx_mps > 0.0f && damped.vx_mps < base.vx_mps);
    assert(damped.vy_mps < -0.06f && fabsf(base.vy_mps) < 1e-6f);
    assert(damped.wz_rad_s > 0.0f && damped.wz_rad_s < base.wz_rad_s);
}

static void Test_Approach_Braking_Envelope(void)
{
    const double pose[3] = {0.0, 0.0, pi / 2.0};
    const double goal[3] = {0.011, 0.0, pi / 2.0};
    const float sliding[3] = {0.40f, 0.0f, 0.0f};
    Point_Plan_t plan;
    Point_Plan_Output_t output;
    assert(Point_Plan_Init(&plan, pose, goal));
    assert(Point_Plan_Step(&plan, &config, pose, sliding, &output));
    const double speed = hypot(output.vx_mps, output.vy_mps);
    const double brake_distance = speed * config.latency_s + speed * speed / (2.0 * config.linear_deceleration_mps2);
    assert(!output.arrived && output.vx_mps > 0.02f && output.vy_mps < -0.02f);
    assert(brake_distance <= goal[0] - 0.5 * config.position_tolerance_m + 1e-7);
}

static void Test_Half_Turn(void)
{
    double pose[3] = {0.0, 0.0, pi - 0.01};
    const double goal[3] = {0.0, 0.0, 0.0};
    for (unsigned moving = 0; moving < 2; ++moving)
    {
        pose[2] = pi - 0.01;
        Point_Plan_t plan;
        Point_Plan_Output_t output;
        const float feedback[3] = {0.0f, 0.0f, moving ? 0.5f : 0.0f};
        const float expected_sign = moving ? 1.0f : -1.0f;
        assert(Point_Plan_Init(&plan, pose, goal));
        for (unsigned index = 0; index < 5; ++index)
        {
            assert(Point_Plan_Step(&plan, &config, pose, feedback, &output));
            assert(expected_sign * output.wz_rad_s > 0.0f);
            pose[2] = -pose[2];
        }
    }
}

int main(void)
{
    for (unsigned axis = 0; axis < 8; ++axis)
        Plant(axis, 0U);
    Plant(0U, 1U);
    Plant(1U, 1U);
    Test_Damping();
    Test_Approach_Braking_Envelope();
    Test_Half_Turn();

    Point_Plan_t plan;
    double start[3] = {0, 0, 3.124139361}, goal[3] = {1.2, 0.6, -3.124139361};
    assert(Point_Plan_Init(&plan, start, goal));
    assert(fabs(plan.length_m - sqrt(1.8)) < 1e-9 && fabs(plan.yaw_delta_rad - 0.034906585) < 1e-6);
    Point_Plan_Output_t out;
    const double displaced[3] = {0.2, -0.1, 1.1};
    const float resting[3] = {0.0f}, invalid[3] = {NAN, 0.0f, 0.0f};
    assert(Point_Plan_Step(&plan, &config, displaced, resting, &out));
    assert(fabs(out.vx_mps * 0.7 - out.vy_mps * 1.0) < 1e-7);
    assert(!Point_Plan_Step(&plan, &config, displaced, invalid, &out) && out.vx_mps == 0 && out.wz_rad_s == 0);
    goal[0] = NAN;
    assert(!Point_Plan_Init(&plan, start, goal));
    assert(!Point_Plan_Step(&plan, &config, displaced, resting, &out));
    puts("PASS point plan: world-frame ramp, half-turn continuity, damping and invalid input");
    return 0;
}
