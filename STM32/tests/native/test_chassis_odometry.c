#include <assert.h>
#include <math.h>
#include <stddef.h>
#include <stdio.h>
#include "chassis_odometry.h"
#include "motor_chassis.h"

#define TEST_PI 3.14159265358979323846

static void near(double actual, double expected, double tolerance)
{
    assert(isfinite(actual) && fabs(actual - expected) <= tolerance);
}

static uint8_t feed(Odometry_State_t *state, float vx, float vy, float wheel_wz,
                    float yaw_rad, uint32_t now_us)
{
    float wheel[3];
    Motor_Chassis_Inverse(vx, vy, wheel_wz, wheel);
    return Odometry_Update(state, wheel, yaw_rad, now_us);
}

/** @brief 验证八向反馈速度积分、世界坐标转向以及首点不产生虚构位移。 */
static void test_eight_directions(void)
{
    static const int axes[8][2] = {
        {1,0}, {-1,0}, {0,1}, {0,-1}, {1,1}, {1,-1}, {-1,1}, {-1,-1}
    };
    for (unsigned turn = 0U; turn < 2U; ++turn)
    {
        const float yaw = turn ? (float)(TEST_PI / 2.0) : 0.0f;
        for (unsigned direction = 0U; direction < 8U; ++direction)
        {
            Odometry_State_t state;
            Odometry_Init(&state);
            const float scale = axes[direction][0] && axes[direction][1] ?
                                0.25f / sqrtf(2.0f) : 0.25f;
            const float vx = scale * axes[direction][0], vy = scale * axes[direction][1];
            assert(feed(&state, vx, vy, 0.0f, yaw, 1000U));
            assert(state.data.valid && state.initialized);
            near(state.data.x_m, 0.0, 0.0);
            near(state.data.y_m, 0.0, 0.0);
            for (unsigned sample = 1U; sample <= 100U; ++sample)
                assert(feed(&state, vx, vy, 0.0f, yaw, 1000U + sample * 10000U));
            near(state.data.x_m, turn ? -vy : vx, 0.000001);
            near(state.data.y_m, turn ? vx : vy, 0.000001);
            near(state.data.vx_mps, vx, 0.000001);
            near(state.data.vy_mps, vy, 0.000001);
            near(state.data.yaw_rad, yaw, 0.000001);
        }
    }
}

/** @brief 三轮共同转动只有诊断角速度；平移为零且航向必须来自 IMU。 */
static void test_rotation_and_heading_source(void)
{
    Odometry_State_t state;
    Odometry_Init(&state);
    assert(feed(&state, 0.0f, 0.0f, 1.0f, 0.0f, 0U));
    for (unsigned sample = 1U; sample <= 100U; ++sample)
        assert(feed(&state, 0.0f, 0.0f, 1.0f, sample * 0.02f, sample * 10000U));
    near(state.data.x_m, 0.0, 0.000001);
    near(state.data.y_m, 0.0, 0.000001);
    near(state.data.wheel_wz_rad_s, 1.0, 0.000001);
    near(state.data.yaw_rad, 2.0, 0.000001);
    Odometry_Init(&state);
    assert(feed(&state, 0.2f, 0.0f, 1.0f, 0.0f, 0U));
    for (unsigned sample = 1U; sample <= 100U; ++sample)
        assert(feed(&state, 0.2f, 0.0f, 1.0f, 0.0f, sample * 10000U));
    near(state.data.x_m, 0.2, 0.000001);
    near(state.data.y_m, 0.0, 0.000001);
    near(state.data.yaw_rad, 0.0, 0.0);
}

/** @brief 四分之一圆的独立几何终点，同时覆盖侧向速度和跨越 pi 的航向。 */
static void test_arcs_and_wrap(void)
{
    const float bodies[3][2] = {{1.0f,0.0f}, {0.0f,1.0f}, {0.3f,-0.2f}};
    for (unsigned index = 0U; index < 3U; ++index)
    {
        Odometry_State_t state;
        Odometry_Init(&state);
        assert(feed(&state, bodies[index][0], bodies[index][1], 0.0f, 0.0f, 0U));
        for (unsigned sample = 1U; sample <= 100U; ++sample)
            assert(feed(&state, bodies[index][0], bodies[index][1], 0.0f,
                        (float)(TEST_PI * sample / 200.0), sample * 10000U));
        const double radius = 2.0 / TEST_PI;
        near(state.data.x_m, radius * (bodies[index][0] - bodies[index][1]), 0.000002);
        near(state.data.y_m, radius * (bodies[index][0] + bodies[index][1]), 0.000002);
    }
    for (int direction = -1; direction <= 1; direction += 2)
    {
        Odometry_State_t state;
        Odometry_Init(&state);
        const float first = (float)(direction * (TEST_PI - 0.01));
        const float last = (float)(direction * (-TEST_PI + 0.01));
        assert(feed(&state, 1.0f, 0.0f, 0.0f, first, 100U));
        assert(feed(&state, 1.0f, 0.0f, 0.0f, last, 20100U));
        /* 绕 pi 的短弧朝负 x，横向两半相消，不应被当成近一整圈。 */
        near(state.data.x_m, -0.02 * sin(0.01) / 0.01, 0.00000002);
        near(state.data.y_m, 0.0, 0.00000001);
        near(state.data.yaw_rad, last, 0.000001);
    }
}

/** @brief 不同采样间隔的线性加速积分一致，并跨过 uint32 微秒计数器回绕。 */
static void test_timing_and_acceleration(void)
{
    static const uint32_t intervals[] = {1000U, 9000U, 25000U, 15000U};
    Odometry_State_t state;
    Odometry_Init(&state);
    uint32_t clock = UINT32_MAX - 10000U, elapsed = 0U;
    assert(feed(&state, 0.0f, 0.0f, 0.0f, 0.0f, clock));
    for (unsigned sample = 0U; sample < 80U; ++sample)
    {
        elapsed += intervals[sample % 4U];
        clock += intervals[sample % 4U];
        const float speed = elapsed * 0.000001f;
        assert(feed(&state, speed, -0.2f * speed, 0.0f, 0.0f, clock));
    }
    assert(elapsed == 1000000U);
    near(state.data.x_m, 0.5, 0.000001);
    near(state.data.y_m, -0.1, 0.000001);
    assert(state.data.sample_time_us == clock);

    const Chassis_Odometry_t before = state.data;
    assert(!feed(&state, -1.0f, 0.3f, 0.5f, 1.0f, clock));
    assert(state.data.valid && state.initialized);
    near(state.data.x_m, before.x_m, 0.0);
    near(state.data.y_m, before.y_m, 0.0);
    near(state.data.yaw_rad, before.yaw_rad, 0.0);
    near(state.data.vx_mps, before.vx_mps, 0.0);
    assert(feed(&state, 1.0f, -0.2f, 0.0f, 0.0f, clock + 50000U));
    near(state.data.x_m, 0.55, 0.000001);
    near(state.data.y_m, -0.11, 0.000001);
}

static void check_invalid(const Odometry_State_t *state, const Chassis_Odometry_t *previous)
{
    assert(!state->data.valid && !state->initialized);
    near(state->data.x_m, previous->x_m, 0.0);
    near(state->data.y_m, previous->y_m, 0.0);
    near(state->data.yaw_rad, previous->yaw_rad, 0.0);
    assert(state->data.sample_time_us == previous->sample_time_us);
    assert(state->data.vx_mps == 0.0f && state->data.vy_mps == 0.0f &&
           state->data.wheel_wz_rad_s == 0.0f);
}

/** @brief 缺测、非法值和恢复均不得跨失效区间增加位移。 */
static void test_invalid_and_recovery(void)
{
    static const float invalid[] = {NAN, INFINITY, -INFINITY};
    for (unsigned input = 0U; input < 4U; ++input)
    {
        for (unsigned bad = 0U; bad < 3U; ++bad)
        {
            Odometry_State_t state;
            Odometry_Init(&state);
            assert(feed(&state, 0.4f, 0.0f, 0.0f, 0.0f, 1000U));
            assert(feed(&state, 0.4f, 0.0f, 0.0f, 0.0f, 11000U));
            const Chassis_Odometry_t previous = state.data;
            float wheel[3];
            Motor_Chassis_Inverse(0.4f, 0.0f, 0.0f, wheel);
            if (input < 3U)
                wheel[input] = invalid[bad];
            assert(!Odometry_Update(&state, wheel, input == 3U ? invalid[bad] : 0.0f, 21000U));
            check_invalid(&state, &previous);
            assert(feed(&state, 0.4f, 0.0f, 0.0f, 0.0f, 900000U));
            near(state.data.x_m, previous.x_m, 0.0);
            assert(feed(&state, 0.4f, 0.0f, 0.0f, 0.0f, 910000U));
            near(state.data.x_m, 0.008, 0.000001);
        }
    }
    Odometry_State_t state;
    Odometry_Init(&state);
    assert(feed(&state, 0.5f, 0.0f, 0.0f, 0.0f, 100U));
    assert(feed(&state, 0.5f, 0.0f, 0.0f, 0.0f, 10100U));
    const Chassis_Odometry_t before_gap = state.data;
    assert(!feed(&state, 0.5f, 0.0f, 0.0f, 0.0f, 60101U));
    check_invalid(&state, &before_gap);
    assert(feed(&state, 0.5f, 0.0f, 0.0f, 0.0f, 70101U));
    near(state.data.x_m, before_gap.x_m, 0.0);
    const Chassis_Odometry_t before_null = state.data;
    assert(!Odometry_Update(&state, NULL, 0.0f, 80101U));
    check_invalid(&state, &before_null);

    assert(feed(&state, 0.5f, 0.0f, 0.0f, 0.0f, 90101U));
    const Chassis_Odometry_t before_reverse_clock = state.data;
    assert(!feed(&state, 0.5f, 0.0f, 0.0f, 0.0f, 90100U));
    check_invalid(&state, &before_reverse_clock);
    Odometry_Init(&state);
    assert(!state.data.valid && !state.initialized && state.data.sample_time_us == 0U);
    assert(state.data.x_m == 0.0 && state.data.y_m == 0.0 && state.data.yaw_rad == 0.0f);
    assert(!Odometry_Update(NULL, NULL, 0.0f, 0U));
    Odometry_Init(NULL);
    Odometry_Invalidate(NULL);
}

/** @brief 小角数值稳定性和外部失效通知不能丢失已累计位姿。 */
static void test_small_angle_and_invalidate(void)
{
    Odometry_State_t state;
    Odometry_Init(&state);
    assert(feed(&state, 1.0f, 0.0f, 0.0f, 0.0f, 0U));
    assert(feed(&state, 1.0f, 0.0f, 0.0f, 0.0000001f, 10000U));
    near(state.data.x_m, 0.01, 0.000000001);
    near(state.data.y_m, 0.0000000005, 0.00000000001);
    const Chassis_Odometry_t previous = state.data;
    Odometry_Invalidate(&state);
    check_invalid(&state, &previous);
    assert(feed(&state, 1.0f, 0.0f, 0.0f, (float)(4.5 * TEST_PI), 1000000U));
    near(state.data.x_m, previous.x_m, 0.0);
    near(state.data.yaw_rad, TEST_PI / 2.0, 0.000002);
    assert(feed(&state, 1.0f, 0.0f, 0.0f, (float)(TEST_PI / 2.0), 1010000U));
    near(state.data.x_m, previous.x_m, 0.00000002);
    near(state.data.y_m, previous.y_m + 0.01, 0.00000002);
}

int main(void)
{
    test_eight_directions();
    test_rotation_and_heading_source();
    test_arcs_and_wrap();
    test_timing_and_acceleration();
    test_invalid_and_recovery();
    test_small_angle_and_invalidate();
    puts("PASS odometry: measured wheels, IMU heading, eight directions, arcs, timing, invalid data and recovery");
    return 0;
}
