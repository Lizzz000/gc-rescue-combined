#include "tracked_visual_servo.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

static const Visual_Servo_Config_t config = {
    .max_speed_mps = 0.30f, .acceleration_mps2 = 0.40f, .deceleration_mps2 = 0.35f,
    .latency_s = 0.12f, .response_q_per_m = {10000.0f, 10000.0f},
};

static void Test_Independent_Plant(unsigned fps, float response, unsigned delay_ms, unsigned outlier)
{
    AdaptiveKalman filter;
    AdaptiveKalman_Init(&filter, 15.0f, 25.0f, 4000.0f);
    Visual_Servo_t servo = {0};
    const float goal[2] = {5000.0f, 7500.0f}, tolerance[2] = {100.0f, 100.0f};
    float remaining[2] = {-0.12f, 0.45f}, velocity[2] = {0}, history[6000][2];
    unsigned next_frame = 0U, frame = 0U, previous_ms = 0U, arrived = 0U, arrival_ms = 0U;
    float max_crossing = 0.0f;
    for (unsigned tick = 0U; tick < 3000U; ++tick)
    {
        const unsigned now = tick * 5U;
        for (unsigned axis = 0U; axis < 2U; ++axis)
            history[tick][axis] = remaining[axis];
        if (!arrived && now >= next_frame)
        {
            const unsigned old = tick > delay_ms / 5U ? tick - delay_ms / 5U : 0U;
            const float sign = frame % 2U ? -1.0f : 1.0f;
            float raw[2] = {goal[0] - response * history[old][0] + sign * 15.0f,
                            goal[1] - response * history[old][1] - sign * 25.0f};
            if (outlier && frame == fps)
                raw[1] += 1800.0f;
            AdaptiveKalman_Update(&filter, raw[0], raw[1], now);
            Tracked_Visual_Servo_Update(&servo, &config, &filter, raw, goal, tolerance, frame ? (now - previous_ms) * 0.001f : 0.0f);
            assert(hypotf(servo.speed[0], servo.speed[1]) <= config.max_speed_mps + 1e-6f);
            if (filter.accepted_mask == 3U && fabsf(goal[0] - raw[0]) <= 100.0f && fabsf(goal[1] - raw[1]) <= 100.0f)
            {
                arrived = 1U;
                arrival_ms = now;
                servo = (Visual_Servo_t){0};
            }
            previous_ms = now;
            next_frame = ++frame * 1000U / fps;
        }
        /* 独立实际执行器：60ms一阶响应、0.45m/s²加减速，1ms级视觉不是直接位置控制。 */
        for (unsigned axis = 0U; axis < 2U; ++axis)
        {
            const float delta = (servo.speed[axis] - velocity[axis]) * 0.005f / 0.06f;
            velocity[axis] += fmaxf(-0.45f * 0.005f, fminf(0.45f * 0.005f, delta));
            remaining[axis] -= velocity[axis] * 0.005f;
        }
        max_crossing = fmaxf(max_crossing, -remaining[1]);
        if (arrived && now > arrival_ms + 1000U)
            break;
    }
    printf("servo fps=%u response=%.0f delay=%u spike=%u arrival=%ums end=%.4f,%.4fm crossing=%.4fm\n",
           fps, response, delay_ms, outlier, arrival_ms, remaining[0], remaining[1], max_crossing);
    assert(arrived && arrival_ms < 10000U);
    /* 以图像窗口和噪声换算模型真值门槛，不能称为实车精度。 */
    assert(fabsf(remaining[0]) < 180.0f / response && fabsf(remaining[1]) < 180.0f / response);
    assert(max_crossing < 100.0f / response);
}

static void Test_Finite_Reversal_And_No_Windup(void)
{
    const float goal[2] = {5000, 7500}, tol[2] = {100, 100};
    float raw[2] = {5000, 7000};
    AdaptiveKalman filter;
    AdaptiveKalman_Init(&filter, 15, 25, 4000);
    Visual_Servo_t servo = {.speed = {.05f, .10f}};
    Tracked_Visual_Servo_Update(&servo, &config, &filter, raw, goal, tol, .02f);
    assert(servo.speed[0] == 0 && servo.speed[1] == 0);
    for (unsigned i = 0; i < 300; ++i)
    {
        AdaptiveKalman_Update(&filter, raw[0], raw[1], i * 20U);
        Tracked_Visual_Servo_Update(&servo, &config, &filter, raw, goal, tol, 0.02f);
    }
    const float stationary_speed = servo.speed[1];
    assert(stationary_speed > 0.08f && stationary_speed < 0.20f);
    raw[1] = 7399;
    AdaptiveKalman_Init(&filter, 15, 25, 4000);
    AdaptiveKalman_Update(&filter, raw[0], raw[1], 6000U);
    Tracked_Visual_Servo_Update(&servo, &config, &filter, raw, goal, tol, 0.02f);
    assert(servo.speed[1] > 0.02f && servo.speed[1] < 0.05f); /* 窗沿外无需0.005m/s爬行。 */
    raw[1] = 8000;
    AdaptiveKalman_Init(&filter, 15, 25, 4000);
    AdaptiveKalman_Update(&filter, raw[0], raw[1], 6020U);
    Tracked_Visual_Servo_Update(&servo, &config, &filter, raw, goal, tol, 0.02f);
    assert(servo.speed[1] == 0.0f);
    Tracked_Visual_Servo_Update(&servo, &config, &filter, raw, goal, tol, 0.02f);
    assert(servo.speed[1] < 0.0f && servo.speed[1] >= -0.008001f);
    raw[0] = NAN;
    Tracked_Visual_Servo_Update(&servo, &config, &filter, raw, goal, tol, 0.02f);
    assert(servo.speed[0] == 0 && servo.speed[1] == 0);
}

int main(void)
{
    Test_Finite_Reversal_And_No_Windup();
    const unsigned rates[] = {15, 30, 50, 100};
    const float responses[] = {5000, 10000, 20000};
    for (unsigned r = 0; r < 4; ++r)
    for (unsigned s = 0; s < 3; ++s)
    for (unsigned spike = 0; spike < 2; ++spike)
        Test_Independent_Plant(rates[r], responses[s], 60U, spike);
    puts("PASS visual servo: 24 closed-loop plants, speed bound, finite arrival, reversal, no windup");
    return 0;
}
