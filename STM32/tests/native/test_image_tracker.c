#include "image_tracker.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>

/** @brief 固定目标的抖动应减小，滤波后中心不应产生持续偏差。 */
static void Test_Static_Jitter(uint32_t interval_ms)
{
    Image_Tracker_t tracker;
    Image_Tracker_Init(&tracker, 15.0f, 25.0f, 4000.0f);
    const float center[2] = {5000.0f, 7500.0f};
    const float jitter[2] = {15.0f, 25.0f};
    float squared_error[2] = {0.0f};
    float error_sum[2] = {0.0f};
    for (uint32_t sample = 0U; sample < 250U; ++sample)
    {
        const float sign = sample % 2U ? -1.0f : 1.0f;
        Image_Tracker_Update(&tracker, center[0] + sign * jitter[0], center[1] - sign * jitter[1], sample * interval_ms);
        if (sample < 50U)
            continue;
        for (unsigned axis = 0U; axis < 2U; ++axis)
        {
            const float error = tracker.position[axis] - center[axis];
            squared_error[axis] += error * error;
            error_sum[axis] += error;
        }
    }
    for (unsigned axis = 0U; axis < 2U; ++axis)
    {
        assert(squared_error[axis] < 200.0f * jitter[axis] * jitter[axis] * 0.25f);
        assert(fabsf(error_sum[axis] / 200.0f) < 0.5f);
    }
}

/** @brief 不均匀帧间隔和时钟回绕时，应保持匀速目标的速度估计和当前位置。 */
static void Test_Variable_Interval_And_Wrap(void)
{
    Image_Tracker_t tracker, wrapping;
    Image_Tracker_Init(&tracker, 15.0f, 25.0f, 4000.0f);
    Image_Tracker_Init(&wrapping, 15.0f, 25.0f, 4000.0f);
    const uint32_t intervals_ms[] = {12U, 37U, 21U, 18U, 54U};
    const uint32_t near_wrap = UINT32_MAX - 43U;
    uint32_t elapsed_ms = 0U;
    Image_Tracker_Update(&tracker, 2500.0f, 8000.0f, 0U);
    Image_Tracker_Update(&wrapping, 2500.0f, 8000.0f, near_wrap);
    for (unsigned sample = 0U; sample < 85U; ++sample)
    {
        elapsed_ms += intervals_ms[sample % (sizeof(intervals_ms) / sizeof(intervals_ms[0]))];
        const float x = 2500.0f + 0.6f * (float)elapsed_ms;
        const float y = 8000.0f - 0.35f * (float)elapsed_ms;
        Image_Tracker_Update(&tracker, x, y, elapsed_ms);
        Image_Tracker_Update(&wrapping, x, y, near_wrap + elapsed_ms);
        for (unsigned axis = 0U; axis < 2U; ++axis)
        {
            assert(tracker.position[axis] == wrapping.position[axis]);
            assert(tracker.velocity[axis] == wrapping.velocity[axis]);
            const float cross = tracker.position_velocity_covariance[axis];
            assert(tracker.position_variance[axis] * tracker.velocity_variance[axis] >= cross * cross);
        }
        if (sample >= 40U)
        {
            assert(fabsf(tracker.position[0] - x) < 0.5f && fabsf(tracker.position[1] - y) < 0.5f);
            assert(fabsf(tracker.velocity[0] - 600.0f) < 1.0f && fabsf(tracker.velocity[1] + 350.0f) < 1.0f);
        }
    }
}

/** @brief 同毫秒重报不得重复收缩协方差；重置后的首帧不继承旧目标。 */
static void Test_Duplicate_And_Reset(void)
{
    Image_Tracker_t tracker;
    Image_Tracker_Init(&tracker, 15.0f, 25.0f, 4000.0f);
    Image_Tracker_Update(&tracker, 2000.0f, 8000.0f, 0U);
    Image_Tracker_Update(&tracker, 2020.0f, 7960.0f, 20U);
    assert(tracker.velocity[0] > 0.0f && tracker.velocity[1] < 0.0f);
    const Image_Tracker_t before = tracker;
    Image_Tracker_Update(&tracker, 6000.0f, 2000.0f, 20U);
    for (unsigned axis = 0U; axis < 2U; ++axis)
    {
        assert(tracker.position[axis] == before.position[axis] && tracker.velocity[axis] == before.velocity[axis]);
        assert(tracker.position_variance[axis] == before.position_variance[axis]);
        assert(tracker.position_velocity_covariance[axis] == before.position_velocity_covariance[axis]);
        assert(tracker.velocity_variance[axis] == before.velocity_variance[axis]);
    }

    /* 较旧的时间戳不得被当成回绕后的巨大时间间隔。 */
    Image_Tracker_Update(&tracker, 6000.0f, 2000.0f, 19U);
    assert(tracker.received_ms == before.received_ms);
    for (unsigned axis = 0U; axis < 2U; ++axis)
    {
        assert(tracker.position[axis] == before.position[axis]);
        assert(tracker.velocity[axis] == before.velocity[axis]);
        assert(tracker.velocity_variance[axis] == before.velocity_variance[axis]);
    }
    Image_Tracker_Init(&tracker, 15.0f, 25.0f, 4000.0f);
    assert(!tracker.initialized);
    Image_Tracker_Update(&tracker, 7000.0f, 1500.0f, 5000U);
    assert(tracker.initialized && tracker.received_ms == 5000U);
    assert(tracker.position[0] == 7000.0f && tracker.position[1] == 1500.0f);
    assert(tracker.velocity[0] == 0.0f && tracker.velocity[1] == 0.0f);
    assert(tracker.position_variance[0] == 225.0f && tracker.position_variance[1] == 625.0f);
}

/** @brief 运行二维图像跟踪器的主机回归。 @return 全部通过返回 0。 */
int main(void)
{
    Test_Static_Jitter(10U);
    Test_Static_Jitter(20U);
    Test_Variable_Interval_And_Wrap();
    Test_Duplicate_And_Reset();
    puts("PASS image tracker: static jitter, variable interval tracking, wrap, duplicate and reset");
    return 0;
}
