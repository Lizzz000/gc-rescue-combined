#include "adaptive_kalman.h"
#include "image_tracker.h"

#include <assert.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>

static uint32_t random_state;

/* 用固定种子产生近似正态噪声，方便重复比较。 */
static float noise(void)
{
    float sum = 0.0f;
    for (unsigned i = 0U; i < 12U; ++i)
    {
        random_state = random_state * 1664525U + 1013904223U;
        sum += (float)(random_state >> 8U) / 16777216.0f;
    }
    return sum - 6.0f;
}

static void compare_scenario(unsigned scenario, double *baseline_error, double *candidate_error)
{
    for (unsigned seed = 1U; seed <= 30U; ++seed)
    {
        random_state = seed;
        Image_Tracker_t baseline;
        AdaptiveKalman candidate;
        Image_Tracker_Init(&baseline, 15.0f, 25.0f, 4000.0f);
        AdaptiveKalman_Init(&candidate, 15.0f, 25.0f, 4000.0f);
        float position = 5000.0f;
        float velocity = 0.0f;
        uint32_t timestamp = 0U;
        const uint32_t intervals[] = {8U, 10U, 13U, 9U};
        for (unsigned sample = 0U; sample < 1500U; ++sample)
        {
            const uint32_t interval = intervals[sample % 4U];
            const float dt = (float)interval * 0.001f;
            timestamp += interval;
            float acceleration = 0.0f;
            if (scenario == 1U)
            {
                const unsigned phase = sample % 300U;
                if (phase < 60U)
                    acceleration = 2200.0f;
                else if (phase < 120U)
                    acceleration = -2200.0f;
            }
            position += velocity * dt + 0.5f * acceleration * dt * dt;
            velocity += acceleration * dt;
            float sigma = 15.0f;
            if (scenario == 2U && sample % 600U >= 300U && sample % 600U < 550U)
                sigma = 75.0f;
            float measured = position + sigma * noise();
            if (scenario == 3U && sample > 50U && sample % 90U == 0U)
                measured += 250.0f;
            Image_Tracker_Update(&baseline, measured, 7500.0f, timestamp);
            AdaptiveKalman_Update(&candidate, measured, 7500.0f, timestamp);
            if (sample < 50U)
                continue;
            const double baseline_delta = (double)baseline.position[0] - position;
            const double candidate_delta = (double)candidate.position[0] - position;
            *baseline_error += baseline_delta * baseline_delta;
            *candidate_error += candidate_delta * candidate_delta;
            assert(isfinite(candidate.position[0]) && isfinite(candidate.velocity[0]));
            const float cross = candidate.position_velocity_covariance[0];
            assert(candidate.position_variance[0] * candidate.velocity_variance[0] >= cross * cross * 0.99999f);
        }
    }
}

static void test_time_and_reset(void)
{
    AdaptiveKalman tracker;
    AdaptiveKalman_Init(&tracker, 15.0f, 25.0f, 4000.0f);
    AdaptiveKalman_Update(&tracker, 2000.0f, 8000.0f, UINT32_MAX - 10U);
    AdaptiveKalman_Update(&tracker, 2020.0f, 7960.0f, 9U);
    const AdaptiveKalman previous = tracker;
    AdaptiveKalman_Update(&tracker, 6000.0f, 2000.0f, 9U);
    AdaptiveKalman_Update(&tracker, 6000.0f, 2000.0f, 8U);
    assert(tracker.position[0] == previous.position[0]);
    assert(tracker.timestamp_ms == previous.timestamp_ms);
    AdaptiveKalman_Init(&tracker, 15.0f, 25.0f, 4000.0f);
    assert(!tracker.initialized && tracker.measurement_scale[0] == 1.0f);
    AdaptiveKalman_Update(&tracker, 7000.0f, 1500.0f, 5000U);
    assert(tracker.position[0] == 7000.0f && tracker.velocity[0] == 0.0f);
}

static void test_persistent_change(void)
{
    AdaptiveKalman tracker;
    AdaptiveKalman_Init(&tracker, 15.0f, 25.0f, 4000.0f);
    for (uint32_t sample = 0U; sample < 100U; ++sample)
        AdaptiveKalman_Update(&tracker, 5000.0f, 7500.0f, sample * 10U);
    for (uint32_t sample = 100U; sample < 200U; ++sample)
        AdaptiveKalman_Update(&tracker, 6000.0f, 7500.0f, sample * 10U);
    assert(fabsf(tracker.position[0] - 6000.0f) < 1.0f);
    assert(fabsf(tracker.velocity[0]) < 3.0f);
}

int main(void)
{
    test_time_and_reset();
    test_persistent_change();
    for (unsigned scenario = 0U; scenario < 4U; ++scenario)
    {
        double baseline_error = 0.0;
        double candidate_error = 0.0;
        compare_scenario(scenario, &baseline_error, &candidate_error);
        const double ratio = sqrt(candidate_error / baseline_error);
        printf("scenario %u: candidate/baseline RMS %.3f\n", scenario, ratio);
        if (scenario == 2U || scenario == 3U)
            assert(ratio < 0.95);
        else
            assert(ratio < 1.10);
    }
    puts("PASS: adaptive Kalman timing, covariance and synthetic comparisons");
    return 0;
}
