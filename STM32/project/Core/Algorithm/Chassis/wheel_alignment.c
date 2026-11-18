#include "wheel_alignment.h"
#include <math.h>

_Static_assert((WHEEL_ALIGNMENT_HISTORY_MS & (WHEEL_ALIGNMENT_HISTORY_MS - 1U)) == 0U,
               "history size must divide the uint32 tick period");

void Wheel_Alignment_Reset(Wheel_Alignment_t *state, const float position[3],
                          const uint32_t time_ms[3], uint32_t latest_ms)
{
    state->common_time_ms = latest_ms;
    for (unsigned i = 0U; i < 3U; ++i)
    {
        state->position_rad[i][time_ms[i] % WHEEL_ALIGNMENT_HISTORY_MS] = position[i];
        state->common_position_rad[i] = position[i];
    }
}

uint32_t Wheel_Alignment_Update(Wheel_Alignment_t *state, const uint32_t previous_ms[3],
                                const uint32_t current_ms[3], const float position[3],
                                uint32_t oldest_ms, float period_rad, float delta[3])
{
    const uint32_t start_ms = state->common_time_ms;
    const uint32_t end_ms = (int32_t)(oldest_ms - start_ms) > 0 ? oldest_ms : start_ms;
    for (unsigned i = 0U; i < 3U; ++i)
    {
        if ((int32_t)(current_ms[i] - end_ms) < 0)
        {
            delta[i] = 0.0f; /* 首组偏早的轮尚未到达原点。 */
            continue;
        }
        const uint32_t interval_ms = current_ms[i] - previous_ms[i];
        const float rad_per_ms = interval_ms ? delta[i] / (float)interval_ms : 0.0f;
        if ((int32_t)(previous_ms[i] - start_ms) < 0)
            state->common_position_rad[i] = position[i] - rad_per_ms * (float)(current_ms[i] - start_ms);

        /* 只填新线段中仍可能被使用的毫秒。保留更早的折点，快轮后续加速
         * 不会改变慢轮尚未追上的历史。相同毫秒有新反馈时采用最新码值。 */
        float *history = state->position_rad[i];
        history[current_ms[i] % WHEEL_ALIGNMENT_HISTORY_MS] = position[i];
        const uint32_t ahead_ms = current_ms[i] - end_ms;
        const uint32_t count = ahead_ms < interval_ms ? ahead_ms : interval_ms;
        for (uint32_t age = 1U; age <= count; ++age)
            history[(current_ms[i] - age) % WHEEL_ALIGNMENT_HISTORY_MS] = position[i] - rad_per_ms * (float)age;

        const float aligned = history[end_ms % WHEEL_ALIGNMENT_HISTORY_MS];
        delta[i] = remainderf(aligned - state->common_position_rad[i], period_rad);
        state->common_position_rad[i] = aligned;
    }
    state->common_time_ms = end_ms;
    return end_ms - start_ms;
}
