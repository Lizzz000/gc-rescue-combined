#ifndef WHEEL_ALIGNMENT_H
#define WHEEL_ALIGNMENT_H

#include <stdint.h>

/* 每轮保留一个反馈窗口内每毫秒的插值位置。2 的幂确保 tick 回绕时槽位连续。 */
#define WHEEL_ALIGNMENT_HISTORY_MS 64U

typedef struct
{
    float position_rad[3][WHEEL_ALIGNMENT_HISTORY_MS];
    float common_position_rad[3];
    uint32_t common_time_ms;
} Wheel_Alignment_t;

/* 时间戳均来自同一毫秒时钟。首组以最晚轮时间为原点。 */
void Wheel_Alignment_Reset(Wheel_Alignment_t *state, const float position[3],
                           const uint32_t time_ms[3], uint32_t latest_ms);

/* 将已校验、已解绕的三轮增量替换为同一时间区间的增量，返回公共时刻的推进量。
 * 每轮时刻须单调，组内时差须小于历史窗口，期间轮角变化须小于半个回绕周期。 */
uint32_t Wheel_Alignment_Update(Wheel_Alignment_t *state, const uint32_t previous_ms[3],
                                const uint32_t current_ms[3], const float position[3],
                                uint32_t oldest_ms, float period_rad, float delta[3]);

#endif
