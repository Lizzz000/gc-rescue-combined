#ifndef VISUAL_OBSERVATION_H
#define VISUAL_OBSERVATION_H

#include <stdint.h>

/* 类型 3/4 共用的视觉输入。sequence 按板端接收次序递增；source_frame 是
 * 采集时刻的低 24 位毫秒，两台设备的时钟不必同步。valid=0 表示没有可用中心。 */
typedef struct
{
    uint8_t valid;
    int16_t x, y;         /**< 归一化图像中心乘 10000。 */
    uint32_t received_ms; /**< STM32 收到本次消息的时刻。 */
    uint32_t sequence;
    uint32_t source_frame;
} Visual_Observation_t;

/* 新阶段仅接收边界后的消息。同一图像的周期重发及乱序旧图不会触发新动作。 */
static inline uint8_t Visual_Observation_Is_New(const Visual_Observation_t *observation,
    uint32_t sequence_boundary, uint8_t source_known, uint32_t source_boundary,
    uint32_t now_ms, uint32_t timeout_ms)
{
    const uint32_t sequence_delta = observation->sequence - sequence_boundary;
    const uint32_t source_delta = (observation->source_frame - source_boundary) & 0xffffffU;
    return observation->valid && sequence_delta != 0U && sequence_delta <= INT32_MAX &&
           (uint32_t)(now_ms - observation->received_ms) < timeout_ms &&
           (!source_known || (source_delta != 0U && source_delta < 0x800000U));
}

#endif
