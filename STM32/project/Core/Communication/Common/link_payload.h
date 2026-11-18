#ifndef LINK_PAYLOAD_H
#define LINK_PAYLOAD_H

#include <stddef.h>
#include <stdint.h>

/* 类型 2 的线格式，在 USB 和 USART6 上相同。速度单位为 0.001 m/s 和 0.001 rad/s。 */
#define LINK_VELOCITY_DLC 7U

typedef struct
{
    uint8_t valid;
    int16_t vx, vy, wz;
} Link_Velocity_Frame_t;

static inline int16_t Link_Read_I16(const uint8_t data[2])
{
    const uint16_t bits = (uint16_t)(data[0] | ((uint16_t)data[1] << 8U));
    return bits <= INT16_MAX ? (int16_t)bits : (int16_t)((int32_t)bits - 65536);
}

static inline uint8_t Link_Is_Velocity_Frame(const uint8_t *data, uint8_t len)
{
    return data != NULL && len == LINK_VELOCITY_DLC && data[0] <= 1U;
}

static inline Link_Velocity_Frame_t Link_Decode_Velocity(const uint8_t *data, uint8_t len)
{
    Link_Velocity_Frame_t frame = {0};
    if (Link_Is_Velocity_Frame(data, len) && data[0] == 1U)
    {
        frame.valid = 1U;
        frame.vx = Link_Read_I16(data + 1U);
        frame.vy = Link_Read_I16(data + 3U);
        frame.wz = Link_Read_I16(data + 5U);
    }
    return frame;
}

#endif
