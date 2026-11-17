#include "link_frame.h"

#include <string.h>

static uint16_t crc16_ibm(const uint8_t *data, uint8_t length)
{
    /* 两次4位查表等价于8次逐位递推，仅占32字节常量。 */
    static const uint16_t crc_nibble_table[16] = {
        0x0000U, 0xCC01U, 0xD801U, 0x1400U, 0xF001U, 0x3C00U, 0x2800U, 0xE401U,
        0xA001U, 0x6C00U, 0x7800U, 0xB401U, 0x5000U, 0x9C01U, 0x8801U, 0x4400U
    };
    uint16_t crc = 0xFFFFU;
    for (uint8_t i = 0U; i < length; ++i)
    {
        crc ^= data[i];
        crc = (uint16_t)((crc >> 4U) ^ crc_nibble_table[crc & 0x0FU]);
        crc = (uint16_t)((crc >> 4U) ^ crc_nibble_table[crc & 0x0FU]);
    }
    return crc;
}

uint8_t LinkFrame_Encode(uint8_t type, const uint8_t *data, uint8_t length, uint8_t *out)
{
    if (out == NULL || length > LINK_FRAME_MAX_PAYLOAD || (length > 0U && data == NULL))
    {
        return 0U;
    }
    out[0] = LINK_FRAME_SOF;
    out[1] = LINK_FRAME_VERSION;
    out[2] = type;
    out[3] = length;
    if (length > 0U)
    {
        memcpy(out + 4U, data, length);
    }
    const uint16_t crc = crc16_ibm(out + 1U, (uint8_t)(3U + length));
    out[4U + length] = (uint8_t)crc;
    out[5U + length] = (uint8_t)(crc >> 8U);
    return (uint8_t)(6U + length);
}

/* 扫描到下一帧头后只搬移一次，first之前的字节已消费或已判坏。 */
static void discard_to_sof(LinkParser *parser, uint8_t first)
{
    uint8_t skip = first;
    while (skip < parser->count && parser->buffer[skip] != LINK_FRAME_SOF)
    {
        ++skip;
    }
    parser->count = (uint8_t)(parser->count - skip);
    if (parser->count > 0U && skip > 0U)
    {
        memmove(parser->buffer, parser->buffer + skip, parser->count);
    }
}

uint8_t LinkParser_Feed(LinkParser *parser, uint8_t byte, LinkFrame *out)
{
    if (parser == NULL || out == NULL)
    {
        return 0U;
    }
    if (parser->count == 0U && byte != LINK_FRAME_SOF)
    {
        return 0U;
    }
    if (parser->count >= LINK_FRAME_MAX_BYTES)
    {
        parser->count = 0U;
    }
    parser->buffer[parser->count++] = byte;

    while (parser->count >= 4U)
    {
        const uint8_t payload_length = parser->buffer[3];
        if (parser->buffer[0] != LINK_FRAME_SOF || parser->buffer[1] != LINK_FRAME_VERSION ||
            payload_length > LINK_FRAME_MAX_PAYLOAD)
        {
            discard_to_sof(parser, 1U);
            continue;
        }
        const uint8_t frame_length = (uint8_t)(6U + payload_length);
        if (parser->count < frame_length)
        {
            return 0U;
        }
        const uint16_t expected_crc = crc16_ibm(parser->buffer + 1U, (uint8_t)(3U + payload_length));
        const uint16_t received_crc = (uint16_t)parser->buffer[4U + payload_length] |
                                      ((uint16_t)parser->buffer[5U + payload_length] << 8U);
        if (expected_crc != received_crc)
        {
            discard_to_sof(parser, 1U);
            continue;
        }
        out->type = parser->buffer[2];
        out->length = payload_length;
        memset(out->data, 0, sizeof(out->data));
        if (payload_length > 0U)
        {
            memcpy(out->data, parser->buffer + 4U, payload_length);
        }
        discard_to_sof(parser, frame_length);
        return 1U;
    }
    return 0U;
}
