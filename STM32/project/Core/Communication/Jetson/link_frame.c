#include "link_frame.h"

#include <string.h>

static uint16_t crc16_ibm(const uint8_t *data, uint8_t length)
{
    uint16_t crc = 0xFFFFU;
    for (uint8_t i = 0U; i < length; ++i)
    {
        crc ^= data[i];
        for (uint8_t bit = 0U; bit < 8U; ++bit)
        {
            if ((crc & 1U) != 0U)
                crc = (uint16_t)((crc >> 1U) ^ 0xA001U);
            else
                crc = (uint16_t)(crc >> 1U);
        }
    }
    return crc;
}

uint8_t LinkFrame_Encode(uint8_t type, const uint8_t *data, uint8_t length, uint8_t *out)
{
    if (out == NULL || length > LINK_FRAME_MAX_PAYLOAD || (length > 0U && data == NULL))
        return 0U;
    out[0] = LINK_FRAME_SOF;
    out[1] = LINK_FRAME_VERSION;
    out[2] = type;
    out[3] = length;
    if (length > 0U)
        memcpy(out + 4U, data, length);
    const uint16_t crc = crc16_ibm(out + 1U, (uint8_t)(3U + length));
    out[4U + length] = (uint8_t)crc;
    out[5U + length] = (uint8_t)(crc >> 8U);
    return (uint8_t)(6U + length);
}

static void drop_until_sof(LinkParser *parser)
{
    while (parser->count > 0U && parser->buffer[0] != LINK_FRAME_SOF)
    {
        memmove(parser->buffer, parser->buffer + 1U, parser->count - 1U);
        --parser->count;
    }
}

static void drop_one(LinkParser *parser)
{
    if (parser->count == 0U)
        return;
    memmove(parser->buffer, parser->buffer + 1U, parser->count - 1U);
    --parser->count;
    drop_until_sof(parser);
}

uint8_t LinkParser_Feed(LinkParser *parser, uint8_t byte, LinkFrame *out)
{
    if (parser == NULL || out == NULL)
        return 0U;
    if (parser->count == 0U && byte != LINK_FRAME_SOF)
        return 0U;
    if (parser->count >= LINK_FRAME_MAX_BYTES)
        parser->count = 0U;
    parser->buffer[parser->count++] = byte;

    while (parser->count >= 4U)
    {
        const uint8_t payload = parser->buffer[3];
        if (parser->buffer[0] != LINK_FRAME_SOF || parser->buffer[1] != LINK_FRAME_VERSION ||
            payload > LINK_FRAME_MAX_PAYLOAD)
        {
            drop_one(parser);
            continue;
        }
        const uint8_t total = (uint8_t)(6U + payload);
        if (parser->count < total)
            return 0U;
        const uint16_t expect = crc16_ibm(parser->buffer + 1U, (uint8_t)(3U + payload));
        const uint16_t actual = (uint16_t)parser->buffer[4U + payload] |
                                ((uint16_t)parser->buffer[5U + payload] << 8U);
        if (expect != actual)
        {
            drop_one(parser);
            continue;
        }
        out->type = parser->buffer[2];
        out->length = payload;
        memset(out->data, 0, sizeof(out->data));
        if (payload > 0U)
            memcpy(out->data, parser->buffer + 4U, payload);
        const uint8_t extra = (uint8_t)(parser->count - total);
        if (extra > 0U)
            memmove(parser->buffer, parser->buffer + total, extra);
        parser->count = extra;
        drop_until_sof(parser);
        return 1U;
    }
    return 0U;
}
