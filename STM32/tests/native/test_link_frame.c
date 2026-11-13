#include <assert.h>
#include <stdio.h>
#include <string.h>

#include "link_frame.h"

static void round_trip(uint8_t type, const uint8_t *payload, uint8_t length)
{
    uint8_t encoded[LINK_FRAME_MAX_BYTES];
    const uint8_t size = LinkFrame_Encode(type, payload, length, encoded);
    assert(size == (uint8_t)(6U + length));
    assert(encoded[0] == LINK_FRAME_SOF && encoded[1] == LINK_FRAME_VERSION);
    assert(encoded[2] == type && encoded[3] == length);
    LinkParser parser = {0};
    LinkFrame frame = {0};
    uint8_t produced = 0U;
    const uint8_t noise[] = {0x11, 0x22, LINK_FRAME_SOF, 0x00};
    for (unsigned i = 0U; i < sizeof(noise); ++i)
        assert(LinkParser_Feed(&parser, noise[i], &frame) == 0U);
    for (uint8_t i = 0U; i < size; ++i)
        produced = LinkParser_Feed(&parser, encoded[i], &frame);
    assert(produced == 1U);
    assert(frame.type == type && frame.length == length);
    if (length > 0U)
        assert(memcmp(frame.data, payload, length) == 0);
}

int main(void)
{
    const uint8_t heartbeat[] = {1U, 0U, 0U, 0U, 0U, 0U, 0U, 0U};
    const uint8_t manual[] = {1U, 0xFEU, 0xFFU, 0U, 0U, 0xE8U, 0x03U};
    round_trip(1U, NULL, 0U);
    round_trip(1U, heartbeat, 8U);
    round_trip(2U, manual, 7U);

    uint8_t good[LINK_FRAME_MAX_BYTES];
    const uint8_t good_size = LinkFrame_Encode(3U, heartbeat, 8U, good);
    good[6] ^= 0xFFU;
    LinkParser parser = {0};
    LinkFrame frame = {0};
    uint8_t produced = 0U;
    for (uint8_t i = 0U; i < good_size; ++i)
        produced |= LinkParser_Feed(&parser, good[i], &frame);
    assert(produced == 0U);
    const uint8_t again = LinkFrame_Encode(3U, heartbeat, 8U, good);
    for (uint8_t i = 0U; i < again; ++i)
        produced = LinkParser_Feed(&parser, good[i], &frame);
    assert(produced == 1U && frame.type == 3U && frame.length == 8U);
    puts("PASS");
    return 0;
}
