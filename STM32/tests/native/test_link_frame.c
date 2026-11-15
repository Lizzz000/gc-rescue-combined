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

static uint16_t reference_crc(const uint8_t *data, unsigned length)
{
    uint16_t crc = 0xFFFFU;
    for (unsigned i = 0U; i < length; ++i)
    {
        crc ^= data[i];
        for (unsigned bit = 0U; bit < 8U; ++bit)
            crc = (uint16_t)((crc >> 1U) ^ ((crc & 1U) ? 0xA001U : 0U));
    }
    return crc;
}

static void test_crc_vectors(void)
{
    for (unsigned value = 0U; value <= 65535U; ++value)
    {
        const uint8_t payload[2] = {(uint8_t)value, (uint8_t)(value >> 8U)};
        uint8_t encoded[LINK_FRAME_MAX_BYTES];
        assert(LinkFrame_Encode((uint8_t)(value >> 5U), payload, 2U, encoded) == 8U);
        const uint16_t crc = reference_crc(encoded + 1U, 5U);
        assert(encoded[6] == (uint8_t)crc && encoded[7] == (uint8_t)(crc >> 8U));
    }
    uint8_t bytes[8], encoded[LINK_FRAME_MAX_BYTES];
    for (unsigned length = 0U; length <= 8U; ++length)
    for (unsigned seed = 0U; seed <= 255U; ++seed)
    {
        for (unsigned i = 0U; i < 8U; ++i) bytes[i] = (uint8_t)(seed + i * 37U);
        const uint8_t size = LinkFrame_Encode((uint8_t)seed, bytes, (uint8_t)length, encoded);
        const uint16_t crc = reference_crc(encoded + 1U, length + 3U);
        assert(encoded[size - 2U] == (uint8_t)crc && encoded[size - 1U] == (uint8_t)(crc >> 8U));
        round_trip((uint8_t)seed, bytes, (uint8_t)length);
    }
}

static void test_single_bit_damage(void)
{
    const uint8_t payload[8] = {1U, 0xA5U, 0xFFU, 0U, 3U, 8U, 127U, 128U};
    uint8_t good[LINK_FRAME_MAX_BYTES], bad[LINK_FRAME_MAX_BYTES];
    const uint8_t length = LinkFrame_Encode(7U, payload, 8U, good);
    for (unsigned bit = 0U; bit < length * 8U; ++bit)
    {
        memcpy(bad, good, length);
        bad[bit / 8U] ^= (uint8_t)(1U << (bit % 8U));
        LinkParser parser = {0}; LinkFrame frame = {0};
        unsigned produced = 0U;
        for (unsigned i = 0U; i < length; ++i) produced += LinkParser_Feed(&parser, bad[i], &frame);
        assert(produced == 0U);
        for (unsigned i = 0U; i < length; ++i) produced += LinkParser_Feed(&parser, good[i], &frame);
        assert(produced == 1U && frame.type == 7U && frame.length == 8U);
        assert(memcmp(frame.data, payload, 8U) == 0);
    }
}

int main(void)
{
    test_crc_vectors();
    test_single_bit_damage();
    const uint8_t payload[] = {1U, 0U, 0U, 0U, 0U, 0U, 0U, 0U};
    const uint8_t manual[] = {1U, 0xFEU, 0xFFU, 0U, 0U, 0xE8U, 0x03U};
    round_trip(1U, NULL, 0U);
    round_trip(1U, payload, 8U);
    round_trip(2U, manual, 7U);

    uint8_t good[LINK_FRAME_MAX_BYTES];
    const uint8_t good_size = LinkFrame_Encode(3U, payload, 8U, good);
    good[6] ^= 0xFFU;
    LinkParser parser = {0};
    LinkFrame frame = {0};
    uint8_t produced = 0U;
    for (uint8_t i = 0U; i < good_size; ++i)
        produced |= LinkParser_Feed(&parser, good[i], &frame);
    assert(produced == 0U);
    const uint8_t again = LinkFrame_Encode(3U, payload, 8U, good);
    for (uint8_t i = 0U; i < again; ++i)
        produced = LinkParser_Feed(&parser, good[i], &frame);
    assert(produced == 1U && frame.type == 3U && frame.length == 8U);
    puts("PASS: frame CRC exhaustive two-byte vectors, payload lengths, bit damage and resynchronization");
    return 0;
}
