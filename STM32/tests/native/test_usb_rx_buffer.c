#include <assert.h>
#include <stdio.h>
#include <string.h>

#include "usb_rx_buffer.h"

static void test_all_positions(void)
{
    static const uint32_t lengths[] = {0U, 1U, 2U, 14U, 63U, 64U, 65U, 255U, 512U, 1022U, 1023U};
    uint8_t input[USB_RX_RING_SIZE];
    for (uint32_t i = 0U; i < sizeof(input); ++i)
        input[i] = (uint8_t)(i * 37U + i / 256U);

    /* 每个环起点均验证 USB 包长、环尾和最大容量，不依赖实现的分段方式。 */
    for (uint16_t start = 0U; start < USB_RX_RING_SIZE; ++start)
    {
        for (uint32_t sample = 0U; sample < sizeof(lengths) / sizeof(lengths[0]); ++sample)
        {
            UsbRxBuffer buffer = {0};
            buffer.head = buffer.tail = start;
            const uint32_t length = lengths[sample];
            UsbRxBuffer_Push(&buffer, input, length, 25U);
            assert(buffer.generation == 0U && buffer.overflow_count == 0U);
            uint8_t byte = 0U;
            for (uint32_t i = 0U; i < length; ++i)
            {
                assert(UsbRxBuffer_Pop(&buffer, &byte, NULL));
                assert(byte == input[i]);
            }
            assert(!UsbRxBuffer_Pop(&buffer, &byte, NULL));
        }
    }
}

static void test_capacity_and_empty_input(void)
{
    UsbRxBuffer buffer = {0};
    uint8_t input[USB_RX_RING_SIZE];
    uint8_t byte = 0U;
    for (uint32_t i = 0U; i < sizeof(input); ++i)
        input[i] = (uint8_t)i;

    /* NULL/0 不复制、不改变空环或已满环，也不能误触发溢出。 */
    UsbRxBuffer_Push(&buffer, NULL, 0U, 25U);
    assert(buffer.head == 0U && buffer.tail == 0U);
    assert(buffer.generation == 0U && buffer.overflow_count == 0U);
    buffer.head = buffer.tail = USB_RX_RING_SIZE - 20U;
    UsbRxBuffer_Push(&buffer, input, 64U, 25U);
    UsbRxBuffer_Push(&buffer, input + 64U, USB_RX_RING_SIZE - 65U, 25U);
    const uint16_t full_head = buffer.head, full_tail = buffer.tail;
    UsbRxBuffer_Push(&buffer, NULL, 0U, 25U);
    assert(buffer.head == full_head && buffer.tail == full_tail);
    assert(buffer.generation == 0U && buffer.overflow_count == 0U);
    for (uint32_t i = 0U; i < USB_RX_RING_SIZE - 1U; ++i)
    {
        assert(UsbRxBuffer_Pop(&buffer, &byte, NULL));
        assert(byte == input[i]);
    }
    assert(!UsbRxBuffer_Pop(&buffer, &byte, NULL));

    /* 满环追加一字节，以及单包超过容量，均丢弃整批并推进代数。 */
    UsbRxBuffer_Push(&buffer, input, USB_RX_RING_SIZE - 1U, 25U);
    UsbRxBuffer_Push(&buffer, input, 1U, 25U);
    assert(buffer.generation == 1U && buffer.overflow_count == 1U);
    assert(!UsbRxBuffer_Pop(&buffer, &byte, NULL));
    UsbRxBuffer_Push(&buffer, input, USB_RX_RING_SIZE, 25U);
    assert(buffer.generation == 2U && buffer.overflow_count == 2U);
    assert(!UsbRxBuffer_Pop(&buffer, &byte, NULL));
    UsbRxBuffer_Push(&buffer, input, UINT32_MAX, 25U);
    assert(buffer.generation == 3U && buffer.overflow_count == 3U);
    assert(!UsbRxBuffer_Pop(&buffer, &byte, NULL));
    UsbRxBuffer_Push(&buffer, input, 64U, 25U);
    for (uint32_t i = 0U; i < 64U; ++i)
    {
        assert(UsbRxBuffer_Pop(&buffer, &byte, NULL));
        assert(byte == input[i]);
    }
    assert(!UsbRxBuffer_Pop(&buffer, &byte, NULL));
}

static void test_wrap_and_reset(void)
{
    UsbRxBuffer buffer = {0};
    uint8_t input[USB_RX_RING_SIZE] = {0};
    uint8_t byte = 0U;
    for (uint32_t i = 0U; i < sizeof(input); ++i)
        input[i] = (uint8_t)i;

    UsbRxBuffer_Push(&buffer, input, USB_RX_RING_SIZE - 1U, 25U);
    for (uint32_t i = 0U; i < USB_RX_RING_SIZE - 2U; ++i)
    {
        assert(UsbRxBuffer_Pop(&buffer, &byte, NULL));
        assert(byte == input[i]);
    }
    UsbRxBuffer_Push(&buffer, input, 8U, 25U);
    assert(UsbRxBuffer_Pop(&buffer, &byte, NULL) && byte == input[USB_RX_RING_SIZE - 2U]);
    for (uint32_t i = 0U; i < 8U; ++i)
        assert(UsbRxBuffer_Pop(&buffer, &byte, NULL) && byte == input[i]);
    assert(!UsbRxBuffer_Pop(&buffer, &byte, NULL));

    UsbRxBuffer_Push(&buffer, input, USB_RX_RING_SIZE - 4U, 25U);
    UsbRxBuffer_Push(&buffer, input, 4U, 25U);
    assert(buffer.overflow_count == 1U && buffer.generation == 1U);
    assert(!UsbRxBuffer_Pop(&buffer, &byte, NULL));
    UsbRxBuffer_Push(&buffer, input, 8U, 25U);
    UsbRxBuffer_Reset(&buffer);
    assert(buffer.generation == 2U && !UsbRxBuffer_Pop(&buffer, &byte, NULL));
    UsbRxBuffer_Push(&buffer, input, 1U, 25U);
    assert(UsbRxBuffer_Pop(&buffer, &byte, NULL) && byte == input[0]);
}

static void test_arrival_tick_wrap(void)
{
    UsbRxBuffer buffer = {0};
    buffer.head = buffer.tail = USB_RX_RING_SIZE - 1U;
    const uint8_t bytes[] = {7U, 8U};
    UsbRxBuffer_Push(&buffer, bytes, 2U, UINT32_MAX - 29U);
    UsbRxBuffer_Push(&buffer, bytes, 1U, 40U);
    uint32_t arrival_ms;
    uint8_t byte;
    assert(UsbRxBuffer_Pop(&buffer, &byte, &arrival_ms) && byte == 7U);
    assert(!UsbRxBuffer_Expired(69U, arrival_ms));
    assert(UsbRxBuffer_Pop(&buffer, &byte, &arrival_ms) && byte == 8U);
    assert(UsbRxBuffer_Expired(70U, arrival_ms));
    assert(UsbRxBuffer_Pop(&buffer, &byte, &arrival_ms) && byte == 7U);
    assert(arrival_ms == 40U && !UsbRxBuffer_Expired(70U, arrival_ms));
}

int main(void)
{
    test_all_positions();
    test_capacity_and_empty_input();
    test_wrap_and_reset();
    test_arrival_tick_wrap();
    puts("PASS: USB ring all positions, capacity, empty input, overflow recovery and reset");
    return 0;
}
