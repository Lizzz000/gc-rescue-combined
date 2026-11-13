#include <assert.h>
#include <stdio.h>
#include <string.h>

#include "usb_rx_buffer.h"

int main(void)
{
    UsbRxBuffer buffer = {0};
    uint8_t input[USB_RX_RING_SIZE] = {0};
    uint8_t byte = 0U;
    for (uint32_t i = 0U; i < sizeof(input); ++i)
        input[i] = (uint8_t)i;

    UsbRxBuffer_Push(&buffer, input, USB_RX_RING_SIZE - 1U);
    for (uint32_t i = 0U; i < USB_RX_RING_SIZE - 2U; ++i)
    {
        assert(UsbRxBuffer_Pop(&buffer, &byte));
        assert(byte == input[i]);
    }
    UsbRxBuffer_Push(&buffer, input, 8U);
    assert(UsbRxBuffer_Pop(&buffer, &byte) && byte == input[USB_RX_RING_SIZE - 2U]);
    for (uint32_t i = 0U; i < 8U; ++i)
        assert(UsbRxBuffer_Pop(&buffer, &byte) && byte == input[i]);
    assert(!UsbRxBuffer_Pop(&buffer, &byte));

    UsbRxBuffer_Push(&buffer, input, USB_RX_RING_SIZE - 4U);
    UsbRxBuffer_Push(&buffer, input, 4U);
    assert(buffer.overflow_count == 1U && buffer.generation == 1U);
    assert(!UsbRxBuffer_Pop(&buffer, &byte));
    UsbRxBuffer_Push(&buffer, input, 8U);
    UsbRxBuffer_Reset(&buffer);
    assert(buffer.generation == 2U && !UsbRxBuffer_Pop(&buffer, &byte));
    UsbRxBuffer_Push(&buffer, input, 1U);
    assert(UsbRxBuffer_Pop(&buffer, &byte) && byte == input[0]);
    puts("PASS: USB ring wrap, overflow and reset");
    return 0;
}
