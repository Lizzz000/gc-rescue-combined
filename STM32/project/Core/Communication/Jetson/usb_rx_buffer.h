#ifndef USB_RX_BUFFER_H
#define USB_RX_BUFFER_H

#include <stdint.h>

#define USB_RX_RING_SIZE 1024U

/* 写入与重置由 USB 中断执行；任务读取时需屏蔽该中断。 */
typedef struct
{
    uint8_t data[USB_RX_RING_SIZE];
    volatile uint16_t head;
    volatile uint16_t tail;
    volatile uint32_t generation;
    volatile uint32_t overflow_count;
} UsbRxBuffer;

static inline void UsbRxBuffer_Reset(UsbRxBuffer *buffer)
{
    buffer->head = 0U;
    buffer->tail = 0U;
    ++buffer->generation;
}

static inline void UsbRxBuffer_Push(UsbRxBuffer *buffer, const uint8_t *data, uint32_t length)
{
    const uint16_t used = (uint16_t)((buffer->head + USB_RX_RING_SIZE - buffer->tail) % USB_RX_RING_SIZE);
    if (length > USB_RX_RING_SIZE - 1U - used)
    {
        ++buffer->overflow_count;
        UsbRxBuffer_Reset(buffer);
        return;
    }
    for (uint32_t i = 0U; i < length; ++i)
    {
        buffer->data[buffer->head] = data[i];
        buffer->head = (uint16_t)((buffer->head + 1U) % USB_RX_RING_SIZE);
    }
}

static inline uint8_t UsbRxBuffer_Pop(UsbRxBuffer *buffer, uint8_t *byte)
{
    if (buffer->head == buffer->tail)
        return 0U;
    *byte = buffer->data[buffer->tail];
    buffer->tail = (uint16_t)((buffer->tail + 1U) % USB_RX_RING_SIZE);
    return 1U;
}

#endif
