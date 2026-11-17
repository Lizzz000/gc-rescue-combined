#ifndef USB_RX_BUFFER_H
#define USB_RX_BUFFER_H

#include <stdint.h>
#include <string.h>

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
    if (length == 0U)
        return;
    const uint16_t head = buffer->head;
    const uint16_t used = (uint16_t)((head + USB_RX_RING_SIZE - buffer->tail) % USB_RX_RING_SIZE);
    if (length > USB_RX_RING_SIZE - 1U - used)
    {
        ++buffer->overflow_count;
        UsbRxBuffer_Reset(buffer);
        return;
    }
    /* 任务读取时屏蔽 USB 中断；整包复制后一次更新 head，避免逐字节访问 volatile。 */
    const uint32_t first = length < USB_RX_RING_SIZE - head ? length : USB_RX_RING_SIZE - head;
    memcpy(buffer->data + head, data, first);
    if (length > first)
        memcpy(buffer->data, data + first, length - first);
    buffer->head = (uint16_t)((head + length) % USB_RX_RING_SIZE);
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
