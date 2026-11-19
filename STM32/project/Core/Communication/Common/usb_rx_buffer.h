#ifndef USB_RX_BUFFER_H
#define USB_RX_BUFFER_H

#include <stdint.h>
#include <string.h>

#define USB_RX_RING_SIZE 1024U
#define USB_RX_MAX_AGE_MS 100U

/* 写入与重置由 USB 中断执行；任务读取时需屏蔽该中断。 */
typedef struct
{
    uint32_t received_ms[USB_RX_RING_SIZE];
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

static inline void UsbRxBuffer_Push(UsbRxBuffer *buffer, const uint8_t *data, uint32_t length,
                                    uint32_t received_ms)
{
    if (length == 0U)
    {
        return;
    }
    const uint16_t head = buffer->head;
    const uint16_t used = (uint16_t)((head + USB_RX_RING_SIZE - buffer->tail) % USB_RX_RING_SIZE);
    if (length > USB_RX_RING_SIZE - 1U - used)
    {
        ++buffer->overflow_count;
        UsbRxBuffer_Reset(buffer);
        return;
    }
    /* 任务读取时屏蔽 USB 中断；整包复制后一次更新 head，避免逐字节访问 volatile。 */
    const uint32_t space_to_end = USB_RX_RING_SIZE - head;
    const uint32_t first_chunk_size = length < space_to_end ? length : space_to_end;
    memcpy(buffer->data + head, data, first_chunk_size);
    if (length > first_chunk_size)
    {
        memcpy(buffer->data, data + first_chunk_size, length - first_chunk_size);
    }
    for (uint32_t i = 0U; i < first_chunk_size; ++i)
        buffer->received_ms[head + i] = received_ms;
    for (uint32_t i = 0U; i < length - first_chunk_size; ++i)
        buffer->received_ms[i] = received_ms;
    buffer->head = (uint16_t)((head + length) % USB_RX_RING_SIZE);
}

static inline uint8_t UsbRxBuffer_Expired(uint32_t now_ms, uint32_t received_ms)
{
    return (uint32_t)(now_ms - received_ms) >= USB_RX_MAX_AGE_MS;
}

static inline uint8_t UsbRxBuffer_Pop(UsbRxBuffer *buffer, uint8_t *byte, uint32_t *received_ms)
{
    if (buffer->head == buffer->tail)
    {
        return 0U;
    }
    *byte = buffer->data[buffer->tail];
    if (received_ms != NULL)
        *received_ms = buffer->received_ms[buffer->tail];
    buffer->tail = (uint16_t)((buffer->tail + 1U) % USB_RX_RING_SIZE);
    return 1U;
}

#endif
