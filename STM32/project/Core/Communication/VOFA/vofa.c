/** @file vofa.c
 * @brief 按 VOFA+ 官方 JustFloat 格式发送姿态或现场测试快照。
 */
#include "vofa.h"
#include "usart.h"
#include <string.h>

/* 静态缓冲区在 DMA 发送期间保持有效；帧尾为 JustFloat 固定标记。 */
static uint8_t vofa_tx_buffer[(VOFA_MAX_CHANNELS + 1U) * sizeof(float)];
static const uint8_t vofa_frame_tail[4] = {0x00, 0x00, 0x80, 0x7F};

void vofa_send_channels(const float *values, uint8_t count)
{
    if (values == NULL || count == 0U || count > VOFA_MAX_CHANNELS ||
        HAL_UART_GetState(&huart6) != HAL_UART_STATE_READY)
    {
        return;
    }

    /* STM32F407 的 float 为 32 位小端格式，直接复制到对应通道。 */
    const uint16_t data_size = (uint16_t)(count * sizeof(float));
    memcpy(vofa_tx_buffer, values, data_size);
    memcpy(&vofa_tx_buffer[data_size], vofa_frame_tail, sizeof(vofa_frame_tail));

    (void)HAL_UART_Transmit_DMA(&huart6, vofa_tx_buffer,
                              (uint16_t)(data_size + sizeof(vofa_frame_tail)));
}

void vofa_send(float roll, float pitch, float yaw)
{
    const float values[3] = {roll, pitch, yaw};
    vofa_send_channels(values, 3U);
}
