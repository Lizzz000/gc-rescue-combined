/** @file vofa.c
 * @brief 按 VOFA+ 官方 JustFloat 格式发送三个姿态角。
 */
#include "vofa.h"
#include "usart.h"
#include <string.h>

/* 静态缓冲区在 DMA 发送期间保持有效；帧尾为 JustFloat 固定标记。 */
static uint8_t vofa_tx_buffer[16] = {
    0, 0, 0, 0,
    0, 0, 0, 0,
    0, 0, 0, 0,
    0x00, 0x00, 0x80, 0x7F
};

void vofa_send(float roll, float pitch, float yaw)
{
    if (HAL_UART_GetState(&huart6) != HAL_UART_STATE_READY)
    {
        return;
    }

    /* STM32F407 的 float 为 32 位小端格式，直接复制到对应通道。 */
    memcpy(&vofa_tx_buffer[0], &roll, sizeof(float));
    memcpy(&vofa_tx_buffer[4], &pitch, sizeof(float));
    memcpy(&vofa_tx_buffer[8], &yaw, sizeof(float));

    (void)HAL_UART_Transmit_DMA(&huart6, vofa_tx_buffer, sizeof(vofa_tx_buffer));
}
