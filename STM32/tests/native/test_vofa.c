#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "usart.h"
#include "vofa.h"

UART_HandleTypeDef huart6;
static HAL_UART_StateTypeDef state = HAL_UART_STATE_READY;
static const uint8_t *in_flight;
static uint32_t sends;

/** @brief 模拟USART6发送状态。 @param[in] huart 串口句柄。 @return 当前状态。 */
HAL_UART_StateTypeDef HAL_UART_GetState(const UART_HandleTypeDef *huart)
{
    assert(huart == &huart6);
    return state;
}

/** @brief 保存DMA持有的原缓冲指针，模拟异步串口发送。
 * @param[in] huart 串口句柄。 @param[in] data 帧缓冲。 @param[in] size 帧长度。
 * @return HAL_OK。
 */
HAL_StatusTypeDef HAL_UART_Transmit_DMA(UART_HandleTypeDef *huart, const uint8_t *data, uint16_t size)
{
    assert(huart == &huart6 && state == HAL_UART_STATE_READY && size == 16U);
    in_flight = data;
    ++sends;
    state = HAL_UART_STATE_BUSY_TX;
    return HAL_OK;
}

/** @brief 验证JustFloat黄金字节序、通道顺序与忙时原帧缓冲不变。 @return 成功为0。 */
int main(void)
{
    const uint8_t first[16] = {
        0x00,0x00,0x80,0x3F, /* roll 1.0° */
        0x00,0x00,0x20,0xC0, /* pitch -2.5° */
        0x00,0x00,0xB4,0x42, /* yaw 90.0° */
        0x00,0x00,0x80,0x7F
    };
    const uint8_t second[16] = {
        0x00,0x00,0x34,0xC3, /* roll -180.0° */
        0x00,0x00,0x00,0x00, /* pitch 0.0° */
        0x00,0x00,0x80,0x3E, /* yaw 0.25° */
        0x00,0x00,0x80,0x7F
    };
    vofa_send(1.0f, -2.5f, 90.0f);
    assert(sends == 1U && memcmp(in_flight, first, sizeof(first)) == 0);
    const uint8_t *first_buffer = in_flight;
    vofa_send(-180.0f, 0.0f, 0.25f);
    assert(sends == 1U && in_flight == first_buffer);
    assert(memcmp(in_flight, first, sizeof(first)) == 0);
    state = HAL_UART_STATE_READY;
    vofa_send(-180.0f, 0.0f, 0.25f);
    assert(sends == 2U && in_flight == first_buffer);
    assert(memcmp(in_flight, second, sizeof(second)) == 0);
    puts("PASS VOFA: JustFloat frame, DMA busy preserves in-flight data");
    return 0;
}
