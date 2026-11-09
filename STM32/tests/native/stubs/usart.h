#ifndef TEST_USART_H
#define TEST_USART_H

#include "main.h"

typedef struct { uint32_t unused; } UART_HandleTypeDef;
typedef enum
{
    HAL_UART_STATE_READY = 0x20U,
    HAL_UART_STATE_BUSY_TX = 0x21U
} HAL_UART_StateTypeDef;

extern UART_HandleTypeDef huart6;
HAL_UART_StateTypeDef HAL_UART_GetState(const UART_HandleTypeDef *huart);
HAL_StatusTypeDef HAL_UART_Transmit_DMA(UART_HandleTypeDef *huart, const uint8_t *data, uint16_t size);

#endif
