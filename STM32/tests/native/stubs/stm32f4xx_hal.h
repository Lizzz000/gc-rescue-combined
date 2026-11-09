#ifndef TEST_STM32_HAL_H
#define TEST_STM32_HAL_H
#include "can.h"
typedef struct { uint32_t unused; } SPI_HandleTypeDef;
typedef struct { uint32_t unused; } I2C_HandleTypeDef;
typedef enum { GPIO_PIN_RESET, GPIO_PIN_SET } GPIO_PinState;
#define I2C_MEMADD_SIZE_8BIT 1U
void HAL_GPIO_WritePin(void *port, uint16_t pin, GPIO_PinState state);
void HAL_GPIO_TogglePin(void *port, uint16_t pin);
HAL_StatusTypeDef HAL_SPI_TransmitReceive(SPI_HandleTypeDef *spi, uint8_t *tx,
                                        uint8_t *rx, uint16_t size, uint32_t timeout);
HAL_StatusTypeDef HAL_I2C_Mem_Read(I2C_HandleTypeDef *i2c, uint16_t address, uint16_t reg,
                                 uint16_t address_size, uint8_t *data, uint16_t length, uint32_t timeout);
HAL_StatusTypeDef HAL_I2C_Mem_Write(I2C_HandleTypeDef *i2c, uint16_t address, uint16_t reg,
                                  uint16_t address_size, uint8_t *data, uint16_t length, uint32_t timeout);
#endif
