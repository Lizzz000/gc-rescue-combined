#ifndef TEST_ADC_H
#define TEST_ADC_H
#include "stm32f4xx_hal.h"
typedef struct { uint32_t unused; } ADC_HandleTypeDef;
extern ADC_HandleTypeDef hadc3;
HAL_StatusTypeDef HAL_ADC_Start(ADC_HandleTypeDef *adc);
HAL_StatusTypeDef HAL_ADC_PollForConversion(ADC_HandleTypeDef *adc, uint32_t timeout);
uint32_t HAL_ADC_GetValue(ADC_HandleTypeDef *adc);
HAL_StatusTypeDef HAL_ADC_Stop(ADC_HandleTypeDef *adc);
#endif
