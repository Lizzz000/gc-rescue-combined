#ifndef TEST_TIM_H
#define TEST_TIM_H
#include "stm32f4xx_hal.h"
typedef struct
{
    uint32_t Counter, Prescaler, Autoreload;
    uint32_t Compare[4];
} TIM_HandleTypeDef;
extern TIM_HandleTypeDef htim2, htim4, htim10;
#define TIM_CHANNEL_1 0U
#define TIM_CHANNEL_3 8U
#define __HAL_TIM_SET_PRESCALER(h, value) ((h)->Prescaler = (value))
#define __HAL_TIM_GET_AUTORELOAD(h) ((h)->Autoreload)
void test_timer_compare(TIM_HandleTypeDef *h, uint32_t channel, uint32_t value);
#define __HAL_TIM_SET_COMPARE(h, channel, value) test_timer_compare(h, channel, value)
#define __HAL_TIM_SetCompare(h, channel, value) __HAL_TIM_SET_COMPARE(h, channel, value)
HAL_StatusTypeDef HAL_TIM_PWM_Start(TIM_HandleTypeDef *h, uint32_t channel);
extern uint32_t timer_start_count;
/** @brief 读取模拟微秒时钟的低32位，并让忙等时间前进。 @param[in] h TIM2句柄。 @return 计数。 */
uint32_t test_timer_counter(TIM_HandleTypeDef *h);
#define __HAL_TIM_GET_COUNTER(h) test_timer_counter(h)
/** @brief 模拟启动已有TIM2。 @param[in] h 句柄。 @return HAL_OK。 */
static inline HAL_StatusTypeDef HAL_TIM_Base_Start(TIM_HandleTypeDef *h)
{ (void)h; ++timer_start_count; return HAL_OK; }
#endif
