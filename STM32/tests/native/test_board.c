#include <assert.h>
#include <math.h>
#include <stdio.h>
#include "adc.h"
#include "tim.h"
#include "main.h"
#include "bsp_adc.h"
#include "bsp_led.h"
#include "bsp_buzzer.h"

ADC_HandleTypeDef hadc3;
TIM_HandleTypeDef htim4;
static HAL_StatusTypeDef start_result, poll_result, stop_result;
static uint32_t raw_adc = 2048U;
static uint32_t starts, polls, reads, stops;
static uint8_t led_on[3];

/** @brief 模拟单次ADC启动。 @param[in] adc ADC3句柄。 @return 注入状态。 */
HAL_StatusTypeDef HAL_ADC_Start(ADC_HandleTypeDef *adc)
{
    assert(adc == &hadc3);
    ++starts;
    return start_result;
}
/** @brief 核对轮询有界且仅在成功启动后执行。
 * @param[in] adc ADC3句柄。 @param[in] timeout 毫秒超时。 @return 注入状态。
 */
HAL_StatusTypeDef HAL_ADC_PollForConversion(ADC_HandleTypeDef *adc, uint32_t timeout)
{
    assert(adc == &hadc3 && start_result == HAL_OK && timeout == 1U);
    ++polls;
    return poll_result;
}
/** @brief 只有当次转换完成才能读取结果。 @param[in] adc ADC3句柄。 @return 原始12位样本。 */
uint32_t HAL_ADC_GetValue(ADC_HandleTypeDef *adc)
{
    assert(adc == &hadc3 && start_result == HAL_OK && poll_result == HAL_OK);
    ++reads;
    return raw_adc;
}
/** @brief 记录停止转换清理。 @param[in] adc ADC3句柄。 @return 注入状态。 */
HAL_StatusTypeDef HAL_ADC_Stop(ADC_HandleTypeDef *adc)
{
    assert(adc == &hadc3);
    ++stops;
    return stop_result;
}
/** @brief 记录PWM输出。 @param[in] h TIM4。 @param[in] channel CH3。 @param[in] value 比较值。 */
void test_timer_compare(TIM_HandleTypeDef *h, uint32_t channel, uint32_t value)
{
    assert(h == &htim4 && channel == TIM_CHANNEL_3);
    h->Compare[channel / 4U] = value;
}
/** @brief 从低电平初值模拟三灯同时翻转；高电平记为亮。
 * @param[in] port GPIO端口。 @param[in] pin GPIO引脚。
 */
void HAL_GPIO_TogglePin(void *port, uint16_t pin)
{
    if (port == LED_RED_GPIO_Port && pin == LED_RED_Pin) led_on[0] ^= 1U;
    else if (port == LED_GREEN_GPIO_Port && pin == LED_GREEN_Pin) led_on[1] ^= 1U;
    else { assert(port == LED_BLUE_GPIO_Port && pin == LED_BLUE_Pin); led_on[2] ^= 1U; }
}

/** @brief 验证真实BSP的电压换算/失败清理，及基本灯光和蜂鸣输出。 @return 成功为0。 */
int main(void)
{
    assert(fabsf(get_battery_voltage() - 16.654066f) < 0.0001f);
    assert(starts == 1U && polls == 1U && reads == 1U && stops == 1U);
    poll_result = HAL_TIMEOUT;
    raw_adc = 4095U;
    assert(isnan(get_battery_voltage())); /* 不能复用上次16.65V或超时后的旧ADC寄存器。 */
    assert(starts == 2U && polls == 2U && reads == 1U && stops == 2U);
    start_result = HAL_ERROR;
    assert(isnan(get_battery_voltage()));
    assert(starts == 3U && polls == 2U && reads == 1U && stops == 3U);
    start_result = poll_result = HAL_OK;
    stop_result = HAL_ERROR;
    assert(isnan(get_battery_voltage()));
    assert(starts == 4U && polls == 3U && reads == 2U && stops == 4U);
    stop_result = HAL_OK;
    assert(fabsf(get_battery_voltage() - 33.3f) < 0.0001f);

    bsp_led_toggle();
    assert(led_on[0] && led_on[1] && led_on[2]);
    buzzer_on(83U, 125U);
    assert(htim4.Prescaler == 83U && htim4.Compare[2] == 125U);
    buzzer_off();
    assert(htim4.Compare[2] == 0U);
    puts("PASS board: ADC conversion and failure cleanup/no stale values, active-high LEDs, buzzer off");
    return 0;
}
