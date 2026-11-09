#include "transport_servo.h"
#include "tim.h"
#include <stddef.h>

#define SERVO_TIMER_HZ 1000000U
#define SERVO_PERIOD_US 20000U

static uint16_t applied_pulse_us;

#if JETSON_AUTO_ENABLED
#if TRANSPORT_SERVO_OPEN_US < 1 || TRANSPORT_SERVO_OPEN_US >= SERVO_PERIOD_US || \
    TRANSPORT_SERVO_CLOSED_US < 1 || TRANSPORT_SERVO_CLOSED_US >= SERVO_PERIOD_US || \
    TRANSPORT_SERVO_OPEN_US == TRANSPORT_SERVO_CLOSED_US
#error "Set distinct measured TRANSPORT_SERVO_OPEN_US and TRANSPORT_SERVO_CLOSED_US"
#endif

static uint32_t commanded_ms;
#else
#if TRANSPORT_SERVO_CALIBRATION_MIN_US < 1 || \
    TRANSPORT_SERVO_CALIBRATION_MAX_US >= SERVO_PERIOD_US || \
    TRANSPORT_SERVO_CALIBRATION_MIN_US > TRANSPORT_SERVO_CALIBRATION_MAX_US
#error "Invalid TRANSPORT_SERVO_CALIBRATION_MIN_US / MAX_US envelope"
#endif
volatile uint16_t jetson_calibration_pulse_us = 0U;
#endif

/** @brief 核对定时器实际时钟和通道模式，详细约束见头文件。 */
uint8_t Transport_Servo_Is_Available(void)
{
    if (htim1.Instance != TIM1 || htim1.Init.CounterMode != TIM_COUNTERMODE_UP ||
        htim1.Init.RepetitionCounter != 0U || htim1.Instance->ARR != SERVO_PERIOD_US - 1U ||
        (htim1.Instance->CCMR1 & (TIM_CCMR1_CC1S | TIM_CCMR1_OC1M)) != TIM_OCMODE_PWM1 ||
        (htim1.Instance->CCER & TIM_CCER_CC1P) != 0U)
        return 0U;

    RCC_ClkInitTypeDef clock_config;
    uint32_t flash_latency;
    HAL_RCC_GetClockConfig(&clock_config, &flash_latency);
    uint32_t timer_hz = HAL_RCC_GetPCLK2Freq();
    if (clock_config.APB2CLKDivider != RCC_HCLK_DIV1)
        timer_hz *= 2U;
    return timer_hz % SERVO_TIMER_HZ == 0U &&
           htim1.Instance->PSC + 1U == timer_hz / SERVO_TIMER_HZ;
}

/** @brief 提交已校验的脉宽，或停止输出。
 * @param[in] pulse_us 高电平宽度，单位 μs；0 表示关闭 PWM。
 * @return 已设置返回 1，定时器配置或 HAL 操作失败返回 0。
 */
static uint8_t Set_Pulse(uint16_t pulse_us)
{
    if (pulse_us != 0U && !Transport_Servo_Is_Available())
        return 0U;
    if (applied_pulse_us == pulse_us)
        return 1U;

    if (pulse_us == 0U && HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1) != HAL_OK)
        return 0U;
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, pulse_us);
    if (applied_pulse_us == 0U)
    {
        /* 先更新预装载值，再启用输出，后续位置切换在下一 PWM 周期生效。 */
        if (HAL_TIM_GenerateEvent(&htim1, TIM_EVENTSOURCE_UPDATE) != HAL_OK ||
            HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1) != HAL_OK)
        {
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0U);
            return 0U;
        }
    }
    applied_pulse_us = pulse_us;
    return 1U;
}

/** @brief 提交舵机脉宽并记录位置命令的起始时刻，参数见头文件。 */
uint8_t Transport_Servo_Command(Transport_Servo_Position_t position, uint32_t now_ms)
{
#if JETSON_AUTO_ENABLED
    if (position != TRANSPORT_SERVO_OPEN && position != TRANSPORT_SERVO_CLOSED)
        return 0U;
    const uint16_t pulse = position == TRANSPORT_SERVO_OPEN ?
        TRANSPORT_SERVO_OPEN_US : TRANSPORT_SERVO_CLOSED_US;
    const uint8_t changed = pulse != applied_pulse_us;
    if (!Set_Pulse(pulse))
        return 0U;
    if (changed)
        commanded_ms = now_ms;
    return 1U;
#else
    (void)position;
    (void)now_ms;
    return 0U;
#endif
}

/** @brief 使用无符号毫秒差读取已提交命令的年龄，参数见头文件。 */
uint8_t Transport_Servo_Elapsed(uint32_t now_ms, uint32_t *elapsed_ms)
{
#if JETSON_AUTO_ENABLED
    if (applied_pulse_us == 0U || elapsed_ms == NULL)
        return 0U;
    *elapsed_ms = now_ms - commanded_ms;
    return 1U;
#else
    (void)now_ms;
    (void)elapsed_ms;
    return 0U;
#endif
}

#if !JETSON_AUTO_ENABLED
/** @brief 消费调试器标定值，行驶中拒绝新脉宽；参数见头文件。 */
void Transport_Servo_Calibration_Loop(uint8_t safe_to_move)
{
    const uint16_t pulse = jetson_calibration_pulse_us;
    if (pulse == applied_pulse_us)
        return;
    if ((pulse != 0U && (!safe_to_move ||
         pulse < TRANSPORT_SERVO_CALIBRATION_MIN_US || pulse > TRANSPORT_SERVO_CALIBRATION_MAX_US)) ||
        !Set_Pulse(pulse))
        jetson_calibration_pulse_us = applied_pulse_us;
}
#endif
