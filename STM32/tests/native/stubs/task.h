#ifndef TEST_RTOS_TASK_H
#define TEST_RTOS_TASK_H
#include "FreeRTOS.h"
extern uint32_t test_tick;
void test_enter_critical(void);
void test_exit_critical(void);
#define taskENTER_CRITICAL() test_enter_critical()
#define taskEXIT_CRITICAL() test_exit_critical()
/** @brief 模拟任务计数。 @return tick。 */
static inline TickType_t xTaskGetTickCount(void) { return test_tick; }
/** @brief 模拟中断内读取同一 RTOS 时基。 @return 当前 tick。 */
static inline TickType_t xTaskGetTickCountFromISR(void) { return test_tick; }
/** @brief 模拟固定周期等待。 @param[in,out] wake 基准tick。 @param[in] ticks 周期tick。 */
void vTaskDelayUntil(TickType_t *wake, TickType_t ticks);
/** @brief 模拟相对等待。 @param[in] ticks 等待tick。 @return 无。 */
void vTaskDelay(TickType_t ticks);
#endif
