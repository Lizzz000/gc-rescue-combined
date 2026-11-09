#ifndef TEST_CMSIS_OS_H
#define TEST_CMSIS_OS_H
#include <stdint.h>
extern uint32_t test_tick;
/** @brief 推进传感器测试的共同微秒时钟。 @param[in] ms 等待毫秒数。 */
void test_delay_ms(uint32_t ms);
typedef enum { osOK = 0 } osStatus;
/** @brief 模拟任务毫秒等待。 @param[in] ms 毫秒。 @return osOK。 */
static inline osStatus osDelay(uint32_t ms) { test_delay_ms(ms); return osOK; }
#endif
