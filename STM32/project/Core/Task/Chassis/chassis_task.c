#include "chassis_task.h"
#include "bsp_can.h"
#include "bsp_adc.h"
#include "bsp_buzzer.h"
#include "bsp_led.h"
#include "chassis.h"
#if CHASSIS_TUNING_ENABLED
#include "live_tuning.h"
#endif
#include "chassis_mode.h"
#if JETSON_AUTO_CAN_ENABLED
#include "jetson_auto.h"
#include "jetson_link.h"
#else
#include "manual_link.h"
#include "manual_control.h"
#if UART6_MANUAL_ENABLED
#include "uart6_manual.h"
#endif
#endif
#include "FreeRTOS.h"
#include "task.h"
#include <math.h>

/* 指示灯翻转后重置同一个任务计数，电压采样周期须整除这一窗口。 */
_Static_assert(CHASSIS_BATTERY_SAMPLE_PERIOD_MS % CHASSIS_TASK_PERIOD_MS == 0U,
               "battery sampling must align to the chassis task period");
_Static_assert(CHASSIS_LED_TOGGLE_PERIOD_MS % CHASSIS_BATTERY_SAMPLE_PERIOD_MS == 0U,
               "LED toggle must align to the battery sampling period");

volatile float battery_voltage = NAN;

/**
 * @brief 初始化电机 CAN1，以已有 5 ms 任务调度手动或自动控制、底盘、板载提示与电压采样。
 * @param[in] argument CMSIS-RTOS 保留参数，未使用。
 * @return 不返回。
 * @note 任务入口沿用教程第 18 章；调用第 2/5/7 章 LED、蜂鸣器、ADC 接口。
 *       以下分频调度为本工程适配，不新增任务：蜂鸣器保持关闭，三色 LED
 *       每 500 ms 同时翻转，电压每 100 ms 更新。闪烁只表示任务运行，
 *       底盘内部每 10 ms 控制一次；正式模式在姿态就绪后请求零速使能。
 *       动作接口与 Chassis_Loop 均由本任务调用，不在中断内调用。
 *       手动联调与自动搬运按编译模式二选一，只初始化本模式的协议快照。
 *       自动上行在底盘调度之后处理。
 */
void test_task(void const *argument)
{
    (void)argument;
    Chassis_Init();
#if JETSON_AUTO_CAN_ENABLED
    Jetson_CAN_Init();
    Jetson_Auto_Init();
#else
    Manual_CAN_Init();
    Manual_Control_Init();
#if UART6_MANUAL_ENABLED
    UART6_Manual_Init();
#endif
#endif
    can_filter_init();
#if CHASSIS_TUNING_ENABLED
    Live_Tuning_Init();
#endif
    buzzer_off();
    TickType_t wake = xTaskGetTickCount();
    battery_voltage = get_battery_voltage();
    uint32_t task_cycles = 0U;
    for (;;)
    {
        vTaskDelayUntil(&wake, pdMS_TO_TICKS(CHASSIS_TASK_PERIOD_MS));
#if CHASSIS_TUNING_ENABLED
        Live_Tuning_Loop();
#elif UART6_MANUAL_ENABLED
        UART6_Manual_Loop();
#elif !JETSON_AUTO_CAN_ENABLED
        Manual_Control_Loop();
#endif
        Chassis_Loop();
#if CHASSIS_TUNING_ENABLED
        Live_Tuning_Snapshot();
#endif
#if JETSON_AUTO_CAN_ENABLED
        Jetson_Auto_Loop();
#endif
        if (++task_cycles % (CHASSIS_BATTERY_SAMPLE_PERIOD_MS / CHASSIS_TASK_PERIOD_MS) == 0U)
            battery_voltage = get_battery_voltage();
        if (task_cycles == CHASSIS_LED_TOGGLE_PERIOD_MS / CHASSIS_TASK_PERIOD_MS)
        {
            bsp_led_toggle();
            task_cycles = 0U;
        }
    }
}
