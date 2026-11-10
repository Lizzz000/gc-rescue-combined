#include "test_task.h"
#include "bsp_can.h"
#include "bsp_adc.h"
#include "bsp_buzzer.h"
#include "bsp_led.h"
#include "chassis.h"
#include "jetson_auto.h"
#include "jetson_can.h"
#include "manual_can.h"
#include "manual_control.h"
#include "transport_servo.h"
#include "FreeRTOS.h"
#include "task.h"
#include <math.h>

volatile float battery_voltage = NAN;

/**
 * @brief 初始化共用 CAN1，以已有 5 ms 任务调度手动或自动控制、底盘、板载提示与电压采样。
 * @param[in] argument CMSIS-RTOS 保留参数，未使用。
 * @return 不返回。
 * @note 任务入口沿用教程第 18 章；调用第 2/5/7 章 LED、蜂鸣器、ADC 接口。
 *       以下分频调度为本工程适配，不新增任务：蜂鸣器保持关闭，三色 LED
 *       每 500 ms 同时翻转，电压每 100 ms 更新。闪烁只表示任务运行，
 *       不表示 IMU 已完成预热或标定；底盘内部每 10 ms 控制一次，默认失能。
 *       动作接口与 Chassis_Loop 均由本任务调用，不在中断内调用。
 *       手动联调与自动搬运二选一。舵机标定和自动上行均在底盘调度之后处理。
 */
void test_task(void const *argument)
{
    (void)argument;
    Chassis_Init();
    Jetson_CAN_Init();
    Manual_CAN_Init();
#if JETSON_AUTO_ENABLED || JETSON_AUTO_CHASSIS_TEST
    Jetson_Auto_Init();
#else
    Manual_Control_Init();
#endif
    can_filter_init();
    buzzer_off();
    TickType_t wake = xTaskGetTickCount();
    battery_voltage = get_battery_voltage();
    uint8_t period_ticks = 0U;
    for (;;)
    {
        vTaskDelayUntil(&wake, pdMS_TO_TICKS(5U));
#if !(JETSON_AUTO_ENABLED || JETSON_AUTO_CHASSIS_TEST)
        Manual_Control_Loop();
#endif
        Chassis_Loop();
#if JETSON_AUTO_ENABLED || JETSON_AUTO_CHASSIS_TEST
        Jetson_Auto_Loop();
#else
        const Manual_Control_State_t state = manual_control_state;
        const uint8_t servo_allowed = (state == MANUAL_DISARMED && !Chassis_Is_Enabled()) ||
                                      (state == MANUAL_READY && Chassis_Is_Stopped());
        Transport_Servo_Calibration_Loop(servo_allowed);
#endif
        if (++period_ticks % 20U == 0U)
            battery_voltage = get_battery_voltage();
        if (period_ticks == 100U)
        {
            bsp_led_toggle();
            period_ticks = 0U;
        }
    }
}
