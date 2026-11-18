#ifndef CHASSIS_MODE_H
#define CHASSIS_MODE_H

/* 标定后改为 1，上电自动搬运；0 为类型 2 手动遥控固件。夹爪由 Jetson 负责。 */
#ifndef JETSON_AUTO_ENABLED
#define JETSON_AUTO_ENABLED 0
#endif

/* 单次底盘联调：由 SWD 显式启动，物资→规则转向→安全区对准及直行；不执行夹爪。 */
#ifndef JETSON_AUTO_CHASSIS_TEST
#define JETSON_AUTO_CHASSIS_TEST 0
#endif
#if JETSON_AUTO_ENABLED && JETSON_AUTO_CHASSIS_TEST
#error "Select either full transport or chassis-only test"
#endif
/* 自动搬运或底盘联调固件：USB 收 Jetson 下行并运行自动流程；否则为类型 2 手动固件。 */
#define JETSON_AUTO_CAN_ENABLED (JETSON_AUTO_ENABLED || JETSON_AUTO_CHASSIS_TEST)
#if defined(CHASSIS_TUNING_ENABLED) && CHASSIS_TUNING_ENABLED && JETSON_AUTO_CAN_ENABLED
#error "Chassis tuning cannot share control with Jetson auto or chassis-only test"
#endif
#ifndef UART6_MANUAL_ENABLED
#define UART6_MANUAL_ENABLED 0
#endif
#if UART6_MANUAL_ENABLED && (JETSON_AUTO_CAN_ENABLED || (defined(CHASSIS_TUNING_ENABLED) && CHASSIS_TUNING_ENABLED))
#error "USART6 manual cannot share chassis control with Jetson or SWD tuning"
#endif

#endif
