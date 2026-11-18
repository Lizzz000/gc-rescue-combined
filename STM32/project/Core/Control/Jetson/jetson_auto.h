#ifndef JETSON_AUTO_H
#define JETSON_AUTO_H

#include "jetson_auto_config.h"
#include <stdint.h>

#if JETSON_AUTO_CAN_ENABLED
#if JETSON_AUTO_CHASSIS_TEST
/* SWD 写 1 启动本次测试；写 0 请求停车。完成/失败后须复位才可再启动。 */
extern volatile uint8_t jetson_test_start;
#endif

/** @brief 正式固件等待姿态后自动使能，配置、视觉会话和夹爪握手就绪后才开始搬运。 */
void Jetson_Auto_Init(void);

/** @brief 在现有 5 ms 任务中推进一次搬运流程，在底盘之后调度 USB 板间握手。 */
void Jetson_Auto_Loop(void);
#endif

#endif
