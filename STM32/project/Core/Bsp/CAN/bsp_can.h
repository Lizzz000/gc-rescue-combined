#ifndef BSP_CAN_H
#define BSP_CAN_H

#include "can.h"

/* CAN1 只收电机反馈 0x000。板间帧走 USB。CAN2 保留初始化但不启动业务。 */
#define MOTOR_CAN hcan1
#define MOTOR_CAN_FILTER_BANK 0U

/**
 * @brief 配置电机反馈过滤器并启动 CAN1；手动与自动模式使用相同配置。
 * @return 无。
 * @note 在CubeMX的CAN初始化之后调用一次；当前仅启动CAN1。
 */
void can_filter_init(void);

#endif
