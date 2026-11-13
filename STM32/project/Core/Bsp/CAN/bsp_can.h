#ifndef BSP_CAN_H
#define BSP_CAN_H

#include "can.h"

/* CAN1 只收电机反馈 0x000。板间帧走 USB。CAN2 保留初始化但不启动业务。 */
#define MOTOR_CAN hcan1
#define MOTOR_CAN_FILTER_BANK 0U

/**
 * @brief 沿用教程例程 can_filter_init 入口，按编译模式配置并启动电机与本模式通信共用的 CAN1。
 * @return 无。
 * @note 在CubeMX的CAN初始化之后调用一次；当前仅启动CAN1。
 */
void can_filter_init(void);

/**
 * @brief 尝试经 USB 提交一条 Jetson 上行运输请求，不占用电机发送邮箱。
 * @param[in] type 上行类型号。
 * @param[in] argument 帧参数，含义见 Jetson_CAN_Encode_Uplink（token 或类型 21 到点类型）。
 * @return HAL_OK 表示已提交，HAL_BUSY 表示链路忙或尚未枚举，错误返回 HAL_ERROR。
 * @note 由底盘所在任务在 Chassis_Loop 后调用；失败由调用方按原步骤重试。
 */
HAL_StatusTypeDef Jetson_CAN_Try_Send_Uplink(uint16_t type, uint32_t argument);

#endif
