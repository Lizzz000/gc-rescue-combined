#ifndef BSP_CAN_H
#define BSP_CAN_H

#include "can.h"

/* 电机与本模式通信（自动收 Jetson 下行，手动收 0x17F）共用 CAN1；CAN2 保留初始化但不启动业务。 */
#define MOTOR_CAN hcan1
#define MOTOR_CAN_FILTER_BANK 0U

/**
 * @brief 沿用教程例程 can_filter_init 入口，按编译模式配置并启动电机与本模式通信共用的 CAN1。
 * @return 无。
 * @note 在CubeMX的CAN初始化之后调用一次；当前仅启动CAN1。
 */
void can_filter_init(void);

/**
 * @brief CAN1 三个发送邮箱均空闲时尝试提交一条 Jetson 上行运输请求。
 * @param[in] id 运输请求的 11 位标准帧 ID。
 * @param[in] argument 帧参数，含义见 Jetson_CAN_Encode_Uplink（token 或 0x208 到点类型）。
 * @return HAL_OK 表示进入邮箱，HAL_BUSY 表示有待发送帧，错误返回对应 HAL 状态。
 * @note 由底盘所在任务在 Chassis_Loop 后调用；失败由调用方按原步骤重试。
 */
HAL_StatusTypeDef Jetson_CAN_Try_Send_Uplink(uint16_t id, uint32_t argument);

#endif
