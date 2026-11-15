/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2019 RoboMaster.
 * 来源：https://github.com/RoboMaster/Development-Board-C-Examples/blob/master/14.CAN/bsp/boards/bsp_can.c
 * 沿用滤波、启动及通知流程。CAN1 只收电机反馈，板间帧走 USB。
 */
#include "bsp_can.h"
#include "dm_motor.h"

/** @brief 只筛选电机反馈 0x000。板间帧走 USB，不进入这组过滤器。
 * @return 无；滤波配置失败时不启动 CAN1。
 */
void can_filter_init(void)
{
    CAN_FilterTypeDef filter = {0};
    filter.FilterBank = MOTOR_CAN_FILTER_BANK;
    filter.FilterActivation = ENABLE;
    filter.FilterMode = CAN_FILTERMODE_IDLIST;
    filter.FilterScale = CAN_FILTERSCALE_16BIT;
    filter.SlaveStartFilterBank = 14U;
    filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
    /* 四个列表项均为 0：仅接收标准数据帧 ID 0，RTR/IDE 位也为零。 */
    if (HAL_CAN_ConfigFilter(&MOTOR_CAN, &filter) != HAL_OK)
        return;

    if (HAL_CAN_Start(&MOTOR_CAN) != HAL_OK)
        return;
    HAL_CAN_ActivateNotification(&MOTOR_CAN, CAN_IT_RX_FIFO0_MSG_PENDING);
}

/** @brief 沿教程 FIFO0 回调取尽 CAN1 待收帧，按标准数据帧 ID 分流。
 * @param[in,out] hcan HAL 提供的句柄；仅处理电机 CAN1，CAN2 不启动业务。
 * @return 无；中断内仅更新接收数据，不执行底盘动作或发送报文。
 * @note 板间帧不从 CAN 进入。StdId 0、DLC 8 才是电机反馈。
 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    if (hcan != &MOTOR_CAN)
        return;
    while (HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO0) != 0U)
    {
        CAN_RxHeaderTypeDef header;
        uint8_t data[8];
        if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &header, data) != HAL_OK)
            break;
        if (header.IDE != CAN_ID_STD || header.RTR != CAN_RTR_DATA || header.DLC > 8U)
            continue;
        if (header.StdId == 0U && header.DLC == 8U)
            DM_Motor_ReceiveFeedback(data);
    }
}
