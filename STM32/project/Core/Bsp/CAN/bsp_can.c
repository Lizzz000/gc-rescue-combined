/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2019 RoboMaster.
 * 来源：https://github.com/RoboMaster/Development-Board-C-Examples/blob/master/14.CAN/bsp/boards/bsp_can.c
 * 沿用滤波、启动及通知流程。CAN1 只收电机反馈，板间帧走 USB。
 */
#include "bsp_can.h"
#include "dm_motor.h"
#include "jetson_can.h"
#include "usb_link.h"

/** @brief 只筛选电机反馈。板间帧改走 USB，不再和电机共用这组过滤器。
 * @return 无；滤波配置失败时不启动 CAN1。
 */
void can_filter_init(void)
{
    static const uint16_t ids[][4] = {
        {0x000U, 0x000U, 0x000U, 0x000U},
    };
    CAN_FilterTypeDef filter = {0};
    filter.FilterActivation = ENABLE;
    filter.FilterMode = CAN_FILTERMODE_IDLIST;
    filter.FilterScale = CAN_FILTERSCALE_16BIT;
    filter.SlaveStartFilterBank = 14U;
    filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
    for (unsigned bank = 0U; bank < sizeof(ids) / sizeof(ids[0]); ++bank)
    {
        filter.FilterBank = MOTOR_CAN_FILTER_BANK + bank;
        /* 列表模式下四个半字都是 ID；位 15:5 是标准 ID，RTR/IDE 位清零。 */
        filter.FilterIdLow = (uint32_t)ids[bank][0] << 5U;
        filter.FilterMaskIdLow = (uint32_t)ids[bank][1] << 5U;
        filter.FilterIdHigh = (uint32_t)ids[bank][2] << 5U;
        filter.FilterMaskIdHigh = (uint32_t)ids[bank][3] << 5U;
        if (HAL_CAN_ConfigFilter(&MOTOR_CAN, &filter) != HAL_OK)
            return;
    }

    if (HAL_CAN_Start(&MOTOR_CAN) != HAL_OK)
        return;
    HAL_CAN_ActivateNotification(&MOTOR_CAN, CAN_IT_RX_FIFO0_MSG_PENDING);
}

/** @brief 把一条上行帧交给 USB。忙或尚未枚举返回 HAL_BUSY，不占用电机邮箱。 */
HAL_StatusTypeDef Jetson_CAN_Try_Send_Uplink(uint16_t id, uint32_t argument)
{
    uint8_t data[8];
    const uint8_t length = Jetson_CAN_Encode_Uplink(id, argument, data);
    if (length == 0U)
        return HAL_ERROR;
    const int sent = Jetson_Link_Send(id, data, length);
    if (sent == 0)
        return HAL_OK;
    if (sent == 1)
        return HAL_BUSY;
    return HAL_ERROR;
}

/** @brief 沿教程 FIFO0 回调取尽 CAN1 待收帧，按标准数据帧 ID 分流。
 * @param[in,out] hcan HAL 提供的句柄；仅处理共用的 CAN1，CAN2 不启动业务。
 * @return 无；中断内仅更新接收数据，不执行底盘动作或发送报文。
 * @note 板间帧不再从 CAN 进入。StdID 0、DLC 8 才是电机反馈。
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
