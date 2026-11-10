/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2019 RoboMaster.
 * 来源：https://github.com/RoboMaster/Development-Board-C-Examples/blob/master/14.CAN/bsp/boards/bsp_can.c
 * 沿用滤波、启动及通知流程；CAN1 共用适配在同一 FIFO0 入口分流电机与 Jetson。
 */
#include "bsp_can.h"
#include "dm_motor.h"
#include "jetson_can.h"

/** @brief 用两组 16 位列表筛选标准数据帧，再启动 CAN1 的 FIFO0 接收。
 * @return 无；任一滤波组配置失败时不启动 CAN1。
 */
void can_filter_init(void)
{
    static const uint16_t ids[][4] = {
        {0x000U, 0x180U, 0x181U, 0x182U},
        {0x190U, 0x191U, 0x192U, 0x193U},
        {JETSON_CAN_MANUAL_ID, JETSON_CAN_MANUAL_ID, JETSON_CAN_MANUAL_ID, JETSON_CAN_MANUAL_ID},
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

/** @brief 由底盘任务提交一条 Jetson 上行帧；参数和返回值见头文件。 */
HAL_StatusTypeDef Jetson_CAN_Try_Send_Uplink(uint16_t id, uint32_t token)
{
    uint8_t data[8];
    const uint8_t length = Jetson_CAN_Encode_Uplink(id, token, data);
    if (length == 0U)
        return HAL_ERROR;
    if (HAL_CAN_GetTxMailboxesFreeLevel(&MOTOR_CAN) != 3U)
        return HAL_BUSY;

    CAN_TxHeaderTypeDef header = {
        .StdId = id,
        .IDE = CAN_ID_STD,
        .RTR = CAN_RTR_DATA,
        .DLC = length,
        .TransmitGlobalTime = DISABLE,
    };
    uint32_t mailbox;
    return HAL_CAN_AddTxMessage(&MOTOR_CAN, &header, data, &mailbox);
}

/** @brief 按 Jetson can_protocol.hpp 识别七种下行帧 ID。
 * @param[in] id 标准 CAN ID，范围 0～0x7FF。
 * @return 已知下行 ID 返回 1，其余返回 0。
 * @note 长度与载荷交由 Jetson_CAN_Receive 严格检查，畸形速度帧也能使旧指令失效。
 */
static uint8_t Is_Jetson_Frame(uint32_t id)
{
    switch (id)
    {
    case JETSON_CAN_MANUAL_ID:
    case 0x180U:
    case 0x181U:
    case 0x182U:
    case 0x190U:
    case 0x191U:
    case 0x192U:
    case 0x193U:
        return 1U;
    default:
        return 0U;
    }
}

/** @brief 沿教程 FIFO0 回调取尽 CAN1 待收帧，按标准数据帧 ID 分流。
 * @param[in,out] hcan HAL 提供的句柄；仅处理共用的 CAN1，CAN2 不启动业务。
 * @return 无；中断内仅更新接收数据，不执行底盘动作或发送报文。
 * @note 本工程共总线适配：电机反馈仍为 StdID=0、DLC=8；Jetson 仅接收约定下行帧。
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
        else if (Is_Jetson_Frame(header.StdId))
            Jetson_CAN_Receive((uint16_t)header.StdId, data, (uint8_t)header.DLC);
    }
}
