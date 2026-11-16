/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2019 RoboMaster.
 * 来源：https://github.com/RoboMaster/Development-Board-C-Examples/blob/master/14.CAN/bsp/boards/bsp_can.c
 * 沿用滤波、启动及通知流程。CAN1 只收电机反馈，板间帧走 USB。
 */
#include "bsp_can.h"
#include "dm_motor.h"
#if CHASSIS_TUNING_ENABLED
#include "FreeRTOS.h"
#include "task.h"
/* 仅现场固件：ISR写入最近8帧；seq最后递增。主机读前后seq相同才采用快照。
 * 每项依次为StdId、载荷[0..3]小端、载荷[4..7]小端、RTOS毫秒。 */
volatile uint32_t can_rx_diag_seq;
volatile uint32_t can_rx_diag[8][4];
#endif

/** @brief 只筛选本车MST_ID=0的运动及参数回复。板间帧走USB。
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
#if CHASSIS_TUNING_ENABLED
    /* RM0090 bxCAN的16位筛选映像：STID[10:0]、RTR、IDE、EXID[17:15]。
     * 仅比较bit4/bit3为0，接收任意标准数据ID，拒绝扩展帧及远程帧。
     * HAL_CAN_ConfigFilter的16bit分支将Low ID/Mask写FR1，High ID/Mask写FR2。 */
    filter.FilterMode = CAN_FILTERMODE_IDMASK;
    filter.FilterMaskIdLow = (1U << 4U) | (1U << 3U);
    filter.FilterMaskIdHigh = (1U << 4U) | (1U << 3U);
#endif
    /* 四个列表项均为0：参数回复同样使用MST_ID=0；0x7FF是请求ID，勿放行。
     * 仅接收标准数据帧，RTR/IDE位也为零。 */
    if (HAL_CAN_ConfigFilter(&MOTOR_CAN, &filter) != HAL_OK)
        return;

    if (HAL_CAN_Start(&MOTOR_CAN) != HAL_OK)
        return;
    HAL_CAN_ActivateNotification(&MOTOR_CAN, CAN_IT_RX_FIFO0_MSG_PENDING);
}

/** @brief 沿教程 FIFO0 回调取尽 CAN1 待收帧，按标准数据帧 ID 分流。
 * @param[in,out] hcan HAL 提供的句柄；仅处理电机 CAN1，CAN2 不启动业务。
 * @return 无；中断内仅更新接收数据，不执行底盘动作或发送报文。
 * @note StdId 0、DLC 8才进入电机解码，解码器先隔离参数回复再更新运动反馈。
 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    if (hcan != &MOTOR_CAN)
        return;
    while (HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO0) != 0U)
    {
        CAN_RxHeaderTypeDef header;
        uint8_t data[8] = {0};
        if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &header, data) != HAL_OK)
            break;
#if CHASSIS_TUNING_ENABLED
        const uint32_t sequence = can_rx_diag_seq;
        volatile uint32_t *entry = can_rx_diag[sequence & 7U];
        entry[0] = header.StdId;
        entry[1] = (uint32_t)data[0] | ((uint32_t)data[1] << 8U) |
            ((uint32_t)data[2] << 16U) | ((uint32_t)data[3] << 24U);
        entry[2] = (uint32_t)data[4] | ((uint32_t)data[5] << 8U) |
            ((uint32_t)data[6] << 16U) | ((uint32_t)data[7] << 24U);
        entry[3] = (uint32_t)xTaskGetTickCountFromISR();
        can_rx_diag_seq = sequence + 1U;
#endif
        if (header.IDE != CAN_ID_STD || header.RTR != CAN_RTR_DATA || header.DLC > 8U)
            continue;
        if (header.StdId == 0U && header.DLC == 8U)
            DM_Motor_ReceiveFeedback(data);
    }
}
