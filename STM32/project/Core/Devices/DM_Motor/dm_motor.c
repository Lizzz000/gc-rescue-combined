/** @file dm_motor.c
 * @brief 依据《调试助手使用说明书（达妙驱动控制协议）V1.4》pp32～37编写。
 *        HAL收发形式参考C板教程pp195～200，达妙载荷不使用DJI的0x200电流帧。
 */
#include "dm_motor.h"
#include "bsp_can.h"
#include "FreeRTOS.h"
#include "task.h"
#include <math.h>

volatile dm_motor_t dm_motor[3];
/* 用户已确认的电机幅值，必须与电机端参数一致。 */
static const float motor_tmax[3] = {1.0f, 1.0f, 2.0f};

/** @brief 沿用手册uint_to_float公式；输入范围及单位见头文件。 */
float uint_to_float(int x_int, float x_min, float x_max, int bits)
{
    float span = x_max - x_min;
    float offset = x_min;
    return (float)x_int * span / (float)((1 << bits) - 1) + offset;
}

/** @brief 沿用手册float_to_uint公式；调用者保证有限且未超界。 */
int float_to_uint(float x, float x_min, float x_max, int bits)
{
    float span = x_max - x_min;
    float offset = x_min;
    return (int)((x - offset) * (float)((1 << bits) - 1) / span);
}

/** @brief 手册MIT打包；保留函数名，使用当前F4 HAL发送接口。 */
HAL_StatusTypeDef ctrl_motor(CAN_HandleTypeDef *hcan, uint16_t id,
                float _pos, float _vel, float _KP, float _KD, float _torq)
{
    if (hcan != &MOTOR_CAN || id < 1U || id > 3U)
        return HAL_ERROR;
    const float torque_limit = motor_tmax[id - 1U];
    if (!isfinite(_pos) || !isfinite(_vel) || !isfinite(_KP) || !isfinite(_KD) || !isfinite(_torq) ||
        _pos < -12.5f || _pos > 12.5f || _vel < -40.0f || _vel > 40.0f ||
        _KP < 0.0f || _KP > 500.0f || _KD < 0.0f || _KD > 5.0f ||
        _torq < -torque_limit || _torq > torque_limit)
        return HAL_ERROR;

    uint16_t pos_tmp = float_to_uint(_pos, -12.5f, 12.5f, 16);
    uint16_t vel_tmp = float_to_uint(_vel, -40.0f, 40.0f, 12);
    uint16_t kp_tmp = float_to_uint(_KP, 0.0f, 500.0f, 12);
    uint16_t kd_tmp = float_to_uint(_KD, 0.0f, 5.0f, 12);
    uint16_t tor_tmp = float_to_uint(_torq, -motor_tmax[id - 1U], motor_tmax[id - 1U], 12);

    CAN_TxHeaderTypeDef header = {0};
    uint32_t mailbox;
    uint8_t data[8];
    header.StdId = id;
    header.IDE = CAN_ID_STD;
    header.RTR = CAN_RTR_DATA;
    header.DLC = 8U;
    data[0] = pos_tmp >> 8;
    data[1] = pos_tmp;
    data[2] = vel_tmp >> 4;
    data[3] = ((vel_tmp & 0xFU) << 4) | (kp_tmp >> 8);
    data[4] = kp_tmp;
    data[5] = kd_tmp >> 4;
    data[6] = ((kd_tmp & 0xFU) << 4) | (tor_tmp >> 8);
    data[7] = tor_tmp;
    /* 保留命令与极值 MIT 报文可能重合，避免普通控制触发使能、失能或清错。 */
    uint8_t reserved = 1U;
    for (unsigned i = 0U; i < 7U; ++i)
        if (data[i] != 0xFFU)
            reserved = 0U;
    if (reserved && data[7] >= 0xFBU && data[7] <= 0xFEU)
        return HAL_ERROR;
    return HAL_CAN_AddTxMessage(hcan, &header, data, &mailbox);
}

/** @brief 手册p37使能帧的薄封装；句柄、ID和返回值见头文件。 */
HAL_StatusTypeDef enable_motor(CAN_HandleTypeDef *hcan, uint16_t id)
{
    if (hcan != &MOTOR_CAN || id < 1U || id > 3U)
        return HAL_ERROR;
    CAN_TxHeaderTypeDef header = {0};
    uint32_t mailbox;
    uint8_t data[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC};
    header.StdId = id;
    header.IDE = CAN_ID_STD;
    header.RTR = CAN_RTR_DATA;
    header.DLC = 8U;
    return HAL_CAN_AddTxMessage(hcan, &header, data, &mailbox);
}

/** @brief 手册p37失能帧的薄封装；句柄、ID和返回值见头文件。 */
HAL_StatusTypeDef disable_motor(CAN_HandleTypeDef *hcan, uint16_t id)
{
    if (hcan != &MOTOR_CAN || id < 1U || id > 3U)
        return HAL_ERROR;
    CAN_TxHeaderTypeDef header = {0};
    uint32_t mailbox;
    uint8_t data[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD};
    header.StdId = id;
    header.IDE = CAN_ID_STD;
    header.RTR = CAN_RTR_DATA;
    header.DLC = 8U;
    return HAL_CAN_AddTxMessage(hcan, &header, data, &mailbox);
}

/**
 * @brief 按达妙手册 p32/p35 解码电机反馈，由 CAN1 共用接收入口调用。
 * @param[in] data 已验证 StdID=0、DLC=8 的八字节载荷，非空；帧内 ID 为 1～3。
 * @return 无；更新dm_motor的对应电机数据，不发送任何控制帧。
 * @note 本工程适配：在接收中断内调用，用 RTOS 中断时基记录到达时间，最后更新序号。
 */
void DM_Motor_ReceiveFeedback(const uint8_t data[8])
{
    if (data == NULL)
        return;
    uint8_t id = data[0] & 0x0FU;
    if (id < 1U || id > 3U)
        return;
    uint8_t index = id - 1U;
    int p_int = (data[1] << 8) | data[2];
    int v_int = (data[3] << 4) | (data[4] >> 4);
    int t_int = ((data[4] & 0xFU) << 8) | data[5];
    dm_motor[index].id = id;
    dm_motor[index].state = data[0] >> 4;
    dm_motor[index].position = uint_to_float(p_int, -12.5f, 12.5f, 16);
    dm_motor[index].velocity = uint_to_float(v_int, -40.0f, 40.0f, 12);
    dm_motor[index].torque = uint_to_float(t_int, -motor_tmax[index], motor_tmax[index], 12);
    dm_motor[index].t_mos = data[6];
    dm_motor[index].t_rotor = data[7];
    dm_motor[index].feedback_time_ms = (uint32_t)xTaskGetTickCountFromISR();
    ++dm_motor[index].feedback_seq;
}
