/** @file dm_motor.c
 * @brief 依据《调试助手使用说明书（达妙驱动控制协议）V1.4》pp32～37编写。
 *        HAL收发形式参考C板教程pp195～200，达妙载荷不使用DJI的0x200电流帧。
 */
#include "dm_motor.h"
#include "bsp_can.h"
#include "FreeRTOS.h"
#include "task.h"

volatile dm_motor_t dm_motor[3];
/* 用户已确认的电机幅值，必须与电机端参数一致。 */
static const float motor_tmax[3] = {1.0f, 1.0f, 2.0f};

#if CHASSIS_TUNING_ENABLED
volatile dm_param_reply_t dm_param_reply[3];
#define DM_PARAM_TIMEOUT_MS 100U
/* 由控制任务发布，CAN ISR消费；提交时短临界区防止回复先于请求登记。 */
static volatile struct
{
    uint8_t active, requested, id, reg;
    uint32_t started_ms;
} param_request;

HAL_StatusTypeDef DM_Motor_ReadParam(uint16_t id, uint8_t reg)
{
    if (id < 1U || id > 3U)
        return HAL_ERROR;

    CAN_TxHeaderTypeDef header = {0};
    uint32_t mailbox;
    uint8_t data[8] = {(uint8_t)id, 0U, 0x33U, reg, 0U, 0U, 0U, 0U};
    header.StdId = 0x7FFU;
    header.IDE = CAN_ID_STD;
    header.RTR = CAN_RTR_DATA;
    header.DLC = 8U;

    taskENTER_CRITICAL();
    for (unsigned i = 0U; i < 3U; ++i)
    {
        if (dm_motor[i].state == 1U)
        {
            taskEXIT_CRITICAL();
            return HAL_ERROR;
        }
    }
    uint32_t now_ms = (uint32_t)xTaskGetTickCount();
    if (param_request.active &&
        (uint32_t)(now_ms - param_request.started_ms) < DM_PARAM_TIMEOUT_MS)
    {
        taskEXIT_CRITICAL();
        return HAL_BUSY;
    }
    param_request.id = (uint8_t)id;
    param_request.reg = reg;
    param_request.started_ms = now_ms;
    param_request.active = 1U;
    HAL_StatusTypeDef result = HAL_CAN_AddTxMessage(&MOTOR_CAN, &header, data, &mailbox);
    if (result == HAL_OK)
        param_request.requested = 1U;
    else
        param_request.active = 0U;
    taskEXIT_CRITICAL();
    return result;
}

/** @brief 参数/运动回复共用MST_ID=0；只发布当前请求的0x33读回。
 * 官方SDK依据：dmBots/motor-sdk/Python例程/u2can/DM_CAN.py，
 * __read_RID_param、__process_set_param_packet。诊断固件提交过读请求后，
 * 优先隔离可能的参数帧及迟到回包，避免同一载荷被解释成运动反馈。
 */
static uint8_t receive_param_reply(const uint8_t data[8], uint32_t now_ms)
{
    if (!param_request.requested)
        return 0U;

    const uint8_t waiting = param_request.active &&
        (uint32_t)(now_ms - param_request.started_ms) < DM_PARAM_TIMEOUT_MS;
    if (!waiting)
        param_request.active = 0U;

    if (data[0] >= 1U && data[0] <= 3U && data[1] == 0U &&
        (data[2] == 0x33U || data[2] == 0x55U || data[2] == 0xAAU))
    {
        if (waiting && data[0] == param_request.id && data[2] == 0x33U &&
            data[3] == param_request.reg)
        {
            volatile dm_param_reply_t *reply = &dm_param_reply[data[0] - 1U];
            reply->reg = data[3];
            reply->value_bits = (uint32_t)data[4] | ((uint32_t)data[5] << 8U) |
                ((uint32_t)data[6] << 16U) | ((uint32_t)data[7] << 24U);
            reply->received_ms = now_ms;
            ++reply->sequence;
            param_request.active = 0U;
        }
        /* 包括迟到、重复、错误寄存器回复，均不可更新运动反馈。 */
        return 1U;
    }
    /* 参数回复的首字节与state=0运动回复不能完全区分。只读窗口内隔离该类帧，
     * 非零状态反馈仍正常处理，使能反馈还会撤销当前读取。 */
    if (waiting && (data[0] >> 4U) == 0U)
        return 1U;
    if ((data[0] >> 4U) == 1U)
        param_request.active = 0U;
    return 0U;
}
#endif

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
    uint32_t now_ms = (uint32_t)xTaskGetTickCountFromISR();
#if CHASSIS_TUNING_ENABLED
    if (receive_param_reply(data, now_ms))
        return;
#endif
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
    dm_motor[index].feedback_time_ms = now_ms;
    ++dm_motor[index].feedback_seq;
}
