/** @file dm_motor.h
 * @brief 达妙V1.4手册第4章MIT收发；不包含底盘控制或保护状态机。
 */
#ifndef DM_MOTOR_H
#define DM_MOTOR_H

#include "can.h"

typedef struct
{
    uint8_t id;
    uint8_t state;    /* 原始状态：0失能，1使能，8～E故障。 */
    float position;  /* rad */
    float velocity;  /* rad/s */
    float torque;    /* N·m */
    uint8_t t_mos;    /* ℃ */
    uint8_t t_rotor;  /* ℃ */
    uint32_t feedback_time_ms; /* 有效反馈的 CAN 接收中断时刻，RTOS 1 tick = 1 ms。 */
    uint32_t feedback_seq;  /* 本机每完整接收一帧后递增，允许回绕到 0。 */
} dm_motor_t;

/* ISR更新最近一次反馈，数组下标0/1/2对应电机ID1/2/3，供调试器观察。 */
extern volatile dm_motor_t dm_motor[3];

typedef struct
{
    uint8_t reg;
    uint32_t value_bits; /* 寄存器原始32位，小端解包；调用者按float或uint32解释。 */
    uint32_t received_ms;
    uint32_t sequence; /* 完整匹配读回后最后递增，允许回绕。 */
} dm_param_reply_t;

extern volatile dm_param_reply_t dm_param_reply[3];

/** @brief 通过官方0x7FF/0x33命令读取一个电机寄存器，不写参数或Flash。
 * @param[in] id 电机ID，1～3。 @param[in] reg 寄存器编号（Kt为1）。
 * @return HAL提交结果；非法ID或已观察到电机使能返回HAL_ERROR；未到期请求返回HAL_BUSY。
 * @note 仅控制任务在底盘失能期间调用；一次只容许一个请求，100ms超时后可重新请求。
 *       HAL_OK不等于读成功，须观察对应dm_param_reply.sequence发生变化并核对reg。
 *       本车MST_ID均为0，回复仍经StdID 0接收。参数读取期间不采纳失能运动反馈，
 *       避免共用ID/载荷长度的参数数据污染位置、速度或停车确认。
 */
HAL_StatusTypeDef DM_Motor_ReadParam(uint16_t id, uint8_t reg);

/**
 * @brief 解码达妙反馈，由共用 CAN1 接收中断在筛选帧头后调用。
 * @param[in] data StdID=0、DLC=8 的八字节载荷，非空；仅接受帧内 ID 1～3。
 * @return 无；优先分流参数回复，其余更新电机反馈、RTOS到达时刻及序号。
 * @note 不在此处读取 HAL FIFO，不执行动作或发送报文。
 */
void DM_Motor_ReceiveFeedback(const uint8_t data[8]);

/**
 * @brief 手册p33的无符号整数到物理量线性换算。
 * @param[in] x_int 原始值，0～(2^bits-1)。
 * @param[in] x_min 物理量下限，单位随位置/速度/扭矩而定。
 * @param[in] x_max 物理量上限，须大于x_min。
 * @param[in] bits 位宽，本项目仅使用12或16。
 * @return [x_min,x_max]内的物理量。
 */
float uint_to_float(int x_int, float x_min, float x_max, int bits);
/**
 * @brief 手册p34的物理量到无符号整数线性换算。
 * @param[in] x 有限物理量，调用者保证在[x_min,x_max]内。
 * @param[in] x_min 物理量下限。
 * @param[in] x_max 物理量上限，须大于x_min。
 * @param[in] bits 位宽，本项目仅使用12或16。
 * @return 0～(2^bits-1)的协议整数；本函数不限幅。
 */
int float_to_uint(float x, float x_min, float x_max, int bits);
/**
 * @brief 手册p34的MIT发送函数，适配当前HAL_CAN_AddTxMessage。
 * @param[in,out] hcan 已启动的电机CAN句柄（由bsp_can.h选择，当前&hcan1）。
 * @param[in] id 电机CAN ID，1～3。
 * @param[in] _pos 位置rad，有限且在[-12.5,12.5]。
 * @param[in] _vel 速度rad/s，有限且在[-40,40]。
 * @param[in] _KP 位置增益N·m/rad，有限且在[0,500]。
 * @param[in] _KD 阻尼增益N·m·s/rad，有限且在[0,5]。
 * @param[in] _torq 扭矩N·m，有限；ID1/2为[-1,1]，ID3为[-2,2]。
 * @return HAL 提交结果；HAL_OK 仅表示进入邮箱，不表示电机已经收到。
 * @note 调用者保证数值范围，且载荷不得使用手册p37保留的FF*7+FB/FC/FD/FE命令。
 */
HAL_StatusTypeDef ctrl_motor(CAN_HandleTypeDef *hcan, uint16_t id,
                float _pos, float _vel, float _KP, float _KD, float _torq);
/**
 * @brief 按手册p37使能字节表发送FF*7+FC；手册未给函数名，此处为薄封装。
 * @param[in,out] hcan 已启动的电机CAN句柄（由bsp_can.h选择，当前&hcan1）。
 * @param[in] id 电机CAN ID，1～3。
 * @return HAL 提交结果；只提交一次 CAN 发送，不安排重复发送。
 */
HAL_StatusTypeDef enable_motor(CAN_HandleTypeDef *hcan, uint16_t id);
/**
 * @brief 按手册p37失能字节表发送FF*7+FD；手册未给函数名，此处为薄封装。
 * @param[in,out] hcan 已启动的电机CAN句柄（由bsp_can.h选择，当前&hcan1）。
 * @param[in] id 电机CAN ID，1～3。
 * @return HAL 提交结果；只提交一次 CAN 发送，不安排重复发送。
 */
HAL_StatusTypeDef disable_motor(CAN_HandleTypeDef *hcan, uint16_t id);

#endif
