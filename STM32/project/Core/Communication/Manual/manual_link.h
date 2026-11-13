#ifndef MANUAL_CAN_H
#define MANUAL_CAN_H

#include <stdint.h>

/* 手动速度是类型 2，长度必须是 7：valid 与 vx/vy/wz。没有心跳的 pause 字节。
 * 经 USB 分帧进入，不走 CAN。自动固件忽略类型 2。
 * 快照由 USB 任务写入、test_task 临界区读取；received_ms 为 1 kHz FreeRTOS 时基。
 */
#define MANUAL_CAN_VELOCITY_ID 2U

typedef struct
{
    uint8_t valid;
    int16_t vx, vy, wz;        /**< 原始定点值：vx/vy 为 0.001 m/s，wz 为 0.001 rad/s。 */
    uint32_t received_ms;
    uint32_t sequence;         /**< 每条类型 2（含无效/畸形）递增，按 uint32_t 自然回绕。 */
    uint32_t invalid_sequence; /**< 仅无效/畸形帧递增；保留同一接收批次的撤权事件。 */
    uint32_t zero_sequence;    /**< 仅有效全零帧递增；保留同一接收批次的停车事件。 */
} Manual_CAN_Velocity_t;

/**
 * @brief 清空手动速度快照；由任务在启动 CAN 接收前调用。
 * @return 无。
 */
void Manual_CAN_Init(void);

/**
 * @brief 在 USB 任务内解析一条类型 2；不调用底盘、不发送。
 * @param[in] data 可读 len 字节；空指针按畸形帧处理。
 * @param[in] len 载荷长度，必须为 7。
 * @return 无；无效、畸形帧立即覆盖上一条速度。
 */
void Manual_CAN_Receive(const uint8_t *data, uint8_t len);

/**
 * @brief 在任务临界区复制手动速度快照。
 * @param[out] out 非空。
 * @return 无；时效与授权由手动控制判断。
 */
void Manual_CAN_Get_Velocity(Manual_CAN_Velocity_t *out);

#endif
