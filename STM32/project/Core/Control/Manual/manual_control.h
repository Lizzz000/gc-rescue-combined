#ifndef MANUAL_CONTROL_H
#define MANUAL_CONTROL_H

#include <stdint.h>

/* C 板手动接收期限及阶段期限，单位 ms。 */
#define MANUAL_VELOCITY_TIMEOUT_MS 250U
#define MANUAL_PREPARE_TIMEOUT_MS 1000U
#define MANUAL_BRAKE_TIMEOUT_MS 500U
/* 0x17F 速度字段的定点倍率：vx/vy 为 0.001 m/s，wz 为 0.001 rad/s。 */
#define MANUAL_VELOCITY_FIXED_POINT_SCALE 1000.0f

typedef enum
{
    MANUAL_DISARMED = 0,
    MANUAL_PREPARING,
    MANUAL_READY,
    MANUAL_RUNNING,
    MANUAL_BRAKING
} Manual_Control_State_t;

/* 只读观察本任务上次处理的业务阶段，不代表电机实际已执行。
 * 底盘随后触发保护时，此量最多滞后一个 5 ms 调度周期；不得修改它绕过条件。 */
extern volatile Manual_Control_State_t manual_control_state;

/** @brief 初始化手动速度控制，保持底盘上电失能，不发送报文。
 * @return 无。
 * @note 在 Chassis_Init、Manual_CAN_Init 后且 CAN 接收启动前调用。
 */
void Manual_Control_Init(void);

/** @brief 在 test_task 的 5 ms 节拍内接入最新 0x17F 速度并处理停车保护。
 * @return 无。
 * @note 先于 Chassis_Loop 调用；不在中断或其他任务调用。新有效全零 0x17F
 *       请求准备使能，三轮确认静止后才接非零速度。速度字段按 /1000 解码，
 *       vx/vy 为 m/s（前/左正），wz 为车体 rad/s（逆时针正）。越界不裁剪。
 *       运行中全零即使与非零同批接收也立即清除运动目标。停车确认期间忽略
 *       合法非零帧，停稳后只接受新的 CAN 帧。初次使能期间提前发送非零仍触发制动。
 *       撤权、超时、反馈异常后制动并失能，必须再收到新零速才重新准备。
 *       0x180 心跳与图像中心、运输握手都不会启动动作。
 */
void Manual_Control_Loop(void);

#endif
