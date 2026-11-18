#ifndef MANUAL_CONTROL_H
#define MANUAL_CONTROL_H
#include <stdint.h>

/* C 板手动接收期限及阶段期限，单位 ms。 */
#define MANUAL_VELOCITY_TIMEOUT_MS 250U
#define MANUAL_PREPARE_TIMEOUT_MS 1000U
#define MANUAL_BRAKE_TIMEOUT_MS 500U
/* 类型 2 速度字段的定点倍率：vx/vy 为 0.001 m/s，wz 为 0.001 rad/s。 */
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

/** @brief 初始化手动速度控制。首次循环在姿态就绪后以零速使能。
 * @return 无。
 * @note 在 Chassis_Init、Manual_CAN_Init 后且 CAN 接收启动前调用。
 */
void Manual_Control_Init(void);

/* USART6 明确接管或在故障后重新使能。USB 使用撤权后的新零帧接管。 */
void Manual_Control_Arm_Request(void);
/* 已接管会话以新鲜全零目标归还控制权，由健康底盘完成剩余减速并保持使能。 */
uint8_t Manual_Control_Release(void);

/** @brief 在 test_task 的 5 ms 节拍内接入最新类型 2 速度并处理停车保护。
 * @return 无。
 * @note 先于 Chassis_Loop 调用；不在中断或其他任务调用。新有效全零类型 2
 *       请求准备使能，包序完成且反馈与姿态有效后接非零速度。速度字段按 /1000 解码，
 *       vx/vy 为 m/s（前/左正），wz 为车体 rad/s（逆时针正）。越界不裁剪。
 *       提交零速接管前持续待命。运行中全零优先于同批非零，平滑减速，
 *       新到的非零速度可直接换向。撤权、接管后的断流和故障触发制动失能。
 *       本机或链路故障后，USB 须先撤权再发新零帧，USART6 须显式重新接管。
 *       类型 1 心跳与图像中心、运输握手都不会启动动作。
 */
void Manual_Control_Loop(void);

#endif
