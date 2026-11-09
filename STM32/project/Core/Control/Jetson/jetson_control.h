#ifndef JETSON_CONTROL_H
#define JETSON_CONTROL_H

#include <stdint.h>

/* C 板接收期限；独立于 Jetson 内部 /cmd_vel 的 0.3 s 缓存期限。 */
#define JETSON_VELOCITY_TIMEOUT_MS 250U
#define JETSON_PREPARE_TIMEOUT_MS 1000U
#define JETSON_BRAKE_TIMEOUT_MS 500U
#define JETSON_MOTOR_TIMEOUT_MS 30U

typedef enum
{
    JETSON_DISARMED = 0,
    JETSON_PREPARING,
    JETSON_READY,
    JETSON_RUNNING,
    JETSON_BRAKING
} Jetson_Control_State_t;

/* 只读观察本任务上次处理的业务阶段，不代表电机实际已执行。
 * 底盘随后触发保护时，此量最多滞后一个 5 ms 调度周期；不得修改它绕过条件。 */
extern volatile Jetson_Control_State_t jetson_control_state;

/** @brief 初始化 CAN 速度控制，保持底盘上电失能，不发送报文。
 * @return 无。
 * @note 在 Chassis_Init、Jetson_CAN_Init 后且 CAN 接收启动前调用。
 */
void Jetson_Control_Init(void);

/** @brief 在 test_task 的 5 ms 节拍内接入最新 CAN 速度并处理停车保护。
 * @return 无。
 * @note 先于 Chassis_Loop 调用；不在中断或其他任务调用。新有效全零 0x180
 *       请求准备使能，三轮确认静止后才接非零速度。速度字段按 /1000 解码，
 *       vx/vy 为 m/s（前/左正），wz 为车体 rad/s（逆时针正）。越界不裁剪。
 *       运行中全零即使与非零同批接收也立即清除运动目标。停车确认期间忽略
 *       合法非零帧，停稳后只接受新的 CAN 帧。初次使能期间提前发送非零仍触发制动。
 *       撤权、超时、反馈异常后制动并失能，必须再收到新零速才重新准备。
 *       本阶段不由图像中心或运输握手启动动作，不发送虚构的物理完成确认。
 */
void Jetson_Control_Loop(void);

/** @brief 检查三轮是否已使能且反馈在 30 ms 内，供同一任务的两种控制模式复用。
 * @return 三轮反馈均满足条件返回 1，否则返回 0。
 */
uint8_t Jetson_Control_Motors_Ready(void);

#endif
