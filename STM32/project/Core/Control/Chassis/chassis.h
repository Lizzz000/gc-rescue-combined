#ifndef CHASSIS_H
#define CHASSIS_H

#include <stdint.h>

/* 运动参数源自旧车 chassis_straight.h。 */
#define CHASSIS_STRAIGHT_MAX_SPEED_MPS 0.50f
/* 11 月 5 日试跑的速度指令在 1 s 内升至约 0.30 m/s，比赛试调将原斜率 0.30 提至 0.45 m/s²。 */
#define CHASSIS_STRAIGHT_ACCELERATION_MPS2 0.45f
/* 2026-11-08 前进实测选用中档，两个后轮加减速度约 18.56 rad/s²。
 * 0.20 m 在约 1.08 s 确认停车，0.50 m 在约 1.74 s；其余方向与负载待复测。 */
#define CHASSIS_DISTANCE_MAX_SPEED_MPS 0.50f
#define CHASSIS_DISTANCE_ACCELERATION_MPS2 0.75f
#define CHASSIS_STRAIGHT_YAW_SIGN 1.0f
#define CHASSIS_STRAIGHT_YAW_KP 2.0f
#define CHASSIS_STRAIGHT_YAW_KI 0.0f
#define CHASSIS_STRAIGHT_YAW_KD 0.25f
#define CHASSIS_STRAIGHT_YAW_INTEGRAL_LIMIT 0.2f
#define CHASSIS_STRAIGHT_YAW_OUTPUT_LIMIT 0.8f
#define CHASSIS_VECTOR_MAX_WZ_RAD_S 2.0f
#define CHASSIS_VECTOR_ACCEL_RAD_S2 1.6f
/* 三轮反馈最大允许年龄，ms；停车确认与电机就绪检查共用。 */
#define CHASSIS_MOTOR_FEEDBACK_TIMEOUT_MS 30U

/* 旧车 chassis_turn.h：保留原一圈角度值、独立 PID 与连续到位条件。 */
#define CHASSIS_TURN_LEFT_ANGLE_DEG 360.0f
#define CHASSIS_TURN_RIGHT_ANGLE_DEG 360.0f
#define CHASSIS_TURN_YAW_KP 2.0f
#define CHASSIS_TURN_YAW_KI 0.0f
#define CHASSIS_TURN_YAW_KD 0.1f
#define CHASSIS_TURN_INTEGRAL_LIMIT 0.2f
/* 定角转向为车体量。纯转向对应三轮各约 6.17 rad/s、12.34 rad/s²。 */
#define CHASSIS_TURN_OUTPUT_LIMIT 2.0f
#define CHASSIS_TURN_ACCEL_RAD_S2 4.0f
#define CHASSIS_TURN_FINE_ANGLE_DEG 5.0f
#define CHASSIS_TURN_ANGLE_TOLERANCE_DEG 2.0f
#define CHASSIS_TURN_RATE_TOLERANCE_RAD_S 0.08f
#define CHASSIS_TURN_SETTLE_TIME_MS 200U
#define CHASSIS_TURN_TIMEOUT_MS 15000U

/* 数值与旧车 chassis_translation.h 一致，斜向合速度与正向相同。 */
typedef enum
{
    CHASSIS_TRANSLATION_NONE = 0,
    CHASSIS_TRANSLATION_FORWARD,
    CHASSIS_TRANSLATION_BACKWARD,
    CHASSIS_TRANSLATION_LEFT,
    CHASSIS_TRANSLATION_RIGHT,
    CHASSIS_TRANSLATION_FORWARD_LEFT,
    CHASSIS_TRANSLATION_FORWARD_RIGHT,
    CHASSIS_TRANSLATION_BACKWARD_LEFT,
    CHASSIS_TRANSLATION_BACKWARD_RIGHT
} Chassis_Translation_Direction_t;

/* 最近三轮反馈正解：vx/vy(m/s)、wz(rad/s)，未收到反馈时初值为零。
 * 三轮反馈并非同时采样；须结合 dm_motor.id/state 判断是否有可用反馈。
 * 此量只供观察，不用于 PID，也不表示车体实际速度已经被独立测量。
 */
extern float chassis_speed[3];

/* 以下接口及 Loop 由同一 test_task 调用；其他任务/中断不得并发修改目标。
 * 先 Init、显式 Enable，再调用动作；默认失能。返回成功表示软件请求接受，
 * 通常切换动作先 Stop，待 Is_Stopped 确认后起步；定距可接转向尾段，见其接口说明。
 * 连续速度模式可直接更新目标。
 */

/** @brief 上电初始化底盘计算状态与两组 PID，不发 CAN；仅在任务启动时调用。
 * @return 无。
 */
void Chassis_Init(void);

/** @brief 查询底盘的软件使能状态，供同一控制任务处理通信故障。
 * @return 已受理使能且尚未请求失能返回 1，否则 0。
 * @note 不表示电机已使能或停稳，起步仍须 Chassis_Is_Stopped 确认。
 */
uint8_t Chassis_Is_Enabled(void);

/** @brief 接受使能请求，按旧零 MIT、FC、零 MIT 顺序分组提交，之后执行动作。
 * @return 首组三轮零目标已提交或本就使能返回 1；请求不能接受返回 0。
 * @note 后两组由 Loop 每 10 ms 提交。三轮新反馈确认静止后才接受起步。
 */
uint8_t Chassis_Enable(void);

/** @brief 清除动作并请求三轮失能；取消旧帧后由 Loop 等待空邮箱发送 FD。
 * @return 无；调用后仍需持续调用 Loop 完成失能帧提交。
 */
void Chassis_Disable(void);

/** @brief 清除动作、斜坡和 PID，保持使能；使能包序结束后周期发送零速度 MIT。
 * @return 无；Kp=0，仅保持旧速度阻尼，不锁定轮子位置。
 */
void Chassis_Stop(void);

/** @brief 查询本次使能、零速指令之后的三轮停车确认，由控制任务调用。
 * @return 连续三组新反馈均为已使能、各轮绝对速度 ≤0.20 rad/s 且最新反馈
 *         均在 30 ms 内返回 1。运动、失能、断流或仍在等待确认时返回 0。
 * @note 定时平移制动成功时可在随后失能前查询。轮速只能判断车轮是否停转。
 */
uint8_t Chassis_Is_Stopped(void);

/** @brief 检查三轮是否已使能、速度有限且反馈未超过 30 ms，供手动与自动控制复用。
 * @return 三轮均满足返回 1，否则 0。
 * @note 只读电机反馈，不改变底盘状态。
 */
uint8_t Chassis_Motors_Ready(void);

/** @brief 从零速待命启动锁航向直行，等价八向接口的 FORWARD。
 * @param[in] speed 合速度，m/s，有限且 (0,0.50]。
 * @param[in] duration_ms 首组运动目标提交后的行驶时长，ms；0 持续，最大 INT32_MAX。
 * @return 已接受返回 1；参数、使能/待命状态或姿态不满足返回 0。
 * @note 到期先发送零速度，确认停车后失能。制动等待上限 500 ms。
 */
uint8_t Chassis_Straight_Start(float speed, uint32_t duration_ms);

/** @brief 从零速待命启动八向平移，锁定启动航向，斜向保持相同合速度。
 * @param[in] direction FORWARD 至 BACKWARD_RIGHT 的八个方向。
 * @param[in] speed 合速度，m/s，有限且 (0,0.50]。
 * @param[in] duration_ms 首组目标提交后计时，ms；0 持续，最大 INT32_MAX，到期制动后失能。
 * @return 已接受返回 1；参数、使能/待命状态或姿态不满足返回 0。
 */
uint8_t Chassis_Translation_Start(Chassis_Translation_Direction_t direction,
                                  float speed, uint32_t duration_ms);

/** @brief 从待命或转向尾段接入八向定距，按给定距离加速、制动并回到零速待命。
 * @param[in] direction 八个车体方向之一，沿用平移枚举。
 * @param[in] distance_m 本段规划距离，m，有限且大于 0。
 * @param[in] speed_limit_mps 本段合速度上限，m/s，范围 (0,0.50]。
 * @return 已接受返回 1，参数、姿态或底盘状态不满足时返回 0。
 * @note 待命起步须三轮确认停车，第一组定距运动帧开始计时。单段最长 INT32_MAX ms。
 *       转向剩余角不超过 5 度时也接受定距衔接，保留原转向终点航向，边走边修偏，
 *       不等待原地精修或停车确认；角速度按转向斜坡接入直行 PID。
 *       其他情况仍须从停车待命启动。规划距离指速度指令的积分，
 *       实际停车距离受轮速跟随、打滑与地形影响。
 */
uint8_t Chassis_Distance_Start(Chassis_Translation_Direction_t direction,
                               float distance_m, float speed_limit_mps);

/** @brief 从零速待命启动相对定角转向，跨越 ±180 度累计转角。
 * @param[in] direction +1 左转，-1 右转。
 * @param[in] angle_deg 相对转角，度，有限且 (0,360]。
 * @return 已接受返回 1；参数、使能/待命状态或姿态不满足返回 0。
 * @note 本次定角参数适配：180 表示半圈，旧整圈可传 CHASSIS_TURN_LEFT_ANGLE_DEG
 *       或 CHASSIS_TURN_RIGHT_ANGLE_DEG；累计角度与 PID 增益沿旧车。
 *       远端按剩余角安排制动，最后 5 度由原 PID 修偏；按 4.0 rad/s² 加减速，
 *       车体角速度上限 2.0 rad/s。正常到位还要求
 *       斜坡指令降到原角速度容差内。显式 Stop/Disable 仍立即清目标。
 *       三轮确认停车后才接受转向，避免提前累计转角或启动超时计时；
 *       到位回零速待命，15 s 超时失能。
 */
uint8_t Chassis_Turn_Start(int8_t direction, float angle_deg);

/** @brief 设置连续车体速度，按线/角加速度联合更新斜坡。
 * @param[in] vx 纵向速度，m/s，前正；与 vy 合速度有限且不超过 0.50。
 * @param[in] vy 横向速度，m/s，左正；与 vx 合速度约束相同。
 * @param[in] wz 角速度，rad/s，逆时针正，有限且绝对值不超过 2.0。
 * @return 零速请求或待命/连续速度模式下接受返回 1；参数或姿态不满足返回 0。
 * @note 三项全零在任一已使能动作中等价 Stop，不依赖姿态就绪；非零目标
 *       从待命起步须确认三轮已停，运行中可直接更新，直至显式 Stop/Disable。
 *       纯平移时保持开始该段纯平移的航向；显式非零 wz 按指令转动，切回纯平移
 *       按当时的姿态重新锁定。航向修正沿用原 PID 和连续速度的加速度限制。
 */
uint8_t Chassis_Set_Velocity(float vx, float vy, float wz);

/** @brief 在现有 5 ms 任务内服务底盘，每 10 ms 更新一次控制，TIM2 实测 dt。
 * @return 无。
 * @note TIM2 为已启动的 1 MHz 计数器；RTOS tick 为 1 ms。运动依赖 20 ms 内的
 *       有效姿态，失效或 CAN 提交失败则停止动作并请求失能，不自行恢复。
 */
void Chassis_Loop(void);

#endif
