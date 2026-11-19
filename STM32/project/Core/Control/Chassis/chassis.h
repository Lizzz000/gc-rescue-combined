#ifndef CHASSIS_H
#define CHASSIS_H

#include <stdint.h>
#include "chassis_pose.h"
#ifndef CHASSIS_TUNING_ENABLED
#define CHASSIS_TUNING_ENABLED 0
#endif
#if CHASSIS_TUNING_ENABLED
/* 专用旋转观测允许纯转 4 rad/s。 */
uint8_t Chassis_Set_Test_Rotation(float wz);
#endif

/* 正常业务目标与底盘连续速度硬限。 */
#define CHASSIS_RUN_SPEED_MPS 0.40f
#define CHASSIS_RUN_YAW_RATE_RAD_S 3.0f
#define CHASSIS_STRAIGHT_MAX_SPEED_MPS 0.50f
/* 11 月 5 日试跑的速度指令在 1 s 内升至约 0.30 m/s，比赛试调将原斜率 0.30 提至 0.45 m/s²。 */
#define CHASSIS_STRAIGHT_ACCELERATION_MPS2 0.45f
/* 2026-11-08 前进实测选用中档，ID2/3两前轮加减速度约 18.56 rad/s²。
 * 0.20 m 在约 1.08 s 确认停车，0.50 m 在约 1.74 s；其余方向与负载待复测。 */
#define CHASSIS_DISTANCE_MAX_SPEED_MPS CHASSIS_STRAIGHT_MAX_SPEED_MPS
#define CHASSIS_DISTANCE_ACCELERATION_MPS2 0.75f
/* 反馈定距的初始控制参数；容差是软件到位判据，实际精度须实车标定。 */
#define CHASSIS_DISTANCE_MIN_M 0.010f
#define CHASSIS_DISTANCE_TOLERANCE_M 0.010f
/* 在验收容差内侧请求停车，给轮速量化与停车滑行保留余量。 */
#define CHASSIS_DISTANCE_STOP_REMAINING_M 0.0015f
/* 末段接近速度克服低速死区，仍不得超过用户限速和剩余距离制动上限。 */
#define CHASSIS_DISTANCE_APPROACH_SPEED_MPS 0.010f
#define CHASSIS_DISTANCE_POSITION_KP 2.0f
#define CHASSIS_DISTANCE_TIMEOUT_FACTOR 3.0
#define CHASSIS_DISTANCE_TIMEOUT_MARGIN_MS 2000U
#define CHASSIS_DISTANCE_STALL_TIMEOUT_MS 1500U
#define CHASSIS_STRAIGHT_YAW_SIGN 1.0f
#define CHASSIS_STRAIGHT_YAW_KP 2.0f
#define CHASSIS_STRAIGHT_YAW_KI 0.0f
#define CHASSIS_STRAIGHT_YAW_KD 0.25f
#define CHASSIS_STRAIGHT_YAW_INTEGRAL_LIMIT 0.2f
#define CHASSIS_STRAIGHT_YAW_OUTPUT_LIMIT 0.8f
#define CHASSIS_VECTOR_MAX_WZ_RAD_S 4.0f
#define CHASSIS_VECTOR_ACCEL_RAD_S2 1.6f
#define CHASSIS_VECTOR_SMOOTHING_S 0.040f
/* 三轮反馈最大允许年龄，ms；停车确认与电机就绪检查共用。 */
#define CHASSIS_MOTOR_FEEDBACK_TIMEOUT_MS 30U
/* 即使接收端仍在刷新，持续无法提交完整速度组也必须中止旧运动。 */
#define CHASSIS_CAN_TX_TIMEOUT_MS 250U

/* 旧车 chassis_turn.h：保留原一圈角度值、独立 PID 与连续到位条件。 */
#define CHASSIS_TURN_LEFT_ANGLE_DEG 360.0f
#define CHASSIS_TURN_RIGHT_ANGLE_DEG 360.0f
#define CHASSIS_TURN_YAW_KP 2.0f
#define CHASSIS_TURN_YAW_KI 0.0f
#define CHASSIS_TURN_YAW_KD 0.1f
#define CHASSIS_TURN_INTEGRAL_LIMIT 0.2f
/* 定角转向为车体量；通过独立的角度反馈和制动距离规划半圈/整圈。 */
#define CHASSIS_TURN_OUTPUT_LIMIT CHASSIS_RUN_YAW_RATE_RAD_S
#define CHASSIS_TURN_ACCEL_RAD_S2 4.0f
#define CHASSIS_TURN_FINE_ANGLE_DEG 5.0f
#define CHASSIS_TURN_ANGLE_TOLERANCE_DEG 2.0f
#define CHASSIS_TURN_ANGLE_EXIT_DEG 3.0f
/* 返航正常交接窗口和一次微调的最大误差。 */
#define CHASSIS_RETURN_POSITION_TOLERANCE_M 0.020
#define CHASSIS_RETURN_YAW_TOLERANCE_RAD 0.174532925f
#define CHASSIS_RETURN_CORRECTION_MAX_M 0.050
#define CHASSIS_RETURN_CORRECTION_MAX_RAD 0.261799388f
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

/* 有效三轮反馈正解：vx/vy(m/s)、wz(rad/s)，无有效里程计时为零。
 * 三轮取一致快照但并非同时采样；轮速只能估计车体速度，无法排除打滑。
 * 控制读取带有效性与时效的 Chassis_Get_Odometry，不单凭此诊断量判断里程。
 */
extern float chassis_speed[3];

/** @brief 复制轮速平移与 IMU 航向里程计，坐标 x 前、y 左、逆时针正。
 * @param[out] data 非空输出；无效时仍复制最后位姿并将 valid 置零。
 * @return 反馈、姿态与里程计均有效且新鲜返回 1，否则 0。
 * @note 与 Loop 同任务调用。失效间隔不补积分，不代表地面绝对定位。
 */
uint8_t Chassis_Get_Odometry(Chassis_Odometry_t *data);

/** @brief 失能或确认停稳时清平移原点与积分历史，运动中拒绝。
 * @return 已清零返回 1，否则 0；首个有效样本仅建立基线。
 * @note 坐标轴仍沿用 INS 航向零参考，不重置 IMU，也不发送电机命令。
 */
uint8_t Chassis_Reset_Odometry(void);

/* 现场调参观察量。只能由同一控制任务通过 Chassis_Set_Tuning 更新参数。
 * MIT 力矩单位已经是 N·m，不在主控重复乘除电机 Kt。
 * 默认启用1.8 kg车体平移惯性项，ID2/3两前轮摩擦项0.015 N·m；纯旋转不加前馈。
 * 2026-11-16以0.25 m/s完成实车前后及双向旋转对照，未推定旋转惯量。 */
extern float chassis_tuning_kd[3];
extern float chassis_tuning_ff_gain;
extern float chassis_tuning_friction[3];
extern float chassis_motor_target[3];
extern float chassis_motor_ff[3];
uint8_t Chassis_Set_Tuning(const float kd[3], float ff_gain, const float friction[3]);

/* 以下接口及 Loop 由同一 test_task 调用；其他任务/中断不得并发修改目标。
 * 先 Init、显式 Enable，再调用动作；默认失能。返回成功表示软件请求接受，
 * 通常切换动作先 Stop，零目标成功提交后可起步；定距可接转向尾段，见其接口说明。
 * 连续速度模式可直接更新目标。
 */

/** @brief 上电初始化底盘计算状态与两组 PID，不发 CAN；仅在任务启动时调用。
 * @return 无。
 */
void Chassis_Init(void);

/** @brief 查询底盘的软件使能状态，供同一控制任务处理通信故障。
 * @return 已受理使能且尚未请求失能返回 1，否则 0。
 * @note 运动准入还须核验使能包序、三轮反馈与姿态。
 */
uint8_t Chassis_Is_Enabled(void);

/* 初次使能的发送资源已就绪：底盘失能、没有待发失能帧、CAN 三邮箱全空。 */
uint8_t Chassis_Can_Enable(void);

/** @brief 接受使能请求，按旧零 MIT、FC、零 MIT 顺序分组提交，之后执行动作。
 * @return 首组三轮零目标已提交或本就使能返回 1；请求不能接受返回 0。
 * @note 后两组由 Loop 每 10 ms 提交。完整提交并收到有效电机反馈后接受起步。
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

/** @brief 正常释放/到位时按加减速曲线回零，保持使能，允许连续向量提前接续。
 * 重复调用沿用原减速进度。三轮零目标实际提交后才允许 Motion_Ready。
 */
void Chassis_Hold(void);

/** @brief 只读查询能否将零目标留给底盘独立执行。正常减速中或首组零 MIT 已提交，
 * 且三轮和姿态有效时返回 1。故障制动、失能和非零目标一律拒绝。
 */
uint8_t Chassis_Can_Release(void);

/** @brief 同时检查本控制拍 IMU 航向、角速度与反馈年龄，允许上电待命先等姿态预热。 */
uint8_t Chassis_Attitude_Ready(void);

/** @brief 查询本次使能、零速指令之后的三轮停车确认，由控制任务调用。
 * @note 故障制动期间返回 0。
 * @return 连续三组新反馈均为已使能、各轮绝对速度 ≤0.20 rad/s 且最新反馈
 *         均在 30 ms 内返回 1。运动、失能、断流或仍在等待确认时返回 0。
 * @note 定时平移结束后可在零速待命期间查询。轮速只能判断车轮是否停转。
 */
uint8_t Chassis_Is_Stopped(void);

/** @brief 检查零目标已提交、无待执行动作、三轮及姿态有效时的运动准入。
 * @note 定距到达后按有效里程计核验并消费终点；诊断要求静止时应使用 Chassis_Is_Stopped。
 *       故障和到期失能期间始终返回 0。
 */
uint8_t Chassis_Motion_Ready(void);

/** @brief 检查三轮是否已使能、速度有限且反馈未超过 30 ms，供手动与自动控制复用。
 * @return 三轮均满足返回 1，否则 0。
 * @note 只读电机反馈，不改变底盘状态。
 */
uint8_t Chassis_Motors_Ready(void);

/** @brief 从零速待命启动锁航向直行，等价八向接口的 FORWARD。
 * @param[in] speed 合速度，m/s，有限且 (0,0.50]。
 * @param[in] duration_ms 首组运动目标提交后的行驶时长，ms；0 持续，最大 INT32_MAX。
 * @return 已接受返回 1；参数、使能/待命状态或姿态不满足返回 0。
 * @note 到期平滑回零并保持使能。
 */
uint8_t Chassis_Straight_Start(float speed, uint32_t duration_ms);

/** @brief 从零速待命启动八向平移，锁定启动航向，斜向保持相同合速度。
 * @param[in] direction FORWARD 至 BACKWARD_RIGHT 的八个方向。
 * @param[in] speed 合速度，m/s，有限且 (0,0.50]。
 * @param[in] duration_ms 首组目标提交后计时，ms；0 持续，最大 INT32_MAX，到期平滑回零待命。
 * @return 已接受返回 1；参数、使能/待命状态或姿态不满足返回 0。
 */
uint8_t Chassis_Translation_Start(Chassis_Translation_Direction_t direction,
                                  float speed, uint32_t duration_ms);

/** @brief 从待命或转向尾段接入八向反馈定距，按实测剩余距离加速、制动。
 * @param[in] direction 八个车体方向之一，沿用平移枚举。
 * @param[in] distance_m 沿锁定方向的位移，m，有限且不小于 CHASSIS_DISTANCE_MIN_M。
 * @param[in] speed_limit_mps 本段合速度上限，m/s，范围 (0,0.50]。
 * @return 已接受返回 1，参数、姿态或底盘状态不满足时返回 0。
 * @note 起步须有效里程计，首个定距控制拍开始计时。
 *       转向剩余角不超过 5 度时也接受定距衔接，保留原转向终点航向，边走边修偏，
 *       不等待原地精修或停车确认；角速度按转向斜坡接入直行 PID。
 *       其他情况仍须从停车待命启动。进度取轮编码器里程计在锁定方向的有符号投影，
 *       剩余 1.5 mm 时进入减速，零目标提交且在 10 mm 交接容差内即可衔接；
 *       末段接近速度仍受请求限速与制动距离约束；断流、过冲、无进展或总超时制动后失能。
 *       轮速里程不能排除打滑，也不提供横向路径闭环，实际距离须实测标定。
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
 *       车体角速度上限 3.0 rad/s。进入角度窗口后沿斜坡回零速待命；
 *       小转角的完成容差不超过请求角度的一半。显式 Stop/Disable 立即清目标，15 s 超时失能。
 */
uint8_t Chassis_Turn_Start(int8_t direction, float angle_deg);

/** @brief 设置连续车体速度，按线/角加速度联合更新斜坡。
 * @param[in] vx 纵向速度，m/s，前正；与 vy 合速度有限且不超过 0.50。
 * @param[in] vy 横向速度，m/s，左正；与 vx 合速度约束相同。
 * @param[in] wz 角速度，rad/s，逆时针正，有限且绝对值不超过 4.0。
 * @return 零速请求或待命/连续速度模式下接受返回 1；参数或姿态不满足返回 0。
 * @note 三项全零在任一已使能动作中等价 Stop，不依赖姿态就绪；非零目标
 *       从待命起步须有新鲜有效反馈，运行中可直接更新，直至显式 Stop/Disable。
 *       纯平移时保持开始该段纯平移的航向；显式非零 wz 按指令转动，切回纯平移
 *       按当时的姿态重新锁定。航向修正沿用原 PID 和连续速度的加速度限制。
 */
uint8_t Chassis_Set_Velocity(float vx, float vy, float wz);

/** @brief 显式三轴速度接口，校验、准入与斜坡同 Chassis_Set_Velocity。
 * @note wz=0表示零角目标，不启用隐式锁航向PID；适用于自行闭环航向的调用方。
 *       三项全零仍等价Stop；本接口不改变其他调用者的锁航向行为。
 */
uint8_t Chassis_Set_Velocity_Explicit_Yaw(float vx, float vy, float wz);

/* 世界系速度入口：平移在固定坐标系内走斜坡，每拍按有效里程计航向换算为车体系。 */
uint8_t Chassis_Set_World_Velocity(float vx, float vy, float wz);

/** @brief 在现有 5 ms 任务内服务底盘，每 10 ms 更新一次控制，TIM2 实测 dt。
 * @return 无。
 * @note TIM2 为已启动的 1 MHz 计数器；RTOS tick 为 1 ms。运动依赖 20 ms 内的
 *       有效姿态，失效或 CAN 提交失败则停止动作并请求失能，不自行恢复。
 */
void Chassis_Loop(void);

#endif
