#ifndef CHASSIS_CONFIG_H
#define CHASSIS_CONFIG_H

#include <stdint.h>

/* 任务周期与 CAN 发帧 */
#define CHASSIS_TASK_PERIOD_MS 5U
#define CHASSIS_CONTROL_PERIOD_MS 10U
#define CHASSIS_BATTERY_SAMPLE_PERIOD_MS 100U
#define CHASSIS_LED_TOGGLE_PERIOD_MS 500U
#define CHASSIS_CAN_INTERFRAME_DELAY_MS 1U
/* 首次 FD 提交后等待各轮回执；缺失回执时限制该轮重发频率。 */
#define CHASSIS_CAN_DISABLE_RETRY_MS 50U
/* 控制调度迟到时限制单次斜坡步长，单位 s。 */
#define CHASSIS_CONTROL_MAX_STEP_S 0.05f

/* 通用速度目标与连续运动限幅 */
#define CHASSIS_RUN_SPEED_MPS 0.40f
#define CHASSIS_RUN_YAW_RATE_RAD_S 3.0f
#define CHASSIS_STRAIGHT_MAX_SPEED_MPS 0.50f
/* 11 月 5 日试跑的速度指令在 1 s 内升至约 0.30 m/s，比赛试调将原斜率 0.30 提至 0.45 m/s²。 */
#define CHASSIS_STRAIGHT_ACCELERATION_MPS2 0.45f

#define CHASSIS_VECTOR_MAX_WZ_RAD_S 4.0f
#define CHASSIS_VECTOR_ACCEL_RAD_S2 1.6f
#define CHASSIS_VECTOR_SMOOTHING_S 0.040f

/* 正式自动搬运基准参数 */
/* 直行速度共用于物资视觉靠近与里程计返航。对准速度用于整区视觉复定位。 */
#define CHASSIS_NAV_TRAVEL_SPEED_MPS 0.50f
#define CHASSIS_NAV_ALIGN_SPEED_MPS 0.33f
/* 快速定距末段的位置增益，单位 1/s，受定距制动包络约束。 */
#define CHASSIS_DISTANCE_FAST_POSITION_KP 4.25f
/* 导航加速度单位 m/s²。正式导航以三分之一的加速度起步，单次联调沿用通用斜坡。 */
#if defined(JETSON_AUTO_CHASSIS_TEST) && JETSON_AUTO_CHASSIS_TEST
#define CHASSIS_NAV_ACCELERATION_MPS2 CHASSIS_STRAIGHT_ACCELERATION_MPS2
#define CHASSIS_NAV_START_ACCEL_MPS2 0.15f
#define CHASSIS_NAV_DECELERATION_MPS2 CHASSIS_STRAIGHT_ACCELERATION_MPS2
#else
#define CHASSIS_NAV_ACCELERATION_MPS2 3.5f
#define CHASSIS_NAV_START_ACCEL_MPS2 (CHASSIS_NAV_ACCELERATION_MPS2 / 3.0f)
/* 同向跟随较低目标速度时的减速度上限，单位 m/s²。 */
#define CHASSIS_NAV_DECELERATION_MPS2 0.75f
#endif
/* 目标反向时先沿用通用斜坡制动至零。 */
#define CHASSIS_NAV_REVERSAL_DECELERATION_MPS2 CHASSIS_STRAIGHT_ACCELERATION_MPS2
/* 同向速度达到此值后允许使用完整导航加速度。 */
#define CHASSIS_NAV_FULL_ACCEL_SPEED_MPS 0.10f
/* 实验选项：0 为原逐拍起步，1 为 MPC，2 为 SMC，3 为 ADRC。 */
#ifndef CHASSIS_NAV_SPEED_CONTROLLER
#define CHASSIS_NAV_SPEED_CONTROLLER 0
#endif

/* 轮反馈定距控制 */
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
/* 总期限取理想轨迹时间乘系数后加余量，无进展期限单独计时。 */
#define CHASSIS_DISTANCE_TIMEOUT_FACTOR 3.0
#define CHASSIS_DISTANCE_TIMEOUT_MARGIN_MS 2000U
#define CHASSIS_DISTANCE_STALL_TIMEOUT_MS 1500U
/* 定距横向修正沿用当前值；不代表新增实车标定结论。 */
#define CHASSIS_DISTANCE_CROSS_GAIN_PER_S 1.5f
#define CHASSIS_DISTANCE_CROSS_SPEED_RATIO 0.25f
/* 无进展检测取固定距离与目标速度对应时间内位移的较小者，保留 double。 */
#define CHASSIS_DISTANCE_PROGRESS_MIN_M 0.002
#define CHASSIS_DISTANCE_PROGRESS_TIME_S 0.1

/* 直行航向保持与角速度反馈 */
#define CHASSIS_STRAIGHT_YAW_SIGN 1.0f
#define CHASSIS_STRAIGHT_YAW_KP 2.0f
#define CHASSIS_STRAIGHT_YAW_KI 0.0f
#define CHASSIS_STRAIGHT_YAW_KD 0.25f
#define CHASSIS_STRAIGHT_YAW_INTEGRAL_LIMIT 0.2f
#define CHASSIS_STRAIGHT_YAW_OUTPUT_LIMIT 0.8f

/* IMU 角速度单边反馈：抑制超速，不提高低速换向时的电机指令。 */
#define CHASSIS_YAW_RATE_FEEDBACK_KP 0.90f
#define CHASSIS_YAW_RATE_FILTER_S 0.025f
#define CHASSIS_YAW_RATE_DEADBAND_RAD_S 0.025f
#define CHASSIS_YAW_RATE_MAX_REDUCTION_RAD_S 0.30f
#define CHASSIS_YAW_RATE_MAX_REDUCTION_RATIO 0.40f

/* 定角转向 */
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
#define CHASSIS_TURN_STOP_RATE_RAD_S 0.10f
#define CHASSIS_TURN_STOP_CONFIRM_MS 20U
#define CHASSIS_TURN_TIMEOUT_MS 15000U

/* 反馈时效、停稳与故障制动 */
/* 三轮反馈最大允许年龄，ms；停车确认与电机就绪检查共用。 */
#define CHASSIS_MOTOR_FEEDBACK_TIMEOUT_MS 30U
#define CHASSIS_MOTOR_FEEDBACK_TIMEOUT_US (CHASSIS_MOTOR_FEEDBACK_TIMEOUT_MS * 1000U)
/* 停稳要求每个车轮的轴速入窗，并满足连续新反馈组数。 */
#define CHASSIS_STOP_SPEED_RAD_S 0.20f
#define CHASSIS_STOP_GROUPS 3U
/* 零 MIT 已交付后，纵向推退允许用三轮编码器独立复核低速估计的静止偏置。 */
#define CHASSIS_ENCODER_STOP_WINDOW_MS 300U
#define CHASSIS_ENCODER_STOP_SPAN_RAD 0.006f   /* 约 16 个编码器计数，轮缘约 0.21 mm。 */
#define CHASSIS_ENCODER_STOP_SPEED_RAD_S 0.32f
#define CHASSIS_ENCODER_STOP_GYRO_RAD_S 0.05f
/* 第五趟送零后实测轮速偏置 0.342 rad/s。较宽速度上限须配更长窗口、更小逐轮位移和角速度。 */
#define CHASSIS_ENCODER_BIAS_WINDOW_MS 600U
#define CHASSIS_ENCODER_BIAS_SPAN_RAD 0.004f   /* 约 10 个位置计数，轮缘约 0.14 mm。 */
#define CHASSIS_ENCODER_BIAS_SPEED_RAD_S 0.42f
#define CHASSIS_ENCODER_BIAS_GYRO_RAD_S 0.025f
/* 纵向停车时才允许短时取消速度阻尼。按轮保留逐帧角增量，2 个原始角度码覆盖静态跳码。 */
#define CHASSIS_NEUTRAL_ARM_MS 120U
#define CHASSIS_NEUTRAL_PEAK_SPEED_RAD_S 0.80f
#define CHASSIS_NEUTRAL_SPEED_BURST_MS 100U
#define CHASSIS_NEUTRAL_POSITION_NOISE_RAD (2.0f * 25.0f / 65535.0f)
#define CHASSIS_NEUTRAL_TRAVEL_RAD 0.016f
/* 即使接收端仍在刷新，持续无法提交完整速度组也必须中止旧运动。 */
#define CHASSIS_CAN_TX_TIMEOUT_MS 250U

/* 故障制动和定距停车后的反馈等待期限，单位 ms。 */
#define CHASSIS_TIMED_BRAKE_LIMIT_MS 500U

/* 里程计与返航交接 */
/* 超过此间隔不外推缺失运动；控制层仍须先检查三轮与 IMU 的反馈时效。 */
#define CHASSIS_ODOMETRY_MAX_INTERVAL_MS 50U
#define CHASSIS_ODOMETRY_MAX_INTERVAL_US (CHASSIS_ODOMETRY_MAX_INTERVAL_MS * 1000U)
#define CHASSIS_ODOMETRY_MAX_IMU_AGE_MS 20U
/* 本车 MIT 反馈位置量程为 [-12.5,+12.5] rad，齿比均为1；回绕周期不是2*pi。 */
#define CHASSIS_ODOMETRY_ENCODER_PERIOD_RAD 25.0f
/* 轮轴速度的反馈有效性门限，单位 rad/s。 */
#define CHASSIS_ODOMETRY_MAX_WHEEL_RAD_S 40.0f

/* 返航正常交接窗口和一次微调的最大误差。 */
#define CHASSIS_RETURN_POSITION_TOLERANCE_M 0.020
#define CHASSIS_RETURN_YAW_TOLERANCE_RAD 0.174532925f
#define CHASSIS_RETURN_CORRECTION_MAX_M 0.050
#define CHASSIS_RETURN_CORRECTION_MAX_RAD 0.261799388f

/* 平移力矩前馈 */
/* 前馈质量用于力矩换算，力矩和摩擦平滑速度分别用 N·m、rad/s。 */
#define CHASSIS_FEEDFORWARD_MASS_KG 1.8f
#define CHASSIS_FEEDFORWARD_LIMIT_NM 0.05f
#define CHASSIS_FRICTION_SPEED_RAD_S 0.5f

/* 两次指令间隔超过此值时跳过本拍的加速前馈，单位 s。 */
#define CHASSIS_FEEDFORWARD_MAX_INTERVAL_S 0.05f

/* 后轮、右前轮、左前轮参数，按 CAN 反馈数组顺序排列。 */
typedef struct
{
    uint16_t id;
    float direction;
    float gear_ratio;
    float default_kd;
    float default_friction_nm;
    float max_motor_rad_s;
} Chassis_Wheel_Config_t;

extern const Chassis_Wheel_Config_t chassis_wheel_config[3];

#endif
