#ifndef CHASSIS_ODOMETRY_H
#define CHASSIS_ODOMETRY_H

#include <stdint.h>
#include "heading_eskf.h"

/* 超过此间隔不外推缺失运动；控制层仍须先检查三轮与 IMU 的反馈时效。 */
#define CHASSIS_ODOMETRY_MAX_INTERVAL_US 50000U
/* 本车 MIT 反馈位置量程为 [-12.5,+12.5] rad，齿比均为1；回绕周期不是2*pi。 */
#define CHASSIS_ODOMETRY_ENCODER_PERIOD_RAD 25.0f
#define CHASSIS_ODOMETRY_MAX_WHEEL_RAD_S 40.0f
#define CHASSIS_ODOMETRY_MAX_SAMPLE_SKEW_US 30000U

/** @brief 三轮反馈平移与 IMU 航向形成的平面里程计快照。
 * @note 世界坐标的 x/y 轴由 IMU 零航向确定，x 前、y 左、航向逆时针为正。
 *       valid 只表示最近一次算法输入连续有效；反馈年龄由调用方判断。
 */
typedef struct
{
    double x_m, y_m;
    float yaw_rad;             /* 正式入口为 YAW 主导的 ESKF 航向，[-pi,pi]，rad。 */
    float vx_mps, vy_mps;       /* 当前车体坐标中的反馈平移速度，m/s。 */
    float wheel_wz_rad_s;       /* 三轮正解角速度，仅诊断，不覆盖 IMU 航向。 */
    uint32_t sample_time_us;    /* 最近接受样本的微秒时刻，允许 uint32 回绕。 */
    uint8_t valid;
} Chassis_Odometry_t;

/** @brief 一个底盘的积分历史；只由同一控制任务更新。 */
typedef struct
{
    Chassis_Odometry_t data;
    float previous_vx_mps, previous_vy_mps, previous_yaw_rad;
    float previous_position_rad[3];
    Heading_ESKF_t heading;
    uint8_t initialized;
    uint8_t input_mode;        /* 内部输入基线类型；切换来源时不跨帧补积分。 */
} Odometry_State_t;

/** @brief 清零位姿与积分历史；空指针不执行操作。 */
void Odometry_Init(Odometry_State_t *state);

/** @brief 撤销有效性与速度历史，保留最后位姿及其时刻。
 * @note 恢复后的首个有效样本只重建基线，不积分断流期间的运动。
 */
void Odometry_Invalidate(Odometry_State_t *state);

/** @brief 用一组三轮速度和同帧 IMU 航向更新平面位姿。
 * @param[in,out] state 本底盘非空状态；先调用 Odometry_Init。
 * @param[in] wheel_rad_s 后/右前/左前三个几何轮速，rad/s，已去除电机方向和齿比。
 * @param[in] yaw_rad IMU 航向，rad；有限值会规范到 [-pi,pi]。
 * @param[in] now_us 当前有效反馈组的微秒时刻。
 * @return 接受样本返回 1；重复时刻、超长间隔或非法数据返回 0。
 * @note 首点只建立基线；后续对前后平移速度取均值，以 IMU 角增量作圆弧积分。
 *       重复时刻不改变状态；非法输入或间隔超过 50 ms 使状态失效。
 *       不核对 CAN 序号、使能状态或样本年龄，也不访问 HAL 或控制电机。
 */
uint8_t Odometry_Update(Odometry_State_t *state, const float wheel_rad_s[3],
                        float yaw_rad, uint32_t now_us);

/** @brief 原始 YAW 对照入口：编码器位置增量和 IMU 航向作 SE(2) 积分。
 * @param[in,out] state 同一控制任务拥有的状态，先调用 Odometry_Init。
 * @param[in] wheel_position_rad 后/右前/左前几何轮角，已去除安装方向，范围±12.5rad。
 * @param[in] wheel_rad_s 同组实际轮速，仅输出诊断速度，不用于平移位置积分。
 * @param[in] yaw_rad 同组 IMU 航向，rad，向左转为正。
 * @param[in] now_us 反馈组时间，us；不接受超过50ms的缺测区间。
 * @return 接受返回1；非法、重复、超长间隔或不可实现的位置跳变返回0。
 * @note 参考 WPILib Odometry::Update/Pose2d::Exp，见 README 与 LICENSE-WPILib.txt。
 *       首点/恢复/输入来源切换只建立基线；25rad解绕，跳变界含30ms采样偏斜。
 *       速度零偏不会形成位移，但轮滑、轮径误差和IMU漂移仍需外部观测校正。
 */
uint8_t Odometry_Update_Position(Odometry_State_t *state, const float wheel_position_rad[3],
                                 const float wheel_rad_s[3], float yaw_rad, uint32_t now_us);

/** @brief 正式底盘入口：位置增量 + SO(2) ESKF 航向。
 * @note 陀螺预测、YAW主观测、编码器转角弱约束；gyro与yaw须同一IMU快照。
 *       首帧/输入切换/失效后恢复重建基线；50ms间隔及轮位置检查同对照入口。
 */
uint8_t Odometry_Update_Position_ESKF(Odometry_State_t *state, const float wheel_position_rad[3],
                                      const float wheel_rad_s[3], float yaw_rad,
                                      float gyro_rad_s, uint32_t now_us);

#endif
