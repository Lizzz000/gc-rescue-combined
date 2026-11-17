#ifndef CHASSIS_ODOMETRY_H
#define CHASSIS_ODOMETRY_H

#include <stdint.h>

/* 超过此间隔不外推缺失运动；控制层仍须先检查三轮与 IMU 的反馈时效。 */
#define CHASSIS_ODOMETRY_MAX_INTERVAL_US 50000U

/** @brief 三轮反馈平移与 IMU 航向形成的平面里程计快照。
 * @note 世界坐标的 x/y 轴由 IMU 零航向确定，x 前、y 左、航向逆时针为正。
 *       valid 只表示最近一次算法输入连续有效；反馈年龄由调用方判断。
 */
typedef struct
{
    double x_m, y_m;
    float yaw_rad;             /* IMU 航向，[-pi,pi]，rad。 */
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
    uint8_t initialized;
} Odometry_State_t;

/** @brief 清零位姿与积分历史；空指针不执行操作。 */
void Odometry_Init(Odometry_State_t *state);

/** @brief 撤销有效性与速度历史，保留最后位姿及其时刻。
 * @note 恢复后的首个有效样本只重建基线，不积分断流期间的运动。
 */
void Odometry_Invalidate(Odometry_State_t *state);

/** @brief 用一组三轮速度和同帧 IMU 航向更新平面位姿。
 * @param[in,out] state 本底盘非空状态；先调用 Odometry_Init。
 * @param[in] wheel_rad_s 前/左后/右后三个几何轮速，rad/s，已去除电机方向和齿比。
 * @param[in] yaw_rad IMU 航向，rad；有限值会规范到 [-pi,pi]。
 * @param[in] now_us 当前有效反馈组的微秒时刻。
 * @return 接受样本返回 1；重复时刻、超长间隔或非法数据返回 0。
 * @note 首点只建立基线；后续对前后平移速度取均值，以 IMU 角增量作圆弧积分。
 *       重复时刻不改变状态；非法输入或间隔超过 50 ms 使状态失效。
 *       不核对 CAN 序号、使能状态或样本年龄，也不访问 HAL 或控制电机。
 */
uint8_t Odometry_Update(Odometry_State_t *state, const float wheel_rad_s[3],
                        float yaw_rad, uint32_t now_us);

#endif
