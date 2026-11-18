#ifndef CHASSIS_POSE_H
#define CHASSIS_POSE_H

#include <stdint.h>

/* x 前、y 左，世界坐标轴由 IMU 零航向确定。速度在车体系下表示。 */
typedef struct
{
    double x_m;
    double y_m;
    float yaw_rad; /* 控制拍航向，逆时针为正，[-pi,pi]。 */
    float vx_mps;
    float vy_mps;
    float wheel_wz_rad_s; /* 轮反馈正解角速度，仅用于诊断。 */
    uint32_t sample_time_us; /* 接受反馈组的控制拍 TIM2 时刻，允许 uint32 回绕。 */
    uint8_t valid;
} Chassis_Odometry_t;

#endif
