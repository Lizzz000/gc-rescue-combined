/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2019 DJI / RoboMaster.
 * 来源：https://github.com/RoboMaster/Development-Board-C-Examples/blob/master/18.ins_task/application/INS_task.c
 * 2026-11-05：将 AHRS_init/AHRS_update/get_angle 移至独立算法模块。
 * 保留教程更新公式和接口，fp32 使用等价 float；time 传给 Mahony 的 dt。
 * 实测 dt 思路参考 basic_framework/modules/imu/ins_task.c 的 INS_Task
 * 与 modules/algorithm/QuaternionEKF.c 的 IMU_QuaternionEKF_Update，
 * revision 1a136eb1ed2f101110348d5a5cd182272eada38e；保留 Mahony 九轴算法。
 */
#include "AHRS.h"
#include "MahonyAHRS.h"
#include <math.h>

/**
 * @brief 由静止平均重力、磁场求姿态初值，参数及返回值见 AHRS.h。
 * @note 重力对准参考 basic_framework 的 InitQuaternion（1a136eb）。
 *       倾斜补偿参考 NXP AN4248 的 Tilt-Compensation Algorithm。
 *       将校正磁场转至水平面，再以水平磁场为世界 +X，与现有 Mahony 一致。
 */
uint8_t AHRS_init(float quat[4], const float accel[3], const float mag[3])
{
    const float roll = atan2f(accel[1], accel[2]);
    const float pitch = atan2f(-accel[0], sqrtf(accel[1] * accel[1] + accel[2] * accel[2]));
    const float sr = sinf(roll), cr = cosf(roll);
    const float mx = cosf(pitch) * mag[0] + sinf(pitch) * (sr * mag[1] + cr * mag[2]);
    const float my = cr * mag[1] - sr * mag[2];
    const float field_sq = mag[0] * mag[0] + mag[1] * mag[1] + mag[2] * mag[2];
    /* 水平投影低至全场的千分之一时，航向易被量化噪声主导。 */
    const uint8_t heading_valid = mx * mx + my * my > 1.0e-6f * field_sq;
    const float yaw = heading_valid ? atan2f(-my, mx) : 0.0f;
    const float cos_half_roll = cosf(0.5f * roll);
    const float sin_half_roll = sinf(0.5f * roll);
    const float cos_half_pitch = cosf(0.5f * pitch);
    const float sin_half_pitch = sinf(0.5f * pitch);
    const float cos_half_yaw = cosf(0.5f * yaw);
    const float sin_half_yaw = sinf(0.5f * yaw);

    quat[0] = cos_half_roll * cos_half_pitch * cos_half_yaw + sin_half_roll * sin_half_pitch * sin_half_yaw;
    quat[1] = sin_half_roll * cos_half_pitch * cos_half_yaw - cos_half_roll * sin_half_pitch * sin_half_yaw;
    quat[2] = cos_half_roll * sin_half_pitch * cos_half_yaw + sin_half_roll * cos_half_pitch * sin_half_yaw;
    quat[3] = cos_half_roll * cos_half_pitch * sin_half_yaw - sin_half_roll * sin_half_pitch * cos_half_yaw;
    return heading_valid;
}

/** @brief 沿教程调用 Mahony 更新；参数及返回值见 AHRS.h。
 * @note 实测 time 传递参考 basic_framework 的 INS_Task/IMU_QuaternionEKF_Update。
 */
void AHRS_update(float quat[4], float time, float gyro[3], float accel[3], float mag[3])
{
    MahonyAHRSupdate(quat, gyro[0], gyro[1], gyro[2], accel[0], accel[1], accel[2], mag[0], mag[1], mag[2], time);
}

/** @brief 将四元数转换为 yaw/pitch/roll；参数及返回值见 AHRS.h。
 * @note 保留教程接口及 yaw/roll 公式；pitch 改用等价齐次 atan2 形式，
 *       避免精确归一化后的浮点舍入使 asin 输入在俯仰极点超出 [-1,1]。
 */
void get_angle(float q[4], float *yaw, float *pitch, float *roll)
{
    *yaw = atan2f(2.0f*(q[0]*q[3]+q[1]*q[2]), 2.0f*(q[0]*q[0]+q[1]*q[1])-1.0f);
    const float sin_pitch = 2.0f * (q[0]*q[2] - q[1]*q[3]);
    const float gravity_y = 2.0f * (q[0]*q[1] + q[2]*q[3]);
    const float gravity_z = q[0]*q[0] - q[1]*q[1] - q[2]*q[2] + q[3]*q[3];
    *pitch = atan2f(sin_pitch, sqrtf(gravity_y*gravity_y + gravity_z*gravity_z));
    *roll = atan2f(2.0f*(q[0]*q[1]+q[2]*q[3]),2.0f*(q[0]*q[0]+q[3]*q[3])-1.0f);
}
