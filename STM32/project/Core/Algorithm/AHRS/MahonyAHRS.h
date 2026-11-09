/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2019 DJI / RoboMaster.
 * 来源：RoboMaster/Development-Board-C-Examples
 * 18.ins_task/components/algorithm/MahonyAHRS.c/h。
 * 原算法及作者信息保留于下方；许可见 LICENSES/RoboMaster-GPL-3.0-or-later.txt。
 * 2026-11-05：用调用方实测 dt 替代固定采样频率，保留九轴及六轴公式。
 * dt 传递思路参考 HNUYueLuRM/basic_framework 的 INS_Task 与
 * IMU_QuaternionEKF_Update（revision 1a136eb1ed2f101110348d5a5cd182272eada38e）；
 * 本模块仍为官方教程 Mahony，不移植该参考工程的 EKF。
 * invSqrt 保留教程接口，改用 sqrtf 精确归一化；属于本工程数值精度适配。
 */
//=====================================================================================================
// MahonyAHRS.h
//=====================================================================================================
//
// Madgwick's implementation of Mayhony's AHRS algorithm.
// See: http://www.x-io.co.uk/node/8#open_source_ahrs_and_imu_algorithms
//
// Date			Author			Notes
// 29/09/2011	SOH Madgwick    Initial release
// 02/10/2011	SOH Madgwick	Optimised for reduced CPU load
//
//=====================================================================================================
#ifndef MahonyAHRS_h
#define MahonyAHRS_h

//----------------------------------------------------------------------------------------------------
// Variable declaration

extern volatile float twoKp;			// 2 * proportional gain (Kp)
extern volatile float twoKi;			// 2 * integral gain (Ki)

//---------------------------------------------------------------------------------------------------
// Function declarations

/** @brief 官方教程九轴姿态更新，参数单位和坐标约定见同名实现。
 * @param[in] dt 实测更新间隔，秒，须为大于 0 的有限值。 @return 无。 */
void MahonyAHRSupdate(float q[4], float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz, float dt);
/** @brief 官方教程六轴姿态更新，参数单位和坐标约定见同名实现。
 * @param[in] dt 实测更新间隔，秒，须为大于 0 的有限值。 @return 无。 */
void MahonyAHRSupdateIMU(float q[4], float gx, float gy, float gz, float ax, float ay, float az, float dt);

#endif
//=====================================================================================================
// End of file
//=====================================================================================================
