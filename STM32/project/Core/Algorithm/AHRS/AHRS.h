/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2019 DJI / RoboMaster.
 * 来源：18.ins_task/application/INS_task.c 的三个姿态接口。
 * 2026-11-05：独立为算法模块，fp32 以等价的 float 表示。
 */
#ifndef AHRS_H
#define AHRS_H
#include <stdint.h>

/**
 * @brief 用静止重力和校正磁场初始化九轴姿态。
 * @param[out] quat 机体系到世界系的单位四元数，顺序 w、x、y、z。
 * @param[in] accel 完整静止标定窗口的平均加速度，m/s²，已统一为机体系，须有效且非零。
 * @param[in] mag 同一窗口有效磁样本的机体系均值，已校正，μT。
 * @return 1 已对准重力和磁航向。0 仅对准重力，yaw 置零，启动任务继续采样。
 * @note 重力对准沿用 basic_framework（1a136eb）的 InitQuaternion 思路。
 *       磁航向按 NXP AN4248 的倾斜补偿方法适配本车坐标。
 */
uint8_t AHRS_init(float quat[4], const float accel[3], const float mag[3]);

/**
 * @brief 使用教程 Mahony 算法更新姿态。
 * @param[in,out] quat 四元数，顺序 w、x、y、z。
 * @param[in] time 本次更新距上次更新的实测时间，秒，须为大于 0 的有限值。
 * @param[in] gyro 统一坐标系的 x/y/z 角速度，rad/s。
 * @param[in] accel 统一坐标系的 x/y/z 加速度，m/s²。
 * @param[in] mag 统一坐标系的 x/y/z 磁场，μT。
 * @return 无。
 * @note 保留官方教程函数签名；实测时间传递参考 basic_framework 的 INS_Task
 *       与 IMU_QuaternionEKF_Update，将 time 用于 Mahony 的积分及积分反馈。
 */
void AHRS_update(float quat[4], float time, float gyro[3], float accel[3], float mag[3]);

/**
 * @brief 沿教程接口将四元数转换为欧拉角。
 * @param[in] q 单位四元数，允许浮点归一化误差，顺序 w、x、y、z。
 * @param[out] yaw 偏航角，rad，范围 [-π, π]。
 * @param[out] pitch 俯仰角，rad，范围 [-π/2, π/2]。
 * @param[out] roll 横滚角，rad，范围 [-π, π]。
 * @return 无。
 * @note pitch 使用等价齐次 atan2 形式，避免俯仰极点的 asin 浮点越界；
 *       yaw/roll 保留教程公式，极点处二者不唯一。
 */
void get_angle(float q[4], float *yaw, float *pitch, float *roll);

#endif
