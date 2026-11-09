/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2019 DJI / RoboMaster.
 * 来源：教程第 18 章 INS_task 入口、四元数和欧拉角定义。
 * 2026-11-05：保留 5 ms 采集任务，每 10 ms 解算并发送姿态。
 */
#ifndef INS_TASK_H
#define INS_TASK_H

#include "BMI088driver.h"
#include "ist8310driver.h"

/* 上电前选择：0 为 BMI088 六轴相对航向，1 为加入 IST8310 的九轴磁航向。 */
#ifndef INS_USE_MAGNETOMETER
#define INS_USE_MAGNETOMETER 0
#endif

#define INS_TASK_INIT_TIME 7
#define INS_YAW_ADDRESS_OFFSET 0
#define INS_PITCH_ADDRESS_OFFSET 1
#define INS_ROLL_ADDRESS_OFFSET 2

extern bmi088_real_data_t bmi088_real_data;
extern ist8310_real_data_t ist8310_real_data;
extern uint8_t BMI088_error;
extern uint8_t ist8310_error;
extern float INS_quat[4];  /**< 四元数，顺序 w、x、y、z。 */
extern float INS_angle[3]; /**< 欧拉角，顺序 yaw、pitch、roll，单位 rad。 */

/** @brief 底盘控制使用的同一次 INS 解算结果。 */
typedef struct
{
    float yaw_deg;            /**< 航向，°，范围 [-180,180]，俯视逆时针为正。 */
    float world_z_rad_s;       /**< 去零偏后世界坐标 Z 轴角速度，rad/s。 */
    uint32_t sample_time_ms;   /**< BMI 读取开始的 RTOS 时刻；当前 1 tick = 1 ms。 */
} INS_control_data_t;

/**
 * @brief 复制底盘所需航向与角速度的一致快照，不触发采样。
 * @param[out] data 快照输出，非空；返回 0 时保持原内容。
 * @return 1 静止标定后已有有效解算；0 尚未就绪、本次 BMI 读取失效或指针为空。
 * @note 在任务上下文调用；短临界区仅复制数据。sample_time_ms 与 xTaskGetTickCount
 *       同源，当前 RTOS 为 1000 Hz；调用方判断样本年龄，不与 HAL 毫秒混算。
 *       六轴以上电静止朝向为零，九轴以窗口内实测磁场对准航向。BMI 失败后恢复首点
 *       仅重建计时，下一有效解算才重新就绪。
 *       world_z_rad_s 保持旧车 IMU_Get_Data 的 Gyro_Odom[2] 含义。
 */
uint8_t INS_Get_Data(INS_control_data_t *data);

/** @brief 初始化所选传感器，完成短时静止标定和初始对准，再周期发送姿态。
 * @param[in] argument CMSIS-RTOS 保留参数，不使用。
 * @return 不返回。
 * @note 入口沿用官方第 18 章；标定与实测 dt 参考 basic_framework 的
 *       Calibrate_MPU_Offset、INS_Task（版本 1a136eb）。上电请静置约 1 s，启动窗口为
 *       48 点×10 ms，同时升温。运动或无效 BMI 时重采。仅九轴初始化 IST8310、
 *       等待有效水平磁场并在运行中每 5 ms 读磁。BMI088、温控、解算及 VOFA
 *       名义每 10 ms 执行，实际积分间隔由 1 MHz TIM2 测量。
 *       温控沿第 18 章 imu_temp_control；读取失败关热，本轮跳过解算/发送，恢复首点
 *       仅重建计时基准。陀螺零偏每次上电重测，磁场使用本板离线校准参数。
 */
void INS_task(void const *argument);

#endif
