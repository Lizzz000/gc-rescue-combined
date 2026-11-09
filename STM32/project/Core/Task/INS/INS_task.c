/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2019 DJI / RoboMaster.
 * 来源：18.ins_task/application/INS_task.c 的入口、数据变量和姿态调用；
 * 数据读取使用教程第 13/11 章，imu_cali_slove 和安装矩阵来自
 * 20.standard_robot/application/INS_task.c（均为 RoboMaster 官方配套例程）。
 * 2026-11-06：48 点静止样本测陀螺零偏并对准重力，九轴模式另对准磁航向。
 * 标定及实测 dt 思路参考 HNUYueLuRM/basic_framework，版本 1a136eb1ed2f101110348d5a5cd182272eada38e。
 * Calibrate_MPU_Offset 改编自 modules/imu/BMI088driver.c（MIT，Copyright (c) 2022 NeoZng）；
 * 许可文本见 LICENSES/basic_framework-MIT.txt。具体适配见函数注释。
 * 底盘快照的世界 Z 角速度参考旧车 Core/IMU/imu_attitude.cpp 的 Publish。
 */
#include "INS_task.h"
#include "AHRS.h"
#include "pid.h"
#include "bsp_imu_pwm.h"
#include "vofa.h"
#include "ist8310driver_middleware.h"
#include "tim.h"
#include "cmsis_os.h"
#include "FreeRTOS.h"
#include "task.h"
#include <math.h>
#include <stddef.h>

bmi088_real_data_t bmi088_real_data;
ist8310_real_data_t ist8310_real_data;
uint8_t BMI088_error = BMI088_NO_SENSOR;
uint8_t ist8310_error = IST8310_NO_SENSOR;

/* 官方第 20 章安装矩阵：BMI088 的 [x,y,z] 转为 [y,-x,z]。 */
#define BMI088_BOARD_INSTALL_SPIN_MATRIX \
    {0.0f, 1.0f, 0.0f}, \
    {-1.0f, 0.0f, 0.0f}, \
    {0.0f, 0.0f, 1.0f}
static const float gyro_scale_factor[3][3] = {BMI088_BOARD_INSTALL_SPIN_MATRIX};
static const float accel_scale_factor[3][3] = {BMI088_BOARD_INSTALL_SPIN_MATRIX};
/* 2026-11-05 本板三维翻转实测：芯片坐标偏移，µT；先减偏移再乘矩阵。
 * 复用教程 mag_scale_factor；IST8310 安装矩阵为单位阵，已与校准矩阵合并。
 * 电脑求解参考 PX4 30e763b 的 lm_mag_fit 模型，非 basic_framework 磁标定。
 * 首轮参数经独立第二轮验证；残差仍明显，装车后须复验，不作绝对航向保证。
 * 原始数据、模型来源和结果见 tools/mag_calibration/data/2026-11-05。
 */
static const float mag_offset[3] = {12.0791868f, -10.5893032f, 11.3505272f};
static const float mag_scale_factor[3][3] = {
    {1.02620890f, -0.0255516196f, 0.163406681f},
    {-0.0255516196f, 1.03227452f, -0.0234870691f},
    {0.163406681f, -0.0234870691f, 0.997928465f}
};
static float INS_gyro[3];
static float INS_accel[3];
static float INS_mag[3];
static float gyro_offset[3]; /* 芯片原坐标下的静止零偏，rad/s；每次上电重新标定。 */
float INS_quat[4] = {0.0f, 0.0f, 0.0f, 0.0f};
float INS_angle[3] = {0.0f, 0.0f, 0.0f};
static INS_control_data_t INS_control_data;
static uint8_t INS_control_ready;

/** @brief 复制底盘控制快照；参数、时基、就绪定义及返回值见 INS_task.h。 */
uint8_t INS_Get_Data(INS_control_data_t *data)
{
    if (data == NULL)
        return 0U;
    taskENTER_CRITICAL();
    uint8_t ready = INS_control_ready;
    if (ready)
        *data = INS_control_data;
    taskEXIT_CRITICAL();
    return ready;
}

#define INS_CALI_SAMPLES 48U
#define INS_CALI_PERIOD_MS 10U

/* 45 ℃及 Kp/Kd/输出限幅沿官方第 18 章；温控由 accel 800 Hz 改为 100 Hz，
 * 离散积分 Ki = 0.2 * 800 / 100 = 1.6，仍需实板验证升温与稳态效果。 */
#define TEMPERATURE_CONTROL 45.0f
#define TEMPERATURE_PID_KP 1600.0f
#define TEMPERATURE_PID_KI 1.6f
#define TEMPERATURE_PID_KD 0.0f
#define TEMPERATURE_PID_MAX_OUT 4500.0f
#define TEMPERATURE_PID_MAX_IOUT 4400.0f
/* 本工程适配：高于目标 5 ℃停止加热。 */
#define INS_TEMP_SHUTOFF 50.0f
/* Bosch BST-BMI088-DS001 Rev1.9 第 7 页，BMI088 工作温度 -40..85 ℃。 */
#define INS_TEMP_MINIMUM (-40.0f)
#define INS_TEMP_MAXIMUM 85.0f

static const float imu_temp_PID[3] = {TEMPERATURE_PID_KP, TEMPERATURE_PID_KI, TEMPERATURE_PID_KD};
static pid_type_def imu_temp_pid;

/**
 * @brief 以 BMI088 温度控制板载加热 PWM。
 * @param[in] temp 本次有效温度，℃；NAN 表示读取失败。
 * @return 无。
 * @note 沿官方第 18 章 imu_temp_control 的 PID_calc、负输出置零和 imu_pwm_set。
 *       本工程从启动即使用 PID，省略原先全功率预热分支；每 10 ms 调用。
 *       无效读数、低于 -40 ℃或达到适配上限 50 ℃时关热并清积分。
 */
static void imu_temp_control(float temp)
{
    if (!isfinite(temp) || temp < INS_TEMP_MINIMUM || temp >= INS_TEMP_SHUTOFF)
    {
        imu_pwm_set(0);
        PID_clear(&imu_temp_pid);
        return;
    }

    PID_calc(&imu_temp_pid, temp, TEMPERATURE_CONTROL);
    if (imu_temp_pid.out < 0.0f)
        imu_temp_pid.out = 0.0f;
    imu_pwm_set((uint16_t)imu_temp_pid.out);
}

/**
 * @brief 读取教程的 STAT1 和三轴数据，更新最近一次原始磁场。
 * @return 1 表示 HAL 读成功，是否有新样本由教程的 status 就绪位区分；0 表示读取失败。
 */
static uint8_t read_ist8310(void)
{
    uint8_t buffer[7];
    if (ist8310_IIC_read_muli_reg(0x02, buffer, sizeof(buffer)))
    {
        ist8310_read_over(buffer, &ist8310_real_data);
        return 1U;
    }
    ist8310_real_data.status &= ~(1 << IST8310_DATA_READY_BIT);
    return 0U;
}

/**
 * @brief 求静止窗口内的陀螺零偏、平均重力与平均磁场。
 * @param[out] imu_mean BMI088 芯片坐标下 gyro 零偏（rad/s）和 accel 均值（m/s²）。
 * @param[out] mag_mean IST8310 新磁样本均值，µT；六轴传 NULL，跳过磁场采集。
 * @return 无；完整窗口通过静止判据后返回，九轴另需取得新磁样本。
 * @note 参考 basic_framework/modules/imu/BMI088driver.c 的 Calibrate_MPU_Offset。
 *       沿用极差、重力模长和均值判据。为上电约 1 秒可用，本工程在当前温度下
 *       每 10 ms 采样，共 48 点。窗口同时维持温控，并为初始航向积累真正就绪的磁场。
 */
static void Calibrate_MPU_Offset(bmi088_real_data_t *imu_mean, ist8310_real_data_t *mag_mean)
{
    while (1)
    {
        TickType_t last_wake_time = xTaskGetTickCount();
        float gyro_sum[3] = {0.0f, 0.0f, 0.0f};
        float accel_sum[3] = {0.0f, 0.0f, 0.0f};
        float mag_sum[3] = {0.0f, 0.0f, 0.0f};
        uint16_t mag_samples = 0U;
        float gyro_max[3] = {0.0f, 0.0f, 0.0f};
        float gyro_min[3] = {0.0f, 0.0f, 0.0f};
        float g_norm_sum = 0.0f, g_norm_max = 0.0f, g_norm_min = 0.0f;
        uint16_t sample;

        for (sample = 0; sample < INS_CALI_SAMPLES; sample++)
        {
            vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(INS_CALI_PERIOD_MS));
            BMI088_read(bmi088_real_data.gyro, bmi088_real_data.accel, &bmi088_real_data.temp);
            imu_temp_control(bmi088_real_data.temp);
            if (mag_mean && read_ist8310() && (ist8310_real_data.status & (1U << IST8310_DATA_READY_BIT)))
            {
                for (uint8_t i = 0; i < 3; i++)
                    mag_sum[i] += ist8310_real_data.mag[i];
                ++mag_samples;
            }
            if (!isfinite(bmi088_real_data.temp) || bmi088_real_data.temp < INS_TEMP_MINIMUM ||
                bmi088_real_data.temp > INS_TEMP_MAXIMUM)
                break;
            float g_norm = sqrtf(bmi088_real_data.accel[0] * bmi088_real_data.accel[0] +
                                 bmi088_real_data.accel[1] * bmi088_real_data.accel[1] +
                                 bmi088_real_data.accel[2] * bmi088_real_data.accel[2]);
            if (!isfinite(g_norm) || !isfinite(bmi088_real_data.gyro[0]) ||
                !isfinite(bmi088_real_data.gyro[1]) || !isfinite(bmi088_real_data.gyro[2]))
                break;

            if (sample == 0U)
            {
                g_norm_max = g_norm_min = g_norm;
                for (uint8_t i = 0; i < 3; i++)
                    gyro_max[i] = gyro_min[i] = bmi088_real_data.gyro[i];
            }
            g_norm_sum += g_norm;
            if (g_norm > g_norm_max) g_norm_max = g_norm;
            if (g_norm < g_norm_min) g_norm_min = g_norm;
            for (uint8_t i = 0; i < 3; i++)
            {
                gyro_sum[i] += bmi088_real_data.gyro[i];
                accel_sum[i] += bmi088_real_data.accel[i];
                if (bmi088_real_data.gyro[i] > gyro_max[i]) gyro_max[i] = bmi088_real_data.gyro[i];
                if (bmi088_real_data.gyro[i] < gyro_min[i]) gyro_min[i] = bmi088_real_data.gyro[i];
            }
            if (g_norm_max - g_norm_min > 0.5f ||
                gyro_max[0] - gyro_min[0] > 0.15f ||
                gyro_max[1] - gyro_min[1] > 0.15f ||
                gyro_max[2] - gyro_min[2] > 0.15f)
                break;
        }

        if (sample != INS_CALI_SAMPLES || (mag_mean && mag_samples == 0U))
            continue;
        if (fabsf(g_norm_sum / INS_CALI_SAMPLES - 9.8f) > 0.5f ||
            fabsf(gyro_sum[0] / INS_CALI_SAMPLES) > 0.01f ||
            fabsf(gyro_sum[1] / INS_CALI_SAMPLES) > 0.01f ||
            fabsf(gyro_sum[2] / INS_CALI_SAMPLES) > 0.01f)
            continue;

        for (uint8_t i = 0; i < 3; i++)
        {
            imu_mean->gyro[i] = gyro_sum[i] / INS_CALI_SAMPLES;
            imu_mean->accel[i] = accel_sum[i] / INS_CALI_SAMPLES;
            if (mag_mean)
                mag_mean->mag[i] = mag_sum[i] / mag_samples;
        }
        return;
    }
}

/**
 * @brief 扣除陀螺零偏与磁场偏移，校准磁场并统一三种传感器坐标。
 * @param[out] gyro 统一坐标后的 x/y/z 角速度，rad/s。
 * @param[out] accel 统一坐标后的 x/y/z 加速度，m/s²。
 * @param[out] mag 统一坐标后的校正磁场，μT；整体尺度为拟合约定，非绝对强度标定。
 * @param[in] bmi088 BMI088 原始传感器坐标下的物理量。
 * @param[in] ist8310 IST8310 原始磁场；六轴或本次 I2C 失败传 NULL，输出零磁场供 Mahony 六轴分支使用。
 * @return 无。
 * @note 函数及矩阵来自官方第 20 章 INS_task.c；先扣芯片轴零偏再旋转的顺序
 *       参考 basic_framework 的 BMI088_Read 与 INS_Task，原始数据保持不变。
 *       磁场沿教程接口适配为 A*(raw-b)，b 是芯片坐标偏移，不是教程的输出加项；
 *       A 已合并当前单位安装矩阵，参数由电脑参考 PX4 模型离线求解，只补偿一次。
 */
static void imu_cali_slove(float gyro[3], float accel[3], float mag[3], bmi088_real_data_t *bmi088, ist8310_real_data_t *ist8310)
{
    for (uint8_t i = 0; i < 3; i++)
    {
        gyro[i] = (bmi088->gyro[0] - gyro_offset[0]) * gyro_scale_factor[i][0] +
                  (bmi088->gyro[1] - gyro_offset[1]) * gyro_scale_factor[i][1] +
                  (bmi088->gyro[2] - gyro_offset[2]) * gyro_scale_factor[i][2];
        accel[i] = bmi088->accel[0] * accel_scale_factor[i][0] + bmi088->accel[1] * accel_scale_factor[i][1] + bmi088->accel[2] * accel_scale_factor[i][2];
        if (ist8310)
            mag[i] = (ist8310->mag[0] - mag_offset[0]) * mag_scale_factor[i][0] +
                     (ist8310->mag[1] - mag_offset[1]) * mag_scale_factor[i][1] +
                     (ist8310->mag[2] - mag_offset[2]) * mag_scale_factor[i][2];
        else
            mag[i] = 0.0f;
    }
}

/** @brief 初始化后周期采集、解算和发送姿态，覆盖生成文件的弱 INS_task。
 * @param[in] argument 未使用。 @return 不返回。
 * @note 入口来自官方第 18 章；实测 dt 传入算法参考 basic_framework 的 INS_Task。
 *       使用已有 1 MHz TIM2 替代 DWT_GetDeltaT，无符号差值处理 32 位计数回绕；
 *       相邻 BMI088 读取开始时刻作为时间基准，标定等待不计入首个积分步长。
 */
void INS_task(void const *argument)
{
    (void)argument;
    uint8_t bmi088_read_flag = 0;

    taskENTER_CRITICAL();
    INS_control_ready = 0U;
    taskEXIT_CRITICAL();
    HAL_TIM_Base_Start(&htim2);
    osDelay(INS_TASK_INIT_TIME);
    while ((BMI088_error = BMI088_init()) != BMI088_NO_ERROR)
    {
        osDelay(100);
    }
    while (INS_USE_MAGNETOMETER && (ist8310_error = ist8310_init()) != IST8310_NO_ERROR)
    {
        osDelay(100);
    }

    PID_init(&imu_temp_pid, PID_POSITION, imu_temp_PID,
             TEMPERATURE_PID_MAX_OUT, TEMPERATURE_PID_MAX_IOUT);
    imu_pwm_set(0);
    HAL_TIM_PWM_Start(&htim10, TIM_CHANNEL_1);
    bmi088_real_data_t imu_mean = {0};
    ist8310_real_data_t mag_mean = {0};
    ist8310_real_data_t *startup_mag = INS_USE_MAGNETOMETER ? &mag_mean : NULL;
    uint8_t heading_valid;
    do
    {
        Calibrate_MPU_Offset(&imu_mean, startup_mag);
        for (uint8_t i = 0; i < 3; i++)
            gyro_offset[i] = imu_mean.gyro[i];
        imu_cali_slove(INS_gyro, INS_accel, INS_mag, &imu_mean, startup_mag);
        /* AHRS_init 用重力对准横滚/俯仰；零磁场时航向从零开始。 */
        heading_valid = AHRS_init(INS_quat, INS_accel, INS_mag);
    } while (INS_USE_MAGNETOMETER && !heading_valid);

    uint32_t last_sample_us = __HAL_TIM_GET_COUNTER(&htim2);
    BMI088_read(bmi088_real_data.gyro, bmi088_real_data.accel, &bmi088_real_data.temp);
    imu_temp_control(bmi088_real_data.temp);

    TickType_t last_wake_time = xTaskGetTickCount();
    while (1)
    {
        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(5));
        float dt = 0.0f;
        uint32_t sample_time_ms = 0U;
        if (bmi088_read_flag)
        {
            sample_time_ms = (uint32_t)xTaskGetTickCount();
            uint32_t sample_us = __HAL_TIM_GET_COUNTER(&htim2);
            /* 上次读数失效时，本次仅重建时间基准，不用新角速度积分缺失区间。 */
            if (isfinite(bmi088_real_data.temp))
                dt = (uint32_t)(sample_us - last_sample_us) * 0.000001f;
            last_sample_us = sample_us;
            BMI088_read(bmi088_real_data.gyro, bmi088_real_data.accel, &bmi088_real_data.temp);
            imu_temp_control(bmi088_real_data.temp);
            if (!isfinite(bmi088_real_data.temp))
            {
                taskENTER_CRITICAL();
                INS_control_ready = 0U;
                taskEXIT_CRITICAL();
            }
        }
        ist8310_real_data_t *mag = NULL;
        if (INS_USE_MAGNETOMETER && read_ist8310())
            mag = &ist8310_real_data;
        if (bmi088_read_flag && isfinite(bmi088_real_data.temp) && dt > 0.0f)
        {
            imu_cali_slove(INS_gyro, INS_accel, INS_mag, &bmi088_real_data, mag);
            AHRS_update(INS_quat, dt, INS_gyro, INS_accel, INS_mag);
            get_angle(INS_quat, INS_angle + INS_YAW_ADDRESS_OFFSET, INS_angle + INS_PITCH_ADDRESS_OFFSET, INS_angle + INS_ROLL_ADDRESS_OFFSET);
            /* 旧车 Publish 使用归一化四元数的机体→世界旋转矩阵乘去零偏 gyro。
             * 此处只取第三行，除以模长平方与旧 Get_Rotation_Matrix 的归一化等价，
             * 保持旋转后的角速度比例，不修改解算四元数。 */
            float q0 = INS_quat[0], q1 = INS_quat[1], q2 = INS_quat[2], q3 = INS_quat[3];
            float norm_squared = q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3;
            INS_control_data_t next = {
                .yaw_deg = INS_angle[INS_YAW_ADDRESS_OFFSET] * 57.295779513f,
                .world_z_rad_s = (2.0f * (q1 * q3 - q0 * q2) * INS_gyro[0] +
                                  2.0f * (q2 * q3 + q0 * q1) * INS_gyro[1] +
                                  (q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3) * INS_gyro[2]) / norm_squared,
                .sample_time_ms = sample_time_ms
            };
            uint8_t ready = isfinite(next.yaw_deg) && isfinite(next.world_z_rad_s);
            taskENTER_CRITICAL();
            INS_control_data = next;
            INS_control_ready = ready;
            taskEXIT_CRITICAL();
            /* VOFA 通道顺序为 roll、pitch、yaw，单位由 rad 转换为度。 */
            vofa_send(INS_angle[INS_ROLL_ADDRESS_OFFSET] * 57.295779513f,
                      INS_angle[INS_PITCH_ADDRESS_OFFSET] * 57.295779513f,
                      INS_angle[INS_YAW_ADDRESS_OFFSET] * 57.295779513f);
        }
        bmi088_read_flag = !bmi088_read_flag;
    }
}
