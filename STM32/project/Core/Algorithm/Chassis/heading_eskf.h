#ifndef HEADING_ESKF_H
#define HEADING_ESKF_H

#include <stdint.h>

/* 平面 SO(2) ESKF：名义状态 yaw/bias，误差状态 delta_yaw/delta_bias。
 * 陀螺预测；AHRS YAW 为主要观测；轮角增量仅弱约束零偏，不累计成绝对航向。
 * 单精度、2x2 对称协方差、无堆内存；噪声参数与相关性限制见 README。 */
typedef struct {
    float yaw_rad, bias_rad_s;
    float p_yaw, p_yaw_bias, p_bias;
    float previous_gyro_rad_s;
    float yaw_gain, wheel_bias_gain; /* 本拍诊断，非固定融合百分比。 */
    uint8_t initialized;
} Heading_ESKF_t;

/* 基线只采用 YAW；重新初始化不跨过缺测区间积分。 */
uint8_t Heading_ESKF_Init(Heading_ESKF_t *state, float yaw_rad, float gyro_rad_s);

/* dt: (0,0.05]s；yaw/gyro 必须有效，gyro 为平面世界 z 角速度。
 * wheel_delta_rad 为当前采样间隔的三轮正解转角；NAN 表示无弱约束。
 * 返回0撤销 initialized，调用方必须使本拍里程计失效；不使用陈旧姿态外推。 */
uint8_t Heading_ESKF_Update(Heading_ESKF_t *state, float yaw_rad, float gyro_rad_s,
                            float wheel_delta_rad, float dt);

#endif
