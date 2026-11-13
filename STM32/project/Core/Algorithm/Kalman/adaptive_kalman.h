#ifndef ADAPTIVE_KALMAN_H
#define ADAPTIVE_KALMAN_H

#include <stdint.h>

/* 按两轴图像坐标分别估计位置与速度，q 为图像归一化坐标乘 10000。 */
typedef struct
{
    float position[2];
    float velocity[2];
    float position_variance[2];
    float velocity_variance[2];
    float position_velocity_covariance[2];
    float measurement_variance[2];
    float measurement_scale[2];
    float acceleration_variance;
    uint32_t timestamp_ms;
    uint8_t initialized;
} AdaptiveKalman;

/* 标准差单位分别为 q、q、q/s²；参数必须为有限正数。 */
void AdaptiveKalman_Init(AdaptiveKalman *filter, float sigma_x, float sigma_y, float sigma_acceleration);

/* 使用源帧时间间隔；重复或回退时间忽略，长间隔由调用者先重新初始化。 */
void AdaptiveKalman_Update(AdaptiveKalman *filter, float x, float y, uint32_t timestamp_ms);

#endif
