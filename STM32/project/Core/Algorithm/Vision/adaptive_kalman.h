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
    uint8_t accepted_mask;       /**< 本帧接受的轴：bit0=x，bit1=y；拒绝帧不驱动换向。 */
    uint8_t rejected_run[2];
    int8_t rejected_side[2];
} AdaptiveKalman;

/* 标准差单位分别为 q、q、q/s²；非法参数使状态未配置，Update 不接受观测。 */
void AdaptiveKalman_Init(AdaptiveKalman *filter, float sigma_x, float sigma_y, float sigma_acceleration);

/* 使用源帧时间；重复/倒序忽略，间隔 >=250ms 自动重建基线，NaN/Inf 不污染状态。
 * 连续三帧同侧大创新重新捕获；孤立异常只预测，不更新速度。 */
void AdaptiveKalman_Update(AdaptiveKalman *filter, float x, float y, uint32_t timestamp_ms);

#endif
