#ifndef IMAGE_TRACKER_H
#define IMAGE_TRACKER_H

#include <stdint.h>

/* x、y 独立估计。q 为 CAN 中心坐标的原始定点单位，10000 q 对应完整图宽或图高。 */
typedef struct
{
    float position[2];                     /**< x、y，q。 */
    float velocity[2];                     /**< x、y，q/s。 */
    float position_variance[2];
    float velocity_variance[2];
    float position_velocity_covariance[2]; /**< 每轴对称 2×2 协方差只保存一个非对角项。 */
    float measurement_variance[2];
    float acceleration_variance;
    uint32_t received_ms;
    uint8_t initialized;
} Image_Tracker_t;

/**
 * @brief 清除旧目标状态，保存二维常速度卡尔曼滤波的噪声参数。
 * @param[in,out] tracker 非空滤波器。
 * @param[in] sigma_x,sigma_y 各轴测量误差的有限正标准差，q。
 * @param[in] sigma_acceleration 两轴共同使用的有限正过程加速度标准差，q/s²。
 * @return 无；下一条观测直接初始化位置并置速度为零。
 */
void Image_Tracker_Init(Image_Tracker_t *tracker, float sigma_x, float sigma_y, float sigma_acceleration);

/**
 * @brief 用一条新 CAN 中心测量预测并修正位置、速度。
 * @param[in,out] tracker 非空且已调用初始化的滤波器。
 * @param[in] x,y 有限的 CAN 中心坐标，q，保持协议原始值。
 * @param[in] received_ms 本帧实际接收时间，ms，按到达顺序传入并支持 uint32_t 回绕。
 * @return 无；同毫秒或时间回退的观测忽略，长间隔由调用方先重新初始化。
 */
void Image_Tracker_Update(Image_Tracker_t *tracker, float x, float y, uint32_t received_ms);

#endif
