#ifndef VISUAL_SERVO_H
#define VISUAL_SERVO_H

#include "adaptive_kalman.h"

typedef struct
{
    float max_speed_mps;
    float acceleration_mps2;
    float deceleration_mps2;
    float latency_s;
    float response_q_per_m[2]; /**< 相机局部图像响应初值；不是相机坐标到世界坐标的变换。 */
} Visual_Servo_Config_t;

typedef struct
{
    float speed[2]; /**< 使图像坐标增加的平移速度，m/s；控制层再映射为车体前/左。 */
} Visual_Servo_t;

/* 仅在新视觉源帧调用。raw/goal/tolerance 为 q；seconds 为源帧间隔。
 * 输出合速限幅，增长限加速度，按 v*t + v²/(2a) 制动；无需积分补偿。
 * 原始入窗轴归零，离群轴减速而不换向，真实换向先降到零。
 * 底盘仍须逐周期执行物理加减速斜坡，丢目标/通信故障由控制层停车。
 */
void Visual_Servo_Update(Visual_Servo_t *state, const Visual_Servo_Config_t *config,
                         const AdaptiveKalman *filter, const float raw[2], const float goal[2],
                         const float tolerance[2], float seconds);

#endif
