#ifndef VISUAL_SERVO_H
#define VISUAL_SERVO_H

#include <stdint.h>

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

typedef struct
{
    float position[2];      /**< 当前图像位置估计，q。 */
    float velocity[2];      /**< 当前图像速度估计，q/s。 */
    uint8_t accepted_mask;  /**< 本帧有效的轴，bit0=x、bit1=y。 */
} Visual_Servo_Estimate_t;

/* 仅在新视觉源帧调用。raw/goal/tolerance 为 q；seconds 为源帧间隔。
 * estimate 可由任何跟踪器提供；无有效估计时传 NULL，清零目标速度。
 * 输出合速限幅，增长限加速度，按 v*t + v²/(2a) 制动；无需积分补偿。
 * 原始入窗轴归零，离群轴减速而不换向，真实换向先降到零。
 * 底盘仍须逐周期执行物理加减速斜坡，丢目标/通信故障由控制层停车。
 */
void Visual_Servo_Update(Visual_Servo_t *state, const Visual_Servo_Config_t *config,
                         const Visual_Servo_Estimate_t *estimate, const float raw[2], const float goal[2],
                         const float tolerance[2], float seconds);

#endif
