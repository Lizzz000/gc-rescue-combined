/* 来源：旧小车 Core/Motor/motor_chassis.c；沿用120度三轮几何。
 * 2026-11-17：用户确认 ID1后轮、ID2右前轮、ID3左前轮；按接地点切向投影
 * 统一正逆解的横向符号，保留已实测的前进与逆时针方向。几何约定见头文件。
 * 此处只做计算，输入检查由底盘动作入口负责，不依赖主控或 HAL。
 */
#include "motor_chassis.h"

/** @brief 三轮逆解；输入/输出单位及轮序见头文件。 */
void Motor_Chassis_Inverse(float vx, float vy, float wz, float wheel_speed[3])
{
    float rotate_speed = CHASSIS_CENTER_DISTANCE * wz;
    wheel_speed[0] = (-vy + rotate_speed) / CHASSIS_WHEEL_RADIUS;
    wheel_speed[1] = (0.8660254f * vx + 0.5f * vy + rotate_speed) / CHASSIS_WHEEL_RADIUS;
    wheel_speed[2] = (-0.8660254f * vx + 0.5f * vy + rotate_speed) / CHASSIS_WHEEL_RADIUS;
}

/** @brief 三轮正解；输入/输出单位及轮序见头文件。 */
void Motor_Chassis_Forward(const float wheel_speed[3], float body_speed[3])
{
    float rear = wheel_speed[0] * CHASSIS_WHEEL_RADIUS;
    float right_front = wheel_speed[1] * CHASSIS_WHEEL_RADIUS;
    float left_front = wheel_speed[2] * CHASSIS_WHEEL_RADIUS;
    body_speed[0] = (right_front - left_front) / 1.7320508f;
    body_speed[1] = (-2.0f * rear + right_front + left_front) / 3.0f;
    body_speed[2] = (rear + right_front + left_front) / (3.0f * CHASSIS_CENTER_DISTANCE);
}
