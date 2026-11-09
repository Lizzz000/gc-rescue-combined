/* 来源：旧小车 Core/Motor/motor_chassis.c；沿用三轮几何。
 * 2026-11-08：按用户实车反馈对调前进/后退，正逆解同时反向纵向分量。
 * 此处只做计算，输入检查由底盘动作入口负责，不依赖主控或 HAL。
 */
#include "motor_chassis.h"

/** @brief 三轮逆解；输入/输出单位及轮序见头文件。 */
void Motor_Chassis_Inverse(float vx, float vy, float wz, float wheel_speed[3])
{
    float rotate_speed = CHASSIS_CENTER_DISTANCE * wz;
    wheel_speed[0] = (vy + rotate_speed) / CHASSIS_WHEEL_RADIUS;
    wheel_speed[1] = (0.8660254f * vx - 0.5f * vy + rotate_speed) / CHASSIS_WHEEL_RADIUS;
    wheel_speed[2] = (-0.8660254f * vx - 0.5f * vy + rotate_speed) / CHASSIS_WHEEL_RADIUS;
}

/** @brief 三轮正解；输入/输出单位及轮序见头文件。 */
void Motor_Chassis_Forward(const float wheel_speed[3], float body_speed[3])
{
    float front = wheel_speed[0] * CHASSIS_WHEEL_RADIUS;
    float left = wheel_speed[1] * CHASSIS_WHEEL_RADIUS;
    float right = wheel_speed[2] * CHASSIS_WHEEL_RADIUS;
    body_speed[0] = (left - right) / 1.7320508f;
    body_speed[1] = (2.0f * front - left - right) / 3.0f;
    body_speed[2] = (front + left + right) / (3.0f * CHASSIS_CENTER_DISTANCE);
}
