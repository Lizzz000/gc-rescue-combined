#ifndef MOTOR_CHASSIS_H
#define MOTOR_CHASSIS_H

/* 旧小车 motor_chassis.h：70 mm 轮径，中心到轮接地点 108 mm。 */
#define CHASSIS_WHEEL_RADIUS 0.035f
#define CHASSIS_CENTER_DISTANCE 0.108f

/**
 * @brief 沿旧小车计算三轮全向底盘逆运动学。
 * @param[in] vx 车体纵向速度，m/s，向前为正。
 * @param[in] vy 车体横向速度，m/s，向左为正。
 * @param[in] wz 车体角速度，rad/s，俯视逆时针为正。
 * @param[out] wheel_speed 非空三元素数组，前/左后/右后几何轮速，rad/s。
 * @return 无。
 * @note 调用方提供有限输入；电机方向和齿比由控制层处理。
 *       纵向正方向按 2026-11-08 用户确认的车头方向校正。
 */
void Motor_Chassis_Inverse(float vx, float vy, float wz, float wheel_speed[3]);

/**
 * @brief 沿旧小车计算三轮全向底盘正运动学。
 * @param[in] wheel_speed 非空三元素数组，前/左后/右后几何轮速，rad/s。
 * @param[out] body_speed 非空三元素数组，依次 vx(m/s)、vy(m/s)、wz(rad/s)。
 * @return 无。
 * @note 输入须已去除电机方向与齿比，坐标约定同逆解。
 */
void Motor_Chassis_Forward(const float wheel_speed[3], float body_speed[3]);

#endif
