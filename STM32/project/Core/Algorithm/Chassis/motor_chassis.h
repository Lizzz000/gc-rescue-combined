#ifndef MOTOR_CHASSIS_H
#define MOTOR_CHASSIS_H

/* 旧小车 motor_chassis.h：70 mm 轮径，中心到轮接地点 108 mm。 */
#define CHASSIS_WHEEL_RADIUS 0.035f
#define CHASSIS_CENTER_DISTANCE 0.108f
/* 2026-11-17：同向30圈、2/3/4/3/2rad/s，按可信YAW相对标定旋转项。
 * 108mm * 编码器29.998987圈 / YAW33.007299圈 = 98.1568mm，取98.2mm。
 * 这是等效旋转臂长，不是实测安装尺寸；平移轮径/投影保持原值。 */
#define CHASSIS_ROTATION_RADIUS 0.0982f

/* 2026-11-17 用户确认的轮序：ID1后轮、ID2右前轮、ID3左前轮。
 * x前/y左，以 L 为中心距，接地点依次 (-L,0)、(L/2,-sqrt(3)L/2)、(L/2,sqrt(3)L/2)。
 * 几何轮速正切向依次 (0,-1)、(sqrt(3)/2,1/2)、(-sqrt(3)/2,1/2)，
 * 均满足切向点乘 (-y,x) = L，因此逆时针转动的三个几何轮速同为正。
 * 实际 MIT 目标/反馈另由控制层乘安装方向 -1；此处不重复处理电机极性。
 */

/**
 * @brief 沿旧小车计算三轮全向底盘逆运动学。
 * @param[in] vx 车体纵向速度，m/s，向前为正。
 * @param[in] vy 车体横向速度，m/s，向左为正。
 * @param[in] wz 车体角速度，rad/s，俯视逆时针为正。
 * @param[out] wheel_speed 非空三元素数组，后/右前/左前几何轮速，rad/s。
 * @return 无。
 * @note 调用方提供有限输入；电机方向和齿比由控制层处理。
 *       轮位、几何正方向和实际电机方向不能混用，见上方坐标约定。
 */
void Motor_Chassis_Inverse(float vx, float vy, float wz, float wheel_speed[3]);

/**
 * @brief 沿旧小车计算三轮全向底盘正运动学。
 * @param[in] wheel_speed 非空三元素数组，后/右前/左前几何轮速，rad/s。
 * @param[out] body_speed 非空三元素数组，依次 vx(m/s)、vy(m/s)、wz(rad/s)。
 * @return 无。
 * @note 输入须已去除电机方向与齿比，坐标约定同逆解。
 */
void Motor_Chassis_Forward(const float wheel_speed[3], float body_speed[3]);

#endif
