#ifndef TRACKED_VISUAL_SERVO_H
#define TRACKED_VISUAL_SERVO_H

#include "adaptive_kalman.h"
#include "visual_servo.h"

/* 使用本帧跟踪结果更新视觉速度目标。跟踪器尚未收到有效帧时速度归零。 */
void Tracked_Visual_Servo_Update(Visual_Servo_t *servo, const Visual_Servo_Config_t *config,
                                 const AdaptiveKalman *tracker, const float raw[2], const float goal[2],
                                 const float tolerance[2], float seconds);

#endif
