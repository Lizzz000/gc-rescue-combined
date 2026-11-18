#ifndef SPIN_OBSERVER_H
#define SPIN_OBSERVER_H
#include <stdint.h>

/* 专用30/35圈测试：以用户选定的YAW计圈，编码器/gyro保留原始累计作对照。
 * YAW与gyro同源，二者一致不能证明独立绝对角度精度。 */
typedef struct {
    double encoder_rad, yaw_rad, gyro_rad;
    float previous_position[3], previous_yaw, previous_rate;
    uint32_t previous_ms;
    uint8_t initialized;
} Spin_Observer_t;

uint8_t Spin_Observer_Update(Spin_Observer_t *state, const float position[3],
                             float yaw_rad, float gyro_rad_s, uint32_t now_ms);
float Spin_Test_Speed(double yaw_rad, unsigned turns, unsigned *stage);
#define SPIN_DEFAULT_TURNS 30U
#define SPIN_EXTENDED_TURNS 35U
#define SPIN_RADIANS_PER_TURN 6.28318530717958647692
#endif
