#include "visual_servo.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>

int main(void)
{
    const Visual_Servo_Config_t config = {
        .max_speed_mps = .30f, .acceleration_mps2 = .40f, .deceleration_mps2 = .35f,
        .latency_s = .12f, .response_q_per_m = {10000, 10000},
    };
    const float goal[2] = {5000, 7500}, tolerance[2] = {100, 100};
    float raw[2] = {4500, 7000};
    Visual_Servo_Estimate_t estimate = {.position = {4500, 7000}, .accepted_mask = 3U};
    Visual_Servo_t servo = {0};
    Visual_Servo_Update(&servo, &config, &estimate, raw, goal, tolerance, .02f);
    assert(fabsf(hypotf(servo.speed[0], servo.speed[1]) - .008f) < 1e-6f);
    assert(servo.speed[0] > 0 && servo.speed[0] == servo.speed[1]);

    /* 两轴等距，Y 的图像收敛速度较快，控制律应较早降低 Y 的速度。 */
    servo = (Visual_Servo_t){.speed = {.2f, .2f}};
    estimate.velocity[1] = 8000;
    Visual_Servo_Update(&servo, &config, &estimate, raw, goal, tolerance, .02f);
    assert(servo.speed[1] > 0 && servo.speed[1] < servo.speed[0]);

    /* X 离群按配置减速，Y 的原始观测入窗立即归零。 */
    const float previous_x = servo.speed[0];
    estimate.accepted_mask = 2U;
    raw[1] = goal[1];
    Visual_Servo_Update(&servo, &config, &estimate, raw, goal, tolerance, .02f);
    assert(fabsf(servo.speed[0] - (previous_x - .007f)) < 1e-6f);
    assert(servo.speed[1] == 0);

    /* X 重新接受且反向时先过零，再开始反向加速。 */
    estimate.accepted_mask = 3U;
    estimate.position[0] = raw[0] = 5500;
    Visual_Servo_Update(&servo, &config, &estimate, raw, goal, tolerance, .02f);
    assert(servo.speed[0] == 0);
    Visual_Servo_Update(&servo, &config, &estimate, raw, goal, tolerance, .02f);
    assert(fabsf(servo.speed[0] + .008f) < 1e-6f);
    Visual_Servo_Update(&servo, &config, NULL, raw, goal, tolerance, .02f);
    assert(servo.speed[0] == 0 && servo.speed[1] == 0);
    puts("PASS standalone visual servo: acceleration, image response, axis rejection, window and reversal");
    return 0;
}
