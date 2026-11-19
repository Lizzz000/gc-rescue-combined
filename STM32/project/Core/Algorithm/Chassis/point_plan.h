#ifndef POINT_PLAN_H
#define POINT_PLAN_H

#include <stdint.h>

typedef struct
{
    double start[2];
    double goal[2];
    double length_m;
    float goal_yaw_rad;
    float yaw_delta_rad;
    float remaining_yaw_rad;
    uint8_t valid;
    uint8_t yaw_started;
} Point_Plan_t;

typedef struct
{
    float linear_limit_mps;
    float angular_limit_rad_s;
    float linear_deceleration_mps2;
    float angular_deceleration_rad_s2;
    float latency_s;
    float position_tolerance_m;
    float yaw_tolerance_rad;
} Point_Plan_Config_t;

typedef struct
{
    float vx_mps; /* 世界 x/y 平移速度；角速度逆时针为正。 */
    float vy_mps;
    float wz_rad_s;
    double remaining_m;
    float yaw_error_rad;
    uint8_t arrived;
} Point_Plan_Output_t;

/* 无障碍平面内最短欧氏直线，航向取[-pi,pi]最短转角；不假设轮式车必须沿车头方向。
 * start/goal=[世界x(m),世界y(m),航向(rad)]；length 是路径下界，不是实走里程。
 * 每拍以世界位姿重算剩余路线，并用车体系实测速度 [vx,vy,wz] 抑制侧滑和超速接近。
 */
uint8_t Point_Plan_Init(Point_Plan_t *plan, const double start[3], const double goal[3]);
uint8_t Point_Plan_Step(Point_Plan_t *plan, const Point_Plan_Config_t *config, const double pose[3],
                        const float measured_body[3], Point_Plan_Output_t *output);

#endif
