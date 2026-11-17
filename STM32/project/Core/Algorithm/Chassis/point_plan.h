#ifndef POINT_PLAN_H
#define POINT_PLAN_H

#include <stdint.h>

typedef struct
{
    double start[2], goal[2], length_m;
    float goal_yaw_rad, yaw_delta_rad;
    uint8_t valid;
} Point_Plan_t;

typedef struct
{
    float linear_limit_mps, angular_limit_rad_s;
    float linear_deceleration_mps2, angular_deceleration_rad_s2, latency_s;
    float position_tolerance_m, yaw_tolerance_rad;
} Point_Plan_Config_t;

typedef struct
{
    float vx_mps, vy_mps, wz_rad_s;
    double remaining_m;
    float yaw_error_rad;
    uint8_t arrived;
} Point_Plan_Output_t;

/* 无障碍平面内最短欧氏直线，航向取[-pi,pi]最短转角；不假设轮式车必须沿车头方向。
 * start/goal=[世界x(m),世界y(m),航向(rad)]；length 是路径下界，不是实走里程。
 * 不做避障、地图搜索或盲目按时间推进。每拍用有效里程计重算到目标的最短剩余线段。
 */
uint8_t Point_Plan_Init(Point_Plan_t *plan, const double start[3], const double goal[3]);
uint8_t Point_Plan_Step(const Point_Plan_t *plan, const Point_Plan_Config_t *config,
                        const double pose[3], Point_Plan_Output_t *output);

#endif
