#ifndef PLANAR_ODOMETRY_H
#define PLANAR_ODOMETRY_H

#include <stdint.h>

/* 将一段车体位移按起止航向映射至世界系，更新 x/y。非有限结果返回 0 并保留原坐标。 */
uint8_t Planar_Odometry_Integrate(double *x_m, double *y_m, double dx, double dy,
                                  float start_yaw, float end_yaw);

#endif
