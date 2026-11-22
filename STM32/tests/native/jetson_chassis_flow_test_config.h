#ifndef JETSON_CHASSIS_FLOW_TEST_CONFIG_H
#define JETSON_CHASSIS_FLOW_TEST_CONFIG_H
#include "jetson_auto_config.h"

/* 实车安全区直行距离仍待标定；离线用 0.4 m 验证完整规划和停车。 */
#undef JETSON_AUTO_SAFETY_STRAIGHT_M
#define JETSON_AUTO_SAFETY_STRAIGHT_M 0.4f

/* 合成帧使用固定中心，现场相机标定不参与单趟联调回归。 */
#undef JETSON_AUTO_TARGET_X
#undef JETSON_AUTO_TARGET_Y
#define JETSON_AUTO_TARGET_X 0.50f
#define JETSON_AUTO_TARGET_Y 0.75f

/* 主机反例在合成窗口的基础上清空横向容差。 */
#ifdef TEST_JETSON_CHASSIS_BAD_WINDOW
#undef JETSON_AUTO_TARGET_TOL_X
#define JETSON_AUTO_TARGET_TOL_X 0.0f
#endif
#endif
