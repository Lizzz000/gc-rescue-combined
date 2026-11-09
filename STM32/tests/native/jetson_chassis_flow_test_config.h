#ifndef JETSON_CHASSIS_FLOW_TEST_CONFIG_H
#define JETSON_CHASSIS_FLOW_TEST_CONFIG_H
#include "jetson_auto_config.h"

/* 只在主机反例中清空视觉容差，其余全部使用真实部署宏。 */
#ifdef TEST_JETSON_CHASSIS_BAD_WINDOW
#undef JETSON_AUTO_TARGET_TOL_X
#define JETSON_AUTO_TARGET_TOL_X 0.0f
#endif
#endif
