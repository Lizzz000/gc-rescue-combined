#include "chassis_config.h"

/* 沿用实车调参的 MIT 阻尼及平移摩擦补偿，速度为电机轴量。 */
const Chassis_Wheel_Config_t chassis_wheel_config[3] =
{
    {1U, -1.0f, 1.0f, 0.22f, 0.000f, 40.0f},
    {2U, -1.0f, 1.0f, 0.27f, 0.015f, 40.0f},
    {3U, -1.0f, 1.0f, 0.22f, 0.015f, 40.0f}
};
