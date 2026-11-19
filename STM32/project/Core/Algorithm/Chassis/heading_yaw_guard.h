#ifndef HEADING_YAW_GUARD_H
#define HEADING_YAW_GUARD_H

#include <stdint.h>

typedef enum
{
    HEADING_YAW_INVALID,
    HEADING_YAW_OBSERVED,
    HEADING_YAW_PREDICTED
} Heading_Yaw_Status_t;

typedef struct
{
    float yaw_rad;
    float previous_gyro_rad_s;
    uint32_t sample_time_ms;
    uint8_t rejected_run;
    Heading_Yaw_Status_t status;
} Heading_Yaw_Guard_t;

/* dt 为 AHRS 本拍实际积分时长，源时间戳用于拒绝重复和倒序观测。
 * 连续三次 YAW 跳点后锁存，重新初始化才解除。 */
Heading_Yaw_Status_t Heading_Yaw_Guard_Update(Heading_Yaw_Guard_t *guard, float yaw_rad,
                                               float gyro_rad_s, uint32_t sample_time_ms, float dt);

#endif
