#ifndef LIVE_TUNING_H
#define LIVE_TUNING_H

#include <stdint.h>

/* 仅现场调试构建使用；主机先写 command/args，最后递增 request_seq 提交。
 * 禁止通过调试器写入状态变量；motion 的主机心跳必须至少每 100 ms 更新。
 */
extern volatile uint32_t tuning_request_seq, tuning_command, tuning_heartbeat;
extern volatile uint32_t tuning_ack_seq, tuning_phase, tuning_error, tuning_sample_seq;
#define TUNING_SAMPLE_COUNT 56U
extern volatile float tuning_args[8], tuning_sample[TUNING_SAMPLE_COUNT];

enum {
    TUNING_IDLE = 0U, TUNING_PREPARING = 1U, TUNING_RUNNING = 2U,
    TUNING_BRAKING = 3U, TUNING_DONE = 4U, TUNING_FAULT = 5U
};
enum {
    TUNING_STOP = 1U, TUNING_READ_PARAM = 2U, TUNING_CONFIG = 3U, TUNING_MOVE = 4U,
    TUNING_ODOM_STATIC = 5U, TUNING_ODOM_DISTANCE = 6U
};
enum { TUNING_ERROR_ODOMETRY = 9U, TUNING_ERROR_ACTION_TIMEOUT = 10U };

/* STATIC: args[3] 保持时间 100..1500 ms，其他参数全零。
 * DISTANCE: args[0] 前进 1 / 后退 2，args[1] 距离 0.01..0.20 m，
 * args[2] 限速 0.02..0.10 m/s，args[3] 独立硬时限 100..10000 ms。
 * 两者均先确认停稳、重置里程计并等待新有效基线；不提供无限期使能。
 * sample[40..47]: 当前 odom x/y/yaw/vx/vy/wheel_wz/valid/age_ms。
 * sample[48..55]: 结束时保留 x/y/yaw/valid/stopped/elapsed_ms/progress_m/goal_m；
 *                 失能后的 valid=0 不覆盖结束前证据，STOP 也不清除该证据。
 */

/* Init 不发送 CAN；Loop/Snapshot 只由 test_task 调用，分别在 Chassis_Loop 前/后。 */
void Live_Tuning_Init(void);
void Live_Tuning_Loop(void);
void Live_Tuning_Snapshot(void);

#endif
