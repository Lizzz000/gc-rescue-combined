#ifndef LIVE_TUNING_H
#define LIVE_TUNING_H

#include <stdint.h>

/* 仅现场调试构建使用；主机先写 command/args，最后递增 request_seq 提交。
 * 禁止通过调试器写入状态变量；motion 的主机心跳必须至少每 100 ms 更新。
 */
extern volatile uint32_t tuning_request_seq, tuning_command, tuning_heartbeat;
extern volatile uint32_t tuning_ack_seq, tuning_phase, tuning_error, tuning_sample_seq;
extern volatile float tuning_args[8], tuning_sample[40];

enum {
    TUNING_IDLE = 0U, TUNING_PREPARING = 1U, TUNING_RUNNING = 2U,
    TUNING_BRAKING = 3U, TUNING_DONE = 4U, TUNING_FAULT = 5U
};
enum {
    TUNING_STOP = 1U, TUNING_READ_PARAM = 2U, TUNING_CONFIG = 3U, TUNING_MOVE = 4U
};

/* Init 不发送 CAN；Loop/Snapshot 只由 test_task 调用，分别在 Chassis_Loop 前/后。 */
void Live_Tuning_Init(void);
void Live_Tuning_Loop(void);
void Live_Tuning_Snapshot(void);

#endif
