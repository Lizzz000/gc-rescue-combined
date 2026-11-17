#ifndef LIVE_TUNING_H
#define LIVE_TUNING_H

#include <stdint.h>

/* 仅现场调试构建使用；主机先写 command/args，最后递增 request_seq 提交。
 * 禁止通过调试器写入状态变量；motion 的主机心跳必须至少每 100 ms 更新。
 */
extern volatile uint32_t tuning_request_seq, tuning_command, tuning_heartbeat;
extern volatile uint32_t tuning_ack_seq, tuning_phase, tuning_error, tuning_sample_seq;
#define TUNING_SAMPLE_COUNT 56U
#define TUNING_STATIC_DRIFT_MAX_M 0.010f
#define TUNING_RETURN_POSITION_TOLERANCE_M 0.020f
#define TUNING_RETURN_YAW_TOLERANCE_RAD 0.17453292519943295f
#define TUNING_RETURN_STOP_POSITION_M 0.010f
#define TUNING_RETURN_STOP_YAW_RAD 0.06981317007977318f
#define TUNING_RETURN_SETTLE_MS 1500U
#define TUNING_RETURN_START_MS 1000U
extern volatile float tuning_args[8], tuning_sample[TUNING_SAMPLE_COUNT];

enum {
    TUNING_IDLE = 0U, TUNING_PREPARING = 1U, TUNING_RUNNING = 2U,
    TUNING_BRAKING = 3U, TUNING_DONE = 4U, TUNING_FAULT = 5U
};
enum {
    TUNING_STOP = 1U, TUNING_READ_PARAM = 2U, TUNING_CONFIG = 3U, TUNING_MOVE = 4U,
    TUNING_ODOM_STATIC = 5U, TUNING_ODOM_DISTANCE = 6U, TUNING_RETURN_HOLD = 7U
};
enum { TUNING_ERROR_ODOMETRY = 9U, TUNING_ERROR_ACTION_TIMEOUT = 10U };

/* STATIC: args[3] 零目标采样时间 100..1500 ms，其他参数全零；不要求采样期间
 * 每一拍都已确认停车。采样后最多 500 ms 等待原三组新反馈确认停稳；采样及确认
 * 期间原点欧氏漂移不得超过 10 mm，反馈/IMU/里程计失效或心跳中断仍立即制动。
 * DISTANCE: args[0] 前进 1 / 后退 2，args[1] 距离 0.01..1.0 m，
 * args[2] 限速 0.02..0.10 m/s，args[3] 独立硬时限 100..20000 ms。
 * 两者均先确认停稳、重置里程计并等待新有效基线；不提供无限期使能。
 * RETURN_HOLD: args=[世界目标x(m), y(m), yaw(rad,-pi..pi), 线限速(.02.. .20m/s),
 * 角限速(.10.. .50rad/s), 硬时限(100..20000ms), 0, 0]；不重置坐标。
 * 起步到目标不得超过1m，运行/保持位置不得离开起步点1.3m。连续全向接近目标，
 * 达到1cm/4deg内层范围后只发零目标，在1500ms内确认原三新组停车，最终仍须
 * 在2cm/10deg外层范围内，DONE保持使能；粗定位航向余量不改变4deg停车触发。
 * 成功后的零目标HOLD无需心跳，但持续检查反馈/IMU/odom；只接受新RETURN或STOP。
 * 首次速度提交前若停车资格暂时撤销，零目标等待最多1000ms；已起步后拒绝立即中止。
 * 运动/到点确认仍受300ms心跳和硬时限保护，任何故障或显式STOP均制动后失能。
 * sample[40..47]: 当前 odom x/y/yaw/vx/vy/wheel_wz/valid/age_ms。
 * sample[48..55]: 结束时保留 x/y/yaw/valid/stopped/elapsed_ms/progress_m/goal_m；
 *                 失能后的 valid=0 不覆盖结束前证据，STOP 也不清除该证据。
 * RETURN的progress/goal为起步目标距离减去当前剩余距离/起步目标距离。
 */

/* Init 不发送 CAN；Loop/Snapshot 只由 test_task 调用，分别在 Chassis_Loop 前/后。 */
void Live_Tuning_Init(void);
void Live_Tuning_Loop(void);
void Live_Tuning_Snapshot(void);

#endif
