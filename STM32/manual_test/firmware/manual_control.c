/* 手动控制只消费类型 2。速度定点与底盘参数复用本车 chassis。自动固件不编译这条路径。
 * F407/RTOS 适配：USB 任务只保存协议快照，现有 test_task 唯一调用底盘接口。
 * 不引入旧车的命令队列、输入框架或新任务；本轮不启用运输动作。
 */
#include "manual_control.h"
#include "manual_link.h"
#include "chassis.h"
#include "FreeRTOS.h"
#include "task.h"
#include <math.h>

volatile Manual_Control_State_t manual_control_state;
static uint32_t blocked_sequence, invalid_sequence, zero_sequence, state_started_ms;
static uint32_t disconnect_sequence;
static uint8_t power_on_pending, control_claimed, require_release, enable_waiting;

/** @brief 判断原始定点三分量是否均为零。
 * @param[in] velocity 当前手动速度快照，非空。
 * @return 全零返回 1，否则 0。
 */
static uint8_t Is_Zero(const Manual_CAN_Velocity_t *velocity)
{
    return velocity->vx == 0 && velocity->vy == 0 && velocity->wz == 0;
}

/** @brief 锁定本轮停车，清除目标；后续报文不能延长制动截止。
 * @param[in] now_ms 当前 RTOS 毫秒。
 * @return 无。
 */
static void Begin_Braking(uint32_t now_ms, uint8_t fault)
{
    Chassis_Stop();
    require_release = fault;
    control_claimed = 0U;
    state_started_ms = now_ms;
    manual_control_state = MANUAL_BRAKING;
}

/** @brief 撤销使能并记住失能后的接收边界，禁止停车期间的零速自动重启。
 * @return 无；失能帧仍由 Chassis_Loop 提交。
 */
static void Disarm(void)
{
    Manual_CAN_Velocity_t boundary;
    Chassis_Disable();
    Manual_CAN_Get_Velocity(&boundary);
    blocked_sequence = boundary.sequence;
    manual_control_state = MANUAL_DISARMED;
}

/** @brief 清除本模块状态；底盘初始化由同一任务完成。 @return 无。 */
void Manual_Control_Init(void)
{
    blocked_sequence = invalid_sequence = zero_sequence = state_started_ms = 0U;
    disconnect_sequence = 0U;
    power_on_pending = 1U;
    control_claimed = require_release = enable_waiting = 0U;
    manual_control_state = MANUAL_DISARMED;
}

void Manual_Control_Arm_Request(void)
{
    require_release = 0U;
    const uint8_t zero[7] = {1U};
    Manual_CAN_Receive(zero, sizeof(zero));
}

uint8_t Manual_Control_Release(void)
{
    Manual_CAN_Velocity_t velocity;
    Manual_CAN_Get_Velocity(&velocity);
    if ((manual_control_state != MANUAL_READY && manual_control_state != MANUAL_RUNNING) ||
        !control_claimed || !velocity.valid || !Is_Zero(&velocity) ||
        velocity.invalid_sequence != invalid_sequence || velocity.disconnect_sequence != disconnect_sequence ||
        (uint32_t)((uint32_t)xTaskGetTickCount() - velocity.received_ms) >= MANUAL_VELOCITY_TIMEOUT_MS)
        return 0U;
    /* 零帧与归还可在同一串口批次到达，先让底盘接下零目标，再撤去会话租约。 */
    if (manual_control_state == MANUAL_RUNNING)
        Chassis_Hold();
    if (!Chassis_Can_Release())
        return 0U;
    blocked_sequence = velocity.sequence;
    control_claimed = 0U;
    manual_control_state = MANUAL_READY;
    return 1U;
}

/** @brief 处理一次最新速度及保护期限；接口条件见头文件。 @return 无。 */
void Manual_Control_Loop(void)
{
    Manual_CAN_Velocity_t velocity;
    Manual_CAN_Get_Velocity(&velocity);
    const uint32_t now_ms = (uint32_t)xTaskGetTickCount();
    const uint8_t revoked = velocity.invalid_sequence != invalid_sequence;
    const uint8_t stop_requested = velocity.zero_sequence != zero_sequence;
    const uint8_t disconnected = velocity.disconnect_sequence != disconnect_sequence;
    invalid_sequence = velocity.invalid_sequence;
    zero_sequence = velocity.zero_sequence;
    disconnect_sequence = velocity.disconnect_sequence;
    const uint8_t fresh = velocity.valid &&
        (uint32_t)(now_ms - velocity.received_ms) < MANUAL_VELOCITY_TIMEOUT_MS;
    const uint8_t zero = Is_Zero(&velocity);

    if (revoked)
    {
        power_on_pending = 0U;
        require_release = !velocity.explicit_release;
        blocked_sequence = velocity.sequence;
    }
    if (manual_control_state == MANUAL_BRAKING)
    {
        if (!Chassis_Is_Enabled() || Chassis_Is_Stopped() ||
            (uint32_t)(now_ms - state_started_ms) >= MANUAL_BRAKE_TIMEOUT_MS)
            Disarm();
        return;
    }
    if (manual_control_state == MANUAL_DISARMED)
    {
        const uint8_t request = fresh && zero && velocity.sequence != blocked_sequence;
        if ((!power_on_pending && !request) || require_release || !Chassis_Attitude_Ready())
        {
            enable_waiting = 0U;
            return;
        }
        if (!enable_waiting)
        {
            enable_waiting = 1U;
            state_started_ms = now_ms;
        }
        const uint8_t can_enable = Chassis_Can_Enable();
        if (!can_enable && (uint32_t)(now_ms - state_started_ms) < MANUAL_PREPARE_TIMEOUT_MS)
            return;
        enable_waiting = power_on_pending = 0U;
        blocked_sequence = velocity.sequence;
        if (!can_enable || !Chassis_Enable())
        {
            require_release = 1U;
            return;
        }
        control_claimed = request;
        state_started_ms = now_ms;
        manual_control_state = MANUAL_PREPARING;
        return;
    }
    if (!Chassis_Is_Enabled())
    {
        /* 底盘自身的姿态/CAN 保护已经失能；不能由缓存里的零帧恢复。 */
        blocked_sequence = velocity.sequence;
        require_release = 1U;
        control_claimed = 0U;
        manual_control_state = MANUAL_DISARMED;
        return;
    }
    /* 按协议用除法还原定点值，避免缩放系数预先舍入影响合法边界。 */
    const float vx = velocity.vx / MANUAL_VELOCITY_FIXED_POINT_SCALE;
    const float vy = velocity.vy / MANUAL_VELOCITY_FIXED_POINT_SCALE;
    const float wz = velocity.wz / MANUAL_VELOCITY_FIXED_POINT_SCALE;
    if (revoked || (control_claimed && (!fresh || disconnected)))
    {
        /* 主动撤权可由后续新零帧接管；链路故障须先收到显式撤权。 */
        Begin_Braking(now_ms, !(revoked && velocity.explicit_release));
        return;
    }
    if (!control_claimed && fresh && zero && velocity.sequence != blocked_sequence)
        control_claimed = 1U;
    if (control_claimed && (hypotf(vx, vy) > CHASSIS_STRAIGHT_MAX_SPEED_MPS ||
                            fabsf(wz) > CHASSIS_VECTOR_MAX_WZ_RAD_S))
    {
        Begin_Braking(now_ms, 1U);
        return;
    }
    if (manual_control_state == MANUAL_PREPARING)
    {
        /* 使能只提交零速。未接管时忽略非零输入。 */
        if ((uint32_t)(now_ms - state_started_ms) >= MANUAL_PREPARE_TIMEOUT_MS ||
            (control_claimed && !zero))
            Begin_Braking(now_ms, 1U);
        else if (Chassis_Motion_Ready())
            manual_control_state = MANUAL_READY;
        return;
    }
    if (!Chassis_Motors_Ready() || (manual_control_state == MANUAL_READY && !Chassis_Attitude_Ready()))
    {
        Begin_Braking(now_ms, 1U);
        return;
    }
    if (!control_claimed)
        return;
    /* 运行中同一批收到全零和非零时，仍执行明确的零速停车。 */
    if (manual_control_state == MANUAL_RUNNING && (zero || stop_requested))
    {
        Chassis_Hold();
        blocked_sequence = velocity.sequence;
        manual_control_state = MANUAL_READY;
        return;
    }
    if (zero || (manual_control_state == MANUAL_READY && velocity.sequence == blocked_sequence))
        return;
    if (Chassis_Set_Velocity(vx, vy, wz))
        manual_control_state = MANUAL_RUNNING;
    else
        Begin_Braking(now_ms, 1U);
}
