/* 协议来源：独立手动 0x17F（与 Jetson 0x180 同格式）；控制与参数复用本车 chassis。
 * F407/RTOS 适配：ISR 只保存协议快照，现有 test_task 唯一调用底盘接口。
 * 不引入旧车的命令队列、输入框架或新任务；本轮不启用运输动作。
 */
#include "manual_control.h"
#include "manual_can.h"
#include "chassis.h"
#include "FreeRTOS.h"
#include "task.h"
#include <math.h>

volatile Manual_Control_State_t manual_control_state;
static uint32_t blocked_sequence, invalid_sequence, zero_sequence, state_started_ms;
static uint8_t preparing_after_stop;

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
static void Begin_Braking(uint32_t now_ms)
{
    Chassis_Stop();
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
    preparing_after_stop = 0U;
    manual_control_state = MANUAL_DISARMED;
}

/** @brief 处理一次最新速度及保护期限；接口条件见头文件。 @return 无。 */
void Manual_Control_Loop(void)
{
    Manual_CAN_Velocity_t velocity;
    Manual_CAN_Get_Velocity(&velocity);
    const uint32_t now_ms = (uint32_t)xTaskGetTickCount();
    const uint8_t revoked = velocity.invalid_sequence != invalid_sequence;
    const uint8_t stop_requested = velocity.zero_sequence != zero_sequence;
    invalid_sequence = velocity.invalid_sequence;
    zero_sequence = velocity.zero_sequence;
    const uint8_t fresh = velocity.valid &&
        (uint32_t)(now_ms - velocity.received_ms) < MANUAL_VELOCITY_TIMEOUT_MS;
    const uint8_t zero = Is_Zero(&velocity);

    if (manual_control_state == MANUAL_BRAKING)
    {
        if (!Chassis_Is_Enabled() || Chassis_Is_Stopped() ||
            (uint32_t)(now_ms - state_started_ms) >= MANUAL_BRAKE_TIMEOUT_MS)
            Disarm();
        return;
    }
    if (manual_control_state == MANUAL_DISARMED)
    {
        if (velocity.sequence == blocked_sequence)
            return;
        if (!(fresh && zero))
        {
            blocked_sequence = velocity.sequence;
            return;
        }
        /* 使能失败时留下这一帧零速，下一拍还能再试。FD 还没发完时不能把零速吃掉。 */
        if (!Chassis_Enable())
            return;
        blocked_sequence = velocity.sequence;
        state_started_ms = now_ms;
        preparing_after_stop = 0U;
        manual_control_state = MANUAL_PREPARING;
        return;
    }
    if (!Chassis_Is_Enabled())
    {
        /* 底盘自身的姿态/CAN 保护已经失能；不能由缓存里的零帧恢复。 */
        blocked_sequence = velocity.sequence;
        manual_control_state = MANUAL_DISARMED;
        return;
    }
    /* 按协议用除法还原定点值，避免缩放系数预先舍入影响合法边界。 */
    const float vx = velocity.vx / MANUAL_VELOCITY_FIXED_POINT_SCALE;
    const float vy = velocity.vy / MANUAL_VELOCITY_FIXED_POINT_SCALE;
    const float wz = velocity.wz / MANUAL_VELOCITY_FIXED_POINT_SCALE;
    if (revoked || !fresh || hypotf(vx, vy) > CHASSIS_STRAIGHT_MAX_SPEED_MPS ||
        fabsf(wz) > CHASSIS_VECTOR_MAX_WZ_RAD_S)
    {
        Begin_Braking(now_ms);
        return;
    }
    if (manual_control_state == MANUAL_PREPARING)
    {
        /* 使能握手须保持零速；运行后停车可忽略早到的非零帧，但反馈仍须正常。 */
        if ((uint32_t)(now_ms - state_started_ms) >= MANUAL_PREPARE_TIMEOUT_MS ||
            (preparing_after_stop ? !Chassis_Motors_Ready() : !zero))
            Begin_Braking(now_ms);
        else if (Chassis_Is_Stopped())
        {
            if (preparing_after_stop)
            {
                /* 停稳后再取接收边界，不能把停车期间最后一帧留给 READY 执行。 */
                Manual_CAN_Get_Velocity(&velocity);
                blocked_sequence = velocity.sequence;
            }
            manual_control_state = MANUAL_READY;
        }
        return;
    }
    if (!Chassis_Motors_Ready())
    {
        Begin_Braking(now_ms);
        return;
    }
    /* 运行中同一批收到全零和非零时，仍执行明确的零速停车。 */
    if (manual_control_state == MANUAL_RUNNING && (zero || stop_requested))
    {
        Chassis_Stop();
        state_started_ms = now_ms;
        preparing_after_stop = 1U;
        manual_control_state = MANUAL_PREPARING;
        return;
    }
    if (zero || (manual_control_state == MANUAL_READY && velocity.sequence == blocked_sequence))
        return;
    if (Chassis_Set_Velocity(vx, vy, wz))
        manual_control_state = MANUAL_RUNNING;
    else
        Begin_Braking(now_ms);
}
