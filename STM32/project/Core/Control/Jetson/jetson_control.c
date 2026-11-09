/* 协议来源：Jetson/bridge_pkg 的 0x180；控制与参数复用本车 chassis。
 * F407/RTOS 适配：ISR 只保存协议快照，现有 test_task 唯一调用底盘接口。
 * 不引入旧车的命令队列、输入框架或新任务；本轮不启用运输动作。
 */
#include "jetson_control.h"
#include "jetson_can.h"
#include "chassis.h"
#include "dm_motor.h"
#include "FreeRTOS.h"
#include "task.h"
#include <math.h>

volatile Jetson_Control_State_t jetson_control_state;
static uint32_t blocked_sequence, invalid_sequence, zero_sequence, state_started_ms;
static uint8_t preparing_after_stop;

/** @brief 判断原始定点三分量是否均为零。
 * @param[in] data 当前协议快照，非空。
 * @return 全零返回 1，否则 0。
 */
static uint8_t Is_Zero(const Jetson_CAN_Data_t *data)
{
    return data->velocity.vx == 0 && data->velocity.vy == 0 && data->velocity.wz == 0;
}

/** @brief 用与反馈 ISR 相同的时基核对三轮是否使能且反馈仍新鲜。
 * @return 三轮反馈存在、状态正常且年龄不超过 30 ms 时返回 1。
 */
uint8_t Jetson_Control_Motors_Ready(void)
{
    uint8_t ready = 1U;
    taskENTER_CRITICAL();
    uint32_t now_ms = (uint32_t)xTaskGetTickCount();
    for (unsigned i = 0U; i < 3U; ++i)
        if (dm_motor[i].id != i + 1U || dm_motor[i].state != 1U ||
            !isfinite(dm_motor[i].velocity) ||
            (uint32_t)(now_ms - dm_motor[i].feedback_time_ms) > JETSON_MOTOR_TIMEOUT_MS)
            ready = 0U;
    taskEXIT_CRITICAL();
    return ready;
}

/** @brief 锁定本轮停车，清除目标；后续报文不能延长制动截止。
 * @param[in] now_ms 当前 RTOS 毫秒。
 * @return 无。
 */
static void Begin_Braking(uint32_t now_ms)
{
    Chassis_Stop();
    state_started_ms = now_ms;
    jetson_control_state = JETSON_BRAKING;
}

/** @brief 撤销使能并记住失能后的接收边界，禁止停车期间的零速自动重启。
 * @return 无；失能帧仍由 Chassis_Loop 提交。
 */
static void Disarm(void)
{
    Jetson_CAN_Data_t boundary;
    Chassis_Disable();
    Jetson_CAN_Get_Data(&boundary);
    blocked_sequence = boundary.velocity.sequence;
    jetson_control_state = JETSON_DISARMED;
}

/** @brief 清除本模块状态；底盘初始化由同一任务完成。 @return 无。 */
void Jetson_Control_Init(void)
{
    blocked_sequence = invalid_sequence = zero_sequence = state_started_ms = 0U;
    preparing_after_stop = 0U;
    jetson_control_state = JETSON_DISARMED;
}

/** @brief 处理一次最新速度及保护期限；接口条件见头文件。 @return 无。 */
void Jetson_Control_Loop(void)
{
    Jetson_CAN_Data_t data;
    Jetson_CAN_Get_Data(&data);
    const uint32_t now_ms = (uint32_t)xTaskGetTickCount();
    const uint8_t revoked = data.velocity.invalid_sequence != invalid_sequence;
    const uint8_t stop_requested = data.velocity.zero_sequence != zero_sequence;
    invalid_sequence = data.velocity.invalid_sequence;
    zero_sequence = data.velocity.zero_sequence;
    const uint8_t fresh = data.velocity.valid &&
        (uint32_t)(now_ms - data.velocity.received_ms) < JETSON_VELOCITY_TIMEOUT_MS;
    const uint8_t zero = Is_Zero(&data);

    if (jetson_control_state == JETSON_BRAKING)
    {
        if (!Chassis_Is_Enabled() || Chassis_Is_Stopped() ||
            (uint32_t)(now_ms - state_started_ms) >= JETSON_BRAKE_TIMEOUT_MS)
            Disarm();
        return;
    }
    if (jetson_control_state == JETSON_DISARMED)
    {
        if (data.velocity.sequence == blocked_sequence)
            return;
        blocked_sequence = data.velocity.sequence;
        if (fresh && zero && Chassis_Enable())
        {
            state_started_ms = now_ms;
            preparing_after_stop = 0U;
            jetson_control_state = JETSON_PREPARING;
        }
        return;
    }
    if (!Chassis_Is_Enabled())
    {
        /* 底盘自身的姿态/CAN 保护已经失能；不能由缓存里的零帧恢复。 */
        blocked_sequence = data.velocity.sequence;
        jetson_control_state = JETSON_DISARMED;
        return;
    }
    /* 按协议用除法还原定点值，避免缩放系数预先舍入影响合法边界。 */
    const float vx = data.velocity.vx / 1000.0f;
    const float vy = data.velocity.vy / 1000.0f;
    const float wz = data.velocity.wz / 1000.0f;
    if (revoked || !fresh || hypotf(vx, vy) > CHASSIS_STRAIGHT_MAX_SPEED_MPS ||
        fabsf(wz) > CHASSIS_VECTOR_MAX_WZ_RAD_S)
    {
        Begin_Braking(now_ms);
        return;
    }
    if (jetson_control_state == JETSON_PREPARING)
    {
        /* 使能握手须保持零速；运行后停车可忽略早到的非零帧，但反馈仍须正常。 */
        if ((uint32_t)(now_ms - state_started_ms) >= JETSON_PREPARE_TIMEOUT_MS ||
            (preparing_after_stop ? !Jetson_Control_Motors_Ready() : !zero))
            Begin_Braking(now_ms);
        else if (Chassis_Is_Stopped())
        {
            if (preparing_after_stop)
            {
                /* 停稳后再取接收边界，不能把停车期间最后一帧留给 READY 执行。 */
                Jetson_CAN_Get_Data(&data);
                blocked_sequence = data.velocity.sequence;
            }
            jetson_control_state = JETSON_READY;
        }
        return;
    }
    if (!Jetson_Control_Motors_Ready())
    {
        Begin_Braking(now_ms);
        return;
    }
    /* 运行中同一批收到全零和非零时，仍执行明确的零速停车。 */
    if (jetson_control_state == JETSON_RUNNING && (zero || stop_requested))
    {
        Chassis_Stop();
        state_started_ms = now_ms;
        preparing_after_stop = 1U;
        jetson_control_state = JETSON_PREPARING;
        return;
    }
    if (zero || (jetson_control_state == JETSON_READY && data.velocity.sequence == blocked_sequence))
        return;
    if (Chassis_Set_Velocity(vx, vy, wz))
        jetson_control_state = JETSON_RUNNING;
    else
        Begin_Braking(now_ms);
}
