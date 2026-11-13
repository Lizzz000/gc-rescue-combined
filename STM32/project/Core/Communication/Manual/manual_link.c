#include "manual_link.h"
#include "jetson_link.h"
#include "FreeRTOS.h"
#include "task.h"

/* 速度帧解码复用 jetson_can；本模块只维护手动快照及其事件序号。 */
static volatile Manual_CAN_Velocity_t manual_velocity;

/** @brief 清空快照；调用时机见头文件。 */
void Manual_CAN_Init(void)
{
    taskENTER_CRITICAL();
    manual_velocity = (Manual_CAN_Velocity_t){0};
    taskEXIT_CRITICAL();
}

/** @brief 解析类型 2 并整体替换快照；输入约束见头文件。 */
void Manual_CAN_Receive(const uint8_t *data, uint8_t len)
{
    Jetson_CAN_Velocity_Frame_t frame;
    Jetson_CAN_Decode_Velocity(data, len, &frame);
    Manual_CAN_Velocity_t velocity = {
        .valid = frame.valid,
        .vx = frame.vx,
        .vy = frame.vy,
        .wz = frame.wz,
        .received_ms = (uint32_t)xTaskGetTickCount(),
        .sequence = manual_velocity.sequence + 1U,
        .invalid_sequence = manual_velocity.invalid_sequence,
        .zero_sequence = manual_velocity.zero_sequence,
    };
    if (!frame.valid)
        ++velocity.invalid_sequence;
    else if (frame.vx == 0 && frame.vy == 0 && frame.wz == 0)
        ++velocity.zero_sequence;
    manual_velocity = velocity;
}

/** @brief 复制快照；调用约束见头文件。 */
void Manual_CAN_Get_Velocity(Manual_CAN_Velocity_t *out)
{
    taskENTER_CRITICAL();
    *out = manual_velocity;
    taskEXIT_CRITICAL();
}
