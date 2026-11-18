#include "manual_link.h"
#include "link_payload.h"
#include "FreeRTOS.h"
#include "task.h"

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
    const Link_Velocity_Frame_t frame = Link_Decode_Velocity(data, len);
    Manual_CAN_Velocity_t velocity = {
        .valid = frame.valid,
        .explicit_release = manual_velocity.explicit_release,
        .vx = frame.vx,
        .vy = frame.vy,
        .wz = frame.wz,
        .received_ms = (uint32_t)xTaskGetTickCount(),
        .sequence = manual_velocity.sequence + 1U,
        .invalid_sequence = manual_velocity.invalid_sequence,
        .zero_sequence = manual_velocity.zero_sequence,
        .disconnect_sequence = manual_velocity.disconnect_sequence,
    };
    if (!frame.valid)
    {
        ++velocity.invalid_sequence;
        velocity.explicit_release = Link_Is_Velocity_Frame(data, len) && data[0] == 0U;
    }
    else if (frame.vx == 0 && frame.vy == 0 && frame.wz == 0)
    {
        ++velocity.zero_sequence;
    }
    manual_velocity = velocity;
}

void Manual_CAN_Disconnect(void)
{
    manual_velocity.valid = 0U;
    ++manual_velocity.disconnect_sequence;
}

/** @brief 复制快照；调用约束见头文件。 */
void Manual_CAN_Get_Velocity(Manual_CAN_Velocity_t *out)
{
    taskENTER_CRITICAL();
    *out = manual_velocity;
    taskEXIT_CRITICAL();
}
