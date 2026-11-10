#include "jetson_can.h"
#include "jetson_auto_config.h"
#include "FreeRTOS.h"
#include "task.h"
#include <stddef.h>
#include <string.h>

/* 来源：Jetson tools/source/can_codec.py；适配小端编解码和 FreeRTOS 快照保护。
 * 接收方不把 0x190/192 的拒绝、0x191 的无效或 0x193 的目的区当速度指令。
 */
static volatile Jetson_CAN_Data_t jetson_data;
static volatile uint8_t expected_reply_command;
static volatile uint32_t expected_reply_token;

/**
 * @brief 逐字节读取小端有符号定点值，不依赖结构体对齐或主机字节序。
 * @param[in] data 至少可读 2 字节。
 * @return int16_t 全范围原始值；缩放由使用方按字段单位处理。
 */
static int16_t Read_Signed(const uint8_t *data)
{
    uint16_t bits = (uint16_t)((uint16_t)data[0] | ((uint16_t)data[1] << 8U));
    return bits <= INT16_MAX ? (int16_t)bits : (int16_t)((int32_t)bits - 65536);
}

/**
 * @brief 读取运输握手的小端 token，允许全零及全部 32 位有效。
 * @param[in] data 至少可读 4 字节。
 * @return 0..UINT32_MAX 的原始 token，不额外施加非零约束。
 */
static uint32_t Read_Token(const uint8_t *data)
{
    return (uint32_t)data[0] | ((uint32_t)data[1] << 8U) |
           ((uint32_t)data[2] << 16U) | ((uint32_t)data[3] << 24U);
}

/** @brief 初始化接收快照；调用时机和返回值见头文件。 */
void Jetson_CAN_Init(void)
{
    taskENTER_CRITICAL();
    jetson_data = (Jetson_CAN_Data_t){0};
    expected_reply_command = 0U;
    expected_reply_token = 0U;
    taskEXIT_CRITICAL();
}

/**
 * @brief 解析 Jetson 下行标准数据帧；详细输入约束见头文件。
 * @note ISR 只更新最新状态；现有 FreeRTOS tick 为 1 ms，不进行业务调度。
 * @return 无；无效速度立即覆盖旧值，其他报文互不改变。
 */
void Jetson_CAN_Receive(uint16_t id, const uint8_t *data, uint8_t len)
{
#if JETSON_AUTO_ENABLED || JETSON_AUTO_CHASSIS_TEST
    /* 独立手动帧不参与自动搬运状态、心跳或接收序号。 */
    if (id == JETSON_CAN_MANUAL_ID)
        return;
#endif
    uint32_t now_ms = (uint32_t)xTaskGetTickCountFromISR();
    switch (id)
    {
    case JETSON_CAN_MANUAL_ID:
    case 0x180U:
    {
        Jetson_CAN_Velocity_t velocity = {0};
        ++jetson_data.rx_sequence;
        velocity.received_ms = now_ms;
        velocity.sequence = jetson_data.velocity.sequence + 1U;
        velocity.invalid_sequence = jetson_data.velocity.invalid_sequence;
        velocity.zero_sequence = jetson_data.velocity.zero_sequence;
        velocity.well_formed = data != NULL && len == 7U && data[0] <= 1U;
        if (id == 0x180U && velocity.well_formed)
        {
            jetson_data.heartbeat_received_ms = now_ms;
            jetson_data.heartbeat_seen = 1U;
        }
        else if (id == 0x180U && len == 0U)
        {
            /* Jetson 以空 0x180 明确报告任务失联，不能等待普通心跳超时。 */
            jetson_data.heartbeat_seen = 0U;
            ++jetson_data.mission_loss_sequence;
        }
        if (velocity.well_formed && data[0] == 1U)
        {
            velocity.valid = 1U;
            velocity.vx = Read_Signed(data + 1U);
            velocity.vy = Read_Signed(data + 3U);
            velocity.wz = Read_Signed(data + 5U);
            if (velocity.vx == 0 && velocity.vy == 0 && velocity.wz == 0)
                ++velocity.zero_sequence;
        }
        else
            ++velocity.invalid_sequence;
        /* valid=0、错误布尔、错误 DLC 或空指针均不保留上条速度。 */
        jetson_data.velocity = velocity;
        break;
    }
    case 0x181U:
    case 0x182U:
    {
        Jetson_CAN_Center_t center = {0};
        center.received_ms = now_ms;
        center.sequence = ++jetson_data.rx_sequence;
        if (data != NULL && len == 8U && data[0] == 1U)
        {
            center.valid = 1U;
            center.x = Read_Signed(data + 1U);
            center.y = Read_Signed(data + 3U);
            center.source_frame = (uint32_t)data[5] | ((uint32_t)data[6] << 8U) |
                                  ((uint32_t)data[7] << 16U);
        }
        if (id == 0x181U)
            jetson_data.target = center;
        else
            jetson_data.safety = center;
        break;
    }
    case 0x190U:
    {
        Jetson_CAN_Arrival_Ack_t reply = {0};
        reply.received_ms = now_ms;
        reply.sequence = ++jetson_data.rx_sequence;
        if (data != NULL && len == 1U && data[0] <= 1U)
        {
            reply.valid = 1U;
            reply.accepted = data[0];
        }
        jetson_data.arrival_ack = reply;
        break;
    }
    case 0x191U:
    {
        Jetson_CAN_ROI_Reply_t reply = {0};
        reply.received_ms = now_ms;
        reply.sequence = ++jetson_data.rx_sequence;
        /* 仅四种合法组合：000、100、111、112；000 保持无效。 */
        if (data != NULL && len == 3U && data[0] == 1U &&
            ((data[1] == 0U && data[2] == 0U) ||
             (data[1] == 1U && (data[2] == 1U || data[2] == 2U))))
        {
            reply.valid = 1U;
            reply.passed = data[1];
            reply.destination = data[2];
        }
        jetson_data.roi_reply = reply;
        break;
    }
    case 0x192U:
    {
        Jetson_CAN_Command_Reply_t reply = {0};
        reply.received_ms = now_ms;
        reply.sequence = ++jetson_data.rx_sequence;
        if (data != NULL && len == 6U && data[0] <= 1U &&
            data[1] >= 1U && data[1] <= 4U)
        {
            uint32_t token = Read_Token(data + 2U);
            if (data[1] != 3U || token == 0U)
            {
                reply.valid = 1U;
                reply.accepted = data[0];
                reply.command = data[1];
                reply.token = token;
            }
        }
        /* 等待当前握手时保留首个匹配的受理帧及到达时间，迟到重复不能覆盖它。 */
        if (expected_reply_command == 0U ||
            (!jetson_data.command_reply.accepted && reply.valid &&
             reply.command == expected_reply_command && reply.token == expected_reply_token))
            jetson_data.command_reply = reply;
        break;
    }
    case 0x193U:
    {
        Jetson_CAN_Turn_Request_t request = {0};
        request.received_ms = now_ms;
        request.sequence = ++jetson_data.rx_sequence;
        if (data != NULL && len == 6U && data[0] == 1U &&
            (data[1] == 1U || data[1] == 2U))
        {
            request.valid = 1U;
            request.destination = data[1];
            request.token = Read_Token(data + 2U);
        }
        jetson_data.turn_to_safety = request;
        break;
    }
    default:
        break;
    }
}

/** @brief 切换命令回执的接收匹配，参数和行为见头文件。 */
void Jetson_CAN_Expect_Command_Reply(uint16_t request_id, uint32_t token)
{
    const uint8_t command = request_id >= JETSON_CAN_TURN_COMPLETED_ID &&
                            request_id <= JETSON_CAN_DELIVERY_COMPLETED_ID ?
        (uint8_t)(request_id - JETSON_CAN_TURN_COMPLETED_ID + 1U) : 0U;
    taskENTER_CRITICAL();
    expected_reply_command = command;
    expected_reply_token = request_id == JETSON_CAN_START_SEARCH_ID ? 0U : token;
    if (command != 0U)
        jetson_data.command_reply = (Jetson_CAN_Command_Reply_t){0};
    taskEXIT_CRITICAL();
}

/** @brief 复制完整快照；任务调用约束与返回值见头文件。 */
void Jetson_CAN_Get_Data(Jetson_CAN_Data_t *out)
{
    if (out == NULL)
        return;
    taskENTER_CRITICAL();
    *out = jetson_data;
    taskEXIT_CRITICAL();
}

/** @brief 编码上行载荷；帧 ID 与字节数沿用 Jetson 协议。 @return CAN DLC，失败为 0。 */
uint8_t Jetson_CAN_Encode_Uplink(uint16_t id, uint32_t token, uint8_t data[8])
{
    if (data == NULL)
        return 0U;

    uint8_t length;
    switch (id)
    {
    case JETSON_CAN_ARRIVAL_ID:
    case JETSON_CAN_START_SEARCH_ID:
        length = 1U;
        break;
    case JETSON_CAN_TURN_COMPLETED_ID:
    case JETSON_CAN_SAFETY_ARRIVED_ID:
    case JETSON_CAN_DELIVERY_COMPLETED_ID:
        length = 5U;
        break;
    default:
        return 0U;
    }

    memset(data, 0, 8U);
    data[0] = 1U;
    if (length == 5U)
    {
        data[1] = (uint8_t)token;
        data[2] = (uint8_t)(token >> 8U);
        data[3] = (uint8_t)(token >> 16U);
        data[4] = (uint8_t)(token >> 24U);
    }
    return length;
}
