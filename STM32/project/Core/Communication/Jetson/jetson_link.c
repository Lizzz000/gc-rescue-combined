#include "jetson_link.h"
#include "FreeRTOS.h"
#include "task.h"
#include <stddef.h>
#include <string.h>

/* 小端编解码和 FreeRTOS 快照保护。
 * 接收方不把类型 5/7 的拒绝、类型 6 的无效或类型 8 的目的区当速度指令。
 */
static volatile Jetson_CAN_Data_t jetson_data;
static volatile uint8_t expected_reply_command;
static volatile uint32_t expected_reply_token;
static volatile uint8_t expected_action_kind;
static volatile uint32_t expected_action_id;

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

/** @brief 校验速度帧格式；约束见头文件。 */
uint8_t Jetson_CAN_Is_Velocity_Frame(const uint8_t *data, uint8_t len)
{
    return Link_Is_Velocity_Frame(data, len);
}

/** @brief 兼容已有速度解码调用。 */
void Jetson_CAN_Decode_Velocity(const uint8_t *data, uint8_t len, Jetson_CAN_Velocity_Frame_t *frame)
{
    *frame = Link_Decode_Velocity(data, len);
}

/** @brief 初始化接收快照；调用时机和返回值见头文件。 */
void Jetson_CAN_Init(void)
{
    taskENTER_CRITICAL();
    jetson_data = (Jetson_CAN_Data_t){0};
    expected_reply_command = 0U;
    expected_reply_token = 0U;
    expected_action_kind = 0U;
    expected_action_id = 0U;
    taskEXIT_CRITICAL();
}

/** @brief 撤销旧 USB 会话的周期流准入标记；调用约束见头文件。 @return 无。 */
void Jetson_CAN_Invalidate_Periodic(void)
{
    jetson_data.periodic_received = 0U;
}

/**
 * @brief 解析 Jetson 下行类型；详细输入约束见头文件。
 * @note 只更新最新状态；现有 FreeRTOS tick 为 1 ms，不进行业务调度。
 * @return 无；各报文只更新各自快照，互不改变。
 */
void Jetson_CAN_Receive(uint16_t type, const uint8_t *data, uint8_t len)
{
    Jetson_CAN_Receive_At(type, data, len, (uint32_t)xTaskGetTickCount());
}

void Jetson_CAN_Receive_At(uint16_t type, const uint8_t *data, uint8_t len, uint32_t received_ms)
{
    const uint32_t now_ms = received_ms;
    switch (type)
    {
    case JETSON_CAN_TARGET_CENTER_ID:
    case JETSON_CAN_SAFETY_CENTER_ID:
    {
        Jetson_CAN_Center_t center = {0};
        center.received_ms = now_ms;
        center.sequence = ++jetson_data.rx_sequence;
        /* 无目标仍会周期发布 valid=0；只有合法中心帧证明周期流还在运行。 */
        if (data != NULL && len == 8U && data[0] <= 1U)
        {
            jetson_data.periodic_received = 1U;
            jetson_data.periodic_received_ms = now_ms;
        }
        if (data != NULL && len == 8U && data[0] == 1U)
        {
            center.valid = 1U;
            center.x = Link_Read_I16(data + 1U);
            center.y = Link_Read_I16(data + 3U);
            center.source_frame = (uint32_t)data[5] | ((uint32_t)data[6] << 8U) |
                                  ((uint32_t)data[7] << 16U);
        }
        if (type == JETSON_CAN_TARGET_CENTER_ID)
        {
            jetson_data.target = center;
        }
        else
        {
            jetson_data.safety = center;
        }
        break;
    }
    case JETSON_CAN_ARRIVAL_REPLY_ID:
    {
        Jetson_CAN_Arrival_Ack_t reply = {0};
        reply.sequence = ++jetson_data.rx_sequence;
        if (data != NULL && len == 1U && data[0] <= 1U)
        {
            reply.valid = 1U;
            reply.accepted = data[0];
        }
        jetson_data.arrival_ack = reply;
        break;
    }
    case JETSON_CAN_ROI_REPLY_ID:
    {
        Jetson_CAN_ROI_Reply_t reply = {0};
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
    case JETSON_CAN_COMMAND_REPLY_ID:
    {
        Jetson_CAN_Command_Reply_t reply = {0};
        reply.sequence = ++jetson_data.rx_sequence;
        if (data != NULL && len == 6U && data[0] <= 1U &&
            data[1] >= 1U && data[1] <= JETSON_CAN_START_ALIGN_COMMAND)
        {
            uint32_t token = Read_Token(data + 2U);
            const uint8_t needs_token = data[1] == 3U || data[1] == JETSON_CAN_POINT_REPLY_COMMAND ||
                                        data[1] == JETSON_CAN_START_ALIGN_COMMAND;
            if (!needs_token || token != 0U)
            {
                reply.valid = 1U;
                reply.accepted = data[0];
                reply.command = data[1];
                reply.token = token;
            }
        }
        /* 等待当前握手时保留首个匹配的受理帧及接收序号，迟到重复不能覆盖它。 */
        if (expected_reply_command == 0U ||
            (!jetson_data.command_reply.accepted && reply.valid &&
             reply.command == expected_reply_command && reply.token == expected_reply_token))
        {
            jetson_data.command_reply = reply;
        }
        break;
    }
    case JETSON_CAN_TURN_REQUEST_ID:
    {
        Jetson_CAN_Turn_Request_t request = {0};
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
    case JETSON_CAN_ACTION_RESULT_ID:
    {
        Jetson_CAN_Action_Result_t result = {0};
        result.sequence = ++jetson_data.rx_sequence;
        if (data != NULL && len == 6U && data[0] <= JETSON_CAN_POINT_REJECT_OPEN &&
            data[1] <= 1U && Read_Token(data + 2U) != 0U)
        {
            result.valid = 1U;
            result.kind = data[0];
            result.success = data[1];
            result.request_id = Read_Token(data + 2U);
        }
        /* 两种终态都锁存，后来的坏帧、旧编号或其他动作不能覆盖。 */
        if (expected_action_id == 0U ||
            (!jetson_data.action_result.valid && result.valid &&
             result.kind == expected_action_kind && result.request_id == expected_action_id))
        {
            jetson_data.action_result = result;
        }
        break;
    }
    default:
        break;
    }
}

/** @brief 切换命令回执的接收匹配，参数和行为见头文件。 */
void Jetson_CAN_Expect_Command_Reply(uint16_t request_type, uint32_t token)
{
    uint8_t command = 0U;
    if (request_type == JETSON_CAN_POINT_REACHED_ID)
    {
        command = JETSON_CAN_POINT_REPLY_COMMAND;
    }
    else if (request_type == JETSON_CAN_START_ALIGN_ID)
    {
        command = JETSON_CAN_START_ALIGN_COMMAND;
    }
    else if (request_type >= JETSON_CAN_TURN_COMPLETED_ID && request_type <= JETSON_CAN_DELIVERY_COMPLETED_ID)
    {
        command = (uint8_t)(request_type - JETSON_CAN_TURN_COMPLETED_ID + 1U);
    }
    taskENTER_CRITICAL();
    expected_reply_command = command;
    expected_reply_token = token;
    if (command != 0U)
    {
        jetson_data.command_reply = (Jetson_CAN_Command_Reply_t){0};
    }
    taskEXIT_CRITICAL();
}

void Jetson_CAN_Expect_Action_Result(uint8_t kind, uint32_t request_id)
{
    taskENTER_CRITICAL();
    expected_action_kind = kind;
    expected_action_id = request_id;
    if (request_id != 0U)
    {
        jetson_data.action_result = (Jetson_CAN_Action_Result_t){0};
    }
    taskEXIT_CRITICAL();
}

/** @brief 复制完整快照；任务调用约束与返回值见头文件。 */
void Jetson_CAN_Get_Data(Jetson_CAN_Data_t *out)
{
    if (out == NULL)
    {
        return;
    }
    taskENTER_CRITICAL();
    *out = jetson_data;
    taskEXIT_CRITICAL();
}

/** @brief 编码上行载荷。 @return 载荷长度，失败为 0。 */
uint8_t Jetson_CAN_Encode_Uplink(uint16_t type, uint32_t argument, uint8_t data[8])
{
    if (data == NULL ||
        ((type == JETSON_CAN_START_SEARCH_ID || type == JETSON_CAN_START_ALIGN_ID) && argument == 0U))
    {
        return 0U;
    }

    uint8_t length;
    switch (type)
    {
    case JETSON_CAN_ARRIVAL_ID:
        length = 1U;
        break;
    case JETSON_CAN_TURN_COMPLETED_ID:
    case JETSON_CAN_START_SEARCH_ID:
    case JETSON_CAN_START_ALIGN_ID:
    case JETSON_CAN_SAFETY_ARRIVED_ID:
    case JETSON_CAN_DELIVERY_COMPLETED_ID:
        length = 5U;
        break;
    default:
        return 0U;
    }

    memset(data, 0, 8U);
    data[0] = type == JETSON_CAN_ARRIVAL_ID && argument == JETSON_CAN_SELECTED_ROUTE ?
              JETSON_CAN_SELECTED_ROUTE : 1U;
    if (length == 5U)
    {
        data[1] = (uint8_t)argument;
        data[2] = (uint8_t)(argument >> 8U);
        data[3] = (uint8_t)(argument >> 16U);
        data[4] = (uint8_t)(argument >> 24U);
    }
    return length;
}

uint8_t Jetson_CAN_Encode_Action_Request(uint8_t kind, uint32_t request_id, uint32_t source_frame, uint8_t data[8])
{
    if (data == NULL || kind > JETSON_CAN_POINT_REJECT_OPEN || request_id == 0U ||
        source_frame > 0xffffffU || (kind != JETSON_CAN_POINT_GRASP && source_frame != 0U))
    {
        return 0U;
    }
    memset(data, 0, 8U);
    data[0] = kind;
    for (unsigned byte = 0U; byte < 4U; ++byte)
    {
        data[byte + 1U] = (uint8_t)(request_id >> (byte * 8U));
    }
    if (kind == JETSON_CAN_POINT_GRASP)
    {
        data[5] = (uint8_t)source_frame;
        data[6] = (uint8_t)(source_frame >> 8U);
        data[7] = (uint8_t)(source_frame >> 16U);
        return 8U;
    }
    return 5U;
}
