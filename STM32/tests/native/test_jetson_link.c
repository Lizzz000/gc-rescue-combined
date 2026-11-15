#include <assert.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>
#include "jetson_link.h"
#include "manual_link.h"
#include "task.h"

uint32_t test_tick;
static unsigned critical_depth, critical_entries, critical_exits;
static uint8_t receive_on_enter, receive_on_exit;
static const uint8_t velocity_frame[7] = {1U, 0x0CU, 0xFEU, 0xFAU, 0U, 0x18U, 0xFCU};
static const uint8_t center_frame[8] = {1U, 1U, 2U, 3U, 4U};

/** @brief 模拟临界区入口；可在屏蔽中断之前接收一帧，复制须包含该帧。 */
void test_enter_critical(void)
{
    assert(critical_depth == 0U);
    if (receive_on_enter)
    {
        receive_on_enter = 0U;
        Jetson_CAN_Receive(JETSON_CAN_TARGET_CENTER_ID, center_frame, sizeof(center_frame));
    }
    ++critical_depth;
    ++critical_entries;
}

/** @brief 模拟临界区出口；恢复中断后接收的新帧不得污染已复制快照。 */
void test_exit_critical(void)
{
    assert(critical_depth == 1U);
    --critical_depth;
    ++critical_exits;
    if (receive_on_exit)
    {
        receive_on_exit = 0U;
        Jetson_CAN_Receive(JETSON_CAN_TARGET_CENTER_ID, center_frame, sizeof(center_frame));
    }
}

/** @brief 读取真实协议模块并核对每次快照成对进入/退出临界区。 @return 当前快照。 */
static Jetson_CAN_Data_t Snapshot(void)
{
    Jetson_CAN_Data_t snapshot;
    unsigned before = critical_entries;
    Jetson_CAN_Get_Data(&snapshot);
    assert(critical_entries == before + 1U && critical_entries == critical_exits);
    assert(critical_depth == 0U);
    return snapshot;
}

/** @brief 核对速度帧格式校验与解码：小端负数、全范围原值，无效或畸形整帧清零。 */
static void Check_Velocity_Decode(void)
{
    Jetson_CAN_Velocity_Frame_t frame;
    Jetson_CAN_Decode_Velocity(velocity_frame, sizeof(velocity_frame), &frame);
    assert(frame.valid && frame.vx == -500 && frame.vy == 250 && frame.wz == -1000);
    const uint8_t extremes[7] = {1U, 0U, 0x80U, 0xFFU, 0x7FU, 0xFFU, 0xFFU};
    Jetson_CAN_Decode_Velocity(extremes, sizeof(extremes), &frame);
    assert(frame.vx == INT16_MIN && frame.vy == INT16_MAX && frame.wz == -1);
    const uint8_t invalid[7] = {0U, 1U, 2U, 3U, 4U, 5U, 6U};
    const uint8_t malformed[7] = {2U, 1U, 2U, 3U, 4U, 5U, 6U};
    assert(Jetson_CAN_Is_Velocity_Frame(invalid, sizeof(invalid)));
    assert(!Jetson_CAN_Is_Velocity_Frame(malformed, sizeof(malformed)));
    assert(!Jetson_CAN_Is_Velocity_Frame(NULL, 7U));
    for (uint8_t len = 0U; len <= 8U; ++len)
        assert(Jetson_CAN_Is_Velocity_Frame(velocity_frame, len) == (len == 7U));
    const uint8_t *frames[] = {invalid, malformed, NULL, velocity_frame};
    const uint8_t lengths[] = {7U, 7U, 7U, 6U};
    for (unsigned i = 0U; i < sizeof(frames) / sizeof(frames[0]); ++i)
    {
        Jetson_CAN_Decode_Velocity(velocity_frame, sizeof(velocity_frame), &frame);
        Jetson_CAN_Decode_Velocity(frames[i], lengths[i], &frame);
        assert(!frame.valid && !frame.vx && !frame.vy && !frame.wz);
    }
}

/** @brief 删除的类型 1 与手动类型 2 均不改变自动快照或接收序号。 */
static void Check_Removed_Type(void)
{
    Jetson_CAN_Init();
    Jetson_CAN_Receive(JETSON_CAN_TARGET_CENTER_ID, center_frame, sizeof(center_frame));
    const Jetson_CAN_Data_t before = Snapshot();
    for (uint8_t len = 0U; len <= 8U; ++len)
    {
        Jetson_CAN_Receive(1U, center_frame, len);
        Jetson_CAN_Receive(1U, NULL, len);
        Jetson_CAN_Receive(MANUAL_CAN_VELOCITY_ID, center_frame, len);
    }
    const Jetson_CAN_Data_t after = Snapshot();
    assert(memcmp(&before, &after, sizeof(before)) == 0);
}

/** @brief 既有中心周期帧表示链路活跃；无目标可续期，坏帧与一次性回执不能续期。 */
static void Check_Periodic_Reception(void)
{
    Jetson_CAN_Init();
    assert(!Snapshot().periodic_received);
    const uint8_t invisible[8] = {0U};
    const uint8_t malformed[8] = {2U};
    for (uint16_t type = JETSON_CAN_TARGET_CENTER_ID; type <= JETSON_CAN_SAFETY_CENTER_ID; ++type)
    {
        test_tick = 0U; /* 允许上电第 0 毫秒，不用时间戳代替 seen 标记。 */
        Jetson_CAN_Receive(type, center_frame, sizeof(center_frame));
        Jetson_CAN_Data_t data = Snapshot();
        assert(data.periodic_received && data.periodic_received_ms == 0U);
        test_tick = UINT32_MAX;
        Jetson_CAN_Receive(type, invisible, sizeof(invisible));
        data = Snapshot();
        assert(data.periodic_received && data.periodic_received_ms == UINT32_MAX);
        assert(!(type == JETSON_CAN_TARGET_CENTER_ID ? data.target.valid : data.safety.valid));
        test_tick = 3U;
        Jetson_CAN_Receive(type, invisible, sizeof(invisible));
        assert(Snapshot().periodic_received_ms == 3U);

        test_tick = 42U;
        for (uint8_t len = 0U; len <= 9U; ++len)
        {
            if (len != 8U)
                Jetson_CAN_Receive(type, invisible, len);
            Jetson_CAN_Receive(type, malformed, len);
            Jetson_CAN_Receive(type, NULL, len);
            assert(Snapshot().periodic_received_ms == 3U);
        }
    }
    const uint8_t reply[8] = {1U, 1U, 1U};
    const uint8_t lengths[] = {1U, 3U, 6U, 6U, 6U};
    for (uint16_t type = 5U; type <= 9U; ++type)
        Jetson_CAN_Receive(type, reply, lengths[type - 5U]);
    Jetson_CAN_Receive(1U, reply, 7U);
    Jetson_CAN_Receive(2U, reply, 7U);
    Jetson_CAN_Receive(255U, reply, 8U);
    assert(Snapshot().periodic_received_ms == 3U);

    Jetson_CAN_Invalidate_Periodic();
    assert(!Snapshot().periodic_received && Snapshot().periodic_received_ms == 3U);
    Jetson_CAN_Receive(JETSON_CAN_COMMAND_REPLY_ID, reply, 6U);
    assert(!Snapshot().periodic_received);
    Jetson_CAN_Receive(JETSON_CAN_SAFETY_CENTER_ID, invisible, sizeof(invisible));
    assert(Snapshot().periodic_received && Snapshot().periodic_received_ms == test_tick);
    Jetson_CAN_Init();
    assert(!Snapshot().periodic_received && !Snapshot().periodic_received_ms);
}



/** @brief 逐项验证精确 DLC；错误长度不得沿用上一条有效数据，推进已知类型序号。 */
static void Check_Lengths(void)
{
    const uint16_t ids[] = {JETSON_CAN_TARGET_CENTER_ID, JETSON_CAN_SAFETY_CENTER_ID, JETSON_CAN_ARRIVAL_REPLY_ID, JETSON_CAN_ROI_REPLY_ID, JETSON_CAN_COMMAND_REPLY_ID, JETSON_CAN_TURN_REQUEST_ID, JETSON_CAN_ACTION_RESULT_ID};
    const uint8_t lengths[] = {8U, 8U, 1U, 3U, 6U, 6U, 6U};
    const uint8_t frame[8] = {1U, 1U, 1U, 0U, 0U, 0U, 0U, 0U};
    for (unsigned i = 0U; i < sizeof(ids) / sizeof(ids[0]); ++i)
    {
        for (uint8_t len = 0U; len <= 8U; ++len)
        {
            if (len == lengths[i])
                continue;
            Jetson_CAN_Receive(ids[i], frame, lengths[i]);
            Jetson_CAN_Data_t before = Snapshot();
            Jetson_CAN_Receive(ids[i], frame, len);
            Jetson_CAN_Data_t data = Snapshot();
            switch (ids[i])
            {
            case JETSON_CAN_TARGET_CENTER_ID: assert(!data.target.valid && !data.target.x && !data.target.y); break;
            case JETSON_CAN_SAFETY_CENTER_ID: assert(!data.safety.valid && !data.safety.x && !data.safety.y); break;
            case JETSON_CAN_ARRIVAL_REPLY_ID: assert(!data.arrival_ack.valid && !data.arrival_ack.accepted); break;
            case JETSON_CAN_ROI_REPLY_ID: assert(!data.roi_reply.valid && !data.roi_reply.passed); break;
            case JETSON_CAN_COMMAND_REPLY_ID: assert(!data.command_reply.valid && !data.command_reply.token); break;
            case JETSON_CAN_TURN_REQUEST_ID: assert(!data.turn_to_safety.valid && !data.turn_to_safety.token); break;
            case JETSON_CAN_ACTION_RESULT_ID: assert(!data.action_result.valid); break;
            default: assert(0);
            }
            assert(data.rx_sequence == before.rx_sequence + 1U);
        }
    }
}

/** @brief 核对坐标原值、拒绝回复和 ROI 的四种合法组合；错误布尔不得赋予效力。 */
static void Check_Centers_And_ROI(void)
{
    const uint8_t center[8] = {1U, 0x88U, 0x13U, 0x4CU, 0xE2U};
    const uint8_t invalid_center[8] = {0U, 1U, 2U, 3U, 4U};
    const uint8_t malformed_center[8] = {2U, 1U, 2U, 3U, 4U};
    Jetson_CAN_Receive(JETSON_CAN_TARGET_CENTER_ID, center, sizeof(center));
    Jetson_CAN_Receive(JETSON_CAN_SAFETY_CENTER_ID, center, sizeof(center));
    Jetson_CAN_Data_t data = Snapshot();
    assert(data.target.valid && data.target.x == 5000 && data.target.y == -7604);
    assert(data.safety.valid && data.safety.x == 5000 && data.safety.y == -7604);
    Jetson_CAN_Receive(JETSON_CAN_TARGET_CENTER_ID, invalid_center, sizeof(invalid_center));
    Jetson_CAN_Receive(JETSON_CAN_SAFETY_CENTER_ID, malformed_center, sizeof(malformed_center));
    data = Snapshot();
    assert(!data.target.valid && !data.target.x && !data.target.y);
    assert(!data.safety.valid && !data.safety.x && !data.safety.y);
    for (uint8_t accepted = 0U; accepted <= 2U; ++accepted)
    {
        Jetson_CAN_Receive(JETSON_CAN_ARRIVAL_REPLY_ID, &accepted, 1U);
        data = Snapshot();
        assert(data.arrival_ack.valid == (accepted <= 1U));
        assert(data.arrival_ack.accepted == (accepted == 1U));
    }
    for (uint8_t valid = 0U; valid <= 2U; ++valid)
    {
        for (uint8_t passed = 0U; passed <= 2U; ++passed)
        {
            for (uint8_t destination = 0U; destination <= 3U; ++destination)
            {
                uint8_t roi[3] = {valid, passed, destination};
                uint8_t effective = valid == 1U &&
                    ((passed == 0U && destination == 0U) ||
                     (passed == 1U && (destination == 1U || destination == 2U)));
                Jetson_CAN_Receive(JETSON_CAN_ROI_REPLY_ID, roi, sizeof(roi));
                data = Snapshot();
                assert(data.roi_reply.valid == effective);
                assert(data.roi_reply.passed == (effective ? passed : 0U));
                assert(data.roi_reply.destination == (effective ? destination : 0U));
            }
        }
    }
}

/** @brief 核对 token 全范围、搜索与动作编号非零，以及目的区/命令/布尔的边界。 */
static void Check_Handshake(void)
{
    const uint32_t tokens[] = {0U, 0x89ABCDEFU, UINT32_MAX};
    for (unsigned t = 0U; t < sizeof(tokens) / sizeof(tokens[0]); ++t)
    {
        for (uint8_t command = 0U; command <= 6U; ++command)
        {
            for (uint8_t accepted = 0U; accepted <= 2U; ++accepted)
            {
                uint32_t token = tokens[t];
                uint8_t reply[6] = {accepted, command, (uint8_t)token, (uint8_t)(token >> 8U),
                                    (uint8_t)(token >> 16U), (uint8_t)(token >> 24U)};
                uint8_t valid = accepted <= 1U && command >= 1U && command <= 5U &&
                                ((command != 3U && command != 5U) || token != 0U);
                Jetson_CAN_Receive(JETSON_CAN_COMMAND_REPLY_ID, reply, sizeof(reply));
                Jetson_CAN_Data_t data = Snapshot();
                assert(data.command_reply.valid == valid);
                assert(data.command_reply.accepted == (valid ? accepted : 0U));
                assert(data.command_reply.command == (valid ? command : 0U));
                assert(data.command_reply.token == (valid ? token : 0U));
            }
        }
        for (uint8_t request = 0U; request <= 2U; ++request)
        {
            for (uint8_t destination = 0U; destination <= 3U; ++destination)
            {
                uint32_t token = tokens[t];
                uint8_t turn[6] = {request, destination, (uint8_t)token, (uint8_t)(token >> 8U),
                                   (uint8_t)(token >> 16U), (uint8_t)(token >> 24U)};
                uint8_t valid = request == 1U && (destination == 1U || destination == 2U);
                Jetson_CAN_Receive(JETSON_CAN_TURN_REQUEST_ID, turn, sizeof(turn));
                Jetson_CAN_Data_t data = Snapshot();
                assert(data.turn_to_safety.valid == valid);
                assert(data.turn_to_safety.destination == (valid ? destination : 0U));
                assert(data.turn_to_safety.token == (valid ? token : 0U));
            }
        }
    }
}

/** @brief 验证任务临界区快照隔离、未知 ID 无副作用和重新初始化清零。 */
static void Check_Snapshot(void)
{
    unsigned entries = critical_entries;
    Jetson_CAN_Get_Data(NULL);
    assert(critical_entries == entries);
    receive_on_enter = receive_on_exit = 1U;
    Jetson_CAN_Data_t before = Snapshot();
    assert(before.target.valid && before.target.received_ms == test_tick);
    Jetson_CAN_Data_t after = Snapshot();
    assert(after.rx_sequence == before.rx_sequence + 1U);
    const uint16_t ignored[] = {0U, JETSON_CAN_ARRIVAL_ID, 0x201U, 0x202U, JETSON_CAN_TURN_COMPLETED_ID, 0x7FFU, 0x1180U};
    for (unsigned i = 0U; i < sizeof(ignored) / sizeof(ignored[0]); ++i)
        Jetson_CAN_Receive(ignored[i], NULL, 255U);
    assert(Snapshot().rx_sequence == after.rx_sequence);
    Jetson_CAN_Init();
    after = Snapshot();
    assert(!after.rx_sequence && !after.action_result.valid);
    assert(!after.target.valid && !after.safety.valid && !after.arrival_ack.valid);
    assert(!after.roi_reply.valid && !after.command_reply.valid && !after.turn_to_safety.valid);
}

/** @brief 核对六种上行帧的 DLC、小端 token、类型 21 到点类型和 HAL 所需的尾部缓冲。 */
static void Check_Uplink(void)
{
    const uint16_t tokenless_ids[] = {JETSON_CAN_ARRIVAL_ID};
    const uint16_t token_ids[] = {JETSON_CAN_TURN_COMPLETED_ID, JETSON_CAN_SAFETY_ARRIVED_ID, JETSON_CAN_START_SEARCH_ID, JETSON_CAN_DELIVERY_COMPLETED_ID};
    const uint8_t tokenless[8] = {0x01U};
    const uint8_t token_frame[8] = {0x01U, 0xEFU, 0xCDU, 0xABU, 0x89U};
    const uint8_t max_token_frame[8] = {0x01U, 0xFFU, 0xFFU, 0xFFU, 0xFFU};
    uint8_t data[8];
    for (unsigned i = 0U; i < sizeof(tokenless_ids) / sizeof(tokenless_ids[0]); ++i)
    {
        memset(data, 0xAA, sizeof(data));
        assert(Jetson_CAN_Encode_Uplink(tokenless_ids[i], UINT32_MAX, data) == 1U);
        assert(memcmp(data, tokenless, sizeof(data)) == 0);
    }
    for (unsigned i = 0U; i < sizeof(token_ids) / sizeof(token_ids[0]); ++i)
    {
        memset(data, 0xAA, sizeof(data));
        assert(Jetson_CAN_Encode_Uplink(token_ids[i], 0x89ABCDEFU, data) == 5U);
        assert(memcmp(data, token_frame, sizeof(data)) == 0);
        assert(Jetson_CAN_Encode_Uplink(token_ids[i], UINT32_MAX, data) == 5U);
        assert(memcmp(data, max_token_frame, sizeof(data)) == 0);
        const uint8_t length = Jetson_CAN_Encode_Uplink(token_ids[i], 0U, data);
        if (token_ids[i] == JETSON_CAN_START_SEARCH_ID)
        {
            assert(length == 0U);
            assert(memcmp(data, max_token_frame, sizeof(data)) == 0);
        }
        else
        {
            assert(length == 5U);
            assert(memcmp(data, tokenless, sizeof(data)) == 0);
        }
    }
    const uint16_t rejected_ids[] = {0U, 1U, JETSON_CAN_TURN_REQUEST_ID, 0x201U, 0x202U, 0x207U, 0xFFFFU};
    for (unsigned i = 0U; i < sizeof(rejected_ids) / sizeof(rejected_ids[0]); ++i)
    {
        assert(Jetson_CAN_Encode_Uplink(rejected_ids[i], UINT32_MAX, data) == 0U);
        assert(memcmp(data, tokenless, sizeof(data)) == 0);
    }
    assert(Jetson_CAN_Encode_Uplink(JETSON_CAN_ARRIVAL_ID, 0U, NULL) == 0U);


    /* 类型 21 独立编码，非零编号覆盖全部 32 位，越界输入不改缓冲区。 */
    for (uint8_t kind = 0U; kind <= 3U; ++kind)
    {
        const uint8_t expected[8] = {kind, 0xEFU, 0xCDU, 0xABU, 0x89U};
        assert(Jetson_CAN_Encode_Action_Request(kind, 0x89ABCDEFU, data) == 5U);
        assert(memcmp(data, expected, sizeof(data)) == 0);
        assert(Jetson_CAN_Encode_Action_Request(kind, 0U, data) == 0U);
        assert(Jetson_CAN_Encode_Action_Request(4U, 1U, data) == 0U);
        assert(Jetson_CAN_Encode_Action_Request(kind, 1U, NULL) == 0U);
        assert(Jetson_CAN_Encode_Uplink(JETSON_CAN_POINT_REACHED_ID, kind, data) == 0U);
        assert(memcmp(data, expected, sizeof(data)) == 0);
    }

}

/** @brief 同一毫秒内，全部已知帧（包括坏帧）计数，类型 1 不计数。 */
static void Check_Receive_Sequence(void)
{
    Jetson_CAN_Init();
    test_tick = 42U;
    const uint8_t frame[8] = {1U, 1U, 1U};
    const uint16_t ids[] = {3U, 4U, 5U, 6U, 7U, 8U, 9U};
    const uint8_t lengths[] = {8U, 8U, 1U, 3U, 6U, 6U, 6U};
    for (unsigned i = 0U; i < 7U; ++i)
        Jetson_CAN_Receive(ids[i], frame, lengths[i]);
    Jetson_CAN_Receive(1U, frame, 8U);
    Jetson_CAN_Data_t data = Snapshot();
    assert(data.rx_sequence == 7U);
    assert(data.target.sequence == 1U && data.target.received_ms == test_tick);
    assert(data.safety.sequence == 2U && data.safety.received_ms == test_tick);
    assert(data.arrival_ack.sequence == 3U && data.roi_reply.sequence == 4U);
    assert(data.command_reply.sequence == 5U && data.turn_to_safety.sequence == 6U);
    assert(data.action_result.sequence == 7U && data.action_result.valid);
    Jetson_CAN_Receive(JETSON_CAN_ACTION_RESULT_ID, frame, 5U);
    data = Snapshot();
    assert(data.rx_sequence == 8U && data.action_result.sequence == 8U && !data.action_result.valid);
}

/** @brief 核对握手等待和普通最新帧模式可切换，收到首个匹配成功后保持原序号。 */
static void Check_Command_Expectation(void)
{
    Jetson_CAN_Init();
    test_tick = 100U;
    const uint8_t turn_ok[6] = {1U, 1U, 1U};
    const uint8_t wrong_token[6] = {1U, 1U, 2U};
    const uint8_t delivery_ok[6] = {1U, 4U, 1U};
    const uint8_t search_ok[6] = {1U, 3U, 0xFFU, 0xFFU, 0xFFU, 0xFFU};
    Jetson_CAN_Receive(JETSON_CAN_COMMAND_REPLY_ID, turn_ok, sizeof(turn_ok));
    assert(Snapshot().command_reply.accepted);
    Jetson_CAN_Expect_Command_Reply(JETSON_CAN_TURN_COMPLETED_ID, 1U);
    assert(!Snapshot().command_reply.valid);
    Jetson_CAN_Receive(JETSON_CAN_COMMAND_REPLY_ID, wrong_token, sizeof(wrong_token));
    assert(!Snapshot().command_reply.valid);
    Jetson_CAN_Receive(JETSON_CAN_COMMAND_REPLY_ID, turn_ok, sizeof(turn_ok));
    const uint32_t sequence = Snapshot().command_reply.sequence;
    test_tick = 120U;
    Jetson_CAN_Receive(JETSON_CAN_COMMAND_REPLY_ID, delivery_ok, sizeof(delivery_ok));
    Jetson_CAN_Receive(JETSON_CAN_COMMAND_REPLY_ID, turn_ok, sizeof(turn_ok));
    const Jetson_CAN_Data_t retained = Snapshot();
    assert(retained.rx_sequence == sequence + 2U && retained.command_reply.sequence == sequence);
    assert(retained.command_reply.valid && retained.command_reply.accepted &&
           retained.command_reply.command == 1U && retained.command_reply.token == 1U);

    Jetson_CAN_Expect_Command_Reply(0U, 0U);
    Jetson_CAN_Receive(JETSON_CAN_COMMAND_REPLY_ID, delivery_ok, sizeof(delivery_ok));
    assert(Snapshot().command_reply.command == 4U);
    Jetson_CAN_Expect_Command_Reply(JETSON_CAN_START_SEARCH_ID, UINT32_MAX);
    const uint8_t old_search[6] = {1U, 3U, 1U};
    const uint8_t zero_search[6] = {1U, 3U};
    Jetson_CAN_Receive(JETSON_CAN_COMMAND_REPLY_ID, old_search, sizeof(old_search));
    Jetson_CAN_Receive(JETSON_CAN_COMMAND_REPLY_ID, zero_search, sizeof(zero_search));
    assert(!Snapshot().command_reply.valid);
    Jetson_CAN_Receive(JETSON_CAN_COMMAND_REPLY_ID, search_ok, sizeof(search_ok));
    const uint32_t search_sequence = Snapshot().command_reply.sequence;
    Jetson_CAN_Receive(JETSON_CAN_COMMAND_REPLY_ID, old_search, sizeof(old_search));
    Jetson_CAN_Receive(JETSON_CAN_COMMAND_REPLY_ID, zero_search, sizeof(zero_search));
    assert(Snapshot().command_reply.command == 3U && Snapshot().command_reply.token == UINT32_MAX &&
           Snapshot().command_reply.sequence == search_sequence);

    /* 类型 21 必须回显当前非零动作编号。 */
    const uint8_t point_ok[6] = {1U, 5U, 7U};
    const uint8_t point_token[6] = {1U, 5U, 1U};
    Jetson_CAN_Expect_Command_Reply(JETSON_CAN_POINT_REACHED_ID, 7U);
    assert(!Snapshot().command_reply.valid);
    Jetson_CAN_Receive(JETSON_CAN_COMMAND_REPLY_ID, search_ok, sizeof(search_ok));
    Jetson_CAN_Receive(JETSON_CAN_COMMAND_REPLY_ID, point_token, sizeof(point_token));
    assert(!Snapshot().command_reply.valid);
    Jetson_CAN_Receive(JETSON_CAN_COMMAND_REPLY_ID, point_ok, sizeof(point_ok));
    assert(Snapshot().command_reply.valid && Snapshot().command_reply.command == 5U &&
           Snapshot().command_reply.token == 7U);
    Jetson_CAN_Init();
    Jetson_CAN_Receive(JETSON_CAN_COMMAND_REPLY_ID, delivery_ok, sizeof(delivery_ok));
    assert(Snapshot().command_reply.command == 4U);
}


/** @brief 动作终态校验 kind/id；匹配成功或失败均锁存，不被坏帧和旧动作冲掉。 */
static void Check_Action_Result(void)
{
    Jetson_CAN_Init();
    for (uint8_t kind = 0U; kind <= 4U; ++kind)
        for (uint8_t success = 0U; success <= 2U; ++success)
            for (uint8_t id = 0U; id <= 1U; ++id)
            {
                const uint8_t bytes[6] = {kind, success, id};
                Jetson_CAN_Receive(JETSON_CAN_ACTION_RESULT_ID, bytes, sizeof(bytes));
                assert(Snapshot().action_result.valid == (kind <= 3U && success <= 1U && id != 0U));
            }
    for (uint8_t success = 0U; success <= 1U; ++success)
    {
        Jetson_CAN_Expect_Action_Result(2U, 0x89ABCDEFU);
        const uint8_t wrong[6] = {2U, 1U, 2U};
        const uint8_t other[6] = {1U, 1U, 0xEFU, 0xCDU, 0xABU, 0x89U};
        const uint8_t good[6] = {2U, success, 0xEFU, 0xCDU, 0xABU, 0x89U};
        Jetson_CAN_Receive(9U, wrong, 6U);
        Jetson_CAN_Receive(9U, other, 6U);
        assert(!Snapshot().action_result.valid);
        Jetson_CAN_Receive(9U, good, 6U);
        const Jetson_CAN_Action_Result_t retained = Snapshot().action_result;
        uint8_t contradictory[6];
        memcpy(contradictory, good, sizeof(contradictory));
        contradictory[1] = (uint8_t)!success;
        Jetson_CAN_Receive(9U, contradictory, 6U);
        Jetson_CAN_Receive(9U, wrong, 6U);
        Jetson_CAN_Receive(9U, NULL, 0U);
        const Jetson_CAN_Data_t after = Snapshot();
        assert(after.rx_sequence == retained.sequence + 3U);
        assert(memcmp(&retained, &after.action_result, sizeof(retained)) == 0);
    }
    Jetson_CAN_Expect_Action_Result(0U, 0U);
    const uint8_t newer[6] = {0U, 1U, 9U};
    Jetson_CAN_Receive(9U, newer, 6U);
    assert(Snapshot().action_result.kind == 0U && Snapshot().action_result.request_id == 9U);
}

/** @brief 运行实际 Jetson 编解码模块的离线边界测试。 @return 全部通过返回 0。 */
int main(void)
{
    Check_Velocity_Decode();
    Check_Removed_Type();
    Check_Periodic_Reception();
    Check_Action_Result();
    Check_Lengths();
    Check_Centers_And_ROI();
    Check_Handshake();
    Check_Snapshot();
    Check_Uplink();
    Check_Receive_Sequence();
    Check_Command_Expectation();
    puts("PASS: Jetson CAN protocol");
    return 0;
}
