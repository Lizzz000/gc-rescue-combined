#include <assert.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>
#include "jetson_can.h"
#include "task.h"

uint32_t test_tick;
static unsigned critical_depth, critical_entries, critical_exits;
static uint8_t receive_on_enter, receive_on_exit;
static const uint8_t velocity_frame[7] = {1U, 0x0CU, 0xFEU, 0xFAU, 0U, 0x18U, 0xFCU};
static const uint8_t zero_frame[7] = {1U, 0U, 0U, 0U, 0U, 0U, 0U};

/** @brief 模拟临界区入口；可在屏蔽中断之前接收一帧，复制须包含该帧。 */
void test_enter_critical(void)
{
    assert(critical_depth == 0U);
    if (receive_on_enter)
    {
        receive_on_enter = 0U;
        Jetson_CAN_Receive(0x180U, velocity_frame, sizeof(velocity_frame));
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
        Jetson_CAN_Receive(0x180U, zero_frame, sizeof(zero_frame));
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

/** @brief 核对小端负数、全范围定点原值及无效帧即时覆盖旧指令。 */
static void Check_Velocity(void)
{
    Jetson_CAN_Init();
    Jetson_CAN_Data_t data = Snapshot();
    assert(!data.velocity.valid && data.velocity.sequence == 0U &&
           !data.velocity.invalid_sequence && !data.velocity.zero_sequence);
    test_tick = 123U;
    Jetson_CAN_Receive(0x180U, velocity_frame, sizeof(velocity_frame));
    data = Snapshot();
    assert(data.velocity.valid && data.velocity.vx == -500 && data.velocity.vy == 250);
    assert(data.velocity.wz == -1000 && data.velocity.received_ms == 123U);
    assert(data.velocity.sequence == 1U && !data.velocity.invalid_sequence);
    const uint8_t extremes[7] = {1U, 0U, 0x80U, 0xFFU, 0x7FU, 0xFFU, 0xFFU};
    Jetson_CAN_Receive(0x180U, extremes, sizeof(extremes));
    data = Snapshot();
    assert(data.velocity.vx == INT16_MIN && data.velocity.vy == INT16_MAX);
    assert(data.velocity.wz == -1 && data.velocity.sequence == 2U);
    for (uint8_t valid = 0U; valid <= 2U; valid += 2U)
    {
        uint8_t invalid[7] = {valid, 1U, 2U, 3U, 4U, 5U, 6U};
        uint32_t sequence = data.velocity.sequence;
        uint32_t invalid_sequence = data.velocity.invalid_sequence;
        ++test_tick;
        Jetson_CAN_Receive(0x180U, invalid, sizeof(invalid));
        data = Snapshot();
        assert(!data.velocity.valid && !data.velocity.vx && !data.velocity.vy && !data.velocity.wz);
        assert(data.velocity.sequence == sequence + 1U && data.velocity.received_ms == test_tick);
        assert(data.velocity.invalid_sequence == invalid_sequence + 1U);
        assert(data.velocity.zero_sequence == 0U);
    }
    Jetson_CAN_Receive(0x180U, zero_frame, sizeof(zero_frame));
    data = Snapshot();
    assert(data.velocity.valid && !data.velocity.vx && !data.velocity.vy && !data.velocity.wz);
    assert(data.velocity.zero_sequence == 1U);
    uint32_t sequence = data.velocity.sequence;
    test_tick = UINT32_MAX;
    Jetson_CAN_Receive(0x180U, NULL, 7U);
    data = Snapshot();
    assert(!data.velocity.valid && data.velocity.sequence == sequence + 1U);
    assert(data.velocity.received_ms == UINT32_MAX && data.velocity.zero_sequence == 1U);
    test_tick = 0U;
    Jetson_CAN_Receive(0x180U, velocity_frame, sizeof(velocity_frame));
    assert(Snapshot().velocity.received_ms == 0U);
    /* 两次接收之间不运行任务：末条有效速度不能抹去前条撤权。 */
    uint32_t invalid_sequence = Snapshot().velocity.invalid_sequence;
    Jetson_CAN_Receive(0x180U, NULL, 0U);
    Jetson_CAN_Receive(0x180U, velocity_frame, sizeof(velocity_frame));
    data = Snapshot();
    assert(data.velocity.valid && data.velocity.vx == -500);
    assert(data.velocity.invalid_sequence == invalid_sequence + 1U && data.velocity.zero_sequence == 1U);
    Jetson_CAN_Receive(0x180U, zero_frame, sizeof(zero_frame));
    data = Snapshot();
    assert(data.velocity.invalid_sequence == invalid_sequence + 1U && data.velocity.zero_sequence == 2U);
}

/** @brief 逐项验证精确 DLC；任何错误长度均不得沿用上一条有效数据。 */
static void Check_Lengths(void)
{
    const uint16_t ids[] = {0x180U, 0x181U, 0x182U, 0x190U, 0x191U, 0x192U, 0x193U};
    const uint8_t lengths[] = {7U, 8U, 8U, 1U, 3U, 6U, 6U};
    const uint8_t frame[8] = {1U, 1U, 1U, 0U, 0U, 0U, 0U, 0U};
    for (unsigned i = 0U; i < sizeof(ids) / sizeof(ids[0]); ++i)
    {
        if (ids[i] != 0x180U)
            Jetson_CAN_Receive(0x180U, velocity_frame, sizeof(velocity_frame));
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
            case 0x180U:
                assert(!data.velocity.valid && !data.velocity.vx);
                assert(data.velocity.sequence == before.velocity.sequence + 1U);
                break;
            case 0x181U: assert(!data.target.valid && !data.target.x && !data.target.y); break;
            case 0x182U: assert(!data.safety.valid && !data.safety.x && !data.safety.y); break;
            case 0x190U: assert(!data.arrival_ack.valid && !data.arrival_ack.accepted); break;
            case 0x191U: assert(!data.roi_reply.valid && !data.roi_reply.passed); break;
            case 0x192U: assert(!data.command_reply.valid && !data.command_reply.token); break;
            case 0x193U: assert(!data.turn_to_safety.valid && !data.turn_to_safety.token); break;
            default: assert(0);
            }
            if (ids[i] != 0x180U)
            {
                assert(data.velocity.valid == before.velocity.valid);
                assert(data.velocity.vx == -500 && data.velocity.vy == 250 && data.velocity.wz == -1000);
                assert(data.velocity.sequence == before.velocity.sequence);
                assert(data.velocity.invalid_sequence == before.velocity.invalid_sequence);
                assert(data.velocity.received_ms == before.velocity.received_ms);
            }
        }
    }
}

/** @brief 核对坐标原值、拒绝回复和 ROI 的四种合法组合；错误布尔不得赋予效力。 */
static void Check_Centers_And_ROI(void)
{
    const uint8_t center[8] = {1U, 0x88U, 0x13U, 0x4CU, 0xE2U};
    const uint8_t invalid_center[8] = {0U, 1U, 2U, 3U, 4U};
    const uint8_t malformed_center[8] = {2U, 1U, 2U, 3U, 4U};
    Jetson_CAN_Receive(0x181U, center, sizeof(center));
    Jetson_CAN_Receive(0x182U, center, sizeof(center));
    Jetson_CAN_Data_t data = Snapshot();
    assert(data.target.valid && data.target.x == 5000 && data.target.y == -7604);
    assert(data.safety.valid && data.safety.x == 5000 && data.safety.y == -7604);
    Jetson_CAN_Receive(0x181U, invalid_center, sizeof(invalid_center));
    Jetson_CAN_Receive(0x182U, malformed_center, sizeof(malformed_center));
    data = Snapshot();
    assert(!data.target.valid && !data.target.x && !data.target.y);
    assert(!data.safety.valid && !data.safety.x && !data.safety.y);
    for (uint8_t accepted = 0U; accepted <= 2U; ++accepted)
    {
        Jetson_CAN_Receive(0x190U, &accepted, 1U);
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
                Jetson_CAN_Receive(0x191U, roi, sizeof(roi));
                data = Snapshot();
                assert(data.roi_reply.valid == effective);
                assert(data.roi_reply.passed == (effective ? passed : 0U));
                assert(data.roi_reply.destination == (effective ? destination : 0U));
            }
        }
    }
}

/** @brief 核对 token 全范围、搜索 token 固定零，以及目的区/命令/布尔的边界。 */
static void Check_Handshake(void)
{
    const uint32_t tokens[] = {0U, 0x89ABCDEFU, UINT32_MAX};
    for (unsigned t = 0U; t < sizeof(tokens) / sizeof(tokens[0]); ++t)
    {
        for (uint8_t command = 0U; command <= 5U; ++command)
        {
            for (uint8_t accepted = 0U; accepted <= 2U; ++accepted)
            {
                uint32_t token = tokens[t];
                uint8_t reply[6] = {accepted, command, (uint8_t)token, (uint8_t)(token >> 8U),
                                    (uint8_t)(token >> 16U), (uint8_t)(token >> 24U)};
                uint8_t valid = accepted <= 1U && command >= 1U && command <= 4U &&
                                (command != 3U || token == 0U);
                Jetson_CAN_Receive(0x192U, reply, sizeof(reply));
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
                Jetson_CAN_Receive(0x193U, turn, sizeof(turn));
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
    assert(before.velocity.valid && before.velocity.vx == -500 && before.velocity.wz == -1000);
    Jetson_CAN_Data_t after = Snapshot();
    assert(after.velocity.valid && !after.velocity.vx && !after.velocity.wz);
    assert(after.velocity.sequence == before.velocity.sequence + 1U);
    const uint16_t ignored[] = {0U, 0x200U, 0x201U, 0x202U, 0x203U, 0x7FFU, 0x1180U};
    for (unsigned i = 0U; i < sizeof(ignored) / sizeof(ignored[0]); ++i)
        Jetson_CAN_Receive(ignored[i], NULL, 255U);
    assert(Snapshot().velocity.sequence == after.velocity.sequence);
    Jetson_CAN_Init();
    after = Snapshot();
    assert(!after.velocity.valid && !after.velocity.sequence && !after.velocity.received_ms);
    assert(!after.target.valid && !after.safety.valid && !after.arrival_ack.valid);
    assert(!after.roi_reply.valid && !after.command_reply.valid && !after.turn_to_safety.valid);
}

/** @brief 核对五种上行帧的 DLC、小端 token 和 HAL 所需的尾部缓冲。 */
static void Check_Uplink(void)
{
    const uint16_t tokenless_ids[] = {0x200U, 0x205U};
    const uint16_t token_ids[] = {0x203U, 0x204U, 0x206U};
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
        assert(Jetson_CAN_Encode_Uplink(token_ids[i], 0U, data) == 5U);
        assert(memcmp(data, tokenless, sizeof(data)) == 0);
    }
    const uint16_t rejected_ids[] = {0U, 0x180U, 0x193U, 0x201U, 0x202U, 0x207U, 0xFFFFU};
    for (unsigned i = 0U; i < sizeof(rejected_ids) / sizeof(rejected_ids[0]); ++i)
    {
        assert(Jetson_CAN_Encode_Uplink(rejected_ids[i], UINT32_MAX, data) == 0U);
        assert(memcmp(data, tokenless, sizeof(data)) == 0);
    }
    assert(Jetson_CAN_Encode_Uplink(0x200U, 0U, NULL) == 0U);
}

/** @brief 同一毫秒内区分速度心跳、已知帧到达顺序与未知帧。 */
static void Check_Receive_Sequence(void)
{
    Jetson_CAN_Init();
    test_tick = 42U;
    Jetson_CAN_Receive(0x180U, zero_frame, sizeof(zero_frame));
    Jetson_CAN_Data_t data = Snapshot();
    assert(data.rx_sequence == 1U && data.velocity.sequence == 1U);
    assert(data.velocity.well_formed && data.velocity.valid);
    assert(data.heartbeat_seen && data.heartbeat_received_ms == 42U);

    const uint8_t invalid[7] = {0U};
    ++test_tick;
    Jetson_CAN_Receive(0x180U, invalid, sizeof(invalid));
    data = Snapshot();
    assert(data.rx_sequence == 2U && data.velocity.sequence == 2U);
    assert(data.velocity.well_formed && !data.velocity.valid && data.velocity.invalid_sequence == 1U);
    assert(data.heartbeat_seen && data.heartbeat_received_ms == 43U);
    const uint8_t malformed[7] = {2U};
    ++test_tick;
    Jetson_CAN_Receive(0x180U, malformed, sizeof(malformed));
    data = Snapshot();
    assert(data.rx_sequence == 3U && data.velocity.sequence == 3U);
    assert(!data.velocity.well_formed && !data.velocity.valid && data.velocity.invalid_sequence == 2U);
    assert(data.heartbeat_received_ms == 43U);
    Jetson_CAN_Receive(0x180U, zero_frame, sizeof(zero_frame) - 1U);
    data = Snapshot();
    assert(data.rx_sequence == 4U && data.velocity.sequence == 4U);
    assert(!data.velocity.well_formed);

    const uint8_t frame[8] = {1U, 1U, 1U};
    Jetson_CAN_Receive(0x181U, frame, 8U);
    Jetson_CAN_Receive(0x182U, frame, 8U);
    Jetson_CAN_Receive(0x190U, frame, 1U);
    Jetson_CAN_Receive(0x191U, frame, 3U);
    Jetson_CAN_Receive(0x192U, frame, 6U);
    Jetson_CAN_Receive(0x193U, frame, 6U);
    Jetson_CAN_Receive(0x194U, frame, 6U);
    data = Snapshot();
    assert(data.rx_sequence == 10U && data.velocity.sequence == 4U);
    assert(data.target.sequence == 5U && data.target.received_ms == test_tick);
    assert(data.safety.sequence == 6U && data.safety.received_ms == test_tick);
    assert(data.arrival_ack.sequence == 7U && data.arrival_ack.received_ms == test_tick);
    assert(data.roi_reply.sequence == 8U && data.roi_reply.received_ms == test_tick);
    assert(data.command_reply.sequence == 9U && data.command_reply.received_ms == test_tick);
    assert(data.turn_to_safety.sequence == 10U && data.turn_to_safety.received_ms == test_tick);
    Jetson_CAN_Receive(0x193U, frame, 5U);
    data = Snapshot();
    assert(data.rx_sequence == 11U && data.turn_to_safety.sequence == 11U && !data.turn_to_safety.valid);
}

/** @brief 核对握手等待和普通最新帧模式可切换，收到首个匹配成功后保持原序号。 */
static void Check_Command_Expectation(void)
{
    Jetson_CAN_Init();
    const uint8_t turn_ok[6] = {1U, 1U, 1U};
    const uint8_t wrong_token[6] = {1U, 1U, 2U};
    const uint8_t delivery_ok[6] = {1U, 4U, 1U};
    const uint8_t search_ok[6] = {1U, 3U};
    Jetson_CAN_Receive(0x192U, turn_ok, sizeof(turn_ok));
    assert(Snapshot().command_reply.accepted);
    Jetson_CAN_Expect_Command_Reply(0x203U, 1U);
    assert(!Snapshot().command_reply.valid);
    Jetson_CAN_Receive(0x192U, wrong_token, sizeof(wrong_token));
    assert(!Snapshot().command_reply.valid);
    Jetson_CAN_Receive(0x192U, turn_ok, sizeof(turn_ok));
    const uint32_t sequence = Snapshot().command_reply.sequence;
    Jetson_CAN_Receive(0x192U, delivery_ok, sizeof(delivery_ok));
    Jetson_CAN_Receive(0x192U, turn_ok, sizeof(turn_ok));
    assert(Snapshot().command_reply.sequence == sequence);

    Jetson_CAN_Expect_Command_Reply(0U, 0U);
    Jetson_CAN_Receive(0x192U, delivery_ok, sizeof(delivery_ok));
    assert(Snapshot().command_reply.command == 4U);
    Jetson_CAN_Expect_Command_Reply(0x205U, UINT32_MAX);
    Jetson_CAN_Receive(0x192U, search_ok, sizeof(search_ok));
    assert(Snapshot().command_reply.command == 3U);
    Jetson_CAN_Init();
    Jetson_CAN_Receive(0x192U, delivery_ok, sizeof(delivery_ok));
    assert(Snapshot().command_reply.command == 4U);
}

/** @brief 运行实际 Jetson 编解码模块的离线边界测试。 @return 全部通过返回 0。 */
int main(void)
{
    /* 手动 ID 可控速度，但不能产生或清除自动心跳。 */
    Jetson_CAN_Init();
    Jetson_CAN_Receive(JETSON_CAN_MANUAL_ID, velocity_frame, sizeof(velocity_frame));
    Jetson_CAN_Data_t manual = Snapshot();
    assert(manual.velocity.valid && manual.velocity.vx == -500 && !manual.heartbeat_seen);
    Jetson_CAN_Receive(JETSON_CAN_MANUAL_ID, NULL, 0U);
    manual = Snapshot();
    assert(!manual.velocity.valid && !manual.heartbeat_seen && manual.mission_loss_sequence == 0U);
    Check_Velocity();
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
