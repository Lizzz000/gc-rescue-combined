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
static const uint8_t zero_frame[7] = {1U, 0U, 0U, 0U, 0U, 0U, 0U};
/* 类型 1 心跳长度 8：alive、pause，然后 vx/vy/wz。下面这帧 alive=1、pause=0，速度同 velocity_frame。 */
static const uint8_t heartbeat_frame[8] = {1U, 0U, 0x0CU, 0xFEU, 0xFAU, 0U, 0x18U, 0xFCU};
static const uint8_t zero_heartbeat[8] = {1U, 0U, 0U, 0U, 0U, 0U, 0U, 0U};

/** @brief 模拟临界区入口；可在屏蔽中断之前接收一帧，复制须包含该帧。 */
void test_enter_critical(void)
{
    assert(critical_depth == 0U);
    if (receive_on_enter)
    {
        receive_on_enter = 0U;
        Jetson_CAN_Receive(JETSON_CAN_HEARTBEAT_ID, heartbeat_frame, sizeof(heartbeat_frame));
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
        Jetson_CAN_Receive(JETSON_CAN_HEARTBEAT_ID, zero_heartbeat, sizeof(zero_heartbeat));
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

/** @brief 类型 1 只认长度 8。alive=1 且 pause 为 0/1 才保活；alive=0 且速度全 0 才是失联。 */
static void Check_Heartbeat(void)
{
    Jetson_CAN_Init();
    uint8_t heartbeat[8] = {1U, 0U, 0U, 0U, 0U, 0U, 0U, 0U};
    test_tick = 10U;
    Jetson_CAN_Receive(JETSON_CAN_HEARTBEAT_ID, heartbeat, sizeof(heartbeat));
    Jetson_CAN_Data_t data = Snapshot();
    assert(data.heartbeat_seen && data.heartbeat_received_ms == 10U && data.rx_sequence == 1U);
    assert(data.heartbeat_sequence == 1U && !data.heartbeat_pause);
    test_tick = 20U;
    heartbeat[1] = 1U;
    Jetson_CAN_Receive(JETSON_CAN_HEARTBEAT_ID, heartbeat, sizeof(heartbeat));
    data = Snapshot();
    assert(data.heartbeat_received_ms == 20U && data.heartbeat_sequence == 2U && data.heartbeat_pause);
    Jetson_CAN_Receive(JETSON_CAN_HEARTBEAT_ID, heartbeat_frame, sizeof(heartbeat_frame));
    data = Snapshot();
    assert(data.heartbeat_sequence == 3U && !data.heartbeat_pause);

    /* 长度不对、pause 越界、alive 越界、空指针和空载荷都不是心跳，也不计失联。 */
    test_tick = 30U;
    heartbeat[1] = 2U;
    const uint8_t malformed[8] = {2U};
    Jetson_CAN_Receive(JETSON_CAN_HEARTBEAT_ID, velocity_frame, sizeof(velocity_frame));
    Jetson_CAN_Receive(JETSON_CAN_HEARTBEAT_ID, heartbeat, sizeof(heartbeat));
    Jetson_CAN_Receive(JETSON_CAN_HEARTBEAT_ID, malformed, sizeof(malformed));
    Jetson_CAN_Receive(JETSON_CAN_HEARTBEAT_ID, NULL, 8U);
    Jetson_CAN_Receive(JETSON_CAN_HEARTBEAT_ID, NULL, 0U);
    data = Snapshot();
    assert(data.heartbeat_seen && data.heartbeat_received_ms == 20U && data.rx_sequence == 8U);
    assert(data.heartbeat_sequence == 3U && !data.heartbeat_pause);
    assert(data.mission_loss_sequence == 0U);
    /* 失联后紧跟心跳：心跳恢复，但失联计数保留给任务。速度非 0 的 alive=0 不算失联。 */
    const uint8_t lost[8] = {0U};
    const uint8_t nonzero_lost[8] = {0U, 0U, 1U, 0U, 0U, 0U, 0U, 0U};
    Jetson_CAN_Receive(JETSON_CAN_HEARTBEAT_ID, lost, sizeof(lost));
    data = Snapshot();
    assert(!data.heartbeat_seen && data.mission_loss_sequence == 1U && data.rx_sequence == 9U);
    Jetson_CAN_Receive(JETSON_CAN_HEARTBEAT_ID, nonzero_lost, sizeof(nonzero_lost));
    data = Snapshot();
    assert(!data.heartbeat_seen && data.mission_loss_sequence == 1U && data.rx_sequence == 10U);
    Jetson_CAN_Receive(JETSON_CAN_HEARTBEAT_ID, zero_heartbeat, sizeof(zero_heartbeat));
    data = Snapshot();
    assert(data.heartbeat_seen && data.heartbeat_received_ms == 30U && data.mission_loss_sequence == 1U);
}

/** @brief 手动 类型 2 对 Jetson 模块是未知 ID：不产生或清除心跳，不推进接收序号。 */
static void Check_Manual_Ignored(void)
{
    Jetson_CAN_Init();
    Jetson_CAN_Receive(JETSON_CAN_HEARTBEAT_ID, heartbeat_frame, sizeof(heartbeat_frame));
    const Jetson_CAN_Data_t before = Snapshot();
    const uint8_t zero_17f[7] = {1U};
    Jetson_CAN_Receive(MANUAL_CAN_VELOCITY_ID, velocity_frame, sizeof(velocity_frame));
    Jetson_CAN_Receive(MANUAL_CAN_VELOCITY_ID, zero_17f, sizeof(zero_17f));
    Jetson_CAN_Receive(MANUAL_CAN_VELOCITY_ID, NULL, 0U);
    const Jetson_CAN_Data_t after = Snapshot();
    assert(memcmp(&before, &after, sizeof(before)) == 0);
    Jetson_CAN_Init();
    Jetson_CAN_Receive(MANUAL_CAN_VELOCITY_ID, velocity_frame, sizeof(velocity_frame));
    const Jetson_CAN_Data_t idle = Snapshot();
    assert(!idle.heartbeat_seen && !idle.rx_sequence && !idle.mission_loss_sequence);
}

/** @brief 逐项验证精确 DLC；错误长度不得沿用上一条有效数据，也不刷新心跳。 */
static void Check_Lengths(void)
{
    const uint16_t ids[] = {JETSON_CAN_HEARTBEAT_ID, JETSON_CAN_TARGET_CENTER_ID, JETSON_CAN_SAFETY_CENTER_ID, JETSON_CAN_ARRIVAL_REPLY_ID, JETSON_CAN_ROI_REPLY_ID, JETSON_CAN_COMMAND_REPLY_ID, JETSON_CAN_TURN_REQUEST_ID};
    const uint8_t lengths[] = {8U, 8U, 8U, 1U, 3U, 6U, 6U};
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
            case JETSON_CAN_HEARTBEAT_ID:
                assert(data.heartbeat_received_ms == before.heartbeat_received_ms);
                assert(data.heartbeat_seen);
                assert(data.mission_loss_sequence == before.mission_loss_sequence);
                break;
            case JETSON_CAN_TARGET_CENTER_ID: assert(!data.target.valid && !data.target.x && !data.target.y); break;
            case JETSON_CAN_SAFETY_CENTER_ID: assert(!data.safety.valid && !data.safety.x && !data.safety.y); break;
            case JETSON_CAN_ARRIVAL_REPLY_ID: assert(!data.arrival_ack.valid && !data.arrival_ack.accepted); break;
            case JETSON_CAN_ROI_REPLY_ID: assert(!data.roi_reply.valid && !data.roi_reply.passed); break;
            case JETSON_CAN_COMMAND_REPLY_ID: assert(!data.command_reply.valid && !data.command_reply.token); break;
            case JETSON_CAN_TURN_REQUEST_ID: assert(!data.turn_to_safety.valid && !data.turn_to_safety.token); break;
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

/** @brief 核对 token 全范围、搜索 token 固定零，以及目的区/命令/布尔的边界。 */
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
                                ((command != 3U && command != 5U) || token == 0U);
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
    assert(before.heartbeat_seen && before.heartbeat_received_ms == test_tick);
    Jetson_CAN_Data_t after = Snapshot();
    assert(after.rx_sequence == before.rx_sequence + 1U);
    const uint16_t ignored[] = {0U, JETSON_CAN_ARRIVAL_ID, 0x201U, 0x202U, JETSON_CAN_TURN_COMPLETED_ID, 0x7FFU, 0x1180U};
    for (unsigned i = 0U; i < sizeof(ignored) / sizeof(ignored[0]); ++i)
        Jetson_CAN_Receive(ignored[i], NULL, 255U);
    assert(Snapshot().rx_sequence == after.rx_sequence);
    Jetson_CAN_Init();
    after = Snapshot();
    assert(!after.heartbeat_seen && !after.rx_sequence && !after.heartbeat_received_ms);
    assert(!after.target.valid && !after.safety.valid && !after.arrival_ack.valid);
    assert(!after.roi_reply.valid && !after.command_reply.valid && !after.turn_to_safety.valid);
}

/** @brief 核对六种上行帧的 DLC、小端 token、类型 21 到点类型和 HAL 所需的尾部缓冲。 */
static void Check_Uplink(void)
{
    const uint16_t tokenless_ids[] = {JETSON_CAN_ARRIVAL_ID, JETSON_CAN_START_SEARCH_ID};
    const uint16_t token_ids[] = {JETSON_CAN_TURN_COMPLETED_ID, JETSON_CAN_SAFETY_ARRIVED_ID, JETSON_CAN_DELIVERY_COMPLETED_ID};
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
    const uint16_t rejected_ids[] = {0U, JETSON_CAN_HEARTBEAT_ID, JETSON_CAN_TURN_REQUEST_ID, 0x201U, 0x202U, 0x207U, 0xFFFFU};
    for (unsigned i = 0U; i < sizeof(rejected_ids) / sizeof(rejected_ids[0]); ++i)
    {
        assert(Jetson_CAN_Encode_Uplink(rejected_ids[i], UINT32_MAX, data) == 0U);
        assert(memcmp(data, tokenless, sizeof(data)) == 0);
    }
    assert(Jetson_CAN_Encode_Uplink(JETSON_CAN_ARRIVAL_ID, 0U, NULL) == 0U);

    /* 类型 21 到点：DLC 1，data[0] 为到点类型；类型越界不编码、不改缓冲区。 */
    const uint8_t grasp[8] = {0x01U};
    const uint8_t release[8] = {0x02U};
    memset(data, 0xAA, sizeof(data));
    assert(Jetson_CAN_Encode_Uplink(JETSON_CAN_POINT_REACHED_ID, 1U, data) == 1U && memcmp(data, grasp, sizeof(data)) == 0);
    memset(data, 0xAA, sizeof(data));
    assert(Jetson_CAN_Encode_Uplink(JETSON_CAN_POINT_REACHED_ID, 2U, data) == 1U && memcmp(data, release, sizeof(data)) == 0);
    const uint32_t bad_points[] = {0U, 3U, 0x101U, UINT32_MAX};
    for (unsigned i = 0U; i < sizeof(bad_points) / sizeof(bad_points[0]); ++i)
    {
        assert(Jetson_CAN_Encode_Uplink(JETSON_CAN_POINT_REACHED_ID, bad_points[i], data) == 0U);
        assert(memcmp(data, release, sizeof(data)) == 0);
    }
}

/** @brief 同一毫秒内区分心跳、已知帧到达顺序与未知帧。 */
static void Check_Receive_Sequence(void)
{
    Jetson_CAN_Init();
    test_tick = 42U;
    Jetson_CAN_Receive(JETSON_CAN_HEARTBEAT_ID, zero_heartbeat, sizeof(zero_heartbeat));
    Jetson_CAN_Data_t data = Snapshot();
    assert(data.rx_sequence == 1U);
    assert(data.heartbeat_seen && data.heartbeat_received_ms == 42U);

    const uint8_t invalid[8] = {1U, 0U, 0U, 0U, 0U, 0U, 0U, 0U};
    ++test_tick;
    Jetson_CAN_Receive(JETSON_CAN_HEARTBEAT_ID, invalid, sizeof(invalid));
    data = Snapshot();
    assert(data.rx_sequence == 2U);
    assert(data.heartbeat_seen && data.heartbeat_received_ms == 43U);
    const uint8_t malformed[8] = {2U};
    ++test_tick;
    Jetson_CAN_Receive(JETSON_CAN_HEARTBEAT_ID, malformed, sizeof(malformed));
    data = Snapshot();
    assert(data.rx_sequence == 3U);
    assert(data.heartbeat_received_ms == 43U);
    Jetson_CAN_Receive(JETSON_CAN_HEARTBEAT_ID, zero_frame, sizeof(zero_frame));
    data = Snapshot();
    assert(data.rx_sequence == 4U && data.heartbeat_received_ms == 43U);

    const uint8_t frame[8] = {1U, 1U, 1U};
    Jetson_CAN_Receive(JETSON_CAN_TARGET_CENTER_ID, frame, 8U);
    Jetson_CAN_Receive(JETSON_CAN_SAFETY_CENTER_ID, frame, 8U);
    Jetson_CAN_Receive(JETSON_CAN_ARRIVAL_REPLY_ID, frame, 1U);
    Jetson_CAN_Receive(JETSON_CAN_ROI_REPLY_ID, frame, 3U);
    Jetson_CAN_Receive(JETSON_CAN_COMMAND_REPLY_ID, frame, 6U);
    Jetson_CAN_Receive(JETSON_CAN_TURN_REQUEST_ID, frame, 6U);
    Jetson_CAN_Receive(0x194U, frame, 6U);
    data = Snapshot();
    assert(data.rx_sequence == 10U);
    assert(data.target.sequence == 5U && data.target.received_ms == test_tick);
    assert(data.safety.sequence == 6U && data.safety.received_ms == test_tick);
    assert(data.arrival_ack.sequence == 7U && data.arrival_ack.received_ms == test_tick);
    assert(data.roi_reply.sequence == 8U && data.roi_reply.received_ms == test_tick);
    assert(data.command_reply.sequence == 9U && data.command_reply.received_ms == test_tick);
    assert(data.turn_to_safety.sequence == 10U && data.turn_to_safety.received_ms == test_tick);
    Jetson_CAN_Receive(JETSON_CAN_TURN_REQUEST_ID, frame, 5U);
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
    Jetson_CAN_Receive(JETSON_CAN_COMMAND_REPLY_ID, turn_ok, sizeof(turn_ok));
    assert(Snapshot().command_reply.accepted);
    Jetson_CAN_Expect_Command_Reply(JETSON_CAN_TURN_COMPLETED_ID, 1U);
    assert(!Snapshot().command_reply.valid);
    Jetson_CAN_Receive(JETSON_CAN_COMMAND_REPLY_ID, wrong_token, sizeof(wrong_token));
    assert(!Snapshot().command_reply.valid);
    Jetson_CAN_Receive(JETSON_CAN_COMMAND_REPLY_ID, turn_ok, sizeof(turn_ok));
    const uint32_t sequence = Snapshot().command_reply.sequence;
    Jetson_CAN_Receive(JETSON_CAN_COMMAND_REPLY_ID, delivery_ok, sizeof(delivery_ok));
    Jetson_CAN_Receive(JETSON_CAN_COMMAND_REPLY_ID, turn_ok, sizeof(turn_ok));
    assert(Snapshot().command_reply.sequence == sequence);

    Jetson_CAN_Expect_Command_Reply(0U, 0U);
    Jetson_CAN_Receive(JETSON_CAN_COMMAND_REPLY_ID, delivery_ok, sizeof(delivery_ok));
    assert(Snapshot().command_reply.command == 4U);
    Jetson_CAN_Expect_Command_Reply(JETSON_CAN_START_SEARCH_ID, UINT32_MAX);
    Jetson_CAN_Receive(JETSON_CAN_COMMAND_REPLY_ID, search_ok, sizeof(search_ok));
    assert(Snapshot().command_reply.command == 3U);

    /* 类型 21 等待命令 5 且 token 固定为 0，不接受同 token 的其他命令。 */
    const uint8_t point_ok[6] = {1U, 5U};
    const uint8_t point_token[6] = {1U, 5U, 1U};
    Jetson_CAN_Expect_Command_Reply(JETSON_CAN_POINT_REACHED_ID, 7U);
    assert(!Snapshot().command_reply.valid);
    Jetson_CAN_Receive(JETSON_CAN_COMMAND_REPLY_ID, search_ok, sizeof(search_ok));
    Jetson_CAN_Receive(JETSON_CAN_COMMAND_REPLY_ID, point_token, sizeof(point_token));
    assert(!Snapshot().command_reply.valid);
    Jetson_CAN_Receive(JETSON_CAN_COMMAND_REPLY_ID, point_ok, sizeof(point_ok));
    assert(Snapshot().command_reply.valid && Snapshot().command_reply.command == 5U &&
           !Snapshot().command_reply.token);
    Jetson_CAN_Init();
    Jetson_CAN_Receive(JETSON_CAN_COMMAND_REPLY_ID, delivery_ok, sizeof(delivery_ok));
    assert(Snapshot().command_reply.command == 4U);
}

/** @brief 运行实际 Jetson 编解码模块的离线边界测试。 @return 全部通过返回 0。 */
int main(void)
{
    Check_Velocity_Decode();
    Check_Heartbeat();
    Check_Manual_Ignored();
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
