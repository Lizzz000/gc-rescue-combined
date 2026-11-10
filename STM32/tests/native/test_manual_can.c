#include <assert.h>
#include <stddef.h>
#include <stdio.h>
#include "manual_can.h"
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
        Manual_CAN_Receive(velocity_frame, sizeof(velocity_frame));
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
        Manual_CAN_Receive(zero_frame, sizeof(zero_frame));
    }
}

/** @brief 读取真实手动模块并核对临界区成对进入/退出。 @return 当前快照。 */
static Manual_CAN_Velocity_t Snapshot(void)
{
    Manual_CAN_Velocity_t snapshot;
    unsigned before = critical_entries;
    Manual_CAN_Get_Velocity(&snapshot);
    assert(critical_entries == before + 1U && critical_entries == critical_exits);
    assert(critical_depth == 0U);
    return snapshot;
}

/** @brief 核对小端负数、全范围定点原值及无效帧即时覆盖旧指令。 */
static void Check_Velocity(void)
{
    Manual_CAN_Init();
    Manual_CAN_Velocity_t data = Snapshot();
    assert(!data.valid && data.sequence == 0U && !data.invalid_sequence && !data.zero_sequence);
    test_tick = 123U;
    Manual_CAN_Receive(velocity_frame, sizeof(velocity_frame));
    data = Snapshot();
    assert(data.valid && data.vx == -500 && data.vy == 250);
    assert(data.wz == -1000 && data.received_ms == 123U);
    assert(data.sequence == 1U && !data.invalid_sequence);
    const uint8_t extremes[7] = {1U, 0U, 0x80U, 0xFFU, 0x7FU, 0xFFU, 0xFFU};
    Manual_CAN_Receive(extremes, sizeof(extremes));
    data = Snapshot();
    assert(data.vx == INT16_MIN && data.vy == INT16_MAX);
    assert(data.wz == -1 && data.sequence == 2U);
    for (uint8_t valid = 0U; valid <= 2U; valid += 2U)
    {
        uint8_t invalid[7] = {valid, 1U, 2U, 3U, 4U, 5U, 6U};
        uint32_t sequence = data.sequence;
        uint32_t invalid_sequence = data.invalid_sequence;
        ++test_tick;
        Manual_CAN_Receive(invalid, sizeof(invalid));
        data = Snapshot();
        assert(!data.valid && !data.vx && !data.vy && !data.wz);
        assert(data.sequence == sequence + 1U && data.received_ms == test_tick);
        assert(data.invalid_sequence == invalid_sequence + 1U);
        assert(data.zero_sequence == 0U);
    }
    Manual_CAN_Receive(zero_frame, sizeof(zero_frame));
    data = Snapshot();
    assert(data.valid && !data.vx && !data.vy && !data.wz);
    assert(data.zero_sequence == 1U);
    uint32_t sequence = data.sequence;
    test_tick = UINT32_MAX;
    Manual_CAN_Receive(NULL, 7U);
    data = Snapshot();
    assert(!data.valid && data.sequence == sequence + 1U);
    assert(data.received_ms == UINT32_MAX && data.zero_sequence == 1U);
    test_tick = 0U;
    Manual_CAN_Receive(velocity_frame, sizeof(velocity_frame));
    assert(Snapshot().received_ms == 0U);
    /* 两次接收之间不运行任务：末条有效速度不能抹去前条撤权。 */
    uint32_t invalid_sequence = Snapshot().invalid_sequence;
    Manual_CAN_Receive(NULL, 0U);
    Manual_CAN_Receive(velocity_frame, sizeof(velocity_frame));
    data = Snapshot();
    assert(data.valid && data.vx == -500);
    assert(data.invalid_sequence == invalid_sequence + 1U && data.zero_sequence == 1U);
    Manual_CAN_Receive(zero_frame, sizeof(zero_frame));
    data = Snapshot();
    assert(data.invalid_sequence == invalid_sequence + 1U && data.zero_sequence == 2U);
}

/** @brief 每种错误 DLC 都撤销上一条有效速度并推进序号。 */
static void Check_Lengths(void)
{
    for (uint8_t len = 0U; len <= 8U; ++len)
    {
        if (len == sizeof(velocity_frame))
            continue;
        uint8_t frame[8] = {1U, 0x0CU, 0xFEU, 0xFAU, 0U, 0x18U, 0xFCU, 0U};
        Manual_CAN_Receive(velocity_frame, sizeof(velocity_frame));
        const Manual_CAN_Velocity_t before = Snapshot();
        Manual_CAN_Receive(frame, len);
        const Manual_CAN_Velocity_t data = Snapshot();
        assert(!data.valid && !data.vx && !data.vy && !data.wz);
        assert(data.sequence == before.sequence + 1U);
        assert(data.invalid_sequence == before.invalid_sequence + 1U);
    }
}

/** @brief 快照隔离：屏蔽前到达的帧被复制，恢复后到达的帧不污染副本；初始化清零。 */
static void Check_Snapshot(void)
{
    receive_on_enter = receive_on_exit = 1U;
    const Manual_CAN_Velocity_t before = Snapshot();
    assert(before.valid && before.vx == -500 && before.wz == -1000);
    const Manual_CAN_Velocity_t after = Snapshot();
    assert(after.valid && !after.vx && !after.wz && after.sequence == before.sequence + 1U);
    Manual_CAN_Init();
    const Manual_CAN_Velocity_t cleared = Snapshot();
    assert(!cleared.valid && !cleared.sequence && !cleared.received_ms);
    assert(!cleared.invalid_sequence && !cleared.zero_sequence);
}

/** @brief 运行真实手动速度快照模块的离线边界测试。 @return 全部通过返回 0。 */
int main(void)
{
    Check_Velocity();
    Check_Lengths();
    Check_Snapshot();
    puts("PASS Manual CAN: 0x17F decode, revocation events and snapshot isolation");
    return 0;
}
