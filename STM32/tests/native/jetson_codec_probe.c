#include "jetson_can.h"
#include "manual_can.h"
#include "task.h"
#include <assert.h>
#include <errno.h>
#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* 仅供离线跨项目互验：输入 ID#HEX，按 bsp_can 分流调用真实解码模块，逐行输出完整 JSON 快照。
 * 0x17F 额外送入 Jetson 模块一次，证明它不改变自动快照。 */
uint32_t test_tick;
static unsigned critical_depth;

/** @brief 模拟任务临界区入口并检查调用配对。 @return 无。 */
void test_enter_critical(void)
{
    assert(critical_depth == 0U);
    ++critical_depth;
}

/** @brief 模拟任务临界区出口并检查调用配对。 @return 无。 */
void test_exit_critical(void)
{
    assert(critical_depth == 1U);
    --critical_depth;
}

/**
 * @brief 将一个十六进制字符转换为数值。
 * @param[in] value 输入字符，允许 0..9、a..f、A..F。
 * @return 0..15；字符非法时返回 -1。
 */
static int Hex_Value(char value)
{
    if (value >= '0' && value <= '9')
        return value - '0';
    if (value >= 'a' && value <= 'f')
        return value - 'a' + 10;
    if (value >= 'A' && value <= 'F')
        return value - 'A' + 10;
    return -1;
}

/**
 * @brief 解析测试文本，不自行实现业务协议或缩放。
 * @param[in] line 以空字符结束的 ID#HEX，载荷允许 0..8 字节。
 * @param[out] id 标准 CAN ID，0..0x7ff。
 * @param[out] data 至少可写 8 字节。
 * @param[out] length 实际载荷字节数。
 * @return 文本格式正确返回 1；否则返回 0。
 */
static int Parse_Frame(char *line, uint16_t *id, uint8_t *data, uint8_t *length)
{
    line[strcspn(line, "\r\n")] = '\0';
    char *separator = strchr(line, '#');
    if (separator == NULL || separator == line)
        return 0;
    for (char *cursor = line; cursor != separator; ++cursor)
        if (Hex_Value(*cursor) < 0)
            return 0;
    char *end;
    errno = 0;
    unsigned long number = strtoul(line, &end, 16);
    if (errno != 0 || end != separator || number > 0x7FFUL)
        return 0;
    size_t digits = strlen(separator + 1);
    if (digits > 16U || (digits % 2U) != 0U)
        return 0;
    for (size_t byte = 0U; byte < digits / 2U; ++byte)
    {
        int high = Hex_Value(separator[1U + 2U * byte]);
        int low = Hex_Value(separator[2U + 2U * byte]);
        if (high < 0 || low < 0)
            return 0;
        data[byte] = (uint8_t)((unsigned)high * 16U + (unsigned)low);
    }
    *id = (uint16_t)number;
    *length = (uint8_t)(digits / 2U);
    return 1;
}

/**
 * @brief 输出中心快照的 JSON 字段，不换算坐标。
 * @param[in] name 固定字段名，仅由本文件调用者提供。
 * @param[in] center 非空中心快照。
 * @return 无。
 */
static void Print_Center(const char *name, const Jetson_CAN_Center_t *center)
{
    printf("\"%s\":{\"valid\":%u,\"x\":%d,\"y\":%d,\"received_ms\":%" PRIu32
           ",\"source_frame\":%" PRIu32 "}", name, (unsigned)center->valid,
           (int)center->x, (int)center->y, center->received_ms, center->source_frame);
}

/** @brief 输出真实模块的完整快照，便于检查未知帧及各路状态互不影响。 @return 无。 */
static void Print_Snapshot(void)
{
    Jetson_CAN_Data_t state;
    Manual_CAN_Velocity_t manual;
    Jetson_CAN_Get_Data(&state);
    Manual_CAN_Get_Velocity(&manual);
    printf("{\"manual\":{\"valid\":%u,\"vx\":%d,\"vy\":%d,\"wz\":%d,"
           "\"received_ms\":%" PRIu32 ",\"sequence\":%" PRIu32 ",\"invalid_sequence\":%" PRIu32
           ",\"zero_sequence\":%" PRIu32 "},",
           (unsigned)manual.valid, (int)manual.vx, (int)manual.vy, (int)manual.wz, manual.received_ms,
           manual.sequence, manual.invalid_sequence, manual.zero_sequence);
    printf("\"heartbeat\":{\"seen\":%u,\"received_ms\":%" PRIu32 ",\"mission_loss_sequence\":%" PRIu32 "},"
           "\"rx_sequence\":%" PRIu32 ",",
           (unsigned)state.heartbeat_seen, state.heartbeat_received_ms, state.mission_loss_sequence,
           state.rx_sequence);
    Print_Center("target", &state.target);
    printf(",");
    Print_Center("safety", &state.safety);
    printf(",\"arrival_ack\":{\"valid\":%u,\"accepted\":%u,\"received_ms\":%" PRIu32 "},",
           (unsigned)state.arrival_ack.valid, (unsigned)state.arrival_ack.accepted,
           state.arrival_ack.received_ms);
    printf("\"roi_reply\":{\"valid\":%u,\"passed\":%u,\"destination\":%u,\"received_ms\":%" PRIu32 "},",
           (unsigned)state.roi_reply.valid, (unsigned)state.roi_reply.passed,
           (unsigned)state.roi_reply.destination, state.roi_reply.received_ms);
    printf("\"command_reply\":{\"valid\":%u,\"accepted\":%u,\"command\":%u,"
           "\"token\":%" PRIu32 ",\"received_ms\":%" PRIu32 "},",
           (unsigned)state.command_reply.valid, (unsigned)state.command_reply.accepted,
           (unsigned)state.command_reply.command, state.command_reply.token,
           state.command_reply.received_ms);
    printf("\"turn_to_safety\":{\"valid\":%u,\"destination\":%u,"
           "\"token\":%" PRIu32 ",\"received_ms\":%" PRIu32 "}}\n",
           (unsigned)state.turn_to_safety.valid, (unsigned)state.turn_to_safety.destination,
           state.turn_to_safety.token, state.turn_to_safety.received_ms);
}

/** @brief 逐行接收离线报文，每行推进模拟时钟 1 ms。 @return 正常为 0，输入或流错误为 1。 */
int main(void)
{
    char line[128];
    Jetson_CAN_Init();
    Manual_CAN_Init();
    while (fgets(line, sizeof(line), stdin) != NULL)
    {
        uint16_t id;
        uint8_t data[8] = {0}, length;
        if (!Parse_Frame(line, &id, data, &length))
        {
            fprintf(stderr, "Invalid input frame: %s\n", line);
            return 1;
        }
        ++test_tick;
        Jetson_CAN_Receive(id, data, length);
        if (id == MANUAL_CAN_VELOCITY_ID)
            Manual_CAN_Receive(data, length);
        Print_Snapshot();
    }
    return ferror(stdin) || ferror(stdout) ? 1 : 0;
}
