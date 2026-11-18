#include "chassis.h"
#include "link_frame.h"
#include "manual_control.h"
#include "manual_link.h"
#include "task.h"
#include "uart6_manual.h"
#include "usart.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

enum
{
    ARM = 0x40, STOP, ORIGIN, RETURN, KEEPALIVE,
    STATUS = 0x80, POSE
};
enum { ACCEPTED = 1, REJECTED, DONE, FAULT };
enum { STATE = 1, ODOM, NO_ORIGIN, LIMIT, TIMEOUT, LINK, UART };
enum { RETURNING = 5, SETTLING };

uint32_t test_tick;
UART_HandleTypeDef huart6;
static uint8_t *uart_rx_byte;
static unsigned critical_depth, uart_aborts, enables, disables, stops;
static unsigned manual_vectors, return_vectors;
static uint8_t enabled, stopped, motors_ready, odom_valid;
static Chassis_Odometry_t position;
static float last_vx, last_vy, last_wz;

static struct
{
    uint8_t state;
    uint8_t flags;
    uint16_t id;
    uint8_t command;
    uint8_t result;
    uint8_t detail;
    uint8_t has_pose;
    int16_t relative[3];
} report;

void test_enter_critical(void) { ++critical_depth; }

void test_exit_critical(void)
{
    assert(critical_depth > 0U);
    --critical_depth;
}

uint8_t Chassis_Enable(void)
{
    assert(critical_depth == 0U && !enabled);
    ++enables;
    enabled = 1U;
    return 1U;
}

void Chassis_Disable(void)
{
    assert(critical_depth == 0U);
    enabled = 0U;
    ++disables;
}

void Chassis_Stop(void)
{
    assert(critical_depth == 0U);
    ++stops;
    last_vx = last_vy = last_wz = 0.0f;
}

uint8_t Chassis_Is_Enabled(void) { return enabled; }
uint8_t Chassis_Is_Stopped(void) { return enabled && stopped; }
uint8_t Chassis_Motors_Ready(void) { return motors_ready; }

uint8_t Chassis_Get_Odometry(Chassis_Odometry_t *out)
{
    if (!enabled || !odom_valid)
        return 0U;
    *out = position;
    return 1U;
}

uint8_t Chassis_Set_Velocity(float vx, float vy, float wz)
{
    assert(critical_depth == 0U && enabled && motors_ready);
    stopped = 0U;
    last_vx = vx;
    last_vy = vy;
    last_wz = wz;
    ++manual_vectors;
    return 1U;
}

uint8_t Chassis_Set_Velocity_Explicit_Yaw(float vx, float vy, float wz)
{
    assert(critical_depth == 0U && enabled && motors_ready);
    assert(isfinite(vx) && isfinite(vy) && isfinite(wz));
    assert(hypotf(vx, vy) <= CHASSIS_STRAIGHT_MAX_SPEED_MPS &&
           fabsf(wz) <= CHASSIS_VECTOR_MAX_WZ_RAD_S);
    stopped = 0U;
    last_vx = vx;
    last_vy = vy;
    last_wz = wz;
    ++return_vectors;
    return 1U;
}

HAL_StatusTypeDef HAL_UART_Receive_IT(UART_HandleTypeDef *uart, uint8_t *data, uint16_t size)
{
    assert(uart == &huart6 && size == 1U && uart->RxState == HAL_UART_STATE_READY);
    uart_rx_byte = data;
    uart->RxState = HAL_UART_STATE_BUSY_RX;
    return HAL_OK;
}

HAL_StatusTypeDef HAL_UART_AbortReceive(UART_HandleTypeDef *uart)
{
    assert(uart == &huart6);
    ++uart_aborts;
    uart_rx_byte = NULL;
    uart->RxState = HAL_UART_STATE_READY;
    return HAL_OK;
}

static uint16_t Get_U16(const uint8_t *data)
{
    return (uint16_t)(data[0] | ((uint16_t)data[1] << 8U));
}

HAL_StatusTypeDef HAL_UART_Transmit_DMA(UART_HandleTypeDef *uart, const uint8_t *data, uint16_t size)
{
    /* 正在接收时也必须能发送完整状态帧和可选的位姿帧。 */
    assert(critical_depth == 0U && uart == &huart6);
    assert(uart->gState == HAL_UART_STATE_READY && uart->RxState == HAL_UART_STATE_BUSY_RX);
    LinkParser parser = {0};
    uint8_t frames = 0U;
    report.has_pose = 0U;
    for (uint16_t i = 0; i < size; ++i)
    {
        LinkFrame frame;
        if (!LinkParser_Feed(&parser, data[i], &frame))
            continue;
        if (frames == 0U)
        {
            assert(frame.type == STATUS && frame.length == 7U);
            report.state = frame.data[0];
            report.flags = frame.data[1];
            report.id = Get_U16(frame.data + 2U);
            report.command = frame.data[4];
            report.result = frame.data[5];
            report.detail = frame.data[6];
        }
        else
        {
            assert(frames == 1U && frame.type == POSE && frame.length == 6U);
            for (unsigned axis = 0U; axis < 3U; ++axis)
                report.relative[axis] = (int16_t)Get_U16(frame.data + axis * 2U);
            report.has_pose = 1U;
        }
        ++frames;
    }
    assert(parser.count == 0U && frames == 1U + report.has_pose);
    assert(report.has_pose == ((report.flags & 3U) == 3U));
    return HAL_OK;
}

static void Feed(const uint8_t *data, uint8_t size)
{
    assert(critical_depth == 0U);
    for (uint8_t i = 0; i < size; ++i)
    {
        assert(huart6.RxState == HAL_UART_STATE_BUSY_RX && uart_rx_byte != NULL);
        *uart_rx_byte = data[i];
        huart6.RxState = HAL_UART_STATE_READY;
        HAL_UART_RxCpltCallback(&huart6);
    }
}

static void Send(uint8_t type, const uint8_t *payload, uint8_t length)
{
    uint8_t encoded[LINK_FRAME_MAX_BYTES];
    uint8_t size = LinkFrame_Encode(type, payload, length, encoded);
    assert(size > 0U);
    Feed(encoded, size);
}

static void Request(uint8_t type, uint16_t id)
{
    const uint8_t payload[] = {(uint8_t)id, (uint8_t)(id >> 8U)};
    Send(type, payload, sizeof(payload));
}

static void Request_Return_Speed(uint16_t id, uint16_t linear_milli, uint16_t angular_milli)
{
    const uint8_t payload[] = {
        (uint8_t)id, (uint8_t)(id >> 8U),
        (uint8_t)linear_milli, (uint8_t)(linear_milli >> 8U),
        (uint8_t)angular_milli, (uint8_t)(angular_milli >> 8U),
    };
    Send(RETURN, payload, sizeof(payload));
}

static void Velocity(int16_t vx, int16_t vy, int16_t wz)
{
    const uint16_t fields[] = {(uint16_t)vx, (uint16_t)vy, (uint16_t)wz};
    uint8_t payload[7] = {1U};
    for (unsigned i = 0U; i < 3U; ++i)
    {
        payload[1U + 2U * i] = (uint8_t)fields[i];
        payload[2U + 2U * i] = (uint8_t)(fields[i] >> 8U);
    }
    Send(MANUAL_CAN_VELOCITY_ID, payload, sizeof(payload));
}

static void Step(uint32_t duration_ms)
{
    test_tick += duration_ms;
    UART6_Manual_Loop();
    assert(critical_depth == 0U);
}

static void Reset(uint32_t initial_ms)
{
    test_tick = initial_ms;
    critical_depth = uart_aborts = enables = disables = stops = 0U;
    manual_vectors = return_vectors = 0U;
    enabled = stopped = 0U;
    motors_ready = odom_valid = 1U;
    position = (Chassis_Odometry_t){0};
    last_vx = last_vy = last_wz = 0.0f;
    memset(&report, 0, sizeof(report));
    huart6.gState = huart6.RxState = HAL_UART_STATE_READY;
    uart_rx_byte = NULL;
    Manual_CAN_Init();
    Manual_Control_Init();
    UART6_Manual_Init();
    Step(0U);
    assert(!enabled && report.state == MANUAL_DISARMED);
}

static void Arm_And_Confirm(uint16_t id)
{
    Request(ARM, id);
    Step(100U);
    assert(enabled && report.state == MANUAL_PREPARING);
    assert(report.command == ARM && report.id == id && report.result == ACCEPTED);
    stopped = 1U;
    Velocity(0, 0, 0);
    Step(100U);
    assert(report.state == MANUAL_READY);
}

static void Test_Manual_Frames_And_Stop(void)
{
    Reset(100U);
    Velocity(0, 0, 0);
    Velocity(100, 0, 0);
    Step(100U);
    assert(enables == 0U && manual_vectors == 0U);

    const uint8_t id[] = {0x23U, 0x01U};
    uint8_t encoded[LINK_FRAME_MAX_BYTES];
    const uint8_t size = LinkFrame_Encode(ARM, id, sizeof(id), encoded);
    encoded[size - 1U] ^= 0x80U;
    Feed(encoded, size);
    Step(100U);
    assert(enables == 0U);

    encoded[size - 1U] ^= 0x80U;
    Feed(encoded, 3U);
    Step(5U);
    Step(45U);
    Feed(encoded + 3U, size - 3U);
    Step(50U);
    assert(enables == 0U);

    Feed(encoded, 3U);
    Step(5U);
    assert(enables == 0U);
    Feed(encoded + 3U, size - 3U);
    Step(5U);
    Step(90U);
    assert(enables == 1U && report.state == MANUAL_PREPARING && report.id == 0x0123U);
    assert(manual_vectors == 0U);
    stopped = 1U;
    Velocity(0, 0, 0);
    Step(100U);
    assert(report.state == MANUAL_READY);

    Velocity(0, 0, 0);
    Request(RETURN, 1U);
    Step(100U);
    assert(report.state == MANUAL_READY && report.command == RETURN);
    assert(report.result == REJECTED && report.detail == NO_ORIGIN && return_vectors == 0U);

    Velocity(100, -30, 200);
    Step(100U);
    assert(report.state == MANUAL_RUNNING && manual_vectors == 1U);
    assert(fabsf(last_vx - 0.10f) < 1e-6f && fabsf(last_vy + 0.03f) < 1e-6f);
    assert(fabsf(last_wz - 0.20f) < 1e-6f);

    Velocity(0, 0, 0);
    Velocity(-100, 0, 0);
    Step(100U);
    assert(stops == 1U && manual_vectors == 1U && report.state == MANUAL_PREPARING);
    assert(last_vx == 0.0f && last_vy == 0.0f && last_wz == 0.0f);
    stopped = 1U;
    Velocity(0, 0, 0);
    Step(100U);
    assert(report.state == MANUAL_READY);

    Velocity(100, 0, 0);
    Send(STOP, NULL, 0U);
    Velocity(100, 0, 0);
    Request(ARM, 2U);
    Step(100U);
    assert(report.command == STOP && report.result == DONE && report.state == MANUAL_BRAKING);
    assert(manual_vectors == 1U && enables == 1U);
    Step(100U);
    assert(!enabled && disables == 1U && report.state == MANUAL_DISARMED);
    Request(ARM, 0x0123U); /* 已处理的旧使能命令不能重新启动。 */
    Velocity(0, 0, 0);
    Step(100U);
    assert(!enabled && enables == 1U);
    Arm_And_Confirm(3U);
}

static void Test_Origin_And_Return(void)
{
    Reset(100U);
    Arm_And_Confirm(1U);
    position = (Chassis_Odometry_t){.x_m = 3.0, .y_m = -2.0, .yaw_rad = 1.5707963268f};
    Velocity(0, 0, 0);
    Request(ORIGIN, 0x0102U);
    Step(100U);
    assert(report.result == DONE && report.command == ORIGIN && report.id == 0x0102U);
    assert(report.relative[0] == 0 && report.relative[1] == 0 && report.relative[2] == 0);

    position.x_m = 3.2;
    position.y_m = -1.6;
    position.yaw_rad = 2.0943951024f;
    Velocity(0, 0, 0);
    Request(ORIGIN, 0x0102U); /* 重复建原点不改变已保存的位姿。 */
    Step(100U);
    assert(report.relative[0] == 400 && report.relative[1] == -200 && report.relative[2] == 3000);

    Request(RETURN, 0x0203U);
    Step(100U);
    assert(report.state == RETURNING && report.result == ACCEPTED && report.id == 0x0203U);
    assert(last_vx < 0.0f && last_vy > 0.0f && last_wz < 0.0f);
    assert(hypotf(last_vx, last_vy) <= 0.10001f && fabsf(last_wz) <= 0.50001f);
    /* 简单的闭环车体模型把返回命令转换成世界系位姿，既检验持续控制也检验停稳复核。 */
    unsigned iterations = 0U;
    while (report.state == RETURNING && iterations++ < 150U)
    {
        const float cosine = cosf(position.yaw_rad);
        const float sine = sinf(position.yaw_rad);
        position.x_m += 0.1 * (cosine * last_vx - sine * last_vy);
        position.y_m += 0.1 * (sine * last_vx + cosine * last_vy);
        position.yaw_rad += 0.1f * last_wz;
        Velocity(100, 0, 0); /* 返航中忽略手动速度；只有保活维持会话。 */
        Send(KEEPALIVE, NULL, 0U);
        Step(100U);
    }
    assert(report.state == SETTLING && manual_vectors == 0U && return_vectors > 1U);
    assert(stops == 1U && last_vx == 0.0f && last_vy == 0.0f && last_wz == 0.0f);
    stopped = 1U;
    Send(KEEPALIVE, NULL, 0U);
    Step(100U);
    assert(report.state == MANUAL_READY && report.result == DONE && report.id == 0x0203U);
    assert(hypot(position.x_m - 3.0, position.y_m + 2.0) < 0.02);
    assert(fabsf(position.yaw_rad - 1.5707963268f) < 0.174532925f);
    Velocity(-100, 0, 0);
    Step(100U);
    assert(report.state == MANUAL_RUNNING && manual_vectors == 1U && last_vx < 0.0f);
}

static void Begin_Return(void)
{
    Arm_And_Confirm(1U);
    Velocity(0, 0, 0);
    Request(ORIGIN, 2U);
    Step(100U);
    assert(report.has_pose && report.result == DONE);
    position.x_m = 0.40;
    Request(RETURN, 3U);
    Step(100U);
    assert(report.state == RETURNING && return_vectors == 1U);
}

static void Assert_Braking(uint8_t detail)
{
    assert(enabled && report.state == MANUAL_BRAKING && !report.has_pose);
    assert(report.result == FAULT && report.detail == detail);
    assert(last_vx == 0.0f && last_vy == 0.0f && last_wz == 0.0f);
    Step(500U);
    assert(!enabled && disables == 1U && report.state == MANUAL_DISARMED);
}

static void Test_Return_Protection(void)
{
    Reset(UINT32_MAX - 450U);
    Begin_Return();
    Step(250U);
    assert(report.command == RETURN && report.id == 3U);
    Assert_Braking(LINK);

    Reset(100U);
    Begin_Return();
    odom_valid = 0U;
    Send(KEEPALIVE, NULL, 0U);
    Step(100U);
    Assert_Braking(ODOM);

    Reset(100U);
    Begin_Return();
    position.x_m = 1.81;
    Send(KEEPALIVE, NULL, 0U);
    Step(100U);
    Assert_Braking(LIMIT);

    Reset(100U);
    Begin_Return();
    for (unsigned i = 0; i < 200U; ++i)
    {
        Send(KEEPALIVE, NULL, 0U);
        Step(100U);
    }
    Assert_Braking(TIMEOUT);

    Reset(100U);
    Begin_Return();
    Send(STOP, NULL, 0U);
    Send(KEEPALIVE, NULL, 0U);
    Step(100U);
    assert(report.state == MANUAL_BRAKING && report.command == STOP && report.result == DONE);
    assert(!report.has_pose && return_vectors == 1U && last_vx == 0.0f);
    Step(500U);
    assert(!enabled && report.state == MANUAL_DISARMED);

    Reset(100U);
    Arm_And_Confirm(1U);
    Velocity(0, 0, 0);
    Request(ORIGIN, 2U);
    Step(100U);
    position.x_m = 1.51;
    Request(RETURN, 3U);
    Step(100U);
    assert(report.result == REJECTED && report.detail == LIMIT && return_vectors == 0U);
}

static void Test_Return_Speed(void)
{
    Reset(100U);
    Arm_And_Confirm(1U);
    Velocity(0, 0, 0);
    Request(ORIGIN, 2U);
    Step(100U);
    position.x_m = 0.40;
    Request_Return_Speed(3U, 200U, 800U);
    Step(100U);
    assert(report.state == RETURNING && report.result == ACCEPTED && report.id == 3U);
    assert(last_vx < -0.19f && last_vx >= -0.20001f && last_vy == 0.0f && last_wz == 0.0f);

    Reset(100U);
    Arm_And_Confirm(4U);
    Velocity(0, 0, 0);
    Request(ORIGIN, 5U);
    Step(100U);
    position.x_m = 0.20;
    unsigned vectors = return_vectors;
    Request_Return_Speed(6U, 501U, 500U);
    Step(100U);
    assert(report.result == REJECTED && report.detail == LIMIT && return_vectors == vectors);

    /* 上一条拒绝不刷新控制租约。重新准备，避免 250 ms 断流先于零转向检查。 */
    Reset(100U);
    Arm_And_Confirm(8U);
    Velocity(0, 0, 0);
    Request(ORIGIN, 9U);
    Step(100U);
    position.x_m = 0.20;
    vectors = return_vectors;
    Request_Return_Speed(10U, 100U, 0U);
    Step(100U);
    assert(report.result == REJECTED && report.detail == LIMIT && return_vectors == vectors);
}

static void Test_Full_Speed_Diagonal_Return(void)
{
    Reset(100U);
    Arm_And_Confirm(1U);
    Velocity(0, 0, 0);
    Request(ORIGIN, 2U);
    Step(100U);

    /* 这一组合在满速绕到车体坐标时会将合速度舍入到 0.50 之上。 */
    position.x_m = -cos(0.82);
    position.y_m = -sin(0.82);
    position.yaw_rad = 0.01f;
    Request_Return_Speed(3U, 500U, 2000U);
    Step(100U);
    assert(report.state == RETURNING && report.result == ACCEPTED);
    assert(hypotf(last_vx, last_vy) >= 0.4999f);
}

static void Test_Uart_Overflow(void)
{
    Reset(100U);
    Arm_And_Confirm(1U);
    Velocity(100, 0, 0);
    Step(100U);
    assert(report.state == MANUAL_RUNNING);
    uint8_t burst[129] = {0};
    Feed(burst, sizeof(burst));
    Velocity(100, 0, 0);
    Step(100U);
    assert(uart_aborts == 1U && manual_vectors == 1U);
    Assert_Braking(UART);

    Arm_And_Confirm(2U);
    HAL_UART_ErrorCallback(&huart6);
    stopped = 0U;
    Step(100U);
    assert(uart_aborts == 2U && report.result == FAULT && report.detail == UART);
}

int main(void)
{
    Test_Manual_Frames_And_Stop();
    Test_Origin_And_Return();
    Test_Return_Protection();
    Test_Return_Speed();
    Test_Full_Speed_Diagonal_Return();
    Test_Uart_Overflow();
    puts("PASS: USART6 manual framing, origin, return and fault braking");
    return 0;
}
