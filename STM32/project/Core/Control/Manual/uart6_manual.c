#include "uart6_manual.h"
#include "chassis.h"
#include "link_frame.h"
#include "manual_control.h"
#include "manual_link.h"
#include "point_plan.h"
#include "usart.h"
#include "FreeRTOS.h"
#include "task.h"
#include <math.h>

enum
{
    UART_ARM = 0x40,
    UART_STOP,
    UART_ORIGIN,
    UART_RETURN,
    UART_KEEPALIVE,
    UART_STATUS = 0x80,
    UART_POSE
};

enum { RESULT_NONE, RESULT_ACCEPTED, RESULT_REJECTED, RESULT_DONE, RESULT_FAULT };
enum { DETAIL_NONE, DETAIL_STATE, DETAIL_ODOM, DETAIL_NO_ORIGIN, DETAIL_LIMIT,
       DETAIL_TIMEOUT, DETAIL_LINK, DETAIL_UART, DETAIL_CHASSIS };
enum { RETURN_INACTIVE, RETURN_RUNNING, RETURN_STOPPING };

#define RX_CAPACITY 128U
#define RX_FRAME_GAP_MS 50U
#define REPORT_INTERVAL_MS 100U
#define RETURN_TIMEOUT_MS 20000U
#define RETURN_MAX_DISTANCE_M 1.5
#define RETURN_FENCE_M 1.8
#define TWO_PI 6.28318530717958647692f

static const Point_Plan_Config_t default_return_config = {
    .linear_limit_mps = 0.10f,
    .angular_limit_rad_s = 0.50f,
    .linear_deceleration_mps2 = 0.30f,
    .angular_deceleration_rad_s2 = 0.80f,
    .latency_s = 0.12f,
    .position_tolerance_m = 0.01f,
    .yaw_tolerance_rad = 0.06981317f,
};
/* 长度 2 的返航沿用上面的默认限速。长度 6 在请求号后追加本次限速。 */
static Point_Plan_Config_t return_config;

static uint8_t rx_byte;
typedef struct
{
    uint8_t data[RX_CAPACITY];
    uint32_t time_ms[RX_CAPACITY];
} Rx_Batch_t;
static Rx_Batch_t rx_batches[2];
static volatile uint8_t rx_bank;
static volatile uint16_t rx_count;
static volatile uint8_t rx_failed;
static LinkParser parser;
static uint32_t parser_activity_ms;
static uint8_t tx_buffer[2U * LINK_FRAME_MAX_BYTES];

static uint8_t session_active, was_enabled, origin_valid, return_state;
static uint32_t arm_started_ms, last_valid_ms, return_started_ms, stop_started_ms, last_report_ms;
static uint16_t last_arm_id, last_origin_id, last_return_id, active_return_id;
static uint16_t reply_id;
static uint8_t reply_command, reply_result, reply_detail;
static Chassis_Odometry_t origin;
static Point_Plan_t return_plan;

static void Reply(uint8_t command, uint16_t id, uint8_t result, uint8_t detail)
{
    reply_command = command;
    reply_id = id;
    reply_result = result;
    reply_detail = detail;
}

static void Stop_Session(uint8_t detail)
{
    if (return_state != RETURN_INACTIVE && detail != DETAIL_NONE)
        Reply(UART_RETURN, active_return_id, RESULT_FAULT, detail);
    else
        Reply(UART_STOP, 0U, detail == DETAIL_NONE ? RESULT_DONE : RESULT_FAULT, detail);
    session_active = was_enabled = origin_valid = 0U;
    return_state = RETURN_INACTIVE;
    /* 复用手动控制的制动与失能期限，也撤销还未生效的零速使能。 */
    Manual_CAN_Receive(NULL, 0U);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *uart)
{
    if (uart != &huart6)
        return;
    if (!rx_failed && rx_count < RX_CAPACITY)
    {
        rx_batches[rx_bank].data[rx_count] = rx_byte;
        rx_batches[rx_bank].time_ms[rx_count] = (uint32_t)xTaskGetTickCountFromISR();
        ++rx_count;
    }
    else
        rx_failed = 1U;
    if (HAL_UART_Receive_IT(uart, &rx_byte, 1U) != HAL_OK)
        rx_failed = 1U;
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *uart)
{
    if (uart == &huart6)
        rx_failed = 1U;
}

static void Service_Request(const LinkFrame *frame, uint32_t now_ms)
{
    /* 返航可只带请求号，或再带平移、转向限速；使能和建原点仍只有请求号。 */
    const uint8_t speed_return = frame->type == UART_RETURN && frame->length == 6U;
    if (!(frame->length == 2U || speed_return))
        return;
    const uint16_t id = (uint16_t)(frame->data[0] | ((uint16_t)frame->data[1] << 8U));
    uint16_t *last_id = &last_return_id;
    if (frame->type == UART_ARM)
        last_id = &last_arm_id;
    else if (frame->type == UART_ORIGIN)
        last_id = &last_origin_id;
    if (id == 0U || id == *last_id)
        return;
    *last_id = id;

    if (frame->type == UART_ARM)
    {
        if (session_active || manual_control_state != MANUAL_DISARMED || Chassis_Is_Enabled())
        {
            Reply(UART_ARM, id, RESULT_REJECTED, DETAIL_STATE);
            return;
        }
        /* 显式使能命令等价于手动状态机的一帧合法零速。 */
        const uint8_t zero[7] = {1U};
        Manual_CAN_Receive(zero, sizeof(zero));
        arm_started_ms = last_valid_ms = now_ms;
        session_active = 1U;
        Reply(UART_ARM, id, RESULT_ACCEPTED, DETAIL_NONE);
        return;
    }

    if (!session_active || return_state != RETURN_INACTIVE ||
        manual_control_state != MANUAL_READY ||
        (frame->type == UART_ORIGIN ? !Chassis_Is_Stopped() : !Chassis_Motion_Ready()))
    {
        Reply(frame->type, id, RESULT_REJECTED, DETAIL_STATE);
        return;
    }

    Chassis_Odometry_t position;
    if (!Chassis_Get_Odometry(&position))
    {
        Reply(frame->type, id, RESULT_REJECTED, DETAIL_ODOM);
        return;
    }

    if (frame->type == UART_ORIGIN)
    {
        origin = position;
        origin_valid = 1U;
        Reply(UART_ORIGIN, id, RESULT_DONE, DETAIL_NONE);
        return;
    }
    if (!origin_valid)
    {
        Reply(UART_RETURN, id, RESULT_REJECTED, DETAIL_NO_ORIGIN);
        return;
    }

    Point_Plan_Config_t config = default_return_config;
    if (speed_return)
    {
        const uint16_t linear_milli = (uint16_t)(frame->data[2] | ((uint16_t)frame->data[3] << 8U));
        const uint16_t angular_milli = (uint16_t)(frame->data[4] | ((uint16_t)frame->data[5] << 8U));
        config.linear_limit_mps = linear_milli / 1000.0f;
        config.angular_limit_rad_s = angular_milli / 1000.0f;
        if (linear_milli == 0U || angular_milli == 0U ||
            config.linear_limit_mps > CHASSIS_STRAIGHT_MAX_SPEED_MPS ||
            config.angular_limit_rad_s > CHASSIS_VECTOR_MAX_WZ_RAD_S)
        {
            Reply(UART_RETURN, id, RESULT_REJECTED, DETAIL_LIMIT);
            return;
        }
        /* 满速斜向转到车体坐标时留出浮点舍入余量，避免合法目标被底盘拒绝。 */
        config.linear_limit_mps = fminf(config.linear_limit_mps, CHASSIS_STRAIGHT_MAX_SPEED_MPS - 1e-6f);
    }

    const double from[3] = {position.x_m, position.y_m, position.yaw_rad};
    const double goal[3] = {origin.x_m, origin.y_m, origin.yaw_rad};
    if (!Point_Plan_Init(&return_plan, from, goal) || return_plan.length_m > RETURN_MAX_DISTANCE_M)
    {
        Reply(UART_RETURN, id, RESULT_REJECTED, DETAIL_LIMIT);
        return;
    }

    return_config = config;
    active_return_id = id;
    return_started_ms = last_valid_ms = now_ms;
    return_state = RETURN_RUNNING;
    Reply(UART_RETURN, id, RESULT_ACCEPTED, DETAIL_NONE);
}

static void Service_Return(uint32_t now_ms)
{
    Chassis_Odometry_t position;
    if (!Chassis_Is_Enabled() || !Chassis_Motors_Ready())
    {
        Stop_Session(DETAIL_CHASSIS);
        return;
    }
    if (!Chassis_Get_Odometry(&position))
    {
        Stop_Session(DETAIL_ODOM);
        return;
    }
    if ((uint32_t)(now_ms - return_started_ms) >= RETURN_TIMEOUT_MS)
    {
        Stop_Session(DETAIL_TIMEOUT);
        return;
    }
    const double remaining = hypot(position.x_m - origin.x_m, position.y_m - origin.y_m);
    if (remaining > RETURN_FENCE_M)
    {
        Stop_Session(DETAIL_LIMIT);
        return;
    }

    if (return_state == RETURN_STOPPING)
    {
        if (remaining > 0.02 || fabsf(remainderf(position.yaw_rad - origin.yaw_rad, TWO_PI)) > 0.174532925f)
            Stop_Session(DETAIL_LIMIT);
        else if (Chassis_Motion_Ready())
        {
            const uint8_t zero[7] = {1U};
            Manual_CAN_Receive(zero, sizeof(zero));
            Reply(UART_RETURN, active_return_id, RESULT_DONE, DETAIL_NONE);
            return_state = RETURN_INACTIVE;
        }
        else if ((uint32_t)(now_ms - stop_started_ms) >= MANUAL_PREPARE_TIMEOUT_MS)
            Stop_Session(DETAIL_TIMEOUT);
        return;
    }

    const double pose[3] = {position.x_m, position.y_m, position.yaw_rad};
    Point_Plan_Output_t output;
    /* 减速度、滞后和到位窗口仍用统一配置，只替换本次命令的速度上限。 */
    if (!Point_Plan_Step(&return_plan, &return_config, pose, &output))
        Stop_Session(DETAIL_ODOM);
    else if (output.arrived)
    {
        Chassis_Stop();
        stop_started_ms = now_ms;
        return_state = RETURN_STOPPING;
    }
    else if (!Chassis_Set_Velocity_Explicit_Yaw(output.vx_mps, output.vy_mps, output.wz_rad_s))
        Stop_Session(DETAIL_CHASSIS);
}

static void Report_Status(uint32_t now_ms)
{
    if ((uint32_t)(now_ms - last_report_ms) < REPORT_INTERVAL_MS || huart6.gState != HAL_UART_STATE_READY)
        return;

    Chassis_Odometry_t position;
    const uint8_t odom_valid = Chassis_Get_Odometry(&position);
    uint8_t state = (uint8_t)manual_control_state;
    if (return_state != RETURN_INACTIVE)
        state = return_state == RETURN_RUNNING ? 5U : 6U;
    const uint8_t status[] = {
        state, (uint8_t)(origin_valid | (odom_valid << 1U)),
        (uint8_t)reply_id, (uint8_t)(reply_id >> 8U), reply_command, reply_result, reply_detail,
    };
    uint8_t tx_length = LinkFrame_Encode(UART_STATUS, status, sizeof(status), tx_buffer);
    if (origin_valid && odom_valid)
    {
        const double dx = position.x_m - origin.x_m;
        const double dy = position.y_m - origin.y_m;
        const float cosine = cosf(origin.yaw_rad);
        const float sine = sinf(origin.yaw_rad);
        const double relative[3] = {
            1000.0 * (cosine * dx + sine * dy),
            1000.0 * (-sine * dx + cosine * dy),
            5729.5779513 * remainderf(position.yaw_rad - origin.yaw_rad, TWO_PI),
        };
        uint8_t payload[6];
        for (unsigned index = 0U; index < 3U; ++index)
        {
            const int16_t value = (int16_t)lround(fmax(-32768.0, fmin(32767.0, relative[index])));
            payload[2U * index] = (uint8_t)value;
            payload[2U * index + 1U] = (uint8_t)((uint16_t)value >> 8U);
        }
        tx_length += LinkFrame_Encode(UART_POSE, payload, sizeof(payload), tx_buffer + tx_length);
    }
    if (HAL_UART_Transmit_DMA(&huart6, tx_buffer, tx_length) == HAL_OK)
        last_report_ms = now_ms;
}

void UART6_Manual_Init(void)
{
    parser = (LinkParser){0};
    session_active = was_enabled = origin_valid = 0U;
    return_state = RETURN_INACTIVE;
    last_arm_id = last_origin_id = last_return_id = active_return_id = 0U;
    Reply(0U, 0U, RESULT_NONE, DETAIL_NONE);
    last_report_ms = (uint32_t)xTaskGetTickCount() - REPORT_INTERVAL_MS;
    taskENTER_CRITICAL();
    rx_count = 0U;
    rx_failed = HAL_UART_Receive_IT(&huart6, &rx_byte, 1U) != HAL_OK;
    taskEXIT_CRITICAL();
}

void UART6_Manual_Loop(void)
{
    taskENTER_CRITICAL();
    const uint32_t now_ms = (uint32_t)xTaskGetTickCount();
    const uint16_t count = rx_count;
    const Rx_Batch_t *received = &rx_batches[rx_bank];
    const uint8_t failure = rx_failed;
    rx_bank ^= 1U;
    rx_count = 0U;
    if (failure)
    {
        HAL_UART_AbortReceive(&huart6);
        rx_failed = HAL_UART_Receive_IT(&huart6, &rx_byte, 1U) != HAL_OK;
    }
    taskEXIT_CRITICAL();

    if (failure || (session_active && (uint32_t)(now_ms - last_valid_ms) >= MANUAL_VELOCITY_TIMEOUT_MS))
    {
        parser = (LinkParser){0};
        Stop_Session(failure ? DETAIL_UART : DETAIL_LINK);
    }
    else
    {
        for (uint16_t index = 0U; index < count; ++index)
        {
            LinkFrame frame;
            const uint32_t received_ms = received->time_ms[index];
            /* 使用中断收字节的时刻，同一批中的长间隔也会撤销半帧。 */
            if (parser.count != 0U && received_ms - parser_activity_ms >= RX_FRAME_GAP_MS)
                parser = (LinkParser){0};
            parser_activity_ms = received_ms;
            if (!LinkParser_Feed(&parser, received->data[index], &frame) ||
                now_ms - received_ms >= MANUAL_VELOCITY_TIMEOUT_MS)
                continue;

            if ((frame.type == UART_STOP && frame.length == 0U) ||
                (frame.type == MANUAL_CAN_VELOCITY_ID && (frame.length != 7U || frame.data[0] != 1U)))
            {
                /* 同批次的停车覆盖前后所有运动请求，半帧也不会留下。 */
                Stop_Session(DETAIL_NONE);
                parser = (LinkParser){0};
                break;
            }
            if (frame.type == UART_ARM || frame.type == UART_ORIGIN || frame.type == UART_RETURN)
                Service_Request(&frame, received_ms);
            else if (session_active && return_state != RETURN_INACTIVE && frame.type == UART_KEEPALIVE &&
                     frame.length == 0U)
                last_valid_ms = received_ms;
            else if (session_active && return_state == RETURN_INACTIVE && frame.type == MANUAL_CAN_VELOCITY_ID)
            {
                Manual_CAN_Receive(frame.data, frame.length);
                last_valid_ms = received_ms;
            }
        }
    }
    if (parser.count != 0U && now_ms - parser_activity_ms >= RX_FRAME_GAP_MS)
        parser = (LinkParser){0};

    if (return_state != RETURN_INACTIVE)
        Service_Return(now_ms);
    else if (origin_valid)
    {
        Chassis_Odometry_t position;
        if (!Chassis_Get_Odometry(&position))
            Stop_Session(DETAIL_ODOM);
    }
    if (return_state == RETURN_INACTIVE)
    {
        Manual_Control_Loop();
        if (session_active)
        {
            if (manual_control_state == MANUAL_BRAKING)
                Stop_Session(DETAIL_CHASSIS);
            else if (manual_control_state != MANUAL_DISARMED)
                was_enabled = 1U;
            else if (was_enabled || (uint32_t)(now_ms - arm_started_ms) >= MANUAL_PREPARE_TIMEOUT_MS)
                Stop_Session(was_enabled ? DETAIL_CHASSIS : DETAIL_TIMEOUT);
        }
    }
    Report_Status(now_ms);
}
