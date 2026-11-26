#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "INS_task.h"
#include "bsp_can.h"
#include "chassis.h"
#include "dm_motor.h"
#include "jetson_auto.h"
#include "jetson_link.h"
#include "motor_chassis.h"
#include "task.h"
#include "tim.h"
#include "usb_link.h"

/* 链接真实自主层、底盘、运动学和 PID；只替换时钟、IMU、CAN 与电机反馈。
 * 通过单趟联调的公开协议进入安全区阶段，不直接改写业务模块的私有状态。
 */
CAN_HandleTypeDef hcan1, hcan2;
TIM_HandleTypeDef htim2;
volatile dm_motor_t dm_motor[3];
uint32_t test_tick;
static uint32_t counter_us, free_mailboxes, nonzero_controls, zero_controls, disable_frames;
static uint8_t motor_state[3], queued_state[3];
static unsigned critical_depth;
static float imu_yaw, imu_yaw_rate;
static uint32_t imu_age_ms;
static uint8_t imu_valid;
static float last_wheels[3], last_command[3];
static double commanded_forward_m;
static double ground_forward_m;
static float straight_heading_deg;
static uint8_t measure_straight;
static uint16_t last_uplink;
static uint32_t last_argument;
static const uint32_t route_token = 0xA13579U;
static UsbLink_State_t usb_state;
static uint8_t periodic_stream;
static uint16_t background_center;

/** @brief USB 连接由场景独立控制，运动控制和制动仍使用真实底盘实现。 */
void UsbLink_Get_State(UsbLink_State_t *state) { *state = usb_state; }

void test_enter_critical(void) { ++critical_depth; }
void test_exit_critical(void) { assert(critical_depth > 0U); --critical_depth; }
void vTaskDelay(TickType_t ticks) { assert(ticks == 1U && critical_depth == 0U); }
uint32_t test_timer_counter(TIM_HandleTypeDef *timer)
{ assert(timer == &htim2); return counter_us; }

uint8_t INS_Get_Data(INS_control_data_t *data)
{
    *data = (INS_control_data_t){.yaw_deg = imu_yaw, .world_z_rad_s = imu_yaw_rate,
                                 .sample_time_ms = test_tick - imu_age_ms};
    return imu_valid;
}

uint32_t HAL_CAN_GetTxMailboxesFreeLevel(CAN_HandleTypeDef *can)
{ assert(can == &hcan1); return free_mailboxes; }

HAL_StatusTypeDef HAL_CAN_AbortTxRequest(CAN_HandleTypeDef *can, uint32_t mailboxes)
{
    assert(can == &hcan1 && mailboxes == (CAN_TX_MAILBOX0 | CAN_TX_MAILBOX1 | CAN_TX_MAILBOX2));
    memcpy(queued_state, motor_state, sizeof(queued_state));
    free_mailboxes = 3U;
    return HAL_OK;
}

static HAL_StatusTypeDef Submit_Motor(CAN_HandleTypeDef *can, uint16_t id)
{
    assert(can == &hcan1 && id >= 1U && id <= 3U && free_mailboxes > 0U);
    --free_mailboxes;
    return HAL_OK;
}

HAL_StatusTypeDef ctrl_motor(CAN_HandleTypeDef *can, uint16_t id, float position,
                            float velocity, float kp, float kd, float torque)
{
    assert(position == 0.0f && kp == 0.0f && kd > 0.0f && isfinite(torque) && fabsf(torque) <= 0.05f);
    assert(isfinite(velocity) && fabsf(velocity) <= 40.0f);
    last_wheels[id - 1U] = -velocity; /* 三轮安装方向均为 -1。 */
    if (id == 3U)
    {
        Motor_Chassis_Forward(last_wheels, last_command);
        if (measure_straight)
            commanded_forward_m += last_command[0] * 0.01;
    }
    if (velocity != 0.0f)
        ++nonzero_controls;
    else
        ++zero_controls;
    return Submit_Motor(can, id);
}

HAL_StatusTypeDef enable_motor(CAN_HandleTypeDef *can, uint16_t id)
{
    const HAL_StatusTypeDef result = Submit_Motor(can, id);
    queued_state[id - 1U] = 1U;
    return result;
}
HAL_StatusTypeDef disable_motor(CAN_HandleTypeDef *can, uint16_t id)
{
    ++disable_frames;
    const HAL_StatusTypeDef result = Submit_Motor(can, id);
    queued_state[id - 1U] = 0U;
    return result;
}

int Jetson_Link_Send(uint8_t type, const uint8_t *payload, uint8_t length)
{
    assert(critical_depth > 0U);
    assert(payload != NULL);
    assert(length == (type == JETSON_CAN_ARRIVAL_ID ? 1U : 5U));
    assert(payload[0] == (type == JETSON_CAN_POINT_REACHED_ID ? 0U : 1U));
    last_uplink = type;
    last_argument = length == 1U ? 0U : (uint32_t)payload[1] |
        ((uint32_t)payload[2] << 8U) | ((uint32_t)payload[3] << 16U) | ((uint32_t)payload[4] << 24U);
    return 0;
}

/* 与固件相同，先服务底盘，再运行自主层；反馈独立于已发送目标。 */
static void Step_Wheels(const float feedback[3])
{
    test_tick += 5U;
    counter_us += 5000U;
    if (periodic_stream && usb_state.connected)
    {
        const uint8_t no_center[8] = {0};
        Jetson_CAN_Receive(background_center, no_center, sizeof(no_center));
    }
    free_mailboxes = 3U;
    memcpy(motor_state, queued_state, sizeof(motor_state));
    for (unsigned i = 0U; i < 3U; ++i)
    {
        dm_motor[i].id = (uint8_t)(i + 1U);
        dm_motor[i].state = motor_state[i];
        dm_motor[i].position = remainderf(dm_motor[i].position + feedback[i] * .005f, 25.0f);
        dm_motor[i].velocity = feedback[i];
        dm_motor[i].feedback_time_ms = test_tick;
        ++dm_motor[i].feedback_seq;
    }
    Chassis_Loop();
    Jetson_Auto_Loop();
    assert(critical_depth == 0U);
}

/** @brief 指定三轮同速供停车、故障用例使用，不冒充平移反馈。 */
static void Step(float feedback_speed)
{
    const float feedback[3] = {feedback_speed,feedback_speed,feedback_speed};
    Step_Wheels(feedback);
}

/* 以同一条两端静止的转动轨迹生成 IMU 航向、瞬时角速度和各轮区间位移。 */
static void Rotate_Feedback_To(float end_deg, unsigned steps)
{
    const double start_deg = imu_yaw;
    const double delta_deg = end_deg - start_deg;
    const double deg_to_rad = 0.017453292519943295;
    for (unsigned i = 1U; i <= steps; ++i)
    {
        const double t = (double)i / steps;
        const float yaw = (float)(start_deg + delta_deg * t * t * (3.0 - 2.0 * t));
        const float mean_rate = (float)((yaw - imu_yaw) * deg_to_rad / 0.005);
        imu_yaw_rate = (float)(delta_deg * deg_to_rad * 6.0 * t * (1.0 - t) / (steps * 0.005));
        imu_yaw = yaw;
        Step(-mean_rate * CHASSIS_ROTATION_RADIUS / CHASSIS_WHEEL_RADIUS);
    }
}

/** @brief 上一拍目标按给定比例实际跟随，独立累计启动航向上的地面位移。 */
static void Step_Follow(float gain)
{
    float feedback[3];
    const double relative = (imu_yaw - straight_heading_deg) * 0.017453292519943295;
    if (measure_straight)
        ground_forward_m += gain * (last_command[0] * cos(relative) -
                                   last_command[1] * sin(relative)) * 0.005;
    for (unsigned i = 0U; i < 3U; ++i)
        feedback[i] = -gain * last_wheels[i];
    Step_Wheels(feedback);
}

static void Receive_Center(uint16_t type, int16_t x, int16_t y)
{
    background_center = type == JETSON_CAN_TARGET_CENTER_ID ?
        JETSON_CAN_SAFETY_CENTER_ID : JETSON_CAN_TARGET_CENTER_ID;
    const uint8_t center[8] = {1U, (uint8_t)x, (uint8_t)((uint16_t)x >> 8U),
        (uint8_t)y, (uint8_t)((uint16_t)y >> 8U), (uint8_t)test_tick,
        (uint8_t)(test_tick >> 8U), (uint8_t)(test_tick >> 16U)};
    Jetson_CAN_Receive(type, center, sizeof(center));
}

static void Receive_Command_Ack(uint8_t command)
{
    const uint8_t reply[6] = {1U, command, (uint8_t)route_token,
        (uint8_t)(route_token >> 8U), (uint8_t)(route_token >> 16U),
        (uint8_t)(route_token >> 24U)};
    Jetson_CAN_Receive(JETSON_CAN_COMMAND_REPLY_ID, reply, sizeof(reply));
}

static void Arrive(uint16_t center_type, uint16_t uplink_type)
{
    for (unsigned i = 0U; i < 2000U && last_uplink != uplink_type; ++i)
    {
        Receive_Center(center_type, 5000, 7500);
        if (center_type == JETSON_CAN_SAFETY_CENTER_ID)
            Step_Follow(0.8f);
        else
            Step(0.0f);
    }
    assert(last_uplink == uplink_type && Chassis_Motion_Ready());
}

static void Start_Safety_Approach(void)
{
    test_tick = 100U;
    counter_us = test_tick * 1000U;
    free_mailboxes = 3U;
    nonzero_controls = critical_depth = 0U;
    zero_controls = disable_frames = 0U;
    memset(motor_state, 0, sizeof(motor_state));
    memset(queued_state, 0, sizeof(queued_state));
    imu_yaw = 0.0f;
    imu_yaw_rate = 0.0f;
    imu_age_ms = 0U;
    imu_valid = 1U;
    memset(last_wheels, 0, sizeof(last_wheels));
    memset(last_command, 0, sizeof(last_command));
    commanded_forward_m = 0.0;
    ground_forward_m = 0.0;
    straight_heading_deg = 0.0f;
    measure_straight = 0U;
    last_uplink = 0U;
    last_argument = 0U;
    usb_state = (UsbLink_State_t){.connected = 1U, .generation = 1U};
    periodic_stream = 1U;
    background_center = JETSON_CAN_SAFETY_CENTER_ID;
    memset((void *)dm_motor, 0, sizeof(dm_motor));
    Chassis_Init();
    Jetson_CAN_Init();
    Jetson_Auto_Init();
    jetson_test_start = 1U;
    Step(0.0f);
    assert(last_uplink == JETSON_CAN_POINT_REACHED_ID && last_argument != 0U);
    uint8_t action[6] = {1U, 5U};
    for (unsigned i = 0U; i < 4U; ++i)
        action[i + 2U] = (uint8_t)(last_argument >> (i * 8U));
    Jetson_CAN_Receive(JETSON_CAN_COMMAND_REPLY_ID, action, sizeof(action));
    action[0] = 0U;
    action[1] = 1U;
    Jetson_CAN_Receive(JETSON_CAN_ACTION_RESULT_ID, action, sizeof(action));
    for (unsigned i = 0U; i < 20U; ++i)
        Step(0.0f);
    assert(Chassis_Is_Enabled() && Chassis_Is_Stopped());
    Arrive(JETSON_CAN_TARGET_CENTER_ID, JETSON_CAN_ARRIVAL_ID);

    const uint8_t accepted = 1U;
    const uint8_t roi[3] = {1U, 1U, 1U};
    const uint8_t turn[6] = {1U, 1U, (uint8_t)route_token,
        (uint8_t)(route_token >> 8U), (uint8_t)(route_token >> 16U),
        (uint8_t)(route_token >> 24U)};
    Jetson_CAN_Receive(JETSON_CAN_ARRIVAL_REPLY_ID, &accepted, 1U);
    Jetson_CAN_Receive(JETSON_CAN_ROI_REPLY_ID, roi, sizeof(roi));
    Jetson_CAN_Receive(JETSON_CAN_TURN_REQUEST_ID, turn, sizeof(turn));
    Step(0.0f);
    Rotate_Feedback_To(180.0f, 450U);
    for (unsigned i = 0U; i < 200U && last_uplink != JETSON_CAN_TURN_COMPLETED_ID; ++i)
        Step(0.0f);
    assert(last_uplink == JETSON_CAN_TURN_COMPLETED_ID && last_argument == route_token);
    Receive_Command_Ack(1U);
    Step(0.0f);
}

static void Safety_Edge_Frame(unsigned axis, int16_t offset)
{
    Receive_Center(JETSON_CAN_SAFETY_CENTER_ID,
                   (int16_t)(5000 + (axis == 0U ? offset : 0)),
                   (int16_t)(7500 + (axis == 1U ? offset : 0)));
    Step(1.0f);
}

static void Test_Safety_Edge_Recovery(unsigned axis)
{
    Start_Safety_Approach();
    Receive_Center(JETSON_CAN_SAFETY_CENTER_ID,
                   (int16_t)(5000 + (axis == 0U ? 200 : 0)),
                   (int16_t)(7500 + (axis == 1U ? 200 : 0)));
    Step(0.0f);
    for (unsigned i = 0U; i < 4U; ++i)
        Step(1.0f);
    assert(nonzero_controls > 0U && !Chassis_Is_Stopped());

    /* 进入外半窗只一帧，随后抖回窗外，三轮此时仍在运动。 */
    Safety_Edge_Frame(axis, 90);
    Step(1.0f);
    const uint32_t before_recovery = nonzero_controls;
    Safety_Edge_Frame(axis, 101);
    for (unsigned i = 0U; i < 8U; ++i)
        Step(1.0f);
    assert(Chassis_Is_Enabled() && !Chassis_Is_Stopped());
    /* 车轮持续报告非零速度，也能恢复窗外所需的微调。 */
    assert(nonzero_controls > before_recovery);

    /* 能继续完成安全区双帧核验和上行握手，证明未锁入 FAULT。 */
    Arrive(JETSON_CAN_SAFETY_CENTER_ID, JETSON_CAN_SAFETY_ARRIVED_ID);
    assert(last_argument == route_token);
    Receive_Command_Ack(2U);
    Step(0.0f);
    assert(!Chassis_Is_Enabled());
}

/** @brief 经公开流程和视觉双帧核验进入定距，起步航向故意不同于原转向终点。 */
static void Start_Safety_Straight(void)
{
    Start_Safety_Approach();
    Rotate_Feedback_To(176.0f, 80U);
    for (unsigned i = 0U; i < 200U && Chassis_Motion_Ready(); ++i)
    {
        Receive_Center(JETSON_CAN_SAFETY_CENTER_ID, 5000, 7500);
        Step(0.0f);
    }
    assert(Chassis_Is_Enabled() && !Chassis_Is_Stopped());
    assert(last_uplink == JETSON_CAN_TURN_COMPLETED_ID);
    commanded_forward_m = 0.0;
    ground_forward_m = 0.0;
    straight_heading_deg = imu_yaw;
    measure_straight = 1U;
}

/** @brief 快慢跟随均由反馈位移决定到达，定距中不消费后续视觉。 */
static void Test_Safety_Straight_Completion(float gain)
{
    Start_Safety_Straight();
    Step_Follow(gain);
    Step_Follow(gain);
    assert(last_command[0] > 0.0f && fabsf(last_command[1]) < 1e-6f);
    assert(fabsf(last_command[2]) < 1e-6f); /* 锁定176度而非旧转向的180度。 */
    imu_yaw = 181.0f;
    Step_Follow(gain);
    Step_Follow(gain);
    assert(last_command[2] < 0.0f);
    imu_yaw = 176.0f;
    unsigned frame;
    for (frame = 0U; frame < 2400U; ++frame)
    {
        if (frame < 20U)
            Receive_Center(JETSON_CAN_SAFETY_CENTER_ID,1000,1000);
        else if (frame == 20U)
        {
            const uint8_t invalid[8] = {0};
            Jetson_CAN_Receive(JETSON_CAN_SAFETY_CENTER_ID,invalid,sizeof(invalid));
        }
        Step_Follow(gain);
        assert(last_uplink == JETSON_CAN_TURN_COMPLETED_ID);
        assert(Chassis_Is_Enabled());
        if (last_command[0] == 0.0f && last_command[1] == 0.0f && last_command[2] == 0.0f)
            break;
    }
    assert(frame < 2400U);
    assert(fabs(ground_forward_m - JETSON_AUTO_SAFETY_STRAIGHT_M) <=
           CHASSIS_DISTANCE_TOLERANCE_M + 0.0003);
    if (gain < 1.0f)
        assert(commanded_forward_m > JETSON_AUTO_SAFETY_STRAIGHT_M);
    /* 维持非零的三轮反馈；零目标提交后两拍内到达握手已发出。 */
    Step(1.0f);
    Step(1.0f);
    assert(last_uplink == JETSON_CAN_SAFETY_ARRIVED_ID && last_argument == route_token);
    assert(!Chassis_Is_Stopped());
    Receive_Command_Ack(2U);
    Step(0.0f);
    assert(!Chassis_Is_Enabled());
}

/** @brief 轮速停滞、反向或只有侧移不能触发安全区到达，上层最终进入故障失能。 */
static void Test_Safety_Straight_No_Progress(void)
{
    for (unsigned scenario = 0U; scenario < 3U; ++scenario)
    {
        Start_Safety_Straight();
        float feedback[3] = {0.0f,0.0f,0.0f};
        if (scenario > 0U)
        {
            Motor_Chassis_Inverse(scenario == 1U ? -0.05f : 0.0f,
                                  scenario == 2U ? 0.05f : 0.0f,0.0f,feedback);
            for (unsigned i = 0U; i < 3U; ++i) feedback[i] = -feedback[i];
        }
        for (unsigned frame = 0U; frame < 520U; ++frame)
        {
            Step_Wheels(feedback);
            assert(last_uplink == JETSON_CAN_TURN_COMPLETED_ID);
            assert(!Chassis_Is_Stopped());
        }
        assert(!Chassis_Is_Enabled() && disable_frames == 3U);
    }
}

/** @brief IMU 无效或超龄触发的是故障制动，不能在失能前一拍伪装为定距完成。 */
static void Test_Safety_Straight_IMU_Fault(void)
{
    for (unsigned stale = 0U; stale < 2U; ++stale)
    {
        Start_Safety_Straight();
        for (unsigned i = 0U; i < 10U; ++i)
            Step_Follow(0.8f);
        assert(nonzero_controls > 0U);
        if (stale)
            imu_age_ms = 21U;
        else
            imu_valid = 0U;
        for (unsigned i = 0U; i < 12U; ++i)
        {
            Step(1.0f);
            assert(last_uplink == JETSON_CAN_TURN_COMPLETED_ID);
        }
        const uint32_t after_fault = nonzero_controls;
        for (unsigned i = 0U; i < 20U; ++i)
        {
            Step(0.0f);
            assert(last_uplink == JETSON_CAN_TURN_COMPLETED_ID);
        }
        assert(!Chassis_Is_Enabled() && disable_frames == 3U);
        assert(nonzero_controls == after_fault && commanded_forward_m < JETSON_AUTO_SAFETY_STRAIGHT_M);
    }
}

/** @brief 实际底盘验证拔线和拍间重连会发送零速，三轮新反馈停稳后再提交失能。 */
static void Test_USB_Loss_Brakes_Real_Chassis(void)
{
    for (unsigned quick_reconnect = 0U; quick_reconnect < 2U; ++quick_reconnect)
    {
        Start_Safety_Approach();
        Receive_Center(JETSON_CAN_SAFETY_CENTER_ID, 5300, 7000);
        Step(0.0f);
        for (unsigned i = 0U; i < 8U; ++i)
            Step(1.0f);
        assert(Chassis_Is_Enabled() && nonzero_controls > 0U);

        usb_state.connected = 0U;
        ++usb_state.generation;
        if (quick_reconnect)
        {
            usb_state.connected = 1U;
            ++usb_state.generation;
        }
        Step(1.0f);
        const uint32_t after_loss = nonzero_controls;
        const uint32_t before_zero = zero_controls;
        for (unsigned i = 0U; i < 8U; ++i)
            Step(1.0f);
        assert(Chassis_Is_Enabled() && nonzero_controls == after_loss && zero_controls > before_zero);
        assert(disable_frames == 0U);

        for (unsigned i = 0U; i < 20U; ++i)
            Step(0.0f);
        assert(!Chassis_Is_Enabled() && disable_frames == 3U);
        usb_state.connected = 1U;
        ++usb_state.generation;
        for (unsigned i = 0U; i < 80U; ++i)
        {
            Receive_Center(JETSON_CAN_SAFETY_CENTER_ID, 5300, 7000);
            Step(0.0f);
        }
        assert(!Chassis_Is_Enabled() && nonzero_controls == after_loss && disable_frames == 3U);
    }
}

int main(void)
{
    Test_Safety_Edge_Recovery(0U);
    Test_Safety_Edge_Recovery(1U);
    Test_USB_Loss_Brakes_Real_Chassis();
    Test_Safety_Straight_Completion(0.55f);
    Test_Safety_Straight_Completion(0.8f);
    Test_Safety_Straight_No_Progress();
    Test_Safety_Straight_IMU_Fault();
    puts("PASS: real auto/chassis safety-edge recovery, wheel-odometry distance/yaw, stall rejection, USB/IMU braking and stop feedback gate");
    return 0;
}
