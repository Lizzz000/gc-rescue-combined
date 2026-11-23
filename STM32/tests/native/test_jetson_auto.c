#include <assert.h>
#include <math.h>
#include <stdio.h>
#include "INS_task.h"
#include "usb_link.h"
#include "chassis.h"
#include "jetson_auto.h"
#include "jetson_link.h"
#include "task.h"

/* 真实协议编解码与自主状态机，底盘反馈及 USB 发送结果由本测试控制。 */
uint32_t test_tick;
/* Jetson 源帧时钟与 STM32 的接收时钟独立。 */
static uint32_t source_time_ms;
static unsigned critical_depth;
static uint8_t can_arrival_before_snapshot;
static uint8_t enabled, stopped, motion_ready, can_enable, enable_tx_failure;
static uint8_t delay_hold_zero;
static uint8_t encoder_stop_confirmed;
static unsigned enables, stops, hard_stops, disables, velocity_requests, timed_velocity_requests;
static unsigned disable_feedback_checks;
static uint8_t disabled_wheel_mask;
static uint8_t tx_ready, tx_error;
static unsigned distance_count, fast_distance_count, turn_count, sent_count;
static unsigned total_turns;
static int8_t last_turn_direction;
static float last_turn_angle, last_vx, last_vy, last_wz;
static uint8_t motors_ready;
static Chassis_Odometry_t odom;
static unsigned return_commands;
#ifdef TEST_JETSON_AUTO_CONFIGURED
static Chassis_Odometry_t expected_origin;
static float expected_return_yaw_rad;
#endif
static uint32_t startup_id, initial_align_id, initial_search_id;
static unsigned startup_queries;
static uint32_t imu_age_ms;
static UsbLink_State_t usb_state;
static uint8_t periodic_stream;
static uint16_t background_center;

/** @brief 提供独立于周期图像报文的 USB 枚举状态和连接代次。 */
void UsbLink_Get_State(UsbLink_State_t *state) { *state = usb_state; }

/** @brief 用另一路无目标坐标维持真实周期流，不刷新场景正在核验的视觉样本。 */
static void Receive_Background_Periodic(void)
{
    if (periodic_stream && usb_state.connected)
    {
        const uint8_t no_center[8] = {0};
        Jetson_CAN_Receive(background_center, no_center, sizeof(no_center));
    }
}

/** @brief 推进自主层，既有视觉场景默认保留周期通信，可单独关闭以测试断流。 */
static void Run_Auto(void)
{
    Receive_Background_Periodic();
    Jetson_Auto_Loop();
}

typedef struct
{
    uint16_t id;
    uint32_t token;
    uint32_t request_id;
    uint32_t source_frame;
} Sent_Frame;

static Sent_Frame sent[64];

/** @brief 模拟接收快照的临界区入口，可在加锁前插入跨 tick 的 CAN 中断。 */
void test_enter_critical(void)
{
    if (can_arrival_before_snapshot)
    {
        assert(critical_depth == 0U);
        can_arrival_before_snapshot = 0U;
        ++test_tick;
        ++source_time_ms;
        const uint8_t target[8] = {1U, 0x88U, 0x13U, 0xA0U, 0x0FU, (uint8_t)source_time_ms, (uint8_t)(source_time_ms >> 8U), (uint8_t)(source_time_ms >> 16U)}; /* (5000,4000) */
        Jetson_CAN_Receive(JETSON_CAN_TARGET_CENTER_ID, target, sizeof(target));
    }
    ++critical_depth;
}
/** @brief 验证临界区配对退出。 */
void test_exit_critical(void) { assert(critical_depth > 0U); --critical_depth; }

/** @brief 模拟静止车辆受理使能，后续运动的停车反馈由场景单独提供。 @return 1。 */
uint8_t Chassis_Enable(void)
{
    ++enables;
    if (enable_tx_failure)
        return 0U;
    enabled = motion_ready = stopped = 1U;
    return 1U;
}

/** @brief 模拟请求失能及撤销停车反馈。 */
void Chassis_Disable(void)
{
    enabled = stopped = motion_ready = 0U;
    ++disables;
}

uint8_t Chassis_Disable_Confirmed(void)
{
    ++disable_feedback_checks;
    return !enabled && disabled_wheel_mask == 7U;
}

/** @brief 记录停车请求；请求本身不生成停车反馈。 */
void Chassis_Stop(void) { ++stops; ++hard_stops; motion_ready = 1U; }
void Chassis_Hold(void) { ++stops; motion_ready = !delay_hold_zero; last_vx = last_vy = last_wz = 0.0f; }
uint8_t Chassis_Attitude_Ready(void) { return imu_age_ms <= 20U; }
uint8_t Chassis_Can_Enable(void) { return can_enable; }
/** @brief 读取模拟底盘使能状态。 @return 软件已使能为 1。 */
uint8_t Chassis_Is_Enabled(void) { return enabled; }
/** @brief 读取场景独立给出的停车反馈。 @return 已使能且确认停车为 1。 */
uint8_t Chassis_Is_Stopped(void) { return enabled && stopped; }
uint8_t Chassis_Encoder_Stop_Confirmed(uint8_t begin)
{
    assert(enabled && motion_ready && begin <= 1U);
    return encoder_stop_confirmed;
}
uint8_t Chassis_Motion_Ready(void) { return enabled && motors_ready && motion_ready; }

/** @brief 提供可单独断开的三轮反馈条件。 @return 反馈正常为 1。 */
uint8_t Chassis_Motors_Ready(void) { return motors_ready; }

/* 测试分别驱动位置和停稳反馈，避免把已发送的速度误当成实测位移。 */
uint8_t Chassis_Get_Odometry(Chassis_Odometry_t *data)
{
    *data = odom;
    return enabled && odom.valid && Chassis_Attitude_Ready();
}

/** @brief 提供可独立设置样本年龄的姿态。 @param[out] data 姿态快照。 @return 有效为 1。 */
uint8_t INS_Get_Data(INS_control_data_t *data)
{
    *data = (INS_control_data_t){0};
    data->sample_time_ms = test_tick - imu_age_ms;
    return 1U;
}

/* 测试记录名义车体速度，停车资格仍由场景提供。 */
static uint8_t Record_Velocity(float vx, float vy, float wz)
{
    assert(enabled && isfinite(vx) && isfinite(vy) && isfinite(wz));
    ++velocity_requests;
    last_vx = vx;
    last_vy = vy;
    last_wz = wz;
    motion_ready = vx == 0.0f && vy == 0.0f && wz == 0.0f;
    if (!motion_ready)
        stopped = 0U;
    return 1U;
}

uint8_t Chassis_Set_Velocity(float vx, float vy, float wz)
{
    assert(wz == 0.0f);
    if (vy != 0.0f)
        assert(vx == 0.0f && fabsf(vy) == JETSON_AUTO_LATERAL_SPEED_MPS);
    else
        assert(vx == JETSON_AUTO_PUSH_SPEED_MPS || vx == -JETSON_AUTO_RETRACT_SPEED_MPS);
    ++timed_velocity_requests;
    return Record_Velocity(vx, vy, wz);
}

uint8_t Chassis_Set_Navigation_Velocity(float x, float y, float wz, uint8_t world_frame)
{
    assert(world_frame <= 1U);
    if (!world_frame)
        return Record_Velocity(x, y, wz);

    const float cosine = cosf(odom.yaw_rad), sine = sinf(odom.yaw_rad);
    const float vx = cosine * x + sine * y;
    const float vy = -sine * x + cosine * y;
    ++return_commands;
    return Record_Velocity(vx, vy, wz);
}

/** @brief 捕获定距请求，清除先前停车反馈。 @return 模拟受理为 1。
 * @param[in] direction 平移方向。 @param[in] distance 规划米数。 @param[in] speed 车体速度上限 m/s。
 */
uint8_t Chassis_Distance_Start(Chassis_Translation_Direction_t direction, float distance, float speed)
{
    assert(direction == CHASSIS_TRANSLATION_BACKWARD);
    assert(Chassis_Motion_Ready() && distance > 0.0f);
    assert(speed == JETSON_AUTO_DISTANCE_SPEED_MPS);
    ++distance_count;
    stopped = motion_ready = 0U;
    return 1U;
}

uint8_t Chassis_Distance_Start_Fast(Chassis_Translation_Direction_t direction, float distance, float speed)
{
    (void)direction;
    (void)distance;
    (void)speed;
    ++fast_distance_count;
    return 0U;
}

/** @brief 捕获定角请求，清除先前停车反馈。 @return 模拟受理为 1。
 * @param[in] direction 左 +1、右 -1。 @param[in] angle_deg 相对角度，度。
 */
uint8_t Chassis_Turn_Start(int8_t direction, float angle_deg)
{
    assert(Chassis_Motion_Ready() && (direction == -1 || direction == 1) && angle_deg > 0.0f);
    last_turn_direction = direction;
    last_turn_angle = angle_deg;
    ++turn_count;
    ++total_turns;
    stopped = motion_ready = 0U;
    return 1U;
}

/** @brief 捕获真实编码后的 USB 上行载荷。 @return 0 已提交，1 链路忙，2 发送错误。
 * @param[in] id 运输协议上行类型。 @param[in] payload 编码载荷。 @param[in] len 载荷字节数。
 */
int Jetson_Link_Send(uint8_t id, const uint8_t *payload, uint8_t len)
{
    assert(critical_depth > 0U);
    assert(payload != NULL);
    const uint8_t grasp = id == JETSON_CAN_POINT_REACHED_ID && payload[0] == JETSON_CAN_POINT_GRASP;
    assert(len == (grasp ? 8U : id == JETSON_CAN_ARRIVAL_ID ? 1U : 5U));
    if (id == JETSON_CAN_POINT_REACHED_ID)
        assert(payload[0] <= JETSON_CAN_POINT_RELEASE);
    else if (id == JETSON_CAN_ARRIVAL_ID)
        assert(payload[0] == JETSON_CAN_SELECTED_ROUTE);
    else
        assert(payload[0] == 1U);
    if (tx_error)
        return 2;
    if (!tx_ready)
        return 1;
    assert(sent_count < sizeof(sent) / sizeof(sent[0]));

    const uint32_t encoded = len == 1U ? 0U : (uint32_t)payload[1] |
        (uint32_t)payload[2] << 8U | (uint32_t)payload[3] << 16U | (uint32_t)payload[4] << 24U;
    if (id == JETSON_CAN_POINT_REACHED_ID && payload[0] == JETSON_CAN_POINT_STARTUP)
    {
        assert(encoded != 0U);
        if (startup_id != 0U)
            assert(startup_id == encoded);
        startup_id = encoded;
        ++startup_queries;
    }
    else if (id == JETSON_CAN_START_ALIGN_ID)
    {
        assert(encoded != 0U && (!initial_align_id || encoded == initial_align_id));
        initial_align_id = encoded;
    }
    else if (id == JETSON_CAN_START_SEARCH_ID &&
             ((turn_count == 1U && initial_search_id == 0U) || encoded == initial_search_id))
    {
        assert(encoded != 0U && initial_align_id != 0U && stopped);
        initial_search_id = encoded;
    }
    else
        sent[sent_count++] = (Sent_Frame){id, id == JETSON_CAN_POINT_REACHED_ID ? payload[0] : encoded,
                                        id == JETSON_CAN_POINT_REACHED_ID ? encoded : 0U,
                                        grasp ? (uint32_t)payload[5] | ((uint32_t)payload[6] << 8U) |
                                                ((uint32_t)payload[7] << 16U) : 0U};
    return 0;
}

/** @brief 初始化独立场景，启用替身外设能力，底盘保持失能。 */
static void Reset_At(uint32_t initial_ms)
{
    test_tick = initial_ms;
    source_time_ms = 0x00ffff00U; /* 接近协议的 24 位源时钟回绕。 */
    critical_depth = 0U;
    can_arrival_before_snapshot = 0U;
    enabled = stopped = motion_ready = 0U;
    delay_hold_zero = 0U;
    encoder_stop_confirmed = 0U;
    can_enable = 1U;
    enable_tx_failure = 0U;
    enables = stops = hard_stops = disables = velocity_requests = timed_velocity_requests = 0U;
    disabled_wheel_mask = 0U;
    total_turns = 0U;
    disable_feedback_checks = 0U;
    distance_count = fast_distance_count = turn_count = sent_count = return_commands = 0U;
    odom = (Chassis_Odometry_t){.valid = 1U};
#ifdef TEST_JETSON_AUTO_CONFIGURED
    expected_origin = odom;
    expected_return_yaw_rad = odom.yaw_rad;
#endif
    imu_age_ms = startup_id = startup_queries = initial_align_id = initial_search_id = 0U;
    tx_ready = motors_ready = 1U;
    tx_error = 0U;
    last_turn_direction = 0;
    last_turn_angle = last_vx = last_vy = last_wz = 0.0f;
    usb_state = (UsbLink_State_t){.connected = 1U, .generation = 1U};
    periodic_stream = 1U;
    background_center = JETSON_CAN_SAFETY_CENTER_ID;
    Jetson_CAN_Init();
    Jetson_Auto_Init();
}

static void Reset(void) { Reset_At(100U); }

/** @brief 推进一拍；背景只发既有类型 3/4 坐标帧，不引入心跳类型。 */
static void Step(void)
{
    test_tick += 5U;
    source_time_ms += 5U;
    Run_Auto();
    assert(critical_depth == 0U);
}

static void Steps(unsigned count)
{
    for (unsigned i = 0U; i < count; ++i)
        Step();
}

/** @brief 注入有效中心。 @param[in] id 目标或安全区 ID。
 * @param[in] x 横向归一化值乘 10000。 @param[in] y 纵向归一化值乘 10000。
 */
static void Receive_Center_With_Times(uint16_t id, int16_t x, int16_t y, uint32_t source, uint32_t received_ms)
{
    background_center = id == JETSON_CAN_TARGET_CENTER_ID ?
        JETSON_CAN_SAFETY_CENTER_ID : JETSON_CAN_TARGET_CENTER_ID;
    const uint8_t bytes[] = {1U, (uint8_t)x, (uint8_t)((uint16_t)x >> 8U),
                            (uint8_t)y, (uint8_t)((uint16_t)y >> 8U),
                            (uint8_t)source, (uint8_t)(source >> 8U), (uint8_t)(source >> 16U)};
    Jetson_CAN_Receive_At(id, bytes, sizeof(bytes), received_ms);
}

static void Receive_Center_At(uint16_t id, int16_t x, int16_t y, uint32_t source)
{
    Receive_Center_With_Times(id, x, y, source, test_tick);
}

static void Receive_Center(uint16_t id, int16_t x, int16_t y)
{
    Receive_Center_At(id, x, y, source_time_ms);
}

#ifdef TEST_JETSON_AUTO_CONFIGURED

/** @brief 把无符号身份写为小端。 @param[out] bytes 四字节输出。 @param[in] token 本趟身份。 */
static void Put_Token(uint8_t bytes[4], uint32_t token)
{
    for (unsigned i = 0U; i < 4U; ++i)
        bytes[i] = (uint8_t)(token >> (8U * i));
}

/** @brief 注入到达请求回执。 @param[in] accepted 1 为受理。 */
static void Receive_Arrival_Ack(uint8_t accepted)
{
    Jetson_CAN_Receive(JETSON_CAN_ARRIVAL_REPLY_ID, &accepted, 1U);
}

/** @brief 注入目的区登记结果。 @param[in] passed 1 为通过。 @param[in] destination 左 1、右 2。 */
static void Receive_Route(uint8_t passed, uint8_t destination)
{
    const uint8_t bytes[] = {1U, passed, destination};
    Jetson_CAN_Receive(JETSON_CAN_ROI_REPLY_ID, bytes, sizeof(bytes));
}

/** @brief 注入本趟转向授权。 @param[in] destination 左 1、右 2。 @param[in] token 本趟身份。 */
static void Receive_Turn(uint8_t destination, uint32_t token)
{
    uint8_t bytes[] = {1U, destination, 0U, 0U, 0U, 0U};
    Put_Token(bytes + 2, token);
    Jetson_CAN_Receive(JETSON_CAN_TURN_REQUEST_ID, bytes, sizeof(bytes));
}

/** @brief 注入运输命令回执。
 * @param[in] accepted 1 为受理。 @param[in] command 回执命令编号。 @param[in] token 对应身份。
 */
static void Receive_Command_Ack(uint8_t accepted, uint8_t command, uint32_t token)
{
    uint8_t bytes[] = {accepted, command, 0U, 0U, 0U, 0U};
    Put_Token(bytes + 2, token);
    Jetson_CAN_Receive(JETSON_CAN_COMMAND_REPLY_ID, bytes, sizeof(bytes));
}


static void Receive_Action_Result(uint8_t kind, uint8_t success, uint32_t id)
{
    uint8_t bytes[6] = {kind, success};
    Put_Token(bytes + 2, id);
    Jetson_CAN_Receive(JETSON_CAN_ACTION_RESULT_ID, bytes, sizeof(bytes));
}

static void Complete_Last_Action(void)
{
    assert(sent_count && sent[sent_count - 1U].id == JETSON_CAN_POINT_REACHED_ID);
    const Sent_Frame action = sent[sent_count - 1U];
    assert(action.request_id != 0U);
    Receive_Command_Ack(1U, 5U, action.request_id);
    Receive_Action_Result((uint8_t)action.token, 1U, action.request_id);
}

/** @brief 核对从真实 USB 载荷解出的上行类型及身份。
 * @param[in] index 发送顺序索引。 @param[in] id 预期类型。 @param[in] token 预期本趟身份。
 */
static void Check_Sent(unsigned index, uint16_t id, uint32_t token)
{
    assert(index < sent_count && sent[index].id == id && sent[index].token == token);
}

/** @brief 给状态转换至多 100 ms。 @param[in] counter 只由测试替身更新的计数。 @param[in] count 最低值。 */
static void Wait_For_Count_At(const unsigned *counter, unsigned count, unsigned line)
{
    for (unsigned i = 0U; *counter < count && i < 20U; ++i)
        Step();
    if (*counter < count)
        fprintf(stderr, "line %u: waiting for count %u, got %u\n", line, count, *counter);
    assert(*counter >= count);
}
#define Wait_For_Count(counter, count) Wait_For_Count_At((counter), (count), __LINE__)

/** @brief 等待本机就绪，夹爪成功前只允许零速。 */
static void Start_Automatic(void)
{
    const uint8_t ignored[8] = {1U};
    Jetson_CAN_Receive(1U, ignored, sizeof(ignored));
    Jetson_CAN_Receive(2U, ignored, 7U); /* 协议类型 2 为手动速度，自动固件应忽略。 */
    imu_age_ms = 21U;
    Steps(3U);
    assert(!enabled && !enables && !startup_id);
    imu_age_ms = 0U;
    odom.valid = 0U;
    Wait_For_Count(&enables, 1U);
    Steps(3U);
    assert(enabled && !startup_id && !turn_count && !velocity_requests);
    odom = expected_origin;
    Step();
    assert(startup_id && startup_queries && enables == 1U);
    Receive_Command_Ack(1U, 5U, startup_id);
    Step();
    assert(enabled && !turn_count && !velocity_requests);
    Receive_Action_Result(0U, 1U, startup_id);
}

static void Acknowledge_Initial_Alignment(void)
{
    for (unsigned i = 0U; !initial_align_id && i < 5U; ++i)
        Step();
    assert(initial_align_id && initial_align_id != startup_id && !turn_count);
    Receive_Command_Ack(1U, JETSON_CAN_START_ALIGN_COMMAND, initial_align_id);
    Step();
}

static void Complete_Search_Turn(void)
{
    odom.yaw_rad = remainderf(odom.yaw_rad + last_turn_direction * 3.141592654f, 6.283185307f);
    stopped = motion_ready = 1U;
}

/* 使能就绪后保存全区中心起点，转满半圈才接受新目标。 */
static void Begin_Target_Test_At_Heading(uint32_t initial_ms, float yaw_rad)
{
    Reset_At(initial_ms);
    expected_origin.yaw_rad = yaw_rad;
    expected_return_yaw_rad = yaw_rad;
    Start_Automatic();
    Acknowledge_Initial_Alignment();
    for (unsigned i = 0U; i < 3U; ++i)
    {
        Receive_Center(JETSON_CAN_SAFETY_CENTER_ID, (int16_t)roundf(JETSON_AUTO_LOCALIZE_X * 10000.0f),
                       (int16_t)roundf(JETSON_AUTO_LOCALIZE_Y * 10000.0f));
        Step();
    }
    Wait_For_Count(&turn_count, 1U);
    assert(last_turn_direction == 1 && last_turn_angle == 180.0f);
    Receive_Center(JETSON_CAN_TARGET_CENTER_ID, 5000, 4000);
    Steps(3U);
    assert(!velocity_requests && !sent_count);
    motion_ready = 1U;
    Receive_Center(JETSON_CAN_TARGET_CENTER_ID, 5000, 4100);
    Steps(3U);
    assert(!velocity_requests && !sent_count); /* 仅提交零速还不能开新目标阶段。 */
    Complete_Search_Turn();
    Steps(4U);
    assert(initial_search_id && initial_search_id != initial_align_id);
    Receive_Center(JETSON_CAN_TARGET_CENTER_ID, 5000, 4000);
    Step();
    assert(!velocity_requests);
    Receive_Command_Ack(1U, 3U, initial_search_id);
    Step();
    assert(turn_count == 1U && !velocity_requests && !sent_count && enabled);
}

static void Begin_Target_Test_At(uint32_t initial_ms) { Begin_Target_Test_At_Heading(initial_ms, 0.0f); }
static void Begin_Target_Test(void) { Begin_Target_Test_At(100U); }

/* 正常无目标周期帧保持会话，但底盘不持续原地旋转。 */
static void Test_Wait_After_Search_Turn(void)
{
    Begin_Target_Test();
    const uint8_t no_target[8] = {0};
    for (unsigned i = 0U; i < 160U; ++i)
    {
        Jetson_CAN_Receive(JETSON_CAN_TARGET_CENTER_ID, no_target, sizeof(no_target));
        Step();
    }
    assert(enabled && stopped && turn_count == 1U && !velocity_requests && !sent_count);

    /* 较新源号在转完前入队、转完后才解析，也不能启动搜索接近。 */
    const uint32_t before_turn = test_tick - 4U;
    Begin_Target_Test_At(before_turn);
    Receive_Center_With_Times(JETSON_CAN_TARGET_CENTER_ID, 5000, 4000, source_time_ms, before_turn);
    Step();
    assert(!velocity_requests && !sent_count);
    Receive_Center(JETSON_CAN_TARGET_CENTER_ID, 5000, 4000);
    Wait_For_Count(&velocity_requests, 1U);
}

/** @brief 在 CAN 快照前跨一个 RTOS tick，包含计数回绕，保持健康链路和追踪。 */
static void Test_CAN_Snapshot_Tick_Order(void)
{
    uint32_t startup_ms = 0U;
    for (unsigned wrap = 0U; wrap < 2U; ++wrap)
    {
        /* 从回绕前启动，保持启动、停稳以及收包的本机时间始终按顺序经过 0。 */
        const uint32_t initial_ms = wrap ? UINT32_MAX - 10U - startup_ms : 100U;
        Begin_Target_Test_At(initial_ms);
        if (!wrap)
            startup_ms = test_tick - initial_ms;
        Receive_Center(JETSON_CAN_TARGET_CENTER_ID, 5000, 4000);
        Step();
        assert(last_vx > 0.0f && last_wz == 0.0f);
        if (wrap)
            assert(test_tick == UINT32_MAX - 5U);
        can_arrival_before_snapshot = 1U;
        Step();
        assert(!can_arrival_before_snapshot && (!wrap || test_tick == 0U));
        Steps(2U);
        assert(enabled && !stops && !disables && last_vx > 0.0f && last_wz == 0.0f);
    }
}

/** @brief 同一源帧的 CAN 重发不能重复更新跟随目标。 */
static void Test_Repeated_Source_Frame(void)
{
    Begin_Target_Test();
    const uint32_t source = source_time_ms;
    const uint8_t frame[8] = {1U, 0x38U, 0x18U, 0x4cU, 0x1dU,
                              (uint8_t)source, (uint8_t)(source >> 8U), (uint8_t)(source >> 16U)};
    Jetson_CAN_Receive(JETSON_CAN_TARGET_CENTER_ID, frame, sizeof(frame));
    Step();
    const unsigned before = velocity_requests;
    Jetson_CAN_Receive(JETSON_CAN_TARGET_CENTER_ID, frame, sizeof(frame));
    Step();
    assert(velocity_requests == before);
    Receive_Center(JETSON_CAN_TARGET_CENTER_ID, 6200, 7500);
    Step();
    assert(velocity_requests == before + 1U);
}

/** @brief 验证预测进窗时先减速，真实入窗才停车并核验新帧。 */
static void Test_Approach_Brakes_And_Verifies(void)
{
    Begin_Target_Test();
    Receive_Center(JETSON_CAN_TARGET_CENTER_ID, 6200, 7500);
    Steps(4U);
    assert(last_vy > 0.0f && last_vy <= 0.005f && last_vx == 0.0f);
    for (unsigned i = 0U; i < 3U; ++i)
    {
        /* 以 20 ms 重报验证旧速率兼容；5 ms 任务轮询不视为新测量。 */
        Receive_Center(JETSON_CAN_TARGET_CENTER_ID, 6200, 7500);
        Steps(4U);
        assert(stops == 0U && last_vy > 0.0f && last_vy <= JETSON_AUTO_APPROACH_SPEED_MPS);
    }
    const unsigned before_slow = velocity_requests;
    Receive_Center(JETSON_CAN_TARGET_CENTER_ID, 5400, 7500);
    Step();
    assert(stops == 0U && velocity_requests == before_slow + 1U && sent_count == 0U);
    assert(last_vx == 0.0f && last_vy > 0.0f && last_vy < 0.03f && last_wz == 0.0f);
    const unsigned moves = velocity_requests;

    Receive_Center(JETSON_CAN_TARGET_CENTER_ID, 5000, 7500);
    Steps(5U);
    assert(stops == 1U && velocity_requests == moves && sent_count == 0U);

    /* 发出零速后漂到 5154 已超出 ±100，恢复连续跟随，不视为到位。 */
    motion_ready = 0U; /* 仍在减速，也可以重新跟踪。 */
    Receive_Center(JETSON_CAN_TARGET_CENTER_ID, 5154, 7500);
    Step();
    assert(last_vx == 0.0f && last_vy > 0.0f && last_vy <= 0.005f && last_wz == 0.0f);
    assert(stops == 1U && velocity_requests == moves + 1U && sent_count == 0U);
    Steps(20U);
    assert(stops == 1U && last_vy > 0.0f && last_vy <= 0.005f);
    Receive_Center(JETSON_CAN_TARGET_CENTER_ID, 5154, 7500);
    Step();
    assert(last_vy > 0.0f && stops == 1U && sent_count == 0U);
    Receive_Center(JETSON_CAN_TARGET_CENTER_ID, 5100, 7400);
    Step();
    assert(stops == 2U && sent_count == 0U);

    Receive_Center(JETSON_CAN_TARGET_CENTER_ID, 5110, 7400);
    Step();
    assert(!stopped && sent_count == 0U);
    stopped = 1U;
    Receive_Center(JETSON_CAN_TARGET_CENTER_ID, 5110, 7400);
    Step();
    assert(sent_count == 0U); /* 首次观察到停稳时已有的图像仍不放行。 */
    Receive_Center_With_Times(JETSON_CAN_TARGET_CENTER_ID, 5110, 7400, source_time_ms, test_tick - 1U);
    Step();
    assert(sent_count == 0U); /* 实际入队时间早于停稳，解析后序号更新也不能放行。 */
    /* 已入窗后，新图像轻微抖出单侧边缘，仍可确认而不反复启停。 */
    const uint32_t verified_frame = source_time_ms & 0xffffffU;
    Receive_Center(JETSON_CAN_TARGET_CENTER_ID, 5110, 7400);
    Wait_For_Count(&sent_count, 1U);
    Check_Sent(0U, JETSON_CAN_POINT_REACHED_ID, 1U);
    assert(sent[0].source_frame == verified_frame);
    Receive_Center(JETSON_CAN_TARGET_CENTER_ID, 4900, 7500);
    Steps(JETSON_AUTO_REQUEST_RETRY_MS / 5U + 1U);
    assert(sent_count == 2U && sent[1].source_frame == verified_frame && sent[1].request_id == sent[0].request_id);
    Receive_Command_Ack(0U, JETSON_CAN_POINT_REPLY_COMMAND, sent[0].request_id);
    Step();
    Receive_Center(JETSON_CAN_TARGET_CENTER_ID, 5000, 7500);
    Step();
    const uint32_t fresh_frame = source_time_ms & 0xffffffU;
    Receive_Center(JETSON_CAN_TARGET_CENTER_ID, 5000, 7500);
    Wait_For_Count(&sent_count, 3U);
    assert(sent[2].source_frame == fresh_frame && sent[2].request_id != sent[0].request_id);
}

/** @brief 目标在制动或核验时丢失，停车等待新的有效目标。 */
static void Test_Approach_Lost_During_Brake(void)
{
    for (unsigned timeout = 0U; timeout < 2U; ++timeout)
    {
        Begin_Target_Test();
        Receive_Center(JETSON_CAN_TARGET_CENTER_ID, 5800, 7500);
        Step();
        Receive_Center(JETSON_CAN_TARGET_CENTER_ID, 5000, 7500);
        Step();
        const unsigned moves = velocity_requests;
        const uint8_t invalid_center[8] = {0};
        Jetson_CAN_Receive(JETSON_CAN_TARGET_CENTER_ID, invalid_center, sizeof(invalid_center));
        Steps(5U);
        if (timeout)
            Steps(JETSON_AUTO_CENTER_TIMEOUT_MS / 5U + 1U);
        assert(stops == 1U && velocity_requests == moves && sent_count == 0U && enabled && !stopped);
        Receive_Center(JETSON_CAN_TARGET_CENTER_ID, 5500, 7000);
        Wait_For_Count(&velocity_requests, moves + 1U);
        assert(last_wz == 0.0f);
    }
}

/** @brief 远离设定点的速度不削弱纠正，远段对角合速保持限幅。 */
static void Test_Approach_Retreat_And_Limit(void)
{
    Begin_Target_Test();
    Receive_Center(JETSON_CAN_TARGET_CENTER_ID, 5800, 7500);
    Steps(16U);
    Receive_Center(JETSON_CAN_TARGET_CENTER_ID, 6200, 7500);
    Step();
    assert(stops == 0U && last_vy > 0.0f && last_vy <= JETSON_AUTO_APPROACH_SPEED_MPS);
    for (unsigned i = 0; i < 4; ++i)
    {
        Receive_Center(JETSON_CAN_TARGET_CENTER_ID, 8500, 5000);
        Steps(4U);
    }
    assert(last_vx > 0.0f && last_vy > 0.0f &&
           hypotf(last_vx, last_vy) <= JETSON_AUTO_APPROACH_SPEED_MPS + 1e-6f);
}

/** @brief 近处横向未收敛时禁止前进，物块对正后再低速进入夹取窗。 */
static void Test_Pickup_Lateral_Before_Forward(void)
{
    Begin_Target_Test();
    Receive_Center(JETSON_CAN_TARGET_CENTER_ID, 5900, 6200);
    Step();
    assert(last_vx == 0.0f && last_vy > 0.0f && !sent_count);
    for (unsigned i = 0U; i < 8U; ++i)
    {
        Receive_Center(JETSON_CAN_TARGET_CENTER_ID, 5000, 6200);
        Steps(4U);
    }
    assert(last_vx > 0.0f && last_vy == 0.0f && !sent_count);
}

/** @brief 横向掠过到位窗边界时保持平滑，纵向仍持续靠近。 */
static void Test_Approach_Axis_Window_Continuity(void)
{
    Begin_Target_Test();
    Receive_Center(JETSON_CAN_TARGET_CENTER_ID, 5101, 7000);
    Step();
    const float outside = last_vy;
    const unsigned updates = velocity_requests;
    Steps(3U);
    assert(velocity_requests == updates);
    Receive_Center(JETSON_CAN_TARGET_CENTER_ID, 5099, 7000);
    Step();
    assert(last_vx > 0.0f && last_vy == 0.0f && outside > 0.0f && outside <= 0.005f);
    assert(stops == 0U && velocity_requests == updates + 1U);
    Receive_Center(JETSON_CAN_TARGET_CENTER_ID, 5050, 7000);
    Step();
    assert(last_vx > 0.0f && last_vy == 0.0f && stops == 0U);
    assert(timed_velocity_requests == 0U && return_commands == 0U);
}

/** @brief 固定窗外点不会无限累加速度；靠近与换向不携带旧积分。 */
static void Test_Approach_No_Windup(void)
{
    Begin_Target_Test();
    float steady = 0.0f;
    for (unsigned i = 0; i < 200; ++i)
    {
        Receive_Center(JETSON_CAN_TARGET_CENTER_ID, 5300, 7500);
        Steps(4U);
        assert(last_vy > 0 && last_vy <= JETSON_AUTO_APPROACH_SPEED_MPS);
        if (i == 150) steady = last_vy;
    }
    assert(fabsf(last_vy - steady) < 1e-5f && last_vy > 0.04f);
    for (unsigned i = 0; i < 6; ++i)
    {
        Receive_Center(JETSON_CAN_TARGET_CENTER_ID, 5101, 7500);
        Steps(4U);
    }
    assert(last_vy > 0.0f && last_vy < steady && stops == 0U);
    for (unsigned i = 0; i < 6; ++i)
    {
        Receive_Center(JETSON_CAN_TARGET_CENTER_ID, 3500, 7500);
        Steps(4U);
    }
    assert(last_vy < 0.0f && fabsf(last_vy) <= JETSON_AUTO_APPROACH_SPEED_MPS && hard_stops == 0U);
}

/** @brief 合速饱和后靠近仍及时减速，不保留饱和补偿。 */
static void Test_Approach_Anti_Windup(void)
{
    Begin_Target_Test();
    for (unsigned i = 0U; i < 100U; ++i)
    {
        Receive_Center(JETSON_CAN_TARGET_CENTER_ID, 6500, 7500);
        Steps(4U);
        assert(last_vy <= JETSON_AUTO_APPROACH_SPEED_MPS);
    }
    assert(fabsf(last_vy - JETSON_AUTO_APPROACH_SPEED_MPS) < 1e-6f);
    for (unsigned i = 0U; i < 6U; ++i)
    {
        Receive_Center(JETSON_CAN_TARGET_CENTER_ID, 5150, 7500);
        Steps(4U);
    }
    assert(last_vy > 0.0f && last_vy < 0.06f && stops == 0U);
}

/** @brief 减速时重报入窗源帧不放行；过期后仍可凭新图像重新取得目标。 */
static void Test_Approach_Requires_New_Frame(void)
{
    Begin_Target_Test();
    Receive_Center(JETSON_CAN_TARGET_CENTER_ID, 5000, 7000);
    Step();
    const uint32_t source = source_time_ms;
    for (unsigned i = 0U; i < JETSON_AUTO_ARM_TIMEOUT_MS / 5U; ++i)
    {
        Receive_Center_At(JETSON_CAN_TARGET_CENTER_ID, 5000, 7500, source);
        Step();
    }
    assert(enabled && !stopped && !sent_count && stops == 1U);
    stopped = 1U;
    Step();
    Receive_Center(JETSON_CAN_TARGET_CENTER_ID, 5000, 7500);
    Step();
    Receive_Center(JETSON_CAN_TARGET_CENTER_ID, 5000, 7500);
    Steps(2U);
    assert(sent_count == 1U && stopped);

    for (unsigned blocked = 0U; blocked < 2U; ++blocked)
    {
        Begin_Target_Test();
        Receive_Center(JETSON_CAN_TARGET_CENTER_ID, 6000, 7500);
        Step();
        Receive_Center(JETSON_CAN_TARGET_CENTER_ID, 5000, 7500);
        Step();
        motion_ready = 0U; /* CAN 尚未提交零目标。 */
        const unsigned ticks = blocked ? JETSON_AUTO_HOLD_TIMEOUT_MS / 5U : 2U;
        for (unsigned i = 0U; i < ticks; ++i)
        {
            Receive_Center(JETSON_CAN_TARGET_CENTER_ID, 5000, 7500);
            Step();
        }
        assert(enabled && !sent_count);
        if (blocked)
        {
            Steps(JETSON_AUTO_BRAKE_TIMEOUT_MS / 5U);
            assert(!enabled && disables == 1U && !sent_count);
        }
        else
        {
            motion_ready = 1U;
            Steps(2U);
            Receive_Center(JETSON_CAN_TARGET_CENTER_ID, 5000, 7500);
            Steps(2U);
            assert(!sent_count); /* 减速阶段的图像不能充当停车后的复核。 */
            stopped = 1U;
            Step();
            Receive_Center(JETSON_CAN_TARGET_CENTER_ID, 5000, 7500);
            Steps(2U);
            assert(sent_count == 1U && stopped);
        }
    }
}

/** @brief 从接近推进到抓取完成并申请目的区，返回本次抓取编号。 */
static uint32_t Prepare_Selected_Route_From_Target(uint8_t destination)
{
    /* 横向超出容差时继续修正；双轴同时入窗后核验新图像。 */
    for (unsigned i = 0; i < 12; ++i)
    {
        Receive_Center(JETSON_CAN_TARGET_CENTER_ID, 5800, 7500);
        Steps(4U);
    }
    assert(last_vx == 0.0f && last_vy > 0.0f && last_wz == 0.0f);
    Receive_Center(JETSON_CAN_TARGET_CENTER_ID, 5000, 7500);
    Steps(4U);
    assert(stops && sent_count == 0U);
    stopped = 1U;
    Step();
    Receive_Center(JETSON_CAN_TARGET_CENTER_ID, 4900, 7600);
    Wait_For_Count(&sent_count, 1U);
    Check_Sent(0U, JETSON_CAN_POINT_REACHED_ID, 1U);
    assert(!distance_count);
    const uint32_t token = sent[0].request_id;
    const unsigned moves = velocity_requests;
    /* 抓取成功直接申请目的区。阻塞首发以验证旧回包不能提前放行。 */
    tx_ready = 0U;
    Complete_Last_Action();
    Steps(4U);
    assert(sent_count == 1U && !distance_count && velocity_requests == moves);
    Receive_Arrival_Ack(1U);
    Receive_Route(1U, destination);
    Receive_Turn(destination, token);
    Steps(3U);
    assert(sent_count == 1U && turn_count == 1U && !return_commands);
    tx_ready = 1U;
    Wait_For_Count(&sent_count, 2U);
    Check_Sent(1U, JETSON_CAN_ARRIVAL_ID, 0U);
    Steps(3U);
    assert(turn_count == 1U && !return_commands && !distance_count && velocity_requests == moves);
    return token;
}

/* 既有独立场景仍从开机开始；连续运输复用当前已恢复的目标阶段。 */
static uint32_t Prepare_Selected_Route(uint8_t destination)
{
    Begin_Target_Test();
    return Prepare_Selected_Route_From_Target(destination);
}

/** @brief 非视觉运动阶段同样受周期流期限约束，停稳后失能且恢复报文不续跑。 */
static void Check_Moving_Stream_Loss(void)
{
    assert(enabled && !stopped);
    periodic_stream = 0U;
    const unsigned before_stops = stops;
    const unsigned before_distances = distance_count;
    const unsigned before_turns = turn_count;
    Steps(JETSON_AUTO_LINK_TIMEOUT_MS / 5U - 1U);
    assert(enabled && stops == before_stops);
    const unsigned before_moves = velocity_requests;
    Step();
    assert(enabled && stops == before_stops + 1U);
    stopped = 1U;
    Step();
    assert(!enabled && disables == 1U);
    periodic_stream = 1U;
    Steps(100U);
    assert(!enabled && enables == 1U && velocity_requests == before_moves &&
           distance_count == before_distances && turn_count == before_turns);
}

/* 从新起点的前方返航，当前航向比启动时面向坡区的航向大约 11°。 */
static void Begin_Return_From_Route(uint8_t destination, uint32_t token)
{
    const unsigned turns_before = turn_count;
    const unsigned before = return_commands;
    Receive_Arrival_Ack(1U);
    Receive_Turn(destination, token);
    Steps(3U);
    assert(return_commands == before && turn_count == turns_before);
    odom = expected_origin;
    odom.x_m += 0.25;
    odom.yaw_rad = expected_return_yaw_rad + 0.20f;
    Receive_Route(1U, destination);
    Receive_Turn(destination == 1U ? 2U : 1U, token);
    Step();
    Receive_Turn(destination, 0U);
    Step();
    Receive_Turn(destination, token ^ 1U);
    Step();
    assert(return_commands == before && turn_count == turns_before && !distance_count);
    Receive_Turn(destination, token);
    Wait_For_Count(&return_commands, before + 1U);
    assert(last_vx < 0.0f && last_wz < 0.0f && turn_count == turns_before);
}

/** @brief 取得有效抓取路由后开始点返航，断周期流时制动失能。 */
static void Test_Return_Stream_Loss(void)
{
    const uint32_t token = Prepare_Selected_Route(1U);
    Begin_Return_From_Route(1U, token);
    Check_Moving_Stream_Loss();
}

/* 模拟到达起点，下一拍用新的里程计快照复核位置。 */
static void Stop_At_Return_Origin(void)
{
    const unsigned stops_before = stops;
    odom = expected_origin;
    odom.yaw_rad = expected_return_yaw_rad;
    Wait_For_Count(&stops, stops_before + 1U);
    Step();
    assert(Chassis_Motion_Ready() && !stopped && sent_count == 2U);
}

static void Verify_Safety_Center(int16_t x, int16_t y)
{
    Receive_Center(JETSON_CAN_SAFETY_CENTER_ID, x, y);
    Step();
}

/* 同一源帧重发不能充当第二帧；本阶段之前已经缓存的帧也不能放行。 */
static void Verify_First_Safety_Frame(int16_t x, int16_t y, const unsigned *counter)
{
    const unsigned count = *counter;
    const uint32_t original_source = source_time_ms;
    Receive_Center(JETSON_CAN_SAFETY_CENTER_ID, x, y);
    Steps(3U);
    assert(*counter == count);
    Receive_Center_At(JETSON_CAN_SAFETY_CENTER_ID, x, y, original_source);
    Step();
    assert(*counter == count);
}

static void Begin_Timed_Lateral(uint8_t destination, uint32_t token)
{
    const unsigned before = timed_velocity_requests;
    Receive_Command_Ack(1U, 1U, token ^ 1U);
    Receive_Center(JETSON_CAN_SAFETY_CENTER_ID, 5000, 7500);
    Steps(4U);
    assert(sent_count == 3U && !distance_count && timed_velocity_requests == before);

    /* 横移只依赖匹配回执，不依赖有效图像。 */
    const uint8_t invisible[8] = {0};
    Jetson_CAN_Receive(JETSON_CAN_SAFETY_CENTER_ID, invisible, sizeof(invisible));
    Receive_Command_Ack(1U, 1U, token);
    /* 错误帧不能抹掉本拍已经收到的匹配回执。 */
    Receive_Command_Ack(1U, 2U, token);
    Receive_Command_Ack(1U, 1U, token ^ 1U);
    Receive_Command_Ack(0U, 1U, token);
    Jetson_CAN_Receive(JETSON_CAN_COMMAND_REPLY_ID, NULL, 0U);
    Step();

    const float lateral = destination == 1U ? JETSON_AUTO_LATERAL_SPEED_MPS : -JETSON_AUTO_LATERAL_SPEED_MPS;
    assert(timed_velocity_requests == before + 1U && !stopped);
    assert(last_vx == 0.0f && last_vy == lateral && last_wz == 0.0f);
}

static void Complete_Timed_Lateral(uint8_t destination, uint32_t token)
{
    const unsigned before_stops = stops;
    const unsigned before_velocity = velocity_requests;
    const double moved_lateral = destination == 1U ? 0.12 : -0.16;
    odom.x_m -= moved_lateral * sin(odom.yaw_rad);
    odom.y_m += moved_lateral * cos(odom.yaw_rad);

    /* 横移中出现的新像素中心不能改变先前提交的动作。 */
    Receive_Center(JETSON_CAN_SAFETY_CENTER_ID, 9000, 2000);
    Steps(JETSON_AUTO_LATERAL_DURATION_MS / 5U - 1U);
    assert(stops == before_stops && velocity_requests == before_velocity && sent_count == 3U);
    Step();
    assert(stops == before_stops + 1U && sent_count == 3U);
    assert(last_vx == 0.0f && last_vy == 0.0f && last_wz == 0.0f);
    Steps(3U);
    assert(!stopped && sent_count == 3U);

    stopped = 1U;
    Wait_For_Count(&sent_count, 4U);
    Check_Sent(3U, JETSON_CAN_SAFETY_ARRIVED_ID, token);
    assert(!distance_count);
}

/* 回退后的里程计含纵向位移和角偏。粗返只消除投送时的横移，并恢复初次参考航向。 */
static void Complete_Coarse_Recenter(double outbound_lateral, int16_t center_x, int16_t center_y,
                                     uint8_t delay, uint32_t token)
{
    const double forward_x = cos(expected_return_yaw_rad), forward_y = sin(expected_return_yaw_rad);
    const double left_x = -forward_y, left_y = forward_x;
    odom.x_m += -0.18 * forward_x + 0.025 * left_x;
    odom.y_m += -0.18 * forward_y + 0.025 * left_y;
    const float heading_error = outbound_lateral > 0.0 ? 0.20f : -0.20f;
    odom.yaw_rad = expected_return_yaw_rad + heading_error;
    odom.vx_mps = -0.10f;  /* 退车残余速度会触发规划器阻尼，仍不应向坡面发速度。 */

    /* ACK 之前及粗返中的图像即使完全居中，也不能取代粗返或提前记录新起点。 */
    Receive_Center(JETSON_CAN_SAFETY_CENTER_ID, center_x, center_y);
    Receive_Command_Ack(1U, 4U, token);
    const unsigned before_world = return_commands;
    Wait_For_Count(&return_commands, before_world + 1U);
    const double world_x = cos(odom.yaw_rad) * last_vx - sin(odom.yaw_rad) * last_vy;
    const double world_y = sin(odom.yaw_rad) * last_vx + cos(odom.yaw_rad) * last_vy;
    assert(fabs(world_x * forward_x + world_y * forward_y) < 1e-5);
    assert((world_x * left_x + world_y * left_y) * outbound_lateral < 0.0);
    assert(hypot(world_x, world_y) <= JETSON_AUTO_RECENTER_SPEED_MPS + 1e-6);
    assert(last_wz * heading_error < 0.0f && turn_count == 1U);
    odom.vx_mps = 0.0f;
    if (delay)
        Steps((JETSON_AUTO_ALIGN_TIMEOUT_MS - 350U) / 5U);

    /* 横移已消除，坡前不追回退丢失的纵向距离。航向尚未恢复时继续纠角。 */
    odom.x_m -= outbound_lateral * left_x + 0.045 * forward_x;
    odom.y_m -= outbound_lateral * left_y + 0.045 * forward_y;
    const unsigned before_yaw = return_commands;
    Wait_For_Count(&return_commands, before_yaw + 1U);
    assert(hypot(last_vx, last_vy) < 1e-5 && last_wz * heading_error < 0.0f);
    const unsigned before_stop = stops;
    odom.yaw_rad = expected_return_yaw_rad;
    Wait_For_Count(&stops, before_stop + 1U);
    assert(!stopped);
    Receive_Center(JETSON_CAN_SAFETY_CENTER_ID, center_x, center_y);
    Step();
    assert(turn_count == 1U && sent_count == 7U);

    const uint32_t cached_source = source_time_ms;
    Receive_Center(JETSON_CAN_SAFETY_CENTER_ID, center_x, center_y);
    stopped = 1U;
    Step();
    const unsigned before_visual = velocity_requests;
    Receive_Center_At(JETSON_CAN_SAFETY_CENTER_ID, center_x - 1000, center_y, cached_source);
    Steps(3U);
    assert(velocity_requests == before_visual && turn_count == 1U && sent_count == 7U);
}

enum { PUSH_RAMP_STUCK = 19U, RETRACT_RAMP_STUCK, RETRACT_WHEELS_MOVING, ENCODER_CONFIRMED };

/* 现场时间线约 850 ms 送出零目标，轮速到约 1700 ms 才进入门限。 */
static void Delayed_Timed_Stop(void)
{
    const unsigned requests = sent_count, brakes = hard_stops, holds = stops;
    assert(!motion_ready && !stopped);
    Steps(850U / 5U);
    motion_ready = 1U;
    Steps(850U / 5U);
    assert(enabled && sent_count == requests && hard_stops == brakes && stops == holds);
    delay_hold_zero = 0U;
}

/* 斜坡永久不归零，或零命令已提交但轮速不合格，各阶段应在各自期限锁故障。 */
static void Check_Timed_Stop_Deadline(unsigned requests)
{
    const unsigned brakes = hard_stops;
    Steps(JETSON_AUTO_HOLD_TIMEOUT_MS / 5U - 1U);
    assert(enabled && hard_stops == brakes && sent_count == requests);
    Step();
    assert(hard_stops == brakes + 1U && sent_count == requests);
    stopped = 1U;
    Step();
    assert(!enabled && sent_count == requests);
}

/* 从待 ROI 阶段完成一趟，按场景确认运动、夹爪或返航失效均能停车。 */
static void Transport_From_Route(uint8_t destination, uint32_t token, uint8_t stuck)
{
    const uint32_t completed_before = jetson_completed_deliveries;
    const unsigned before_fast_distance = fast_distance_count;
    Begin_Return_From_Route(destination, token);
    if (stuck == 3U)
    {
        Check_Moving_Stream_Loss();
        return;
    }
    if (stuck == 9U)
    {
        const unsigned before = stops;
        odom.valid = 0U;
        Step();
        assert(stops == before + 1U && sent_count == 2U);
        stopped = 1U;
        Step();
        assert(!enabled && disables == 1U);
        return;
    }
    Receive_Turn(destination, token);
    Steps(4U);
    assert(sent_count == 2U && turn_count == 1U);
    Stop_At_Return_Origin();
    if (stuck == 10U)
    {
        odom.x_m += 0.15;
        stopped = 1U;
        Steps(3U);
        assert(!enabled && disables == 1U && sent_count == 2U);
        return;
    }
    stopped = 1U;
    Wait_For_Count(&sent_count, 3U);
    Check_Sent(2U, JETSON_CAN_TURN_COMPLETED_ID, token);
    const Chassis_Odometry_t lateral_origin = odom;
    Begin_Timed_Lateral(destination, token);
    if (stuck == 13U)
    {
        const unsigned before = stops;
        odom.x_m += JETSON_AUTO_ALIGN_MAX_M + 0.10;
        Step();
        assert(stops == before + 1U && sent_count == 3U);
        stopped = 1U;
        Step();
        assert(!enabled && disables == 1U && !distance_count);
        return;
    }
    if (stuck == 14U)
    {
        Check_Moving_Stream_Loss();
        return;
    }
    Complete_Timed_Lateral(destination, token);
    const double outbound_lateral = -(odom.x_m - lateral_origin.x_m) * sin(expected_return_yaw_rad) +
                                    (odom.y_m - lateral_origin.y_m) * cos(expected_return_yaw_rad);
    Receive_Command_Ack(1U, 2U, token);
    const unsigned before_push = velocity_requests;
    const unsigned timed_before_push = timed_velocity_requests;
    Wait_For_Count(&velocity_requests, before_push + 1U);
    assert(timed_velocity_requests == timed_before_push + 1U);
    assert(fabsf(last_vx - JETSON_AUTO_PUSH_SPEED_MPS) < 1e-6f && last_vy == 0.0f && last_wz == 0.0f);
    if (stuck == 4U)
    {
        Check_Moving_Stream_Loss();
        return;
    }

    const unsigned before_push_stop = stops;
    /* 推进期间的大幅中心变化及丢帧，只能维持链路，不能改变动作。 */
    const uint8_t invisible[8] = {0};
    for (unsigned i = 0U; i < 8U; ++i)
    {
        Receive_Center(JETSON_CAN_SAFETY_CENTER_ID, i % 2U ? 1000 : 9000, 1000);
        Step();
        Jetson_CAN_Receive(JETSON_CAN_SAFETY_CENTER_ID, invisible, sizeof(invisible));
        Step();
        assert(enabled && stops == before_push_stop && velocity_requests == before_push + 1U && sent_count == 4U);
    }
    if ((stuck >= 6U && stuck <= 8U) || stuck == 12U)
    {
        if (stuck == 6U) motors_ready = 0U;
        if (stuck == 7U) enabled = 0U;
        if (stuck == 8U) usb_state.connected = 0U;
        if (stuck == 12U) odom.valid = 0U;
        Step();
        assert(stops == before_push_stop + 1U && sent_count == 4U);
        stopped = 1U;
        Step();
        assert(!enabled && !distance_count && velocity_requests == before_push + 1U);
        return;
    }
    Steps(JETSON_AUTO_PUSH_DURATION_MS / 5U - 17U);
    assert(stops == before_push_stop && sent_count == 4U);
    delay_hold_zero = (stuck == 0U || stuck == PUSH_RAMP_STUCK);
    Step();
    assert(stops == before_push_stop + 1U && sent_count == 4U);
    if (stuck == 1U || stuck == PUSH_RAMP_STUCK)
    {
        Check_Timed_Stop_Deadline(4U);
        return;
    }
    if (stuck == 0U)
        Delayed_Timed_Stop();
    encoder_stop_confirmed = stuck == ENCODER_CONFIRMED;
    stopped = !encoder_stop_confirmed;
    Wait_For_Count(&sent_count, 5U);
    encoder_stop_confirmed = 0U;
    Check_Sent(4U, JETSON_CAN_POINT_REACHED_ID, 2U);
    assert(!distance_count);
    const unsigned before_retract = velocity_requests;
    Steps(3U);
    assert(velocity_requests == before_retract);
    if (stuck == 2U)
    {
        const uint32_t action_id = sent[4].request_id;
        Receive_Command_Ack(1U, 5U, action_id);
        Receive_Action_Result(2U, 0U, action_id);
        stopped = 1U; /* 故障制动收到停车反馈。 */
        Steps(2U);
        assert(!enabled && disables == 1U && velocity_requests == before_retract);
        return;
    }
    Complete_Last_Action();
    Wait_For_Count(&velocity_requests, before_retract + 1U);
    assert(timed_velocity_requests == timed_before_push + 2U);
    assert(fabsf(last_vx + JETSON_AUTO_RETRACT_SPEED_MPS) < 1e-6f && last_vy == 0.0f && last_wz == 0.0f);
    if (stuck == 5U)
    {
        Check_Moving_Stream_Loss();
        return;
    }
    const unsigned before_retract_stop = stops;
    Steps(JETSON_AUTO_RETRACT_DURATION_MS / 5U - 1U);
    assert(stops == before_retract_stop && sent_count == 5U);
    delay_hold_zero = (stuck == 0U || stuck == RETRACT_RAMP_STUCK);
    Step();
    assert(stops == before_retract_stop + 1U && sent_count == 5U);
    if (stuck == RETRACT_RAMP_STUCK || stuck == RETRACT_WHEELS_MOVING)
    {
        Check_Timed_Stop_Deadline(5U);
        return;
    }
    if (stuck == 0U)
        Delayed_Timed_Stop();
    Steps(4U);
    assert(!stopped && sent_count == 5U);
    encoder_stop_confirmed = stuck == ENCODER_CONFIRMED;
    stopped = !encoder_stop_confirmed;
    Wait_For_Count(&sent_count, 6U);
    encoder_stop_confirmed = 0U;
    Check_Sent(5U, JETSON_CAN_DELIVERY_COMPLETED_ID, token);

    Receive_Command_Ack(1U, 4U, token ^ 1U);
    Steps(JETSON_AUTO_REQUEST_RETRY_MS / 5U + 1U);
    assert(turn_count == 1U && !distance_count);
    assert(sent_count == 7U);
    Check_Sent(6U, JETSON_CAN_DELIVERY_COMPLETED_ID, token);

    const int16_t center_x = (int16_t)lroundf(JETSON_AUTO_LOCALIZE_X * 10000.0f);
    const int16_t center_y = (int16_t)lroundf(JETSON_AUTO_LOCALIZE_Y * 10000.0f);
    assert(fast_distance_count == before_fast_distance);
    const uint32_t recenter_start = test_tick;
    Complete_Coarse_Recenter(outbound_lateral, center_x, center_y, stuck == 15U, token);
    const unsigned before_localizing = velocity_requests;
    assert(!distance_count && fast_distance_count == before_fast_distance);
    if (stuck == 15U)
    {
        assert(enabled && test_tick - recenter_start < JETSON_AUTO_ALIGN_TIMEOUT_MS);
        /* 粗返已经用掉大部分预算，进入视觉后不能重新获得完整的对准时限。 */
        Steps((JETSON_AUTO_ALIGN_TIMEOUT_MS - (test_tick - recenter_start)) / 5U + 3U);
        assert(!enabled && disables == 1U && turn_count == 1U && sent_count == 7U);
        return;
    }
    if (stuck == 16U)
    {
        /* 此位移距粗返终点仍小于上限，距回退停点已超限，视觉阶段须沿用后者。 */
        const double extra = -copysign(JETSON_AUTO_ALIGN_MAX_M - fabs(outbound_lateral) + 0.02,
                                       outbound_lateral);
        odom.x_m -= sin(expected_return_yaw_rad) * extra;
        odom.y_m += cos(expected_return_yaw_rad) * extra;
        Steps(2U);
        assert(!enabled && disables == 1U && turn_count == 1U && sent_count == 7U);
        return;
    }
    if (stuck == 11U)
    {
        Steps(JETSON_AUTO_CENTER_TIMEOUT_MS / 5U + JETSON_AUTO_BRAKE_TIMEOUT_MS / 5U);
        assert(enabled && !disables);  /* 短暂遮挡期间保持零速和使能。 */
        Steps((JETSON_AUTO_ALIGN_TIMEOUT_MS + JETSON_AUTO_BRAKE_TIMEOUT_MS) / 5U);
        assert(!enabled && turn_count == 1U && sent_count == 7U);
        return;
    }
    Receive_Center(JETSON_CAN_SAFETY_CENTER_ID, center_x, center_y - 1000);
    Wait_For_Count(&velocity_requests, before_localizing + 1U);
    assert(last_vx > 0.0f && last_vy == 0.0f && last_wz == 0.0f);

    /* 左右投送都重新以全区中心复核时的位姿建起点。 */
    odom.x_m += 0.11;
    odom.y_m = destination == 1U ? 0.03 : -0.30;
    /* 打滑后复定位允许带角度误差，下一趟仍须用开机时面向坡区的参考航向。 */
    odom.yaw_rad += 0.80f;
    Verify_Safety_Center(center_x, center_y);
    stopped = 1U;
    Step();
    /* 下发零速后 x 越界两次才微调，期间 y 越界的独立证据不会覆盖 x。 */
    const unsigned before_adjust = velocity_requests;
    Receive_Center(JETSON_CAN_SAFETY_CENTER_ID, center_x + 105, center_y);
    Step();
    Receive_Center(JETSON_CAN_SAFETY_CENTER_ID, center_x, center_y + 105);
    Step();
    assert(velocity_requests == before_adjust && turn_count == 1U);
    Receive_Center(JETSON_CAN_SAFETY_CENTER_ID, center_x + 105, center_y);
    Step();
    assert(velocity_requests == before_adjust + 1U && last_vy < 0.0f && last_vx == 0.0f);
    Verify_Safety_Center(center_x, center_y);
    Verify_Safety_Center(center_x, center_y);
    assert(turn_count == 1U);
    stopped = 1U;
    Step();
    Verify_First_Safety_Frame(center_x, center_y, &turn_count);
    assert(enabled && !disables && jetson_completed_deliveries == completed_before);
    Receive_Center(JETSON_CAN_SAFETY_CENTER_ID, center_x, center_y);
    if (JETSON_AUTO_DELIVERY_LIMIT && completed_before + 1U == JETSON_AUTO_DELIVERY_LIMIT)
    {
        const unsigned before_fault = hard_stops, before_motion = velocity_requests;
        Wait_For_Count(&disables, 1U);
        assert(!enabled && !disable_feedback_checks && turn_count == 1U && sent_count == 7U);
        Step();
        disabled_wheel_mask = 1U;
        Step();
        disabled_wheel_mask = 3U;
        Step();
        assert(disable_feedback_checks == 3U && hard_stops == before_fault);
        assert(jetson_completed_deliveries == completed_before);
        if (stuck == 18U)
            usb_state.connected = 0U;
        if (stuck != 17U)
            disabled_wheel_mask = 7U;
        Step();
        const unsigned checks = disable_feedback_checks;
        Receive_Center(JETSON_CAN_TARGET_CENTER_ID, 5000, 4000);
        Steps((JETSON_AUTO_BRAKE_TIMEOUT_MS + JETSON_AUTO_REQUEST_RETRY_MS) / 5U + 2U);
        assert(hard_stops == before_fault + (stuck == 17U || stuck == 18U));
        if (stuck != 17U)
            assert(disable_feedback_checks == checks);
        disabled_wheel_mask = 7U;
        usb_state.connected = 1U; /* 迟来反馈或重连都不启动另一趟。 */
        Steps(3U);
        assert(!enabled && enables == 1U && disables == 1U && turn_count == 1U &&
               sent_count == 7U && velocity_requests == before_motion);
        assert(jetson_completed_deliveries == completed_before + (stuck != 17U && stuck != 18U));
        return;
    }
    Wait_For_Count(&turn_count, 2U);
    assert(jetson_completed_deliveries == completed_before + 1U);
    assert(last_turn_angle == 180.0f && sent_count == 7U);
    expected_origin = odom;
    Steps(3U);
    assert(sent_count == 7U);
    Complete_Search_Turn();
    Wait_For_Count(&sent_count, 8U);
    const uint32_t search_id = sent[7].token;
    assert(search_id != 0U);
    Check_Sent(7U, JETSON_CAN_START_SEARCH_ID, search_id);

    const unsigned velocities_before = velocity_requests;
    Receive_Center(JETSON_CAN_TARGET_CENTER_ID, 5000, 4000);
    Receive_Command_Ack(1U, 3U, 0U);
    Receive_Command_Ack(1U, 3U, token);
    Receive_Command_Ack(1U, 4U, search_id);
    Steps(JETSON_AUTO_REQUEST_RETRY_MS / 5U + 1U);
    assert(velocity_requests == velocities_before && sent_count == 9U);
    Check_Sent(8U, JETSON_CAN_START_SEARCH_ID, search_id);
    Receive_Command_Ack(1U, 3U, search_id);
    Receive_Command_Ack(1U, 3U, token);
    Step();
    assert(velocity_requests == velocities_before);
    Receive_Center(JETSON_CAN_TARGET_CENTER_ID, 5000, 4000);
    Wait_For_Count(&velocity_requests, velocities_before + 1U);
    assert(last_vx > 0.0f && last_wz == 0.0f);
    assert(!distance_count && turn_count == 2U);
    assert(fast_distance_count == before_fast_distance);
}

static void Test_Return_Recovers_Small_Error(void)
{
    const uint32_t token = Prepare_Selected_Route(1U);
    Begin_Return_From_Route(1U, token);
    Receive_Turn(1U, token);
    Steps(4U);
    Stop_At_Return_Origin();
    const unsigned before = return_commands;
    odom.x_m += 0.025;
    Steps(2U);
    assert(enabled && !disables && return_commands == before && sent_count == 2U);

    stopped = 1U;
    Wait_For_Count(&return_commands, before + 1U);
    assert(last_vx < 0.0f);
    Stop_At_Return_Origin();
    stopped = 1U;
    Wait_For_Count(&sent_count, 3U);
    Check_Sent(2U, JETSON_CAN_TURN_COMPLETED_ID, token);
    assert(enabled && !disables);
}

static void Test_Long_Return(void)
{
    const uint32_t token = Prepare_Selected_Route(1U);
    odom = expected_origin;
    odom.x_m += 1.60;
    odom.yaw_rad = expected_return_yaw_rad;
    Receive_Arrival_Ack(1U);
    Receive_Route(1U, 1U);
    Receive_Turn(1U, token);
    const unsigned before = return_commands;
    Wait_For_Count(&return_commands, before + 1U);
    assert(enabled && !disables && last_vx < 0.0f && sent_count == 2U);

    Stop_At_Return_Origin();
    stopped = 1U;
    Wait_For_Count(&sent_count, 3U);
    Check_Sent(2U, JETSON_CAN_TURN_COMPLETED_ID, token);
    assert(enabled && !disables);
}

static void Test_Transport(uint8_t destination, uint8_t stuck)
{
    const uint32_t token = Prepare_Selected_Route(destination);
    Transport_From_Route(destination, token, stuck);
}

/** @brief 一次初始化连续完成左、右、左三趟，核对新起点和请求身份。 */
static void Test_Consecutive_Transports(uint8_t second_fault)
{
    Begin_Target_Test_At_Heading(100U, 0.37f);
    const uint32_t first_token = Prepare_Selected_Route_From_Target(1U);
    Transport_From_Route(1U, first_token, 0U);
    assert(expected_origin.yaw_rad > expected_return_yaw_rad + 0.70f);
    assert(hypot(expected_origin.x_m, expected_origin.y_m) > CHASSIS_RETURN_CORRECTION_MAX_M);
    const uint32_t first_grasp = sent[0].request_id;
    const uint32_t first_release = sent[4].request_id;
    const uint32_t first_search = sent[7].token;
    const uint32_t first_startup = startup_id;
    const unsigned first_startup_queries = startup_queries;
    assert(enabled && !stopped && enables == 1U && disables == 0U && jetson_completed_deliveries == 1U);

    /* 只清测试记录，保持真实自动/解析器状态、时钟、USB会话、反馈和动作号连续。 */
    stops = velocity_requests = distance_count = sent_count = return_commands = 0U;
    turn_count = 1U;
    const uint32_t second_token = Prepare_Selected_Route_From_Target(2U);
    assert(sent[0].request_id != first_grasp && sent[0].request_id != first_release &&
           sent[0].request_id != first_search);
    Transport_From_Route(2U, second_token, second_fault);
    if (JETSON_AUTO_DELIVERY_LIMIT == 2U)
    {
        assert(!enabled && disables == 1U && total_turns == 2U);
        assert(second_token != first_token && sent[4].request_id != first_release);
        assert(jetson_completed_deliveries == (second_fault ? 1U : 2U));
        assert(startup_id == first_startup && startup_queries == first_startup_queries);
        return;
    }
    assert(sent[4].request_id != first_release && sent[7].token != first_search);
    assert(enabled && enables == 1U && disables == 0U && startup_id == first_startup &&
           startup_queries == first_startup_queries);
    /* 每趟在全区中心建起点，所以向左投送仍有自己的定时横移。 */
    stops = velocity_requests = distance_count = sent_count = return_commands = 0U;
    turn_count = 1U;
    const uint32_t third_token = Prepare_Selected_Route_From_Target(1U);
    Transport_From_Route(1U, third_token, 0U);
    assert(startup_id == first_startup && startup_queries == first_startup_queries);
    puts("PASS: left/right timed deliveries with refreshed common-center origins");
}

/* 板端消费 Jetson 已登记的目的区：绿、橙、绿、绿、橙对应左、右、左、左、右。 */
static void Test_Five_Deliveries(void)
{
    const uint8_t destinations[] = {1U, 2U, 1U, 1U, 2U};
    Begin_Target_Test_At_Heading(100U, 0.37f);
    uint32_t search = initial_search_id;
    const uint32_t startup = startup_id;
    for (unsigned leg = 0U; leg < 5U; ++leg)
    {
        assert(enabled && !disables && total_turns == leg + 1U && jetson_completed_deliveries == leg);
        const uint32_t grasp = Prepare_Selected_Route_From_Target(destinations[leg]);
        assert(grasp == sent[0].request_id && grasp > search);
        Transport_From_Route(destinations[leg], grasp, 0U);
        assert(sent[4].request_id > grasp && jetson_completed_deliveries == leg + 1U);
        assert(startup == startup_id && enables == 1U);
        if (leg + 1U < 5U)
        {
            assert(enabled && !disables && total_turns == leg + 2U);
            search = sent[7].token;
            assert(search > sent[4].request_id);
            /* 下一趟复用同一控制器、时钟和会话，只清理测试桩的逐趟记录。 */
            stops = velocity_requests = distance_count = sent_count = return_commands = 0U;
            turn_count = 1U;
        }
    }
    assert(!enabled && disables == 1U && total_turns == 5U && disabled_wheel_mask == 7U);
    assert(jetson_completed_deliveries == 5U && sent_count == 7U && !hard_stops);
    puts("PASS: five deliveries left/right/left/left/right, five search turns, confirmed disable after the fifth");
}

/** @brief 进入转向完成握手并暂时阻塞发送邮箱。
 * @param[in] wrap 1 表示让回执等待跨过 RTOS 毫秒回绕。
 */
static uint32_t Prepare_Turn_Wait(uint8_t wrap)
{
    Begin_Target_Test_At(wrap ? UINT32_MAX - 3250U : 100U);
    const uint32_t token = Prepare_Selected_Route_From_Target(2U);
    Begin_Return_From_Route(2U, token);
    Stop_At_Return_Origin();

    stopped = 1U;
    tx_ready = 0U;
    Step();
    assert(sent_count == 2U);
    return token;
}

/** @brief USB 忙时保持等待；发送错误立即制动，随后失能且不自动重启。 */
static void Test_USB_Send_Error(void)
{
    Prepare_Turn_Wait(0U);
    const unsigned stops_before = stops;
    Steps(3U);
    assert(enabled && sent_count == 2U && stops == stops_before);
    tx_ready = 1U;
    tx_error = 1U;
    Step();
    assert(enabled && sent_count == 2U && stops == stops_before + 1U);
    Step();
    assert(!enabled && disables == 1U && sent_count == 2U);
    tx_error = 0U;
    Steps(3U);
    assert(!enabled && enables == 1U && sent_count == 2U);
}

/** @brief 普通握手按消费拍截止：期限前成功，首次等于/超过截止及早到晚消费均停车，含回绕。 */
static void Test_Request_Deadline(void)
{
    Prepare_Turn_Wait(0U);
    const unsigned stops_before_mailbox_timeout = stops;
    Steps(JETSON_AUTO_REQUEST_TIMEOUT_MS / 5U - 1U);
    assert(enabled && sent_count == 2U && stops == stops_before_mailbox_timeout);
    tx_ready = 1U;
    Step();
    assert(sent_count == 2U && stops == stops_before_mailbox_timeout + 1U);

    for (unsigned wrap = 0U; wrap < 2U; ++wrap)
    {
        for (unsigned scenario = 0U; scenario < 4U; ++scenario)
        {
            const uint32_t token = Prepare_Turn_Wait((uint8_t)wrap);
            const uint32_t stage_ms = test_tick;
            Steps(100U);
            tx_ready = 1U;
            Step();
            Check_Sent(2U, JETSON_CAN_TURN_COMPLETED_ID, token);
            const unsigned stops_before_reply = stops;
            Steps(JETSON_AUTO_REQUEST_TIMEOUT_MS / 5U - 102U);
            const unsigned before_final = sent_count;
            assert(enabled && stops == stops_before_reply && before_final > 3U);
            /* 场景 0：到期前消费。1/2：首次在截止/截止后到达。3：到期前到达但截止时消费。 */
            test_tick = stage_ms + JETSON_AUTO_REQUEST_TIMEOUT_MS +
                (scenario == 0U || scenario == 3U ? (uint32_t)-1 : scenario == 2U ? 5U : 0U);
            Receive_Command_Ack(1U, 1U, token);
            if (scenario == 3U)
                ++test_tick;
            const unsigned before_velocity = velocity_requests;
            Run_Auto();
            assert(sent_count == before_final);
            Step();
            if (scenario == 0U)
                assert(enabled && stops == stops_before_reply && velocity_requests == before_velocity + 1U &&
                       last_vx == 0.0f && last_vy == -JETSON_AUTO_LATERAL_SPEED_MPS);
            else
                assert(!enabled && stops == stops_before_reply + 1U && velocity_requests == before_velocity);
        }
    }
}

/** @brief 目的区请求被拒绝时不得松爪、后退或返航。 */
static void Test_Reject_Route(void)
{
    for (unsigned accepted = 0U; accepted < 2U; ++accepted)
    {
        const uint32_t token = Prepare_Selected_Route(1U);
        const unsigned before_stop = stops;
        const unsigned before_moves = velocity_requests;
        Receive_Arrival_Ack((uint8_t)accepted);
        Receive_Route(0U, 0U);
        Receive_Turn(1U, token);
        Steps(2U);
        assert(!enabled && disables == 1U && stops == before_stop + 1U);
        assert(sent_count == 2U && !distance_count && !return_commands && velocity_requests == before_moves);
    }
}

/** @brief 合法无目标帧保持链路，等待新图像时已删除的类型 1不参与控制。 */
static void Test_Visual_Timeout_Without_Type1(void)
{
    uint32_t startup_elapsed = 0U;
    for (unsigned wrap = 0U; wrap < 2U; ++wrap)
    {
        Begin_Target_Test_At(wrap ? UINT32_MAX - 120U - startup_elapsed : 100U);
        if (!wrap) startup_elapsed = test_tick - 100U;
        Receive_Center(JETSON_CAN_TARGET_CENTER_ID, 5000, 4000);
        Step();
        assert(last_vx > 0.0f);
        const unsigned before_stop = stops, before_search = velocity_requests;
        const uint8_t legacy[8] = {0U};
        for (unsigned i = 0U; i < 200U; ++i)
        {
            const uint8_t no_target[8] = {0};
            Jetson_CAN_Receive(1U, legacy, sizeof(legacy));
            Jetson_CAN_Receive(JETSON_CAN_TARGET_CENTER_ID, no_target, sizeof(no_target));
            Step();
            stopped = 1U;
        }
        assert(enabled && !disables && stops == before_stop + 1U &&
               velocity_requests == before_search && turn_count == 1U);
    }
}

/** @brief 未枚举或尚无合法周期流时保持使能零速，不请求启动夹爪。 */
static void Test_Startup_Link_Gate(void)
{
    Reset();
    periodic_stream = 0U;
    Steps(80U);
    assert(!startup_queries && enabled && enables == 1U && !stops && !turn_count);
    const uint8_t legacy[8] = {1U};
    const uint8_t malformed[8] = {2U};
    Jetson_CAN_Receive(1U, legacy, sizeof(legacy));
    Jetson_CAN_Receive(JETSON_CAN_TARGET_CENTER_ID, malformed, sizeof(malformed));
    Jetson_CAN_Receive(JETSON_CAN_SAFETY_CENTER_ID, legacy, 5U);
    Steps(80U);
    assert(!startup_queries && enabled && enables == 1U && !stops && !velocity_requests);

    usb_state.connected = 0U;
    Receive_Center(JETSON_CAN_TARGET_CENTER_ID, 5000, 4000);
    Steps(3U);
    assert(!startup_queries && enabled && enables == 1U && !stops && !turn_count);
    usb_state.connected = 1U;
    ++usb_state.generation;
    periodic_stream = 1U;
    Step();
    assert(startup_queries && enabled && enables == 1U && !turn_count);
}

/* 邮箱暂忙可短暂等待。已授予发送机会后的 HAL 失败及长期占用均锁存。 */
static void Test_Startup_CAN_Fault(void)
{
    for (unsigned submit = 0U; submit < 2U; ++submit)
    {
        Reset();
        can_enable = 0U;
        Steps(4U);
        assert(!enables && !stops && !startup_queries);
        if (submit)
        {
            can_enable = 1U;
            enable_tx_failure = 1U;
        }
        Steps(JETSON_AUTO_ARM_TIMEOUT_MS / 5U + 1U);
        assert(stops == 1U && !enabled && enables == submit && !turn_count && !startup_queries);
        can_enable = 1U;
        enable_tx_failure = 0U;
        Steps(10U);
        assert(enables == submit && !enabled);
    }
}

/** @brief 除合法现有周期坐标外，坏帧、旧类型、未知类型和回执都不能延长运行许可。 */
static void Receive_Nonperiodic_Noise(void)
{
    const uint8_t malformed[8] = {2U};
    const uint8_t old_heartbeat[8] = {1U};
    Jetson_CAN_Receive(JETSON_CAN_TARGET_CENTER_ID, malformed, sizeof(malformed));
    Jetson_CAN_Receive(JETSON_CAN_SAFETY_CENTER_ID, old_heartbeat, 5U);
    Jetson_CAN_Receive(JETSON_CAN_TARGET_CENTER_ID, NULL, 0U);
    Jetson_CAN_Receive(1U, old_heartbeat, sizeof(old_heartbeat));
    Jetson_CAN_Receive(255U, old_heartbeat, sizeof(old_heartbeat));
    Receive_Arrival_Ack(1U);
    Receive_Route(1U, 1U);
    Receive_Command_Ack(1U, 3U, 0U);
    Receive_Action_Result(0U, 1U, startup_id);
}

/** @brief 250 ms 等号刹停含时钟回绕；一直没有停车反馈时在 500 ms 等号失能。 */
static void Test_Periodic_Stream_Timeout(void)
{
    uint32_t startup_elapsed = 0U;
    for (unsigned wrap = 0U; wrap < 2U; ++wrap)
    {
        Begin_Target_Test_At(wrap ? UINT32_MAX - 120U - startup_elapsed : 100U);
        if (!wrap) startup_elapsed = test_tick - 100U;
        periodic_stream = 0U;
        Receive_Center(JETSON_CAN_TARGET_CENTER_ID, 5000, 4000);
        Run_Auto();
        const uint32_t last_periodic_ms = test_tick;
        const unsigned before_loss = stops;
        for (unsigned i = 0U; i < JETSON_AUTO_LINK_TIMEOUT_MS / 5U - 1U; ++i)
        {
            Receive_Nonperiodic_Noise();
            Step();
        }
        assert(test_tick - last_periodic_ms == JETSON_AUTO_LINK_TIMEOUT_MS - 5U);
        /* 畸形视觉帧可先停止目标接近，但不得提前锁存链路故障。 */
        assert(enabled && stops <= before_loss + 1U);
        const unsigned before_stops = stops, before_moves = velocity_requests;
        Receive_Nonperiodic_Noise();
        Step();
        assert(test_tick - last_periodic_ms == JETSON_AUTO_LINK_TIMEOUT_MS);
        assert(enabled && stops == before_stops + 1U && velocity_requests == before_moves);
        const uint32_t brake_ms = test_tick;
        test_tick = brake_ms + JETSON_AUTO_BRAKE_TIMEOUT_MS - 1U;
        Run_Auto();
        assert(enabled && disables == 0U);
        test_tick = brake_ms + JETSON_AUTO_BRAKE_TIMEOUT_MS;
        Run_Auto();
        assert(!enabled && disables == 1U);
        periodic_stream = 1U;
        Receive_Center(JETSON_CAN_TARGET_CENTER_ID, 5000, 4000);
        Steps(100U);
        assert(!enabled && enables == 1U && velocity_requests == before_moves);
    }
}

/** @brief 拔线下一拍停车，拍间短断重连也凭连接代次停车；重连及新帧不会自动续跑。 */
static void Test_USB_Disconnect_And_Reconnect(void)
{
    for (unsigned reconnect_between_steps = 0U; reconnect_between_steps < 2U; ++reconnect_between_steps)
    {
        Begin_Target_Test();
        Receive_Center(JETSON_CAN_TARGET_CENTER_ID, 5000, 4000);
        Step();
        const unsigned before_stops = stops;
        const unsigned before_moves = velocity_requests;
        usb_state.connected = 0U;
        ++usb_state.generation;
        if (reconnect_between_steps)
        {
            usb_state.connected = 1U;
            ++usb_state.generation;
            Receive_Center(JETSON_CAN_TARGET_CENTER_ID, 5000, 4000);
        }
        Step();
        assert(enabled && stops == before_stops + 1U && velocity_requests == before_moves);
        stopped = 1U;
        Step();
        assert(!enabled && disables == 1U);
        usb_state.connected = 1U;
        ++usb_state.generation;
        Receive_Center(JETSON_CAN_TARGET_CENTER_ID, 5000, 4000);
        Steps(100U);
        assert(!enabled && enables == 1U && velocity_requests == before_moves);
    }
}

/** @brief 夹爪受理前后断流都要锁存，迟到结果不得恢复运动。 */
static void Test_Startup_Stream_Loss(void)
{
    for (unsigned accepted = 0U; accepted < 2U; ++accepted)
    {
        Reset();
        Step();
        assert(startup_id && startup_queries);
        if (accepted)
            Receive_Command_Ack(1U, 5U, startup_id);
        Step();
        assert(enabled && !turn_count);
        periodic_stream = 0U;
        Steps(JETSON_AUTO_LINK_TIMEOUT_MS / 5U);
        assert(enabled && stops == 1U);
        stopped = 1U;
        Step();
        assert(!enabled && disables == 1U);
        periodic_stream = 1U;
        Receive_Action_Result(0U, 1U, startup_id);
        Steps(10U);
        assert(!enabled && enables == 1U && !turn_count);
    }
}





/** @brief 启动无限等受理；受理后截止固定，乱序结果和失败均不绕过启动门槛。 */
static void Test_Startup_Action(void)
{
    Reset();
    Steps(JETSON_AUTO_ACTION_TIMEOUT_MS / 5U * 2U);
    assert(startup_queries > 2U && enabled && !turn_count && !stops && !disables);
    const uint32_t first_id = startup_id;
    Receive_Action_Result(0U, 1U, startup_id);
    Step();
    assert(enabled && !turn_count); /* 结果先到，ACK 丢失则保持零速。 */
    Receive_Command_Ack(1U, 5U, startup_id);
    Step();
    Step();
    assert(enabled && enables == 1U && initial_align_id != 0U && turn_count == 0U);

    Reset();
    Step();
    assert(startup_id != first_id); /* 软件重新初始化不能复用动作号。 */
    Receive_Action_Result(0U, 1U, first_id);
    Receive_Command_Ack(1U, 5U, first_id);
    Step();
    assert(enabled && !turn_count);
    Receive_Command_Ack(1U, 5U, startup_id);
    Step();
    const uint32_t accepted_ms = test_tick;
    for (unsigned i = 0U; i < 3U; ++i)
    {
        test_tick = accepted_ms + (i + 1U) * 3000U;
        Receive_Command_Ack(1U, 5U, startup_id);
        Run_Auto();
        assert(enabled && !turn_count && !stops);
    }
    test_tick = accepted_ms + JETSON_AUTO_ACTION_TIMEOUT_MS;
    Run_Auto();
    assert(enabled && !turn_count && stops == 1U);
    stopped = 1U;
    Receive_Action_Result(0U, 1U, startup_id);
    Steps(3U);
    assert(!enabled && enables == 1U && !turn_count);

    Reset();
    Step();
    Receive_Action_Result(0U, 0U, startup_id);
    stopped = 1U;
    Step();
    Receive_Command_Ack(1U, 5U, startup_id);
    Receive_Action_Result(0U, 1U, startup_id);
    Step();
    assert(!enabled && enables == 1U && !turn_count && stops == 1U);

    Reset();
    Step();
    Receive_Command_Ack(1U, 5U, startup_id);
    Step();
    const uint32_t boundary = test_tick + JETSON_AUTO_ACTION_TIMEOUT_MS;
    test_tick = boundary - 1U;
    Run_Auto();
    assert(enabled && !turn_count && !stops);
    test_tick = boundary;
    Receive_Action_Result(0U, 1U, startup_id);
    Run_Auto();
    assert(enabled && !turn_count && stops == 1U);
    stopped = 1U;
    Step();
    assert(!enabled && disables == 1U);

    Reset();
    Step();
    Receive_Command_Ack(1U, 5U, startup_id);
    Receive_Action_Result(0U, 1U, startup_id);
    imu_age_ms = 21U;
    Step();
    const unsigned queries_before_fault = startup_queries;
    test_tick += JETSON_AUTO_ACTION_TIMEOUT_MS + 1U;
    Step();
    assert(!enabled && stops == 1U && startup_queries == queries_before_fault);
    imu_age_ms = 0U;
    Step();
    assert(!enabled && enables == 1U && !turn_count);
}

/** @brief 开机对准只使用握手后的整区观测，停稳双帧后记录新起点并转向。 */
static void Test_Initial_Alignment(void)
{
    const int16_t x = (int16_t)roundf(JETSON_AUTO_LOCALIZE_X * 10000.0f);
    const int16_t y = (int16_t)roundf(JETSON_AUTO_LOCALIZE_Y * 10000.0f);
    Reset();
    Start_Automatic();
    Steps(3U);
    assert(initial_align_id && !turn_count);
    Receive_Center(JETSON_CAN_SAFETY_CENTER_ID, x, y);
    Receive_Command_Ack(1U, JETSON_CAN_START_ALIGN_COMMAND, startup_id);
    Steps(3U);
    assert(!turn_count && !velocity_requests);
    Acknowledge_Initial_Alignment();
    Steps(3U);
    assert(!turn_count && !velocity_requests); /* 已缓存的整区帧不能放行。 */
    Receive_Center(JETSON_CAN_TARGET_CENTER_ID, x, y);
    Step();
    assert(!turn_count && !velocity_requests);
    Verify_Safety_Center(x + 420, y);
    assert(last_vy < 0.0f && !stopped && turn_count == 0U);
    Verify_Safety_Center(x, y);
    Verify_Safety_Center(x, y);
    Verify_Safety_Center(x, y);
    assert(!turn_count); /* 停稳之前的入窗帧不能用于复核。 */
    const unsigned moves_before_stop = velocity_requests;
    Verify_Safety_Center(x - 250, y);
    Verify_Safety_Center(x - 250, y);
    Verify_Safety_Center(x + 250, y);
    Verify_Safety_Center(x + 250, y);
    assert(velocity_requests == moves_before_stop && !turn_count);
    odom.x_m += 0.017;
    odom.y_m -= 0.030;
    odom.yaw_rad = 0.15f;
    expected_origin = odom;
    expected_return_yaw_rad = odom.yaw_rad;
    stopped = 1U;
    Step();
    Verify_First_Safety_Frame(x, y, &turn_count);
    Verify_Safety_Center(x, y);
    assert(turn_count == 1U && last_turn_angle == 180.0f && !initial_search_id);

    Receive_Center(JETSON_CAN_TARGET_CENTER_ID, 5000, 4000);
    Step();
    assert(!initial_search_id);
    Complete_Search_Turn();
    Steps(3U);
    assert(initial_search_id && initial_search_id != initial_align_id);
    const unsigned moves = velocity_requests;
    Receive_Command_Ack(1U, 3U, initial_align_id);
    Verify_Safety_Center(x, y);
    Receive_Center(JETSON_CAN_TARGET_CENTER_ID, 5000, 4000);
    Step();
    assert(velocity_requests == moves);
    Receive_Command_Ack(1U, 3U, initial_search_id);
    Step();
    assert(velocity_requests == moves);

    const uint32_t token = Prepare_Selected_Route_From_Target(1U);
    Begin_Return_From_Route(1U, token);
    Stop_At_Return_Origin();
    stopped = 1U;
    Wait_For_Count(&sent_count, 3U);
    Check_Sent(2U, JETSON_CAN_TURN_COMPLETED_ID, token);

    Reset();
    Start_Automatic();
    Acknowledge_Initial_Alignment();
    Steps(JETSON_AUTO_INITIAL_CENTER_TIMEOUT_MS / 5U + 1U);
    assert(!enabled && !turn_count && !velocity_requests);

    Reset();
    Start_Automatic();
    Acknowledge_Initial_Alignment();
    odom.x_m += JETSON_AUTO_ALIGN_MAX_M + 0.01;
    Steps(2U);
    assert(!enabled && !turn_count && !velocity_requests);
}

static void Test_Initial_Camera_Warmup(void)
{
    const int16_t x = (int16_t)roundf(JETSON_AUTO_LOCALIZE_X * 10000.0f);
    const int16_t y = (int16_t)roundf(JETSON_AUTO_LOCALIZE_Y * 10000.0f);
    const uint8_t invisible[8] = {0};
    for (unsigned lose_again = 0U; lose_again <= 1U; ++lose_again)
    {
        Reset_At(UINT32_MAX - 3000U);
        Start_Automatic();
        Receive_Center(JETSON_CAN_SAFETY_CENTER_ID, x, y);
        const uint32_t old_frame = source_time_ms;
        Acknowledge_Initial_Alignment();
        Steps(4500U / 5U);
        Receive_Center_At(JETSON_CAN_SAFETY_CENTER_ID, x, y, old_frame);
        Step();
        Receive_Center_With_Times(JETSON_CAN_SAFETY_CENTER_ID, x, y, source_time_ms, test_tick - 300U);
        Steps(500U / 5U - 1U);
        assert(enabled && stopped && !turn_count && !velocity_requests && !hard_stops);

        Verify_Safety_Center(x, y);
        const uint32_t first_center_ms = test_tick;
        Jetson_CAN_Receive(JETSON_CAN_SAFETY_CENTER_ID, invisible, sizeof(invisible));
        Steps(8000U / 5U);
        assert(enabled && stopped && !turn_count && !velocity_requests && !hard_stops);
        Verify_Safety_Center(x, y);
        if (!lose_again)
        {
            Verify_Safety_Center(x, y);
            Verify_Safety_Center(x, y);
            assert(turn_count == 1U);
            continue;
        }

        Jetson_CAN_Receive(JETSON_CAN_SAFETY_CENTER_ID, invisible, sizeof(invisible));
        Steps((JETSON_AUTO_ALIGN_TIMEOUT_MS - (test_tick - first_center_ms)) / 5U - 1U);
        assert(enabled && !hard_stops && !turn_count && !velocity_requests);
        Verify_Safety_Center(x, y);  /* 到期时的新图不能重启对准时间。 */
        assert(hard_stops == 1U && !turn_count && !velocity_requests);
        Step();
        assert(!enabled && disables == 1U);
    }
}

/* 绿块实跑末段 x 约 5570、y 已入窗。独立横移死区模型不执行小于 0.025 m/s 的请求。 */
static void Test_Localize_Lateral_Deadband(void)
{
    const int16_t x = (int16_t)roundf(JETSON_AUTO_LOCALIZE_X * 10000.0f);
    const int16_t y = (int16_t)roundf(JETSON_AUTO_LOCALIZE_Y * 10000.0f);
    for (int side = -1; side <= 1; side += 2)
    {
        Reset();
        Start_Automatic();
        Acknowledge_Initial_Alignment();
        float observed_x = x + side * 140;
        float peak = 0.0f;
        for (unsigned frame = 0U; frame < 50U && !turn_count; ++frame)
        {
            Verify_Safety_Center((int16_t)lroundf(observed_x), y);
            Steps(7U);
            if (turn_count)
                break;
            assert(last_vx == 0.0f && side * last_vy <= 0.0f);
            peak = fmaxf(peak, fabsf(last_vy));
            if (fabsf(last_vy) >= 0.025f)
                observed_x += last_vy * 0.040f * 12000.0f;
            if (last_vy == 0.0f)
                stopped = 1U;
        }
        assert(peak >= 0.025f && turn_count == 1U && fabsf(observed_x - x) <= 100.0f);
    }

    Reset();
    Start_Automatic();
    Acknowledge_Initial_Alignment();
    for (unsigned i = 0U; i < 8U; ++i)
    {
        Verify_Safety_Center(x + 140, y);
        Steps(7U);
    }
    const unsigned moving_stops = stops;
    Verify_Safety_Center(x + 95, y + 80);
    assert(stops == moving_stops && last_vy < 0.0f && last_vx == 0.0f && !turn_count);
    Steps(7U);
    Verify_Safety_Center(x + 92, y);
    assert(stops == moving_stops + 1U && !turn_count);
    Verify_Safety_Center(x, y);
    assert(!turn_count);
    stopped = 1U;
    Step();
    Verify_First_Safety_Frame(x, y, &turn_count);
    Verify_Safety_Center(x, y);
    assert(turn_count == 1U);
}

/** @brief 类型21两种报文任意顺序/分别丢失；重复查询同编号，终态失败和截止不续期。 */
static void Test_Action_Transactions(void)
{
    for (unsigned scenario = 0U; scenario < 8U; ++scenario)
    {
        Begin_Target_Test();
        Receive_Center(JETSON_CAN_TARGET_CENTER_ID, 5000, 7500);
        Step();
        Receive_Center(JETSON_CAN_TARGET_CENTER_ID, 4900, 7600);
        if (scenario == 6U)
        {
            tx_ready = 0U;
            Steps(3U);
            Receive_Command_Ack(1U, 5U, startup_id + 1U);
            Receive_Action_Result(1U, 1U, startup_id + 1U);
            Steps(3U);
            assert(!sent_count && !distance_count);
            tx_ready = 1U;
        }
        Wait_For_Count(&sent_count, 1U);
        Check_Sent(0U, JETSON_CAN_POINT_REACHED_ID, 1U);
        const uint32_t id = sent[0].request_id;
        if (scenario == 6U)
        {
            assert(id == initial_search_id + 1U);
            Step();
            assert(!distance_count); /* 首次提交前的同编号回执已被清除。 */
        }
        /* 入等待阶段后下一拍才提交首帧；动作截止从入阶段起算。 */
        const uint32_t started_ms = test_tick - 5U;
        Receive_Action_Result(1U, 1U, id - 1U);
        Receive_Action_Result(2U, 1U, id);
        Step();
        assert(!distance_count);
        if (scenario == 1U)
            Receive_Action_Result(1U, 1U, id); /* 先终态、ACK 丢失。 */
        else
            Receive_Command_Ack(1U, 5U, id); /* 先ACK、终态丢失。 */
        Steps(JETSON_AUTO_REQUEST_RETRY_MS / 5U + 1U);
        assert(sent_count >= 2U && !distance_count);
        assert(sent[1].request_id == id && sent[1].token == 1U);
        if (scenario == 3U || scenario == 4U)
        {
            stopped = 1U;
            test_tick = started_ms + JETSON_AUTO_ACTION_TIMEOUT_MS - 1U;
            Receive_Command_Ack(1U, 5U, id);
            Run_Auto();
            assert(enabled && !distance_count);
            test_tick = started_ms + JETSON_AUTO_ACTION_TIMEOUT_MS;
            if (scenario == 4U)
                Receive_Action_Result(1U, 1U, id);
            Run_Auto();
            Step();
            assert(!enabled && !distance_count);
            continue;
        }
        if (scenario == 5U)
        {
            /* 合法10秒动作不能套用普通请求3.45秒截止。 */
            Steps(1940U);
            assert(enabled && !distance_count && sent_count > 30U);
            for (unsigned i = 0U; i < sent_count; ++i)
                assert(sent[i].request_id == id);
        }
        if (scenario == 7U)
        {
            /* 保位交接后执行满 10 秒动作并收到成功，最长预算内的终态仍须放行。 */
            test_tick = started_ms + 13190U;
            Run_Auto();
            assert(enabled && !distance_count);
        }
        Receive_Command_Ack(1U, 5U, id);
        Receive_Action_Result(1U, scenario == 2U ? 0U : 1U, id);
        Jetson_CAN_Receive(9U, NULL, 0U);
        Receive_Action_Result(1U, 1U, id - 1U);
        if (scenario == 2U)
        {
            stopped = 1U;
            Steps(2U);
            assert(!enabled && !distance_count && disables == 1U);
        }
        else
        {
            stopped = 0U;
            const unsigned before_route = sent_count;
            Wait_For_Count(&sent_count, before_route + 1U);
            Check_Sent(before_route, JETSON_CAN_ARRIVAL_ID, 0U);
            assert(!distance_count);
        }
    }
}
#else
/** @brief 未填写生产参数时保持使能与零输出，仍须监视本机反馈。 */
static void Test_Unconfigured(void)
{
    Reset();
    Receive_Center(JETSON_CAN_TARGET_CENTER_ID, 5000, 8000);
    Steps(80U);
    assert(enabled && enables == 1U && !velocity_requests && !turn_count && !startup_queries && !sent_count);
    motors_ready = 0U;
    stopped = 1U;
    Steps(2U);
    assert(!enabled && disables == 1U);
    motors_ready = 1U;
    Steps(80U);
    assert(!enabled && enables == 1U);
}
#endif

/** @brief 执行配置对应的测试。 @return 通过为 0。 */
int main(void)
{
#ifdef TEST_JETSON_AUTO_CONFIGURED
    Test_Wait_After_Search_Turn();
    Test_CAN_Snapshot_Tick_Order();
    Test_Repeated_Source_Frame();
    Test_Approach_Brakes_And_Verifies();
    Test_Approach_Lost_During_Brake();
    Test_Approach_Retreat_And_Limit();
    Test_Pickup_Lateral_Before_Forward();
    Test_Approach_Axis_Window_Continuity();
    Test_Approach_No_Windup();
    Test_Approach_Anti_Windup();
    Test_Approach_Requires_New_Frame();
    if (JETSON_AUTO_DELIVERY_LIMIT == 1U)
    {
        Test_Transport(1U, 0U);
        Test_Transport(1U, 17U);  /* 失能确认超时。 */
        Test_Transport(1U, 18U);  /* 失能确认前 USB 断开。 */
        puts("PASS: single delivery stops after relocalization and confirmed disable, with fault latching");
    }
    else if (JETSON_AUTO_DELIVERY_LIMIT == 5U)
        Test_Five_Deliveries();
    else
        Test_Consecutive_Transports(0U);
    if (JETSON_AUTO_DELIVERY_LIMIT == 2U)
    {
        Test_Consecutive_Transports(2U);  /* 第二次松爪失败不计数。 */
        Test_Consecutive_Transports(11U); /* 第二次整区复核超时。 */
        Test_Consecutive_Transports(17U); /* 少一轮失能反馈。 */
        Test_Consecutive_Transports(18U); /* 等待失能时 USB 断开。 */
        puts("PASS: left then right, two confirmed deliveries, two search turns and confirmed disable");
    }
    Test_Transport(2U, 0U);
    Test_Transport(1U, 1U);
    Test_Transport(1U, 2U);
    Test_Transport(1U, 3U);
    Test_Transport(1U, 4U);
    Test_Transport(1U, 5U);
    Test_Transport(1U, 6U);
    Test_Transport(1U, 7U);
    Test_Transport(1U, 8U);
    Test_Transport(1U, 9U);
    Test_Transport(1U, 10U);
    Test_Transport(1U, 11U);
    Test_Transport(1U, 12U);
    Test_Transport(2U, 13U);
    Test_Transport(1U, 14U);
    Test_Transport(2U, 15U);
    Test_Transport(1U, 16U);
    Test_Transport(1U, PUSH_RAMP_STUCK);
    Test_Transport(1U, RETRACT_RAMP_STUCK);
    Test_Transport(1U, RETRACT_WHEELS_MOVING);
    Test_Transport(1U, ENCODER_CONFIRMED);
    Test_Return_Recovers_Small_Error();
    Test_Long_Return();
    Test_Return_Stream_Loss();
    Test_Request_Deadline();
    Test_USB_Send_Error();
    Test_Reject_Route();
    Test_Visual_Timeout_Without_Type1();
    Test_Startup_Link_Gate();
    Test_Startup_CAN_Fault();
    Test_Periodic_Stream_Timeout();
    Test_USB_Disconnect_And_Reconnect();
    Test_Startup_Stream_Loss();
    Test_Startup_Action();
    Test_Initial_Alignment();
    Test_Initial_Camera_Warmup();
    Test_Localize_Lateral_Deadband();
    Test_Action_Transactions();
    puts("PASS: Jetson auto transport, action results, live target loss, USB sessions and periodic-stream fail-safe");
#else
    Test_Unconfigured();
    puts("PASS: unconfigured Jetson auto holds enabled at zero, blocks all mission actions and latches hardware faults");
#endif
    return 0;
}
