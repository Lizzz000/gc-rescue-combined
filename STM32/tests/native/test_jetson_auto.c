#include <assert.h>
#include <math.h>
#include <stdio.h>
#include "INS_task.h"
#include "bsp_can.h"
#include "chassis.h"
#include "jetson_auto.h"
#include "jetson_can.h"
#include "jetson_control.h"
#include "task.h"
#include "transport_servo.h"

/* 真实下行解析与自主状态机，反馈、机构及 CAN 发送由本测试控制。 */
CAN_HandleTypeDef hcan1, hcan2;
uint32_t test_tick;
static unsigned critical_depth;
static uint8_t can_arrival_before_snapshot;
static uint8_t enabled, stopped;
static unsigned enables, stops, disables, velocity_requests;
static uint8_t tx_ready;
static unsigned distance_count, turn_count, sent_count;
static Chassis_Translation_Direction_t last_translation;
static float last_distance;
static int8_t last_turn_direction;
static float last_turn_angle, last_vx, last_vy, last_wz;
static uint8_t servo_available;
static uint8_t motors_ready;
static unsigned servo_commands;
static uint32_t servo_started, imu_age_ms;
static Transport_Servo_Position_t servo_position;

typedef struct
{
    uint16_t id;
    uint32_t token;
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
        const uint8_t heartbeat[7] = {0U};
        const uint8_t target[8] = {1U, 0x88U, 0x13U, 0xA0U, 0x0FU, (uint8_t)test_tick, (uint8_t)(test_tick >> 8U), (uint8_t)(test_tick >> 16U)}; /* (5000,4000) */
        Jetson_CAN_Receive(0x180U, heartbeat, sizeof(heartbeat));
        Jetson_CAN_Receive(0x181U, target, sizeof(target));
    }
    ++critical_depth;
}
/** @brief 验证临界区配对退出。 */
void test_exit_critical(void) { assert(critical_depth > 0U); --critical_depth; }

/** @brief 模拟软件受理使能，停车反馈由场景单独提供。 @return 1。 */
uint8_t Chassis_Enable(void)
{
    enabled = 1U;
    ++enables;
    return 1U;
}

/** @brief 模拟请求失能及撤销停车反馈。 */
void Chassis_Disable(void)
{
    enabled = stopped = 0U;
    ++disables;
}

/** @brief 记录停车请求；请求本身不生成停车反馈。 */
void Chassis_Stop(void) { ++stops; }
/** @brief 读取模拟底盘使能状态。 @return 软件已使能为 1。 */
uint8_t Chassis_Is_Enabled(void) { return enabled; }
/** @brief 读取场景独立给出的停车反馈。 @return 已使能且确认停车为 1。 */
uint8_t Chassis_Is_Stopped(void) { return enabled && stopped; }

/** @brief 提供可单独断开的三轮反馈条件。 @return 反馈正常为 1。 */
uint8_t Jetson_Control_Motors_Ready(void) { return motors_ready; }

/** @brief 提供可独立设置样本年龄的姿态。 @param[out] data 姿态快照。 @return 有效为 1。 */
uint8_t INS_Get_Data(INS_control_data_t *data)
{
    *data = (INS_control_data_t){0};
    data->sample_time_ms = test_tick - imu_age_ms;
    return 1U;
}

/** @brief 捕获自动导航速度。 @return 模拟受理为 1。
 * @param[in] vx 前向 m/s。 @param[in] vy 左向 m/s。 @param[in] wz 左转 rad/s。
 */
uint8_t Chassis_Set_Velocity(float vx, float vy, float wz)
{
    assert(enabled && isfinite(vx) && isfinite(vy) && isfinite(wz));
    ++velocity_requests;
    last_vx = vx;
    last_vy = vy;
    last_wz = wz;
    if (vx != 0.0f || vy != 0.0f || wz != 0.0f)
        stopped = 0U;
    return 1U;
}

/** @brief 捕获定距请求，清除先前停车反馈。 @return 模拟受理为 1。
 * @param[in] direction 平移方向。 @param[in] distance 规划米数。 @param[in] speed 车体速度上限 m/s。
 */
uint8_t Chassis_Distance_Start(Chassis_Translation_Direction_t direction, float distance, float speed)
{
    assert(enabled && stopped && distance > 0.0f && speed == JETSON_AUTO_DISTANCE_SPEED_MPS);
    last_translation = direction;
    last_distance = distance;
    ++distance_count;
    stopped = 0U;
    return 1U;
}

/** @brief 捕获定角请求，清除先前停车反馈。 @return 模拟受理为 1。
 * @param[in] direction 左 +1、右 -1。 @param[in] angle_deg 相对角度，度。
 */
uint8_t Chassis_Turn_Start(int8_t direction, float angle_deg)
{
    assert(enabled && stopped && (direction == -1 || direction == 1) && angle_deg > 0.0f);
    last_turn_direction = direction;
    last_turn_angle = angle_deg;
    ++turn_count;
    stopped = 0U;
    return 1U;
}

/** @brief 模拟板卡已完成舵机 PWM 配置。 @return 可用为 1。 */
uint8_t Transport_Servo_Is_Available(void) { return servo_available; }

/** @brief 记录舵机请求；不制造到位或送达反馈。 @return 模拟受理为 1。
 * @param[in] position 请求位置。 @param[in] now_ms 当前模拟毫秒计数。
 */
uint8_t Transport_Servo_Command(Transport_Servo_Position_t position, uint32_t now_ms)
{
    assert(servo_available && Chassis_Is_Stopped() && now_ms == test_tick);
    if (servo_commands == 0U || position != servo_position)
    {
        servo_position = position;
        servo_started = now_ms;
    }
    ++servo_commands;
    return 1U;
}

/** @brief 返回上次切换目标后经过的时间。
 * @param[in] now_ms 模拟当前时刻。 @param[out] elapsed_ms 经过时间 ms。
 * @return 有有效指令且输出非空为 1，否则为 0。
 */
uint8_t Transport_Servo_Elapsed(uint32_t now_ms, uint32_t *elapsed_ms)
{
    if (servo_commands == 0U || elapsed_ms == NULL)
        return 0U;
    *elapsed_ms = now_ms - servo_started;
    return 1U;
}

/** @brief 捕获自主层提交的上行请求。 @return HAL_OK 模拟入队，HAL_BUSY 表示邮箱未就绪。
 * @param[in] id 已有运输协议上行 ID。 @param[in] token 对应本趟身份。
 */
HAL_StatusTypeDef Jetson_CAN_Try_Send_Uplink(uint16_t id, uint32_t token)
{
    assert(critical_depth > 0U);
    if (!tx_ready)
        return HAL_BUSY;
    assert(sent_count < sizeof(sent) / sizeof(sent[0]));
    uint8_t payload[8];
    const uint8_t len = Jetson_CAN_Encode_Uplink(id, token, payload);
    assert(len == 1U || len == 5U);
    assert(payload[0] == 1U);
    const uint32_t encoded = len == 1U ? 0U : (uint32_t)payload[1] |
        (uint32_t)payload[2] << 8U | (uint32_t)payload[3] << 16U | (uint32_t)payload[4] << 24U;
    sent[sent_count++] = (Sent_Frame){id, encoded};
    return HAL_OK;
}

/** @brief 初始化独立场景，启用替身外设能力，底盘保持失能。 */
static void Reset(void)
{
    test_tick = 100U;
    critical_depth = 0U;
    can_arrival_before_snapshot = 0U;
    enabled = stopped = 0U;
    enables = stops = disables = velocity_requests = 0U;
    distance_count = turn_count = sent_count = servo_commands = 0U;
    imu_age_ms = 0U;
    servo_available = tx_ready = motors_ready = 1U;
    last_turn_direction = 0;
    last_turn_angle = last_vx = last_vy = last_wz = 0.0f;
    last_distance = 0.0f;
    last_translation = CHASSIS_TRANSLATION_NONE;
    Jetson_CAN_Init();
    Jetson_Auto_Init();
}

/** @brief 推进一拍 5 ms。
 * @param[in] heartbeat 1 注入合法 0x180 传输心跳，0 模拟 CAN 断流。
 */
static void Step(uint8_t heartbeat)
{
    test_tick += 5U;
    const uint8_t bytes[7] = {0};
    if (heartbeat)
        Jetson_CAN_Receive(0x180U, bytes, sizeof(bytes));
    Jetson_Auto_Loop();
    assert(critical_depth == 0U);
}

/** @brief 按 5 ms 推进数拍。 @param[in] count 拍数。 @param[in] heartbeat 是否注入心跳。 */
static void Steps(unsigned count, uint8_t heartbeat)
{
    for (unsigned i = 0U; i < count; ++i)
        Step(heartbeat);
}

/** @brief 注入有效中心。 @param[in] id 目标或安全区 ID。
 * @param[in] x 横向归一化值乘 10000。 @param[in] y 纵向归一化值乘 10000。
 */
static void Receive_Center(uint16_t id, int16_t x, int16_t y)
{
    const uint8_t bytes[] = {1U, (uint8_t)x, (uint8_t)((uint16_t)x >> 8U),
                            (uint8_t)y, (uint8_t)((uint16_t)y >> 8U),
                            (uint8_t)test_tick, (uint8_t)(test_tick >> 8U), (uint8_t)(test_tick >> 16U)};
    Jetson_CAN_Receive(id, bytes, sizeof(bytes));
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
    Jetson_CAN_Receive(0x190U, &accepted, 1U);
}

/** @brief 注入 ROI 判定。 @param[in] passed 1 为通过。 @param[in] destination 左 1、右 2。 */
static void Receive_Roi(uint8_t passed, uint8_t destination)
{
    const uint8_t bytes[] = {1U, passed, destination};
    Jetson_CAN_Receive(0x191U, bytes, sizeof(bytes));
}

/** @brief 注入本趟转向授权。 @param[in] destination 左 1、右 2。 @param[in] token 本趟身份。 */
static void Receive_Turn(uint8_t destination, uint32_t token)
{
    uint8_t bytes[] = {1U, destination, 0U, 0U, 0U, 0U};
    Put_Token(bytes + 2, token);
    Jetson_CAN_Receive(0x193U, bytes, sizeof(bytes));
}

/** @brief 注入运输命令回执。
 * @param[in] accepted 1 为受理。 @param[in] command 回执命令编号。 @param[in] token 对应身份。
 */
static void Receive_Command_Ack(uint8_t accepted, uint8_t command, uint32_t token)
{
    uint8_t bytes[] = {accepted, command, 0U, 0U, 0U, 0U};
    Put_Token(bytes + 2, token);
    Jetson_CAN_Receive(0x192U, bytes, sizeof(bytes));
}

/** @brief 核对指定上行事件的 ID 及身份。帧字节布局由 CAN 模块单测核对。
 * @param[in] index 发送顺序索引。 @param[in] id 预期标准 ID。 @param[in] token 预期本趟身份。
 */
static void Check_Sent(unsigned index, uint16_t id, uint32_t token)
{
    assert(index < sent_count && sent[index].id == id && sent[index].token == token);
}

/** @brief 给状态转换至多 100 ms。 @param[in] counter 只由测试替身更新的计数。 @param[in] count 最低值。 */
static void Wait_For_Count(const unsigned *counter, unsigned count)
{
    for (unsigned i = 0U; *counter < count && i < 20U; ++i)
        Step(1U);
    assert(*counter >= count);
}

/** @brief 模拟一次有效心跳开机，并确认停车反馈前不启动舵机或导航。 */
static void Start_Automatic(void)
{
    Steps(3U, 0U);
    const uint8_t malformed[6] = {0};
    Jetson_CAN_Receive(0x180U, malformed, sizeof(malformed));
    Steps(3U, 0U);
    assert(!enabled && !enables && !servo_commands && !velocity_requests);

    imu_age_ms = 21U;
    Steps(3U, 1U);
    assert(!enabled && !enables && !servo_commands && !velocity_requests);
    imu_age_ms = 0U;
    Wait_For_Count(&enables, 1U);
    Steps(3U, 1U);
    assert(enabled && !servo_commands && !velocity_requests);
    stopped = 1U;
    Wait_For_Count(&servo_commands, 1U);
    assert(servo_position == TRANSPORT_SERVO_OPEN);
}

/** @brief 开机后等到自主模式进入寻找目标。 */
static void Begin_Target_Test(void)
{
    Reset();
    Start_Automatic();
    Wait_For_Count(&velocity_requests, 1U);
    assert(last_vx == 0.0f && last_vy == 0.0f && last_wz != 0.0f);
}

/** @brief 从刚确认停稳的一拍推进到核验截止，模拟等待期间晚到的两个窗内中心。
 * @param[in] id 物资或安全区中心 CAN ID。
 */
static void Reach_Verify_Cutoff(uint16_t id)
{
    Receive_Center(id, 5000, 7500);
    Steps((JETSON_AUTO_VERIFY_WAIT_MS - 1U) / 5U, 1U);
    Receive_Center(id, 5000, 7500);
    Step(1U);
}

/** @brief 在 CAN 快照前跨一个 RTOS tick，包含计数回绕，保持健康链路和追踪。 */
static void Test_CAN_Snapshot_Tick_Order(void)
{
    for (unsigned wrap = 0U; wrap < 2U; ++wrap)
    {
        Begin_Target_Test();
        Receive_Center(0x181U, 5000, 4000);
        Step(1U);
        assert(last_vx > 0.0f && last_wz == 0.0f);
        if (wrap)
            test_tick = UINT32_MAX - 5U;
        can_arrival_before_snapshot = 1U;
        Step(0U);
        assert(!can_arrival_before_snapshot && (!wrap || test_tick == 0U));
        Steps(2U, 1U);
        assert(enabled && !stops && !disables && last_vx > 0.0f && last_wz == 0.0f);
    }
}

/** @brief 同一源帧的 CAN 重发不能重复更新跟随目标。 */
static void Test_Repeated_Source_Frame(void)
{
    Begin_Target_Test();
    const uint32_t source = test_tick;
    const uint8_t frame[8] = {1U, 0x38U, 0x18U, 0x4cU, 0x1dU,
                              (uint8_t)source, (uint8_t)(source >> 8U), (uint8_t)(source >> 16U)};
    Jetson_CAN_Receive(0x181U, frame, sizeof(frame));
    Step(1U);
    const unsigned before = velocity_requests;
    Jetson_CAN_Receive(0x181U, frame, sizeof(frame));
    Step(1U);
    assert(velocity_requests == before);
    Receive_Center(0x181U, 6200, 7500);
    Step(1U);
    assert(velocity_requests == before + 1U);
}

/** @brief 验证预测进窗时先减速，真实入窗才停车并核验新帧。 */
static void Test_Approach_Brakes_And_Verifies(void)
{
    Begin_Target_Test();
    Receive_Center(0x181U, 6200, 7500);
    Steps(4U, 1U);
    assert(fabsf(last_vy - 0.088f) < 1e-6f && last_vx == 0.0f);
    for (unsigned i = 0U; i < 3U; ++i)
    {
        /* 以 20 ms 重报验证旧速率兼容；5 ms 任务轮询不视为新测量。 */
        Receive_Center(0x181U, 6200, 7500);
        Steps(4U, 1U);
        assert(stops == 0U && last_vy >= 0.088f && last_vy <= 0.10f);
    }
    const unsigned before_slow = velocity_requests;
    Receive_Center(0x181U, 5400, 7500);
    Step(1U);
    assert(stops == 0U && velocity_requests == before_slow + 1U && servo_commands == 1U);
    assert(last_vx == 0.0f && last_vy > 0.0f && last_vy < 0.03f && last_wz == 0.0f);
    const unsigned moves = velocity_requests;

    Receive_Center(0x181U, 5000, 7500);
    Steps(5U, 1U);
    stopped = 1U;
    Step(1U);
    Reach_Verify_Cutoff(0x181U);
    assert(stops == 1U && velocity_requests == moves && servo_commands == 1U);

    /* 停稳后漂到 5154 已超出 ±100，恢复连续跟随，不视为到位。 */
    Receive_Center(0x181U, 5154, 7500);
    Step(1U);
    assert(last_vx == 0.0f && fabsf(last_vy - 0.005f) < 1e-6f && last_wz == 0.0f);
    assert(stops == 1U && velocity_requests == moves + 1U && servo_commands == 1U);
    Steps(20U, 1U);
    assert(stops == 1U && fabsf(last_vy - 0.005f) < 1e-6f);
    Receive_Center(0x181U, 5154, 7500);
    Step(1U);
    assert(last_vy >= 0.005f && stops == 1U && servo_commands == 1U);
    Receive_Center(0x181U, 5100, 7400);
    Step(1U);
    assert(stops == 2U && servo_commands == 1U);

    stopped = 1U;
    Step(1U);
    Reach_Verify_Cutoff(0x181U);
    assert(servo_commands == 1U);
    /* 复核阶段若三轮重新滚动，下一次停稳须重算收帧边界。 */
    stopped = 0U;
    Step(1U);
    stopped = 1U;
    Step(1U);
    Reach_Verify_Cutoff(0x181U);
    assert(servo_commands == 1U);
    Receive_Center(0x181U, 5100, 7400);
    Step(1U);
    assert(servo_commands == 2U && servo_position == TRANSPORT_SERVO_CLOSED);
}

/** @brief 目标在制动或停稳核验时丢失，停稳并确认失效后恢复搜索。 */
static void Test_Approach_Lost_During_Brake(void)
{
    for (unsigned timeout = 0U; timeout < 2U; ++timeout)
    {
        Begin_Target_Test();
        Receive_Center(0x181U, 5800, 7500);
        Step(1U);
        Receive_Center(0x181U, 5000, 7500);
        Step(1U);
        const unsigned moves = velocity_requests;
        const uint8_t invalid_center[5] = {0};
        Jetson_CAN_Receive(0x181U, invalid_center, sizeof(invalid_center));
        Steps(5U, 1U);
        stopped = 1U;
        Step(1U);
        assert(stops == 1U && velocity_requests == moves && servo_commands == 1U);

        if (timeout)
            Steps((JETSON_AUTO_VERIFY_WAIT_MS + 4U) / 5U, 1U);
        else
        {
            Reach_Verify_Cutoff(0x181U);
            assert(velocity_requests == moves && servo_commands == 1U);
            Jetson_CAN_Receive(0x181U, invalid_center, sizeof(invalid_center));
            Step(1U);
        }
        Steps(4U, 1U);
        assert(stops == 1U && velocity_requests == moves + 1U && servo_commands == 1U);
        assert(last_vx == 0.0f && last_vy == 0.0f && last_wz != 0.0f);
    }
}

/** @brief 远离设定点的速度不削弱纠正，远段对角合速保持限幅。 */
static void Test_Approach_Retreat_And_Limit(void)
{
    Begin_Target_Test();
    Receive_Center(0x181U, 5800, 7500);
    Steps(16U, 1U);
    Receive_Center(0x181U, 6200, 7500);
    Step(1U);
    assert(stops == 0U && last_vy > 0.064f && last_vy <= JETSON_AUTO_APPROACH_SPEED_MPS);
    Receive_Center(0x181U, 8500, 5000);
    Step(1U);
    assert(last_vx > 0.0f && last_vy > 0.0f &&
           fabsf(hypotf(last_vx, last_vy) - JETSON_AUTO_APPROACH_SPEED_MPS) < 1e-6f);
}

/** @brief 横向掠过到位窗边界时保持平滑，纵向仍持续靠近。 */
static void Test_Approach_Axis_Window_Continuity(void)
{
    Begin_Target_Test();
    Receive_Center(0x181U, 5101, 7000);
    Step(1U);
    const float outside = last_vy;
    const unsigned updates = velocity_requests;
    Steps(3U, 1U);
    assert(velocity_requests == updates);
    Receive_Center(0x181U, 5099, 7000);
    Step(1U);
    assert(last_vx > 0.0f && last_vy == 0.0f && outside > 0.0f && outside <= JETSON_AUTO_APPROACH_MIN_SPEED_MPS);
    assert(stops == 0U && velocity_requests == updates + 1U);
    Receive_Center(0x181U, 5050, 7000);
    Step(1U);
    assert(last_vx > 0.0f && last_vy == 0.0f && stops == 0U);
}

/** @brief 验证低速连续增力、按到达间隔积分、上限、反向和入窗清零。 */
static void Test_Approach_Integral(void)
{
    Begin_Target_Test();
    Receive_Center(0x181U, 5300, 7500);
    Step(1U);
    assert(fabsf(last_vy - 0.016f) < 1e-6f && stops == 0U);
    Steps(7U, 1U);
    assert(fabsf(last_vy - 0.016f) < 1e-6f);
    Receive_Center(0x181U, 5300, 7500);
    Step(1U);
    assert(fabsf(last_vy - 0.0176f) < 1e-6f);
    for (unsigned i = 0U; i < 75U; ++i)
    {
        Steps(3U, 1U);
        Receive_Center(0x181U, 5300, 7500);
        Step(1U);
    }
    assert(fabsf(last_vy - (0.016f + JETSON_AUTO_APPROACH_I_LIMIT_MPS)) < 1e-6f);
    assert(stops == 0U && servo_commands == 1U);

    /* 只差一个定点单位时，即便历史积分已饱和，仍保留小幅同向修正。 */
    Receive_Center(0x181U, 5101, 7500);
    Step(1U);
    assert(last_vy == JETSON_AUTO_APPROACH_MIN_SPEED_MPS && stops == 0U);

    Receive_Center(0x181U, 3500, 7500);
    Step(1U);
    assert(last_vy < 0.0f && stops == 0U);
}

/** @brief 验证远段合速度饱和时不会累积靠近积分。 */
static void Test_Approach_Anti_Windup(void)
{
    Begin_Target_Test();
    Receive_Center(0x181U, 6500, 7500);
    Steps(4U, 1U);
    for (unsigned i = 0U; i < 39U; ++i)
    {
        Receive_Center(0x181U, 6500, 7500);
        Steps(4U, 1U);
        assert(fabsf(last_vy - JETSON_AUTO_APPROACH_SPEED_MPS) < 1e-6f);
    }
    Receive_Center(0x181U, 6100, 7500);
    Step(1U);
    assert(last_vy > 0.0f && last_vy < 0.099f && stops == 0U);
}

/** @brief 轮速未达到停车确认条件时，按视觉停车期限失能，不额外延长故障制动。 */
static void Test_Approach_Brake_Timeout(void)
{
    Begin_Target_Test();
    Receive_Center(0x181U, 5000, 7500);
    Step(1U);
    assert(stops == 1U && enabled && servo_commands == 1U);
    Steps(JETSON_AUTO_VISION_STOP_TIMEOUT_MS / 5U - 1U, 1U);
    assert(enabled && stops == 1U && servo_commands == 1U);
    Steps(2U, 1U);
    assert(!enabled && disables == 1U && servo_commands == 1U && sent_count == 0U);
}

/** @brief 从开机推进到夹爪闭合、停车后申请 ROI。
 * @param[in] destination 夹具用于旧回复的目的区。 @param[in] token 夹具用于旧授权的身份。
 */
static void Prepare_ROI(uint8_t destination, uint32_t token)
{
    Begin_Target_Test();

    /* 横向超出容差时继续修正；双轴同时入窗后等待停车反馈。 */
    Receive_Center(0x181U, 5800, 7500);
    Step(1U);
    assert(last_vx == 0.0f && last_vy > 0.0f && last_wz == 0.0f);
    Receive_Center(0x181U, 5000, 7500);
    Steps(4U, 1U);
    assert(stops && servo_position == TRANSPORT_SERVO_OPEN && sent_count == 0U);
    stopped = 1U;
    Step(1U);
    Reach_Verify_Cutoff(0x181U);
    assert(servo_position == TRANSPORT_SERVO_OPEN);
    Receive_Center(0x181U, 4900, 7600);
    for (unsigned i = 0U; servo_position == TRANSPORT_SERVO_OPEN && i < 20U; ++i)
        Step(1U);
    assert(servo_position == TRANSPORT_SERVO_CLOSED && !distance_count && !sent_count);
    Wait_For_Count(&distance_count, 1U);
    assert(last_translation == CHASSIS_TRANSLATION_BACKWARD && !sent_count);
    assert(last_distance == JETSON_AUTO_ROI_BACK_M);
    assert(test_tick - servo_started >= JETSON_AUTO_SERVO_CLOSE_MS);
    Steps(4U, 1U);
    assert(!sent_count);

    /* 邮箱忙期间的旧回包不得在发出 200 后释放转向。 */
    tx_ready = 0U;
    stopped = 1U;
    Steps(3U, 1U);
    Receive_Arrival_Ack(1U);
    Receive_Roi(1U, destination);
    Receive_Turn(destination, token ^ 1U);
    Steps(3U, 1U);
    assert(!sent_count && !turn_count);
    tx_ready = 1U;
    Wait_For_Count(&sent_count, 1U);
    Check_Sent(0U, 0x200U, 0U);
    Steps(3U, 1U);
    assert(!turn_count);
}

/** @brief 从开机验证完整运输及匹配身份。
 * @param[in] destination 左 1、右 2。 @param[in] token 使用无符号完整 32 位。
 */
static void Test_Transport(uint8_t destination, uint32_t token, uint8_t stuck)
{
    Prepare_ROI(destination, token);
    Receive_Arrival_Ack(1U);
    Receive_Turn(destination, token);
    Steps(3U, 1U);
    assert(!turn_count);
    Receive_Roi(1U, destination);
    Wait_For_Count(&turn_count, 1U);
    assert(last_turn_direction == (destination == 1U ? 1 : -1) && last_turn_angle == 180.0f);
    Receive_Turn(destination, token);
    Steps(4U, 1U);
    assert(sent_count == 1U && turn_count == 1U);
    stopped = 1U;
    Wait_For_Count(&sent_count, 2U);
    Check_Sent(1U, 0x203U, token);

    /* 错误身份不能令安全区到达或启动推送。 */
    Receive_Command_Ack(1U, 1U, token ^ 1U);
    Receive_Center(0x182U, 5000, 7500);
    Steps(4U, 1U);
    assert(sent_count == 2U && distance_count == 1U);
    Receive_Command_Ack(1U, 1U, token);
    /* 同一任务拍的其他命令、旧 token、拒绝及坏帧均不得抹掉已收到的受理。 */
    Receive_Command_Ack(1U, 2U, token);
    Receive_Command_Ack(1U, 1U, token ^ 1U);
    Receive_Command_Ack(0U, 1U, token);
    Jetson_CAN_Receive(0x192U, NULL, 0U);
    Step(1U);
    /* 安全区仅在窗沿来回波动时维持微调，避免每次进窗都突然清零。 */
    const unsigned stops_before_alignment = stops;
    for (unsigned i = 0U; i < 4U; ++i)
    {
        Receive_Center(0x182U, i % 2U == 0U ? 5101 : 5099, 7500);
        Steps(4U, 1U);
        assert(last_vy >= 0.0f && stops == stops_before_alignment && sent_count == 2U);
    }
    if (destination == 2U)
        test_tick = UINT32_MAX - 100U; /* 右区复核等待跨过 RTOS 毫秒回绕。 */
    Receive_Center(0x182U, 5000, 7500);
    Step(1U);
    stopped = 1U;
    Step(1U);
    Reach_Verify_Cutoff(0x182U);
    assert(sent_count == 2U);
    /* 两次同轴同向越界才微调，期间另一轴越界不覆盖已有证据。 */
    const unsigned alignment_updates = velocity_requests;
    Receive_Center(0x182U, 5105, 7595);
    Steps(3U, 1U);
    assert(sent_count == 2U && velocity_requests == alignment_updates);
    Receive_Center(0x182U, 5095, destination == 1U ? 7500 : 7605);
    Step(1U);
    assert(sent_count == 2U && velocity_requests == alignment_updates);
    Receive_Center(0x182U, 5105, 7595);
    Step(1U);
    assert(sent_count == 2U && velocity_requests == alignment_updates + 1U && last_vy > 0.0f);
    Receive_Center(0x182U, 5000, 7500);
    Step(1U);
    stopped = 1U;
    Step(1U);
    Reach_Verify_Cutoff(0x182U);
    assert(sent_count == 2U);
    /* 同一坐标快照的多次轮询不能充当第二个确认样本。 */
    const uint32_t repeated_source = test_tick;
    Receive_Center(0x182U, 5000, 7500);
    Steps(4U, 1U);
    assert(sent_count == 2U);
    const uint8_t repeated[8] = {1U, 0x88U, 0x13U, 0x4cU, 0x1dU,
        (uint8_t)repeated_source, (uint8_t)(repeated_source >> 8U), (uint8_t)(repeated_source >> 16U)};
    Jetson_CAN_Receive(0x182U, repeated, sizeof(repeated));
    Step(1U);
    assert(sent_count == 2U);
    Receive_Center(0x182U, 5000, 7500);
    Wait_For_Count(&sent_count, 3U);
    Check_Sent(2U, 0x204U, token);
    assert(distance_count == 1U);

    Receive_Command_Ack(1U, 2U, token);
    const unsigned before_push = velocity_requests;
    Wait_For_Count(&velocity_requests, before_push + 1U);
    assert(fabsf(last_vx - JETSON_AUTO_PUSH_SPEED_MPS) < 1e-6f && last_vy == 0.0f && last_wz == 0.0f);
    const unsigned before_push_stop = stops;
    Steps(JETSON_AUTO_PUSH_DURATION_MS / 5U - 1U, 1U);
    assert(stops == before_push_stop && servo_position == TRANSPORT_SERVO_CLOSED);
    Step(1U);
    assert(stops == before_push_stop + 1U && servo_position == TRANSPORT_SERVO_CLOSED);
    if (stuck)
    {
        Steps((2U * JETSON_BRAKE_TIMEOUT_MS) / 5U + 1U, 1U);
        assert(!enabled && disables == 1U && servo_position == TRANSPORT_SERVO_CLOSED);
        return;
    }
    stopped = 1U;
    Step(1U);
    assert(servo_position == TRANSPORT_SERVO_OPEN && distance_count == 1U && sent_count == 3U);
    const unsigned before_retract = velocity_requests;
    Wait_For_Count(&velocity_requests, before_retract + 1U);
    assert(fabsf(last_vx + JETSON_AUTO_PUSH_SPEED_MPS) < 1e-6f && last_vy == 0.0f && last_wz == 0.0f);
    assert(test_tick - servo_started >= JETSON_AUTO_SERVO_OPEN_MS);
    const unsigned before_retract_stop = stops;
    Steps(JETSON_AUTO_PUSH_DURATION_MS / 5U - 1U, 1U);
    assert(stops == before_retract_stop && sent_count == 3U);
    Step(1U);
    assert(stops == before_retract_stop + 1U && sent_count == 3U);
    stopped = 1U;
    Wait_For_Count(&sent_count, 4U);
    Check_Sent(3U, 0x206U, token);

    Receive_Command_Ack(1U, 4U, token ^ 1U);
    const unsigned servo_commands_after_release = servo_commands;
    Steps(JETSON_AUTO_REQUEST_RETRY_MS / 5U + 1U, 1U);
    assert(turn_count == 1U && distance_count == 1U && servo_commands == servo_commands_after_release);
    assert(sent_count == 5U);
    Check_Sent(4U, 0x206U, token);
    Receive_Command_Ack(1U, 4U, token);
    Wait_For_Count(&turn_count, 2U);
    assert(last_turn_direction == (destination == 1U ? -1 : 1) && last_turn_angle == 180.0f);
    assert(sent_count == 5U);
    Steps(3U, 1U);
    assert(sent_count == 5U);
    stopped = 1U;
    Wait_For_Count(&sent_count, 6U);
    Check_Sent(5U, 0x205U, 0U);

    const unsigned velocities_before = velocity_requests;
    Steps(3U, 1U);
    assert(velocity_requests == velocities_before);
    Receive_Command_Ack(1U, 3U, 0U);
    Step(1U);
    Receive_Center(0x181U, 5000, 4000);
    Wait_For_Count(&velocity_requests, velocities_before + 1U);
    assert(last_vx != 0.0f || last_vy != 0.0f || last_wz != 0.0f);
    assert(distance_count == 1U && turn_count == 2U);
}

/** @brief 进入转向完成握手并暂时阻塞发送邮箱。
 * @param[in] token 本趟路由号。
 * @param[in] wrap 1 表示让回执等待跨过 RTOS 毫秒回绕。
 */
static void Prepare_Turn_Wait(uint32_t token, uint8_t wrap)
{
    Prepare_ROI(1U, token);
    Receive_Arrival_Ack(1U);
    Receive_Roi(1U, 1U);
    Receive_Turn(1U, token);
    Wait_For_Count(&turn_count, 1U);

    stopped = 1U;
    tx_ready = 0U;
    if (wrap)
        test_tick = UINT32_MAX - 3200U;
    Step(1U);
    assert(sent_count == 1U);
}

/** @brief 核对邮箱等待、及时但迟处理的回执、逾期回执及重报期限。 */
static void Test_Request_Deadline(void)
{
    const uint32_t token = 0x12345678U;
    Prepare_Turn_Wait(token, 0U);
    const unsigned stops_before_mailbox_timeout = stops;
    Steps(JETSON_AUTO_REQUEST_TIMEOUT_MS / 5U - 1U, 1U);
    assert(enabled && sent_count == 1U && stops == stops_before_mailbox_timeout);
    tx_ready = 1U;
    Step(1U);
    assert(sent_count == 1U && stops == stops_before_mailbox_timeout + 1U);

    for (unsigned test = 0U; test < 3U; ++test)
    {
        const uint8_t timely = test != 1U;
        Prepare_Turn_Wait(token, test == 2U);
        Steps(100U, 1U);
        tx_ready = 1U;
        Step(1U);
        Check_Sent(1U, 0x203U, token);
        const uint32_t first_tx_ms = test_tick;
        const unsigned stops_before_reply = stops;
        const uint32_t budget_ms =
            JETSON_AUTO_REQUEST_TIMEOUT_MS + JETSON_AUTO_REQUEST_LINK_MARGIN_MS;
        Steps(budget_ms / 5U - 1U, 1U);
        const unsigned before_final = sent_count;
        assert(enabled && stops == stops_before_reply && before_final > 2U);

        /* 回执由 ISR 到达。本轮任务恰在截止处理，或者看到已经逾期的回执。 */
        test_tick = first_tx_ms + budget_ms - (timely ? 1U : 0U);
        Receive_Command_Ack(1U, 1U, token);
        test_tick = first_tx_ms + budget_ms;
        /* 截止前已收到的成功回执不能被这条逾期重复覆盖。 */
        Receive_Command_Ack(1U, 1U, token);
        Jetson_Auto_Loop();
        assert(sent_count == before_final);
        if (timely)
        {
            const unsigned before_velocity = velocity_requests;
            Receive_Center(0x182U, 5600, 7000);
            Step(1U);
            assert(enabled && stops == stops_before_reply && velocity_requests == before_velocity + 1U);
        }
        else
        {
            assert(stops == stops_before_reply + 1U);
            Step(1U);
            assert(!enabled);
        }
    }
}

/** @brief ROI 拒绝时先张爪、再退离并停稳，随后才发 205。 */
static void Test_Reject_And_Resume(void)
{
    Prepare_ROI(1U, 8U);
    Receive_Arrival_Ack(1U);
    Receive_Roi(0U, 0U);
    for (unsigned i = 0U; servo_position == TRANSPORT_SERVO_CLOSED && i < 20U; ++i)
        Step(1U);
    assert(servo_position == TRANSPORT_SERVO_OPEN && sent_count == 1U && distance_count == 1U);
    Wait_For_Count(&distance_count, 2U);
    assert(last_translation == CHASSIS_TRANSLATION_BACKWARD && !turn_count);
    assert(last_distance == JETSON_AUTO_RECOVERY_BACK_M);
    assert(test_tick - servo_started >= JETSON_AUTO_SERVO_OPEN_MS);
    Steps(3U, 1U);
    assert(sent_count == 1U);
    stopped = 1U;
    Wait_For_Count(&sent_count, 2U);
    Check_Sent(1U, 0x205U, 0U);
    Receive_Command_Ack(1U, 3U, 0U);
    const unsigned velocities_before = velocity_requests;
    Wait_For_Count(&velocity_requests, velocities_before + 1U);
    assert(last_wz != 0.0f && distance_count == 2U && !turn_count);
}

/** @brief 区分目标丢失时主动搜索与通信断流时保护停车。 */
static void Test_Lost_Target_And_Link(void)
{
    Reset();
    Start_Automatic();
    Steps(JETSON_AUTO_SERVO_OPEN_MS / 5U, 1U);
    Receive_Center(0x181U, 5000, 4000);
    Wait_For_Count(&velocity_requests, 1U);
    assert(last_vx > 0.0f);

    const uint8_t invalid_center[5] = {0};
    Jetson_CAN_Receive(0x181U, invalid_center, sizeof(invalid_center));
    Steps(15U, 1U);
    assert(last_vx == 0.0f && last_vy == 0.0f && last_wz != 0.0f);

    const unsigned previous_stops = stops;
    Steps(65U, 0U);
    assert(stops > previous_stops || disables > 0U);
    stopped = 1U;
    Steps(110U, 0U);
    assert(!enabled && disables > 0U && sent_count == 0U);
    Receive_Center(0x181U, 5000, 4000);
    Steps(20U, 1U);
    assert(!enabled && enables == 1U && sent_count == 0U);
}

/** @brief 畸形帧既不立即撤销自动心跳，也不延长最近一条合法心跳的期限。 */
static void Test_Auto_Heartbeat_Age(void)
{
    for (unsigned wrap = 0U; wrap < 2U; ++wrap)
    {
        Begin_Target_Test();
        if (wrap)
            test_tick = UINT32_MAX - 120U;
        Receive_Center(0x181U, 5000, 4000);
        Step(1U);
        assert(last_vx > 0.0f);
        const unsigned before = stops;
        const uint8_t malformed[6] = {0U};
        for (unsigned elapsed = 5U; elapsed <= JETSON_VELOCITY_TIMEOUT_MS; elapsed += 5U)
        {
            test_tick += 5U;
            Jetson_CAN_Receive(0x180U, malformed, sizeof(malformed));
            Jetson_Auto_Loop();
            assert(enabled && stops == before + (elapsed == JETSON_VELOCITY_TIMEOUT_MS));
        }
        stopped = 1U;
        Step(1U);
        assert(!enabled && disables == 1U);
    }
}

/** @brief Jetson 报告任务失联时，下一次调度立即停车。 */
static void Test_Mission_Lost(void)
{
    Begin_Target_Test();
    Receive_Center(0x181U, 5000, 4000);
    Step(1U);
    assert(last_vx > 0.0f);
    const unsigned before = stops;
    const uint8_t unused = 0U;
    Jetson_CAN_Receive(0x180U, &unused, 0U);
    const uint8_t recovered[7] = {0U};
    Jetson_CAN_Receive(0x180U, recovered, sizeof(recovered));
    Step(0U);
    assert(stops == before + 1U && enabled);
    stopped = 1U;
    Step(0U);
    assert(!enabled && disables == 1U);
}
/** @brief 板端缺少舵机外设时，参数有效也不能开始自主移动。 */
static void Test_Servo_Unavailable(void)
{
    Reset();
    servo_available = 0U;
    Jetson_Auto_Init();
    Receive_Center(0x181U, 5000, 8000);
    Steps(20U, 1U);
    assert(!enabled && !enables && !servo_commands && !velocity_requests && !sent_count);
}
#else
/** @brief 未填写生产参数时，即便收到合法心跳和中心也始终不使能。 */
static void Test_Unconfigured(void)
{
    Reset();
    Receive_Center(0x181U, 5000, 8000);
    Steps(80U, 1U);
    assert(!enabled && !enables && !servo_commands && !velocity_requests && !sent_count);
}
#endif

/** @brief 执行配置对应的测试。 @return 通过为 0。 */
int main(void)
{
#ifdef TEST_JETSON_AUTO_CONFIGURED
    Test_CAN_Snapshot_Tick_Order();
    Test_Repeated_Source_Frame();
    Test_Approach_Brakes_And_Verifies();
    Test_Approach_Lost_During_Brake();
    Test_Approach_Retreat_And_Limit();
    Test_Approach_Axis_Window_Continuity();
    Test_Approach_Integral();
    Test_Approach_Anti_Windup();
    Test_Approach_Brake_Timeout();
    Test_Transport(1U, 0x89ABCDEFU, 0U);
    Test_Transport(2U, 0U, 0U);
    Test_Transport(1U, 123U, 1U);
    Test_Request_Deadline();
    Test_Reject_And_Resume();
    Test_Lost_Target_And_Link();
    Test_Auto_Heartbeat_Age();
    Test_Mission_Lost();
    Test_Servo_Unavailable();
    puts("PASS: Jetson auto transport, simulated stop gates, tokens, reacquisition and link loss");
#else
    Test_Unconfigured();
    puts("PASS: unconfigured Jetson auto leaves motors, servo and uplink idle");
#endif
    return 0;
}
