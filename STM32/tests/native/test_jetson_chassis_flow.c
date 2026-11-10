/* 沿用自主层测试替身，只运行真实配置下无夹爪单趟流程。 */
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include "INS_task.h"
#include "bsp_can.h"
#include "chassis.h"
#include "jetson_auto.h"
#include "jetson_can.h"
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
uint8_t Chassis_Motors_Ready(void) { return motors_ready; }

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


/** @brief 校验本轮任何阶段均未请求舵机或定距动作。 */
static void Flow_No_Actuator(void)
{
    assert(servo_commands == 0U && distance_count == 0U);
}

/** @brief 显式启动联调，保持真实配置的未标定机构参数为零。 */
static void Flow_Start(void)
{
    Reset();
    assert(JETSON_AUTO_ENABLED == 0 && JETSON_AUTO_CHASSIS_TEST == 1);
    assert(TRANSPORT_SERVO_OPEN_US == 0U && TRANSPORT_SERVO_CLOSED_US == 0U);
    assert(JETSON_AUTO_ROI_BACK_M == 0.0f);
    servo_available = 0U;
    Steps(5U, 1U);
    assert(!enabled && !enables && jetson_test_start == 0U);
    jetson_test_start = 1U;
    imu_age_ms = 21U;
    Steps(2U, 1U);
    assert(!enabled);
    imu_age_ms = 0U;
    Step(1U);
    assert(enabled && enables == 1U);
    stopped = 1U;
    Step(1U);
    Receive_Center(0x181U, 5000, 4000);
    Step(1U);
    assert(last_vx > 0.0f && last_wz == 0.0f && !stopped);
    Flow_No_Actuator();
}

/** @brief 用新鲜窗内坐标和独立停车反馈推进到上行握手。
 * @param[in] id 目标0x181或安全区0x182。 @param[in] uplink 预期上行ID。
 */
static void Flow_Arrive(uint16_t id, uint16_t uplink)
{
    const unsigned before = sent_count;
    Receive_Center(id, 5000, 7500);
    Step(1U);
    Receive_Center(id, 5000, 7500);
    Step(1U);
    stopped = 1U;
    for (unsigned i = 0U; i < JETSON_AUTO_VERIFY_WAIT_MS / 5U + 20U && sent_count == before; ++i)
    {
        Receive_Center(id, 5000, 7500);
        Step(1U);
    }
    assert(sent_count == before + 1U && sent[before].id == uplink);
    Flow_No_Actuator();
}

/** @brief 核验左右路由、真实握手、到位锁止及机构命令为零。
 * @param[in] destination 左区1或右区2。
 */
static void Flow_One_Shot(uint8_t destination)
{
    const uint32_t token = 0xA13579U;
    Flow_Start();
    Flow_Arrive(0x181U, 0x200U);
    Receive_Arrival_Ack(1U);
    Receive_Roi(1U, destination);
    Receive_Turn(destination, token);
    Step(1U);
    assert(turn_count == 1U && last_turn_direction == (destination == 1U ? 1 : -1));
    assert(last_turn_angle == 180.0f && !stopped);
    Steps(10U, 1U);
    assert(sent_count == 1U);
    stopped = 1U;
    Steps(2U, 1U);
    Check_Sent(1U, 0x203U, token);
    Receive_Command_Ack(1U, 1U, token + 1U);
    Step(1U);
    const unsigned count = velocity_requests;
    Receive_Center(0x182U, 5000, 4000);
    Step(1U);
    assert(velocity_requests == count);
    Receive_Command_Ack(1U, 1U, token);
    Step(1U);
    Receive_Center(0x182U, 5000, 4000);
    Step(1U);
    assert(velocity_requests > count && last_vx > 0.0f && last_wz == 0.0f);
    Flow_Arrive(0x182U, 0x204U);
    Check_Sent(2U, 0x204U, token);
    Receive_Command_Ack(1U, 2U, token);
    Step(1U);
    assert(!enabled && disables == 1U);
    const unsigned after = velocity_requests;
    for (unsigned i = 0U; i < 100U; ++i)
    {
        jetson_test_start = (uint8_t)(i & 1U);
        Receive_Center(0x181U, 5000, 4000);
        Step(1U);
    }
    assert(enables == 1U && velocity_requests == after && sent_count == 3U);
    Flow_No_Actuator();
}

/** @brief ROI拒绝只停车结束，不能执行原搬运恢复的松爪和后退。 */
static void Flow_Rejection(void)
{
    Flow_Start();
    Flow_Arrive(0x181U, 0x200U);
    Receive_Arrival_Ack(1U);
    Receive_Roi(0U, 0U);
    Steps(2U, 1U);
    assert(!enabled && turn_count == 0U);
    jetson_test_start = 0U;
    Step(1U);
    jetson_test_start = 1U;
    Steps(3U, 1U);
    assert(enables == 1U);
    Flow_No_Actuator();
}

/** @brief 丢目标停车等待新帧；链路丢失及启动撤销均结束，且不主动搜索。 */
static void Flow_Interruptions(void)
{
    Flow_Start();
    const unsigned before = stops;
    Steps(55U, 1U);
    assert(stops > before && last_wz == 0.0f && turn_count == 0U);
    stopped = 1U;
    Receive_Center(0x181U, 5000, 4000);
    Step(1U);
    assert(last_vx > 0.0f && !stopped);
    Steps(55U, 0U);
    stopped = 1U;
    Step(0U);
    assert(!enabled);
    Flow_No_Actuator();

    Flow_Start();
    jetson_test_start = 0U;
    Step(1U);
    stopped = 1U;
    Step(1U);
    assert(!enabled);
    jetson_test_start = 1U;
    Steps(5U, 1U);
    assert(enables == 1U);
    Flow_No_Actuator();
}

/** @brief 正常视觉停车允许600ms；真正停稳后仍等待270ms并核对新坐标。 */
static void Flow_Visual_Stop_Wait(void)
{
    Flow_Start();
    Receive_Center(0x181U, 5000, 7500);
    Step(1U);
    const unsigned moves = velocity_requests;
    const unsigned braking_stops = stops;
    for (unsigned i = 0U; i < 120U; ++i)
    {
        Receive_Center(0x181U, 5000, 7500);
        Step(1U);
    }
    assert(enabled && !stopped && !sent_count && stops == braking_stops);
    assert(velocity_requests == moves);
    stopped = 1U;
    Receive_Center(0x181U, 5000, 7500);
    Step(1U);
    for (unsigned i = 0U; i < JETSON_AUTO_VERIFY_WAIT_MS / 5U; ++i)
    {
        Receive_Center(0x181U, 5000, 7500);
        Step(1U);
    }
    assert(enabled && stopped && !sent_count && velocity_requests == moves);
    /* 截止后新图像仍在窗外，必须重新对准，不能使用停车前/等待中的窗内图像。 */
    Receive_Center(0x181U, 5101, 7500);
    Step(1U);
    assert(!stopped && !sent_count && velocity_requests == moves + 1U);
    Flow_Arrive(0x181U, 0x200U);
    Flow_No_Actuator();

    Flow_Start();
    Receive_Center(0x181U, 5000, 7500);
    Step(1U);
    const unsigned stops_before_timeout = stops;
    Steps(120U, 1U);
    assert(enabled && !sent_count && stops == stops_before_timeout);
    Steps(JETSON_AUTO_VISION_STOP_TIMEOUT_MS / 5U - 121U, 1U);
    assert(enabled && !sent_count && stops == stops_before_timeout);
    Step(1U); /* 1000ms：超时Halt仍保留最初视觉制动时刻。 */
    assert(enabled && !sent_count && stops == stops_before_timeout + 1U);
    Step(1U); /* 下一任务拍失能，不再追加500ms故障制动等待。 */
    assert(!enabled && disables == 1U && !sent_count);
    Flow_No_Actuator();
}

/** @brief 90秒总期限覆盖导航和未等到链路的启动等待；到期不可重启。 */
static void Flow_Deadline(void)
{
    Flow_Start();
    test_tick += JETSON_AUTO_CHASSIS_TEST_TIMEOUT_MS;
    Step(1U);
    stopped = 1U;
    Step(1U);
    assert(!enabled);
    Steps(3U, 1U);
    assert(enables == 1U);
    Flow_No_Actuator();

    Reset();
    jetson_test_start = 1U;
    Step(0U);
    test_tick += JETSON_AUTO_CHASSIS_TEST_TIMEOUT_MS;
    Step(0U);
    Steps(3U, 1U);
    assert(enables == 0U);
}

/** @brief 执行底盘单趟联调合同；缺少视觉窗口时必须始终失能。 @return 0通过。 */
int main(void)
{
#ifdef TEST_JETSON_CHASSIS_BAD_WINDOW
    (void)Flow_One_Shot;
    (void)Flow_Rejection;
    (void)Flow_Interruptions;
    (void)Flow_Visual_Stop_Wait;
    (void)Flow_Deadline;
    Reset();
    jetson_test_start = 1U;
    Steps(50U, 1U);
    assert(!enabled && !enables && !velocity_requests && !sent_count);
    Flow_No_Actuator();
    puts("PASS: chassis-only invalid-window gate");
#else
    Flow_One_Shot(1U);
    Flow_One_Shot(2U);
    Flow_Rejection();
    Flow_Interruptions();
    Flow_Visual_Stop_Wait();
    Flow_Deadline();
    puts("PASS: chassis-only gate, routing, handshake, stopping, loss and deadline");
#endif
    return 0;
}
