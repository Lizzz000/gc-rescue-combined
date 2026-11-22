/* 沿用自主层测试替身，只运行真实配置下无夹爪单趟流程。 */
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
static unsigned critical_depth;
static uint8_t can_arrival_before_snapshot;
static uint8_t enabled, stopped, motion_ready;
static unsigned enables, stops, disables, velocity_requests;
static uint8_t tx_ready;
static unsigned distance_count, turn_count, sent_count;
static int8_t last_turn_direction;
static float last_turn_angle, last_vx, last_vy, last_wz, last_distance;
static Chassis_Translation_Direction_t last_distance_direction;
static uint8_t motors_ready;
static uint32_t imu_age_ms, startup_id;
static UsbLink_State_t usb_state;
static uint16_t background_center;

/** @brief 以已枚举且连接代次稳定的 USB 替身运行正常单趟流程。 */
void UsbLink_Get_State(UsbLink_State_t *state) { *state = usb_state; }

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
        const uint8_t target[8] = {1U, 0x88U, 0x13U, 0xA0U, 0x0FU, (uint8_t)test_tick, (uint8_t)(test_tick >> 8U), (uint8_t)(test_tick >> 16U)}; /* (5000,4000) */
        Jetson_CAN_Receive(JETSON_CAN_TARGET_CENTER_ID, target, sizeof(target));
    }
    ++critical_depth;
}
/** @brief 验证临界区配对退出。 */
void test_exit_critical(void) { assert(critical_depth > 0U); --critical_depth; }

/** @brief 模拟软件受理使能，停车反馈由场景单独提供。 @return 1。 */
uint8_t Chassis_Enable(void)
{
    enabled = motion_ready = 1U;
    ++enables;
    return 1U;
}

/** @brief 模拟请求失能及撤销停车反馈。 */
void Chassis_Disable(void)
{
    enabled = stopped = motion_ready = 0U;
    ++disables;
}

/** @brief 记录停车请求；请求本身不生成停车反馈。 */
void Chassis_Stop(void) { ++stops; motion_ready = 1U; }
void Chassis_Hold(void) { ++stops; motion_ready = 1U; }
uint8_t Chassis_Attitude_Ready(void) { return imu_age_ms <= 20U; }
uint8_t Chassis_Can_Enable(void) { return !enabled; }
/** @brief 读取模拟底盘使能状态。 @return 软件已使能为 1。 */
uint8_t Chassis_Is_Enabled(void) { return enabled; }
/** @brief 读取场景独立给出的停车反馈。 @return 已使能且确认停车为 1。 */
uint8_t Chassis_Is_Stopped(void) { return enabled && stopped; }
uint8_t Chassis_Motion_Ready(void) { return enabled && motors_ready && motion_ready; }

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
uint8_t Chassis_Set_Navigation_Velocity(float vx, float vy, float wz, uint8_t world_frame)
{
    assert(!world_frame && enabled && isfinite(vx) && isfinite(vy) && isfinite(wz));
    ++velocity_requests;
    last_vx = vx;
    last_vy = vy;
    last_wz = wz;
    motion_ready = vx == 0.0f && vy == 0.0f && wz == 0.0f;
    if (!motion_ready)
        stopped = 0U;
    return 1U;
}

/* 单趟底盘联调结束于对准及定距，不进入限时盲推或退出。 */
uint8_t Chassis_Set_Velocity(float vx, float vy, float wz)
{
    (void)vx;
    (void)vy;
    (void)wz;
    assert(0 && "unexpected timed delivery movement");
    return 0U;
}

/** @brief 捕获定距请求，清除先前停车反馈。 @return 模拟受理为 1。
 * @param[in] direction 平移方向。 @param[in] distance 规划米数。 @param[in] speed 车体速度上限 m/s。
 */
uint8_t Chassis_Distance_Start(Chassis_Translation_Direction_t direction, float distance, float speed)
{
    last_distance_direction = direction;
    last_distance = distance;
    assert(Chassis_Motion_Ready() && distance > 0.0f && speed == JETSON_AUTO_DISTANCE_SPEED_MPS);
    ++distance_count;
    stopped = motion_ready = 0U;
    return 1U;
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
    stopped = motion_ready = 0U;
    return 1U;
}

/** @brief 捕获真实编码后的 USB 上行载荷。 @return 0 已提交，1 链路忙。
 * @param[in] id 运输协议上行类型；底盘联调仅发类型 21 启动查询。
 * @param[in] payload 编码载荷。 @param[in] len 载荷字节数。
 */
int Jetson_Link_Send(uint8_t id, const uint8_t *payload, uint8_t len)
{
    assert(critical_depth > 0U);
    assert(payload != NULL);
    assert(len == (id == JETSON_CAN_ARRIVAL_ID ? 1U : 5U));
    assert(payload[0] == (id == JETSON_CAN_POINT_REACHED_ID ? 0U : 1U));
    if (!tx_ready)
        return 1;
    assert(sent_count < sizeof(sent) / sizeof(sent[0]));
    const uint32_t encoded = len == 1U ? 0U : (uint32_t)payload[1] |
        (uint32_t)payload[2] << 8U | (uint32_t)payload[3] << 16U | (uint32_t)payload[4] << 24U;
    if (id == JETSON_CAN_POINT_REACHED_ID)
        startup_id = encoded;
    else
        sent[sent_count++] = (Sent_Frame){id, encoded};
    return 0;
}

/** @brief 初始化独立场景，启用替身外设能力，底盘保持失能。 */
static void Reset(void)
{
    test_tick = 100U;
    critical_depth = 0U;
    can_arrival_before_snapshot = 0U;
    enabled = stopped = motion_ready = 0U;
    enables = stops = disables = velocity_requests = 0U;
    distance_count = turn_count = sent_count = 0U;
    imu_age_ms = startup_id = 0U;
    tx_ready = motors_ready = 1U;
    last_turn_direction = 0;
    last_turn_angle = last_vx = last_vy = last_wz = last_distance = 0.0f;
    last_distance_direction = CHASSIS_TRANSLATION_NONE;
    usb_state = (UsbLink_State_t){.connected = 1U, .generation = 1U};
    background_center = JETSON_CAN_SAFETY_CENTER_ID;
    Jetson_CAN_Init();
    Jetson_Auto_Init();
}

static void Step(void)
{
    test_tick += 5U;
    /* 另一中心持续无目标，保持板间周期流；当前测试中心仍按真实接收年龄过期。 */
    const uint8_t no_center[8] = {0};
    Jetson_CAN_Receive(background_center, no_center, sizeof(no_center));
    Jetson_Auto_Loop();
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
static void Receive_Center(uint16_t id, int16_t x, int16_t y)
{
    background_center = id == JETSON_CAN_TARGET_CENTER_ID ?
        JETSON_CAN_SAFETY_CENTER_ID : JETSON_CAN_TARGET_CENTER_ID;
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
    Jetson_CAN_Receive(JETSON_CAN_ARRIVAL_REPLY_ID, &accepted, 1U);
}

/** @brief 注入 ROI 判定。 @param[in] passed 1 为通过。 @param[in] destination 左 1、右 2。 */
static void Receive_Roi(uint8_t passed, uint8_t destination)
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

/** @brief 核对从真实 USB 载荷解出的上行类型及身份。
 * @param[in] index 发送顺序索引。 @param[in] id 预期类型。 @param[in] token 预期本趟身份。
 */
static void Check_Sent(unsigned index, uint16_t id, uint32_t token)
{
    assert(index < sent_count && sent[index].id == id && sent[index].token == token);
}


/** @brief 校验安全区视觉核验之前未请求定距动作。 */
static void Flow_No_Actuator(void)
{
    assert(distance_count == 0U);
}

/** @brief 显式启动底盘联调，确认此模式无需执行夹爪和 ROI 流程。 */
static void Flow_Start(void)
{
    Reset();
    assert(JETSON_AUTO_ENABLED == 0 && JETSON_AUTO_CHASSIS_TEST == 1);
    Steps(5U);
    assert(!enabled && !enables && jetson_test_start == 0U);
    jetson_test_start = 1U;
    imu_age_ms = 21U;
    Steps(2U);
    assert(!enabled && startup_id != 0U);
    Receive_Command_Ack(1U, 5U, startup_id);
    uint8_t result[6] = {0U, 1U};
    Put_Token(result + 2, startup_id);
    Jetson_CAN_Receive(9U, result, sizeof(result));
    imu_age_ms = 0U;
    Step();
    assert(enabled && enables == 1U);
    Step();
    Receive_Center(JETSON_CAN_TARGET_CENTER_ID, 5000, 4000);
    Step();
    assert(last_vx > 0.0f && last_wz == 0.0f && !stopped);
    Flow_No_Actuator();
}

/** @brief 用新鲜窗内坐标在轮速非零时推进到上行握手。
 * @param[in] id 目标类型 3或安全区类型 4。 @param[in] uplink 预期上行ID。
 */
static void Flow_Arrive(uint16_t id, uint16_t uplink)
{
    const unsigned before = sent_count;
    Receive_Center(id, 5000, 7500);
    Step();
    Receive_Center(id, 5000, 7500);
    Step();
    for (unsigned i = 0U; i < 4U && sent_count == before; ++i)
    {
        Receive_Center(id, 5000, 7500);
        Step();
    }
    assert(sent_count == before + 1U && sent[before].id == uplink);
    Flow_No_Actuator();
}

/** @brief 窗内停车并双帧核验后只启动一次定距；视觉漂移或失效不能提前到达。 */
static void Flow_Safety_Straight(void)
{
    const unsigned before = sent_count;
    Receive_Center(JETSON_CAN_SAFETY_CENTER_ID, 5000, 7500);
    Step();
    Receive_Center(JETSON_CAN_SAFETY_CENTER_ID, 5000, 7500);
    Step();
    Steps(10U);
    assert(!stopped && distance_count == 0U && sent_count == before);
    for (unsigned i = 0U; i < 4U && !distance_count; ++i)
    {
        Receive_Center(JETSON_CAN_SAFETY_CENTER_ID, 5000, 7500);
        Step();
    }
    assert(distance_count == 1U && !stopped && sent_count == before);
    assert(last_distance_direction == CHASSIS_TRANSLATION_FORWARD);
    assert(last_distance == JETSON_AUTO_SAFETY_STRAIGHT_M);
    const unsigned velocities = velocity_requests, stop_requests = stops;
    for (unsigned i = 0U; i < 160U; ++i)
    {
        if (i < 20U)
            Receive_Center(JETSON_CAN_SAFETY_CENTER_ID, 1000, 1000);
        else if (i == 20U)
        {
            const uint8_t invalid[8] = {0};
            Jetson_CAN_Receive(JETSON_CAN_SAFETY_CENTER_ID, invalid, sizeof(invalid));
        }
        Step();
    }
    assert(enabled && distance_count == 1U && sent_count == before);
    assert(velocity_requests == velocities && stops == stop_requests);
    motion_ready = 1U;
    Steps(2U);
    assert(!stopped && sent_count == before + 1U && sent[before].id == JETSON_CAN_SAFETY_ARRIVED_ID);
}

/** @brief 核验左右路由、真实握手、定距后到位锁止及夹爪命令为零。
 * @param[in] destination 左区1或右区2。
 */
static void Flow_One_Shot(uint8_t destination)
{
    const uint32_t token = 0xA13579U;
    Flow_Start();
    Flow_Arrive(JETSON_CAN_TARGET_CENTER_ID, JETSON_CAN_ARRIVAL_ID);
    Receive_Arrival_Ack(1U);
    Receive_Roi(1U, destination);
    Receive_Turn(destination, token);
    Step();
    assert(turn_count == 1U && last_turn_direction == (destination == 1U ? 1 : -1));
    assert(last_turn_angle == 180.0f && !stopped);
    Steps(10U);
    assert(sent_count == 1U);
    motion_ready = 1U;
    Steps(2U);
    Check_Sent(1U, JETSON_CAN_TURN_COMPLETED_ID, token);
    Receive_Command_Ack(1U, 1U, token + 1U);
    Step();
    const unsigned count = velocity_requests;
    Receive_Center(JETSON_CAN_SAFETY_CENTER_ID, 5000, 4000);
    Step();
    assert(velocity_requests == count);
    Receive_Command_Ack(1U, 1U, token);
    Step();
    Receive_Center(JETSON_CAN_SAFETY_CENTER_ID, 5000, 4000);
    Step();
    assert(velocity_requests > count && last_vx > 0.0f && last_wz == 0.0f);
    Flow_Safety_Straight();
    Check_Sent(2U, JETSON_CAN_SAFETY_ARRIVED_ID, token);
    Receive_Command_Ack(1U, 2U, token);
    Step();
    assert(!enabled && disables == 1U);
    const unsigned after = velocity_requests;
    for (unsigned i = 0U; i < 100U; ++i)
    {
        jetson_test_start = (uint8_t)(i & 1U);
        Receive_Center(JETSON_CAN_TARGET_CENTER_ID, 5000, 4000);
        Step();
    }
    assert(enables == 1U && velocity_requests == after && sent_count == 3U);
    assert(distance_count == 1U);
}

/** @brief ROI拒绝只停车结束，不能执行原搬运恢复的松爪和后退。 */
static void Flow_Rejection(void)
{
    Flow_Start();
    Flow_Arrive(JETSON_CAN_TARGET_CENTER_ID, JETSON_CAN_ARRIVAL_ID);
    Receive_Arrival_Ack(1U);
    Receive_Roi(0U, 0U);
    stopped = 1U;
    Steps(2U);
    assert(!enabled && turn_count == 0U);
    jetson_test_start = 0U;
    Step();
    jetson_test_start = 1U;
    Steps(3U);
    assert(enables == 1U);
    Flow_No_Actuator();
}

/** @brief 丢目标停车等待新帧；中心长期丢失及启动撤销均结束，且不主动搜索。 */
static void Flow_Interruptions(void)
{
    Flow_Start();
    const unsigned before = stops;
    Steps(55U);
    assert(stops > before && last_wz == 0.0f && turn_count == 0U);
    stopped = 1U;
    Receive_Center(JETSON_CAN_TARGET_CENTER_ID, 5000, 4000);
    Step();
    assert(last_vx > 0.0f && !stopped);
    Steps(160U);
    stopped = 1U;
    Step();
    assert(!enabled);
    Flow_No_Actuator();

    Flow_Start();
    jetson_test_start = 0U;
    Step();
    stopped = 1U;
    Step();
    assert(!enabled);
    jetson_test_start = 1U;
    Steps(5U);
    assert(enables == 1U);
    Flow_No_Actuator();
}

/** @brief 新图漂出到位窗口时，在反馈仍在减速的情况下恢复跟随。 */
static void Flow_Visual_Recheck(void)
{
    Flow_Start();
    Receive_Center(JETSON_CAN_TARGET_CENTER_ID, 5000, 7500);
    Step();
    const unsigned moves = velocity_requests;
    Steps(4U);
    assert(enabled && !stopped && !sent_count && velocity_requests == moves);
    motion_ready = 0U;
    Receive_Center(JETSON_CAN_TARGET_CENTER_ID, 5101, 7500);
    Step();
    assert(!stopped && !sent_count && velocity_requests == moves);
    Receive_Center(JETSON_CAN_TARGET_CENTER_ID, 5121, 7500);
    Step();
    assert(!stopped && !sent_count && velocity_requests == moves + 1U);
    Flow_Arrive(JETSON_CAN_TARGET_CENTER_ID, JETSON_CAN_ARRIVAL_ID);
    Flow_No_Actuator();
}

/** @brief 90秒总期限覆盖导航和未等到启动动作结果；到期不可重启。 */
static void Flow_Deadline(void)
{
    Flow_Start();
    test_tick += JETSON_AUTO_CHASSIS_TEST_TIMEOUT_MS;
    Step();
    stopped = 1U;
    Step();
    assert(!enabled);
    Steps(3U);
    assert(enables == 1U);
    Flow_No_Actuator();

    Reset();
    jetson_test_start = 1U;
    Step();
    test_tick += JETSON_AUTO_CHASSIS_TEST_TIMEOUT_MS;
    Step();
    Steps(3U);
    assert(enables == 0U);
}

/** @brief 执行底盘单趟联调合同；缺少视觉窗口时必须始终失能。 @return 0通过。 */
int main(void)
{
#ifdef TEST_JETSON_CHASSIS_BAD_WINDOW
    (void)Flow_One_Shot;
    (void)Flow_Rejection;
    (void)Flow_Interruptions;
    (void)Flow_Visual_Recheck;
    (void)Flow_Deadline;
    Reset();
    jetson_test_start = 1U;
    Steps(50U);
    assert(!enabled && !enables && !velocity_requests && !sent_count);
    Flow_No_Actuator();
    puts("PASS: chassis-only invalid-window gate");
#else
    Flow_One_Shot(1U);
    Flow_One_Shot(2U);
    Flow_Rejection();
    Flow_Interruptions();
    Flow_Visual_Recheck();
    Flow_Deadline();
    puts("PASS: chassis-only gate, routing, straight distance, handshake, stopping, loss and deadline");
#endif
    return 0;
}
