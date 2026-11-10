#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "jetson_can.h"
#include "manual_can.h"
#include "manual_control.h"
#include "chassis.h"
#include "motor_chassis.h"
#include "dm_motor.h"
#include "INS_task.h"
#include "tim.h"
#include "task.h"

/* 编译真实协议、手动控制、底盘、几何与 PID；只模拟时钟、IMU、反馈和 CAN 入队。
 * 返回 HAL_OK 仅代表模拟入队，不代表实物执行，也不以理想反馈代替实车验收。
 */
CAN_HandleTypeDef hcan1, hcan2;
TIM_HandleTypeDef htim2;
volatile dm_motor_t dm_motor[3];
uint32_t test_tick;
static uint32_t counter_us, free_mailboxes, controls, enables, disables, aborts;
static uint32_t nonzero_controls, imu_age_ms;
static unsigned critical_depth;
static uint8_t imu_ready, abort_pending, last_sent_id;
static int imu_reads_before_failure;
static float motor_velocity[3];

typedef enum { NO_FEEDBACK, QUIET_FEEDBACK, MOVING_FEEDBACK } Feedback_Mode_t;

/** @brief 模拟任务屏蔽 CAN 接收中断。 @return 无。 */
void test_enter_critical(void) { ++critical_depth; }

/** @brief 核对临界区成对退出。 @return 无。 */
void test_exit_critical(void) { assert(critical_depth > 0U); --critical_depth; }

/** @brief 模拟三帧发送之间让出一拍；时钟只由测试场景推进。
 * @param[in] ticks 等待 tick，必须为 1。 @return 无。
 */
void vTaskDelay(TickType_t ticks)
{
    assert(ticks == 1U && last_sent_id >= 1U && last_sent_id <= 2U);
    last_sent_id = 0U;
}

/** @brief 读取模拟 TIM2 计数。 @param[in] timer TIM2。 @return 微秒低 32 位。 */
uint32_t test_timer_counter(TIM_HandleTypeDef *timer)
{ assert(timer == &htim2); return counter_us; }

/** @brief 提供可独立设置年龄和失效时刻的姿态快照。
 * @param[out] data 姿态输出。 @return 1 有效，0 未就绪。
 */
uint8_t INS_Get_Data(INS_control_data_t *data)
{
    assert(data != NULL);
    if (!imu_ready || imu_reads_before_failure == 0)
        return 0U;
    if (imu_reads_before_failure > 0)
        --imu_reads_before_failure;
    data->yaw_deg = 0.0f;
    data->world_z_rad_s = 0.0f;
    data->sample_time_ms = test_tick - imu_age_ms;
    return 1U;
}

/** @brief 返回模拟 CAN1 三邮箱余量。
 * @param[in] can CAN1。 @return 空闲邮箱数 0～3。
 */
uint32_t HAL_CAN_GetTxMailboxesFreeLevel(CAN_HandleTypeDef *can)
{ assert(can == &hcan1); return free_mailboxes; }

/** @brief 捕获实际底盘提交的 MIT 目标；每帧只占用一个模拟邮箱。
 * @param[in] can CAN1。 @param[in] id 电机 1～3。
 * @param[in] position 位置 rad。 @param[in] velocity 电机轮轴速度 rad/s。
 * @param[in] kp 位置增益。 @param[in] kd 原车速度阻尼。 @param[in] torque 扭矩 N·m。
 * @return HAL_OK，仅代表模拟入队。
 */
HAL_StatusTypeDef ctrl_motor(CAN_HandleTypeDef *can, uint16_t id, float position,
                            float velocity, float kp, float kd, float torque)
{
    const float expected_kd[3] = {0.2f, 0.25f, 0.2f};
    assert(can == &hcan1 && id >= 1U && id <= 3U && free_mailboxes > 0U && !abort_pending);
    assert(position == 0.0f && kp == 0.0f && torque == 0.0f && kd == expected_kd[id - 1U]);
    assert(isfinite(velocity) && fabsf(velocity) <= 40.0f);
    motor_velocity[id - 1U] = velocity;
    last_sent_id = (uint8_t)id;
    --free_mailboxes;
    ++controls;
    if (velocity != 0.0f)
        ++nonzero_controls;
    return HAL_OK;
}

/** @brief 模拟使能帧入队，不修改电机反馈状态。
 * @param[in] can CAN1。 @param[in] id 电机 1～3。 @return HAL_OK。
 */
HAL_StatusTypeDef enable_motor(CAN_HandleTypeDef *can, uint16_t id)
{
    assert(can == &hcan1 && id >= 1U && id <= 3U && free_mailboxes > 0U && !abort_pending);
    last_sent_id = (uint8_t)id;
    --free_mailboxes;
    ++enables;
    return HAL_OK;
}

/** @brief 模拟失能帧入队，不冒充真实停车确认。
 * @param[in] can CAN1。 @param[in] id 电机 1～3。 @return HAL_OK。
 */
HAL_StatusTypeDef disable_motor(CAN_HandleTypeDef *can, uint16_t id)
{
    assert(can == &hcan1 && id >= 1U && id <= 3U && free_mailboxes > 0U && !abort_pending);
    last_sent_id = (uint8_t)id;
    --free_mailboxes;
    ++disables;
    return HAL_OK;
}

/** @brief 模拟异步撤销，邮箱须由后续显式完成事件释放。
 * @param[in] can CAN1。 @param[in] mailboxes 三邮箱掩码。 @return HAL_OK。
 */
HAL_StatusTypeDef HAL_CAN_AbortTxRequest(CAN_HandleTypeDef *can, uint32_t mailboxes)
{
    assert(can == &hcan1 && mailboxes == (CAN_TX_MAILBOX0 | CAN_TX_MAILBOX1 | CAN_TX_MAILBOX2));
    abort_pending = free_mailboxes != 3U;
    ++aborts;
    return HAL_OK;
}

/** @brief 独立推进毫秒与微秒时钟，保留自然回绕。
 * @param[in] milliseconds 推进时间 ms。 @return 无。
 */
static void Advance(uint32_t milliseconds)
{ test_tick += milliseconds; counter_us += milliseconds * 1000U; }

/** @brief 为三轮分别提供一条新反馈，不由发送函数自动生成。
 * @param[in] speed 各轮反馈 rad/s。 @param[in] state 达妙原始状态。 @return 无。
 */
static void Feedback(float speed, uint8_t state)
{
    for (unsigned i = 0U; i < 3U; ++i)
    {
        dm_motor[i].id = (uint8_t)(i + 1U);
        dm_motor[i].state = state;
        dm_motor[i].velocity = speed;
        dm_motor[i].feedback_time_ms = test_tick;
        ++dm_motor[i].feedback_seq;
    }
}

/** @brief 按实际任务顺序执行控制，再服务底盘，不添加周期后同步。
 * @param[in] feedback 是否提供静止/运动反馈；运动反馈固定 1 rad/s。
 * @param[in] complete_can 1 表示上一组发送/撤销已完成。 @return 无。
 */
static void Service(Feedback_Mode_t feedback, uint8_t complete_can)
{
    if (complete_can)
    {
        free_mailboxes = 3U;
        abort_pending = 0U;
    }
    if (feedback != NO_FEEDBACK)
        Feedback(feedback == QUIET_FEEDBACK ? 0.0f : 1.0f, 1U);
    Manual_Control_Loop();
    Chassis_Loop();
    assert(critical_depth == 0U);
}

/** @brief 推进一个普通 10 ms 控制周期并模拟前组 CAN 完成。
 * @param[in] feedback 反馈方式。 @return 无。
 */
static void Step(Feedback_Mode_t feedback)
{ Advance(10U); Service(feedback, 1U); }

/** @brief 编码真实协议的有符号小端速度字段，再调用真实解码器。
 * @param[in] vx 前向定点值。 @param[in] vy 左向定点值。
 * @param[in] wz 车体角速度定点值。 @param[in] valid 有效字节。 @param[in] len DLC。
 * @return 无；线速度字段单位 0.001 m/s，角速度字段单位 0.001 rad/s。
 */
static void Receive(int16_t vx, int16_t vy, int16_t wz, uint8_t valid, uint8_t len)
{
    uint16_t bits[3] = {(uint16_t)vx, (uint16_t)vy, (uint16_t)wz};
    uint8_t data[8] = {valid, 0U, 0U, 0U, 0U, 0U, 0U, 0U};
    for (unsigned i = 0U; i < 3U; ++i)
    {
        data[1U + 2U * i] = (uint8_t)bits[i];
        data[2U + 2U * i] = (uint8_t)(bits[i] >> 8U);
    }
    Manual_CAN_Receive(data, len);
}

/** @brief 提交一条有效全零速度。 @return 无。 */
static void Receive_Zero(void) { Receive(0, 0, 0, 1U, 7U); }

/** @brief 初始化独立场景，不向私有控制状态注入值。
 * @param[in] milliseconds 初始 RTOS 时刻。 @param[in] microseconds 初始 TIM2 计数。
 * @return 无。
 */
static void Reset(uint32_t milliseconds, uint32_t microseconds)
{
    test_tick = milliseconds;
    counter_us = microseconds;
    free_mailboxes = 3U;
    controls = enables = disables = aborts = nonzero_controls = imu_age_ms = 0U;
    critical_depth = 0U;
    imu_ready = 1U;
    imu_reads_before_failure = -1;
    abort_pending = last_sent_id = 0U;
    memset(motor_velocity, 0, sizeof(motor_velocity));
    memset((void *)dm_motor, 0, sizeof(dm_motor));
    Chassis_Init();
    Jetson_CAN_Init();
    Manual_CAN_Init();
    Manual_Control_Init();
    assert(!Chassis_Is_Enabled() && manual_control_state == MANUAL_DISARMED);
    assert(controls == 0U && enables == 0U && disables == 0U);
}

/** @brief 核对最近三轮 MIT 目标已经全零。 @return 无。 */
static void Assert_Zero_Target(void)
{
    for (unsigned i = 0U; i < 3U; ++i)
        assert(motor_velocity[i] == 0.0f);
}

/** @brief 新零帧启动真实使能包序；只到首组已使能零 MIT，不提供停车结论。
 * @return 无。
 */
static void Prepare_Zero(void)
{
    uint32_t before_controls = controls, before_enables = enables;
    Receive_Zero();
    Service(NO_FEEDBACK, 1U);
    assert(manual_control_state == MANUAL_PREPARING && Chassis_Is_Enabled());
    assert(controls == before_controls + 3U && enables == before_enables);
    Assert_Zero_Target();
    Step(QUIET_FEEDBACK);
    assert(enables == before_enables + 3U && !Chassis_Is_Stopped());
    Step(QUIET_FEEDBACK);
    assert(!Chassis_Is_Stopped() && manual_control_state == MANUAL_PREPARING);
}

/** @brief 零速基线之后提供恰好三组新停车反馈，验证前两组不足以起步。
 * @return 无。
 */
static void Confirm_Ready(void)
{
    for (unsigned group = 0U; group < 3U; ++group)
    {
        Step(QUIET_FEEDBACK);
        assert(manual_control_state == (group == 2U ? MANUAL_READY : MANUAL_PREPARING));
        assert(Chassis_Is_Stopped() == (group == 2U));
        Assert_Zero_Target();
    }
}

/** @brief 准备正常起步；仍通过真实使能与三组停车确认。 @return 无。 */
static void Prepare_Ready(void) { Prepare_Zero(); Confirm_Ready(); }

/** @brief 从就绪状态提交新非零速度，并等待第一组非零 MIT。
 * @param[in] vx 纵向定点值。 @param[in] vy 横向定点值。 @param[in] wz 角速度定点值。
 * @return 无。
 */
static void Start_Moving(int16_t vx, int16_t vy, int16_t wz)
{
    Receive(vx, vy, wz, 1U, 7U);
    Service(QUIET_FEEDBACK, 1U);
    assert(manual_control_state == MANUAL_RUNNING && Chassis_Is_Enabled());
    Step(MOVING_FEEDBACK);
    assert(manual_control_state == MANUAL_RUNNING && nonzero_controls > 0U);
}

/** @brief 持续刷新同一速度，保留真实斜坡至给定周期数。
 * @param[in] vx 纵向原始定点值。 @param[in] vy 横向原始定点值。
 * @param[in] wz 角速度原始定点值。 @param[in] periods 10 ms 周期数。 @return 无。
 */
static void Run_Target(int16_t vx, int16_t vy, int16_t wz, unsigned periods)
{
    for (unsigned i = 0U; i < periods; ++i)
    {
        Advance(10U);
        Receive(vx, vy, wz, 1U, 7U);
        Service(MOVING_FEEDBACK, 1U);
        assert(manual_control_state == MANUAL_RUNNING && Chassis_Is_Enabled());
    }
}

/** @brief 核对上电与旧非零帧不能使能，零帧时刻 0 合法，三组确认不可省略。
 * @return 无。
 */
static void Test_Startup(void)
{
    Reset(0U, 0U);
    Service(NO_FEEDBACK, 1U);
    Receive(100, 0, 0, 1U, 7U);
    Service(QUIET_FEEDBACK, 1U);
    for (unsigned i = 0U; i < 4U; ++i)
        Step(QUIET_FEEDBACK);
    assert(manual_control_state == MANUAL_DISARMED && !Chassis_Is_Enabled());
    assert(controls == 0U && enables == 0U);
    Receive_Zero();
    Receive(100, 0, 0, 1U, 7U);
    Step(QUIET_FEEDBACK);
    assert(manual_control_state == MANUAL_DISARMED && !Chassis_Is_Enabled());

    Reset(0U, 0U);
    Prepare_Zero();
    Manual_CAN_Velocity_t data;
    Manual_CAN_Get_Velocity(&data);
    assert(data.valid && data.sequence == 1U && data.received_ms == 0U);
    for (unsigned i = 0U; i < 3U; ++i)
    {
        Step(NO_FEEDBACK);
        assert(manual_control_state == MANUAL_PREPARING && !Chassis_Is_Stopped());
    }
    Confirm_Ready();
    assert(nonzero_controls == 0U);
    Start_Moving(100, 0, 0);
    assert(motor_velocity[1] < 0.0f && motor_velocity[2] > 0.0f);

    Reset(0U, 0U);
    Prepare_Zero();
    Receive(100, 0, 0, 1U, 7U);
    Step(QUIET_FEEDBACK);
    assert(manual_control_state == MANUAL_BRAKING && nonzero_controls == 0U);
}

/** @brief 验证运行后收到单帧全零仍立即清目标，停稳前非零不得执行或触发失能。
 * @param[in] repeat 1 持续发送换向请求，0 不在确认期间持续重发。
 * @param[in] batch 1 模拟同一任务周期内全零后又收到非零。
 * @return 无。
 */
static void Test_Stop_Before_Reverse(uint8_t repeat, uint8_t batch)
{
    Reset(0U, 0U);
    Prepare_Ready();
    Start_Moving(0, 500, 0);
    uint32_t before_nonzero = nonzero_controls;

    Advance(5U);
    Receive_Zero();
    if (batch)
        Receive(0, -500, 0, 1U, 7U);
    const uint32_t stop_ms = test_tick;
    Service(MOVING_FEEDBACK, 1U);
    assert(manual_control_state == MANUAL_PREPARING);
    Step(MOVING_FEEDBACK);
    Assert_Zero_Target();

    Advance(10U);
    assert(test_tick - stop_ms == 20U);
    Receive(0, -500, 0, 1U, 7U);
    Service(MOVING_FEEDBACK, 1U);
    assert(manual_control_state == MANUAL_PREPARING && Chassis_Is_Enabled());
    Assert_Zero_Target();

    for (unsigned group = 0U; group < 3U; ++group)
    {
        Advance(10U);
        if (repeat)
            Receive(0, -500, 0, 1U, 7U);
        Service(QUIET_FEEDBACK, 1U);
        assert(manual_control_state == (group == 2U ? MANUAL_READY : MANUAL_PREPARING));
        assert(Chassis_Is_Enabled());
        Assert_Zero_Target();
    }
    Step(QUIET_FEEDBACK);
    assert(manual_control_state == MANUAL_READY && nonzero_controls == before_nonzero);
    assert(disables == 0U && aborts == 0U);
    Assert_Zero_Target();
    if (repeat)
    {
        Start_Moving(0, -500, 0);
        assert(nonzero_controls > before_nonzero && motor_velocity[0] > 0.0f);
    }
}

/** @brief 运行后等待停稳仍检查轮反馈超时，并保留原停稳等待期限。
 * @return 无。
 */
static void Test_Stop_Preparing_Protection(void)
{
    for (unsigned stale_feedback = 0U; stale_feedback < 2U; ++stale_feedback)
    {
        Reset(0U, 0U);
        Prepare_Ready();
        Start_Moving(0, 500, 0);
        Receive_Zero();
        Service(MOVING_FEEDBACK, 1U);
        assert(manual_control_state == MANUAL_PREPARING);

        const uint32_t deadline = stale_feedback ? CHASSIS_MOTOR_FEEDBACK_TIMEOUT_MS + 1U : MANUAL_PREPARE_TIMEOUT_MS;
        const Feedback_Mode_t feedback = stale_feedback ? NO_FEEDBACK : MOVING_FEEDBACK;
        for (uint32_t elapsed = 10U; elapsed < deadline; elapsed += 10U)
        {
            Advance(10U);
            Receive(0, -500, 0, 1U, 7U);
            Service(feedback, 1U);
            assert(manual_control_state == MANUAL_PREPARING);
            Assert_Zero_Target();
        }
        Advance(stale_feedback ? 1U : 10U);
        Receive(0, -500, 0, 1U, 7U);
        Service(feedback, 1U);
        assert(manual_control_state == MANUAL_BRAKING && Chassis_Is_Enabled());
        Assert_Zero_Target();
    }
}

/** @brief 核对正负车体角速度经真实三轮逆解与方向后，各轮目标符号和幅值。
 * @return 无。
 */
static void Test_Rotation(void)
{
    for (int sign = -1; sign <= 1; sign += 2)
    {
        Reset(0U, 0U);
        Prepare_Ready();
        Start_Moving(0, 0, (int16_t)(sign * 1000));
        Run_Target(0, 0, (int16_t)(sign * 1000), 70U);
        float expected = -(float)sign * 0.108f / 0.035f;
        for (unsigned wheel = 0U; wheel < 3U; ++wheel)
            assert(fabsf(motor_velocity[wheel] - expected) < 0.00001f);
    }
}

/** @brief 制动期收到新零/非零都不得复位 500 ms 截止；结束后缓存帧不能再使能。
 * @param[in] braking_start 制动起点 RTOS 毫秒。 @return 无。
 */
static void Assert_Braking_Locked(uint32_t braking_start)
{
    while ((uint32_t)(test_tick - braking_start) < 490U)
    {
        Advance(10U);
        if (((test_tick - braking_start) / 10U) % 2U)
            Receive_Zero();
        else
            Receive(100, 0, 0, 1U, 7U);
        Service(MOVING_FEEDBACK, 1U);
        assert(manual_control_state == MANUAL_BRAKING && Chassis_Is_Enabled());
        Assert_Zero_Target();
    }
    uint32_t elapsed = test_tick - braking_start;
    assert(elapsed <= 499U);
    Advance(499U - elapsed);
    Receive_Zero();
    Service(MOVING_FEEDBACK, 1U);
    assert(manual_control_state == MANUAL_BRAKING && Chassis_Is_Enabled());
    Advance(1U);
    Service(MOVING_FEEDBACK, 1U);
    assert(manual_control_state == MANUAL_DISARMED && !Chassis_Is_Enabled());
    /* 即使当前缓存是有效新零，也必须在失能完成后再来一帧零。 */
    for (unsigned i = 0U; i < 3U; ++i)
    {
        Step(QUIET_FEEDBACK);
        assert(manual_control_state == MANUAL_DISARMED && !Chassis_Is_Enabled());
    }
    assert(disables == 3U);
    Receive(100, 0, 0, 1U, 7U);
    Step(QUIET_FEEDBACK);
    assert(manual_control_state == MANUAL_DISARMED && !Chassis_Is_Enabled());
    Receive_Zero();
    Service(QUIET_FEEDBACK, 1U);
    assert(manual_control_state == MANUAL_PREPARING && Chassis_Is_Enabled());
}

/** @brief 无效或畸形帧与有效零、非零同批到达时，仍执行撤权制动。
 * @return 无。
 */
static void Test_Invalid_Batch(void)
{
    for (unsigned malformed = 0U; malformed < 2U; ++malformed)
    {
        Reset(0U, 0U);
        Prepare_Ready();
        Start_Moving(100, 0, 0);
        uint32_t before_nonzero = nonzero_controls;
        Advance(10U);
        Receive_Zero();
        Receive(100, 0, 0, malformed ? 1U : 0U, malformed ? 6U : 7U);
        Receive(200, 0, 0, 1U, 7U);
        Manual_CAN_Velocity_t data;
        Manual_CAN_Get_Velocity(&data);
        assert(data.valid && data.vx == 200 && data.invalid_sequence == 1U);
        Service(MOVING_FEEDBACK, 1U);
        assert(manual_control_state == MANUAL_BRAKING && nonzero_controls == before_nonzero);
        Assert_Zero_Target();
        Assert_Braking_Locked(test_tick);
    }
}

/** @brief 命令年龄 249 ms 仍可用，250 ms 开始制动；跨 tick 回绕保持相同结果。
 * @param[in] wrap 1 使用临近 RTOS/TIM2 回绕的起点。 @return 无。
 */
static void Test_Timeout(uint8_t wrap)
{
    Reset(wrap ? UINT32_MAX - 100U : 0U, wrap ? UINT32_MAX - 40000U : 0U);
    if (wrap)
        for (unsigned i = 0U; i < 3U; ++i)
            dm_motor[i].feedback_seq = UINT32_MAX - 2U;
    Prepare_Ready();
    if (wrap)
        assert(dm_motor[0].feedback_seq < 10U);
    Start_Moving(100, 0, 0);
    Receive(100, 0, 0, 1U, 7U);
    uint32_t received_ms = test_tick;
    Advance(249U);
    Service(MOVING_FEEDBACK, 1U);
    assert((uint32_t)(test_tick - received_ms) == 249U);
    assert(manual_control_state == MANUAL_RUNNING && Chassis_Is_Enabled());
    uint32_t before_nonzero = nonzero_controls;
    Advance(1U);
    Service(MOVING_FEEDBACK, 1U);
    assert(manual_control_state == MANUAL_BRAKING && nonzero_controls == before_nonzero);
    uint32_t braking_start = test_tick;
    Step(MOVING_FEEDBACK);
    Assert_Zero_Target();
    Assert_Braking_Locked(braking_start);
}

/** @brief 核对电机反馈 30 ms 边界、异常状态、非有限值和 IMU 有效期。
 * @return 无。
 */
static void Test_Feedback_And_IMU(void)
{
    for (unsigned fault = 0U; fault < 5U; ++fault)
    {
        Reset(0U, 0U);
        Prepare_Ready();
        Start_Moving(100, 0, 0);
        Receive(100, 0, 0, 1U, 7U);
        Feedback(1.0f, 1U);
        if (fault == 0U)
        {
            for (unsigned i = 0U; i < 3U; ++i)
                dm_motor[i].feedback_time_ms = test_tick - 30U;
            Service(NO_FEEDBACK, 1U);
            assert(manual_control_state == MANUAL_RUNNING);
            Advance(1U);
        }
        else if (fault == 1U)
            dm_motor[1].state = 8U;
        else if (fault == 2U)
            dm_motor[1].velocity = NAN;
        else if (fault == 3U)
        {
            imu_age_ms = 20U;
            Service(NO_FEEDBACK, 1U);
            assert(manual_control_state == MANUAL_RUNNING);
            imu_age_ms = 21U;
        }
        else
            imu_ready = 0U;
        uint32_t before_nonzero = nonzero_controls;
        Service(NO_FEEDBACK, 1U);
        assert(manual_control_state == MANUAL_BRAKING && nonzero_controls == before_nonzero);
        Step(MOVING_FEEDBACK);
        Assert_Zero_Target();
    }
}

/** @brief 底盘自身在本轮失能后，最多下一任务周期撤销运行显示，缓存命令不能恢复。
 * @return 无。
 */
static void Test_Chassis_Protection(void)
{
    for (unsigned fault = 0U; fault < 3U; ++fault)
    {
        Reset(0U, 0U);
        Prepare_Ready();
        Start_Moving(100, 0, 0);
        Advance(10U);
        if (fault == 2U)
            Receive_Zero();
        else
            Receive(100, 0, 0, 1U, 7U);
        uint32_t before_nonzero = nonzero_controls;
        if (fault == 0U)
        {
            /* 第一次读取供 Control 接受目标，第二次在真实 Chassis_Loop 中失效。 */
            imu_reads_before_failure = 1;
            Service(MOVING_FEEDBACK, 1U);
        }
        else
        {
            free_mailboxes = 2U;
            Service(MOVING_FEEDBACK, 0U);
        }
        assert(!Chassis_Is_Enabled() && aborts == 1U && nonzero_controls == before_nonzero);
        Advance(5U);
        Service(QUIET_FEEDBACK, 1U);
        assert(manual_control_state == MANUAL_DISARMED && !Chassis_Is_Enabled());
        for (unsigned i = 0U; i < 4U; ++i)
        {
            Step(QUIET_FEEDBACK);
            assert(manual_control_state == MANUAL_DISARMED && !Chassis_Is_Enabled());
        }
        assert(disables == 3U && nonzero_controls == before_nonzero);
    }
}

/** @brief 核对原始定点边界与合速度限制，不允许对角超限被单轴检查遗漏。
 * @return 无。
 */
static void Test_Ranges(void)
{
    const int16_t accepted[][3] = {
        {500, 0, 0}, {-500, 0, 0}, {0, 500, 0}, {0, -500, 0},
        {0, 0, 2000}, {0, 0, -2000}, {300, 400, 2000}, {-300, -400, -2000}
    };
    const int16_t rejected[][3] = {
        {501, 0, 0}, {0, -501, 0}, {0, 0, 2001}, {0, 0, -2001},
        {400, 400, 0}, {-400, 400, 0}, {32767, 0, 0}
    };
    for (unsigned i = 0U; i < sizeof(accepted) / sizeof(accepted[0]); ++i)
    {
        Reset(0U, 0U);
        Prepare_Ready();
        Start_Moving(accepted[i][0], accepted[i][1], accepted[i][2]);
    }
    for (unsigned i = 0U; i < sizeof(rejected) / sizeof(rejected[0]); ++i)
    {
        Reset(0U, 0U);
        Prepare_Ready();
        Receive(rejected[i][0], rejected[i][1], rejected[i][2], 1U, 7U);
        Step(QUIET_FEEDBACK);
        assert(manual_control_state == MANUAL_BRAKING && nonzero_controls == 0U);
    }
    Reset(0U, 0U);
    Prepare_Ready();
    Start_Moving(500, 0, 1000);
    Run_Target(500, 0, 1000, 120U);
    float wheels[3] = {-motor_velocity[0], -motor_velocity[1], -motor_velocity[2]};
    float body[3];
    Motor_Chassis_Forward(wheels, body);
    assert(fabsf(body[0] - 0.5f) < 0.00001f && fabsf(body[1]) < 0.00001f);
    assert(fabsf(body[2] - 1.0f) < 0.00001f);
}

/** @brief 发送一组 0x180 心跳流量：有效全零、有效非零、valid=0 全零、畸形和空帧。 @return 无。 */
static void Receive_Heartbeat_Traffic(void)
{
    const uint8_t zero[7] = {1U};
    const uint8_t moving[7] = {1U, 0xF4U, 0x01U};
    const uint8_t empty[7] = {0U};
    const uint8_t malformed[7] = {2U};
    Jetson_CAN_Receive(0x180U, zero, sizeof(zero));
    Jetson_CAN_Receive(0x180U, moving, sizeof(moving));
    Jetson_CAN_Receive(0x180U, empty, sizeof(empty));
    Jetson_CAN_Receive(0x180U, malformed, sizeof(malformed));
    Jetson_CAN_Receive(0x180U, NULL, 0U);
}

/** @brief 手动固件中 0x180 不能使能、撤权、停车或续期 0x17F：只看手动快照。 @return 无。 */
static void Test_Heartbeat_Isolated(void)
{
    Reset(0U, 0U);
    for (unsigned i = 0U; i < 10U; ++i)
    {
        Receive_Heartbeat_Traffic();
        Step(QUIET_FEEDBACK);
        assert(manual_control_state == MANUAL_DISARMED && !Chassis_Is_Enabled() && enables == 0U);
    }
    Manual_CAN_Velocity_t velocity;
    Manual_CAN_Get_Velocity(&velocity);
    assert(velocity.sequence == 0U && !velocity.invalid_sequence && !velocity.zero_sequence);

    Prepare_Ready();
    Start_Moving(100, 0, 0);
    Receive(100, 0, 0, 1U, 7U);
    const uint32_t received_ms = test_tick;
    /* 0x180 持续到达期间，手动运行既不被撤权也不被延长：最后一条 0x17F 起 250 ms 制动。 */
    while ((uint32_t)(test_tick - received_ms) < MANUAL_VELOCITY_TIMEOUT_MS - 10U)
    {
        Advance(10U);
        Receive_Heartbeat_Traffic();
        Service(MOVING_FEEDBACK, 1U);
        assert(manual_control_state == MANUAL_RUNNING && Chassis_Is_Enabled());
    }
    const uint32_t before_nonzero = nonzero_controls;
    Advance(MANUAL_VELOCITY_TIMEOUT_MS - (test_tick - received_ms));
    Receive_Heartbeat_Traffic();
    Service(MOVING_FEEDBACK, 1U);
    assert((uint32_t)(test_tick - received_ms) == MANUAL_VELOCITY_TIMEOUT_MS);
    assert(manual_control_state == MANUAL_BRAKING && nonzero_controls == before_nonzero);
}

/** @brief 验证真实协议到 CAN 目标提交的通信控制组合。 @return 成功返回 0。 */
int main(void)
{
    Test_Startup();
    Test_Stop_Before_Reverse(0U, 0U);
    Test_Stop_Before_Reverse(1U, 0U);
    Test_Stop_Before_Reverse(1U, 1U);
    Test_Stop_Preparing_Protection();
    Test_Rotation();
    Test_Invalid_Batch();
    Test_Timeout(0U);
    Test_Timeout(1U);
    Test_Feedback_And_IMU();
    Test_Chassis_Protection();
    Test_Ranges();
    Test_Heartbeat_Isolated();
    puts("PASS Manual control: 0x17F codec/chassis startup, invalid batches, deadlines, guards, wraparound, 0x180 isolation");
    return 0;
}
