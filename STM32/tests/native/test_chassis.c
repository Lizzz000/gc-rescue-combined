#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "motor_chassis.h"
#include "chassis.h"
#include "dm_motor.h"
#include "INS_task.h"
#include "tim.h"
#include "task.h"

CAN_HandleTypeDef hcan1, hcan2;
TIM_HandleTypeDef htim2;
volatile dm_motor_t dm_motor[3];
uint32_t test_tick;
static uint32_t counter_us, free_mailboxes, controls, enables, disables, aborts, delays;
static float motor_velocity[3], yaw_deg, yaw_rate;
static uint32_t sample_age;
static uint8_t imu_ready, read_advances_tick, fail_control_id, abort_pending;
static uint8_t fail_enable_id, fail_disable_id, last_sent_id;
static unsigned critical_depth;

/** @brief 模拟任务屏蔽 CAN RX 中断。 @return 无。 */
void test_enter_critical(void) { ++critical_depth; }

/** @brief 检查每次反馈快照均成对退出临界区。 @return 无。 */
void test_exit_critical(void) { assert(critical_depth > 0U); --critical_depth; }

/** @brief 检查成功帧后的1ms让出；模拟时钟仍由step统一推进。
 * @param[in] ticks 等待tick，必须为1。
 * @return 无；同一帧不得重复等待，最后一帧不得等待。
 */
void vTaskDelay(TickType_t ticks)
{
    assert(ticks == 1U && last_sent_id >= 1U && last_sent_id <= 2U);
    ++delays;
    last_sent_id = 0U;
}

/** @brief 模拟已启动的1MHz TIM2，可独立于RTOS毫秒自然回绕。 */
uint32_t test_timer_counter(TIM_HandleTypeDef *timer)
{ assert(timer == &htim2); return counter_us; }

/** @brief 模拟同一快照；可在复制期间切换RTOS tick，核对读取后判断年龄。 */
uint8_t INS_Get_Data(INS_control_data_t *data)
{
    assert(data != NULL);
    if (!imu_ready) return 0U;
    if (read_advances_tick) { ++test_tick; read_advances_tick = 0U; }
    data->yaw_deg = yaw_deg;
    data->world_z_rad_s = yaw_rate;
    data->sample_time_ms = test_tick - sample_age;
    return 1U;
}

/** @brief bxCAN三个邮箱，每成功提交一帧占用一个，完成由测试显式推进。 */
uint32_t HAL_CAN_GetTxMailboxesFreeLevel(CAN_HandleTypeDef *can)
{ assert(can == &hcan1); return free_mailboxes; }

/** @brief 检查沿用旧车方向和各轮MIT阻尼；捕获实际业务调用的速度目标。 */
HAL_StatusTypeDef ctrl_motor(CAN_HandleTypeDef *can, uint16_t id, float position,
                            float velocity, float kp, float kd, float torque)
{
    const float expected_kd[3] = {0.2f, 0.25f, 0.2f};
    assert(can == &hcan1 && id >= 1U && id <= 3U && free_mailboxes > 0U);
    assert(position == 0.0f && kp == 0.0f && torque == 0.0f && kd == expected_kd[id-1U]);
    assert(isfinite(velocity) && fabsf(velocity) <= 40.0f && !abort_pending);
    last_sent_id = 0U;
    if (id == fail_control_id) { fail_control_id = 0U; return HAL_ERROR; }
    last_sent_id = (uint8_t)id;
    motor_velocity[id-1U] = velocity;
    --free_mailboxes; ++controls;
    return HAL_OK;
}

/** @brief 使能只占用邮箱，不假装电机已执行。 */
HAL_StatusTypeDef enable_motor(CAN_HandleTypeDef *can, uint16_t id)
{
    assert(can == &hcan1 && id >= 1U && id <= 3U && free_mailboxes > 0U && !abort_pending);
    last_sent_id = 0U;
    if (id == fail_enable_id) { fail_enable_id = 0U; return HAL_ERROR; }
    last_sent_id = (uint8_t)id;
    --free_mailboxes; ++enables;
    return HAL_OK;
}

/** @brief 取消尚未完成时禁止提交FD，确保不存在新旧控制帧交错。 */
HAL_StatusTypeDef disable_motor(CAN_HandleTypeDef *can, uint16_t id)
{
    assert(can == &hcan1 && id >= 1U && id <= 3U && free_mailboxes > 0U && !abort_pending);
    last_sent_id = 0U;
    if (id == fail_disable_id) { fail_disable_id = 0U; return HAL_ERROR; }
    last_sent_id = (uint8_t)id;
    --free_mailboxes; ++disables;
    return HAL_OK;
}

/** @brief 异步取消不释放邮箱，须由下一次总线完成事件释放。 */
HAL_StatusTypeDef HAL_CAN_AbortTxRequest(CAN_HandleTypeDef *can, uint32_t mailboxes)
{
    assert(can == &hcan1 && mailboxes == (CAN_TX_MAILBOX0 | CAN_TX_MAILBOX1 | CAN_TX_MAILBOX2));
    ++aborts; abort_pending = free_mailboxes != 3U;
    return HAL_OK;
}

/** @brief 推进真实采样间隔，选择是否完成前一轮CAN发送/取消。 */
static void step(uint32_t elapsed_us, uint8_t complete_can)
{
    counter_us += elapsed_us;
    test_tick += elapsed_us / 1000U;
    if (complete_can) { free_mailboxes = 3U; abort_pending = 0U; }
    Chassis_Loop();
    assert(critical_depth == 0U);
}

/** @brief 为指定车轮产生一帧反馈。 @param[in] wheel 数组下标 0～2。
 * @param[in] speed 实测轮速 rad/s。 @param[in] state 达妙反馈状态。 @return 无。
 */
static void wheel_feedback(unsigned wheel, float speed, uint8_t state)
{
    dm_motor[wheel].id = (uint8_t)(wheel + 1U);
    dm_motor[wheel].velocity = speed;
    dm_motor[wheel].state = state;
    dm_motor[wheel].feedback_time_ms = test_tick;
    ++dm_motor[wheel].feedback_seq;
}

/** @brief 返回一组已使能的静止反馈并推进一个控制周期。 @return 无。 */
static void stopped_group(void)
{
    counter_us += 10000U;
    test_tick += 10U;
    for (unsigned i = 0U; i < 3U; ++i)
        wheel_feedback(i, 0.0f, 1U);
    step(0U, 1U);
}

/** @brief 在已发送零速指令后，连续模拟三组静止反馈。 @return 无。 */
static void confirm_stopped(void)
{
    for (unsigned i = 0U; i < 3U; ++i)
    {
        stopped_group();
        assert(Chassis_Is_Stopped() == (i == 2U));
    }
}

/** @brief 每场景重建独立初始状态，不通过私有getter观测业务。 */
static void reset_case(uint32_t us, uint32_t ms)
{
    counter_us = us; test_tick = ms; free_mailboxes = 3U;
    controls = enables = disables = aborts = sample_age = delays = 0U;
    imu_ready = 1U; read_advances_tick = fail_control_id = abort_pending = 0U;
    fail_enable_id = fail_disable_id = last_sent_id = 0U;
    yaw_deg = yaw_rate = 0.0f;
    memset(motor_velocity, 0, sizeof(motor_velocity));
    memset((void *)dm_motor, 0, sizeof(dm_motor));
    Chassis_Init();
    assert(enables == 0U && controls == 0U && disables == 0U);
}

/** @brief 三轮方向固定-1，反解捕获的真实指令为车体速度供动作断言。 */
static void body_target(float body[3])
{
    float wheels[3] = {-motor_velocity[0], -motor_velocity[1], -motor_velocity[2]};
    Motor_Chassis_Forward(wheels, body);
}

/** @brief 比较物理量，浮点误差小于协议量化前的1e-5。 */
static void near(float actual, float expected)
{ assert(fabsf(actual - expected) < 0.00001f); }

/** @brief 三组初始化严格分时：先零MIT、再FC、再零MIT，每组恰占3邮箱。 */
static uint8_t enable_ready(void)
{
    uint32_t delay_before = delays;
    assert(Chassis_Enable());
    assert(controls == 3U && enables == 0U && free_mailboxes == 0U);
    assert(delays == delay_before + 2U);
    for (unsigned i = 0; i < 3U; ++i) near(motor_velocity[i], 0.0f);
    assert(!Chassis_Turn_Start(1,CHASSIS_TURN_LEFT_ANGLE_DEG) && !Chassis_Set_Velocity(0.1f,0,0));
    imu_ready = 0U;
    assert(Chassis_Set_Velocity(0,0,0)); /* 尚未使能完成、姿态未ready仍能清除动作。 */
    imu_ready = 1U;
    assert(Chassis_Enable()); /* 重复请求不能重新排队或重复第一组。 */
    assert(controls == 3U && enables == 0U && delays == delay_before + 2U);
    step(10000U,1U);
    assert(controls == 3U && enables == 3U && free_mailboxes == 0U);
    assert(delays == delay_before + 4U);
    assert(!Chassis_Turn_Start(1,CHASSIS_TURN_LEFT_ANGLE_DEG) && !Chassis_Set_Velocity(0.1f,0,0));
    step(10000U,1U);
    assert(controls == 6U && enables == 3U && free_mailboxes == 0U);
    assert(delays == delay_before + 6U);
    for (unsigned i = 0; i < 3U; ++i) near(motor_velocity[i], 0.0f);
    assert(!Chassis_Is_Stopped());
    confirm_stopped();
    controls = 0U; /* 各动作测试后续只统计活动控制帧，初始化帧已在上面断言。 */
    return 1U;
}

/** @brief 验证零速后的三轮新帧、停稳阈值、迟到反馈、计数回绕及下次起步。 @return 无。 */
static void test_stop_feedback(void)
{
    reset_case(0U, UINT32_MAX - 85U);
    assert(!Chassis_Is_Stopped());
    assert(Chassis_Enable());
    step(10000U, 1U); /* FC 已发送；此时收到的低速帧不能确认后续零速。 */
    for (unsigned i = 0U; i < 3U; ++i)
    {
        dm_motor[i].feedback_seq = UINT32_MAX - 2U;
        wheel_feedback(i, 0.0f, 1U);
    }
    step(10000U, 1U);
    for (unsigned i = 0U; i < 4U; ++i)
    {
        step(10000U, 1U);
        assert(!Chassis_Is_Stopped());
    }
    assert(!Chassis_Distance_Start(CHASSIS_TRANSLATION_FORWARD, 0.30f, 0.5f));
    /* 多次收到前两轮，第三轮尚无新帧，调用次数也不能充当确认次数。 */
    for (unsigned i = 0U; i < 4U; ++i)
    {
        wheel_feedback(0U, -0.20f, 1U);
        wheel_feedback(1U,  0.20f, 1U);
        step(10000U, 1U);
        assert(!Chassis_Is_Stopped());
    }
    wheel_feedback(0U, -0.20f, 1U);
    wheel_feedback(1U,  0.20f, 1U);
    wheel_feedback(2U,  0.0f, 1U);
    step(10000U, 1U);
    assert(!Chassis_Is_Stopped());
    stopped_group();
    assert(!Chassis_Is_Stopped());
    stopped_group();
    assert(Chassis_Is_Stopped());

    /* 任务延迟时，第一个轮子的帧比其余两帧早 40 ms，三份缓存不能拼一组。 */
    wheel_feedback(0U, 0.0f, 1U);
    test_tick += 40U;
    counter_us += 40000U;
    wheel_feedback(1U, 0.0f, 1U);
    wheel_feedback(2U, 0.0f, 1U);
    step(0U, 1U);
    assert(!Chassis_Is_Stopped());
    confirm_stopped();

    /* 刚确认后，单轮反向超速或失能都应撤销确认，须再等三组。 */
    wheel_feedback(1U, -0.201f, 1U);
    step(10000U, 1U);
    assert(!Chassis_Is_Stopped() && !Chassis_Set_Velocity(0.1f, 0, 0));
    stopped_group(); stopped_group();
    assert(!Chassis_Is_Stopped());
    wheel_feedback(2U, 0.0f, 0U);
    step(10000U, 1U);
    assert(!Chassis_Is_Stopped());
    confirm_stopped();

    /* 两轮持续上报、第三轮断流，不能带着上一次已停结果起步。 */
    for (unsigned i = 0U; i < 20U; ++i)
    {
        wheel_feedback(0U, 0.0f, 1U);
        wheel_feedback(1U, 0.0f, 1U);
        step(10000U, 1U);
        if (i == 2U) assert(Chassis_Is_Stopped());  /* 最后一组恰满 30 ms。 */
        if (i >= 3U) assert(!Chassis_Is_Stopped());
    }
    assert(!Chassis_Is_Stopped() && !Chassis_Turn_Start(1, 90.0f));
    confirm_stopped();
    assert(Chassis_Set_Velocity(0.1f, 0, 0));
    assert(!Chassis_Is_Stopped());
    step(10000U, 1U);
    Chassis_Stop();
    for (unsigned i = 0U; i < 3U; ++i)
        wheel_feedback(i, 0.0f, 1U); /* 首组停车指令尚未发出。 */
    step(10000U, 1U);
    stopped_group();
    Chassis_Stop(); /* 重复零速请求应让本轮制动继续计数。 */
    stopped_group();
    assert(!Chassis_Is_Stopped() && !Chassis_Straight_Start(0.1f, 0U));
    stopped_group();
    assert(Chassis_Is_Stopped());
    Chassis_Disable();
    assert(!Chassis_Is_Stopped());
    stopped_group();
    assert(!Chassis_Is_Stopped());
}

/** @brief 定时直行到期保持零速制动，三轮确认后失能；断流最多等待 500 ms。 @return 无。 */
static void test_timed_brake(void)
{
    for (unsigned with_feedback = 0U; with_feedback < 2U; ++with_feedback)
    {
        reset_case(0U, UINT32_MAX - 100U);
        assert(enable_ready() && Chassis_Straight_Start(0.4f, 20U));
        step(10000U, 1U); /* 此时开始累计 20 ms。 */
        step(10000U, 1U);
        assert(motor_velocity[1] < 0.0f);
        step(10000U, 1U);
        for (unsigned i = 0U; i < 3U; ++i)
            near(motor_velocity[i], 0.0f);
        assert(!Chassis_Is_Stopped() && disables == 0U);
        if (with_feedback)
        {
            stopped_group(); stopped_group();
            assert(!Chassis_Is_Stopped() && disables == 0U);
            stopped_group();
            assert(Chassis_Is_Stopped() && disables == 0U);
            assert(!Chassis_Straight_Start(0.1f, 0U)); /* 定时动作的 FD 尚未发完。 */
        }
        else
        {
            for (unsigned i = 0U; i < 49U; ++i)
                step(10000U, 1U);
            assert(!Chassis_Is_Stopped() && disables == 0U && aborts == 0U);
        }
        step(10000U, 1U);
        step(10000U, 1U);
        assert(disables == 3U && !Chassis_Is_Stopped());
    }
}

/** @brief MIT/FC/FD整组各两次等待；MIT/FC遇错退出，FD保留尝试其余电机的行为。 */
static void test_frame_spacing(void)
{
    reset_case(0U,0U);
    assert(enable_ready());
    uint32_t before = delays;
    step(10000U,1U);
    assert(controls == 3U && delays == before + 2U);
    Chassis_Disable();
    step(10000U,1U);
    assert(disables == 3U && delays == before + 4U);

    for (uint8_t id = 1U; id <= 3U; ++id)
    {
        reset_case(0U,0U);
        fail_control_id = id;
        assert(!Chassis_Enable());
        assert(controls == id - 1U && delays == id - 1U && aborts == 1U);
        assert(enables == 0U && disables == 0U);

        reset_case(0U,0U);
        assert(Chassis_Enable());
        fail_enable_id = id;
        step(10000U,1U);
        assert(enables == id - 1U && delays == 2U + id - 1U && aborts == 1U);
        assert(controls == 3U && disables == 0U);

        reset_case(0U,0U);
        Chassis_Disable();
        fail_disable_id = id;
        step(10000U,1U);
        assert(disables == 2U && delays == (id < 3U ? 1U : 2U));
        before = delays;
        step(10000U,1U);
        assert(disables == 5U && delays == before + 2U);
    }
}

/** @brief 独立物理矢量核对120度三轮几何，不用正反解互相掩盖符号错误。 */
static void test_kinematics(void)
{
    const float input[3][3] = {{1,0,0}, {0,1,0}, {0,0,1}};
    const float expected[3][3] = {
        {0.0f, 24.743583f, -24.743583f},
        {28.571429f, -14.285714f, -14.285714f},
        {3.0857143f, 3.0857143f, 3.0857143f}
    };
    float wheels[3], body[3];
    for (unsigned vector = 0; vector < 3U; ++vector)
    {
        Motor_Chassis_Inverse(input[vector][0], input[vector][1], input[vector][2], wheels);
        for (unsigned wheel = 0; wheel < 3U; ++wheel)
            assert(fabsf(wheels[wheel] - expected[vector][wheel]) < 0.00001f);
    }
    const float front_only[3] = {1,0,0};
    Motor_Chassis_Forward(front_only, body);
    assert(fabsf(body[0]) < 0.000001f);
    assert(fabsf(body[1] - 0.023333333f) < 0.000001f);
    assert(fabsf(body[2] - 0.10802469f) < 0.000001f);
    const float counter_rotating[3] = {0,-1,1};
    Motor_Chassis_Forward(counter_rotating, body);
    assert(fabsf(body[0] + 0.040414519f) < 0.000001f && body[1] == 0.0f && body[2] == 0.0f);
}

/** @brief 默认不动，FC与MIT隔开，5ms调度下10ms控制，实测抖动/回绕保持斜坡。 */
static void test_timing_and_parameters(void)
{
    float body[3];
    reset_case(UINT32_MAX - 7000U, UINT32_MAX - 15U);
    for (unsigned i = 0; i < 20U; ++i) step(5000U, 1U);
    assert(controls == 0U && enables == 0U && disables == 0U);
    dm_motor[1].velocity = 1.0f; dm_motor[2].velocity = -1.0f;
    step(10000U, 1U);
    near(chassis_speed[0], -0.040414519f); near(chassis_speed[1], 0); near(chassis_speed[2], 0);
    assert(controls == 0U); /* 反馈正解可在失能时观察，不触发运动。 */
    assert(!Chassis_Straight_Start(0.4f, 0U));
    assert(enable_ready() && enables == 3U && controls == 0U);
    step(5000U, 1U); assert(controls == 0U);
    step(5000U, 1U); assert(controls == 3U);
    body_target(body); near(body[0], 0); near(body[1], 0); near(body[2], 0);
    yaw_deg = 179.0f;
    read_advances_tick = 1U;
    assert(Chassis_Straight_Start(0.4f, 0U));
    yaw_deg = -179.0f; yaw_rate = 0.4f;
    step(15000U, 1U);
    body_target(body);
    near(body[0], 0.00675f); near(body[1], 0.0f);
    near(body[2], -0.16981317f); /* -2deg*2 - 0.25*0.4，检验原PID与跨零。 */
    assert(motor_velocity[1] < motor_velocity[2]);
    step(25000U, 1U);
    body_target(body); near(body[0], 0.018f); /* 0.45 m/s² 累计作用 40 ms。 */
    assert(controls == 9U && disables == 0U);

    /* 分别跨TIM2和RTOS计数零点；定时从首组运动MIT而非Start起计。 */
    reset_case(UINT32_MAX - 55000U, UINT32_MAX - 70U);
    assert(enable_ready());
    assert(Chassis_Straight_Start(0.4f, 20U));
    step(10000U, 1U); assert(controls == 3U && disables == 0U);
    body_target(body); near(body[0], 0.0045f);
    step(10000U, 1U); assert(controls == 6U && disables == 0U);
    step(10000U, 0U); assert(aborts == 1U && disables == 0U && controls == 6U);
    assert(!Chassis_Enable());
    step(10000U, 0U); assert(disables == 0U && controls == 6U);
    step(10000U, 1U); assert(disables == 3U && controls == 6U);
}

/** @brief 再次使能后重新确认三轮已停，前一次静止反馈不得放行起步。 */
static void test_reenable_order(void)
{
    reset_case(0U,0U);
    assert(enable_ready() && Chassis_Set_Velocity(0.4f,0,0));
    step(10000U,1U);
    assert(motor_velocity[1] != 0.0f);
    Chassis_Disable();
    step(10000U,0U); assert(disables == 0U);
    step(10000U,1U); assert(disables == 3U);
    step(10000U,1U);
    controls = enables = 0U;
    assert(Chassis_Enable() && controls == 3U && enables == 0U);
    for (unsigned i = 0; i < 3U; ++i) near(motor_velocity[i],0);
    assert(!Chassis_Straight_Start(0.4f,0U));
    step(10000U,1U); assert(controls == 3U && enables == 3U);
    step(10000U,1U); assert(controls == 6U);
    for (unsigned i = 0; i < 3U; ++i) near(motor_velocity[i],0);
    assert(!Chassis_Straight_Start(0.4f,0U));
    confirm_stopped();
    assert(Chassis_Straight_Start(0.4f,0U));
    step(10000U,1U);
    float body[3]; body_target(body); near(body[0],0.0045f);
}

/** @brief 核对八向从零在第 89 个 10 ms 周期达到 0.4 m/s，4 s 时仍维持同一目标。 */
static void test_translation(void)
{
    const float expected[8][2] = {
        {0.4f,0},{-0.4f,0},{0,0.4f},{0,-0.4f},
        {0.282842712f,0.282842712f},{0.282842712f,-0.282842712f},
        {-0.282842712f,0.282842712f},{-0.282842712f,-0.282842712f}
    };
    /* 直接核对发给 ID1/2/3 的轮速，覆盖前后校正及斜向组合。 */
    const float expected_motor[8][3] = {
        {0, -9.89743314f, 9.89743314f}, {0, 9.89743314f, -9.89743314f},
        {-11.42857143f, 5.71428571f, 5.71428571f}, {11.42857143f, -5.71428571f, -5.71428571f},
        {-8.08122034f, -2.95793191f, 11.03915225f}, {8.08122034f, -11.03915225f, 2.95793191f},
        {-8.08122034f, 11.03915225f, -2.95793191f}, {8.08122034f, 2.95793191f, -11.03915225f}
    };
    for (unsigned direction = 1; direction <= 8U; ++direction)
    {
        reset_case(0U, 0U);
        assert(enable_ready());
        assert(Chassis_Translation_Start((Chassis_Translation_Direction_t)direction, 0.4f, 0U));
        for (unsigned i = 0; i < 88U; ++i) step(10000U, 1U);
        float body[3]; body_target(body);
        near(hypotf(body[0],body[1]), 0.396f);
        step(10000U, 1U);
        body_target(body); near(hypotf(body[0],body[1]), 0.4f);
        for (unsigned i = 89U; i < 400U; ++i) step(10000U, 1U);
        body_target(body);
        near(body[0], expected[direction-1U][0]); near(body[1], expected[direction-1U][1]);
        for (unsigned i = 0U; i < 3U; ++i)
            near(motor_velocity[i], expected_motor[direction-1U][i]);
        near(hypotf(body[0],body[1]), 0.4f); near(body[2], 0.0f);
        assert(disables == 0U && aborts == 0U && enables == 3U && controls == 1200U);
    }
}

/** @brief 从三轮 MIT 指令积分八向定距的位移，检查方向、速度、加减速和到时停车。
 * @param[in] direction 八向平移枚举。
 * @param[in] ux,uy 该方向在车体前、左轴的单位分量。
 * @param[in] distance_m 请求的正距离，m。
 * @param[in] limit_mps 该次平移的速度上限，m/s。
 * @return 无。积分针对下发的速度指令，未模拟地面和电机。
 */
static void check_distance_profile(Chassis_Translation_Direction_t direction, float ux, float uy,
                                   float distance_m, float limit_mps)
{
    const double acceleration = 0.75;
    const double peak = fmin(limit_mps, sqrt(acceleration * distance_m));
    /* 加减速共占 peak²/a 的路程，其余路程以 peak 巡航。 */
    const double duration_s = peak / acceleration + distance_m / peak;
    unsigned moving_frames = (unsigned)ceil(duration_s * 100.0);
    double commanded_distance = 0.0;
    float body[3], previous = 0.0f, observed_peak = 0.0f;
    uint8_t braking = 0U;

    reset_case(UINT32_MAX - 180000U, UINT32_MAX - 180U);
    assert(enable_ready());
    assert(Chassis_Distance_Start(direction, distance_m, limit_mps));
    for (unsigned frame = 0; frame < moving_frames; ++frame)
    {
        step(10000U,1U);
        body_target(body);
        const float speed = ux * body[0] + uy * body[1];
        near(ux * body[1] - uy * body[0], 0.0f);
        near(body[2], 0.0f);
        assert(speed >= 0.0f && speed <= limit_mps + 0.00001f);
        assert(fabsf(speed - previous) <= 0.00755f);
        if (speed < previous - 0.00001f)
            braking = 1U;
        if (braking)
            assert(speed <= previous + 0.00001f);
        observed_peak = fmaxf(observed_peak, speed);
        commanded_distance += speed * 0.01;
        previous = speed;
    }
    assert(fabs(commanded_distance - distance_m) < fmin(0.000002, 0.0001 * distance_m));
    assert(observed_peak > 0.0f && peak - observed_peak < 0.0038);
    assert(!Chassis_Turn_Start(1,10.0f));
    yaw_deg = 10.0f; /* 终点这一拍，平移和锁航向必须一起结束。 */
    step(10000U,1U);
    body_target(body);
    near(body[0],0.0f); near(body[1],0.0f); near(body[2],0.0f);
    assert(previous <= 0.00755f && disables == 0U && aborts == 0U);
    assert(!Chassis_Distance_Start(direction, distance_m, limit_mps));
    confirm_stopped();
    assert(Chassis_Distance_Start(direction, distance_m, limit_mps));
}

/** @brief 核对三角形、梯形、低限速及不足一个控制周期的定距请求。 @return 无。 */
static void test_distance_profiles(void)
{
    const float c = 0.7071067812f;
    const float units[8][2] = {{1,0},{-1,0},{0,1},{0,-1},{c,c},{c,-c},{-c,c},{-c,-c}};
    for (unsigned i = 0; i < 8U; ++i)
        check_distance_profile((Chassis_Translation_Direction_t)(i + 1U),
                               units[i][0], units[i][1], 0.30f, 0.50f);
    check_distance_profile(CHASSIS_TRANSLATION_FORWARD, 1, 0, 0.50f, 0.50f);
    check_distance_profile(CHASSIS_TRANSLATION_BACKWARD, -1, 0, 0.50f * 0.50f / 0.75f, 0.50f);
    check_distance_profile(CHASSIS_TRANSLATION_FORWARD_LEFT, c, c, 1.91f, 0.50f);
    check_distance_profile(CHASSIS_TRANSLATION_FORWARD, 1, 0, 1.15f, 0.50f);
    check_distance_profile(CHASSIS_TRANSLATION_LEFT, 0, 1, 0.257f, 0.21f);
    check_distance_profile(CHASSIS_TRANSLATION_FORWARD, 1, 0, 0.000004f, 0.50f);
}

/** @brief 核对定距的使能顺序、变周期、抢占、复用和故障停机。 @return 无。 */
static void test_distance_lifecycle(void)
{
    const Chassis_Translation_Direction_t forward = CHASSIS_TRANSLATION_FORWARD;
    const float invalid[] = {0.0f, -1.0f, INFINITY, -INFINITY, NAN};
    float body[3];
    reset_case(0U,0U);
    assert(!Chassis_Distance_Start(forward, 0.5f, 0.5f));
    assert(enable_ready());
    for (unsigned i = 0U; i < sizeof(invalid)/sizeof(invalid[0]); ++i)
    {
        assert(!Chassis_Distance_Start(forward, invalid[i], 0.5f));
        assert(!Chassis_Distance_Start(forward, 0.5f, invalid[i]));
    }
    assert(!Chassis_Distance_Start(forward, 0.5f, 0.501f));
    assert(!Chassis_Distance_Start(forward, 1e30f, 0.5f));
    assert(!Chassis_Distance_Start(forward, 0.5f, 1e-30f));
    assert(!Chassis_Distance_Start(CHASSIS_TRANSLATION_NONE, 0.5f, 0.5f));
    assert(!Chassis_Distance_Start((Chassis_Translation_Direction_t)9, 0.5f, 0.5f));
    sample_age = 21U;
    assert(!Chassis_Distance_Start(forward, 0.5f, 0.5f));
    sample_age = 0U;
    assert(Chassis_Distance_Start(forward, 0.5f, 0.5f));
    step(10000U,1U); body_target(body); near(body[0],0.00375f);
    assert(!Chassis_Distance_Start(forward, 0.3f, 0.5f));
    yaw_deg = 2.0f;
    step(37000U,1U); body_target(body); near(body[0],0.0315f);
    near(body[2], -0.06981317f); /* 定距时沿原 PID 修正已偏离的航向。 */
    Chassis_Stop();
    step(10000U,1U); body_target(body);
    near(body[0],0.0f); near(body[1],0.0f); near(body[2],0.0f);
    assert(disables == 0U && aborts == 0U);
    confirm_stopped();
    assert(Chassis_Distance_Start(forward, 0.3f, 0.3f));
    step(10000U,1U); body_target(body); near(body[0],0.00375f);
    imu_ready = 0U;
    step(10000U,1U); assert(aborts == 1U);
    step(10000U,1U); assert(disables == 3U);
    imu_ready = 1U;
    assert(!Chassis_Distance_Start(forward, 0.3f, 0.3f));
}

/** @brief 连续三轴用同一缩放率，既限制线加速度也限制角加速度，更新目标不清斜坡。 */
static void test_vector(void)
{
    float body[3];
    reset_case(0U,0U);
    assert(enable_ready());
    assert(Chassis_Set_Velocity(0.24f,0.32f,0.8f));
    step(10000U,1U);
    body_target(body); near(body[0],0.0027f); near(body[1],0.0036f); near(body[2],0.009f);
    /* 连续速度入口的前向分量也必须采用实车校正后的电机方向。 */
    near(motor_velocity[0], -0.13062857f);
    near(motor_velocity[1], -0.04315053f);
    near(motor_velocity[2], 0.09046482f);
    step(15000U,1U);
    body_target(body); near(body[0],0.00675f); near(body[1],0.009f); near(body[2],0.0225f);
    assert(Chassis_Set_Velocity(0.024f,0.032f,-0.8f));
    step(10000U,1U);
    float changed[3]; body_target(changed);
    near(changed[2], 0.0065f); /* 反向请求按原 1.6 rad/s² 在 10 ms 内减少 0.016 rad/s。 */
    assert(hypotf(changed[0]-body[0],changed[1]-body[1]) <= 0.00451f);
    near((changed[0]-body[0])/(0.024f-body[0]), (changed[2]-body[2])/(-0.8f-body[2]));
    assert(changed[0] > body[0]); /* 更新目标未从零重新爬升。 */
    assert(!Chassis_Set_Velocity(0.4f,0.4f,0.0f));
    assert(!Chassis_Set_Velocity(NAN,0.0f,0.0f));
    assert(Chassis_Set_Velocity(0.0f,0.0f,0.0f));
    step(10000U,1U);
    body_target(body); near(body[0],0); near(body[1],0); near(body[2],0);
    assert(disables == 0U);
}

/** @brief 待命中的新动作沿用三轮发帧节拍，首拍只累计接单后的时间。 */
static void test_motion_start_cadence(void)
{
    const uint32_t accepted_us[] = {2000U, 5000U, 10000U};
    for (unsigned phase = 0U; phase < sizeof(accepted_us) / sizeof(accepted_us[0]); ++phase)
    for (unsigned action = 0U; action < 4U; ++action)
    {
        float body[3];
        reset_case(UINT32_MAX - 55000U, UINT32_MAX - 55U);
        assert(enable_ready());
        counter_us += accepted_us[phase];
        test_tick += accepted_us[phase] / 1000U;
        assert(Chassis_Is_Stopped());
        if (action == 0U)
            assert(Chassis_Set_Velocity(0.4f, 0.0f, 0.0f));
        else if (action == 1U)
            assert(Chassis_Straight_Start(0.4f, 10U));
        else if (action == 2U)
            assert(Chassis_Turn_Start(1, 180.0f));
        else
            assert(Chassis_Distance_Start(CHASSIS_TRANSLATION_FORWARD, 0.2f, 0.4f));
        const uint32_t first_us = 10000U - accepted_us[phase];
        step(first_us, 1U);
        assert(controls == 3U);
        body_target(body);
        const float dt = first_us * 0.000001f;
        const float vx = action == 2U ? 0.0f : action == 3U ?
            0.00375f : CHASSIS_STRAIGHT_ACCELERATION_MPS2 * dt;
        near(body[0], vx);
        near(body[1], 0.0f);
        near(body[2], action == 2U ? CHASSIS_TURN_ACCEL_RAD_S2 * dt : 0.0f);
        step(10000U, 1U);
        assert(controls == 6U && aborts == 0U);
        if (action == 1U)
        {
            /* 同刻接单的零斜坡拍不占定时行驶时长。 */
            body_target(body);
            near(body[0], first_us == 0U ? 0.0045f : 0.0f);
        }
    }
}

/** @brief 在 5 ms 任务中回放追踪速度和锁航向修正，模拟 tick 内唤醒抖动。
 * @return 无；核对三轮提交次数、间隔和按 TIM2 实际间隔计算的轮速增量。
 */
static void test_tracking_wheel_cadence(void)
{
    const uint16_t wake_us[] = {820U, 270U, 660U, 150U, 920U, 350U, 100U, 450U, 550U, 50U};
    const float wheel_acceleration_limit =
        (CHASSIS_STRAIGHT_ACCELERATION_MPS2 + CHASSIS_CENTER_DISTANCE * CHASSIS_VECTOR_ACCEL_RAD_S2) /
        CHASSIS_WHEEL_RADIUS;
    reset_case(UINT32_MAX - 90000U, UINT32_MAX - 85U);
    assert(enable_ready());
    assert(Chassis_Set_Velocity(0.40f, 0.0f, 0.0f));
    const uint32_t epoch_us = counter_us;
    uint32_t last_group_us = epoch_us, max_gap_us = 0U;
    unsigned groups = 0U;
    float previous[3] = {0}, max_wheel_delta = 0.0f;
    for (unsigned slot = 1U; slot <= 1000U; ++slot)
    {
        counter_us = epoch_us + slot * 5000U + wake_us[(slot - 1U) % 10U];
        test_tick += 5U;
        free_mailboxes = 3U;
        if (slot % 4U == 1U)
        {
            const float phase = slot * 0.006f;
            yaw_deg = 2.0f * sinf(phase);
            yaw_rate = 0.02f * cosf(phase);
            assert(Chassis_Set_Velocity(0.40f * cosf(phase), 0.40f * sinf(phase), 0.0f));
        }
        const uint32_t before = controls;
        Chassis_Loop();
        if (controls != before)
        {
            assert(controls == before + 3U);
            const uint32_t gap = counter_us - last_group_us;
            if (gap > max_gap_us) max_gap_us = gap;
            last_group_us = counter_us;
            for (unsigned i = 0U; i < 3U; ++i)
            {
                const float delta = fabsf(motor_velocity[i] - previous[i]);
                assert(delta <= wheel_acceleration_limit * (gap * 0.000001f) + 0.00001f);
                max_wheel_delta = fmaxf(max_wheel_delta, delta);
                previous[i] = motor_velocity[i];
            }
            ++groups;
        }
    }
    printf("TRACK wheel groups %u/500, max gap %lu us, max target step %.3f rad/s\n",
           groups, (unsigned long)max_gap_us, max_wheel_delta);
    fflush(stdout);
    assert(groups == 500U && max_gap_us <= 10400U && max_wheel_delta < 0.19f);
    assert(disables == 0U && aborts == 0U);

    Chassis_Stop();
    step(5000U, 1U); assert(controls == 1500U);
    step(5000U, 1U); assert(controls == 1503U);
    for (unsigned i = 0U; i < 3U; ++i) near(motor_velocity[i], 0.0f);
    confirm_stopped();
}

/** @brief 验证纯横移锁航向、跨包角、显式转向与平移切换及停车后重锁。 */
static void test_vector_yaw_hold(void)
{
    float body[3];
    reset_case(0U, 0U);
    assert(enable_ready());
    yaw_deg = 179.0f;
    assert(Chassis_Set_Velocity(0.0f, 0.09f, 0.0f));
    for (unsigned i = 0U; i < 25U; ++i)
        step(10000U, 1U);
    body_target(body); near(body[1], 0.09f); near(body[2], 0.0f);

    yaw_deg = -179.0f;
    yaw_rate = 0.4f;
    assert(Chassis_Set_Velocity(0.0f, 0.08f, 0.0f));
    step(10000U, 1U);
    body_target(body); near(body[2], -0.016f);
    for (unsigned i = 0U; i < 12U; ++i)
    {
        assert(Chassis_Set_Velocity(0.0f, 0.08f, 0.0f));
        step(10000U, 1U);
    }
    body_target(body); near(body[1], 0.08f);
    near(body[2], -0.16981317f); /* 保留 179° 目标，按 -2° 和实测角速度修偏。 */

    assert(Chassis_Set_Velocity(0.0f, 0.0f, 0.20f));
    step(10000U, 1U);
    body_target(body); near(body[2], -0.15381317f);
    for (unsigned i = 0U; i < 30U; ++i)
        step(10000U, 1U);
    body_target(body); near(body[1], 0.0f); near(body[2], 0.20f);

    yaw_deg = -162.0f;
    yaw_rate = 0.0f;
    assert(Chassis_Set_Velocity(0.0f, 0.08f, 0.0f));
    step(10000U, 1U);
    body_target(body); near(body[2], 0.18875f);
    for (unsigned i = 0U; i < 25U; ++i)
        step(10000U, 1U);
    body_target(body); near(body[1], 0.08f); near(body[2], 0.0f);
    yaw_deg = -160.0f;
    step(10000U, 1U);
    body_target(body); near(body[2], -0.016f);

    assert(Chassis_Set_Velocity(0.0f, 0.0f, 0.0f));
    step(10000U, 1U);
    body_target(body); near(body[0], 0.0f); near(body[1], 0.0f); near(body[2], 0.0f);
    confirm_stopped();
    yaw_deg = 10.0f;
    assert(Chassis_Set_Velocity(0.08f, 0.0f, 0.0f));
    step(10000U, 1U);
    body_target(body); near(body[0], 0.0045f); near(body[2], 0.0f);
}

/** @brief 全零向量在平移和转向中都是零速STOP，保留使能且允许下一动作。 */
static void test_zero_vector_stops_actions(void)
{
    float body[3];
    reset_case(0U,0U);
    assert(enable_ready() && Chassis_Straight_Start(0.4f,0U));
    step(10000U,1U);
    body_target(body); assert(body[0] > 0.0f);
    imu_ready = 0U;
    assert(Chassis_Set_Velocity(0,0,0));
    step(10000U,1U);
    body_target(body); near(body[0],0); near(body[1],0); near(body[2],0);
    assert(disables == 0U && aborts == 0U && enables == 3U);
    confirm_stopped();
    imu_ready = 1U;
    assert(Chassis_Turn_Start(1,CHASSIS_TURN_LEFT_ANGLE_DEG));
    step(10000U,1U);
    body_target(body); assert(body[2] > 0.0f);
    imu_ready = 0U;
    assert(Chassis_Set_Velocity(0,0,0));
    step(10000U,1U);
    body_target(body); near(body[0],0); near(body[1],0); near(body[2],0);
    assert(disables == 0U && aborts == 0U && enables == 3U);
    confirm_stopped();
    imu_ready = 1U;
    assert(Chassis_Translation_Start(CHASSIS_TRANSLATION_LEFT,0.4f,0U));
}

/** @brief 拒绝非法角度；双向180/360度跨±180、200ms连续到位和15s超时沿旧逻辑。 */
static void test_turn(void)
{
    const float invalid_angles[] = {0.0f, -1.0f, 360.01f, INFINITY, -INFINITY, NAN};
    reset_case(0U,0U);
    assert(enable_ready());
    for (unsigned i = 0U; i < sizeof(invalid_angles)/sizeof(invalid_angles[0]); ++i)
    {
        assert(!Chassis_Turn_Start(1,invalid_angles[i]));
        assert(!Chassis_Turn_Start(-1,invalid_angles[i]));
    }
    assert(!Chassis_Turn_Start(0,180.0f) && !Chassis_Turn_Start(2,180.0f) &&
           !Chassis_Turn_Start(-2,180.0f));
    assert(controls == 0U && enables == 3U && disables == 0U && aborts == 0U);
    assert(Chassis_Turn_Start(1,180.0f)); /* 非法参数未改变待命状态。 */

    for (unsigned half_turns = 1U; half_turns <= 2U; ++half_turns)
    for (int direction = -1; direction <= 1; direction += 2)
    {
        float body[3];
        float angle_deg = half_turns * 180.0f;
        reset_case(0U, 0U);
        yaw_deg = direction * 170.0f;
        assert(enable_ready() && Chassis_Turn_Start((int8_t)direction,angle_deg));
        step(10000U,1U);
        body_target(body); near(body[2],direction*0.04f);
        assert(!Chassis_Straight_Start(0.4f,0U)); /* 起始航向不能视为完成请求转角。 */
        for (unsigned sample = 1U; sample <= half_turns * 2U; ++sample)
        {
            yaw_deg = remainderf(direction*(170.0f+90.0f*sample),360.0f);
            yaw_rate = direction*0.2f;
            step(10000U,1U);
        }
        /* 剩余1°和低角速度测试转向原Kp=2,Kd=0.1，随后回到请求角度终点。 */
        yaw_deg = remainderf(direction*(170.0f+angle_deg-1.0f),360.0f);
        yaw_rate = direction*0.05f;
        for (unsigned i = 0U; i < 3U; ++i) step(10000U,1U);
        body_target(body); near(body[2],direction*0.029906585f);
        yaw_deg = remainderf(direction*(170.0f+angle_deg),360.0f); yaw_rate = 0.0f;
        for (unsigned i = 0; i < 10U; ++i) step(10000U,1U);
        yaw_rate = direction*0.09f; step(10000U,1U); /* 大于0.08需重新累计。 */
        yaw_rate = 0.0f; step(10000U,1U);
        for (unsigned i = 0; i < 19U; ++i) step(10000U,1U);
        assert(!Chassis_Straight_Start(0.4f,0U));
        step(10000U,1U); /* 满200ms，回零速使能待命。 */
        body_target(body); near(body[2],0);
        assert(!Chassis_Is_Stopped());
        confirm_stopped();
        assert(disables == 0U && Chassis_Straight_Start(0.4f,0U));
    }
    reset_case(0U,UINT32_MAX-5000U);
    assert(enable_ready() && Chassis_Turn_Start(1,CHASSIS_TURN_LEFT_ANGLE_DEG));
    for (unsigned i = 0; i < 1499U; ++i) step(10000U,1U);
    assert(aborts == 0U && disables == 0U);
    uint32_t before = controls;
    step(10000U,0U);
    assert(aborts == 1U && controls == before && disables == 0U);
    step(10000U,1U); assert(disables == 3U && controls == before);
}

/** @brief 核对转向实际 dt 斜坡、限速、到位制动与中止后清零，计时跨回绕。
 * @return 无；左右方向均覆盖，模拟突然报告到角以排除带大速度提前结束。
 */
static void test_turn_ramp(void)
{
    for (int direction = -1; direction <= 1; direction += 2)
    {
        float body[3], previous;
        reset_case(UINT32_MAX - 60000U, UINT32_MAX - 70U);
        assert(enable_ready() && Chassis_Turn_Start((int8_t)direction, 180.0f));
        step(25000U,1U); body_target(body); near(body[2],direction * 0.100f);
        step(10000U,1U); body_target(body); near(body[2],direction * 0.140f);
        step(15000U,1U); body_target(body); near(body[2],direction * 0.200f);
        previous = body[2];
        for (unsigned i = 0U; i < 60U; ++i)
        {
            step(10000U,1U);
            body_target(body);
            assert(fabsf(body[2]) <= 1.00001f);
            assert(fabsf(body[2] - previous) <= 0.04001f);
            previous = body[2];
        }
        near(body[2], 1.0f * direction);

        /* 瞬间到角、实际角速度已低时，较大的待发目标仍须减速。 */
        yaw_deg = direction * 180.0f;
        yaw_rate = 0.0f;
        for (unsigned i = 0U; i < 21U; ++i)
        {
            step(10000U,1U);
            body_target(body);
            near(body[2], direction * (1.0f - 0.040f * (i + 1U)));
        }
        assert(fabsf(body[2]) > 0.08f && !Chassis_Is_Stopped());
        previous = body[2];
        for (unsigned i = 0U; i < 65U; ++i)
        {
            step(10000U,1U);
            body_target(body);
            assert(fabsf(body[2] - previous) <= 0.04001f);
            previous = body[2];
        }
        near(body[2],0.0f);
        confirm_stopped();

        /* 上一段斜坡已清零，反向新动作从首拍 0.04 rad/s 开始。 */
        assert(Chassis_Turn_Start((int8_t)-direction, 90.0f));
        step(10000U,1U); body_target(body); near(body[2],-direction * 0.040f);
        for (unsigned i = 0U; i < 9U; ++i) step(10000U,1U);
        Chassis_Stop();
        step(10000U,1U); body_target(body); near(body[2],0.0f);
        assert(disables == 0U && !Chassis_Is_Stopped());
        confirm_stopped();
        assert(Chassis_Turn_Start((int8_t)direction, 90.0f));
        step(10000U,1U); body_target(body); near(body[2],direction * 0.040f);
        Chassis_Disable();
        step(10000U,1U);
        assert(disables == 3U && !Chassis_Is_Stopped());
    }
}

/** @brief 独立核对剩余 6° 的制动包络、5° 处原 PID 接管以及过冲反向修正。
 * @return 无；固定模拟角度以分离速度目标与真实车体动态，左右均受同一斜坡约束。
 */
static void test_turn_braking_and_fine(void)
{
    for (int direction = -1; direction <= 1; direction += 2)
    {
        float body[3], previous;
        reset_case(0U,0U);
        assert(enable_ready() && Chassis_Turn_Start((int8_t)direction, 180.0f));
        for (unsigned i = 0U; i < 50U; ++i) step(10000U,1U);
        body_target(body); near(body[2], direction * 1.0f);

        /* 6° 制动角对应 0.915291 rad/s，原 P 输出仅 0.209440 rad/s。 */
        yaw_deg = direction * 174.0f;
        step(10000U,1U); body_target(body); near(body[2], direction * 0.96f);
        previous = body[2];
        for (unsigned i = 0U; i < 25U; ++i)
        {
            step(10000U,1U); body_target(body);
            assert(fabsf(body[2] - previous) <= 0.04001f);
            previous = body[2];
        }
        near(body[2], direction * 0.9152912f);

        /* 恰好 5° 时切回 Kp=2；目标变化仍经过每拍 0.04 rad/s 的限增量。 */
        yaw_deg = direction * 175.0f;
        step(10000U,1U); body_target(body);
        near(body[2], direction * 0.8752912f);
        previous = body[2];
        for (unsigned i = 0U; i < 30U; ++i)
        {
            step(10000U,1U); body_target(body);
            assert(fabsf(body[2] - previous) <= 0.04001f);
            previous = body[2];
        }
        near(body[2], direction * 0.17453293f);
        assert(!Chassis_Is_Stopped());

        /* 越过终点 3°，同向角速度 0.2 rad/s，原 Kd=0.1 促使反向修正。 */
        yaw_deg = direction * -177.0f;
        yaw_rate = direction * 0.2f;
        for (unsigned i = 0U; i < 10U; ++i)
        {
            step(10000U,1U); body_target(body);
            assert(fabsf(body[2] - previous) <= 0.04001f);
            previous = body[2];
        }
        near(body[2], direction * -0.12471976f);
        assert(!Chassis_Is_Stopped() && disables == 0U && aborts == 0U);
    }
}

/** @brief 以理想轮速跟随闭环推进左右小角、半圈与整圈，检查跨零、斜坡和停车。
 * @return 无；只验证指令算法，地面抓地与电机动态由实测判断。
 */
static void test_turn_closed_loop(void)
{
    const float angles[] = {1.0f, 5.0f, 6.0f, 10.0f, 90.0f, 180.0f, 360.0f};
    const uint32_t intervals_us[] = {10000U,15000U,10000U,20000U,10000U,25000U};
    for (unsigned angle = 0U; angle < sizeof(angles)/sizeof(angles[0]); ++angle)
    for (int direction = -1; direction <= 1; direction += 2)
    {
        float body[3] = {0.0f,0.0f,0.0f}, travelled_deg = 0.0f, peak = 0.0f;
        float overshoot_deg = 0.0f;
        unsigned fine_exits = 0U, reversals = 0U;
        uint8_t in_fine = angles[angle] <= 5.0f;
        int last_direction = direction;
        reset_case(UINT32_MAX - 120000U, UINT32_MAX - 120U);
        const float start_yaw = direction * 170.0f;
        yaw_deg = start_yaw;
        assert(enable_ready() && Chassis_Turn_Start((int8_t)direction, angles[angle]));
        uint32_t started_ms = test_tick;
        unsigned frame;
        for (frame = 0U; frame < 900U && !Chassis_Is_Stopped(); ++frame)
        {
            uint32_t interval = intervals_us[frame % 6U];
            float dt = interval * 0.000001f;
            const float previous = body[2];
            yaw_rate = previous;
            travelled_deg += yaw_rate * dt * 57.295779513f;
            overshoot_deg = fmaxf(overshoot_deg, direction * travelled_deg - angles[angle]);
            uint8_t next_fine = fabsf(direction * angles[angle] - travelled_deg) <= 5.0f;
            if (in_fine && !next_fine) ++fine_exits;
            in_fine = next_fine;
            yaw_deg = remainderf(start_yaw + travelled_deg, 360.0f);
            for (unsigned i = 0U; i < 3U; ++i)
                wheel_feedback(i, motor_velocity[i], 1U);
            step(interval,1U);
            body_target(body);
            near(body[0],0.0f); near(body[1],0.0f);
            assert(fabsf(body[2]) <= 1.00001f);
            /* 正常 Stop 在低速到位后回零；其余更新须满足斜坡界限。 */
            if (body[2] != 0.0f || fabsf(previous) > 0.08f)
                assert(fabsf(body[2] - previous) <= 4.0f * dt + 0.00001f);
            peak = fmaxf(peak, fabsf(body[2]));
            if (last_direction * body[2] < -0.01f)
            {
                ++reversals;
                last_direction = -last_direction;
            }
        }
        printf("TURN ideal %+.0f deg: %.3f s, error %.3f deg, overshoot %.3f deg, fine exits %u, reversals %u\n",
               direction * angles[angle], (uint32_t)(test_tick - started_ms) * 0.001,
               travelled_deg - direction * angles[angle], overshoot_deg, fine_exits, reversals);
        assert(frame < 900U && (uint32_t)(test_tick - started_ms) < 15000U);
        assert(disables == 0U && aborts == 0U && Chassis_Is_Stopped());
        assert(direction * travelled_deg > 0.0f);
        assert(fabsf(travelled_deg - direction * angles[angle]) <= 2.01f);
        assert(fine_exits == 0U && reversals <= 1U);
        assert(peak > 0.0f);
        if (angles[angle] >= 180.0f)
            near(peak,1.0f);
        assert(Chassis_Straight_Start(0.1f,0U));
    }
}

/** @brief 转向末段直接接定距，核对最新姿态、原终点航向、变 dt 衔接与拒绝请求不改状态。
 * @return 无；左右半圈/整圈均覆盖，5° 边界与过冲场景不等待停车确认。
 */
static void test_turn_distance_handoff(void)
{
    const uint32_t intervals_us[] = {15000U,25000U,10000U};
    for (unsigned half_turns = 1U; half_turns <= 2U; ++half_turns)
    for (int direction = -1; direction <= 1; direction += 2)
    for (unsigned overshot = 0U; overshot < 2U; ++overshot)
    {
        float body[3], previous;
        const float angle = 180.0f * half_turns;
        const float start_yaw = 170.0f * direction;
        const float remaining = direction * (overshot ? -3.0f : 5.0f);
        reset_case(UINT32_MAX - 280000U, UINT32_MAX - 280U);
        yaw_deg = start_yaw;
        assert(enable_ready() && Chassis_Turn_Start((int8_t)direction, angle));
        /* 整圈起点与终点 yaw 相同，累计转角尚未完成时仍必须拒绝。 */
        assert(!Chassis_Distance_Start(CHASSIS_TRANSLATION_FORWARD,1.0f,0.5f));
        for (unsigned i = 0U; i < 20U; ++i) step(10000U,1U);
        body_target(body); near(body[2], direction * 0.8f);
        for (unsigned i = 1U; i < half_turns * 2U; ++i)
        {
            yaw_deg = remainderf(start_yaw + direction * 90.0f * i, 360.0f);
            yaw_rate = direction * 0.2f;
            step(10000U,1U);
        }
        body_target(body); previous = body[2];
        for (unsigned i = 0U; i < 3U; ++i)
            wheel_feedback(i, motor_velocity[i], 1U); /* 反馈仍在转动，不能伪装停车。 */
        assert(!Chassis_Is_Stopped());
        uint32_t before = controls;
        step(5000U,1U); assert(controls == before);

        /* INS 已更新，但 Service_Turn 尚未消费这次跨圈后的末段角度。 */
        yaw_deg = remainderf(start_yaw + direction * angle - direction * 5.01f,360.0f);
        assert(!Chassis_Distance_Start(CHASSIS_TRANSLATION_FORWARD,1.0f,0.5f));
        yaw_deg = remainderf(start_yaw + direction * angle - remaining,360.0f);
        assert(!Chassis_Distance_Start(CHASSIS_TRANSLATION_NONE,1.0f,0.5f));
        assert(!Chassis_Distance_Start((Chassis_Translation_Direction_t)9,1.0f,0.5f));
        assert(!Chassis_Distance_Start(CHASSIS_TRANSLATION_FORWARD,NAN,0.5f));
        assert(!Chassis_Distance_Start(CHASSIS_TRANSLATION_FORWARD,1.0f,0.501f));
        assert(!Chassis_Distance_Start(CHASSIS_TRANSLATION_FORWARD,1e30f,0.5f));
        sample_age = 21U;
        assert(!Chassis_Distance_Start(CHASSIS_TRANSLATION_FORWARD,1.0f,0.5f));
        sample_age = 0U; imu_ready = 0U;
        assert(!Chassis_Distance_Start(CHASSIS_TRANSLATION_FORWARD,1.0f,0.5f));
        imu_ready = 1U;
        assert(!Chassis_Straight_Start(0.1f,0U)); /* 定时直行仍要求先停车。 */
        assert(Chassis_Distance_Start(CHASSIS_TRANSLATION_FORWARD,1.0f,0.5f));
        assert(!Chassis_Is_Stopped() && controls == before);
        assert(!Chassis_Distance_Start(CHASSIS_TRANSLATION_FORWARD,0.5f,0.5f));
        step(5000U,1U); /* 仍在原来 10 ms 时点发送，接受请求不重置控制时基。 */
        assert(controls == before + 3U);
        body_target(body); near(body[0],0.00375f); near(body[1],0.0f);
        near(body[2],previous - direction * 0.04f);
        previous = body[2];

        for (unsigned i = 0U; i < 40U; ++i)
        {
            uint32_t interval = intervals_us[i % 3U];
            step(interval,1U); body_target(body);
            assert(fabsf(body[2] - previous) <= interval * 0.000004f + 0.00001f);
            previous = body[2];
        }
        /* 保留转向终点；若误锁切换时的 yaw，这里只剩 -Kd*rate。 */
        const float expected = remaining * 0.03490658504f - direction * 0.05f;
        near(body[2],expected);
        yaw_rate = -direction * 0.6f;
        step(10000U,1U); body_target(body);
        near(body[2],expected + direction * 0.2f); /* 已追上后退出衔接，恢复原直行 PID。 */
        assert(disables == 0U && aborts == 0U);

        Chassis_Stop(); step(10000U,1U); body_target(body);
        near(body[0],0.0f); near(body[2],0.0f);
        confirm_stopped();
        yaw_rate = 0.0f;
        assert(Chassis_Distance_Start(CHASSIS_TRANSLATION_FORWARD,0.5f,0.5f));
        yaw_deg = remainderf(yaw_deg + 2.0f,360.0f);
        step(10000U,1U); body_target(body);
        near(body[2],-0.06981317f); /* 普通待命启动不残留转向衔接斜坡。 */
    }
}

/** @brief 最新 yaw 恰差 180° 时按实测旋向或请求方向消歧，不漏掉末段接续。
 * @return 无；正反方向、零与非零陀螺读数均检查。
 */
static void test_turn_handoff_half_circle_delta(void)
{
    for (int direction = -1; direction <= 1; direction += 2)
    for (unsigned moving = 0U; moving < 2U; ++moving)
    {
        float body[3];
        reset_case(0U,0U);
        assert(enable_ready() && Chassis_Turn_Start((int8_t)direction,180.0f));
        step(10000U,1U);
        yaw_deg = -direction * 180.0f; /* 包角表示刻意选与实际旋向相反的半圈符号。 */
        yaw_rate = direction * (moving ? 0.2f : 0.0f);
        assert(!Chassis_Is_Stopped());
        assert(Chassis_Distance_Start(CHASSIS_TRANSLATION_LEFT,0.5f,0.5f));
        for (unsigned i = 0U; i < 5U; ++i) step(10000U,1U);
        body_target(body); near(body[0],0.0f); assert(body[1] > 0.0f);
        near(body[2],moving ? -direction * 0.05f : 0.0f);
    }
}

/** @brief 极短定距仍按原到时 Stop 结束，未完成的转向衔接不延长动作或污染下一次启动。
 * @return 无；显式 Stop 与正常结束的清零语义一致。
 */
static void test_turn_handoff_short_distance(void)
{
    float body[3];
    reset_case(0U,0U);
    assert(enable_ready() && Chassis_Turn_Start(1,180.0f));
    for (unsigned i = 0U; i < 20U; ++i) step(10000U,1U);
    yaw_deg = 177.0f;
    assert(Chassis_Distance_Start(CHASSIS_TRANSLATION_FORWARD,0.000004f,0.5f));
    step(10000U,1U); body_target(body);
    near(body[0],0.0004f); near(body[2],0.76f);
    assert(!Chassis_Is_Stopped());
    step(10000U,1U); body_target(body);
    near(body[0],0.0f); near(body[2],0.0f);
    confirm_stopped();
    assert(Chassis_Straight_Start(0.1f,0U));
    yaw_deg = 179.0f;
    step(10000U,1U); body_target(body);
    near(body[2],-0.06981317f);
    assert(disables == 0U && aborts == 0U);
}

/** @brief 理想跟随下左右转向末段边走边修偏，验证不停轮接续及下一段定距终止。
 * @return 无；仅模拟命令跟随，不把路径积分当作实车地面位移。
 */
static void test_turn_handoff_closed_loop(void)
{
    for (int direction = -1; direction <= 1; direction += 2)
    {
        float body[3] = {0.0f,0.0f,0.0f}, travelled_deg = 0.0f;
        double commanded_distance = 0.0;
        uint8_t handed_off = 0U, translated_while_turning = 0U;
        unsigned handoff_frame = 0U, frame;
        reset_case(UINT32_MAX - 120000U,UINT32_MAX - 120U);
        yaw_deg = direction * 170.0f;
        assert(enable_ready() && Chassis_Turn_Start((int8_t)direction,180.0f));
        for (frame = 0U; frame < 900U; ++frame)
        {
            yaw_rate = body[2];
            travelled_deg += yaw_rate * 0.01f * 57.295779513f;
            commanded_distance += body[0] * 0.01;
            yaw_deg = remainderf(direction * 170.0f + travelled_deg,360.0f);
            for (unsigned i = 0U; i < 3U; ++i)
                wheel_feedback(i,motor_velocity[i],1U);
            step(10000U,1U); body_target(body);
            if (!handed_off)
            {
                assert(!Chassis_Is_Stopped());
                handed_off = Chassis_Distance_Start(CHASSIS_TRANSLATION_FORWARD,1.0f,0.5f);
                if (handed_off)
                {
                    assert(fabsf(direction * 180.0f - travelled_deg) <= 5.0f);
                    assert(fabsf(yaw_rate) > 0.08f);
                    handoff_frame = frame;
                }
            }
            else if (body[0] > 0.0f && fabsf(yaw_rate) > 0.08f)
                translated_while_turning = 1U;
            if (handed_off && Chassis_Is_Stopped())
                break;
        }
        assert(frame < 900U && handed_off && translated_while_turning);
        assert(fabs(commanded_distance - 1.0) < 0.000002);
        assert(fabsf(travelled_deg - direction * 180.0f) < 2.0f);
        assert(disables == 0U && aborts == 0U && enables == 3U);
        printf("TURN+DISTANCE ideal %+d deg: handoff %.2f s, total %.2f s, final error %.3f deg\n",
               direction * 180,(handoff_frame + 1U) * 0.01,(frame + 1U) * 0.01,
               travelled_deg - direction * 180.0f);
    }
}

/** @brief 姿态期限、异步取消和部分发送失败均停止生成新运动指令。 */
static void test_failure_paths(void)
{
    reset_case(0U,0U);
    assert(enable_ready()); imu_ready = 0U;
    assert(!Chassis_Straight_Start(0.4f,0U));
    imu_ready = 1U; sample_age = 21U;
    assert(!Chassis_Straight_Start(0.4f,0U));
    sample_age = 20U;
    assert(Chassis_Straight_Start(0.4f,0U));
    step(10000U,1U); assert(controls == 3U);
    sample_age = 21U;
    step(10000U,1U); assert(aborts == 1U && controls == 3U);
    step(10000U,1U); assert(disables == 3U);
    imu_ready = 1U; sample_age = 0U;
    step(10000U,1U); assert(enables == 3U && controls == 3U); /* 恢复不自动重启。 */

    reset_case(0U,0U);
    assert(enable_ready() && Chassis_Straight_Start(0.4f,0U));
    fail_control_id = 2U;
    step(10000U,1U); assert(controls == 1U && aborts == 1U);
    step(10000U,0U); assert(disables == 0U && controls == 1U);
    step(10000U,1U); assert(disables == 3U);
}

/** @brief 执行底盘真实源码回归。 @return 成功为0。 */
int main(void)
{
    test_frame_spacing();
    test_stop_feedback();
    test_timed_brake();
    test_kinematics();
    test_timing_and_parameters();
    test_reenable_order();
    test_translation();
    test_distance_profiles();
    test_distance_lifecycle();
    test_vector();
    test_motion_start_cadence();
    test_tracking_wheel_cadence();
    test_vector_yaw_hold();
    test_zero_vector_stops_actions();
    test_turn();
    test_turn_ramp();
    test_turn_braking_and_fine();
    test_turn_closed_loop();
    test_turn_distance_handoff();
    test_turn_handoff_half_circle_delta();
    test_turn_handoff_short_distance();
    test_turn_handoff_closed_loop();
    test_failure_paths();
    puts("PASS chassis: kinematics, measured dt/wrap, 8-way/vector/turn, distance and fresh three-wheel stop");
    return 0;
}
