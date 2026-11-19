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
static float captured_torque[3], captured_kd[3];
static uint8_t tuning_test;
static uint32_t sample_age;
static uint8_t imu_ready, read_advances_tick, fail_control_id, abort_pending;
static uint8_t read_injects_feedback;
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
    data->yaw_predicted = 0U;
    assert(data != NULL);
    if (!imu_ready) return 0U;
    if (read_advances_tick) { ++test_tick; read_advances_tick = 0U; }
    if (read_injects_feedback)
    {
        ++test_tick;
        for (unsigned i = 0U; i < 3U; ++i)
        {
            dm_motor[i].id = (uint8_t)(i + 1U);
            dm_motor[i].state = 1U;
            dm_motor[i].velocity = motor_velocity[i];
            dm_motor[i].feedback_time_ms = test_tick;
            ++dm_motor[i].feedback_seq;
        }
        read_injects_feedback = 0U;
    }
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
    assert(position == 0.0f && kp == 0.0f);
    if (!tuning_test)
        assert(torque == 0.0f && kd == expected_kd[id-1U]);
    assert(isfinite(torque) && fabsf(torque) <= 0.05f);
    assert(isfinite(velocity) && fabsf(velocity) <= 40.0f && !abort_pending);
    last_sent_id = 0U;
    if (id == fail_control_id) { fail_control_id = 0U; return HAL_ERROR; }
    last_sent_id = (uint8_t)id;
    motor_velocity[id-1U] = velocity;
    captured_torque[id-1U] = torque;
    captured_kd[id-1U] = kd;
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
    read_injects_feedback = 0U;
    fail_enable_id = fail_disable_id = last_sent_id = 0U;
    yaw_deg = yaw_rate = 0.0f;
    tuning_test = 0U;
    memset(motor_velocity, 0, sizeof(motor_velocity));
    memset((void *)dm_motor, 0, sizeof(dm_motor));
    Chassis_Init();
    /* 既有运动规划回归保留零前馈基线；默认实车配置另有独立用例。 */
    const float legacy_kd[3] = {0.2f, 0.25f, 0.2f}, no_friction[3] = {0.0f, 0.0f, 0.0f};
    assert(Chassis_Set_Tuning(legacy_kd, 0.0f, no_friction));
    assert(enables == 0U && controls == 0U && disables == 0U);
}

/** @brief 三轮方向固定-1，反解捕获的真实指令为车体速度供动作断言。 */
static void body_target(float body[3])
{
    float wheels[3] = {-motor_velocity[0], -motor_velocity[1], -motor_velocity[2]};
    Motor_Chassis_Forward(wheels, body);
}

/** @brief 推进外部轮速采样，不把控制目标直接写入里程计状态。 */
static void feedback_step(uint32_t elapsed_us, const float wheels[3])
{
    counter_us += elapsed_us;
    test_tick += elapsed_us / 1000U;
    for (unsigned i = 0U; i < 3U; ++i)
    {
        dm_motor[i].position = remainderf(dm_motor[i].position + wheels[i] * elapsed_us * .000001f, 25.0f);
        wheel_feedback(i, wheels[i], 1U);
    }
    step(0U,1U);
}

/** @brief 模拟上一拍目标的部分跟随，独立累计固定航向下的物理位移。
 * @note 使用零阶保持积分模拟地面；不调用被测里程计积分函数。
 */
static void plant_step(uint32_t elapsed_us, float gain, double *x, double *y)
{
    float body[3], wheels[3];
    body_target(body);
    const double angle = yaw_deg * 0.017453292519943295;
    const double dt = elapsed_us * 1e-6;
    if (x != NULL)
        *x += gain * (body[0] * cos(angle) - body[1] * sin(angle)) * dt;
    if (y != NULL)
        *y += gain * (body[0] * sin(angle) + body[1] * cos(angle)) * dt;
    for (unsigned i = 0U; i < 3U; ++i)
        wheels[i] = gain * motor_velocity[i];
    feedback_step(elapsed_us, wheels);
}

/** @brief 比较物理量，浮点误差小于协议量化前的1e-5。 */
static void near_at(float actual, float expected, unsigned line)
{
    if (fabsf(actual-expected)>=.00001f)
        fprintf(stderr,"line %u: actual %.9f, expected %.9f\n",line,actual,expected);
    assert(fabsf(actual - expected) < 0.00001f);
}
#define near(actual, expected) near_at((actual), (expected), __LINE__)

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
    for (unsigned i = 0U; i < 3U; ++i)
        wheel_feedback(i, 0.0f, 1U);
    step(10000U, 1U); /* FC 已发送；此时收到的低速帧不能确认后续零速。 */
    step(10000U, 1U);
    assert(!Chassis_Motion_Ready()); /* FC 前的三帧仍在 30 ms 内。 */
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
    assert(!Chassis_Is_Stopped() && Chassis_Motion_Ready());
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
    assert(!Chassis_Is_Stopped() && Chassis_Motion_Ready());
    stopped_group();
    assert(Chassis_Is_Stopped());
    Chassis_Disable();
    assert(!Chassis_Is_Stopped());
    stopped_group();
    assert(!Chassis_Is_Stopped());
}

/** @brief 正常定时结束斜坡回零并保持使能；减速期间失去有效反馈仍故障制动。 */
static void test_timed_brake(void)
{
    for (unsigned with_feedback = 0U; with_feedback < 2U; ++with_feedback)
    {
        reset_case(0U, UINT32_MAX - 100U);
        assert(enable_ready() && Chassis_Straight_Start(0.4f, 20U));
        plant_step(10000U, 1.0f, NULL, NULL); /* 此时开始累计 20 ms。 */
        plant_step(10000U, 1.0f, NULL, NULL);
        assert(motor_velocity[1] < 0.0f);
        const float before = motor_velocity[1];
        plant_step(10000U, 1.0f, NULL, NULL);
        assert(before < motor_velocity[1] && motor_velocity[1] < 0.0f && !Chassis_Motion_Ready());
        if (with_feedback)
        {
            for (unsigned i = 0; i < 100; ++i)
                plant_step(10000U, 1.0f, NULL, NULL);
            assert(Chassis_Is_Stopped() && Chassis_Motion_Ready() && disables == 0U && aborts == 0U);
            assert(Chassis_Straight_Start(0.1f, 0U));
        }
        else
        {
            for (unsigned i = 0U; i < 65U; ++i)
                step(10000U, 1U);
            assert(disables == 3U && !Chassis_Is_Stopped());
        }
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
    const float input[][3] = {
        {1,0,0}, {0,1,0}, {0,0,1}, {-1,0,0}, {0,-1,0}, {0,0,-1},
        {.18f,-.07f,.3f}, {-.12f,.16f,-.4f}
    };
    /* 用户确认的后/右前/左前接地点；以刚体接地点速度作独立判据。 */
    const float positions[3][2] = {
        {-.0982f,0}, {.0491f,-.0491f * sqrtf(3.0f)}, {.0491f,.0491f * sqrtf(3.0f)}
    };
    float wheels[3], body[3];
    for (unsigned vector = 0; vector < sizeof(input) / sizeof(input[0]); ++vector)
    {
        float physical_wheels[3];
        Motor_Chassis_Inverse(input[vector][0], input[vector][1], input[vector][2], wheels);
        for (unsigned wheel = 0; wheel < 3U; ++wheel)
        {
            const float x = positions[wheel][0], y = positions[wheel][1];
            const float tx = -y / .0982f, ty = x / .0982f;
            const float contact_vx = input[vector][0] - input[vector][2] * y;
            const float contact_vy = input[vector][1] + input[vector][2] * x;
            physical_wheels[wheel] = (tx * contact_vx + ty * contact_vy) / .035f;
            assert(fabsf(wheels[wheel] - physical_wheels[wheel]) < 0.00001f);
        }
        Motor_Chassis_Forward(physical_wheels, body);
        for (unsigned axis = 0; axis < 3U; ++axis)
            assert(fabsf(body[axis] - input[vector][axis]) < .000001f);
    }
    const float rear_only[3] = {1,0,0};
    Motor_Chassis_Forward(rear_only, body);
    assert(fabsf(body[0]) < 0.000001f);
    assert(fabsf(body[1] + 0.023333333f) < 0.000001f);
    assert(fabsf(body[2] - 0.11880516f) < 0.000001f);
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
    const float feedback[3] = {0.0f,1.0f,-1.0f};
    feedback_step(10000U,feedback);
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
    near(body[2], -0.024f); /* 跨零的 -0.1698 rad/s PID 请求按 1.6 rad/s² 接入。 */
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
    /* 上一拍的邮箱还没清空时跳过本拍，不卸力。下一拍邮箱空闲后继续发。 */
    step(10000U, 0U); assert(aborts == 0U && disables == 0U && controls == 6U);
    assert(Chassis_Is_Enabled());
    step(10000U, 1U); assert(disables == 0U && controls == 9U && aborts == 0U);
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
    /* 直接核对后/右前/左前轮的 MIT 目标，覆盖前后、左右及斜向组合。 */
    const float expected_motor[8][3] = {
        {0, -9.89743314f, 9.89743314f}, {0, 9.89743314f, -9.89743314f},
        {11.42857143f, -5.71428571f, -5.71428571f}, {-11.42857143f, 5.71428571f, 5.71428571f},
        {8.08122034f, -11.03915225f, 2.95793191f}, {-8.08122034f, -2.95793191f, 11.03915225f},
        {8.08122034f, 2.95793191f, -11.03915225f}, {-8.08122034f, 11.03915225f, -2.95793191f}
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

/** @brief 独立轮速跟随模拟验证八向反馈定距，覆盖变周期、非零航向和时钟回绕。 */
static void check_distance_profile(Chassis_Translation_Direction_t direction, float ux, float uy,
                                   float distance_m, float limit_mps, float gain, float heading_deg)
{
    const uint32_t intervals[] = {10000U,15000U,25000U};
    double x = 0.0, y = 0.0;
    float body[3], previous = 0.0f, peak = 0.0f;
    unsigned frame;
    reset_case(UINT32_MAX - 180000U, UINT32_MAX - 180U);
    yaw_deg = heading_deg;
    assert(enable_ready());
    assert(Chassis_Distance_Start(direction, distance_m, limit_mps));
    assert(!Chassis_Reset_Odometry());
    const double angle = heading_deg * 0.017453292519943295;
    const double world_x = ux * cos(angle) - uy * sin(angle);
    const double world_y = ux * sin(angle) + uy * cos(angle);
    for (frame = 0U; frame < 1800U; ++frame)
    {
        const uint32_t elapsed = intervals[frame % 3U];
        plant_step(elapsed,gain,&x,&y);
        body_target(body);
        const float speed = ux * body[0] + uy * body[1];
        near(ux * body[1] - uy * body[0],0.0f);
        near(body[2],0.0f);
        assert(speed >= 0.0f && speed <= limit_mps + 0.00001f);
        /* 终点置零属于停车请求；其余运动拍仍受既有加减速度限制。 */
        if (speed > 0.0f)
            assert(fabsf(speed - previous) <= CHASSIS_DISTANCE_ACCELERATION_MPS2 * elapsed * 1e-6f + 0.00002f);
        peak = fmaxf(peak,speed);
        previous = speed;
        assert(Chassis_Is_Enabled() && disables == 0U && aborts == 0U);
        if (Chassis_Is_Stopped())
            break;
    }
    assert(frame < 1800U && peak > 0.0f);
    const double progress = world_x * x + world_y * y;
    assert(fabs(progress - distance_m) <= CHASSIS_DISTANCE_TOLERANCE_M + 0.00025);
    assert(fabs(world_x * y - world_y * x) < 0.00001);
    Chassis_Odometry_t odom;
    assert(Chassis_Get_Odometry(&odom));
    assert(fabs(odom.x_m - x) < 0.00025 && fabs(odom.y_m - y) < 0.00025);
    assert(Chassis_Reset_Odometry());
    stopped_group();
    stopped_group();
    assert(Chassis_Distance_Start(direction,distance_m,limit_mps));
}

/** @brief 快慢跟随均按反馈到达；速度指令积分与固定运行时长不能替代距离。 */
static void test_distance_profiles(void)
{
    const float c = 0.7071067812f;
    const float units[8][2] = {{1,0},{-1,0},{0,1},{0,-1},{c,c},{c,-c},{-c,c},{-c,-c}};
    for (unsigned i = 0U; i < 8U; ++i)
    {
        check_distance_profile((Chassis_Translation_Direction_t)(i + 1U),
                               units[i][0],units[i][1],0.30f,0.50f,0.55f,137.0f);
        check_distance_profile((Chassis_Translation_Direction_t)(i + 1U),
                               units[i][0],units[i][1],0.30f,0.50f,0.80f,0.0f);
    }
    check_distance_profile(CHASSIS_TRANSLATION_FORWARD,1,0,0.50f,0.50f,1.0f,0.0f);
    check_distance_profile(CHASSIS_TRANSLATION_FORWARD_LEFT,c,c,1.91f,0.50f,0.80f,-179.0f);
    check_distance_profile(CHASSIS_TRANSLATION_LEFT,0,1,0.257f,0.21f,0.55f,0.0f);
    check_distance_profile(CHASSIS_TRANSLATION_FORWARD,1,0,CHASSIS_DISTANCE_MIN_M,0.50f,1.0f,0.0f);
    /* 末段接近下限不能抬高用户请求的更小速度。 */
    check_distance_profile(CHASSIS_TRANSLATION_FORWARD,1,0,0.015f,0.004f,1.0f,0.0f);
    check_distance_profile(CHASSIS_TRANSLATION_BACKWARD,-1,0,0.015f,0.006f,1.0f,0.0f);
}

/** @brief 低速摩擦死区及零目标后小回退不能把停车点卡在验收外沿。
 * @note 独立地面模型按目标减去 5 mm/s 死区、停车后短暂反向反馈，非实车精度证明。
 */
static void test_distance_deadband_and_rollback(void)
{
    for (int direction = -1; direction <= 1; direction += 2)
    {
        reset_case(0U,0U);
        assert(enable_ready());
        assert(Chassis_Distance_Start(direction > 0 ? CHASSIS_TRANSLATION_FORWARD :
                                     CHASSIS_TRANSLATION_BACKWARD,0.05f,0.05f));
        unsigned frame, zero_frames = 0U;
        uint8_t was_moving = 0U;
        double ground = 0.0;
        for (frame = 0U; frame < 500U; ++frame)
        {
            float body[3], feedback[3];
            body_target(body);
            const float requested = direction * body[0];
            assert(requested >= 0.0f && requested <= 0.05001f);
            float actual = direction * fmaxf(0.0f, requested - 0.005f);
            if (requested > 0.0f)
                was_moving = 1U;
            else if (was_moving)
            {
                if (zero_frames == 0U)
                {
                    Chassis_Odometry_t at_zero;
                    assert(Chassis_Get_Odometry(&at_zero));
                    assert(direction * at_zero.x_m >= 0.048); /* 至少留 1 mm 回退余量。 */
                }
                /* 模拟停车前两拍共 0.12 mm 的小回退，不把零目标当成真实静止。 */
                if (++zero_frames <= 2U)
                    actual = -direction * 0.006f;
            }
            ground += actual * 0.01;
            Motor_Chassis_Inverse(actual,0.0f,0.0f,feedback);
            for (unsigned i = 0U; i < 3U; ++i)
                feedback[i] = -feedback[i];
            feedback_step(10000U,feedback);
            assert(Chassis_Is_Enabled() && disables == 0U && aborts == 0U);
            if (Chassis_Is_Stopped())
                break;
        }
        assert(frame < 500U && zero_frames >= 3U);
        Chassis_Odometry_t end;
        assert(Chassis_Get_Odometry(&end));
        assert(fabs(0.05 - direction * end.x_m) < 0.002);
        assert(fabs(0.05 - direction * ground) <= CHASSIS_DISTANCE_TOLERANCE_M);
        assert(fabs(end.x_m - ground) < 0.00001);
        printf("DISTANCE deadband/rollback %+d: %.3f s, measured %.6f m, error %.3f mm\n",
               direction,(frame + 1U) * 0.01,direction * end.x_m,
               (0.05 - direction * end.x_m) * 1000.0);
    }
}

/** @brief 核对定距准入、变周期、抢占、复用和姿态故障停机。 */
static void test_distance_lifecycle(void)
{
    const Chassis_Translation_Direction_t forward = CHASSIS_TRANSLATION_FORWARD;
    const float invalid[] = {0.0f,-1.0f,INFINITY,-INFINITY,NAN};
    float body[3];
    reset_case(0U,0U);
    assert(!Chassis_Distance_Start(forward,0.5f,0.5f));
    assert(enable_ready());
    for (unsigned i = 0U; i < sizeof(invalid)/sizeof(invalid[0]); ++i)
    {
        assert(!Chassis_Distance_Start(forward,invalid[i],0.5f));
        assert(!Chassis_Distance_Start(forward,0.5f,invalid[i]));
    }
    assert(!Chassis_Distance_Start(forward,CHASSIS_DISTANCE_MIN_M * 0.5f,0.5f));
    assert(!Chassis_Distance_Start(forward,0.5f,0.501f));
    assert(!Chassis_Distance_Start(forward,1e30f,0.5f));
    assert(!Chassis_Distance_Start(forward,0.5f,1e-30f));
    assert(!Chassis_Distance_Start(CHASSIS_TRANSLATION_NONE,0.5f,0.5f));
    assert(!Chassis_Distance_Start((Chassis_Translation_Direction_t)9,0.5f,0.5f));
    sample_age = 21U;
    assert(!Chassis_Distance_Start(forward,0.5f,0.5f));
    sample_age = 0U;
    assert(Chassis_Distance_Start(forward,0.5f,0.5f));
    plant_step(10000U,1.0f,NULL,NULL); body_target(body);
    near(body[0],0.0075f);
    assert(!Chassis_Distance_Start(forward,0.3f,0.5f));
    yaw_deg = 2.0f;
    plant_step(37000U,1.0f,NULL,NULL); body_target(body); near(body[0],0.03525f);
    near(body[2],-0.0592f);
    Chassis_Stop();
    step(10000U,1U); body_target(body);
    near(body[0],0.0f); near(body[1],0.0f); near(body[2],0.0f);
    assert(disables == 0U && aborts == 0U);
    confirm_stopped();
    assert(Chassis_Distance_Start(forward,0.3f,0.3f));
    plant_step(10000U,1.0f,NULL,NULL); body_target(body); near(body[0],0.0075f);
    imu_ready = 0U;
    step(10000U,1U); assert(aborts == 0U && Chassis_Is_Enabled());
    for (unsigned i = 0U; i < 60U && Chassis_Is_Enabled(); ++i)
        step(10000U,1U);
    assert(disables == 3U && !Chassis_Is_Enabled() && !Chassis_Is_Stopped());
    imu_ready = 1U;
    assert(!Chassis_Distance_Start(forward,0.3f,0.3f));
}

/** @brief 零轮速、反向和横移均不能伪造前向到达，超时只能进入故障失能。 */
static void test_distance_no_progress(void)
{
    for (unsigned scenario = 0U; scenario < 3U; ++scenario)
    {
        float wheels[3] = {0.0f,0.0f,0.0f};
        reset_case(0U,0U);
        assert(enable_ready());
        assert(Chassis_Distance_Start(CHASSIS_TRANSLATION_FORWARD,0.03f,0.5f));
        if (scenario > 0U)
        {
            Motor_Chassis_Inverse(scenario == 1U ? -0.05f : 0.0f,
                                  scenario == 2U ? 0.05f : 0.0f,0.0f,wheels);
            for (unsigned i = 0U; i < 3U; ++i) wheels[i] = -wheels[i];
        }
        for (unsigned frame = 0U; frame < 230U; ++frame)
        {
            feedback_step(10000U,wheels);
            assert(!Chassis_Is_Stopped() && !Chassis_Motion_Ready());
        }
        assert(!Chassis_Is_Enabled() && disables == 3U);
    }
}

/** @brief 反馈断流、重复帧、非法数据和超长采样均不得继续定距或报告正常完成。 */
static void test_distance_feedback_faults(void)
{
    for (unsigned fault = 0U; fault < 6U; ++fault)
    {
        reset_case(0U,0U);
        assert(enable_ready());
        assert(Chassis_Distance_Start(CHASSIS_TRANSLATION_FORWARD,0.5f,0.5f));
        for (unsigned frame = 0U; frame < 5U; ++frame)
            plant_step(10000U,1.0f,NULL,NULL);
        Chassis_Odometry_t before, after;
        assert(Chassis_Get_Odometry(&before));
        after = before;
        if (fault == 5U)
            plant_step(60000U,1.0f,NULL,NULL);
        else
        {
            for (unsigned frame = 0U; frame < 8U; ++frame)
            {
                counter_us += 10000U; test_tick += 10U;
                for (unsigned i = 0U; i < 3U; ++i)
                {
                    if (fault == 0U && i == 2U) continue; /* 单轮断流。 */
                    if (fault == 1U) { dm_motor[i].feedback_time_ms = test_tick; continue; }
                    wheel_feedback(i,motor_velocity[i],1U);
                }
                if (fault == 2U) dm_motor[1].velocity = NAN;
                if (fault == 3U) dm_motor[1].state = 0U;
                if (fault == 4U) dm_motor[1].id = 0U;
                step(0U,1U);
                assert(!Chassis_Is_Stopped());
            }
        }
        if (fault <= 1U)
        {
            (void)Chassis_Get_Odometry(&after);
            assert(fabs(after.x_m - before.x_m) < 1e-9);
            assert(fabs(after.y_m - before.y_m) < 1e-9);
        }
        for (unsigned frame = 0U; frame < 60U && Chassis_Is_Enabled(); ++frame)
            stopped_group();
        assert(!Chassis_Is_Enabled() && !Chassis_Is_Stopped() && disables == 3U);
    }
}

/** @brief IMU读取跨毫秒时收到的新CAN帧不得被当作超龄一整圈的旧反馈。 */
static void test_distance_feedback_tick_race(void)
{
    reset_case(0U,UINT32_MAX - 100U);
    assert(enable_ready());
    assert(Chassis_Distance_Start(CHASSIS_TRANSLATION_FORWARD,0.3f,0.5f));
    read_injects_feedback = 1U;
    step(10000U,1U);
    Chassis_Odometry_t odom;
    assert(Chassis_Get_Odometry(&odom));
    assert(Chassis_Is_Enabled() && !Chassis_Is_Stopped());
    float body[3]; body_target(body);
    assert(body[0] > 0.0f && disables == 0U);
    Chassis_Stop();
    step(10000U,1U);
    confirm_stopped();
}

/** @brief 定距减速后的短暂回退可在容差内接续，大幅滑行仍触发保护。 */
static void test_distance_coast_after_zero(uint8_t small_rollback)
{
    reset_case(0U,0U);
    assert(enable_ready());
    assert(Chassis_Distance_Start(CHASSIS_TRANSLATION_FORWARD,0.05f,0.5f));
    float body[3];
    unsigned frame;
    for (frame = 0U; frame < 1000U; ++frame)
    {
        plant_step(10000U,1.0f,NULL,NULL);
        body_target(body);
        if (frame > 0U && body[0] == 0.0f)
            break;
    }
    assert(frame < 1000U && !Chassis_Is_Stopped());
    float feedback[3];
    Motor_Chassis_Inverse(small_rollback ? -0.2f : 0.4f, 0.0f, 0.0f, feedback);
    for (unsigned i = 0U; i < 3U; ++i) feedback[i] = -feedback[i];
    if (small_rollback)
    {
        for (unsigned i = 0U; i < 3U; ++i)
            feedback_step(10000U, feedback);
        confirm_stopped();
        assert(Chassis_Is_Enabled() && disables == 0U);
        assert(Chassis_Turn_Start(1, 90.0f));
        return;
    }
    for (unsigned i = 0U; i < 8U; ++i)
    {
        feedback_step(10000U,feedback);
        assert(!Chassis_Is_Stopped());
    }
    for (unsigned i = 0U; i < 10U; ++i)
    {
        stopped_group();
        assert(!Chassis_Is_Stopped());
    }
    assert(!Chassis_Is_Enabled() && disables == 3U);
}

/** @brief 有持续小进展也不能无限延长动作，必须遵守独立的定距总期限。 */
static void test_distance_total_timeout(void)
{
    reset_case(0U,0U);
    assert(enable_ready());
    assert(Chassis_Distance_Start(CHASSIS_TRANSLATION_FORWARD,0.03f,0.5f));
    float feedback[3];
    Motor_Chassis_Inverse(0.004f,0.0f,0.0f,feedback);
    for (unsigned i = 0U; i < 3U; ++i) feedback[i] = -feedback[i];
    for (unsigned frame = 0U; frame < 400U; ++frame)
    {
        feedback_step(10000U,feedback);
        assert(!Chassis_Is_Stopped());
        /* 每0.5s有2mm进展，已经超过1.5s堵转期限但仍应继续。 */
        if (frame == 249U) assert(Chassis_Is_Enabled());
    }
    assert(!Chassis_Is_Enabled() && disables == 3U);
}

/** @brief 同一个到位样本可直接接续。要求实测静止的调用者仍受 500 ms 期限保护。 */
static void test_distance_stop_deadline(uint8_t accept_motion)
{
    reset_case(0U,0U);
    assert(enable_ready());
    assert(Chassis_Distance_Start(CHASSIS_TRANSLATION_FORWARD,0.05f,0.5f));
    float body[3];
    unsigned frame;
    for (frame = 0U; frame < 1000U; ++frame)
    {
        plant_step(10000U,1.0f,NULL,NULL);
        body_target(body);
        if (frame > 0U && body[0] == 0.0f) break;
    }
    assert(frame < 1000U && !Chassis_Is_Stopped());
    Chassis_Odometry_t at_zero, rotating;
    assert(Chassis_Get_Odometry(&at_zero));
    const float feedback[3] = {1.0f,1.0f,1.0f};
    if (accept_motion)
    {
        feedback_step(10000U, feedback);
        assert(!Chassis_Is_Stopped() && Chassis_Motion_Ready());
        assert(Chassis_Turn_Start(1, 90.0f));
        return;
    }
    for (frame = 0U; frame < 70U; ++frame)
    {
        feedback_step(10000U,feedback);
        assert(!Chassis_Is_Stopped());
    }
    assert(Chassis_Get_Odometry(&rotating));
    assert(fabs(rotating.x_m - at_zero.x_m) < 0.0001);
    assert(fabs(rotating.y_m - at_zero.y_m) < 0.0001);
    assert(!Chassis_Is_Enabled() && disables == 3U);
}

/** @brief CAN拥塞不得推迟里程越界故障判断，释放邮箱后只能发失能。 */
static void test_distance_overshoot_busy_can(void)
{
    reset_case(0U,0U);
    assert(enable_ready());
    assert(Chassis_Distance_Start(CHASSIS_TRANSLATION_FORWARD,0.03f,0.5f));
    plant_step(10000U,1.0f,NULL,NULL);
    const uint32_t before = controls;
    float feedback[3];
    Motor_Chassis_Inverse(0.4f,0.0f,0.0f,feedback);
    for (unsigned i = 0U; i < 3U; ++i) feedback[i] = -feedback[i];
    free_mailboxes = 2U;
    for (unsigned frame = 0U; frame < 75U; ++frame)
    {
        counter_us += 10000U; test_tick += 10U;
        for (unsigned i = 0U; i < 3U; ++i)
        {
            dm_motor[i].position = remainderf(dm_motor[i].position + feedback[i] * .01f,25.f);
            wheel_feedback(i,feedback[i],1U);
        }
        step(0U,0U);
        assert(controls == before && !Chassis_Is_Stopped());
    }
    /* 远早于3.2s总期限；总线仍忙，软件已失能，但FD尚未提交。 */
    assert(!Chassis_Is_Enabled() && disables == 0U);
    step(10000U,1U);
    assert(disables == 3U && controls == before);
}

static void test_hold_tx_timeout(void)
{
    reset_case(0U, UINT32_MAX - 130U);
    assert(enable_ready() && Chassis_Set_Velocity(.4f,0,0));
    for (unsigned i = 0; i < 100; ++i) plant_step(10000U,1.0f,NULL,NULL);
    Chassis_Hold();
    const uint32_t before = controls;
    /* Fresh feedback is independent of the three occupied CAN TX mailboxes. */
    for (unsigned elapsed = 0; elapsed < CHASSIS_CAN_TX_TIMEOUT_MS; elapsed += 10)
    {
        for (unsigned i = 0; i < 3; ++i) wheel_feedback(i, motor_velocity[i], 1U);
        step(10000U,0U);
        assert(controls == before && Chassis_Is_Enabled() && !Chassis_Motion_Ready());
    }
    for (unsigned i = 0; i < 3; ++i) wheel_feedback(i, motor_velocity[i], 1U);
    step(10000U,0U);
    assert(!Chassis_Is_Enabled() && aborts == 1U && disables == 0U);
    step(10000U,1U);
    assert(controls == before && disables == 3U);
}

/** @brief 连续三轴用同一缩放率，既限制线加速度也限制角加速度，更新目标不清斜坡。 */
static void test_vector(void)
{
    float body[3];
    reset_case(0U,0U);
    assert(enable_ready());
    assert(Chassis_Set_Velocity(0.24f,0.32f,0.8f));
    step(10000U,1U);
    body_target(body); near(body[0],0.00054f); near(body[1],0.00072f); near(body[2],0.0018f);
    /* 连续速度入口按已确认的后/右前/左前轮布局分解三个分量。 */
    near(motor_velocity[0], 0.07760571f * .2f);
    near(motor_velocity[1], -0.14348767f * .2f);
    near(motor_velocity[2], -0.00987233f * .2f);
    step(15000U,1U);
    body_target(body);
    near(body[0] / body[1],.75f);
    assert(body[0] > .00054f && body[0] < .00675f && body[2] > .0018f && body[2] < .0225f);
    assert(Chassis_Set_Velocity(0.024f,0.032f,-0.8f));
    step(10000U,1U);
    float changed[3]; body_target(changed);
    assert(fabsf(changed[2]-body[2]) <= .01601f);
    assert(hypotf(changed[0]-body[0],changed[1]-body[1]) <= 0.00451f);
    assert(changed[0] > body[0]); /* 更新目标未从零重新爬升。 */
    for (unsigned i = 0; i < 100; ++i) step(10000U, 1U);
    body_target(body); near(body[2], -.8f);
    assert(!Chassis_Set_Velocity(0.4f,0.4f,0.0f));
    assert(!Chassis_Set_Velocity(NAN,0.0f,0.0f));
    assert(Chassis_Set_Velocity(0.0f,0.0f,0.0f));
    step(10000U,1U);
    body_target(body); near(body[0],0); near(body[1],0); near(body[2],0);
    assert(disables == 0U);
}

/* Use the issued motor commands to check ordinary release, retargeting during
 * braking and the zero-frame handoff. Feed fresh encoder frames independently. */
static void test_smooth_hold_and_redirect(float speed, float yaw_rate)
{
    float body[3], prior[3];
    reset_case(0U,0U);
    assert(Chassis_Attitude_Ready());
    sample_age = 21U; assert(!Chassis_Attitude_Ready()); sample_age = 0U;
    assert(enable_ready() && Chassis_Set_Velocity_Explicit_Yaw(speed,0,yaw_rate));
    for (unsigned i=0; i<320U; ++i)
        plant_step(10000U,1.0f,NULL,NULL);
    body_target(body); near(body[0],speed); near(body[2],yaw_rate);

    Chassis_Hold();
    for (unsigned i=0; i<15U; ++i)
    {
        memcpy(prior,body,sizeof(body));
        Chassis_Hold();
        assert(!Chassis_Motion_Ready());
        plant_step(10000U,1.0f,NULL,NULL);
        body_target(body);
        assert(body[0] > 0 && body[0] < prior[0] && prior[0]-body[0] <= .00751f);
        assert(body[2] > 0 && body[2] < prior[2] && prior[2]-body[2] <= .04001f);
    }
    /* A fresh diagonal reverse command can take over the deceleration immediately.
     * Neither a zero frame nor a physical stop is needed in between. */
    assert(Chassis_Set_Velocity_Explicit_Yaw(-0.8f * speed,0.6f * speed,-yaw_rate));
    for (unsigned i=0; i<550U; ++i)
    {
        memcpy(prior,body,sizeof(body));
        plant_step(10000U,1.0f,NULL,NULL);
        body_target(body);
        assert(hypotf(body[0]-prior[0],body[1]-prior[1]) <= .00451f);
        assert(fabsf(body[2]-prior[2]) <= .01601f);
        if (i==0U) assert(body[0] > .2f && body[1] > 0 && body[2] > 1.5f);
    }
    near(body[0],-0.8f * speed); near(body[1],0.6f * speed); near(body[2],-yaw_rate);
    Chassis_Hold();
    unsigned ticks = 0;
    do
    {
        Chassis_Hold();
        plant_step(10000U,1.0f,NULL,NULL);
        assert(++ticks < 150U);
    } while (!Chassis_Motion_Ready());
    for (unsigned i=0; i<3; ++i) near(motor_velocity[i],0.0f);
    assert(Chassis_Is_Enabled() && enables == 3U && disables == 0U);
    printf("HOLD mixed %.2f m/s + %.1f rad/s: stable zero after %.3f s, no intermediate disable\n",
           speed, yaw_rate, ticks * .010);
    assert(Chassis_Set_Velocity(.4f,0,0));
    plant_step(10000U,1.0f,NULL,NULL);
    const float wheel_first = fabsf(motor_velocity[1]);
    const float unfiltered_step = .0045f * .8660254f / .035f;
    near(wheel_first,unfiltered_step*.2f);
    printf("VECTOR first wheel step %.5f -> %.5f rad/s for a .40 m/s request\n",
           unfiltered_step,wheel_first);
    for (unsigned i=0; i<100; ++i) plant_step(10000U,1.0f,NULL,NULL);
    Chassis_Hold();
    /* A non-ready drive while ramping down forces immediate hard brake; a new
     * velocity is rejected even before the next control tick. */
    dm_motor[2].state=8U;
    assert(!Chassis_Set_Velocity(.2f,0,0));
    step(10000U,1U);
    for (unsigned i=0; i<3; ++i) near(motor_velocity[i],0.0f);
    assert(!Chassis_Motion_Ready());
    for (unsigned i=0; i<52; ++i) step(10000U,1U);
    assert(!Chassis_Is_Enabled() && disables==3U);
}

/** @brief 待命中的新动作沿用三轮发帧节拍，首拍只累计接单后的时间。 */
static void test_yaw_rate_feedback(void)
{
    for (int direction = -1; direction <= 1; direction += 2)
    {
        reset_case(0U, 0U);
        assert(enable_ready() && Chassis_Set_Velocity(0.0f, 0.0f, direction * 0.40f));
        for (unsigned tick = 0; tick < 100U; ++tick)
            plant_step(10000U, 1.0f, NULL, NULL);
        float body[3];
        body_target(body);
        near(body[2], direction * 0.40f);

        /* 模拟实测角速度超过指令，不允许推高输出、反转或突破角加速度。 */
        yaw_rate = direction * 0.65f;
        for (unsigned tick = 0; tick < 100U; ++tick)
        {
            const float previous = body[2];
            plant_step(10000U, 1.0f, NULL, NULL);
            body_target(body);
            assert(direction * body[2] >= 0.0f && fabsf(body[2]) <= 0.40001f);
            assert(fabsf(body[2] - previous) <= 0.01601f);
        }
        near(body[2], direction * 0.24f);

        Chassis_Hold();
        for (unsigned tick = 0; tick < 85U; ++tick)
        {
            const float previous = body[2];
            plant_step(10000U, 1.0f, NULL, NULL);
            body_target(body);
            assert(direction * body[2] >= 0.0f && fabsf(body[2] - previous) <= 0.04001f);
        }
        near(body[2], 0.0f);
        assert(Chassis_Motion_Ready() && disables == 0U);
    }
}

static void test_world_velocity_after_turn(void)
{
    reset_case(0U, 0U);
    tuning_test = 1U;
    const float kd[3] = {0.2f, 0.25f, 0.2f}, no_friction[3] = {0.0f};
    assert(Chassis_Set_Tuning(kd, 1.0f, no_friction));
    assert(enable_ready() && Chassis_Set_Velocity(0.0f, 0.0f, 1.0f));

    float body[3];
    for (unsigned tick = 0; tick < 60; ++tick)
    {
        body_target(body);
        yaw_rate = body[2];
        yaw_deg += yaw_rate * 0.01f * (180.0f / 3.14159265f);
        plant_step(10000U, 1.0f, NULL, NULL);
    }
    Chassis_Hold();
    assert(!Chassis_Motion_Ready() && Chassis_Can_Release());
    assert(Chassis_Set_World_Velocity(-0.20f, 0.12f, 1.2f));

    for (unsigned tick = 0; tick < 210; ++tick)
    {
        body_target(body);
        yaw_rate = body[2];
        yaw_deg = remainderf(yaw_deg + yaw_rate * 0.01f * (180.0f / 3.14159265f), 360.0f);
        plant_step(10000U, 1.0f, NULL, NULL);
        if (tick >= 140U)
        {
            Chassis_Odometry_t pose;
            assert(Chassis_Get_Odometry(&pose));
            const float cosine = cosf(pose.yaw_rad), sine = sinf(pose.yaw_rad);
            body_target(body);
            near(cosine * body[0] - sine * body[1], -0.20f);
            near(sine * body[0] + cosine * body[1], 0.12f);
            near(body[2], 1.2f);
            for (unsigned wheel = 0; wheel < 3; ++wheel)
                near(chassis_motor_ff[wheel], 0.0f);
        }
    }

    /* 返航失去有效里程计仍进入保护。 */
    imu_ready = 0U;
    for (unsigned tick = 0; tick < 65U && Chassis_Is_Enabled(); ++tick)
        plant_step(10000U, 1.0f, NULL, NULL);
    assert(!Chassis_Is_Enabled() && disables == 3U);
}

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
            CHASSIS_DISTANCE_ACCELERATION_MPS2 * dt : CHASSIS_STRAIGHT_ACCELERATION_MPS2 * dt;
        near(body[0], action == 0U ? vx * dt / (CHASSIS_VECTOR_SMOOTHING_S + dt) : vx);
        near(body[1], 0.0f);
        near(body[2], action == 2U ? CHASSIS_TURN_ACCEL_RAD_S2 * dt : 0.0f);
        step(10000U, 1U);
        assert(controls == 6U && aborts == 0U);
        if (action == 1U)
        {
            /* 同刻接单的零斜坡拍不占定时行驶时长。 */
            body_target(body);
            if (first_us == 0U) near(body[0], 0.0045f);
            else assert(body[0] >= 0 && body[0] < vx);
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
        (CHASSIS_STRAIGHT_ACCELERATION_MPS2 + CHASSIS_ROTATION_RADIUS * CHASSIS_VECTOR_ACCEL_RAD_S2) /
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
    for (unsigned i = 0U; i < 60U; ++i)
        step(10000U, 1U);
    body_target(body); near(body[1], 0.09f); near(body[2], 0.0f);

    yaw_deg = -179.0f;
    yaw_rate = 0.4f;
    assert(Chassis_Set_Velocity(0.0f, 0.08f, 0.0f));
    step(10000U, 1U);
    body_target(body); near(body[2], -0.0032f);
    for (unsigned i = 0U; i < 60U; ++i)
    {
        assert(Chassis_Set_Velocity(0.0f, 0.08f, 0.0f));
        step(10000U, 1U);
    }
    body_target(body); near(body[1], 0.08f);
    near(body[2], -0.16981317f); /* 保留 179° 目标，按 -2° 和实测角速度修偏。 */

    yaw_rate = 0.0f;
    assert(Chassis_Set_Velocity(0.0f, 0.0f, 0.20f));
    step(10000U, 1U);
    body_target(body); near(body[2], -0.16661317f);
    for (unsigned i = 0U; i < 75U; ++i)
        step(10000U, 1U);
    body_target(body); near(body[1], 0.0f); near(body[2], 0.20f);

    yaw_deg = -162.0f;
    yaw_rate = 0.0f;
    assert(Chassis_Set_Velocity(0.0f, 0.08f, 0.0f));
    step(10000U, 1U);
    body_target(body); near(body[2], 0.19775f);
    for (unsigned i = 0U; i < 75U; ++i)
        step(10000U, 1U);
    body_target(body); near(body[1], 0.08f); near(body[2], 0.0f);
    yaw_deg = -160.0f;
    step(10000U, 1U);
    body_target(body); near(body[2], -0.0032f);

    assert(Chassis_Set_Velocity(0.0f, 0.0f, 0.0f));
    step(10000U, 1U);
    body_target(body); near(body[0], 0.0f); near(body[1], 0.0f); near(body[2], 0.0f);
    confirm_stopped();
    yaw_deg = 10.0f;
    assert(Chassis_Set_Velocity(0.08f, 0.0f, 0.0f));
    step(10000U, 1U);
    body_target(body); near(body[0], 0.0009f); near(body[2], 0.0f);
}

/** @brief 显式零角目标不调用锁航向PID，切回旧接口仍按当前姿态重新锁定。 */
static void test_vector_explicit_yaw(void)
{
    float body[3];
    reset_case(0U,0U);
    assert(enable_ready());
    assert(!Chassis_Set_Velocity_Explicit_Yaw(.1f,0,4.001f));
    assert(Chassis_Set_Velocity_Explicit_Yaw(.1f,0,.5f));
    for (unsigned i = 0U; i < 60U; ++i)
    {
        yaw_deg = i * .5f;
        yaw_rate = 1.0f;
        step(10000U,1U);
        body_target(body);
        assert(body[2] >= 0.0f && body[2] <= .50001f);
    }
    near(body[2],.3f); /* 实测 1 rad/s 触发最大 40% 降幅。 */
    assert(Chassis_Set_Velocity_Explicit_Yaw(.1f,0,0));
    for (unsigned i = 0U; i < 60U; ++i)
    {
        yaw_deg = 40.0f + i * .5f;
        step(10000U,1U);
        body_target(body);
        assert(body[2] >= -.00001f && body[2] <= .50001f);
    }
    near(body[2],0); /* 大航向误差及角速反馈也不能使隐式PID越过目标角限幅。 */
    yaw_rate = 0;
    assert(Chassis_Set_Velocity(.1f,0,0));
    step(10000U,1U);
    body_target(body); near(body[2],0); /* 切回原API重新锁当前姿态。 */
    yaw_deg += 2.0f;
    step(10000U,1U);
    body_target(body); near(body[2],-.0032f);
    assert(Chassis_Set_Velocity_Explicit_Yaw(.1f,0,0));
    step(10000U,1U);
    body_target(body); assert(fabsf(body[2]) < .0032f);
    imu_ready = 0U;
    assert(Chassis_Set_Velocity_Explicit_Yaw(0,0,0));
    step(10000U,1U);
    body_target(body); near(body[0],0); near(body[1],0); near(body[2],0);
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

/** @brief 拒绝非法角度；双向180/360度跨±180，实测到角后可直接接续，15s超时仍失能。 */
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
            const float offset = sample == half_turns * 2U ? 4.0f : 0.0f;
            yaw_deg = remainderf(direction*(170.0f+90.0f*sample-offset),360.0f);
            yaw_rate = direction*0.2f;
            step(10000U,1U);
        }
        yaw_deg = remainderf(direction*(170.0f+angle_deg),360.0f);
        yaw_rate = direction*0.15f;
        for (unsigned i = 0; i < 8U; ++i)
        {
            const float moving[3] = {0.5f, 0.5f, 0.5f};
            feedback_step(10000U, moving);
        }
        body_target(body); near(body[2],0);
        assert(!Chassis_Is_Stopped() && Chassis_Motion_Ready());
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
        for (unsigned i = 0U; i < 80U; ++i)
        {
            step(10000U,1U);
            body_target(body);
            assert(fabsf(body[2]) <= 3.00001f);
            assert(fabsf(body[2] - previous) <= 0.04001f);
            previous = body[2];
        }
        near(body[2], 3.0f * direction);

        /* 瞬间到角、实际角速度已低时，较大的待发目标仍须减速。 */
        yaw_deg = direction * 180.0f;
        yaw_rate = 0.0f;
        for (unsigned i = 0U; i < 21U; ++i)
        {
            step(10000U,1U);
            body_target(body);
            near(body[2], direction * (3.0f - 0.040f * (i + 1U)));
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
        body_target(body); near(body[2], direction * 2.0f);

        /* 6° 制动角对应 0.915291 rad/s，原 P 输出仅 0.209440 rad/s。 */
        yaw_deg = direction * 174.0f;
        step(10000U,1U); body_target(body); near(body[2], direction * 1.96f);
        previous = body[2];
        for (unsigned i = 0U; i < 35U; ++i)
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

        yaw_deg = direction * 179.5f;
        plant_step(10000U, 1.0f, NULL, NULL);
        yaw_deg = direction * 177.5f;
        for (unsigned i = 0U; i < 4U; ++i)
            plant_step(10000U, 1.0f, NULL, NULL);
        body_target(body);
        near(body[2], 0.0f);
        assert(Chassis_Motion_Ready() && disables == 0U);
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
            assert(fabsf(body[2]) <= 3.00001f);
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
            near(peak,3.0f);
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
        for (unsigned i = 0U; i < 20U; ++i) plant_step(10000U,1.0f,NULL,NULL);
        body_target(body); near(body[2], direction * 0.8f);
        for (unsigned i = 1U; i < half_turns * 2U; ++i)
        {
            yaw_deg = remainderf(start_yaw + direction * 90.0f * i, 360.0f);
            yaw_rate = direction * 0.2f;
            plant_step(10000U,1.0f,NULL,NULL);
        }
        body_target(body); previous = body[2];
        for (unsigned i = 0U; i < 3U; ++i)
            wheel_feedback(i, motor_velocity[i], 1U); /* 反馈仍在转动，不能伪装停车。 */
        assert(!Chassis_Is_Stopped());
        uint32_t before = controls;
        plant_step(5000U,1.0f,NULL,NULL); assert(controls == before);

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
        plant_step(5000U,1.0f,NULL,NULL); /* 仍在原来 10 ms 时点发送，接受请求不重置控制时基。 */
        assert(controls == before + 3U);
        body_target(body); near(body[0],0.0075f); near(body[1],0.0f);
        near(body[2],previous - direction * 0.04f);
        previous = body[2];

        for (unsigned i = 0U; i < 40U; ++i)
        {
            uint32_t interval = intervals_us[i % 3U];
            plant_step(interval,1.0f,NULL,NULL); body_target(body);
            assert(fabsf(body[2] - previous) <= interval * 0.000004f + 0.00001f);
            previous = body[2];
        }
        /* 保留转向终点；若误锁切换时的 yaw，这里只剩 -Kd*rate。 */
        const float expected = remaining * 0.03490658504f - direction * 0.05f;
        near(body[2],expected);
        yaw_rate = -direction * 0.6f;
        plant_step(10000U,1.0f,NULL,NULL); body_target(body);
        near(body[2],expected + direction * 0.016f); /* 退出转向衔接后，直行 PID 也限角加速度。 */
        assert(disables == 0U && aborts == 0U);

        Chassis_Stop(); plant_step(10000U,1.0f,NULL,NULL); body_target(body);
        near(body[0],0.0f); near(body[2],0.0f);
        confirm_stopped();
        yaw_rate = 0.0f;
        assert(Chassis_Distance_Start(CHASSIS_TRANSLATION_FORWARD,0.5f,0.5f));
        yaw_deg = remainderf(yaw_deg + 2.0f,360.0f);
        plant_step(10000U,1.0f,NULL,NULL); body_target(body);
        near(body[2],-0.016f);
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
        plant_step(10000U,1.0f,NULL,NULL);
        yaw_deg = -direction * 180.0f; /* 包角表示刻意选与实际旋向相反的半圈符号。 */
        yaw_rate = direction * (moving ? 0.2f : 0.0f);
        assert(!Chassis_Is_Stopped());
        assert(Chassis_Distance_Start(CHASSIS_TRANSLATION_LEFT,0.5f,0.5f));
        for (unsigned i = 0U; i < 5U; ++i) plant_step(10000U,1.0f,NULL,NULL);
        body_target(body); near(body[0],0.0f); assert(body[1] > 0.0f);
        near(body[2],moving ? -direction * 0.05f : 0.0f);
    }
}

/** @brief 小于可测范围的请求不抢占转向，最短有效定距按反馈完成并清除衔接。 */
static void test_turn_handoff_short_distance(void)
{
    float body[3];
    reset_case(0U,0U);
    assert(enable_ready() && Chassis_Turn_Start(1,180.0f));
    for (unsigned i = 0U; i < 20U; ++i) plant_step(10000U,1.0f,NULL,NULL);
    yaw_deg = 177.0f;
    assert(!Chassis_Distance_Start(CHASSIS_TRANSLATION_FORWARD,0.000004f,0.5f));
    assert(Chassis_Distance_Start(CHASSIS_TRANSLATION_FORWARD,CHASSIS_DISTANCE_MIN_M,0.5f));
    plant_step(10000U,1.0f,NULL,NULL); body_target(body);
    assert(body[0] > 0.0f); near(body[2],0.76f);
    assert(!Chassis_Is_Stopped());
    unsigned frame;
    for (frame = 0U; frame < 500U && !Chassis_Is_Stopped(); ++frame)
        plant_step(10000U,1.0f,NULL,NULL);
    assert(frame < 500U && Chassis_Is_Stopped());
    body_target(body); near(body[0],0.0f); near(body[2],0.0f);
    assert(Chassis_Straight_Start(0.1f,0U));
    yaw_deg = 179.0f;
    step(10000U,1U); body_target(body);
    near(body[2],-0.016f);
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
        double ground_x = 0.0, ground_y = 0.0;
        uint8_t handed_off = 0U, translated_while_turning = 0U;
        unsigned handoff_frame = 0U, frame;
        reset_case(UINT32_MAX - 120000U,UINT32_MAX - 120U);
        yaw_deg = direction * 170.0f;
        assert(enable_ready() && Chassis_Turn_Start((int8_t)direction,180.0f));
        for (frame = 0U; frame < 900U; ++frame)
        {
            yaw_rate = body[2];
            travelled_deg += yaw_rate * 0.01f * 57.295779513f;
            const double midpoint = (direction * 170.0f + travelled_deg -
                yaw_rate * 0.005f * 57.295779513f) * 0.017453292519943295;
            ground_x += (body[0] * cos(midpoint) - body[1] * sin(midpoint)) * 0.01;
            ground_y += (body[0] * sin(midpoint) + body[1] * cos(midpoint)) * 0.01;
            yaw_deg = remainderf(direction * 170.0f + travelled_deg,360.0f);
            plant_step(10000U,1.0f,NULL,NULL); body_target(body);
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
        const double target_angle = direction * 350.0 * 0.017453292519943295;
        const double travelled_m = ground_x * cos(target_angle) + ground_y * sin(target_angle);
        assert(fabs(travelled_m - 1.0) <= CHASSIS_DISTANCE_TOLERANCE_M + 0.0005);
        assert(fabsf(travelled_deg - direction * 180.0f) < 2.0f);
        assert(disables == 0U && aborts == 0U && enables == 3U);
        printf("TURN+DISTANCE ideal %+d deg: handoff %.2f s, total %.2f s, final error %.3f deg\n",
               direction * 180,(handoff_frame + 1U) * 0.01,(frame + 1U) * 0.01,
               travelled_deg - direction * 180.0f);
    }
}

/** @brief 故障制动的静止反馈仅用于失能，不得对调用方报告正常动作完成。 */
static void test_fault_stop_is_not_completion(void)
{
    for (unsigned fault = 0U; fault < 2U; ++fault)
    {
        reset_case(0U, UINT32_MAX - 100U);
        assert(enable_ready());
        assert(Chassis_Distance_Start(CHASSIS_TRANSLATION_FORWARD, 0.4f, 0.5f));
        step(10000U, 1U);
        if (fault)
            sample_age = 21U;
        else
            imu_ready = 0U;
        step(10000U, 1U);
        assert(Chassis_Is_Enabled() && !Chassis_Is_Stopped());
        stopped_group();
        stopped_group();
        /* 第三组反馈模拟在底盘 Loop 与自主层之间到达。 */
        test_tick += 5U;
        counter_us += 5000U;
        for (unsigned i = 0U; i < 3U; ++i)
            wheel_feedback(i, 0.0f, 1U);
        assert(Chassis_Is_Enabled() && !Chassis_Is_Stopped());
        assert(!Chassis_Distance_Start(CHASSIS_TRANSLATION_FORWARD, 0.1f, 0.5f));
        step(5000U, 1U);
        step(10000U, 1U);
        assert(!Chassis_Is_Enabled() && disables == 3U); /* 按停稳失能，不额外等 500 ms。 */
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
    step(10000U,1U); assert(aborts == 0U && controls == 6U && Chassis_Is_Enabled());
    for (unsigned i = 0U; i < 60U && Chassis_Is_Enabled(); ++i)
        step(10000U,1U);
    assert(disables == 3U && !Chassis_Is_Enabled());
    imu_ready = 1U; sample_age = 0U;
    step(10000U,1U);
    assert(!Chassis_Is_Enabled());
    assert(!Chassis_Straight_Start(0.4f, 0U)); /* 恢复不自动重启。 */

    reset_case(0U,0U);
    assert(enable_ready() && Chassis_Straight_Start(0.4f,0U));
    fail_control_id = 2U;
    step(10000U,1U); assert(controls == 1U && aborts == 1U);
    step(10000U,0U); assert(disables == 0U && controls == 1U);
    step(10000U,1U); assert(disables == 3U);
}

/** @brief 1.8 kg 惯性前馈方向、限幅及停机清零；非法调参不部分生效。 */
static void test_live_tuning(void)
{
    const float kd[3] = {0.15f, 0.18f, 0.15f};
    const float zero[3] = {0.0f, 0.0f, 0.0f};
    const float friction[3] = {0.03f, 0.03f, 0.03f};
    for (int direction = -1; direction <= 1; direction += 2)
    {
        reset_case(0U, 0U);
        tuning_test = 1U;
        assert(!Chassis_Set_Tuning(NULL, 1.0f, zero));
        assert(!Chassis_Set_Tuning(kd, NAN, zero));
        assert(!Chassis_Set_Tuning(kd, 1.1f, zero));
        const float invalid_kd[3] = {0.15f, 0.18f, NAN};
        assert(!Chassis_Set_Tuning(invalid_kd, 1.0f, zero));
        near(chassis_tuning_kd[0], 0.2f);
        assert(Chassis_Set_Tuning(kd, 1.0f, zero));
        assert(enable_ready());
        for (unsigned i = 0; i < 3; ++i)
        {
            near(captured_torque[i], 0.0f);
            near(captured_kd[i], kd[i]);
        }
        assert(Chassis_Set_Velocity(direction * 0.25f, 0.0f, 0.0f));
        assert(!Chassis_Set_Tuning(kd, 0.0f, zero));
        step(direction > 0 ? 25000U : 10000U, 1U);
        near(captured_torque[0], 0.0f);
        const float first_dt = direction > 0 ? .025f : .010f;
        near(captured_torque[1], -direction * 1.8f * 0.45f * 0.035f / 1.7320508f *
                                 first_dt / (.040f + first_dt));
        near(captured_torque[2], -captured_torque[1]);
        /* 100 ms 空白不会形成补偿尖峰。 */
        step(100000U, 1U);
        for (unsigned i = 0; i < 3; ++i) near(captured_torque[i], 0.0f);
        Chassis_Stop();
        step(10000U, 1U);
        for (unsigned i = 0; i < 3; ++i) near(captured_torque[i], 0.0f);
        confirm_stopped();
        assert(Chassis_Set_Tuning(kd, 1.0f, friction));
        assert(Chassis_Set_Velocity(direction * 0.25f, 0.0f, 0.0f));
        step(10000U, 1U);
        assert(captured_torque[1] * direction < -.00328f);
        assert(captured_torque[2] * direction > .00328f);
        Chassis_Stop();
        step(10000U, 1U);
        confirm_stopped();
        assert(Chassis_Set_Velocity(0.0f, 0.0f, direction * 0.5f));
        step(10000U, 1U);
        for (unsigned i = 0; i < 3; ++i)
            near(captured_torque[i], 0.0f); /* 本轮摩擦补偿仅标定平移，不用于纯转。 */
        /* 姿态失效的第一拍已经清零前馈，保留原速度阻尼制动。 */
        imu_ready = 0U;
        step(10000U, 1U);
        for (unsigned i = 0; i < 3; ++i) near(captured_torque[i], 0.0f);
        Chassis_Disable();
        for (unsigned i = 0; i < 3; ++i) near(chassis_motor_ff[i], 0.0f);
    }
    reset_case(0U, 0U);
}

/** @brief 正式启动采用实车前馈配置；使能、停车、纯旋转仍无前馈。 */
static void test_default_tuning_profile(void)
{
    reset_case(0U, 0U);
    Chassis_Init();
    tuning_test = 1U;
    const float expected_kd[3] = {0.22f, 0.27f, 0.22f};
    near(chassis_tuning_ff_gain, 1.0f);
    near(chassis_tuning_friction[0], 0.0f);
    near(chassis_tuning_friction[1], 0.015f);
    near(chassis_tuning_friction[2], 0.015f);
    assert(enable_ready());
    for (unsigned i = 0; i < 3U; ++i)
    {
        near(captured_kd[i], expected_kd[i]);
        near(captured_torque[i], 0.0f);
    }
    assert(Chassis_Set_Velocity(0.25f, 0.0f, 0.0f));
    step(10000U, 1U);
    assert(captured_torque[1] < -.00328f && captured_torque[2] > .00328f);
    Chassis_Stop();
    step(10000U, 1U);
    for (unsigned i = 0; i < 3U; ++i) near(captured_torque[i], 0.0f);
    confirm_stopped();
    assert(Chassis_Set_Velocity(0.0f, 0.0f, 0.5f));
    step(10000U, 1U);
    for (unsigned i = 0; i < 3U; ++i) near(captured_torque[i], 0.0f);
    reset_case(0U, 0U);
}

/** @brief 从实际轮位重建惯性前馈的合力和转矩，防止前馈与逆解共同镜像。 */
static void test_translation_feedforward_geometry(void)
{
    const float axes[][2] = {{1,0}, {0,1}, {-1,0}, {0,-1}, {1,1}, {-1,1}};
    const float positions[3][2] = {
        {-.108f,0}, {.054f,-.054f * sqrtf(3.0f)}, {.054f,.054f * sqrtf(3.0f)}
    };
    const float kd[3] = {.2f,.25f,.2f}, no_friction[3] = {0,0,0};
    for (unsigned axis = 0; axis < sizeof(axes) / sizeof(axes[0]); ++axis)
    {
        reset_case(0U, 0U);
        tuning_test = 1U;
        assert(Chassis_Set_Tuning(kd, 1.0f, no_friction) && enable_ready());
        const float norm = hypotf(axes[axis][0], axes[axis][1]);
        const float dx = axes[axis][0] / norm, dy = axes[axis][1] / norm;
        assert(Chassis_Set_Velocity_Explicit_Yaw(.25f * dx, .25f * dy, 0));
        step(10000U, 1U);
        float fx = 0, fy = 0, moment = 0;
        for (unsigned wheel = 0; wheel < 3U; ++wheel)
        {
            const float x = positions[wheel][0], y = positions[wheel][1];
            const float tangential_speed = (-y / .108f) * (.0009f * dx) +
                                           (x / .108f) * (.0009f * dy);
            near(motor_velocity[wheel], -tangential_speed / .035f);
            /* 电机方向为-1，原始MIT转矩须先转回几何切向力。 */
            const float force = -captured_torque[wheel] / .035f;
            const float wx = force * (-y / .108f), wy = force * (x / .108f);
            fx += wx;
            fy += wy;
            moment += x * wy - y * wx;
        }
        near(fx, 1.8f * .09f * dx);
        near(fy, 1.8f * .09f * dy);
        near(moment, 0);
    }
}

/** @brief 执行底盘真实源码回归。 @return 成功为0。 */
int main(void)
{
#if CHASSIS_TUNING_ENABLED
    reset_case(0U,0U);
    assert(enable_ready());
    assert(!Chassis_Set_Test_Rotation(4.01f));
    assert(!Chassis_Set_Test_Rotation(NAN));
    assert(Chassis_Set_Test_Rotation(4));
    for(unsigned i=0;i<320;++i)plant_step(10000U,1.0f,NULL,NULL);
    float spin_body[3];body_target(spin_body);
    near(spin_body[0],0);near(spin_body[1],0);near(spin_body[2],4);
#endif
    test_frame_spacing();
    test_stop_feedback();
    test_timed_brake();
    test_kinematics();
    test_timing_and_parameters();
    test_reenable_order();
    test_translation();
    test_distance_profiles();
    test_distance_deadband_and_rollback();
    test_distance_lifecycle();
    test_distance_no_progress();
    test_distance_feedback_faults();
    test_distance_feedback_tick_race();
    test_distance_coast_after_zero(1U);
    test_distance_coast_after_zero(0U);
    test_distance_total_timeout();
    test_distance_stop_deadline(0U);
    test_distance_stop_deadline(1U);
    test_distance_overshoot_busy_can();
    test_hold_tx_timeout();
    test_vector();
    test_smooth_hold_and_redirect(CHASSIS_RUN_SPEED_MPS, CHASSIS_RUN_YAW_RATE_RAD_S);
    test_smooth_hold_and_redirect(CHASSIS_STRAIGHT_MAX_SPEED_MPS, CHASSIS_VECTOR_MAX_WZ_RAD_S);
    test_yaw_rate_feedback();
    test_world_velocity_after_turn();
    test_motion_start_cadence();
    test_tracking_wheel_cadence();
    test_vector_yaw_hold();
    test_vector_explicit_yaw();
    test_zero_vector_stops_actions();
    test_turn();
    test_turn_ramp();
    test_turn_braking_and_fine();
    test_turn_closed_loop();
    test_turn_distance_handoff();
    test_turn_handoff_half_circle_delta();
    test_turn_handoff_short_distance();
    test_turn_handoff_closed_loop();
    test_fault_stop_is_not_completion();
    test_failure_paths();
    test_live_tuning();
    test_default_tuning_profile();
    test_translation_feedforward_geometry();
    puts("PASS chassis: kinematics, measured dt/wrap, 8-way/vector/turn, distance and fresh three-wheel stop");
    return 0;
}
