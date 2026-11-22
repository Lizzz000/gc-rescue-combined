#include "heading_eskf.h"
#include "heading_yaw_guard.h"
#include "chassis_odometry.h"
#include "motor_chassis.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

static const double tau = 6.28318530717958647692;
static void covariance(const Heading_ESKF_t *s)
{
    assert(s->initialized && isfinite(s->yaw_rad) && fabsf(s->bias_rad_s)<=.05f);
    assert(s->p_yaw>0 && s->p_bias>0);
    assert((double)s->p_yaw*s->p_bias >= (double)s->p_yaw_bias*s->p_yaw_bias);
}

static void test_yaw_priority_and_noise(void)
{
    for (unsigned period=5;period<=20;period+=5)
    {
        Heading_ESKF_t s;
        const double dt=period*.001;
        assert(Heading_ESKF_Init(&s,3.1f,2.02f));
        double truth=3.1, raw_square=0, filtered_square=0;
        for (unsigned i=1;i<=80000/period;++i)
        {
            truth+=2*dt;
            const float noise=.012f*sinf(i*2.17f)+.006f*cosf(i*1.31f);
            const float yaw=remainder(truth+noise,tau);
            /* 轮角低10%，且周期性丢脉冲/短时轮滑；不能把误差累计到航向。 */
            const float wheel=i%300<12 ? -4.0f*dt : 1.8f*dt;
            assert(Heading_ESKF_Update(&s,yaw,2.02f,wheel,dt));
            covariance(&s);
            assert(s.yaw_gain>s.wheel_bias_gain*20);
            if(i>1000/period)
            {
                const double error=remainder(s.yaw_rad-truth,tau);
                assert(fabs(error)<.012); /* <0.7度，轮累计已相差数圈。 */
                raw_square+=noise*noise; filtered_square+=error*error;
            }
        }
        assert(filtered_square<raw_square*.30);
        assert(fabs(s.bias_rad_s-.02f)<.015f);
    }
}

static void test_prediction_and_joseph(void)
{
    Heading_ESKF_t s;
    assert(Heading_ESKF_Init(&s,0,.1f));
    /* 无轮观测的首拍，与独立double线性KF协方差/均值计算核对。 */
    const double dt=.01, a=s.p_yaw+dt*dt*s.p_bias+.0004*dt+2.5e-7*dt*dt*dt/3;
    const double b=-dt*s.p_bias-.5*2.5e-7*dt*dt, c=s.p_bias+2.5e-7*dt;
    const double r=.00030461742, k0=a/(a+r), k1=b/(a+r);
    assert(Heading_ESKF_Update(&s,.002f,.1f,NAN,.01f));
    assert(fabs(s.yaw_rad-(.001+k0*.001))<1e-8);
    assert(fabs(s.bias_rad_s-k1*.001)<1e-9);
    assert(fabs(s.p_yaw-(a-a*k0))<1e-9);
    assert(fabs(s.p_yaw_bias-(b-k0*b))<1e-10);
    assert(fabs(s.p_bias-(c-k1*b))<1e-9);
    assert(s.wheel_bias_gain==0);
    for(unsigned i=0;i<20000;++i)
    {
        const float yaw=remainder((i+1)*.04,tau);
        assert(Heading_ESKF_Update(&s,yaw,4, i%7 ? .04f : NAN,.01f));
        covariance(&s);
    }
    Heading_ESKF_t normal, slip;
    assert(Heading_ESKF_Init(&normal,0,0));slip=normal;
    assert(Heading_ESKF_Update(&normal,0,0,0,.01f));
    assert(Heading_ESKF_Update(&slip,0,0,.01f,.01f));
    assert(slip.wheel_bias_gain<normal.wheel_bias_gain/20);
    assert(Heading_ESKF_Update(&slip,0,0,.1f,.01f) && slip.wheel_bias_gain==0);
}

static void test_invalid_and_odometry(void)
{
    Heading_ESKF_t s;
    assert(!Heading_ESKF_Init(NULL,0,0));
    assert(!Heading_ESKF_Init(&s,NAN,0));
    const float bad_dt[]={0,-.01f,.051f,NAN,INFINITY};
    for(unsigned i=0;i<sizeof(bad_dt)/sizeof(bad_dt[0]);++i)
    {
        assert(Heading_ESKF_Init(&s,0,0));
        assert(!Heading_ESKF_Update(&s,0,0,0,bad_dt[i]) && !s.initialized);
    }
    assert(Heading_ESKF_Init(&s,0,0));
    assert(!Heading_ESKF_Update(&s,NAN,0,0,.01f));
    assert(Heading_ESKF_Init(&s,0,0));s.p_yaw_bias=1;
    assert(!Heading_ESKF_Update(&s,0,0,0,.01f));

    Odometry_State_t odom; Odometry_Init(&odom);
    float p[3]={0}, wheels[3]; Motor_Chassis_Inverse(.2f,0,1,wheels);
    uint32_t us=UINT32_MAX-50000U;
    assert(Odometry_Update_Position_ESKF(&odom,p,wheels,0,1,us));
    for(unsigned i=1;i<=500;++i)
    {
        for(unsigned k=0;k<3;++k)p[k]=remainderf(p[k]+wheels[k]*.01f,25);
        us+=10000U;
        assert(Odometry_Update_Position_ESKF(&odom,p,wheels,remainder(i*.01,tau),1,us));
    }
    assert(fabs(odom.data.x_m-.2*sin(5))<.00001);
    assert(fabs(odom.data.y_m-.2*(1-cos(5)))<.00001);
    Odometry_State_t previous=odom;
    assert(!Odometry_Update_Position_ESKF(&odom,p,wheels,0,0,us));
    assert(memcmp(&previous,&odom,sizeof(odom))==0); /* 重复时刻完全无副作用。 */
    assert(!Odometry_Update_Position_ESKF(&odom,p,wheels,0,0,us+50001));
    assert(!odom.data.valid && !odom.heading.initialized);
    p[0]+=1; us+=60000U;
    assert(Odometry_Update_Position_ESKF(&odom,p,wheels,.7f,0,us));
    assert(odom.data.x_m==previous.data.x_m && odom.data.y_m==previous.data.y_m);
    assert(odom.data.yaw_rad==.7f);
    assert(Odometry_Update_Position(&odom,p,wheels,.8f,us+10000));
    assert(!odom.heading.initialized && odom.data.yaw_rad==.8f);
    assert(Odometry_Update_Position_ESKF(&odom,p,wheels,.9f,0,us+20000));
    assert(odom.heading.initialized && odom.data.yaw_rad==.9f);
    assert(!Odometry_Update_Position_ESKF(&odom,p,wheels,.9f,NAN,us+30000));
}


static void test_yaw_jump_and_source_time(void)
{
    Heading_Yaw_Guard_t guard = {0};
    const float jump = 1.570796327f;
    assert(Heading_Yaw_Guard_Update(&guard, 0, 0, UINT32_MAX - 9U, 0.01f) == HEADING_YAW_OBSERVED);
    assert(Heading_Yaw_Guard_Update(&guard, jump, 0, 0U, 0.01f) == HEADING_YAW_PREDICTED);
    assert(guard.yaw_rad == 0);
    assert(Heading_Yaw_Guard_Update(&guard, jump, 0, 0U, 0.01f) == HEADING_YAW_PREDICTED);
    assert(Heading_Yaw_Guard_Update(&guard, jump, 0, UINT32_MAX, 0.01f) == HEADING_YAW_PREDICTED);
    assert(guard.rejected_run == 1U);
    assert(Heading_Yaw_Guard_Update(&guard, 0, 0, 10U, 0.01f) == HEADING_YAW_OBSERVED);
    assert(Heading_Yaw_Guard_Update(&guard, jump, 0, 20U, 0.01f) == HEADING_YAW_PREDICTED);
    assert(Heading_Yaw_Guard_Update(&guard, jump, 0, 30U, 0.01f) == HEADING_YAW_PREDICTED);
    assert(Heading_Yaw_Guard_Update(&guard, jump, 0, 40U, 0.01f) == HEADING_YAW_INVALID);
    assert(Heading_Yaw_Guard_Update(&guard, 0, 0, 50U, 0.01f) == HEADING_YAW_INVALID);

    guard = (Heading_Yaw_Guard_t){0};
    assert(Heading_Yaw_Guard_Update(&guard, 3.13f, 4, 0U, 0.01f) == HEADING_YAW_OBSERVED);
    assert(Heading_Yaw_Guard_Update(&guard, -3.1132f, 4, 10U, 0.01f) == HEADING_YAW_OBSERVED);
    Heading_ESKF_t heading;
    assert(Heading_ESKF_Init(&heading, 0, 0));
    assert(Heading_ESKF_Update(&heading, jump, 0, 0, .01f));
    assert(heading.yaw_rad == 0 && heading.yaw_gain == 0);
    assert(Heading_ESKF_Update_Observed(&heading, jump, 0, 0, .01f, 0U));
    assert(heading.yaw_rejected_run == 1U);
    assert(Heading_ESKF_Update(&heading, jump, 0, 0, .01f));
    assert(!Heading_ESKF_Update(&heading, jump, 0, 0, .01f));
}

static void test_encoder_receive_time(void)
{
    Odometry_State_t odom;
    const float zero[3] = {0};
    const float jumped[3] = {1.5f, 0, 0};
    const uint32_t first[3] = {UINT32_MAX - 9U, UINT32_MAX - 19U, UINT32_MAX - 9U};
    const uint32_t second[3] = {0, 0, 0};
    Odometry_Init(&odom);
    assert(Odometry_Update_Position_ESKF_Timed(&odom, zero, zero, first, 0, 0, 0, 1U));
    assert(!Odometry_Update_Position_ESKF_Timed(&odom, jumped, zero, second, 0, 0, 10000U, 1U));
    assert(!odom.data.valid && odom.data.x_m == 0 && odom.data.y_m == 0);

    /* 恢复首帧重建基线。后轮间隔10ms，右前轮间隔20ms，分别校验。 */
    assert(Odometry_Update_Position_ESKF_Timed(&odom, zero, zero, first, 0, 0, 20000U, 1U));
    const float asynchronous[3] = {.430f, .790f, .400f};
    assert(Odometry_Update_Position_ESKF_Timed(&odom, asynchronous, zero, second, 0, 0, 30000U, 0U));
    assert(odom.data.valid && odom.heading.yaw_gain == 0);

    Odometry_Init(&odom);
    assert(Odometry_Update_Position_ESKF(&odom, zero, zero, 0, 0, 0));
    assert(!Odometry_Update_Position_ESKF(&odom, jumped, zero, 0, 0, 10000U));
    assert(odom.data.y_m == 0);

    Odometry_Init(&odom);
    const float yaw_jump = 1.570796327f;
    assert(Odometry_Update_Position_ESKF(&odom, zero, zero, 0, 0, 0));
    assert(Odometry_Update_Position_ESKF(&odom, zero, zero, yaw_jump, 0, 10000U));
    assert(Odometry_Update_Position_ESKF(&odom, zero, zero, yaw_jump, 0, 20000U));
    assert(!Odometry_Update_Position_ESKF(&odom, zero, zero, yaw_jump, 0, 30000U));
    assert(!Odometry_Update_Position_ESKF(&odom, zero, zero, 0, 0, 40000U));
    assert(!odom.data.valid && odom.data.yaw_rad == 0);
    Odometry_Init(&odom);
    assert(Odometry_Update_Position_ESKF(&odom, zero, zero, 0, 0, 50000U));
}

/* 解析轨迹从静止出发。编码器按各自的采样时刻生成，独立计算地面真值。 */
static void trajectory(double t, int pure_turn, double pose[3], double velocity[3])
{
    const double shape[3] = {pure_turn ? 0.0 : .21, pure_turn ? 0.0 : .09, pure_turn ? 1.0 : 0.0};
    const double frequency[3] = {1.7, 1.1, 1.7};
    for (unsigned axis = 0; axis < 3; ++axis)
    {
        pose[axis] = shape[axis] * (1.0 - cos(frequency[axis] * t));
        velocity[axis] = shape[axis] * frequency[axis] * sin(frequency[axis] * t);
    }
}

static float quantize(double value, double span, unsigned codes)
{
    return (float)(round((value / span + .5) * codes) * span / codes - .5 * span);
}

static double asynchronous_rms_mm(int pure_turn, double *peak_mm)
{
    const double tangent[3][2] = {{0, -1}, {.8660254037844386, .5}, {-.8660254037844386, .5}};
    const unsigned steps = 1000U;
    Odometry_State_t odom;
    Odometry_Init(&odom);
    double squares = 0;
    *peak_mm = 0;
    for (unsigned i = 0; i <= steps; ++i)
    {
        /* 正常收包间隔为 4～16 ms；RTOS ms 和 TIM2 us 分别回绕。 */
        const unsigned delay[3] = {0, i ? (5 * i + 1) % 7 : 0, i ? (2 * i + 3) % 7 : 0};
        const unsigned max_delay = delay[1] > delay[2] ? delay[1] : delay[2];
        uint32_t stamp[3];
        float position[3], wheel[3];
        double pose[3], velocity[3];
        for (unsigned k = 0; k < 3; ++k)
        {
            const unsigned ms = 10 * i - delay[k];
            stamp[k] = UINT32_MAX - 97U + ms;
            trajectory(.001 * ms, pure_turn, pose, velocity);
            const double phase = (tangent[k][0] * pose[0] + tangent[k][1] * pose[1] +
                                  CHASSIS_ROTATION_RADIUS * pose[2]) / CHASSIS_WHEEL_RADIUS;
            const double rate = (tangent[k][0] * velocity[0] + tangent[k][1] * velocity[1] +
                                 CHASSIS_ROTATION_RADIUS * velocity[2]) / CHASSIS_WHEEL_RADIUS;
            /* 初角靠近量程边界，旋转与平移用例都经过 25 rad 编码器回绕。 */
            position[k] = quantize(remainder(12.45 + phase, 25.0), 25.0, 65535);
            wheel[k] = quantize(rate, 80.0, 4095);
        }
        trajectory(.01 * i, pure_turn, pose, velocity);
        assert(Odometry_Update_Position_ESKF_Timed(&odom, position, wheel, stamp, (float)pose[2],
               (float)velocity[2], UINT32_MAX - 49999U + i * 10000U, 1U));
        assert(!odom.wheel_yaw_rejected);
        trajectory(.001 * (10 * i - max_delay), pure_turn, pose, velocity);
        const double error_mm = 1000.0 * hypot(odom.data.x_m - pose[0], odom.data.y_m - pose[1]);
        squares += error_mm * error_mm;
        *peak_mm = fmax(*peak_mm, error_mm);
    }
    return sqrt(squares / (steps + 1U));
}

static void test_asynchronous_translation(void)
{
    double spin_peak, translation_peak;
    const double spin_rms = asynchronous_rms_mm(1, &spin_peak);
    const double translation_rms = asynchronous_rms_mm(0, &translation_peak);
    printf("Asynchronous encoder RMS/peak (mm): pure turn %.4f/%.4f, translation %.4f/%.4f\n",
           spin_rms, spin_peak, translation_rms, translation_peak);
    fflush(stdout);
    assert(spin_rms < .07 && spin_peak < .15);
    assert(translation_rms < .07 && translation_peak < .15);
}

static void test_encoder_intervals_without_overlap(void)
{
    /* 第二组后，前轮都已越过下一组最慢后轮的时刻。 */
    const uint32_t stamps[][3] = {{100, 100, 100}, {104, 112, 116}, {106, 116, 121},
                                  {106, 119, 124}, {110, 123, 129}, {130, 130, 130}};
    const float bias[3] = {.35f, -.24f, .19f};
    const float fixed_angle[3] = {-11, 2, 7};
    const float encoder_code = 25.0f / 65535.0f;
    const double vx = .18, vy = -.08;
    float wheel[3];
    Motor_Chassis_Inverse((float)vx, (float)vy, 0, wheel);
    Odometry_State_t moving, parked, late_spin;
    Odometry_Init(&moving);
    Odometry_Init(&parked);
    Odometry_Init(&late_spin);
    for (unsigned i = 0; i < sizeof(stamps) / sizeof(stamps[0]); ++i)
    {
        const uint32_t *stamp = stamps[i];
        const uint32_t now_us = stamp[2] * 1000U;
        float position[3];
        for (unsigned k = 0; k < 3; ++k)
            position[k] = wheel[k] * (.001f * (stamp[k] - 100U));
        if (i == 3)
            position[0] += encoder_code; /* 同一毫秒内收到一个新的后轮位置码。 */
        const double previous_y = moving.data.y_m;
        assert(Odometry_Update_Position_ESKF_Timed(&moving, position, wheel, stamp, 0, 0, now_us, 1U));
        assert(Odometry_Update_Position_ESKF_Timed(&parked, fixed_angle, bias, stamp, 0, 0, now_us, 1U));
        assert(parked.data.x_m == 0 && parked.data.y_m == 0);
        /* 两前轮在 116 ms 都有静止观测，之后等速纯转；110 ms 前的位姿不应预支这次运动。 */
        float spin_angle[3], spin_rate[3];
        for (unsigned k = 0; k < 3; ++k)
        {
            spin_angle[k] = stamp[k] > 116U ? .010f * (stamp[k] - 116U) : 0.0f;
            spin_rate[k] = stamp[k] > 116U ? 10.0f : 0.0f;
        }
        const float yaw = spin_angle[2] * CHASSIS_WHEEL_RADIUS / CHASSIS_ROTATION_RADIUS;
        const float gyro = spin_rate[2] * CHASSIS_WHEEL_RADIUS / CHASSIS_ROTATION_RADIUS;
        assert(Odometry_Update_Position_ESKF_Timed(&late_spin, spin_angle, spin_rate, stamp, yaw, gyro, now_us, 1U));
        if (stamp[0] <= 110U)
            assert(hypot(late_spin.data.x_m, late_spin.data.y_m) < 1e-8);
        const double duration = .001 * (stamp[0] - 100U);
        assert(hypot(moving.data.x_m - vx * duration, moving.data.y_m - vy * duration) < .000011);
        if (i == 3)
            assert(fabs(moving.data.y_m - previous_y + (2.0 / 3.0) * CHASSIS_WHEEL_RADIUS * encoder_code) < 1e-8);
    }
    /* 三轮在 130 ms 再次同步，前面各拍留下的增量应全部结清。 */
    assert(hypot(moving.data.x_m - .030 * vx, moving.data.y_m - .030 * vy) < 1e-8);
}

static void test_alignment_reset_and_source_switch(void)
{
    const uint32_t first[3] = {100, 100, 100}, skewed[3] = {108, 113, 116};
    const uint32_t recover[3] = {158, 160, 164}, synced[3] = {172, 172, 172};
    const uint32_t last[3] = {208, 208, 208};
    const float zero[3] = {0}, moved[3] = {0, .06f, -.08f};
    const float new_baseline[3] = {9, -5, -8};
    Odometry_State_t odom;
    Odometry_Init(&odom);
    assert(Odometry_Update_Position_ESKF_Timed(&odom, zero, zero, first, 0, 0, 100000U, 1U));
    assert(Odometry_Update_Position_ESKF_Timed(&odom, moved, zero, skewed, 0, 0, 116000U, 1U));
    const double x = odom.data.x_m, y = odom.data.y_m;
    assert(hypot(x, y) > .0001);
    Odometry_Invalidate(&odom);
    assert(!odom.data.valid);
    assert(Odometry_Update_Position_ESKF_Timed(&odom, new_baseline, zero, recover, 0, 0, 164000U, 1U));
    assert(Odometry_Update_Position_ESKF_Timed(&odom, new_baseline, zero, synced, 0, 0, 172000U, 1U));
    assert(odom.data.x_m == x && odom.data.y_m == y);
    /* 原始位置、同步 ESKF、异步 ESKF 各自重建位置基线。 */
    assert(Odometry_Update_Position(&odom, zero, zero, 0, 184000U));
    assert(Odometry_Update_Position_ESKF(&odom, moved, zero, 0, 0, 194000U));
    assert(Odometry_Update_Position_ESKF_Timed(&odom, new_baseline, zero, last, 0, 0, 208000U, 1U));
    assert(odom.data.x_m == x && odom.data.y_m == y);
}

static void test_timed_wheel_yaw_rejection(void)
{
    Odometry_State_t odom;
    Odometry_Init(&odom);
    const uint32_t first_ms = UINT32_MAX - 25U;
    const uint32_t first_us = UINT32_MAX - 35000U;
    float position[3] = {0.0f};
    const float stationary[3] = {0.0f};
    const float asymmetric_slip[3] = {8.0f, 0.0f, 0.0f};
    Odometry_Feedback_Timing_t timing = {
        .wheel_time_ms = {first_ms, first_ms, first_ms},
        .imu_time_ms = first_ms,
        .control_time_ms = first_ms,
    };
    assert(Odometry_Update_Feedback(&odom, position, stationary, &timing, 0, 0, first_us, 1U));

    for (unsigned frame = 1U; frame <= 9U; ++frame)
    {
        const float *wheel = frame <= 4U ? asymmetric_slip : stationary;
        timing.control_time_ms = first_ms + frame * 10U;
        for (unsigned i = 0U; i < 3U; ++i)
        {
            timing.wheel_time_ms[i] = timing.control_time_ms;
            position[i] += wheel[i] * 0.01f;
        }

        /* 重复和乱序 IMU 不能计入连续证据。 */
        const uint8_t observe_yaw = frame != 2U;
        if (frame == 9U)
            timing.imu_time_ms = timing.control_time_ms - 15U;
        else if (observe_yaw)
            timing.imu_time_ms = timing.control_time_ms;
        assert(Odometry_Update_Feedback(&odom, position, wheel, &timing, 0, 0,
                                        first_us + frame * 10000U, observe_yaw));
        assert(odom.data.valid);

        if (frame < 4U)
        {
            assert(!odom.wheel_yaw_rejected);
            assert(odom.heading.wheel_bias_gain > 0.0f);
        }
        else
        {
            assert(odom.wheel_yaw_rejected);
            assert(odom.heading.wheel_bias_gain == 0.0f);
        }
    }
    assert(odom.data.y_m < 0.0); /* 门控只关闭轮角弱观测，保留平移积分。 */

    timing.control_time_ms += 10U;
    timing.imu_time_ms = timing.control_time_ms;
    for (unsigned i = 0U; i < 3U; ++i)
        timing.wheel_time_ms[i] = timing.control_time_ms;
    assert(Odometry_Update_Feedback(&odom, position, stationary, &timing, 0, 0, first_us + 100000U, 1U));
    assert(!odom.wheel_yaw_rejected && odom.heading.wheel_bias_gain > 0.0f);

    Odometry_Invalidate(&odom);
    assert(!odom.wheel_yaw_rejected && !odom.wheel_yaw_streak && !odom.data.valid);
}

int main(void)
{
    test_yaw_jump_and_source_time();
    test_encoder_receive_time();
    test_asynchronous_translation();
    test_encoder_intervals_without_overlap();
    test_alignment_reset_and_source_switch();
    test_timed_wheel_yaw_rejection();
    test_prediction_and_joseph();test_yaw_priority_and_noise();test_invalid_and_odometry();
    puts("PASS planar ESKF: YAW priority, weak/slipping encoder, gyro bias, noise, PSD, SO(2) wrap and SE(2) integration");
    return 0;
}
