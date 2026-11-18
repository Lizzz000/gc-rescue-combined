#include "heading_eskf.h"
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
    assert(Heading_Yaw_Guard_Update(&guard, 0, 0, UINT32_MAX - 9U) == HEADING_YAW_OBSERVED);
    assert(Heading_Yaw_Guard_Update(&guard, jump, 0, 0U) == HEADING_YAW_PREDICTED);
    assert(guard.yaw_rad == 0);
    assert(Heading_Yaw_Guard_Update(&guard, jump, 0, 0U) == HEADING_YAW_PREDICTED);
    assert(Heading_Yaw_Guard_Update(&guard, jump, 0, UINT32_MAX) == HEADING_YAW_PREDICTED);
    assert(guard.rejected_run == 1U);
    assert(Heading_Yaw_Guard_Update(&guard, 0, 0, 10U) == HEADING_YAW_OBSERVED);
    assert(Heading_Yaw_Guard_Update(&guard, jump, 0, 20U) == HEADING_YAW_PREDICTED);
    assert(Heading_Yaw_Guard_Update(&guard, jump, 0, 30U) == HEADING_YAW_PREDICTED);
    assert(Heading_Yaw_Guard_Update(&guard, jump, 0, 40U) == HEADING_YAW_INVALID);
    assert(Heading_Yaw_Guard_Update(&guard, 0, 0, 50U) == HEADING_YAW_INVALID);

    guard = (Heading_Yaw_Guard_t){0};
    assert(Heading_Yaw_Guard_Update(&guard, 3.13f, 4, 0U) == HEADING_YAW_OBSERVED);
    assert(Heading_Yaw_Guard_Update(&guard, -3.1132f, 4, 10U) == HEADING_YAW_OBSERVED);
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

int main(void)
{
    test_yaw_jump_and_source_time();
    test_encoder_receive_time();
    test_prediction_and_joseph();test_yaw_priority_and_noise();test_invalid_and_odometry();
    puts("PASS planar ESKF: YAW priority, weak/slipping encoder, gyro bias, noise, PSD, SO(2) wrap and SE(2) integration");
    return 0;
}
