/* 主机相对耗时基准：实际算法C，固定工作量；不能代替STM32 DWT周期测量。 */
#include "AHRS.h"
#include "pid.h"
#include "adaptive_kalman.h"
#include "motor_chassis.h"
#include "chassis_odometry.h"
#include <stdio.h>
#include <time.h>

#define N 1000000U
static volatile float sink;
static void report(const char *name, clock_t start)
{
    printf("%s %.3f\n",name,1e9*(double)(clock()-start)/CLOCKS_PER_SEC/N);
}
int main(void)
{
    float q[4]={1,0,0,0},g[3]={.01f,-.02f,.1f},a[3]={0,0,9.80665f},m[3]={0};
    clock_t t=clock();
    for(unsigned i=0;i<N;++i) AHRS_update(q,.01f,g,a,m);
    sink=q[0]; report("ahrs_update_ns",t);
    float yaw,pitch,roll;
    t=clock();for(unsigned i=0;i<N;++i) get_angle(q,&yaw,&pitch,&roll);
    sink=yaw;report("euler_ns",t);
    pid_type_def pid;const float gains[3]={2,.5f,.2f};
    PID_init(&pid,PID_POSITION,gains,3,2);
    t=clock();for(unsigned i=0;i<N;++i) sink=PID_Step(&pid,i%20<10?.1f:-.1f,.02f,.005f);
    report("pid_ns",t);
    float wheels[3],body[3];
    t=clock();for(unsigned i=0;i<N;++i){Motor_Chassis_Inverse(.2f,.1f,.5f,wheels);Motor_Chassis_Forward(wheels,body);sink=body[0];}
    report("kinematics_ns",t);
    AdaptiveKalman k;AdaptiveKalman_Init(&k,15,25,4000);
    t=clock();for(unsigned i=0;i<N;++i) AdaptiveKalman_Update(&k,5000+(i%2?15:-15),7500,i*10U);
    sink=k.position[0];report("kalman_ns",t);
    Odometry_State_t odom;Odometry_Init(&odom);float positions[3]={0},velocities[3]={.1f,.1f,.1f};
    t=clock();for(unsigned i=0;i<N;++i){for(unsigned j=0;j<3;++j){positions[j]+=.001f;if(positions[j]>12.5f)positions[j]-=25;}
        Odometry_Update_Position(&odom,positions,velocities,.1f,i*10000U);}
    sink=(float)odom.data.x_m;report("odometry_ns",t);
    return 0;
}
