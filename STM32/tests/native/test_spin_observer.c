#include "spin_observer.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>

int main(void)
{
    const double pi = 3.14159265358979323846;
    for (unsigned trial=0; trial<4; ++trial)
    {
        const unsigned turns = trial < 2 ? 30U : 35U;
        Spin_Observer_t state={0};
        double angle=0, yaw=3.1, speed=0;
        unsigned mask=0;
        uint32_t now=UINT32_MAX-200;
        float position[3]={0};
        assert(Spin_Observer_Update(&state, position, yaw, 0, now));
        for (unsigned sample=0; sample<25000; ++sample)
        {
            unsigned stage=0;
            const float target=Spin_Test_Speed(state.yaw_rad,turns,&stage);
            mask|=1U<<stage;
            const double change=fmax(-.008,fmin(.008,target-speed)); /* 1.6rad/s² */
            const double previous=speed;
            speed+=change;
            angle+=.5*(speed+previous)*.005;
            const double bias=(trial % 2) ? .005 : 0;
            yaw+=.5*(speed+previous)*.005+bias*.005;
            for(unsigned i=0;i<3;++i)position[i]=remainder(angle*.0982/.035,25.0);
            now+=5U;
            assert(Spin_Observer_Update(&state,position,remainder(yaw,2*pi),speed+bias,now));
            if(target==0 && speed==0)break;
        }
        assert(mask==31);
        assert(fabs(state.yaw_rad-turns*SPIN_RADIANS_PER_TURN)<.03);
        assert(fabs(state.encoder_rad-angle)<.001);
        if(trial % 2 == 0)assert(fabs(state.yaw_rad-angle)<.001);
        else assert(state.yaw_rad-state.encoder_rad>.3); /* 编码器有偏差时仍以YAW完成指定圈数。 */
        assert(fabs(state.gyro_rad-state.yaw_rad)<.001);
        assert(!Spin_Observer_Update(&state,position,0,0,now+51U));
        position[0]=NAN;
        assert(!Spin_Observer_Update(&state,position,0,0,now+5U));
    }
    unsigned stage=9;
    assert(Spin_Test_Speed(NAN,30,&stage)==0);
    assert(Spin_Test_Speed(-.3,30,&stage)==0);
    assert(Spin_Test_Speed(30*SPIN_RADIANS_PER_TURN+.3,30,&stage)==0);
    assert(Spin_Test_Speed(0,30,&stage)==2 && stage==0);
    assert(Spin_Test_Speed(40,30,&stage)==3 && stage==1);
    assert(Spin_Test_Speed(80,30,&stage)==4 && stage==2);
    assert(Spin_Test_Speed(120,30,&stage)==3 && stage==3);
    assert(Spin_Test_Speed(160,30,&stage)==2 && stage==4);
    assert(Spin_Test_Speed(0,34,&stage)==0);
    for (unsigned i=1; i<5; ++i)
    {
        const double boundary=i*7*SPIN_RADIANS_PER_TURN;
        Spin_Test_Speed(boundary-.001,35,&stage);
        assert(stage==i-1);
        Spin_Test_Speed(boundary+.001,35,&stage);
        assert(stage==i);
    }
    puts("PASS spin: 30/35 YAW turns, 2/3/4/3/2 profile, gyro bias, multiwrap and clock rollover");
    return 0;
}
