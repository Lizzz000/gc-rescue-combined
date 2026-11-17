#include "point_plan.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>

static const Point_Plan_Config_t cfg={.2f,.5f,.3f,.8f,.12f,.01f,.06981317f};
static void Plant(unsigned axis)
{
    const double angle=axis*3.141592653589793/4;
    double pose[3]={.31,-.27,2.8}, start[3]={.31,-.27,2.8};
    double goal[3]={start[0]+1.2*cos(angle),start[1]+1.2*sin(angle),-2.2};
    Point_Plan_t plan;
    assert(Point_Plan_Init(&plan,start,goal));
    assert(fabs(plan.length_m-1.2)<1e-9 && fabs(plan.yaw_delta_rad)<3.141593);
    double velocity[3]={0}, traveled=0, turn=0, worst_cross=0;
    unsigned stopped=0, at=0;
    for (unsigned step=0;step<4000;++step)
    {
        Point_Plan_Output_t output;
        assert(Point_Plan_Step(&plan,&cfg,pose,&output));
        assert(hypot(output.vx_mps,output.vy_mps)<=.200001 && fabs(output.wz_rad_s)<=.500001);
        if(output.arrived && !stopped) {stopped=1;at=step;}
        double target[3]={cos(pose[2])*output.vx_mps-sin(pose[2])*output.vy_mps,
                          sin(pose[2])*output.vx_mps+cos(pose[2])*output.vy_mps,output.wz_rad_s};
        if(stopped) target[0]=target[1]=target[2]=0;
        for(unsigned i=0;i<3;++i)
        {
            double increment=(target[i]-velocity[i])*.005/.06;
            double limit=(i==2?1.5:.45)*.005;
            velocity[i]+=fmax(-limit,fmin(limit,increment));
            pose[i]+=velocity[i]*.005;
        }
        traveled+=hypot(velocity[0],velocity[1])*.005;
        turn+=fabs(velocity[2])*.005;
        worst_cross=fmax(worst_cross,fabs((pose[0]-start[0])*sin(angle)-(pose[1]-start[1])*cos(angle)));
        if(stopped && step>at+200) break;
    }
    double error=hypot(goal[0]-pose[0],goal[1]-pose[1]);
    printf("path axis=%u straight=%.3fm traveled=%.4fm error=%.4fm time=%.3fs turn=%.3frad\n",
           axis,plan.length_m,traveled,error,at*.005,turn);
    assert(stopped && at*.005<9.0 && error<.02 && worst_cross<.005);
    assert(traveled>=1.18 && traveled<=1.22 && turn<=fabs(plan.yaw_delta_rad)+.08);
}
int main(void)
{
    for(unsigned i=0;i<8;++i) Plant(i);
    Point_Plan_t p;
    double start[3]={0,0,3.124139361}, goal[3]={1.2,.6,-3.124139361};
    assert(Point_Plan_Init(&p,start,goal));
    assert(fabs(p.length_m-sqrt(1.8))<1e-9 && fabs(p.yaw_delta_rad-.034906585)<1e-6);
    Point_Plan_Output_t out;
    double displaced[3]={.2,-.1,1.1};
    assert(Point_Plan_Step(&p,&cfg,displaced,&out));
    double wx=cos(displaced[2])*out.vx_mps-sin(displaced[2])*out.vy_mps;
    double wy=sin(displaced[2])*out.vx_mps+cos(displaced[2])*out.vy_mps;
    assert(fabs(wx*.7-wy*1.0)<1e-7); /* 偏离后仍取当前点到目标的最短直线。 */
    goal[0]=NAN;
    assert(!Point_Plan_Init(&p,start,goal));
    assert(!Point_Plan_Step(&p,&cfg,displaced,&out) && out.vx_mps==0 && out.wz_rad_s==0);
    puts("PASS point plan: shortest line/angle, 8 closed-loop directions, braking and invalid input");
    return 0;
}
