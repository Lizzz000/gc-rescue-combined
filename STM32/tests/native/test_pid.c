#include <assert.h>
#include <math.h>
#include <stdio.h>
#include "pid.h"

/** @brief 连续时间积分不随步长改变，测量微分无目标阶跃冲击，饱和可退积分。 */
static void test_continuous_pid(void)
{
    pid_type_def fast, slow;
    const float integral[3] = {0.0f, 2.0f, 0.0f};
    PID_init(&fast, PID_POSITION, integral, 10.0f, 5.0f);
    PID_init(&slow, PID_POSITION, integral, 10.0f, 5.0f);
    for (unsigned i = 0; i < 100U; ++i) PID_Step(&fast, 3.0f, 0.0f, 0.01f);
    for (unsigned i = 0; i < 50U; ++i) PID_Step(&slow, 3.0f, 0.0f, 0.02f);
    assert(fast.Iout == 5.0f && slow.Iout == 5.0f);
    PID_clear(&fast); PID_clear(&slow);
    for (unsigned i = 0; i < 20U; ++i) PID_Step(&fast, 3.0f, 0.0f, 0.01f);
    for (unsigned i = 0; i < 10U; ++i) PID_Step(&slow, 3.0f, 0.0f, 0.02f);
    assert(fabsf(fast.Iout - 1.2f) < 0.00001f && fabsf(slow.Iout - fast.Iout) < 0.00001f);

    const float derivative[3] = {2.0f, 0.0f, 0.5f};
    PID_init(&fast, PID_POSITION, derivative, 10.0f, 5.0f);
    assert(PID_Step(&fast, 1.0f, 4.0f, 0.01f) == 0.0f);
    assert(fast.Dout == -2.0f);
    assert(PID_Step(&fast, 2.0f, 4.0f, 0.02f) == 2.0f && fast.Dout == -2.0f);

    const float antiwindup[3] = {2.0f, 1.0f, 0.0f};
    PID_init(&fast, PID_POSITION, antiwindup, 3.0f, 5.0f);
    for (unsigned i = 0; i < 100U; ++i) assert(PID_Step(&fast, 2.0f, 0.0f, 0.01f) == 3.0f);
    assert(fast.Iout == 0.0f);
    assert(fabsf(PID_Step(&fast, 1.0f, 0.0f, 2.0f) - 3.0f) < 0.00001f);
    assert(fast.Iout == 1.0f); /* 输出边界只接受剩余的积分空间。 */
    assert(PID_Step(&fast, -1.0f, 0.0f, 0.1f) < 0.0f && fast.Iout < 1.0f);
    PID_clear(&fast);
    for (unsigned i = 0; i < 100U; ++i) assert(PID_Step(&fast, -2.0f, 0.0f, 0.01f) == -3.0f);
    assert(fast.Iout == 0.0f);
}

/** @brief 检查真实温控PID的输出/积分约束与清零后的状态。 @return 成功为0。 */
int main(void)
{
    pid_type_def pid;
    const float gains[3] = {1600.0f, 1.6f, 0.0f};
    PID_init(&pid, PID_POSITION, gains, 4500.0f, 4400.0f);
    for (unsigned i = 0; i < 200U; ++i)
    {
        float output = PID_calc(&pid, 20.0f, 45.0f);
        assert(isfinite(output) && output >= -4500.0f && output <= 4500.0f);
        assert(fabsf(pid.Iout) <= 4400.0f);
    }
    assert(pid.out == 4500.0f && pid.Iout == 4400.0f);
    for (unsigned i = 0; i < 600U; ++i)
    {
        float output = PID_calc(&pid, 55.0f, 45.0f);
        assert(isfinite(output) && output >= -4500.0f && output <= 4500.0f);
        assert(fabsf(pid.Iout) <= 4400.0f);
    }
    assert(pid.out == -4500.0f && pid.Iout == -4400.0f);
    PID_clear(&pid);
    assert(pid.out == 0.0f && pid.Iout == 0.0f && pid.Pout == 0.0f && pid.Dout == 0.0f);
    assert(pid.Kp == gains[0] && pid.Ki == gains[1] && pid.max_out == 4500.0f);
    assert(PID_calc(&pid, 45.0f, 45.0f) == 0.0f);
    const float discrete[3] = {2.0f, 0.5f, 3.0f};
    PID_init(&pid, PID_POSITION, discrete, 100.0f, 50.0f);
    assert(PID_calc(&pid, 0.0f, 1.0f) == 5.5f);
    assert(PID_calc(&pid, 0.5f, 1.0f) == 0.25f);
    test_continuous_pid();
    puts("PASS PID: discrete regression, continuous I, measurement D, symmetric antiwindup");
    return 0;
}
