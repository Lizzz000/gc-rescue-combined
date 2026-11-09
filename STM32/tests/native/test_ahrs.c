#include <assert.h>
#include <math.h>
#include <stdio.h>
#include "MahonyAHRS.h"
#include "AHRS.h"

#define PI 3.14159265358979323846f

/** @brief 独立从四元数计算Z轴角。 @param[in] q 四元数w/x/y/z。 @return yaw弧度。 */
static float yaw(const float q[4])
{
    return atan2f(2.0f * (q[0] * q[3] + q[1] * q[2]),
                  q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3]);
}

/**
 * @brief 验证重力初始化得到有限单位四元数，并把实测重力旋到世界+Z。
 * @param[in] accel 静止加速度向量，m/s²，有限且非零。
 * @param[in] check_yaw 非零时验证初始yaw为0；俯仰极点不检查欧拉航向。
 * @return 无；重力对准或四元数不符时断言失败。
 */
static void assert_gravity_initialization(const float accel[3], unsigned check_yaw)
{
    float input[3] = {accel[0],accel[1],accel[2]};
    float mag[3] = {0,0,0};
    float q[4] = {0,0,0,0};
    assert(AHRS_init(q, input, mag) == 0U);
    float norm_squared = 0.0f;
    for (unsigned i = 0; i < 4U; ++i)
    {
        assert(isfinite(q[i]));
        norm_squared += q[i] * q[i];
    }
    assert(fabsf(norm_squared - 1.0f) < 1e-6f);

    /* 独立的四元数旋转矩阵，不复用初始化中的atan2或半角表达式。 */
    float w=q[0], x=q[1], y=q[2], z=q[3];
    float world_x = (w*w+x*x-y*y-z*z)*accel[0]
                  + 2.0f*(x*y-w*z)*accel[1] + 2.0f*(x*z+w*y)*accel[2];
    float world_y = 2.0f*(x*y+w*z)*accel[0]
                  + (w*w-x*x+y*y-z*z)*accel[1] + 2.0f*(y*z-w*x)*accel[2];
    float world_z = 2.0f*(x*z-w*y)*accel[0] + 2.0f*(y*z+w*x)*accel[1]
                  + (w*w-x*x-y*y+z*z)*accel[2];
    float gravity_norm = sqrtf(accel[0]*accel[0] + accel[1]*accel[1] + accel[2]*accel[2]);
    assert(fabsf(world_x) < 1e-4f && fabsf(world_y) < 1e-4f);
    assert(fabsf(world_z - gravity_norm) < 1e-4f);
    if (check_yaw)
        assert(fabsf(yaw(q)) < 1e-6f);
}

/**
 * @brief 俯仰极点及附近的理想静止输入经六轴/九轴更新后仍应输出有限角度。
 * @param[in] pitch_deg 已知俯仰角，度，范围[-90,90]。
 * @param[in] roll_deg 已知横滚角，度。
 * @param[in] use_mag 非零使用九轴；零使用六轴。
 * @return 无；pitch非有限或偏离已知角度时断言失败。
 */
static void assert_pitch_pole(double pitch_deg, double roll_deg, unsigned use_mag)
{
    const double rad = 3.14159265358979323846 / 180.0;
    double sp=sin(pitch_deg*rad), cp=cos(pitch_deg*rad);
    double sr=sin(roll_deg*rad), cr=cos(roll_deg*rad);
    /* 已知yaw=0的旋转矩阵将世界重力与磁场转入机体，不从待测四元数反推输入。 */
    float accel[3] = {(float)(-9.81*sp),(float)(9.81*cp*sr),(float)(9.81*cp*cr)};
    float mag[3] = {0,0,0};
    if (use_mag)
    {
        mag[0] = (float)(25.0*cp - 40.0*sp);
        mag[1] = (float)((25.0*sp + 40.0*cp)*sr);
        mag[2] = (float)((25.0*sp + 40.0*cp)*cr);
    }
    float q[4];
    assert(AHRS_init(q, accel, mag) == use_mag);
    for (unsigned i = 0; i <= 1000U; ++i)
    {
        if (i)
            MahonyAHRSupdate(q, 0,0,0, accel[0],accel[1],accel[2],
                            mag[0],mag[1],mag[2], 0.01f);
        float heading, pitch, roll;
        get_angle(q, &heading, &pitch, &roll);
        assert(isfinite(heading) && isfinite(pitch) && isfinite(roll));
        assert(fabs(pitch / rad - pitch_deg) < 0.001);
    }
}

/**
 * @brief 从静止姿态产生一次纯积分反馈，测量其随时间间隔的缩放。
 * @param[in] dt 更新时间，秒，大于 0。
 * @param[in] use_mag 非零使用九轴；零使用六轴。
 * @return 本次重力纠正产生的横滚角，rad。
 */
static float integral_roll_step(float dt, unsigned use_mag)
{
    float reset[4] = {1,0,0,0};
    float q[4] = {1,0,0,0};
    float saved_kp = twoKp, saved_ki = twoKi;
    twoKp = 0.0f;
    twoKi = 0.0f;
    MahonyAHRSupdateIMU(reset, 0,0,0, 0,0,9.80665f, dt);
    twoKi = 1.0f;
    if (use_mag)
        MahonyAHRSupdate(q, 0,0,0, 0,6.93435f,6.93435f, 1,0,0, dt);
    else
        MahonyAHRSupdateIMU(q, 0,0,0, 0,6.93435f,6.93435f, dt);
    float angle = 2.0f * atan2f(q[1], q[0]);
    twoKi = 0.0f;
    MahonyAHRSupdateIMU(reset, 0,0,0, 0,0,9.80665f, dt);
    twoKp = saved_kp;
    twoKi = saved_ki;
    return angle;
}

/**
 * @brief 已知正确姿态与理想静止传感器输入运行100秒，不应产生伪反馈。
 * @param[in] initial 初始单位四元数，顺序w/x/y/z。
 * @param[in] accel 已知姿态下的重力向量，m/s²。
 * @param[in] mag 已知姿态下的磁场向量，μT；全零验证六轴分支。
 * @param[in] roll_deg 已知横滚角，度。
 * @param[in] pitch_deg 已知俯仰角，度。
 * @param[in] yaw_deg 已知航向角，度。
 * @return 无；四元数长度或姿态偏离时断言失败。
 */
static void assert_stationary_orientation(const float initial[4], const float accel[3],
                                         const float mag[3], float roll_deg,
                                         float pitch_deg, float yaw_deg)
{
    float q[4];
    if (mag[0] != 0.0f || mag[1] != 0.0f || mag[2] != 0.0f)
    {
        assert(AHRS_init(q, accel, mag) == 1U);
        /* 初值本身就应与已知姿态一致，q 和 -q 表示同一旋转。 */
        float dot = 0.0f;
        for (unsigned i = 0; i < 4U; ++i)
            dot += q[i] * initial[i];
        assert(fabsf(fabsf(dot) - 1.0f) < 2e-6f);
    }
    for (unsigned i = 0; i < 4U; ++i)
        q[i] = initial[i];
    for (unsigned i = 0; i < 10000U; ++i)
    {
        MahonyAHRSupdate(q, 0,0,0, accel[0],accel[1],accel[2],
                        mag[0],mag[1],mag[2], 0.01f);
        float norm_squared = q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3];
        assert(fabsf(norm_squared - 1.0f) < 2e-6f);
    }
    float heading, pitch, roll;
    get_angle(q, &heading, &pitch, &roll);
    assert(fabsf(roll * 180.0f / PI - roll_deg) < 0.001f);
    assert(fabsf(pitch * 180.0f / PI - pitch_deg) < 0.001f);
    assert(fabsf(heading * 180.0f / PI - yaw_deg) < 0.001f);
}

/** @brief 验证实测时间积分、重力静止、磁航向纠正及零磁退化。 @return 成功为0。 */
int main(void)
{
    float quat[4] = {0,0,0,0};
    float gravity[3] = {0,0,9.80665f}, no_mag[3] = {0,0,0};
    assert(AHRS_init(quat, gravity, no_mag) == 0U);
    assert(quat[0] == 1.0f && quat[1] == 0.0f && quat[2] == 0.0f && quat[3] == 0.0f);
    const float tilted_gravity[3] = {3.0f, -4.0f, 8.0f};
    const float parallel_mag[3] = {15.0f, -20.0f, 40.0f};
    assert(AHRS_init(quat, tilted_gravity, parallel_mag) == 0U);
    const float west_mag[3] = {0, 25, 40};
    assert(AHRS_init(quat, gravity, west_mag) == 1U);
    assert(fabsf(yaw(quat) + PI / 2.0f) < 1e-6f);
    assert(AHRS_init(quat, gravity, no_mag) == 0U);

    const float initial_gravity[][3] = {
        {0,0,9.81f}, {3.0f,-4.0f,8.0f}, {0,0,-9.81f},
        {9.81f,0,0}, {-9.81f,0,0}
    };
    for (unsigned i = 0; i < sizeof initial_gravity / sizeof initial_gravity[0]; ++i)
        assert_gravity_initialization(initial_gravity[i], i < 3U);

    const double pole_angles[][2] = {
        {90.0,0}, {-90.0,0}, {89.9809,-175.0}, {-89.9809,-175.0},
        {89.999,35.0}, {-89.999,35.0}
    };
    for (unsigned use_mag = 0; use_mag < 2U; ++use_mag)
        for (unsigned i = 0; i < sizeof pole_angles / sizeof pole_angles[0]; ++i)
            assert_pitch_pole(pole_angles[i][0], pole_angles[i][1], use_mag);

    float stationary[4] = {1,0,0,0};
    for (unsigned i = 0; i < 100U; ++i)
        MahonyAHRSupdateIMU(stationary, 0,0,0, 0,0,9.80665f, 0.01f);
    assert(fabsf(stationary[0] - 1.0f) < 0.003f);
    assert(fabsf(stationary[1]) < 1e-6f && fabsf(stationary[2]) < 1e-6f && fabsf(stationary[3]) < 1e-6f);

    float rotation[4] = {1,0,0,0};
    for (unsigned i = 0; i < 100U; ++i)
        MahonyAHRSupdateIMU(rotation, 0,0,PI / 2.0f, 0,0,9.80665f, 0.01f);
    assert(fabsf(yaw(rotation) - PI / 2.0f) < 0.002f);

    float six_axis[4] = {cosf(PI / 8.0f),0,0,sinf(PI / 8.0f)};
    float nine_axis[4] = {cosf(PI / 8.0f),0,0,sinf(PI / 8.0f)};
    for (unsigned i = 0; i < 2000U; ++i)
    {
        float dt = i % 2U ? 0.016f : 0.004f;
        MahonyAHRSupdateIMU(six_axis, 0,0,0, 0,0,9.80665f, dt);
        MahonyAHRSupdate(nine_axis, 0,0,0, 0,0,9.80665f, 1,0,0, dt);
    }
    assert(fabsf(yaw(six_axis) - PI / 4.0f) < 0.002f);
    assert(fabsf(yaw(nine_axis)) < 0.01f);

    float missing_mag[4] = {1,0,0,0}, reference[4] = {1,0,0,0};
    for (unsigned i = 0; i < 100U; ++i)
    {
        float dt = i % 2U ? 0.007f : 0.015f;
        MahonyAHRSupdate(missing_mag, 0.1f,-0.2f,0.3f, 0,0,9.80665f, 0,0,0, dt);
        MahonyAHRSupdateIMU(reference, 0.1f,-0.2f,0.3f, 0,0,9.80665f, dt);
    }
    for (unsigned i = 0; i < 4U; ++i)
        assert(fabsf(missing_mag[i] - reference[i]) < 1e-6f);

    float angular_rate[3] = {0,0,PI / 2.0f};
    for (unsigned i = 0; i < 100U; ++i)
        AHRS_update(quat, 0.01f, angular_rate, gravity, no_mag);
    assert(fabsf(yaw(quat) - PI / 2.0f) < 0.002f);

    /* 100 次调用只有 0.65 秒；不能按调用次数假定经过了 1 秒。 */
    float varying_six[4] = {1,0,0,0}, varying_nine[4] = {1,0,0,0};
    const float intervals[] = {0.003f, 0.005f, 0.011f, 0.007f};
    float elapsed = 0.0f;
    twoKp = 0.0f; /* 只检验角速度积分；磁场反馈另由上面的收敛测试检验。 */
    for (unsigned i = 0; i < 100U; ++i)
    {
        float dt = intervals[i % 4U];
        elapsed += dt;
        AHRS_update(varying_six, dt, angular_rate, gravity, no_mag);
        MahonyAHRSupdate(varying_nine, 0,0,PI / 2.0f, 0,0,9.80665f, 1,0,0, dt);
    }
    twoKp = 1.0f;
    assert(fabsf(elapsed - 0.65f) < 1e-6f);
    assert(fabsf(yaw(varying_six) - PI / 2.0f * elapsed) < 0.002f);
    assert(fabsf(yaw(varying_nine) - PI / 2.0f * elapsed) < 0.002f);

    /* 首次 Ki 反馈先按 dt 积分，再按 dt 转成角度，步长三倍应产生九倍转角。 */
    for (unsigned use_mag = 0; use_mag < 2U; ++use_mag)
    {
        float short_step = integral_roll_step(0.01f, use_mag);
        float long_step = integral_roll_step(0.03f, use_mag);
        assert(short_step > 0.0f);
        assert(fabsf(long_step / short_step - 9.0f) < 0.001f);
    }

    /* 独立的ZYX姿态向量：yaw=70°、pitch=-20°、roll=30°。 */
    float known_orientation[4] = {0.7534408929f,0.3049979070f,0.0087997743f,0.5824308213f};
    float heading, pitch, roll;
    get_angle(known_orientation, &heading, &pitch, &roll);
    assert(fabsf(heading - 70.0f * PI / 180.0f) < 1e-5f);
    assert(fabsf(pitch + 20.0f * PI / 180.0f) < 1e-5f);
    assert(fabsf(roll - 30.0f * PI / 180.0f) < 1e-5f);

    /* 独立ZYX旋转矩阵将世界重力[0,0,9.81]及磁场[25,0,40]转到机体所得向量。
     * 初始姿态已经正确，零陀螺输入应保持它；水平零航向无法暴露非单位q的伪反馈。 */
    const float yaw_112[4] = {0.5591929035f,0,0,0.8290375726f};
    const float level_accel[3] = {0,0,9.81f};
    const float yaw_112_mag[3] = {-9.3651648354f,-23.1795963642f,40.0f};
    assert_stationary_orientation(yaw_112, level_accel, yaw_112_mag, 0,0,112);

    const float roll_30[4] = {0.9659258263f,0.2588190451f,0,0};
    const float roll_30_accel[3] = {0,4.905f,8.4957092111f};
    const float roll_30_mag[3] = {25.0f,20.0f,34.6410161514f};
    assert_stationary_orientation(roll_30, roll_30_accel, no_mag, 30,0,0);
    assert_stationary_orientation(roll_30, roll_30_accel, roll_30_mag, 30,0,0);

    const float tilted[4] = {0.5824308213f,0.0087997743f,0.3049979070f,0.7534408929f};
    const float tilted_accel[3] = {-3.3552176060f,4.6091923050f,7.9833552540f};
    const float tilted_mag[3] = {-21.7156508541f,-3.0133118485f,41.7654218186f};
    assert_stationary_orientation(tilted, tilted_accel, tilted_mag, 30,20,110);

    puts("PASS AHRS: tilt-compensated startup heading, gravity initialization, finite pole angles, variable dt, magnetic correction, 6-axis fallback, known orientations and unit quaternion");
    return 0;
}
