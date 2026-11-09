#include <assert.h>
#include <math.h>
#include <setjmp.h>
#include <stdio.h>
#include <string.h>
#include "main.h"
#include "tim.h"
#include "task.h"
#include "BMI088driver.h"
#include "ist8310driver.h"
#include "ist8310driver_middleware.h"
#include "INS_task.h"
#include "MahonyAHRS.h"
#include "vofa.h"

/* 独立验收：48 点、每点 10 ms。默认启动磁场 [250,0,90] LSB 经电脑端双精度
 * A*(raw-b) 校正为 [66.8565531,8.9557789,25.6500239] µT，水平板应得到下列 yaw。 */
#define EXPECTED_CALI_SAMPLES 48U
#define INTERRUPTED_CALI_SAMPLES 16U
#define EXPECTED_LEVEL_YAW_DEG (INS_USE_MAGNETOMETER ? -7.62964512f : 0.0f)

SPI_HandleTypeDef hspi1;
I2C_HandleTypeDef hi2c3;
TIM_HandleTypeDef htim2, htim10;
uint32_t test_tick, timer_start_count;
static uint8_t accel_regs[128], gyro_regs[128], mag_regs[128];
static uint8_t selected, spi_reg, spi_read;
static uint32_t spi_position, accel_reads, mag_reads, loops;
static uint32_t vofa_calls, sampling_waits;
static uint64_t clock_us, boot_us, last_bmi_us, baseline_us;
static uint32_t timer_offset, baseline_counter;
static uint8_t task_mode, runtime_started, wrapped;
static uint32_t calibration_samples, output_limit;
static uint32_t pwm_starts, heater_updates, last_controlled_read, cold_heating;
static uint32_t spi_calls, failed_spi_calls, runtime_skips;
static uint32_t error_vofa_count;
static uint8_t inject_transfer_error, failure_pending, recovery_pending, spi_timeout_pending;
static float last_quat[4];
static uint64_t previous_fusion_us;
static uint32_t magnetic_failures, initial_gravity_checks;
static HAL_StatusTypeDef next_i2c_status = HAL_OK;
static enum { STATIC_BIAS, MOVING_MEAN, MOVING_PEAK, BAD_GYRO_ID, DYNAMIC_WRAP, COLD_START,
              CALI_TEMP_INVALID, RUNTIME_ERROR, TILTED_WORLD, TILTED_AVERAGE, MAGNETIC_READS,
              START_MAG_RETRY } scenario;
static jmp_buf task_finished;
static unsigned critical_depth, critical_entries;

/** @brief 只允许短、成对的快照临界区，所有HAL和任务等待仍在区外。 */
void test_enter_critical(void) { assert(critical_depth == 0U); ++critical_depth; ++critical_entries; }
void test_exit_critical(void) { assert(critical_depth == 1U); --critical_depth; }

/** @brief 未就绪不覆盖调用者原数据。 */
static void assert_ins_not_ready(void)
{
    INS_control_data_t before = {123.0f, 456.0f, 789U}, after = before;
    assert(INS_Get_Data(&after) == 0U);
    assert(memcmp(&before, &after, sizeof(before)) == 0);
    assert(critical_depth == 0U);
}

/** @brief 用已知机体重力方向验证启动姿态，独立于AHRS_init的求四元数步骤。 */
static void assert_averaged_gravity(void)
{
    /* 芯片均值[-5461,3277,8874]经[y,-x,z]安装矩阵后为此向量。 */
    const float gravity[3] = {3277.0f, 5461.0f, 8874.0f};
    float length = sqrtf(gravity[0]*gravity[0] + gravity[1]*gravity[1] + gravity[2]*gravity[2]);
    float w = INS_quat[0], x = INS_quat[1], y = INS_quat[2], z = INS_quat[3];
    float norm = w*w + x*x + y*y + z*z;
    assert(norm > 0.9f && norm < 1.1f);
    const float measured[3] = {2.0f*(x*z-w*y)/norm, 2.0f*(w*x+y*z)/norm,
                               (w*w-x*x-y*y+z*z)/norm};
    for (unsigned i = 0U; i < 3U; ++i)
        assert(fabsf(measured[i] - gravity[i]/length) < 0.00002f);
}

/** @brief 对真实算法入口作分支对照，检查磁标定、I2C错误走六轴及磁场恢复。
 * @note 校正期望值由首轮标定参数保留9位有效数字后在电脑端用double独立计算；
 *       不调用生产端校正函数，不在测试中重写Mahony数学公式。
 */
static void assert_magnetic_update(void)
{
    const uint8_t failed = vofa_calls == 1U;
    const uint8_t no_new_data = vofa_calls == 2U;
    const float field[3] = {30.0f, vofa_calls < 3U ? 30.0f : -45.0f,
                            vofa_calls < 3U ? 0.0f : 6.0f};
    const float corrected[2][3] = {
        {15.498623588530f, 41.707988292773f, -9.351957348783f},
        {18.395435144530f, -35.853523121827f, -1.602856376283f}
    };
    const float *expected_field = corrected[vofa_calls < 3U ? 0U : 1U];
    /* 原始物理量保持不变；融合对照会检出遗漏校正、重复扣偏或矩阵顺序错误。 */
    for (unsigned i = 0U; i < 3U; ++i)
        assert(fabsf(ist8310_real_data.mag[i] - field[i]) < 0.00001f);
    assert(!!(ist8310_real_data.status & (1U << IST8310_DATA_READY_BIT)) == !(failed || no_new_data));
    assert(twoKi == 0.0f); /* 两条参考入口不共享非零积分历史。 */
    float six_axis[4], nine_axis[4];
    memcpy(six_axis, last_quat, sizeof(six_axis));
    memcpy(nine_axis, last_quat, sizeof(nine_axis));
    float dt = (float)(last_bmi_us - previous_fusion_us) * 1e-6f;
    MahonyAHRSupdateIMU(six_axis, 0,0,0, 0,0,9.8f, dt);
    MahonyAHRSupdate(nine_axis, 0,0,0, 0,0,9.8f,
                     expected_field[0], expected_field[1], expected_field[2], dt);
    float separation = 0.0f;
    for (unsigned i = 0U; i < 4U; ++i)
    {
        assert(fabsf(INS_quat[i] - (failed ? six_axis[i] : nine_axis[i])) < 0.000002f);
        separation += fabsf(six_axis[i] - nine_axis[i]);
    }
    assert(separation > 0.0001f); /* 非零磁偏差保证错误分支不会偶然得到同一结果。 */
    previous_fusion_us = last_bmi_us;
}

/** @brief 按共同微秒时钟推进RTOS毫秒等待。 @param[in] ms 等待毫秒数。 */
void test_delay_ms(uint32_t ms)
{
    assert(critical_depth == 0U);
    clock_us += (uint64_t)ms * 1000U;
    test_tick = (uint32_t)(clock_us / 1000U);
}

/** @brief TIM2读取返回同一时钟低32位，1us步进使微秒忙等可完成。
 * @param[in] h TIM2句柄。 @return 模拟计数，允许自然回绕。
 */
uint32_t test_timer_counter(TIM_HandleTypeDef *h)
{
    assert(h == &htim2);
    uint32_t value = (uint32_t)clock_us + timer_offset;
    ++clock_us;
    test_tick = (uint32_t)(clock_us / 1000U);
    return value;
}

/** @brief 核对TIM10 CH1启动时加热已关闭。 @param[in] h TIM10。 @param[in] channel CH1。 @return HAL_OK。 */
HAL_StatusTypeDef HAL_TIM_PWM_Start(TIM_HandleTypeDef *h, uint32_t channel)
{
    assert(h == &htim10 && channel == TIM_CHANNEL_1);
    assert(htim10.Compare[0] == 0U);
    ++pwm_starts;
    return HAL_OK;
}

/** @brief 观察真实温控输出，检查范围、冷加热及错误立即关热。
 * @param[in] h TIM10。 @param[in] channel CH1。 @param[in] value 加热比较值。
 */
void test_timer_compare(TIM_HandleTypeDef *h, uint32_t channel, uint32_t value)
{
    assert(h == &htim10 && channel == TIM_CHANNEL_1 && value <= 4500U);
    h->Compare[0] = value;
    if (!task_mode || accel_reads == 0U) return;
    last_controlled_read = accel_reads;
    ++heater_updates;
    if (accel_reads <= 10U)
    {
        assert(value > 0U);
        ++cold_heating;
    }
    if (!isfinite(bmi088_real_data.temp) || bmi088_real_data.temp < -40.0f || bmi088_real_data.temp >= 50.0f)
        assert(value == 0U);
}

/** @brief 记录真INS输出，核对校准完成、单位和真实采样时间下的积分角度。
 * @param[in] roll 横滚角，度。 @param[in] pitch 俯仰角，度。 @param[in] yaw 航向角，度。
 */
void vofa_send(float roll, float pitch, float yaw)
{
    const float rad_to_deg = 180.0f / 3.14159265358979323846f;
    assert(isfinite(roll) && isfinite(pitch) && isfinite(yaw));
    assert(fabsf(roll - INS_angle[2] * rad_to_deg) < 0.0001f);
    assert(fabsf(pitch - INS_angle[1] * rad_to_deg) < 0.0001f);
    assert(fabsf(yaw - INS_angle[0] * rad_to_deg) < 0.0001f);
    INS_control_data_t snapshot;
    assert(critical_depth == 0U && INS_Get_Data(&snapshot) == 1U);
    assert(fabsf(snapshot.yaw_deg - yaw) < 0.0001f);
    assert(snapshot.sample_time_ms == (uint32_t)(last_bmi_us / 1000U));
    float expected_world_z = scenario == DYNAMIC_WRAP || (scenario == TILTED_WORLD && vofa_calls > 0U)
                                 ? 1000.0f * 0.0010652644f : 0.0f;
    assert(fabsf(snapshot.world_z_rad_s - expected_world_z) < 0.00002f);
    assert(sampling_waits == calibration_samples);
    assert(accel_reads == calibration_samples + 2U + vofa_calls + runtime_skips);
    assert(mag_reads == (INS_USE_MAGNETOMETER ? calibration_samples + 2U * (vofa_calls + 1U + runtime_skips) : 0U));
    if (scenario == COLD_START && vofa_calls == 0U)
    {
        assert(bmi088_real_data.temp == 25.0f && htim10.Compare[0] > 0U);
        assert(clock_us - boot_us < 1000000U); /* 包含所选传感器复位和 48×10 ms 的模拟任务时间。 */
    }
    if (scenario == DYNAMIC_WRAP)
    {
        /* 使用归一化无关的四元数角度，独立核对动态积分。 */
        float angle = atan2f(2.0f * (INS_quat[0] * INS_quat[3] + INS_quat[1] * INS_quat[2]),
                            INS_quat[0]*INS_quat[0] + INS_quat[1]*INS_quat[1] -
                            INS_quat[2]*INS_quat[2] - INS_quat[3]*INS_quat[3]);
        float expected = EXPECTED_LEVEL_YAW_DEG / rad_to_deg +
                         1000.0f * 0.0010652644f * (float)(last_bmi_us - baseline_us) * 1e-6f;
        assert(fabsf(angle - expected) < 0.0001f);
        assert(fabsf(roll) < 0.001f && fabsf(pitch) < 0.001f);
        if (vofa_calls == 0U)
            assert(last_bmi_us - baseline_us > 10000U && last_bmi_us - baseline_us < 15000U);
        if ((uint32_t)(last_bmi_us + timer_offset) < baseline_counter)
            wrapped = 1U;
    }
    else if (scenario == TILTED_AVERAGE)
        assert_averaged_gravity();
    else if (scenario == MAGNETIC_READS)
        assert_magnetic_update();
    else if (scenario != TILTED_WORLD)
    {
        /* 原始gyro有非零偏置，校准后静止姿态不能继续漂移。 */
        assert(fabsf(roll) < 0.001f && fabsf(pitch) < 0.001f);
        assert(fabsf(yaw - EXPECTED_LEVEL_YAW_DEG) < 0.001f);
    }
    memcpy(last_quat, INS_quat, sizeof(last_quat));
    if (scenario == TILTED_WORLD && vofa_calls == 0U)
    {
        /* 构造90度横滚：机体Y角速度应成为世界Z，不能直接发布芯片/机体gyro[2]。 */
        INS_quat[0] = INS_quat[1] = 0.7071067811865475f;
        INS_quat[2] = INS_quat[3] = 0.0f;
    }
    if (++vofa_calls == output_limit)
        longjmp(task_finished, 1);
}

/** @brief 写入有符号传感器原始字节。 @param[out] data 2字节输出。 @param[in] value 原始值。 */
static void raw16(uint8_t *data, int16_t value)
{
    data[0] = (uint8_t)value;
    data[1] = (uint16_t)value >> 8;
}

/** @brief 按当前采样编号提供可重复的校准/运动输入。 */
static void task_sample(void)
{
    uint32_t end_calibration = calibration_samples;
    uint32_t cali_index = accel_reads;
    uint32_t run_index = accel_reads > end_calibration + 1U ? accel_reads - end_calibration - 1U : 0U;
    if (accel_reads > 1U && accel_reads <= end_calibration)
        assert(last_controlled_read == accel_reads - 1U); /* 标定每次都必须温控。 */
    if (accel_reads <= end_calibration + 1U) assert(vofa_calls == 0U);

    int16_t gx = 4, gy = -5, gz = 6;
    if (scenario == MOVING_MEAN && cali_index != 0U && cali_index <= EXPECTED_CALI_SAMPLES) gx = 100;
    if (scenario == TILTED_AVERAGE && cali_index != 0U && cali_index <= EXPECTED_CALI_SAMPLES) gx = 100;
    if (scenario == MOVING_PEAK && cali_index == INTERRUPTED_CALI_SAMPLES) gx = 400;
    if (scenario == DYNAMIC_WRAP && accel_reads > end_calibration) gz += 1000;
    raw16(&accel_regs[0x12], 0); raw16(&accel_regs[0x14], 0); raw16(&accel_regs[0x16], 10922);
    if (scenario == TILTED_AVERAGE && cali_index > EXPECTED_CALI_SAMPLES && accel_reads != end_calibration + 1U)
    {
        /* 完整成功窗口含等量正负扰动；失败窗口重力方向不同，不能混进均值。
         * 标定后的单次baseline仍故意给竖直重力，启动姿态须坚持窗口平均。 */
        int noise = accel_reads <= end_calibration ? (cali_index % 2U ? 1 : -1) : 0;
        raw16(&accel_regs[0x12], (int16_t)(-5461 + 120*noise));
        raw16(&accel_regs[0x14], (int16_t)(3277 + 80*noise));
        raw16(&accel_regs[0x16], (int16_t)(8874 + 40*noise));
    }
    if (scenario == TILTED_WORLD && vofa_calls > 0U)
    {
        gx -= 1000; /* 安装矩阵[y,-x,z]使芯片-X成为机体+Y。 */
        raw16(&accel_regs[0x12], -10922); raw16(&accel_regs[0x16], 0);
    }
    raw16(&gyro_regs[2], gx); raw16(&gyro_regs[4], gy); raw16(&gyro_regs[6], gz);
    gyro_regs[0] = scenario == BAD_GYRO_ID && cali_index == INTERRUPTED_CALI_SAMPLES ? 0xFF : 0x0F;
    accel_regs[0] = scenario == RUNTIME_ERROR && run_index == 8U ? 0U : 0x1E;
    mag_regs[2] = 1;
    memset(&mag_regs[3], 0, 6);

    int16_t temperature_raw = accel_reads <= 10U || scenario == COLD_START ? 16 : 176;
    if (scenario == CALI_TEMP_INVALID && cali_index == INTERRUPTED_CALI_SAMPLES) temperature_raw = -1024;
    if (scenario == RUNTIME_ERROR && (run_index == 2U || run_index == 5U || run_index == 7U || run_index == 10U))
        temperature_raw = 168; /* 44℃先产生正加热。 */
    if (scenario == RUNTIME_ERROR && run_index == 6U)
    {
        assert(htim10.Compare[0] > 0U);
        temperature_raw = 256; /* 55℃必须切断上一轮的正加热输出。 */
    }
    if (scenario == RUNTIME_ERROR && run_index == 11U)
    {
        assert(htim10.Compare[0] > 0U);
        temperature_raw = -1024; /* 寄存器0x80/0x00解码为-105℃，不能当作低温继续加热。 */
    }
    if (scenario == RUNTIME_ERROR && run_index == 8U)
    {
        memset(&accel_regs[0x12], 0, 6);
        temperature_raw = 0; /* accel读回全0、gyro正常、HAL_OK；不能误判为23℃。 */
    }
    accel_regs[0x22] = (uint16_t)temperature_raw >> 3;
    accel_regs[0x23] = ((uint16_t)temperature_raw & 7U) << 5;
    if (scenario == RUNTIME_ERROR && run_index == 3U)
    {
        inject_transfer_error = 1U;
        failure_pending = 1U;
        spi_timeout_pending = 1U;
        error_vofa_count = vofa_calls;
        ++runtime_skips;
    }
    if (scenario == RUNTIME_ERROR && run_index == 8U)
    {
        assert(htim10.Compare[0] > 0U && gyro_regs[0] == 0x0F);
        failure_pending = 1U;
        spi_timeout_pending = 0U;
        error_vofa_count = vofa_calls;
        ++runtime_skips;
    }
    if (scenario == RUNTIME_ERROR && (run_index == 4U || run_index == 9U))
    {
        recovery_pending = 1U;
        ++runtime_skips; /* 恢复首样本只重建时间基准，不积分整段中断。 */
    }
    if (scenario == DYNAMIC_WRAP && accel_reads == end_calibration)
        timer_offset = UINT32_MAX - 7000U - (uint32_t)clock_us;
    if (accel_reads > end_calibration)
    {
        last_bmi_us = clock_us - 1U;
        if (accel_reads == end_calibration + 1U)
        {
            baseline_us = last_bmi_us;
            baseline_counter = (uint32_t)last_bmi_us + timer_offset;
        }
    }
}

/** @brief 为模拟传感器提供独立已知原始值。 */
static void sample_registers(void)
{
    const uint8_t accel[6] = {0xFF,0x7F,0x00,0x80,0x01,0x00};
    const uint8_t gyro[6] = {0xFF,0xFF,0xC8,0,0x38,0xFF};
    const uint8_t mag[7] = {1,0xFF,0x7F,0,0x80,0xFF,0xFF};
    memcpy(&accel_regs[0x12],accel,6);
    memcpy(&gyro_regs[2],gyro,6);
    memcpy(&mag_regs[2],mag,7);
    accel_regs[0x22] = 0xFF; accel_regs[0x23] = 0xE0; /* 11bit -1 → 22.875℃。 */
}

/** @brief 模拟BMI片选与IST复位引脚。 @param[in] port 端口。 @param[in] pin 引脚。 @param[in] state 电平。 */
void HAL_GPIO_WritePin(void *port, uint16_t pin, GPIO_PinState state)
{
    (void)pin;
    if (port == RSTN_IST8310_GPIO_Port && task_mode)
        assert(INS_USE_MAGNETOMETER); /* 六轴启动应完全独立于 IST 硬件。 */
    if (port == RSTN_IST8310_GPIO_Port && state == GPIO_PIN_RESET)
    { memset(mag_regs,0,sizeof(mag_regs)); mag_regs[0] = 0x10; sample_registers(); }
    uint8_t device = port == CS1_ACCEL_GPIO_Port ? 1U : (port == CS1_GYRO_GPIO_Port ? 2U : 0U);
    if (!device) return;
    if (state == GPIO_PIN_RESET) { assert(selected == 0U); selected = device; spi_position = 0U; }
    else if (selected == device) selected = 0U;
}

/** @brief 按SPI协议提供寄存器字节，独立检验加速度计dummy。
 * @param[in] spi 句柄。 @param[in] tx 发出字节。 @param[out] rx 收到字节。
 * @param[in] size 长度。 @param[in] timeout 超时。 @return HAL_OK。
 */
HAL_StatusTypeDef HAL_SPI_TransmitReceive(SPI_HandleTypeDef *spi, uint8_t *tx,
                                        uint8_t *rx, uint16_t size, uint32_t timeout)
{
    assert(spi == &hspi1 && selected != 0U && size == 1U && timeout == 2U);
    ++spi_calls;
    uint8_t *registers = selected == 1U ? accel_regs : gyro_regs;
    *rx = 0xEE;
    if (spi_position == 0U)
    {
        spi_reg = *tx & 0x7FU; spi_read = *tx & 0x80U;
        if (selected == 1U && spi_read && spi_reg == 0x12)
        {
            ++accel_reads;
            if (task_mode) task_sample();
        }
    }
    else if (spi_read)
    {
        if (!(selected == 1U && spi_position == 1U)) *rx = registers[spi_reg++];
    }
    else
    {
        registers[spi_reg] = *tx;
        if ((selected == 1U && spi_reg == 0x7E && *tx == 0xB6) ||
            (selected == 2U && spi_reg == 0x14 && *tx == 0xB6))
        {
            uint8_t id = registers[0]; memset(registers,0,128); registers[0] = id;
            sample_registers();
        }
    }
    if (inject_transfer_error)
    {
        assert(htim10.Compare[0] > 0U); /* 确认故障确实发生在加热中。 */
        inject_transfer_error = 0U;
        failed_spi_calls = spi_calls;
        test_delay_ms(2U);
        return HAL_TIMEOUT;
    }
    ++spi_position;
    return HAL_OK;
}

/** @brief 模拟IST寄存器读取，并核对7bit地址左移。
 * @param[in] i2c 句柄。 @param[in] address 设备地址。 @param[in] reg 起始寄存器。
 * @param[in] address_size 地址宽度。 @param[out] data 缓冲。 @param[in] length 字节数。
 * @param[in] timeout 超时。 @return 正常为HAL_OK，故障夹具返回HAL_ERROR/HAL_TIMEOUT。
 */
HAL_StatusTypeDef HAL_I2C_Mem_Read(I2C_HandleTypeDef *i2c, uint16_t address, uint16_t reg,
                                 uint16_t address_size, uint8_t *data, uint16_t length, uint32_t timeout)
{
    assert(!task_mode || INS_USE_MAGNETOMETER);
    assert(i2c == &hi2c3 && address == 0x1C && address_size == 1U); (void)timeout;
    assert(reg + length <= sizeof(mag_regs));
    if (reg == 2U && length == 7U)
    {
        ++mag_reads;
        if (next_i2c_status != HAL_OK)
        {
            HAL_StatusTypeDef result = next_i2c_status;
            next_i2c_status = HAL_OK;
            return result; /* 故障不写buffer，刻意留下上次STAT1=1的完整帧。 */
        }
        if (task_mode && !runtime_started)
        {
            mag_regs[2] = 1U;
            int16_t x = 250, y = 0, z = 90;
            if (scenario == START_MAG_RETRY && mag_reads <= EXPECTED_CALI_SAMPLES)
            {
                /* 混合 HAL 失败和无新数据，即使全局保留上次有效磁场也应重采。 */
                if (mag_reads % 2U) return HAL_ERROR;
                mag_regs[2] = 0U;
            }
            else if (scenario == START_MAG_RETRY && mag_reads <= 2U * EXPECTED_CALI_SAMPLES)
            {
                /* 经 A*(raw-b) 后 [-0.012161,0.001218,66.499664] µT，水平分量约万分之二。 */
                x = 4; y = -31; z = 266;
            }
            else if (scenario == MAGNETIC_READS)
            {
                /* 窗首、窗尾各一帧新样本，均值恰为默认向量；其间无新帧并保留旧缓存。 */
                if (mag_reads == 1U) { x = 230; y = -50; z = 20; }
                else if (mag_reads == EXPECTED_CALI_SAMPLES) { x = 270; y = 50; z = 160; }
                else if (mag_reads % 2U == 0U) return HAL_TIMEOUT;
                else { mag_regs[2] = 0U; x = -600; y = 700; z = -800; }
            }
            raw16(&mag_regs[3], x); raw16(&mag_regs[5], y); raw16(&mag_regs[7], z);
        }
        else if (task_mode && scenario != MAGNETIC_READS)
            return HAL_ERROR;
        else if (task_mode)
        {
            const uint32_t run_read = mag_reads - calibration_samples;
            if (run_read == 3U || run_read == 4U)
            {
                ++magnetic_failures;
                return run_read == 3U ? HAL_TIMEOUT : HAL_ERROR;
            }
            mag_regs[2] = run_read == 5U || run_read == 6U ? 0U : 1U;
            raw16(&mag_regs[3], mag_regs[2] ? 100 : -300);
            raw16(&mag_regs[5], mag_regs[2] ? (run_read <= 2U ? 100 : -150) : 500);
            raw16(&mag_regs[7], mag_regs[2] ? (run_read <= 2U ? 0 : 20) : 100);
        }
    }
    memcpy(data,&mag_regs[reg],length);
    return HAL_OK;
}

/** @brief 模拟IST寄存器写入。 @param[in] i2c 句柄。 @param[in] address 设备地址。
 * @param[in] reg 寄存器。 @param[in] address_size 地址宽度。 @param[in] data 数据。
 * @param[in] length 字节数。 @param[in] timeout 超时。 @return HAL_OK。
 */
HAL_StatusTypeDef HAL_I2C_Mem_Write(I2C_HandleTypeDef *i2c, uint16_t address, uint16_t reg,
                                  uint16_t address_size, uint8_t *data, uint16_t length, uint32_t timeout)
{
    assert(!task_mode || INS_USE_MAGNETOMETER);
    assert(i2c == &hi2c3 && address == 0x1C && address_size == 1U); (void)timeout;
    assert(reg + length <= sizeof(mag_regs)); memcpy(&mag_regs[reg],data,length);
    return HAL_OK;
}

/** @brief 校准10ms、运行5ms共用真实时间，并在动态场景加入唤醒抖动。
 * @param[in,out] wake 基准tick。 @param[in] ticks 周期tick。
 */
void vTaskDelayUntil(TickType_t *wake, TickType_t ticks)
{
    assert(critical_depth == 0U);
    if (!runtime_started || vofa_calls == 0U) assert_ins_not_ready();
    if (failure_pending && runtime_started)
    {
        assert_ins_not_ready();
        assert(isnan(bmi088_real_data.temp) && htim10.Compare[0] == 0U);
        assert(last_controlled_read == accel_reads);
        if (spi_timeout_pending) assert(spi_calls == failed_spi_calls);
        else assert(accel_regs[0] == 0U && gyro_regs[0] == 0x0F);
        assert(vofa_calls == error_vofa_count);
        assert(memcmp(INS_quat, last_quat, sizeof(last_quat)) == 0);
        failure_pending = 0U;
    }
    if (recovery_pending)
    {
        assert_ins_not_ready();
        assert(vofa_calls == error_vofa_count && memcmp(INS_quat, last_quat, sizeof(last_quat)) == 0);
        assert(htim10.Compare[0] == 0U); /* 无效温度清除积分，恢复到45℃不能沿用旧加热。 */
        recovery_pending = 0U;
    }
    *wake += ticks;
    uint64_t deadline = (uint64_t)*wake * 1000U;
    if (ticks == 10U)
    {
        assert(!runtime_started);
        assert(++sampling_waits <= calibration_samples);
    }
    else
    {
        assert(ticks == 5U);
        if (!runtime_started)
        {
            if (scenario == TILTED_AVERAGE)
            {
                assert(accel_reads == calibration_samples + 1U);
                assert(bmi088_real_data.accel[0] == 0.0f && bmi088_real_data.accel[1] == 0.0f);
                assert_averaged_gravity(); /* 在第一轮融合前检查，排除后来反馈纠正的掩盖。 */
                if (!INS_USE_MAGNETOMETER)
                {
                    const float *q = INS_quat;
                    float yaw = atan2f(2.0f*(q[0]*q[3] + q[1]*q[2]), 1.0f - 2.0f*(q[2]*q[2] + q[3]*q[3]));
                    assert(fabsf(yaw) < 0.00001f); /* 板子倾斜时六轴仍以开机朝向为零。 */
                }
                ++initial_gravity_checks;
            }
            if (scenario == MAGNETIC_READS || scenario == START_MAG_RETRY)
            {
                /* 第一轮融合前，水平静置时根据独立校准向量直接检查磁航向初值。 */
                float initial_yaw = 2.0f * atan2f(INS_quat[3], INS_quat[0]) * 57.295779513f;
                assert(fabsf(initial_yaw - EXPECTED_LEVEL_YAW_DEG) < 0.001f);
            }
            memcpy(last_quat, INS_quat, sizeof(last_quat));
            previous_fusion_us = baseline_us;
        }
        runtime_started = 1U;
        if (scenario == DYNAMIC_WRAP)
        {
            const uint32_t jitter[6] = {0,3000,0,7000,0,2000};
            deadline += jitter[loops % 6U];
        }
        ++loops;
    }
    if (deadline > clock_us) clock_us = deadline;
    test_tick = (uint32_t)(clock_us / 1000U);
}

/** @brief 启动一次真实INS任务，检查完整校准重试与之后的数据采集。 */
static void run_ins_case(void)
{
    task_mode = 1U;
    runtime_started = wrapped = failure_pending = recovery_pending = 0U;
    sampling_waits = accel_reads = mag_reads = loops = vofa_calls = 0U;
    runtime_skips = heater_updates = cold_heating = last_controlled_read = 0U;
    magnetic_failures = initial_gravity_checks = 0U;
    timer_offset = 0U;
    output_limit = scenario == DYNAMIC_WRAP ? 20U : (scenario == RUNTIME_ERROR ? 8U :
                   (scenario == MAGNETIC_READS ? 4U : 5U));
    boot_us = clock_us;
    /* 静止均值不合格或无可用磁航向须重采完整窗。运动、坏ID及无效温度在第16点中断。 */
    calibration_samples = scenario == START_MAG_RETRY ? 3U * EXPECTED_CALI_SAMPLES :
                          (scenario == MOVING_MEAN || scenario == TILTED_AVERAGE) ? 2U * EXPECTED_CALI_SAMPLES :
                          (scenario == MOVING_PEAK || scenario == BAD_GYRO_ID || scenario == CALI_TEMP_INVALID) ?
                          INTERRUPTED_CALI_SAMPLES + EXPECTED_CALI_SAMPLES : EXPECTED_CALI_SAMPLES;
    if (setjmp(task_finished) == 0) INS_task(NULL);
    assert(BMI088_error == 0U && ist8310_error == (INS_USE_MAGNETOMETER ? IST8310_NO_ERROR : IST8310_NO_SENSOR));
    assert(sampling_waits == calibration_samples);
    assert(loops == 2U * (output_limit + runtime_skips));
    assert(accel_reads == calibration_samples + 1U + output_limit + runtime_skips);
    assert(mag_reads == (INS_USE_MAGNETOMETER ? calibration_samples + 2U * (output_limit + runtime_skips) : 0U));
    assert(cold_heating == 10U && heater_updates >= calibration_samples);
    assert(fabsf(bmi088_real_data.gyro[0] - (scenario == TILTED_WORLD ? -996.0f : 4.0f) * 0.0010652644f) < 0.000001f);
    assert(fabsf(bmi088_real_data.gyro[1] + 5.0f * 0.0010652644f) < 0.000001f);
    assert(fabsf(bmi088_real_data.gyro[2] - (scenario == DYNAMIC_WRAP ? 1006.0f : 6.0f) * 0.0010652644f) < 0.000001f);
    if (scenario == DYNAMIC_WRAP) assert(wrapped);
    if (scenario == RUNTIME_ERROR) assert(runtime_skips == 4U);
    if (scenario == TILTED_AVERAGE) assert(initial_gravity_checks == 1U);
    if (scenario == MAGNETIC_READS) assert(magnetic_failures == 2U);
}

/** @brief 验证教程驱动初始化、物理量转换及任务采集周期。 @return 成功为0。 */
int main(void)
{
    assert_ins_not_ready();
    assert(INS_Get_Data(NULL) == 0U);
    accel_regs[0] = 0x1E; gyro_regs[0] = 0x0F; mag_regs[0] = 0x10;
    assert(BMI088_init() == 0U && selected == 0U);
    assert(accel_regs[0x7D] == 4 && accel_regs[0x40] == 0xAB && accel_regs[0x41] == 0);
    assert(gyro_regs[0x15] == 0x80 && gyro_regs[0x10] == 0x82);
    float gyro[3], accel[3], temperature;
    BMI088_read(gyro,accel,&temperature);
    assert(fabsf(gyro[0] + 0.0010652644f) < 0.000001f);
    assert(fabsf(gyro[1] - 0.21305289f) < 0.000001f && fabsf(gyro[2] + 0.21305289f) < 0.000001f);
    assert(fabsf(accel[0] - 29.40628f) < 0.001f && accel[1] < -29.4f && accel[2] > 0);
    assert(temperature == 22.875f);
    assert(ist8310_init() == 0U);
    assert(mag_regs[0x0B] == 8 && mag_regs[0x41] == 9 && mag_regs[0x42] == 0xC0 && mag_regs[0x0A] == 0x0B);
    ist8310_real_data_t mag = {0};
    uint8_t frame[7] = {1,0xFF,0x7F,0,0x80,0xFF,0xFF};
    ist8310_read_over(frame,&mag);
    assert(mag.status == (1U << IST8310_DATA_READY_BIT));
    assert(fabsf(mag.mag[0] - 9830.1f) < 0.01f && fabsf(mag.mag[1] + 9830.4f) < 0.01f);
    assert(fabsf(mag.mag[2] + 0.3f) < 0.00001f);
    frame[0] = 0; ist8310_read_over(frame,&mag);
    assert(mag.status == 0U && fabsf(mag.mag[2] + 0.3f) < 0.00001f);
    uint8_t received[7], saved[7];
    assert(ist8310_IIC_read_muli_reg(2U, received, sizeof(received)) == 1U);
    memcpy(saved, received, sizeof(saved));
    next_i2c_status = HAL_ERROR;
    assert(ist8310_IIC_read_muli_reg(2U, received, sizeof(received)) == 0U);
    assert(memcmp(received, saved, sizeof(received)) == 0);
    const unsigned last_case = INS_USE_MAGNETOMETER ? START_MAG_RETRY : TILTED_AVERAGE;
    for (scenario = STATIC_BIAS; scenario <= last_case; ++scenario)
        run_ins_case();
    assert(timer_start_count == last_case + 1U && pwm_starts == last_case + 1U && critical_entries > 100U && critical_depth == 0U);
    printf("PASS sensors: %u-axis, %u startup/runtime cases\n", INS_USE_MAGNETOMETER ? 9U : 6U, last_case + 1U);
    return 0;
}
