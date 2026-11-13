#include <assert.h>
#include <math.h>
#include <setjmp.h>
#include <stdio.h>
#include <string.h>
#include "bsp_can.h"
#include "dm_motor.h"
#include "chassis_task.h"
#include "task.h"
#include "tim.h"
#include "bsp_adc.h"
#include "bsp_led.h"
#include "bsp_buzzer.h"
#include "jetson_auto.h"
#include "jetson_link.h"
#include "manual_link.h"
#include "manual_control.h"

/* 板间帧改走 USB。两种编译模式的 CAN 过滤器都只留电机反馈。 */
#define TEST_FILTER_BANKS 1U

static CAN_TypeDef can1_regs, can2_regs;
CAN_HandleTypeDef hcan1 = { &can1_regs };
CAN_HandleTypeDef hcan2 = { &can2_regs };
uint32_t test_tick;
TIM_HandleTypeDef htim4;
static uint32_t pwm_starts, buzzer_starts, buzzer_stops, led_toggles, battery_reads;
static CAN_FilterTypeDef filters[TEST_FILTER_BANKS];
static uint32_t filter_count, failed_filter_bank;
static CAN_TxHeaderTypeDef tx_header;
static CAN_RxHeaderTypeDef rx_header;
static uint8_t tx_data[8], rx_data[8];
static struct
{
    CAN_RxHeaderTypeDef header;
    uint8_t data[8];
} rx_frames[16];
static uint32_t rx_head, rx_tail, rx_pending;
static uint32_t jetson_rx_count;
static uint16_t jetson_rx_id;
static uint8_t jetson_rx_length, jetson_rx_data[8];
static uint32_t manual_rx_count;
static uint8_t manual_rx_length, manual_rx_data[8];
static uint32_t tx_count, rx_count, loops, setup_steps;
static uint32_t tx_mailboxes_free = 3U;
static uint32_t chassis_inits, chassis_loops;
static uint32_t jetson_can_inits, manual_can_inits, manual_control_inits, manual_control_loops;
static uint32_t jetson_auto_inits, jetson_auto_loops;
static HAL_StatusTypeDef start_result = HAL_OK;
static HAL_StatusTypeDef tx_result = HAL_OK;
static HAL_StatusTypeDef rx_result = HAL_OK;
static jmp_buf task_finished;

static const uint8_t uplink_data[8] = {0x01U, 0xEFU, 0xCDU, 0xABU, 0x89U};

/** @brief 用已知编码核对 BSP 对类型号、token 与 8 字节缓冲的传递。 */
uint8_t Jetson_CAN_Encode_Uplink(uint16_t type, uint32_t argument, uint8_t data[8])
{
    if (type != JETSON_CAN_TURN_COMPLETED_ID)
        return 0U;
    assert(argument == 0x89ABCDEFU);
    memcpy(data, uplink_data, sizeof(uplink_data));
    return 5U;
}

/** @brief 捕获共用入口交给 Jetson 的帧，确认未混入电机报文。
 * @param[in] type 类型号。 @param[in] data 数据。 @param[in] length 字节数。 @return 无。
 */
void Jetson_CAN_Receive(uint16_t type, const uint8_t *data, uint8_t length)
{
    assert(length <= 8U);
    jetson_rx_id = type;
    jetson_rx_length = length;
    memset(jetson_rx_data, 0, sizeof(jetson_rx_data));
    memcpy(jetson_rx_data, data, length);
    ++jetson_rx_count;
}

/** @brief 捕获共用入口交给手动模块的类型 2。
 * @param[in] data 数据。 @param[in] length 字节数。 @return 无。
 */
void Manual_CAN_Receive(const uint8_t *data, uint8_t length)
{
    assert(length <= 8U);
    manual_rx_length = length;
    memset(manual_rx_data, 0, sizeof(manual_rx_data));
    memcpy(manual_rx_data, data, length);
    ++manual_rx_count;
}

/** @brief 把当前测试帧放入模拟 FIFO，可连续放入多帧验证同次回调分流。
 * @return 无。
 */
static void Queue_Current_Frame(void)
{
    assert(rx_pending < 16U);
    rx_frames[rx_tail].header = rx_header;
    memcpy(rx_frames[rx_tail].data, rx_data, sizeof(rx_data));
    rx_tail = (rx_tail + 1U) % 16U;
    ++rx_pending;
}

/** @brief 注入一帧并调用真实接收回调。 @return 无。 */
static void Receive_Current_Frame(void)
{
    assert(rx_pending == 0U);
    Queue_Current_Frame();
    HAL_CAN_RxFifo0MsgPendingCallback(&hcan1);
}

/** @brief 核对底盘与通信状态先初始化，再开启 CAN 接收。 @return 无。 */
void Chassis_Init(void) { assert(setup_steps == 0U && test_tick == 0U); ++chassis_inits; }
/** @brief 核对协议初始化先于 CAN 中断。 @return 无。 */
void Jetson_CAN_Init(void)
{ assert(chassis_inits == 1U && setup_steps == 0U); ++jetson_can_inits; }
/** @brief 核对手动快照初始化先于 CAN 中断。 @return 无。 */
void Manual_CAN_Init(void)
{ assert(chassis_inits == 1U && setup_steps == 0U); ++manual_can_inits; }
/** @brief 核对控制初始化先于 CAN 中断。 @return 无。 */
void Manual_Control_Init(void)
{ assert(manual_can_inits == 1U && setup_steps == 0U); ++manual_control_inits; }
/** @brief 核对通信控制在同一任务且先于底盘周期执行。 @return 无。 */
void Manual_Control_Loop(void)
{
    assert(manual_control_inits == 1U && test_tick == 5U * (manual_control_loops + 1U));
    assert(chassis_loops == manual_control_loops);
    ++manual_control_loops;
}
/** @brief 核对原任务每5ms调用一次底盘调度；动作算法另由chassis suite编译验证。
 * @return 无。
 */
void Chassis_Loop(void)
{
    assert(test_tick == 5U * (chassis_loops + 1U));
#if JETSON_AUTO_CAN_ENABLED
    assert(jetson_auto_loops == chassis_loops);
#else
    assert(manual_control_loops == chassis_loops + 1U);
#endif
    ++chassis_loops;
}

/** @brief 核对自动流程初始化在协议快照之后、CAN 中断之前。 @return 无。 */
void Jetson_Auto_Init(void)
{ assert(jetson_can_inits == 1U && setup_steps == 0U); ++jetson_auto_inits; }
/** @brief 核对自动流程在底盘周期之后执行。 @return 无。 */
void Jetson_Auto_Loop(void)
{
    assert(jetson_auto_inits == 1U && chassis_loops == jetson_auto_loops + 1U);
    ++jetson_auto_loops;
}

/** @brief 捕获不应发生的蜂鸣PWM启动。 @param[in] h TIM4句柄。 @param[in] channel CH3。 @return HAL_OK。 */
HAL_StatusTypeDef HAL_TIM_PWM_Start(TIM_HandleTypeDef *h, uint32_t channel)
{ assert(h == &htim4 && channel == TIM_CHANNEL_3); ++pwm_starts; return HAL_OK; }
/** @brief 捕获不应发生的鸣叫调用。 @param[in] psc 分频值。 @param[in] pwm 比较值。 */
void buzzer_on(uint16_t psc, uint16_t pwm)
{ assert(psc == 83U && pwm == 125U); ++buzzer_starts; }
/** @brief 记录任务启动时关闭蜂鸣器。 */
void buzzer_off(void) { assert(test_tick == 0U); ++buzzer_stops; }
/** @brief 每500ms翻转一次三灯。 */
void bsp_led_toggle(void) { assert(test_tick % 500U == 0U); ++led_toggles; }
/** @brief 每100ms返回新电压。 @return 模拟电压V。 */
float get_battery_voltage(void)
{ assert(test_tick % 100U == 0U); ++battery_reads; return 24.0f + 0.01f * battery_reads; }

/** @brief 捕获 CAN1 滤波配置。 @param[in] hcan 句柄。 @param[in] filter 配置。
 * @return 指定滤波组失败时返回 HAL_ERROR，其余返回 HAL_OK。
 */
HAL_StatusTypeDef HAL_CAN_ConfigFilter(CAN_HandleTypeDef *hcan, CAN_FilterTypeDef *filter)
{
    assert(hcan == &hcan1 && filter_count < TEST_FILTER_BANKS && filter->FilterBank == filter_count);
    filters[filter_count++] = *filter;
    setup_steps |= 1U;
    return filter->FilterBank == failed_filter_bank ? HAL_ERROR : HAL_OK;
}
/** @brief 模拟启动CAN1。 @param[in,out] hcan 句柄。 @return HAL_OK。 */
HAL_StatusTypeDef HAL_CAN_Start(CAN_HandleTypeDef *hcan)
{
    assert(hcan == &hcan1 && filter_count == TEST_FILTER_BANKS && setup_steps == 1U);
    setup_steps |= 2U;
    return start_result;
}
/** @brief 核对FIFO0通知。 @param[in] hcan 句柄。 @param[in] notifications 中断位。 @return HAL_OK。 */
HAL_StatusTypeDef HAL_CAN_ActivateNotification(CAN_HandleTypeDef *hcan, uint32_t notifications)
{ assert(hcan == &hcan1 && notifications == CAN_IT_RX_FIFO0_MSG_PENDING && (setup_steps & 2U)); setup_steps |= 4U; return HAL_OK; }
/** @brief 捕获真实驱动发出的报文。 @param[in] hcan 句柄。 @param[in] header 帧头。
 * @param[in] data 8字节数据。 @param[out] mailbox 邮箱位。 @return HAL_OK。 */
HAL_StatusTypeDef HAL_CAN_AddTxMessage(CAN_HandleTypeDef *hcan, CAN_TxHeaderTypeDef *header,
                                     uint8_t *data, uint32_t *mailbox)
{
    assert(hcan == &hcan1); tx_header = *header; memcpy(tx_data,data,8);
    *mailbox = 1U; ++tx_count; return tx_result;
}

/** @brief 提供共享 CAN1 发送邮箱的模拟空闲数量。 */
uint32_t HAL_CAN_GetTxMailboxesFreeLevel(CAN_HandleTypeDef *hcan)
{
    assert(hcan == &hcan1);
    return tx_mailboxes_free;
}

/** @brief 返回模拟 FIFO 中的待收帧数。
 * @param[in] hcan CAN1。 @param[in] fifo FIFO0。 @return 待收帧数。
 */
uint32_t HAL_CAN_GetRxFifoFillLevel(const CAN_HandleTypeDef *hcan, uint32_t fifo)
{
    assert(hcan == &hcan1 && fifo == CAN_RX_FIFO0);
    return rx_pending;
}
/** @brief 向实际接收回调提供 FIFO 队首帧。 @param[in] hcan 句柄。 @param[in] fifo FIFO0。
 * @param[out] header 帧头。 @param[out] data 数据。 @return 测试指定的接收状态。 */
HAL_StatusTypeDef HAL_CAN_GetRxMessage(CAN_HandleTypeDef *hcan, uint32_t fifo,
                                     CAN_RxHeaderTypeDef *header, uint8_t *data)
{
    assert(hcan == &hcan1 && fifo == CAN_RX_FIFO0 && rx_pending != 0U);
    ++rx_count;
    if (rx_result != HAL_OK)
        return rx_result;
    *header = rx_frames[rx_head].header;
    memcpy(data, rx_frames[rx_head].data, 8);
    rx_head = (rx_head + 1U) % 16U;
    --rx_pending;
    return HAL_OK;
}
/** @brief 测试中初始化失败即失败。 */
void Error_Handler(void) { assert(0); }
/** @brief 执行1秒任务后退出。 @param[in,out] wake tick基准。 @param[in] ticks 周期。 */
void vTaskDelayUntil(TickType_t *wake, TickType_t ticks)
{
    assert(ticks == 5U); *wake += ticks; test_tick = *wake;
    if (++loops == 201U) longjmp(task_finished,1);
}

static HAL_StatusTypeDef link_result = HAL_OK;
static uint16_t link_id;
static uint8_t link_length, link_data[8];
static uint32_t link_count;

/** @brief 固件里的强符号在 USB 文件。本测试只记录上行有没有离开 CAN 邮箱。 */
int Jetson_Link_Send(uint8_t type, const uint8_t *data, uint8_t length)
{
    link_id = type;
    link_length = length;
    memcpy(link_data, data, length);
    ++link_count;
    if (link_result == HAL_OK)
        return 0;
    if (link_result == HAL_BUSY)
        return 1;
    return 2;
}

/** @brief 上行走 USB。未知类型不发送；忙和失败由链路返回，不占用 CAN 邮箱。 */
static void Check_Uplink_Submission(void)
{
    const uint32_t before = tx_count;
    const uint32_t links = link_count;
    assert(Jetson_CAN_Try_Send_Uplink(0U, 0U) == HAL_ERROR && tx_count == before);
    assert(link_count == links);
    link_result = HAL_BUSY;
    assert(Jetson_CAN_Try_Send_Uplink(JETSON_CAN_TURN_COMPLETED_ID, 0x89ABCDEFU) == HAL_BUSY && tx_count == before);
    link_result = HAL_ERROR;
    assert(Jetson_CAN_Try_Send_Uplink(JETSON_CAN_TURN_COMPLETED_ID, 0x89ABCDEFU) == HAL_ERROR && tx_count == before);
    link_result = HAL_OK;
    assert(Jetson_CAN_Try_Send_Uplink(JETSON_CAN_TURN_COMPLETED_ID, 0x89ABCDEFU) == HAL_OK && tx_count == before);
    assert(link_id == JETSON_CAN_TURN_COMPLETED_ID && link_length == 5U);
    assert(memcmp(link_data, uplink_data, 5) == 0);
}

/** @brief 验证手册MIT向量、反馈解码、CAN1筛选与任务无自动命令。 @return 成功为0。 */
int main(void)
{
    const uint8_t zero[8] = {0x7F,0xFF,0x7F,0xF0,0,0,7,0xFF};
    for (failed_filter_bank = 0U; failed_filter_bank < TEST_FILTER_BANKS; ++failed_filter_bank)
    {
        can_filter_init();
        assert(filter_count == failed_filter_bank + 1U && setup_steps == 1U);
        setup_steps = filter_count = 0U;
    }
    failed_filter_bank = UINT32_MAX;
    start_result = HAL_ERROR;
    can_filter_init();
    assert(filter_count == TEST_FILTER_BANKS && setup_steps == 3U);
    setup_steps = filter_count = 0U;
    start_result = HAL_OK;
    can_filter_init();
    assert(filter_count == TEST_FILTER_BANKS && setup_steps == 7U);
    const uint16_t accepted_ids[TEST_FILTER_BANKS][4] = {
        {0x000U, 0x000U, 0x000U, 0x000U},
    };
    for (unsigned bank = 0U; bank < TEST_FILTER_BANKS; ++bank)
    {
        const CAN_FilterTypeDef *filter = &filters[bank];
        assert(filter->FilterBank == bank && filter->SlaveStartFilterBank == 14U);
        assert(filter->FilterMode == CAN_FILTERMODE_IDLIST &&
               filter->FilterScale == CAN_FILTERSCALE_16BIT);
        assert(filter->FilterActivation == ENABLE && filter->FilterFIFOAssignment == CAN_FILTER_FIFO0);
        /* 四项都应是标准数据帧，16 位编码的低五位为零。 */
        assert(filter->FilterIdLow == ((uint32_t)accepted_ids[bank][0] << 5U));
        assert(filter->FilterMaskIdLow == ((uint32_t)accepted_ids[bank][1] << 5U));
        assert(filter->FilterIdHigh == ((uint32_t)accepted_ids[bank][2] << 5U));
        assert(filter->FilterMaskIdHigh == ((uint32_t)accepted_ids[bank][3] << 5U));
    }
    Check_Uplink_Submission();
    enable_motor(&hcan1,1);
    for (unsigned i = 0; i < 7; ++i) assert(tx_data[i] == 0xFF);
    assert(tx_data[7] == 0xFC && tx_header.StdId == 1 && tx_header.DLC == 8);
    assert(tx_header.IDE == CAN_ID_STD && tx_header.RTR == CAN_RTR_DATA);
    disable_motor(&hcan1,3); assert(tx_data[7] == 0xFD);
    ctrl_motor(&hcan1,2,0,0,0,0,0); assert(memcmp(tx_data,zero,8) == 0);
    ctrl_motor(&hcan1,1,-12.5f,-40,0,0,-1);
    for (unsigned i = 0; i < 8; ++i) assert(tx_data[i] == 0);
    ctrl_motor(&hcan1,3,12.5f,40,500,5,2);
    for (unsigned i = 0; i < 8; ++i) assert(tx_data[i] == 0xFF);
    assert(float_to_uint(0,-12.5f,12.5f,16) == 32767);
    assert(uint_to_float(0,-40,40,12) == -40 && uint_to_float(4095,-40,40,12) == 40);
    tx_result = HAL_BUSY;
    assert(ctrl_motor(&hcan1,1,0,0,0,0,0) == HAL_BUSY);
    assert(enable_motor(&hcan1,1) == HAL_BUSY && disable_motor(&hcan1,1) == HAL_BUSY);
    tx_result = HAL_OK;
    uint32_t previous_tx = tx_count;
    assert(enable_motor(&hcan2,1) == HAL_ERROR);
    assert(disable_motor(&hcan2,1) == HAL_ERROR);
    assert(ctrl_motor(&hcan2,1,0,0,0,0,0) == HAL_ERROR);
    ctrl_motor(&hcan1,0,0,0,0,0,0);
    assert(tx_count == previous_tx);
    rx_header.DLC = 8;
    const uint8_t low[8] = {0x11,0,0,0,0,0,25,26};
    test_tick = 12U;
    memcpy(rx_data,low,8); Receive_Current_Frame();
    assert(dm_motor[0].id == 1 && dm_motor[0].state == 1);
    assert(dm_motor[0].position == -12.5f && dm_motor[0].velocity == -40 && dm_motor[0].torque == -1);
    assert(dm_motor[0].t_mos == 25 && dm_motor[0].t_rotor == 26);
    const uint8_t high[8] = {0x83,0xFF,0xFF,0xFF,0xFF,0xFF,80,81};
    test_tick = 27U;
    memcpy(rx_data,high,8); Receive_Current_Frame();
    assert(dm_motor[2].id == 3 && dm_motor[2].state == 8 && dm_motor[2].torque == 2);
    assert(dm_motor[0].feedback_seq == 1U && dm_motor[1].feedback_seq == 0U && dm_motor[2].feedback_seq == 1U);
    assert(dm_motor[0].feedback_time_ms == 12U && dm_motor[1].feedback_time_ms == 0U && dm_motor[2].feedback_time_ms == 27U);
    test_tick = 40U;
    uint32_t previous_rx = rx_count;
    HAL_CAN_RxFifo0MsgPendingCallback(&hcan2); assert(rx_count == previous_rx);
    rx_data[0] = 0x01; rx_header.StdId = 1;
    Receive_Current_Frame(); assert(dm_motor[0].state == 1);
    rx_header.StdId = 0; rx_header.DLC = 7;
    Receive_Current_Frame(); assert(dm_motor[0].state == 1);
    rx_header.DLC = 8; rx_header.IDE = CAN_ID_EXT;
    Receive_Current_Frame(); assert(dm_motor[0].state == 1);
    rx_header.IDE = CAN_ID_STD; rx_header.RTR = CAN_RTR_REMOTE;
    Receive_Current_Frame(); assert(dm_motor[0].state == 1);
    rx_header.RTR = CAN_RTR_DATA; rx_result = HAL_ERROR;
    Receive_Current_Frame();
    assert(rx_pending == 1U);
    rx_head = rx_tail; rx_pending = 0U;
    rx_result = HAL_OK; rx_data[0] = 0x10;
    Receive_Current_Frame();
    rx_data[0] = 0x14;
    Receive_Current_Frame();
    assert(dm_motor[0].feedback_seq == 1U && dm_motor[1].feedback_seq == 0U && dm_motor[2].feedback_seq == 1U);
    assert(dm_motor[0].feedback_time_ms == 12U && dm_motor[1].feedback_time_ms == 0U && dm_motor[2].feedback_time_ms == 27U);
    dm_motor[1].feedback_seq = UINT32_MAX;
    test_tick = UINT32_MAX;
    rx_data[0] = 0x02;
    Receive_Current_Frame();
    assert(dm_motor[1].id == 2 && dm_motor[1].state == 0 && dm_motor[1].feedback_seq == 0U);
    assert(dm_motor[1].feedback_time_ms == UINT32_MAX);
    test_tick = 0U;
    Receive_Current_Frame();
    assert(dm_motor[0].feedback_seq == 1U && dm_motor[1].feedback_seq == 1U && dm_motor[2].feedback_seq == 1U);
    assert(dm_motor[0].feedback_time_ms == 12U && dm_motor[1].feedback_time_ms == 0U && dm_motor[2].feedback_time_ms == 27U);
    assert(jetson_rx_count == 0U && manual_rx_count == 0U);

    /* CAN 回调只接受 StdID 0、DLC 8 的电机反馈。板间帧不在这条总线上。 */
    test_tick = 51U;
    memcpy(rx_data, low, 8);
    Queue_Current_Frame();
    rx_header.StdId = 1U;
    rx_header.DLC = 8U;
    Queue_Current_Frame();
    rx_header.StdId = 0x100U;
    rx_header.DLC = 8U;
    Queue_Current_Frame();
    previous_rx = rx_count;
    HAL_CAN_RxFifo0MsgPendingCallback(&hcan1);
    assert(rx_pending == 0U && rx_count == previous_rx + 3U);
    assert(dm_motor[0].feedback_seq == 2U && dm_motor[0].feedback_time_ms == 51U);
    assert(jetson_rx_count == 0U && manual_rx_count == 0U);
    const uint32_t jetson_before = jetson_rx_count, manual_before = manual_rx_count;

    const uint16_t other_ids[] = {1U, 2U, 16U, 21U, 0x100U, 0x7FFU};
    for (unsigned i = 0U; i < sizeof(other_ids) / sizeof(other_ids[0]); ++i)
    {
        rx_header.StdId = other_ids[i];
        rx_header.DLC = 8U;
        Receive_Current_Frame();
        assert(jetson_rx_count == jetson_before && manual_rx_count == manual_before);
    }
    const uint32_t jetson_expected = jetson_before;
    const uint32_t manual_expected = manual_before;
    assert(jetson_rx_count == jetson_expected && manual_rx_count == manual_expected);
    assert(dm_motor[0].feedback_seq == 2U);

    rx_header.StdId = 0U;
    rx_header.DLC = 8U;
    rx_header.IDE = CAN_ID_EXT;
    Receive_Current_Frame();
    rx_header.IDE = CAN_ID_STD;
    rx_header.RTR = CAN_RTR_REMOTE;
    Receive_Current_Frame();
    rx_header.RTR = CAN_RTR_DATA;
    rx_header.DLC = 9U;
    Receive_Current_Frame();
    assert(jetson_rx_count == jetson_expected && manual_rx_count == manual_expected);
    assert(dm_motor[0].feedback_seq == 2U && dm_motor[1].feedback_seq == 1U && dm_motor[2].feedback_seq == 1U);
    DM_Motor_ReceiveFeedback(NULL);
    assert(dm_motor[0].feedback_seq == 2U);
    previous_rx = rx_count;
    HAL_CAN_RxFifo0MsgPendingCallback(&hcan1);
    assert(rx_count == previous_rx);

    setup_steps = filter_count = 0U; test_tick = 0U;
    if (setjmp(task_finished) == 0) test_task(NULL);
    assert(tx_count == previous_tx);
    assert(pwm_starts == 0U && buzzer_starts == 0U && buzzer_stops == 1U);
    assert(led_toggles == 2U && battery_reads == 11U);
    assert(chassis_inits == 1U && chassis_loops == 200U);
#if JETSON_AUTO_CAN_ENABLED
    assert(jetson_can_inits == 1U && jetson_auto_inits == 1U && jetson_auto_loops == 200U);
    assert(manual_can_inits == 0U && manual_control_inits == 0U && manual_control_loops == 0U);
#else
    assert(manual_can_inits == 1U && manual_control_inits == 1U && manual_control_loops == 200U);
    assert(jetson_can_inits == 0U && jetson_auto_inits == 0U && jetson_auto_loops == 0U);
#endif
    assert(fabsf(battery_voltage - 24.11f) < 0.0001f);
    puts(JETSON_AUTO_CAN_ENABLED ? "PASS CAN auto: motor-only filter, MIT feedback and 5ms task ordering"
                       : "PASS CAN manual: motor-only filter, MIT feedback and 5ms task ordering");
    return 0;
}
