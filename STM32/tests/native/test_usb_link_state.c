#include <assert.h>
#include <setjmp.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include "stm32f4xx_hal.h"
#include "jetson_link.h"

/* 只替代硬件寄存器和 HAL 调用；下方直接编译真实 USB 回调、任务和协议解析路径。 */
#define JETSON_AUTO_ENABLED 1
typedef struct { uint32_t PLLCFGR; } TestRcc;
typedef struct { uint32_t Pin, Mode, Pull, Speed, Alternate; } GPIO_InitTypeDef;
typedef struct {
    uint32_t dev_endpoints, ep0_mps, speed, dma_enable, phy_itface, Sof_enable;
    uint32_t low_power_enable, lpm_enable, battery_charging_enable;
    uint32_t vbus_sensing_enable, use_dedicated_ep1;
} PCD_InitTypeDef;
typedef struct { void *Instance; PCD_InitTypeDef Init; uint32_t Setup[3]; } PCD_HandleTypeDef;
static TestRcc test_rcc;
static uint32_t test_sysclk_source, test_sysclk_hz;
static HAL_StatusTypeDef test_init_status, test_start_status;
static unsigned init_calls, start_calls, irq_enables;
static HAL_StatusTypeDef test_tx_status = HAL_OK;
static uint32_t test_rx_count;
#define RCC (&test_rcc)
#define USB_OTG_FS ((void *)1)
#define GPIOA ((void *)2)
#define RCC_CFGR_SWS_PLL 2U
#define RCC_PLLCFGR_PLLQ 0x0F000000U
#define RCC_PLLCFGR_PLLQ_Pos 24U
#define RCC_PLLCFGR_PLLP 0x00030000U
#define RCC_PLLCFGR_PLLP_Pos 16U
#define GPIO_PIN_10 (1U << 10U)
#define GPIO_PIN_11 (1U << 11U)
#define GPIO_PIN_12 (1U << 12U)
#define GPIO_MODE_AF_PP 2U
#define GPIO_PULLUP 1U
#define GPIO_NOPULL 0U
#define GPIO_SPEED_FREQ_VERY_HIGH 3U
#define GPIO_AF10_OTG_FS 10U
#define OTG_FS_IRQn 67U
#define PCD_SPEED_FULL 1U
#define PCD_PHY_EMBEDDED 2U
#define EP_TYPE_CTRL 0U
#define EP_TYPE_BULK 2U
#define EP_TYPE_INTR 3U
#define __HAL_RCC_GET_SYSCLK_SOURCE() test_sysclk_source
#define __HAL_RCC_GPIOA_CLK_ENABLE() ((void)0)
#define __HAL_RCC_USB_OTG_FS_CLK_ENABLE() ((void)0)
#define HAL_RCC_GetSysClockFreq() test_sysclk_hz
#define HAL_GetUIDw0() 0x01234567U
#define HAL_GetUIDw1() 0x89ABCDEFU
#define HAL_GetUIDw2() 0xAABBCCDDU

/** @brief 接受未参与本测试的 HAL 参数并返回成功；不模拟硬件时序。 */
static HAL_StatusTypeDef Test_Hal_Ok(unsigned unused, ...)
{
    (void)unused;
    return HAL_OK;
}

/** @brief 注入发送忙/成功；回调和状态变化仍使用生产实现。 */
static HAL_StatusTypeDef Test_Transmit(PCD_HandleTypeDef *pcd, uint8_t ep, uint8_t *data, uint32_t length)
{
    (void)pcd; (void)ep; (void)data; (void)length;
    return test_tx_status;
}

/** @brief 为真实 OUT 回调提供已收字节数。 */
static uint32_t Test_Rx_Count(PCD_HandleTypeDef *pcd, uint8_t ep)
{
    (void)pcd; (void)ep;
    return test_rx_count;
}

#define HAL_PCD_EP_SetStall(...) Test_Hal_Ok(0U, __VA_ARGS__)
#define HAL_PCD_EP_Open(...) Test_Hal_Ok(0U, __VA_ARGS__)
#define HAL_PCD_EP_Receive(...) Test_Hal_Ok(0U, __VA_ARGS__)
#define HAL_PCD_EP_Transmit Test_Transmit
#define HAL_PCD_EP_GetRxCount Test_Rx_Count
#define HAL_PCD_SetAddress(...) Test_Hal_Ok(0U, __VA_ARGS__)
#define HAL_GPIO_Init(...) Test_Hal_Ok(0U, __VA_ARGS__)
#define HAL_NVIC_SetPriority(...) Test_Hal_Ok(0U, __VA_ARGS__)
#define HAL_NVIC_EnableIRQ(irq) (++irq_enables)
#define HAL_PCD_Init(pcd) (++init_calls, test_init_status)
#define HAL_PCDEx_SetRxFiFo(...) Test_Hal_Ok(0U, __VA_ARGS__)
#define HAL_PCDEx_SetTxFiFo(...) Test_Hal_Ok(0U, __VA_ARGS__)
#define HAL_PCD_Start(pcd) (++start_calls, test_start_status)
#define HAL_PCD_IRQHandler(...) Test_Hal_Ok(0U, __VA_ARGS__)
#define HAL_PCD_DevDisconnect(...) Test_Hal_Ok(0U, __VA_ARGS__)
#define HAL_PCD_DevConnect(...) Test_Hal_Ok(0U, __VA_ARGS__)

#include "../../project/Core/Communication/Common/usb_link.c"

uint32_t test_tick;
static unsigned critical_depth, critical_entries;
static jmp_buf task_finished;
static unsigned task_step;
static uint8_t task_frame[LINK_FRAME_MAX_BYTES], task_frame_length;

/** @brief 检查实际快照与任务的临界区成对使用。 */
void test_enter_critical(void)
{
    assert(critical_depth == 0U);
    ++critical_depth;
    ++critical_entries;
}

/** @brief 检查退出临界区时已经屏蔽过 USB 中断。 */
void test_exit_critical(void)
{
    assert(critical_depth == 1U);
    --critical_depth;
}

/** @brief 用真实 SET_CONFIGURATION 处理器设置或撤销设备配置。 */
static void Set_Configuration(uint8_t value)
{
    memset(hpcd.Setup, 0, sizeof(hpcd.Setup));
    uint8_t *setup = (uint8_t *)hpcd.Setup;
    setup[1] = 9U;
    setup[2] = value;
    HAL_PCD_SetupStageCallback(&hpcd);
}

/** @brief 读取真实连接快照，并验证复制进入了临界区。 */
static UsbLink_State_t State(void)
{
    UsbLink_State_t state;
    const unsigned before = critical_entries;
    UsbLink_Get_State(&state);
    assert(!critical_depth && critical_entries == before + 1U);
    return state;
}

/** @brief 读取真实协议层的周期流准入标记。 */
static uint8_t Periodic_Seen(void)
{
    Jetson_CAN_Data_t data;
    Jetson_CAN_Get_Data(&data);
    return data.periodic_received;
}

/** @brief 注入合法无目标周期帧与待处理字节，为断连清理准备旧会话数据。 */
static void Seed_Old_Session(void)
{
    const uint8_t absent[8] = {0U};
    Jetson_CAN_Receive(JETSON_CAN_TARGET_CENTER_ID, absent, sizeof(absent));
    UsbRxBuffer_Push(&rx_buffer, absent, sizeof(absent));
    assert(Periodic_Seen() && rx_buffer.head != rx_buffer.tail);
}

/** @brief 回调后连接须离线、代数递增、周期准入及残留字节均清空。 */
static void Check_Offline(uint32_t previous)
{
    const UsbLink_State_t state = State();
    assert(!state.connected && state.generation == previous + 1U);
    assert(!Periodic_Seen() && rx_buffer.head == rx_buffer.tail);
    uint8_t byte;
    assert(!UsbRxBuffer_Pop(&rx_buffer, &byte));
}

/** @brief 验证真实回调：忙不离线、挂起/恢复保留代数、复位/撤销配置/断开清会话。 */
static void Check_Callbacks(void)
{
    Jetson_CAN_Init();
    assert(!State().connected);
    const unsigned entries = critical_entries;
    UsbLink_Get_State(NULL);
    assert(critical_entries == entries);
    Set_Configuration(1U);
    assert(State().connected);
    const uint32_t online = State().generation;
    const uint8_t payload[1] = {1U};
    assert(Jetson_Link_Send(16U, payload, 1U) == 0);
    assert(Jetson_Link_Send(16U, payload, 1U) == 1); /* 正在发送。 */
    assert(State().connected && State().generation == online);
    HAL_PCD_DataInStageCallback(&hpcd, USB_EP_IN & 0x7FU);
    test_tx_status = HAL_BUSY;
    assert(Jetson_Link_Send(16U, payload, 1U) == 1); /* HAL 瞬时忙。 */
    assert(State().connected && State().generation == online);
    test_tx_status = HAL_OK;

    Seed_Old_Session();
    HAL_PCD_SuspendCallback(&hpcd);
    Check_Offline(online);
    assert(Jetson_Link_Send(16U, payload, 1U) == 1);
    test_rx_count = 3U;
    HAL_PCD_DataOutStageCallback(&hpcd, USB_EP_OUT);
    assert(rx_buffer.head == rx_buffer.tail); /* 离线后的迟到 OUT 不入环。 */
    HAL_PCD_ResumeCallback(&hpcd);
    assert(State().connected && State().generation == online + 1U && !Periodic_Seen());

    Seed_Old_Session();
    uint32_t previous = State().generation;
    HAL_PCD_ResetCallback(&hpcd);
    Check_Offline(previous);
    HAL_PCD_ResumeCallback(&hpcd);
    assert(!State().connected); /* 复位后必须重新配置，Resume 不能代替枚举。 */
    Set_Configuration(1U);
    assert(State().connected && !Periodic_Seen());

    Seed_Old_Session();
    previous = State().generation;
    Set_Configuration(0U);
    Check_Offline(previous);
    HAL_PCD_ResumeCallback(&hpcd);
    assert(!State().connected);
    Set_Configuration(1U);

    Seed_Old_Session();
    previous = State().generation;
    HAL_PCD_DisconnectCallback(&hpcd);
    Check_Offline(previous);
    Set_Configuration(1U); /* 在两次控制轮询之间重连，仍留下 generation 证据。 */
    assert(State().connected && State().generation == previous + 1U && !Periodic_Seen());

    Seed_Old_Session();
    previous = State().generation;
    Set_Configuration(1U); /* 重配同一配置也不继承旧周期流和半帧。 */
    assert(State().connected && State().generation == previous + 1U && !Periodic_Seen());
    assert(rx_buffer.head == rx_buffer.tail);
    link_state.generation = UINT32_MAX;
    HAL_PCD_SuspendCallback(&hpcd);
    assert(State().generation == 0U && !State().connected);
}

/** @brief 在真实 USB 任务的等待点注入断连前半帧、断连后余帧和新完整帧。 */
void test_delay_ms(uint32_t ms)
{
    test_tick += ms;
    if (ms == 200U)
    {
        Set_Configuration(1U);
        return;
    }
    assert(ms == 1U && !critical_depth);
    switch (task_step++)
    {
    case 0U:
        UsbRxBuffer_Push(&rx_buffer, task_frame, 5U);
        break;
    case 1U:
        assert(parser.count == 5U && !Periodic_Seen());
        HAL_PCD_SuspendCallback(&hpcd);
        HAL_PCD_ResumeCallback(&hpcd);
        UsbRxBuffer_Push(&rx_buffer, task_frame + 5U, task_frame_length - 5U);
        break;
    case 2U:
        assert(!Periodic_Seen()); /* 旧会话前缀不能和重连后的后缀拼帧。 */
        task_frame[task_frame_length - 1U] ^= 0x80U;
        UsbRxBuffer_Push(&rx_buffer, task_frame, task_frame_length);
        break;
    case 3U:
        assert(!Periodic_Seen()); /* 坏 CRC 不能续期。 */
        task_frame[task_frame_length - 1U] ^= 0x80U;
        UsbRxBuffer_Push(&rx_buffer, task_frame, task_frame_length);
        break;
    case 4U:
        assert(Periodic_Seen()); /* 新会话完整 valid=0 帧可恢复准入。 */
        longjmp(task_finished, 1);
    default:
        assert(0);
    }
}

/** @brief 仅当时钟和 PCD 启动都成功时允许创建接收任务并启用中断。 */
static void Check_Init(void)
{
    test_sysclk_hz = 168000000U;
    test_rcc.PLLCFGR = 7U << RCC_PLLCFGR_PLLQ_Pos;
    assert(!UsbLink_Init()); /* 还未切换到 PLL。 */
    test_sysclk_source = RCC_CFGR_SWS_PLL;
    test_rcc.PLLCFGR = 4U << RCC_PLLCFGR_PLLQ_Pos;
    assert(!UsbLink_Init()); /* 336 MHz / 4 = 84 MHz。 */
    test_rcc.PLLCFGR = 7U << RCC_PLLCFGR_PLLQ_Pos;
    ++test_sysclk_hz;
    assert(!UsbLink_Init()); /* 整除截断不能把错误频率伪装成 48 MHz。 */
    --test_sysclk_hz;
    assert(init_calls == 0U && start_calls == 0U && irq_enables == 0U);
    test_init_status = HAL_ERROR;
    assert(!UsbLink_Init());
    assert(init_calls == 1U && start_calls == 0U && irq_enables == 0U);
    test_init_status = HAL_OK;
    test_start_status = HAL_BUSY;
    assert(!UsbLink_Init());
    assert(init_calls == 2U && start_calls == 1U && irq_enables == 0U);
    test_start_status = HAL_OK;
    assert(UsbLink_Init());
    assert(init_calls == 3U && start_calls == 2U && irq_enables == 1U);
    assert(strcmp(serial_text, "0123456789ABCDEFAABBCCDD") == 0);
}

/** @brief 运行 USB 启动、状态与接收任务回归。 */
int main(void)
{
    Check_Init();
    Check_Callbacks();
    Jetson_CAN_Init();
    parser = (LinkParser){0};
    const uint8_t absent[8] = {0U};
    task_frame_length = LinkFrame_Encode(JETSON_CAN_TARGET_CENTER_ID, absent, sizeof(absent), task_frame);
    assert(task_frame_length == LINK_FRAME_MAX_BYTES);
    if (setjmp(task_finished) == 0)
        UsbLink_Task(NULL);
    assert(task_step == 5U);
    puts("PASS: USB session callbacks, short disconnect, stale RX and periodic stream");
    return 0;
}
