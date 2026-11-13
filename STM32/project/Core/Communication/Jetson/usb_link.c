#include "usb_link.h"

#include "jetson_auto_config.h"
#include "jetson_link.h"
#include "link_frame.h"
#include "usb_rx_buffer.h"
#include "main.h"
#if !JETSON_AUTO_CAN_ENABLED
#include "manual_link.h"
#endif

#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "task.h"

#include <string.h>

/* C 板 USB 座接 OTG FS：PA11 为 D-，PA12 为 D+，PA10 为 OTG ID。没有 ULPI PHY。
 * 这条链路是全速 12 Mbit/s，不是高速 480 Mbit/s。不检测 VBUS。
 * ID 悬空时必须为高，核心才把本机当成设备。中断优先级 6，中断只把字节放入环；组帧在独立任务。
 */

#define USB_EP_OUT 0x01U
#define USB_EP_IN 0x81U

static PCD_HandleTypeDef hpcd;
static uint8_t out_buffer[64];
static uint8_t ep0_buffer[128];
static uint8_t tx_packet[LINK_FRAME_MAX_BYTES];
static uint8_t line_coding[7] = {0x00, 0xC2, 0x01, 0x00, 0x00, 0x00, 0x08};
static char serial_text[25];
static volatile uint8_t configured;
static volatile uint8_t tx_busy;
/* 控制传输：设备发出数据后，主机还要回一个 0 长度状态包。状态包本身不能再回一个空包。 */
static uint8_t ep0_status_out;
static uint8_t ep0_line_coding;
static uint16_t ep0_in_sent;
static uint16_t ep0_in_total;
/* 只给在线查看。复位和 SETUP 次数用来判断主机有没有把枚举走完。 */
volatile uint32_t usb_dbg_reset;
volatile uint32_t usb_dbg_setup;
volatile uint8_t usb_dbg_last[8];
volatile uint8_t usb_dbg_log[16];
volatile uint8_t usb_dbg_log_i;
static UsbRxBuffer rx_buffer;
static LinkParser parser;

/* 设备描述符：ST 虚拟串口的 VID/PID，内核才会直接绑定 cdc_acm。序列号用芯片 UID。
 * bcdUSB 0x0200 只是规范版本。bMaxPacketSize0 为 64，按全速设备枚举，没有高速限定描述符。
 */
static const uint8_t device_descriptor[] = {
    18, 1, 0x00, 0x02, 0x02, 0x00, 0x00, 64,
    0x83, 0x04, 0x40, 0x57, 0x00, 0x01, 1, 2, 3, 1,
};

/* 配置描述符 67 字节：通信接口 + 数据接口。批量端点 64 字节。 */
static const uint8_t config_descriptor[] = {
    9, 2, 67, 0, 2, 1, 0, 0x80, 50,
    9, 4, 0, 0, 1, 0x02, 0x02, 0x01, 0,
    5, 0x24, 0x00, 0x10, 0x01,
    5, 0x24, 0x01, 0x00, 0x01,
    4, 0x24, 0x02, 0x02,
    5, 0x24, 0x06, 0x00, 0x01,
    7, 5, 0x82, 3, 8, 0, 16,
    9, 4, 1, 0, 2, 0x0A, 0x00, 0x00, 0,
    7, 5, USB_EP_OUT, 2, 64, 0, 0,
    7, 5, USB_EP_IN, 2, 64, 0, 0,
};

static void stall_ep0(void)
{
    HAL_PCD_EP_SetStall(&hpcd, 0x80U);
    HAL_PCD_EP_SetStall(&hpcd, 0x00U);
}

static void open_data_endpoints(void)
{
    HAL_PCD_EP_Open(&hpcd, 0x82U, 8U, EP_TYPE_INTR);
    HAL_PCD_EP_Open(&hpcd, USB_EP_IN, 64U, EP_TYPE_BULK);
    HAL_PCD_EP_Open(&hpcd, USB_EP_OUT, 64U, EP_TYPE_BULK);
    HAL_PCD_EP_Receive(&hpcd, USB_EP_OUT, out_buffer, sizeof(out_buffer));
    tx_busy = 0U;
    configured = 1U;
}

static uint8_t fill_string(uint8_t index)
{
    const char *text = NULL;
    if (index == 0U)
    {
        ep0_buffer[0] = 4U;
        ep0_buffer[1] = 3U;
        ep0_buffer[2] = 0x09U;
        ep0_buffer[3] = 0x04U;
        return 4U;
    }
    if (index == 1U)
        text = "GC Rescue";
    else if (index == 2U)
        text = "STM32 Link";
    else if (index == 3U)
        text = serial_text;
    if (text == NULL)
        return 0U;
    const uint8_t chars = (uint8_t)strlen(text);
    ep0_buffer[0] = (uint8_t)(2U + chars * 2U);
    ep0_buffer[1] = 3U;
    for (uint8_t i = 0U; i < chars; ++i)
    {
        ep0_buffer[2U + i * 2U] = (uint8_t)text[i];
        ep0_buffer[3U + i * 2U] = 0U;
    }
    return ep0_buffer[0];
}

/* EP0 一次最多 64 字节。配置描述符有 67 字节，主机按 255 来要，必须再发第二包。 */
static void ep0_send_chunk(void)
{
    uint16_t left = (uint16_t)(ep0_in_total - ep0_in_sent);
    uint16_t chunk = left;
    if (chunk > 64U)
        chunk = 64U;
    ep0_status_out = (chunk == left) ? 1U : 0U;
    HAL_PCD_EP_Transmit(&hpcd, 0x00U, &ep0_buffer[ep0_in_sent], chunk);
    ep0_in_sent = (uint16_t)(ep0_in_sent + chunk);
}

static void ep0_send(const uint8_t *data, uint16_t size)
{
    if (data != ep0_buffer)
        memcpy(ep0_buffer, data, size);
    ep0_in_sent = 0U;
    ep0_in_total = size;
    ep0_send_chunk();
}

static void ep0_send_zlp(void)
{
    ep0_in_sent = 0U;
    ep0_in_total = 0U;
    ep0_status_out = 0U;
    HAL_PCD_EP_Transmit(&hpcd, 0x00U, NULL, 0U);
}

static void send_descriptor(uint16_t value, uint16_t length)
{
    const uint8_t kind = (uint8_t)(value >> 8U);
    const uint8_t index = (uint8_t)value;
    const uint8_t *data = NULL;
    uint16_t size = 0U;
    if (kind == 1U)
    {
        data = device_descriptor;
        size = sizeof(device_descriptor);
    }
    else if (kind == 2U)
    {
        data = config_descriptor;
        size = sizeof(config_descriptor);
    }
    else if (kind == 3U)
    {
        size = fill_string(index);
        data = ep0_buffer;
    }
    if (data == NULL || size == 0U)
    {
        stall_ep0();
        return;
    }
    if (length < size)
        size = length;
    ep0_send(data, size);
}

static void handle_setup(void)
{
    const uint8_t *setup = (const uint8_t *)hpcd.Setup;
    const uint8_t type = setup[0];
    const uint8_t request = setup[1];
    const uint16_t value = (uint16_t)setup[2] | ((uint16_t)setup[3] << 8U);
    const uint16_t length = (uint16_t)setup[6] | ((uint16_t)setup[7] << 8U);

    if ((type & 0x60U) == 0x20U)
    {
        if (request == 0x20U && length == 7U)
        {
            ep0_line_coding = 1U;
            HAL_PCD_EP_Receive(&hpcd, 0x00U, line_coding, 7U);
            return;
        }
        if (request == 0x21U)
        {
            const uint16_t size = length < 7U ? length : 7U;
            ep0_send(line_coding, size);
            return;
        }
        if (request == 0x22U)
        {
            ep0_send_zlp();
            return;
        }
        stall_ep0();
        return;
    }

    if (request == 6U && (type & 0x80U) != 0U)
    {
        send_descriptor(value, length);
        return;
    }
    if (request == 5U)
    {
        /* 地址要在状态空包之前写进 DCFG。核心等这个空包结束才开始用新地址。 */
        HAL_PCD_SetAddress(&hpcd, (uint8_t)(value & 0x7FU));
        ep0_send_zlp();
        return;
    }
    if (request == 9U)
    {
        if (value != 0U)
            open_data_endpoints();
        else
            configured = 0U;
        ep0_send_zlp();
        return;
    }
    if (request == 0U && (type & 0x80U) != 0U)
    {
        ep0_buffer[0] = 0U;
        ep0_buffer[1] = 0U;
        ep0_send(ep0_buffer, 2U);
        return;
    }
    stall_ep0();
}

void HAL_PCD_ResetCallback(PCD_HandleTypeDef *pcd)
{
    (void)pcd;
    configured = 0U;
    ep0_status_out = 0U;
    ep0_line_coding = 0U;
    ep0_in_sent = 0U;
    ep0_in_total = 0U;
    tx_busy = 0U;
    UsbRxBuffer_Reset(&rx_buffer);
    usb_dbg_reset++;
    HAL_PCD_EP_Open(&hpcd, 0x00U, 64U, EP_TYPE_CTRL);
    HAL_PCD_EP_Open(&hpcd, 0x80U, 64U, EP_TYPE_CTRL);
}

void HAL_PCD_SetupStageCallback(PCD_HandleTypeDef *pcd)
{
    (void)pcd;
    const uint8_t *setup = (const uint8_t *)hpcd.Setup;
    for (uint8_t i = 0U; i < 8U; ++i)
        usb_dbg_last[i] = setup[i];
    usb_dbg_log[usb_dbg_log_i & 15U] = setup[1];
    usb_dbg_log_i++;
    usb_dbg_setup++;
    handle_setup();
}

void HAL_PCD_DataInStageCallback(PCD_HandleTypeDef *pcd, uint8_t epnum)
{
    if (epnum == 0U)
    {
        if (ep0_in_sent < ep0_in_total)
        {
            ep0_send_chunk();
            return;
        }
        /* 官方 USB 库在控制 IN 结束后先停掉 EP0 IN，再收主机的状态空包。
         * 只发了状态空包时，停掉 IN 是为了让下一次 SETUP 能进来。 */
        HAL_PCD_EP_SetStall(pcd, 0x80U);
        if (ep0_status_out)
        {
            ep0_status_out = 0U;
            HAL_PCD_EP_Receive(pcd, 0x00U, NULL, 0U);
        }
        return;
    }
    if (epnum == (USB_EP_IN & 0x7FU))
        tx_busy = 0U;
}

void HAL_PCD_DataOutStageCallback(PCD_HandleTypeDef *pcd, uint8_t epnum)
{
    if (epnum == 0U)
    {
        if (ep0_line_coding)
        {
            ep0_line_coding = 0U;
            ep0_send_zlp();
            return;
        }
        HAL_PCD_EP_SetStall(pcd, 0x00U);
        return;
    }
    if (epnum == USB_EP_OUT)
    {
        const uint32_t count = HAL_PCD_EP_GetRxCount(pcd, USB_EP_OUT);
        UsbRxBuffer_Push(&rx_buffer, out_buffer, count);
        HAL_PCD_EP_Receive(pcd, USB_EP_OUT, out_buffer, sizeof(out_buffer));
    }
}

static void read_uid(void)
{
    const uint32_t *uid = (const uint32_t *)0x1FFF7A10U;
    static const char hex[] = "0123456789ABCDEF";
    for (uint8_t word = 0U; word < 3U; ++word)
    {
        uint32_t value = uid[word];
        for (int nibble = 7; nibble >= 0; --nibble)
            serial_text[word * 8U + (uint8_t)(7 - nibble)] = hex[(value >> (nibble * 4)) & 0xFU];
    }
    serial_text[24] = '\0';
}

/* SystemClock_Config 的 PLLQ 保持 4。VCO 336 MHz 时 USB 时钟是 84 MHz，这里改成 7 得到 48 MHz。
 * 改 PLL 前先切到 HSI，改完再切回 PLL。SYSCLK 仍是 168 MHz。只由 UsbLink_Init 调用。
 */
static uint32_t usb_clock_hz(void)
{
    if (__HAL_RCC_GET_SYSCLK_SOURCE() != RCC_CFGR_SWS_PLL)
        return 0U;
    const uint32_t pllq = (RCC->PLLCFGR & RCC_PLLCFGR_PLLQ) >> RCC_PLLCFGR_PLLQ_Pos;
    const uint32_t pllp = (((RCC->PLLCFGR & RCC_PLLCFGR_PLLP) >> RCC_PLLCFGR_PLLP_Pos) + 1U) * 2U;
    if (pllq < 2U)
        return 0U;
    return (HAL_RCC_GetSysClockFreq() / pllq) * pllp;
}

static uint8_t wait_pll(FlagStatus ready)
{
    const uint32_t start = HAL_GetTick();
    while (__HAL_RCC_GET_FLAG(RCC_FLAG_PLLRDY) != ready)
    {
        if ((HAL_GetTick() - start) > 100U)
            return 0U;
    }
    return 1U;
}

static void usb_clock_48mhz(void)
{
    if (usb_clock_hz() == 48000000U)
        return;

    __HAL_RCC_HSI_ENABLE();
    const uint32_t start = HAL_GetTick();
    while (__HAL_RCC_GET_FLAG(RCC_FLAG_HSIRDY) == RESET)
    {
        if ((HAL_GetTick() - start) > 100U)
            return;
    }

    RCC_ClkInitTypeDef clk = {0};
    clk.ClockType = RCC_CLOCKTYPE_SYSCLK;
    clk.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
    if (HAL_RCC_ClockConfig(&clk, FLASH_LATENCY_5) != HAL_OK)
        return;

    __HAL_RCC_PLL_DISABLE();
    if (!wait_pll(RESET))
    {
        /* PLL 没关掉就停在 HSI 上。能重新打开就切回去，避免 CPU 留在 16 MHz。 */
        __HAL_RCC_PLL_ENABLE();
        if (wait_pll(SET))
        {
            clk.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
            HAL_RCC_ClockConfig(&clk, FLASH_LATENCY_5);
        }
        return;
    }
    MODIFY_REG(RCC->PLLCFGR, RCC_PLLCFGR_PLLQ, 7U << RCC_PLLCFGR_PLLQ_Pos);
    __HAL_RCC_PLL_ENABLE();
    if (!wait_pll(SET))
        return;

    clk.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    HAL_RCC_ClockConfig(&clk, FLASH_LATENCY_5);
}

void HAL_PCD_MspInit(PCD_HandleTypeDef *pcd)
{
    if (pcd->Instance != USB_OTG_FS)
        return;
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_USB_OTG_FS_CLK_ENABLE();
    GPIO_InitTypeDef pin = {0};
    /* PA10 是 OTG ID。电缆不把 ID 拉低时保持高电平，避免核心进入主机。D+/D- 不加内部上下拉。 */
    pin.Pin = GPIO_PIN_10;
    pin.Mode = GPIO_MODE_AF_PP;
    pin.Pull = GPIO_PULLUP;
    pin.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    pin.Alternate = GPIO_AF10_OTG_FS;
    HAL_GPIO_Init(GPIOA, &pin);
    pin.Pin = GPIO_PIN_11 | GPIO_PIN_12;
    pin.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &pin);
    HAL_NVIC_SetPriority(OTG_FS_IRQn, 6, 0);
    HAL_NVIC_EnableIRQ(OTG_FS_IRQn);
}

void UsbLink_Init(void)
{
    usb_clock_48mhz();
    if (usb_clock_hz() != 48000000U)
        return;
    read_uid();
    hpcd.Instance = USB_OTG_FS;
    hpcd.Init = (PCD_InitTypeDef){0};
    hpcd.Init.dev_endpoints = 4U;
    hpcd.Init.ep0_mps = 64U;
    hpcd.Init.speed = PCD_SPEED_FULL; /* 全速 12 Mbit/s，不是高速 480 Mbit/s。 */
    hpcd.Init.dma_enable = DISABLE;
    hpcd.Init.phy_itface = PCD_PHY_EMBEDDED;
    hpcd.Init.Sof_enable = DISABLE;
    hpcd.Init.low_power_enable = DISABLE;
    hpcd.Init.lpm_enable = DISABLE;
    hpcd.Init.battery_charging_enable = DISABLE;
    hpcd.Init.vbus_sensing_enable = DISABLE;
    hpcd.Init.use_dedicated_ep1 = DISABLE;
    if (HAL_PCD_Init(&hpcd) != HAL_OK)
        return;
    /* 复位后的 RX FIFO 是 512 字，起点已经超出 OTG FS 实际的 320 字，描述符写不进去。
     * C 板官方底盘例程把 RX 设为 128、EP0 设为 64。这里还要给批量 IN 和中断 IN 各留一份。 */
    HAL_PCDEx_SetRxFiFo(&hpcd, 0x80);
    HAL_PCDEx_SetTxFiFo(&hpcd, 0, 0x40);
    HAL_PCDEx_SetTxFiFo(&hpcd, 1, 0x60);
    HAL_PCDEx_SetTxFiFo(&hpcd, 2, 0x20);
    HAL_PCD_Start(&hpcd);
}

void OTG_FS_IRQHandler(void)
{
    HAL_PCD_IRQHandler(&hpcd);
}

int Jetson_Link_Send(uint8_t type, const uint8_t *data, uint8_t length)
{
    if (!configured || tx_busy)
        return 1;
    const uint8_t bytes = LinkFrame_Encode(type, data, length, tx_packet);
    if (bytes == 0U)
        return 2;
    tx_busy = 1U;
    if (HAL_PCD_EP_Transmit(&hpcd, USB_EP_IN, tx_packet, bytes) != HAL_OK)
    {
        tx_busy = 0U;
        return 1;
    }
    return 0;
}

void UsbLink_Task(void const *argument)
{
    (void)argument;
    /* 主机可能还停在上一次描述符失败里。拉低 D+ 再放开，让它重新枚举。 */
    osDelay(200);
    HAL_PCD_DevDisconnect(&hpcd);
    osDelay(200);
    HAL_PCD_DevConnect(&hpcd);
    uint32_t parsed_generation = rx_buffer.generation;
    for (;;)
    {
        for (;;)
        {
            uint8_t byte;
            LinkFrame frame;
            taskENTER_CRITICAL();
            if (parsed_generation != rx_buffer.generation)
            {
                parser = (LinkParser){0};
                parsed_generation = rx_buffer.generation;
            }
            if (!UsbRxBuffer_Pop(&rx_buffer, &byte))
            {
                taskEXIT_CRITICAL();
                break;
            }
            if (!LinkParser_Feed(&parser, byte, &frame))
            {
                taskEXIT_CRITICAL();
                continue;
            }
#if JETSON_AUTO_CAN_ENABLED
            Jetson_CAN_Receive(frame.type, frame.data, frame.length);
#else
            if (frame.type == MANUAL_CAN_VELOCITY_ID)
                Manual_CAN_Receive(frame.data, frame.length);
#endif
            taskEXIT_CRITICAL();
        }
        osDelay(1);
    }
}
