#include "usb_link.h"

#include "jetson_auto_config.h"
#include "jetson_can.h"
#include "link_frame.h"
#include "main.h"
#if !JETSON_AUTO_CAN_ENABLED
#include "manual_can.h"
#endif

#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "task.h"

#include <string.h>

/* PA11/PA12 是 OTG FS。不检测 VBUS，因为 PA9 没有接 VBUS。优先级 6，任务临界区罩得住。 */

#define USB_RX_RING 1024U
#define USB_EP_OUT 0x01U
#define USB_EP_IN 0x81U

static PCD_HandleTypeDef hpcd;
static uint8_t out_buffer[64];
static uint8_t ep0_buffer[128];
static uint8_t tx_packet[LINK_FRAME_MAX_BYTES];
static uint8_t line_coding[7] = {0x00, 0xC2, 0x01, 0x00, 0x00, 0x00, 0x08};
static char serial_text[25];
static uint8_t device_address;
static volatile uint8_t address_pending;
static volatile uint8_t configured;
static volatile uint8_t tx_busy;
static uint8_t rx_ring[USB_RX_RING];
static volatile uint16_t rx_head;
static volatile uint16_t rx_tail;
static LinkParser parser;

/* 设备描述符：ST 虚拟串口的 VID/PID，内核才会直接绑定 cdc_acm。序列号用芯片 UID。 */
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

static void rx_push(const uint8_t *data, uint32_t length)
{
    for (uint32_t i = 0U; i < length; ++i)
    {
        const uint16_t next = (uint16_t)((rx_head + 1U) % USB_RX_RING);
        if (next == rx_tail)
            return;
        rx_ring[rx_head] = data[i];
        rx_head = next;
    }
}

static uint8_t rx_pop(uint8_t *byte)
{
    if (rx_head == rx_tail)
        return 0U;
    *byte = rx_ring[rx_tail];
    rx_tail = (uint16_t)((rx_tail + 1U) % USB_RX_RING);
    return 1U;
}

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
    if (data != ep0_buffer)
        memcpy(ep0_buffer, data, size);
    HAL_PCD_EP_Transmit(&hpcd, 0x00U, ep0_buffer, size);
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
            HAL_PCD_EP_Receive(&hpcd, 0x00U, line_coding, 7U);
            return;
        }
        if (request == 0x21U)
        {
            const uint16_t size = length < 7U ? length : 7U;
            memcpy(ep0_buffer, line_coding, size);
            HAL_PCD_EP_Transmit(&hpcd, 0x00U, ep0_buffer, size);
            return;
        }
        if (request == 0x22U)
        {
            HAL_PCD_EP_Transmit(&hpcd, 0x00U, NULL, 0U);
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
        device_address = (uint8_t)(value & 0x7FU);
        address_pending = 1U;
        HAL_PCD_EP_Transmit(&hpcd, 0x00U, NULL, 0U);
        return;
    }
    if (request == 9U)
    {
        if (value != 0U)
            open_data_endpoints();
        else
            configured = 0U;
        HAL_PCD_EP_Transmit(&hpcd, 0x00U, NULL, 0U);
        return;
    }
    if (request == 0U && (type & 0x80U) != 0U)
    {
        ep0_buffer[0] = 0U;
        ep0_buffer[1] = 0U;
        HAL_PCD_EP_Transmit(&hpcd, 0x00U, ep0_buffer, 2U);
        return;
    }
    stall_ep0();
}

void HAL_PCD_ResetCallback(PCD_HandleTypeDef *pcd)
{
    (void)pcd;
    configured = 0U;
    address_pending = 0U;
    tx_busy = 0U;
    HAL_PCD_EP_Open(&hpcd, 0x00U, 64U, EP_TYPE_CTRL);
    HAL_PCD_EP_Open(&hpcd, 0x80U, 64U, EP_TYPE_CTRL);
}

void HAL_PCD_SetupStageCallback(PCD_HandleTypeDef *pcd)
{
    (void)pcd;
    handle_setup();
}

void HAL_PCD_DataInStageCallback(PCD_HandleTypeDef *pcd, uint8_t epnum)
{
    (void)pcd;
    if (epnum == 0U && address_pending)
    {
        HAL_PCD_SetAddress(&hpcd, device_address);
        address_pending = 0U;
        return;
    }
    if (epnum == (USB_EP_IN & 0x7FU))
        tx_busy = 0U;
}

void HAL_PCD_DataOutStageCallback(PCD_HandleTypeDef *pcd, uint8_t epnum)
{
    if (epnum == 0U)
    {
        HAL_PCD_EP_Transmit(pcd, 0x00U, NULL, 0U);
        return;
    }
    if (epnum == USB_EP_OUT)
    {
        const uint32_t count = HAL_PCD_EP_GetRxCount(pcd, USB_EP_OUT);
        rx_push(out_buffer, count);
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

void HAL_PCD_MspInit(PCD_HandleTypeDef *pcd)
{
    if (pcd->Instance != USB_OTG_FS)
        return;
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_USB_OTG_FS_CLK_ENABLE();
    GPIO_InitTypeDef pin = {0};
    pin.Pin = GPIO_PIN_11 | GPIO_PIN_12;
    pin.Mode = GPIO_MODE_AF_PP;
    pin.Pull = GPIO_NOPULL;
    pin.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    pin.Alternate = GPIO_AF10_OTG_FS;
    HAL_GPIO_Init(GPIOA, &pin);
    HAL_NVIC_SetPriority(OTG_FS_IRQn, 6, 0);
    HAL_NVIC_EnableIRQ(OTG_FS_IRQn);
}

void UsbLink_Init(void)
{
    read_uid();
    hpcd.Instance = USB_OTG_FS;
    hpcd.Init = (PCD_InitTypeDef){0};
    hpcd.Init.dev_endpoints = 4U;
    hpcd.Init.ep0_mps = 64U;
    hpcd.Init.speed = PCD_SPEED_FULL;
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
    HAL_PCD_Start(&hpcd);
}

void UsbLink_Irq(void)
{
    HAL_PCD_IRQHandler(&hpcd);
}

int Jetson_Link_Send(uint16_t id, const uint8_t *data, uint8_t length)
{
    if (!configured || tx_busy)
        return 1;
    const uint8_t bytes = LinkFrame_Encode(id, data, length, tx_packet);
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
    for (;;)
    {
        uint8_t byte;
        while (rx_pop(&byte))
        {
            LinkFrame frame;
            if (!LinkParser_Feed(&parser, byte, &frame))
                continue;
            taskENTER_CRITICAL();
#if JETSON_AUTO_CAN_ENABLED
            Jetson_CAN_Receive(frame.id, frame.data, frame.length);
#else
            if (frame.id == MANUAL_CAN_VELOCITY_ID)
                Manual_CAN_Receive(frame.data, frame.length);
#endif
            taskEXIT_CRITICAL();
        }
        osDelay(1);
    }
}
