#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "bsp_can.h"
#include "dm_motor.h"
#include "task.h"

CAN_HandleTypeDef hcan1, hcan2;
uint32_t test_tick;
extern volatile uint32_t can_rx_diag_seq, can_rx_diag[8][4];
static CAN_FilterTypeDef captured_filter;
static CAN_RxHeaderTypeDef incoming;
static uint8_t payload[8];
static unsigned pending, decoded, receives;

HAL_StatusTypeDef HAL_CAN_ConfigFilter(CAN_HandleTypeDef *can, CAN_FilterTypeDef *filter)
{ assert(can == &hcan1); captured_filter = *filter; return HAL_OK; }
HAL_StatusTypeDef HAL_CAN_Start(CAN_HandleTypeDef *can)
{ assert(can == &hcan1); return HAL_OK; }
HAL_StatusTypeDef HAL_CAN_ActivateNotification(CAN_HandleTypeDef *can, uint32_t bits)
{ assert(can == &hcan1 && bits == CAN_IT_RX_FIFO0_MSG_PENDING); return HAL_OK; }
uint32_t HAL_CAN_GetRxFifoFillLevel(const CAN_HandleTypeDef *can, uint32_t fifo)
{ assert(can == &hcan1 && fifo == CAN_RX_FIFO0); return pending; }
HAL_StatusTypeDef HAL_CAN_GetRxMessage(CAN_HandleTypeDef *can, uint32_t fifo,
                                     CAN_RxHeaderTypeDef *header, uint8_t *data)
{
    assert(can == &hcan1 && fifo == CAN_RX_FIFO0 && pending);
    *header = incoming;
    memcpy(data, payload, 8U);
    --pending;
    ++receives;
    return HAL_OK;
}
void DM_Motor_ReceiveFeedback(const uint8_t data[8])
{ assert(incoming.StdId == 0U && incoming.DLC == 8U && data[0] == 3U); ++decoded; }

static void deliver(void)
{ pending = 1U; HAL_CAN_RxFifo0MsgPendingCallback(&hcan1); assert(!pending); }

int main(void)
{
    can_filter_init();
    assert(captured_filter.FilterMode == CAN_FILTERMODE_IDMASK);
    assert(captured_filter.FilterScale == CAN_FILTERSCALE_16BIT);
    assert(captured_filter.FilterIdLow == 0U && captured_filter.FilterIdHigh == 0U);
    assert(captured_filter.FilterMaskIdLow == 0x18U && captured_filter.FilterMaskIdHigh == 0x18U);
    /* 两个16位mask槽都只接收任意ID的标准数据帧。 */
    for (unsigned id = 0U; id <= 0x7FFU; ++id)
    {
        const uint32_t frame = id << 5U;
        assert((frame & captured_filter.FilterMaskIdLow) == 0U);
        assert(((frame | (1U << 4U)) & captured_filter.FilterMaskIdLow) != 0U);
        assert(((frame | (1U << 3U)) & captured_filter.FilterMaskIdHigh) != 0U);
    }
    incoming.DLC = 8U;
    const uint8_t data[8] = {3U, 0x12U, 0x34U, 0x56U, 0x78U, 0x9AU, 0xBCU, 0xDEU};
    memcpy(payload, data, 8U);
    for (unsigned n = 0U; n < 10U; ++n)
    {
        incoming.StdId = n == 0U ? 0U : 0x100U + n;
        test_tick = 100U + n;
        deliver();
        assert(can_rx_diag_seq == n + 1U);
        assert(can_rx_diag[n & 7U][0] == incoming.StdId);
        assert(can_rx_diag[n & 7U][1] == 0x56341203U);
        assert(can_rx_diag[n & 7U][2] == 0xDEBC9A78U);
        assert(can_rx_diag[n & 7U][3] == test_tick);
    }
    assert(decoded == 1U && can_rx_diag[0][0] == 0x108U && can_rx_diag[1][0] == 0x109U);
    /* 即使测试绕过硬件，原始记录在软件筛选前，非法帧不会交给DM解码。 */
    incoming.StdId = 0U; incoming.IDE = CAN_ID_EXT; deliver();
    incoming.IDE = CAN_ID_STD; incoming.RTR = CAN_RTR_REMOTE; deliver();
    incoming.RTR = CAN_RTR_DATA; incoming.DLC = 7U; deliver();
    assert(decoded == 1U && can_rx_diag_seq == 13U);
    can_rx_diag_seq = UINT32_MAX;
    incoming.DLC = 8U; incoming.StdId = 3U; deliver();
    assert(can_rx_diag_seq == 0U && can_rx_diag[7][0] == 3U && decoded == 1U);
    const unsigned before = receives;
    HAL_CAN_RxFifo0MsgPendingCallback(&hcan2);
    assert(receives == before);
    puts("PASS CAN diagnostics: standard-data mask, raw ring/wrap and unchanged ID0 decode");
    return 0;
}
