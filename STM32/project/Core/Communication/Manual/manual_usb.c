#include "usb_link_handler.h"
#include "manual_link.h"

void UsbLink_OnFrame(const LinkFrame *frame)
{
    if (frame->type == MANUAL_CAN_VELOCITY_ID)
        Manual_CAN_Receive(frame->data, frame->length);
}

void UsbLink_OnDisconnect(void)
{
    /* 手动控制任务根据最后一个速度帧的接收时间撤权。 */
}
