#include "usb_link_handler.h"
#include "manual_link.h"

void UsbLink_OnFrame(const LinkFrame *frame, uint32_t received_ms)
{
    if (frame->type == MANUAL_CAN_VELOCITY_ID)
        Manual_CAN_Receive_At(frame->data, frame->length, received_ms);
}

void UsbLink_OnDisconnect(void)
{
    Manual_CAN_Disconnect();
}
