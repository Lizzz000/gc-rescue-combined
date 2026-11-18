#include "usb_link_handler.h"
#include "manual_link.h"

void UsbLink_OnFrame(const LinkFrame *frame)
{
    if (frame->type == MANUAL_CAN_VELOCITY_ID)
        Manual_CAN_Receive(frame->data, frame->length);
}

void UsbLink_OnDisconnect(void)
{
    Manual_CAN_Disconnect();
}
