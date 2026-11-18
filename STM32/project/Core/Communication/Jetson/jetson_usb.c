#include "usb_link_handler.h"
#include "usb_link.h"
#include "jetson_link.h"

void UsbLink_OnFrame(const LinkFrame *frame)
{
    Jetson_CAN_Receive(frame->type, frame->data, frame->length);
}

void UsbLink_OnDisconnect(void)
{
    Jetson_CAN_Invalidate_Periodic();
}

int Jetson_Link_Send(uint8_t type, const uint8_t *data, uint8_t length)
{
    return UsbLink_Send(type, data, length);
}
