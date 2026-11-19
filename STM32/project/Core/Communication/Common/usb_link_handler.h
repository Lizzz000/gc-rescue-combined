#ifndef USB_LINK_HANDLER_H
#define USB_LINK_HANDLER_H

#include "link_frame.h"

/* 选定的固件模式提供这两个回调。收帧在 USB 任务临界区内同步完成。 */
void UsbLink_OnFrame(const LinkFrame *frame, uint32_t received_ms);

/* USB 中断或屏蔽该中断的任务临界区调用；不得阻塞或调度底盘。 */
void UsbLink_OnDisconnect(void);

#endif
