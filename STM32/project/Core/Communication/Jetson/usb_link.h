#ifndef USB_LINK_H
#define USB_LINK_H

#include <stdint.h>

/* STM32 作为 USB 设备，Jetson 作为主机。中断只把字节放进环形缓冲，组帧在独立任务里。
 * 返回值：0 已提交，1 忙或尚未枚举，2 长度非法。忙不能当成底盘故障。
 */

void UsbLink_Init(void);
void UsbLink_Irq(void);
void UsbLink_Task(void const *argument);
int Jetson_Link_Send(uint16_t id, const uint8_t *data, uint8_t length);

#endif
