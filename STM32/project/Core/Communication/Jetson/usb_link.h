#ifndef USB_LINK_H
#define USB_LINK_H

#include <stdint.h>

/* STM32 作为 USB 设备，Jetson 作为主机。C 板座接 OTG FS（PA11 D-、PA12 D+、PA10 OTG ID），
 * 无 ULPI，全速 12 Mbit/s。不检测 VBUS。中断优先级 6，只把字节放入环；组帧在独立任务。
 * 返回值：0 已提交，1 忙或尚未枚举，2 长度非法。忙不能当成底盘故障。
 */

void UsbLink_Init(void);
void UsbLink_Task(void const *argument);
int Jetson_Link_Send(uint8_t type, const uint8_t *data, uint8_t length);

#endif
