#ifndef USB_LINK_H
#define USB_LINK_H

#include <stdint.h>

/* STM32 作为 USB 设备，Jetson 作为主机。C 板座接 OTG FS（PA11 D-、PA12 D+、PA10 OTG ID），
 * 无 ULPI，全速 12 Mbit/s。不检测 VBUS。中断优先级 6，只把字节放入环；组帧在独立任务。
 * 返回值：0 已提交，1 忙或尚未枚举，2 长度非法。忙不能当成底盘故障。
 */

typedef struct
{
    uint8_t connected; /**< 已枚举配置且总线未挂起；发送忙不改变此值。 */
    uint32_t generation; /**< 每次复位、挂起或断开递增，恢复连接不回退，按 uint32_t 回绕。 */
} UsbLink_State_t;

/**
 * @brief 在任务临界区读取连接状态及会话代数，捕捉两个控制周期之间的短暂断连。
 * @param[out] out 非空时写入快照；空指针不操作。
 * @return 无；无 VBUS 检测时，还需由业务检查既有周期帧的接收期限。
 */
void UsbLink_Get_State(UsbLink_State_t *out);

/** @return 1 表示 USB 外设已配置并保持断开，可创建接入总线并处理收发的任务。 */
uint8_t UsbLink_Init(void);
void UsbLink_Task(void const *argument);
int UsbLink_Send(uint8_t type, const uint8_t *data, uint8_t length);

#endif
