#ifndef TEST_TASK_H
#define TEST_TASK_H

/** 板卡供电电压，V；使用名义 VDDA=3.3 V 换算，ADC 读取失败时为 NaN。 */
extern volatile float battery_voltage;

/**
 * @brief 沿用教程18的test_task入口，初始化电机 CAN1，按5ms调度手动或自动控制、底盘、提示和测压。
 * @param[in] argument 未使用的RTOS参数。
 * @return 不返回；CAN1中断只处理电机反馈，板间协议由USB任务接收，底盘默认失能。
 * @note Chassis_Loop内部10ms控制；动作接口均在本任务调用，不从中断调用。
 *       蜂鸣器保持关闭；LED每500ms翻转，电压每100ms采样，无新增RTOS任务。
 *       手动控制在底盘计算前调度，自动流程在底盘计算后调度，不新增CAN发送源。
 */
void test_task(void const *argument);

#endif
