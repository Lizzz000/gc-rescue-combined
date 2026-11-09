/** @file vofa.h
 * @brief VOFA+ JustFloat 三通道姿态发送接口。
 * @note vofa_send 是连接官方协议与 STM32 HAL 的必要封装，并非教程原函数。
 * 协议来源：https://www.vofa.plus/docs/learning/dataengines/justfloat/
 */
#ifndef VOFA_H
#define VOFA_H

/**
 * @brief 通过 USART6 DMA 发送 roll、pitch、yaw 三个 JustFloat 通道。
 * @param[in] roll 横滚角，单位度，有限值。
 * @param[in] pitch 俯仰角，单位度，有限值。
 * @param[in] yaw 航向角，单位度，有限值。
 * @return 无；串口忙时跳过本帧。
 * @note USART6 和 DMA 由 CubeMX 在 RTOS 启动前初始化；由单个任务每 10 ms 调用，USART6 发送由本模块独占。
 */
void vofa_send(float roll, float pitch, float yaw);

#endif
