/** @file vofa.h
 * @brief VOFA+ JustFloat 姿态及现场测试快照发送接口。
 * @note vofa_send 是连接官方协议与 STM32 HAL 的必要封装，并非教程原函数。
 * 协议来源：https://www.vofa.plus/docs/learning/dataengines/justfloat/
 */
#ifndef VOFA_H
#define VOFA_H

#include <stdint.h>

#ifndef CHASSIS_TUNING_ENABLED
#define CHASSIS_TUNING_ENABLED 0
#endif

#define VOFA_MAX_CHANNELS 56U

/**
 * @brief 将有限数量的 float 通道复制到静态缓冲后通过 USART6 DMA 发送。
 * @param[in] values 通道数组；返回后调用方可复用该数组。
 * @param[in] count 通道数，范围 1 到 VOFA_MAX_CHANNELS。
 * @return 无；串口忙、空指针或通道数越界时跳过本帧。
 * @note 发送由单个任务独占；正常构建由 INS 发送姿态，现场测试构建由底盘
 *       任务发送快照，不能由两个任务同时调用。帧尾为 JustFloat 固定标记。
 */
void vofa_send_channels(const float *values, uint8_t count);

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
