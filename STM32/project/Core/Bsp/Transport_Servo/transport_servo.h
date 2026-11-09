#ifndef TRANSPORT_SERVO_H
#define TRANSPORT_SERVO_H

#include "jetson_auto_config.h"
#include <stdint.h>

typedef enum
{
    TRANSPORT_SERVO_OPEN,
    TRANSPORT_SERVO_CLOSED
} Transport_Servo_Position_t;

/**
 * @brief 检查 CubeMX 生成的 TIM1_CH1 PWM 配置。
 * @return 配置可用返回 1，否则返回 0。
 * @note 从底盘任务调用，此函数不启动 PWM。
 */
uint8_t Transport_Servo_Is_Available(void);

/**
 * @brief 非阻塞提交开或合指令，首次成功调用时启动 PWM。
 * @param[in] position 目标位置，使用 TRANSPORT_SERVO_OPEN 或 TRANSPORT_SERVO_CLOSED。
 * @param[in] now_ms 当前任务的单调毫秒计数，允许 uint32_t 回绕。
 * @return 指令已提交返回 1，配置无效或启动失败返回 0。
 * @note 仅从底盘任务调用。同位置重复指令不重新计时。
 */
uint8_t Transport_Servo_Command(Transport_Servo_Position_t position, uint32_t now_ms);

/**
 * @brief 读取上次成功切换目标位置后的时间。
 * @param[in] now_ms 与提交命令使用同一时基的当前毫秒计数。
 * @param[out] elapsed_ms 非空时写入经过的毫秒数。
 * @return 有已提交的目标且输出指针非空时返回 1，否则返回 0。
 * @note 计时值不代表舵机实际到位。
 */
uint8_t Transport_Servo_Elapsed(uint32_t now_ms, uint32_t *elapsed_ms);

#if !JETSON_AUTO_ENABLED
/* 调试器写入的高电平脉宽，单位 μs；0 停止 PWM。 */
extern volatile uint16_t jetson_calibration_pulse_us;

/**
 * @brief 在手动模式处理一次舵机标定输入。
 * @param[in] safe_to_move 底盘确认可调整时为 1，其他情况拒绝新非零脉宽。
 * @return 无；拒绝或设置失败时恢复上次已接受的脉宽。
 * @note 只在底盘任务调用。0 停止输出，手动标定时应在车辆静止下操作。
 */
void Transport_Servo_Calibration_Loop(uint8_t safe_to_move);
#endif

#endif
