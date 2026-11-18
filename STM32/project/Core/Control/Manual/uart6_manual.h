#ifndef UART6_MANUAL_H
#define UART6_MANUAL_H

/* USART6 串口遥控由现有 5 ms 底盘任务独占运行。 */
void UART6_Manual_Init(void);
void UART6_Manual_Loop(void);

#endif
