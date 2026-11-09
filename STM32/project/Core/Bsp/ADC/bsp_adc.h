/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2019 RoboMaster.
 * 来源：官方 7.ADC_24V_power/bsp/boards/bsp_adc.c。
 */
#ifndef BSP_ADC_H
#define BSP_ADC_H

/**
 * @brief 单次读取 ADC3_IN8，并按板载分压计算电源电压。
 * @return 电压，单位 V，名义量程 0~33.3 V；启动、等待或停止失败返回 NAN。
 * @note 沿用教程 get_battery_voltage，fp32 改用等价 float。
 *       使用名义 VDDA=3.3 V、12 位 ADC 和 200 kΩ/22 kΩ 分压；
 *       未启用依赖 ADC1 的内部基准校准，精度需上板用实测电压核对。
 *       保留 CubeMX 的 ADC3_CH8、480 周期配置，轮询超时参数为 1 ms。
 *       由单一任务使用 ADC3，不与其他 ADC3 采集并发调用。
 */
float get_battery_voltage(void);

#endif
