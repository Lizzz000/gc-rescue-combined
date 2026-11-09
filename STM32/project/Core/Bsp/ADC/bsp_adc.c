/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2019 RoboMaster.
 * 来源：https://github.com/RoboMaster/Development-Board-C-Examples/blob/master/7.ADC_24V_power/bsp/boards/bsp_adc.c
 * 保留 get_battery_voltage/adcx_get_chx_value 名称与电压换算。
 * 裁掉 ADC1 内部基准、芯片温度及硬件版本功能；不重复配置采样通道。
 */
#include "bsp_adc.h"
#include "adc.h"
#include <math.h>

/* 教程默认比例等于 3.3/4095；这里只代表名义 VDDA，并非实测基准。 */
static const float voltage_vrefint_proportion = 3.3f / 4095.0f;

/**
 * @brief 读取已配置的 ADC3 单通道，停止转换后才发布本次采样值。
 * @param[in,out] ADCx ADC3 句柄；通道和 480 周期采样时间由 CubeMX 配置。
 * @param[out] value 成功时写入 0~4095 的原始计数，失败时保持不变。
 * @return HAL_OK 表示采样和停止成功，否则返回相应 HAL 错误或超时状态。
 * @note 改编自教程同名函数：去掉动态通道重配参数，增加输出参数和状态返回，
 *       避免原函数在采样失败后仍读取旧值；所有路径均尝试停止 ADC。
 */
static HAL_StatusTypeDef adcx_get_chx_value(ADC_HandleTypeDef *ADCx, uint16_t *value)
{
    uint16_t sample = 0U;
    HAL_StatusTypeDef status = HAL_ADC_Start(ADCx);

    if (status == HAL_OK)
    {
        status = HAL_ADC_PollForConversion(ADCx, 1U);
    }
    if (status == HAL_OK)
    {
        sample = (uint16_t)HAL_ADC_GetValue(ADCx);
    }

    HAL_StatusTypeDef stop_status = HAL_ADC_Stop(ADCx);
    if (status != HAL_OK)
    {
        return status;
    }
    if (stop_status != HAL_OK)
    {
        return stop_status;
    }

    *value = sample;
    return HAL_OK;
}

/**
 * @brief 沿用教程按 200 kΩ/22 kΩ 分压换算板卡电源电压。
 * @return 名义 3.3 V 基准下的电源电压，单位 V；采集失败返回 NAN。
 * @note 来源：教程第 7 章 bsp_adc.c 的同名函数；保留分压换算，裁掉 ADC1 校准。
 */
float get_battery_voltage(void)
{
    uint16_t adcx = 0U;
    if (adcx_get_chx_value(&hadc3, &adcx) != HAL_OK)
    {
        return NAN;
    }

    return (float)adcx * voltage_vrefint_proportion * 10.090909090909090909f;
}
