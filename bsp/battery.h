/**
 * @file    battery.h
 * @brief   电池电压检测 — 基于 ESP32 ADC2
 *
 * 三步拆解术:
 *   1. 直观理解: 电池电压太高 (12V)，ESP32 的 ADC 只能测 0~3.3V，
 *      所以用两个电阻"分压"——好比用两个水桶把高水压降到安全范围，
 *      然后 ADC 只读小桶的水位，再按比例反算大桶。
 *   2. 追根溯源: 没有电池检测，就像开车没有油表——
 *      不知道还剩多少电，只能等突然断电才知道没电了。
 *   3. 正规解释: ESP32-S3 内置两路 12 位 SAR ADC，GPIO14 属于 ADC2_CH3。
 *      使用 ESP-IDF 的 adc_oneshot 模式 + 曲线拟合校准，消除 ADC 非线性误差。
 *
 * 硬件接线:
 *   V_bat → 40.2KΩ → GPIO14 → 10KΩ → GND
 *   分压比 = (40.2 + 10) / 10 = 5.02
 *   若 GPIO14 测得 2.46V，则电池电压 = 2.46 × 5.02 ≈ 12.35V
 *
 * 裸机使用 (推荐):
 *   @code
 *   battery_init();
 *   battery_update();               // 手动触发一次 ADC 读取
 *   float v = battery_get_voltage();
 *   @endcode
 *
 * 注意:
 *   - 裸机模式下需要周期性调用 battery_update()，不创建后台任务
 *   - ADC2 与 Wi-Fi 共用，Wi-Fi 开启时 ADC2 可能无法使用
 */

#pragma once
#ifdef __cplusplus
extern "C" {
#endif

#include "stdint.h"
#include "car_config.h"     /* PIN_BATTERY_ADC / BATTERY_DIVIDER_RATIO */

/** 电池电压检测引脚 (ADC2_CH3) — 定义在 car_config.h */
#define BATTERY_GPIO PIN_BATTERY_ADC

/**
 * @brief 初始化 ADC 模块 (不创建 FreeRTOS 任务)
 *
 * 配置 ADC2_CH3 为单次采样模式，启用曲线拟合校准。
 * 裸机模式下仅初始化硬件，不启动后台轮询。
 */
void battery_init(void);

/**
 * @brief 手动触发一次 ADC 读取 (裸机模式核心 API)
 *
 * 读取 ADC2_CH3 原始值 → 校准 → 换算为电池电压 → 缓存。
 * 后续调用 battery_get_voltage() 返回缓存值，不会重新触发 ADC。
 * 建议在主循环中每 500~1000ms 调用一次。
 */
void battery_update(void);

/**
 * @brief 内置阻塞测试循环 (会创建 FreeRTOS 任务，仅用于独立测试)
 */
void battery_test(void);

/**
 * @brief 获取最近一次更新的电池电压
 * @return 电池电压 (V)，如 12.46
 * @note   必须先调用 battery_update() 更新缓存，否则返回 0 或旧值
 */
float battery_get_voltage(void);

#ifdef __cplusplus
}
#endif
