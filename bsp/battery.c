/**
 * @file    battery.c
 * @brief   电池电压检测实现 — ADC1_CH1 单次采样 + 曲线拟合校准
 *
 * 工作流程:
 *   1. battery_init()    → 初始化 ADC2 单元 + 校准模块
 *   2. battery_update()  → 主循环中周期性调用, 读取 ADC → 校准 → 换算电压
 *   3. battery_get_voltage() → 取缓存值
 */

#include "battery.h"

#include "stdio.h"
#include "stdint.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_err.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali_scheme.h"
#include "sdkconfig.h"

/* ──── ADC 配置 ──── */
#define ADC_ATTEN_BATTERY   ADC_ATTEN_DB_12      /* 衰减 12dB, 量程 ~3.3V (ADC_ATTEN_DB_11 已弃用) */
#define ADC_CHANNEL_BATTERY ADC_CHANNEL_1         /* IO2 = ADC1_CH1 */

const static char *TAG = "BATTERY";

/* ──── 电压缓存 (static, 模块内共享) ──── */
static int   adc_raw         = 0;      /* ADC 原始读数 */
static int   cali_voltage    = 0;      /* 校准后电压 (mV) */
static float battery_voltage = 0.0f;   /* 换算后的电池电压 (V) */

/* ──── ADC 硬件句柄 ──── */
adc_oneshot_unit_handle_t battery_handle      = NULL;  /* ADC1 单元句柄 */
adc_cali_handle_t         battery_cali_handle = NULL;  /* 校准句柄 */

/* ================================================================
 * 校准初始化 — 使用 ESP-IDF 曲线拟合算法消除 ADC 非线性
 * ================================================================ */
static bool battery_adc_calibration_init(adc_unit_t unit, adc_channel_t channel,
                                          adc_atten_t atten, adc_cali_handle_t *out_handle)
{
    adc_cali_handle_t handle = NULL;
    esp_err_t ret = ESP_FAIL;
    bool calibrated = false;

    if (!calibrated) {
        ESP_LOGI(TAG, "校准方案: 曲线拟合 (Curve Fitting)");
        adc_cali_curve_fitting_config_t cali_config = {
            .unit_id  = unit,
            .chan     = channel,
            .atten    = atten,
            .bitwidth = ADC_BITWIDTH_DEFAULT,
        };
        ret = adc_cali_create_scheme_curve_fitting(&cali_config, &handle);
        if (ret == ESP_OK) {
            calibrated = true;
        }
    }

    *out_handle = handle;
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "ADC 校准成功");
    } else {
        ESP_LOGE(TAG, "ADC 校准失败 (参数错误或内存不足)");
    }
    return calibrated;
}

/* ──── 释放校准资源 ──── */
static void __attribute__((unused)) battery_adc_calibration_deinit(adc_cali_handle_t handle)
{
    ESP_ERROR_CHECK(adc_cali_delete_scheme_curve_fitting(handle));
}

/* ================================================================
 * 电压换算 — 分压比反算
 *
 * 电路: V_bat — 40.2KΩ — GPIO2 — 10KΩ — GND
 * V_gpio = V_bat × 10 / (40.2 + 10)
 * V_bat  = V_gpio × (40.2 + 10) / 10 = V_gpio × 5.02
 * 分压比在 car_config.h 中配置: BATTERY_DIVIDER_RATIO
 * ================================================================ */
static void battery_update_voltage(int gpio_voltage_mV)
{
    battery_voltage = gpio_voltage_mV / 1000.0f * BATTERY_DIVIDER_RATIO;
}

/* ================================================================
 * ADC 硬件初始化 — 创建 ADC2 单元 + 配置通道 + 启用校准
 * ================================================================ */
static void battery_adc_init(void)
{
    /* 创建 ADC2 单次采样单元 */
    adc_oneshot_unit_init_cfg_t init_config1 = {
        .unit_id = ADC_UNIT_1,                       /* IO2 属于 ADC1 */
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &battery_handle));

    /* 配置通道: 位宽 + 衰减 */
    adc_oneshot_chan_cfg_t config = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,            /* 默认 12 位 */
        .atten    = ADC_ATTEN_BATTERY,               /* 衰减 11dB, 量程 ~3.3V */
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(battery_handle, ADC_CHANNEL_BATTERY, &config));

    /* 启用曲线拟合校准 */
    battery_adc_calibration_init(ADC_UNIT_1, ADC_CHANNEL_BATTERY,
                                 ADC_ATTEN_BATTERY, &battery_cali_handle);
}

/* ================================================================
 * 公开 API
 * ================================================================ */

/* 初始化 (裸机模式) */
void battery_init(void)
{
    battery_adc_init();
    ESP_LOGI(TAG, "电池 ADC 初始化完成（裸机模式，需在主循环调用 battery_update()）");
}

/* 手动触发一次 ADC 读取 → 校准 → 换算电压 */
void battery_update(void)
{
    ESP_ERROR_CHECK(adc_oneshot_read(battery_handle, ADC_CHANNEL_BATTERY, &adc_raw));
    ESP_ERROR_CHECK(adc_cali_raw_to_voltage(battery_cali_handle, adc_raw, &cali_voltage));
    battery_update_voltage(cali_voltage);
}

/* 内置测试循环 (会创建 FreeRTOS 任务，仅用于独立测试) */
void battery_test(void)
{
    ESP_LOGI(TAG, "电池电压检测");

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(500));
        float voltage = battery_get_voltage();
        ESP_LOGI(TAG, "电池电压: %.2fV", voltage);
    }
}

/* 获取缓存电压值 */
float battery_get_voltage(void)
{
    return battery_voltage;
}
