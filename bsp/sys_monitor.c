/**
 * @file    sys_monitor.c
 * @brief   系统监控实现 — 上电自检 + 电池 EMA 滤波 + 蜂鸣器/LED 告警
 */

#include "sys_monitor.h"

#include <stdio.h>
#include <string.h>
#include <math.h>

#include "esp_timer.h"

#include "car_config.h"
#include "battery.h"
#include "beep.h"
#include "led.h"
#include "debug_log.h"

#include "microros_bridge.h"
#include "display_status.h"
#include "ydlidar_x3pro.h"
#include "mpu6050.h"

/* ================================================================
 * 模块级状态
 * ================================================================ */

/** @brief EMA 滤波后的电池电压 */
static float g_bat_filtered = 0.0f;

/** @brief EMA 滤波器是否已初始化 (首帧直接赋值) */
static bool g_bat_filter_init = false;

/** @brief 上次触发蜂鸣器告警的时间戳 (us), 冷却用 */
static int64_t g_last_alarm_us = 0;

/** @brief 上次自检结果 (OLED 持久显示用) */
static sys_error_t g_last_boot_error = SYS_OK;

/** @brief 当前告警是否已触发过 (状态变化时重置) */
static bool g_alarm_triggered = false;

/* ================================================================
 * 电池 EMA 滤波
 * ================================================================ */

/**
 * @brief EMA 低通滤波: filtered = alpha × raw + (1-alpha) × filtered
 *
 * alpha=0.1 意味着每次采样新值权重 10%，旧值 90%。
 * 约 10 次采样后达到新稳态 (63% 收敛), 有效抑制电机负载瞬态压降。
 */
static float battery_ema_filter(float raw)
{
    if (!g_bat_filter_init) {
        g_bat_filtered = raw;
        g_bat_filter_init = true;
    } else {
        g_bat_filtered = BATTERY_EMA_ALPHA * raw +
                         (1.0f - BATTERY_EMA_ALPHA) * g_bat_filtered;
    }
    return g_bat_filtered;
}

/* ================================================================
 * 上电自检
 * ================================================================ */

sys_error_t sys_monitor_boot_check(void)
{
    sys_error_t first_err = SYS_OK;
    bool wifi_ok, agent_ok, radar_ok, imu_ok, bat_ok;

    /* 1. WiFi */
    wifi_ok = wifi_status_is_connected();
    if (!wifi_ok && first_err == SYS_OK) first_err = SYS_ERR_WIFI;

    /* 2. Agent */
    agent_ok = microros_bridge_is_agent_connected();
    if (!agent_ok && first_err == SYS_OK) first_err = SYS_ERR_AGENT;

    /* 3. 雷达 — 检查是否有数据产出 */
    radar_ok = (ydlidar_x3pro_get_scan() != NULL);
    if (!radar_ok && first_err == SYS_OK) first_err = SYS_ERR_RADAR;

    /* 4. IMU */
    imu_ok = imu_check();
    if (!imu_ok && first_err == SYS_OK) first_err = SYS_ERR_IMU;

    /* 5. 电池 */
    battery_update();
    float voltage = battery_get_voltage();
    (void)battery_ema_filter(voltage);  /* 启动滤波器 */
    bat_ok = (voltage >= BATTERY_WARN_V);
    if (!bat_ok && first_err == SYS_OK) first_err = SYS_ERR_BATTERY;

    if (first_err == SYS_OK) {
        g_last_boot_error = SYS_OK;
        /* 一切正常: 短促一声确认 */
        debug_log_printf("[SYS] Boot check: ALL OK  "
                         "WiFi=%c Agt=%c Rdr=%c IMU=%c Bat=%.1fV\n",
                         wifi_ok?'+':'-', agent_ok?'+':'-',
                         radar_ok?'+':'-', imu_ok?'+':'-',
                         (double)voltage);
        beep_play_pattern(1, 50, 0);
    } else {
        g_last_boot_error = first_err;
        /* 异常: 短促 2 声 */
        debug_log_printf("[SYS] Boot check: FAIL (%s)  "
                         "WiFi=%c Agt=%c Rdr=%c IMU=%c Bat=%.1fV\n",
                         sys_error_string(first_err),
                         wifi_ok?'+':'-', agent_ok?'+':'-',
                         radar_ok?'+':'-', imu_ok?'+':'-',
                         (double)voltage);
        beep_play_pattern(2, 100, 100);
    }

    return first_err;
}

/* ================================================================
 * 持续电池监控
 * ================================================================ */

battery_state_t sys_monitor_battery_tick(void)
{
    /* 1. 读取 + EMA 滤波 */
    battery_update();
    float raw = battery_get_voltage();
    float filtered = battery_ema_filter(raw);

    /* 2. 阈值判断 */

    /* 合理性校验：3S 锂电不可能低于 6.0V(单节2.0V)。
     * 低于 BATTERY_INVALID_BELOW_V 说明分压器没接 / ADC 通道不对，
     * 属于"采样无效"，按正常处理，避免持续误告警。 */
    if (filtered < BATTERY_INVALID_BELOW_V) {
        static bool s_invalid_logged = false;
        if (!s_invalid_logged) {
            debug_log_printf("[SYS] Battery reading invalid: %.1fV, alarm disabled\n",
                             (double)filtered);
            s_invalid_logged = true;
        }
        g_alarm_triggered = false;
        return BAT_NORMAL;
    }

    battery_state_t state;
    if (filtered < BATTERY_CRITICAL_V) {
        state = BAT_CRITICAL;
    } else if (filtered < BATTERY_WARN_V) {
        state = BAT_WARNING;
    } else {
        state = BAT_NORMAL;
    }

    /* 3. 根据状态触发蜂鸣器/LED
     *
     * 蜂鸣器原则: 不连续响，不聒噪.
     *   - CRITICAL: 短促 3 声 (250ms on/off), 30s 冷却后重复
     *   - WARNING: 不响 (仅 OLED 显示)
     *   - NORMAL: 不响, LED 常亮
     *
     * LED 原则:
     *   - CRITICAL: 快闪 (led_flash 每 ~100ms 调用一次, interval=5 → 250ms 周期)
     *   - WARNING: 慢闪 (led_flash interval=20)
     *   - NORMAL: agent 在线时亮, 离线时慢闪 */

    int64_t now_us = esp_timer_get_time();
    int64_t cooldown_us = (int64_t)BATTERY_ALARM_COOLDOWN_S * 1000000LL;

    if (state == BAT_CRITICAL) {
        /* LED 快闪: led_flash 调用在 main 循环中, interval=5 → ~250ms 周期 */
        /* led_flash 由 main 循环调用, 这里不做 */

        /* 蜂鸣器告警: 30s 冷却 */
        if (!g_alarm_triggered ||
            (now_us - g_last_alarm_us) >= cooldown_us) {
            debug_log_printf("[SYS] Battery CRITICAL: %.1fV (raw=%.1fV)\n",
                             (double)filtered, (double)raw);
#if BATTERY_ALARM_BEEP_ENABLE
            beep_play_pattern(3, 100, 100);
#endif
            g_last_alarm_us = now_us;
            g_alarm_triggered = true;
        }
    } else if (state == BAT_WARNING) {
        /* 预警: 不响, 仅 OLED 显示 */
        g_alarm_triggered = false;
    } else {
        /* 正常: 电压恢复到 WARN+0.5V 以上才重置告警状态 */
        if (filtered >= BATTERY_WARN_V + 0.5f) {
            g_alarm_triggered = false;
        }
    }

    return state;
}

float sys_monitor_get_filtered_voltage(void)
{
    return g_bat_filtered;
}

sys_error_t sys_monitor_get_last_error(void)
{
    return g_last_boot_error;
}

/* ================================================================
 * 错误码 → OLED 显示文字
 * ================================================================ */

const char* sys_error_string(sys_error_t err)
{
    switch (err) {
        case SYS_OK:        return "OK";
        case SYS_ERR_WIFI:  return "WiFi未连";
        case SYS_ERR_AGENT: return "Agent断";
        case SYS_ERR_RADAR: return "雷达无数据";
        case SYS_ERR_IMU:   return "IMU异常";
        case SYS_ERR_BATTERY: return "电池低";
        default:            return "未知";
    }
}
