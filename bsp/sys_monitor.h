/**
 * @file    sys_monitor.h
 * @brief   系统监控 — 上电自检 + 持续电池监控 + 蜂鸣器/LED 提示
 *
 * 三步拆解术:
 *   1. 直观理解: 机器人上电后自动"体检"一遍——
 *      WiFi 通不通、Agent 连没连、雷达转没转、IMU 好没好、电池够不够，
 *      全通过就嘟一声，有问题就短促两声并在屏幕显示故障项。
 *   2. 追根溯源: 没有自检，你可能跑出去半圈才发现雷达没开、
 *      IMU 漂移、电池快没电——回头已经晚了。
 *   3. 正规解释: 上电时逐一检查各子系统状态，合并为自检结果，
 *      通过 beep_play_pattern() 非阻塞反馈，错误信息推送 OLED。
 *      持续运行时通过 EMA 滤波监控电池电压，分两档报警。
 *
 * 电池阈值 (3S LiPo, 见 car_config.h):
 *   WARNING  = 10.5V (3.5V/cell)  — OLED 显示预警
 *   CRITICAL = 10.0V (3.33V/cell) — 蜂鸣器 3 声 + LED 快闪, 30s 重复
 */

#ifndef SYS_MONITOR_H
#define SYS_MONITOR_H

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/** 自检错误码 */
typedef enum {
    SYS_OK = 0,             /**< 全部正常 */
    SYS_ERR_WIFI,           /**< WiFi 未连接 */
    SYS_ERR_AGENT,          /**< Agent 未连接 */
    SYS_ERR_RADAR,          /**< 雷达无数据 */
    SYS_ERR_IMU,            /**< MPU6050 未检测到 */
    SYS_ERR_BATTERY         /**< 电池低于预警值 */
} sys_error_t;

/** 电池状态 */
typedef enum {
    BAT_NORMAL = 0,         /**< 正常 (> WARN) */
    BAT_WARNING,            /**< 预警 (WARN ~ CRITICAL) */
    BAT_CRITICAL            /**< 告警 (< CRITICAL) */
} battery_state_t;

/* ================================================================
 * 公开 API
 * ================================================================ */

/**
 * @brief 上电自检 — 检查 WiFi/Agent/雷达/IMU/电池
 *
 * 所有正常: 蜂鸣器长鸣 1 秒 (beep_on_time(1000)).
 * 任意异常: 蜂鸣器短促 2 声 (250ms on/off).
 *
 * @return 第一个发现的错误码, SYS_OK=全正常
 *
 * @note  必须在 OLED 已初始化、WiFi 已连接、microros_bridge_init
 *         已完成后调用. 非阻塞, 蜂鸣由 beep_handle 驱动.
 */
sys_error_t sys_monitor_boot_check(void);

/**
 * @brief 电池状态持续监控 (每周期调用)
 *
 * 执行: battery_update() → EMA 滤波 → 阈值比较 → 蜂鸣器/LED 告警.
 * 低于 CRITICAL 时蜂鸣器 3 声 + LED 快闪, 30s 冷却后重复.
 *
 * @return 当前电池状态
 */
battery_state_t sys_monitor_battery_tick(void);

/**
 * @brief 获取最近一次自检的错误码 (供 OLED 持久显示)
 * @return 上次 sys_monitor_boot_check() 的结果
 */
sys_error_t sys_monitor_get_last_error(void);

/**
 * @brief 获取 EMA 滤波后的电池电压 (更平滑)
 * @return 滤波后电压 (V)
 */
float sys_monitor_get_filtered_voltage(void);

/**
 * @brief 自检错误 → OLED 可显示的短字符串
 * @param err  错误码
 * @return 中文描述 (如 "WiFi未连"、"雷达无数据")
 */
const char* sys_error_string(sys_error_t err);

#ifdef __cplusplus
}
#endif

#endif /* SYS_MONITOR_H */
