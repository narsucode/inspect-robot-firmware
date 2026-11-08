/**
 * @file    display_status.c
 * @brief   OLED 状态显示实现 — 读取各模块状态 → 格式化 → 写屏
 */

#include "display_status.h"

#include <stdio.h>
#include <string.h>
#include <math.h>

#include "esp_timer.h"

#include "car_config.h"
#include "battery.h"
#include "oled.h"
#include "cmd_vel_control.h"
#include "microros_bridge.h"
#include "sys_monitor.h"

/* =============================================================================
 * 模块内部状态变量
 * ========================================================================== */

/** @brief WiFi连接状态，由main.c调用wifi_status_set_connected更新 */
static bool g_wifi_connected = false;

/** @brief 屏幕刷新请求标记，事件置位，主循环消费 */
static volatile bool g_display_dirty = false;

/** @brief 快速刷新窗口截止时间(us)，收到cmd_vel后2s内高频刷新 */
static int64_t g_fast_refresh_until_us = 0;

/** @brief 快速刷新窗口时长(us)：2秒 */
#define FAST_REFRESH_WINDOW_US  2000000LL

/* =============================================================================
 * WiFi状态接口
 * ========================================================================== */

void wifi_status_set_connected(bool connected)
{
    g_wifi_connected = connected;
}

bool wifi_status_is_connected(void)
{
    return g_wifi_connected;
}

/* =============================================================================
 * 屏幕刷新控制接口
 * ========================================================================== */

/** @brief 请求刷新屏幕，并开启快速刷新计时窗口 */
void display_status_request_update(void)
{
    g_display_dirty = true;
    g_fast_refresh_until_us = esp_timer_get_time() + FAST_REFRESH_WINDOW_US;
}

/** @brief 消费刷新请求，返回是否需要更新屏幕，同时清除脏标记 */
bool display_status_consume_request(void)
{
    bool dirty = g_display_dirty;
    g_display_dirty = false;
    return dirty;
}

/** @brief 判断当前是否处于快速刷新时间段 */
bool display_status_in_fast_window(void)
{
    return esp_timer_get_time() < g_fast_refresh_until_us;
}

/* =============================================================================
 * OLED 屏幕刷新主函数
 * ========================================================================== */

void display_status_update(void)
{
    char line[17];  /**< FONT_16_EN：每行16字符 + 字符串结束符'\0' */

    /* 第0行：硬件名称 + 固件版本 */
    snprintf(line, sizeof(line), "%s %s",
             CAR_HARDWARE_NAME, CAR_FIRMWARE_VERSION);
    oled_show_string(0, 0, line, FONT_16_EN);

    /* 第1行：电池电压 / WiFi状态 / micro-ROS Agent连接状态
     * 电池使用sys_monitor维护的EMA滤波电压，数值更平滑
     * Agent连通时默认标记WiFi有效 */
    float voltage = sys_monitor_get_filtered_voltage();
    bool agent_ok = microros_bridge_is_agent_connected();
    snprintf(line, sizeof(line), "B:%4.1fV W:%c A:%c",
             (double)voltage,
             (wifi_status_is_connected() || agent_ok) ? '+' : '-',
             agent_ok ? '+' : '-');
    oled_show_string(0, 2, line, FONT_16_EN);

    /* 第2行：cmd_vel指令 / 系统错误提示
     * 存在系统错误时，打印错误信息；无错误则打印vx、wz */
    sys_error_t boot_err = sys_monitor_get_last_error();
    if (boot_err != SYS_OK) {
        snprintf(line, sizeof(line), "ERR:%s",
                 sys_error_string(boot_err));
    } else {
        float vx, wz;
        cmd_vel_get_last_command(&vx, &wz);
        snprintf(line, sizeof(line), "v%+.2f w%+.2f",
                 (double)vx, (double)wz);
    }
    oled_show_string(0, 4, line, FONT_16_EN);

    /* 第3行：左右轮目标速度 / 实际速度
     * 数值 ×100 放大显示，省去小数点，节省屏幕空间 */
    float tl, tr, al, ar;
    cmd_vel_get_wheel_state(&tl, &tr, &al, &ar);
    snprintf(line, sizeof(line), "L%+03d%+03d R%+03d%+03d",
             (int)(tl * 100.0f), (int)(al * 100.0f),
             (int)(tr * 100.0f), (int)(ar * 100.0f));
    oled_show_string(0, 6, line, FONT_16_EN);
}