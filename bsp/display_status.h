/**
 * @file    display_status.h
 * @brief   OLED 状态显示 — 4 行实时信息
 *
 * 三步拆解术:
 *   1. 直观理解: 屏幕是小车的"仪表盘"——
 *      一眼看到电池剩多少、WiFi 连没连、Agent 通不通、
 *      遥控发了什么命令、轮子目标跑多快、实际跑了多快。
 *   2. 追根溯源: 没有屏幕，拔掉 USB 就是瞎子——
 *      你不知道 Agent 是不是挂了、电池是不是要没电了。
 *   3. 正规解释: 从各驱动模块读取实时状态，格式化后
 *      通过 OLED 驱动写到 SSD1306 屏幕，~2Hz 刷新。
 *
 * 屏幕布局 (FONT_16_EN, 4行×16字):
 *   ┌──────────────────────┐
 *   │YouDaoBot v1.0        │  行 0: 硬件 + 版本
 *   │B:12.8V W:+ A:+       │  行 1: 电池 / WiFi / Agent
 *   │v+.44 w+.00           │  行 2: cmd_vel (vx, wz)
 *   │L+44+46 R+44+47       │  行 3: 左右轮 目标/实际 (×100)
 *   └──────────────────────┘
 *
 * 图例:
 *   W:+ = WiFi 已连接, W:- = 未连接
 *   A:+ = Agent 已连接, A:- = 未连接
 *   v = linear.x (m/s), w = angular.z (rad/s)
 *   L/R 数字 = 目标速度/实际速度 ×100 (例: +44 = 0.44 m/s)
 */

#ifndef DISPLAY_STATUS_H
#define DISPLAY_STATUS_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>

/**
 * @brief 刷新 OLED 状态显示 (~2Hz 调用)
 *
 * 从各驱动模块读取:
 *   - battery_get_voltage()         电池电压
 *   - wifi_status_is_connected()    WiFi 状态
 *   - microros_bridge_is_agent_connected()  Agent 状态
 *   - cmd_vel_get_last_command()    最近命令
 *   - cmd_vel_get_wheel_state()     四轮状态
 *
 * 读取后立即写 OLED，不缓存。
 */
void display_status_update(void);

/**
 * @brief 设置 WiFi 连接状态 (由 main.c 在连接/断线时调用)
 * @param connected  true=已连接, false=未连接
 */
void wifi_status_set_connected(bool connected);

/**
 * @brief 查询 WiFi 连接状态
 * @return true=已连接, false=未连接
 */
bool wifi_status_is_connected(void);

/**
 * @brief 请求刷新屏幕 (由 cmd_vel 回调等事件触发)
 *
 * 设置一个脏标记, main 循环在空闲时检测并刷新。
 * 不立即写 OLED（避免 I2C 阻塞控制循环）。
 */
void display_status_request_update(void);

/**
 * @brief 检查并消费屏幕刷新请求
 * @return true=需要刷新 (消费标记), false=无请求
 */
bool display_status_consume_request(void);

/**
 * @brief 检查是否处于快速刷新窗口 (收到 cmd_vel 后 2s 内)
 * @return true=快速模式, false=正常模式
 */
bool display_status_in_fast_window(void);

#ifdef __cplusplus
}
#endif

#endif /* DISPLAY_STATUS_H */
