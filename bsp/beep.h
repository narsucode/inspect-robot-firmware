/**
 * @file    beep.h
 * @brief   蜂鸣器驱动 — GPIO 高低电平控制
 *
 * 三步拆解术:
 *   1. 直观理解: 蜂鸣器就是一个"电喇叭"——
 *      GPIO 输出高电平 → 喇叭响, 输出低电平 → 喇叭停。
 *      本质上就是一个可以快速开关的扬声器。
 *   2. 追根溯源: 没有蜂鸣器，电机启动/停止、按键反馈、故障告警
 *      都只能靠肉眼观察，调试效率大打折扣。
 *   3. 正规解释: 使用 GPIO 推挽输出模式驱动有源蜂鸣器 (内部自带振荡器)。
 *      通过 BEEP_ACTIVE_LEVEL 宏适配高/低电平有效的蜂鸣器型号。
 *
 * 裸机使用 (推荐):
 *   @code
 *   beep_init();              // 初始化 GPIO
 *   // 主循环中:
 *   beep_on();                // 响
 *   beep_off();               // 停
 *   beep_handle();            // 每 10ms 调用，处理定时关闭
 *   @endcode
 */

#pragma once
#ifdef __cplusplus
extern "C" {
#endif
#include "stdint.h"
#include "stdbool.h"
#include "car_config.h"         /* PIN_BEEP 在此统一配置 */

/* ──── 硬件配置 ──── */
#define BEEP_GPIO           PIN_BEEP   /* 蜂鸣器控制引脚 (car_config.h) */

/* 蜂鸣器有效电平: 0=低电平响, 1=高电平响
 * 常见有源蜂鸣器多为高电平响，无源蜂鸣器需 PWM */
#define BEEP_ACTIVE_LEVEL  1

/* ──── 内部状态枚举 ──── */
#define BEEP_STATE_OFF        0            /* 关闭 */
#define BEEP_STATE_ON_ALWAYS  1            /* 持续响 */
#define BEEP_STATE_ON_DELAY   2            /* 延时关闭中 */
#define BEEP_STATE_PATTERN    3            /* 模式演奏中 (多声短促) */

/* ──── 快捷宏 ──── */
#define BEEP_ON()       beep_on()
#define BEEP_OFF()      beep_off()

/**
 * @brief 初始化蜂鸣器 GPIO (裸机模式，不创建 FreeRTOS 任务)
 *
 * 配置引脚为推挽输出，默认关闭蜂鸣器。
 * 裸机模式需要在主循环中周期性调用 beep_handle() 处理延时关闭。
 */
void beep_init(void);

/** @brief 打开蜂鸣器 (持续响，直到调用 beep_off) */
void beep_on(void);

/** @brief 关闭蜂鸣器 */
void beep_off(void);

/**
 * @brief 设置蜂鸣器延时关闭
 * @param time  持续时间 (ms)
 *              - time=0    → 立即关闭
 *              - time=1    → 持续响 (等价于 beep_on)
 *              - time>=10  → 响指定毫秒后自动关闭
 *              - 自动钳位到 [10, 10000] 范围
 * @note  依赖主循环每 10ms 调用 beep_handle() 来倒计时
 */
void beep_on_time(uint16_t time);

/**
 * @brief 蜂鸣器状态机轮询 (裸机核心 API)
 *
 * 处理延时关闭倒计时。主循环中每 10ms 调用一次。
 * 如果未设置延时则立即返回，不产生额外开销。
 */
void beep_handle(void);

/** @brief 内置测试 (阻塞运行，会创建 FreeRTOS 任务) */
void beep_test(void);

/**
 * @brief 演奏多声短促蜂鸣 (非阻塞)
 * @param count   响声次数 (1~10)
 * @param on_ms   每次响多久 (ms, 建议 50~500)
 * @param off_ms  两声之间停多久 (ms, 建议 50~500)
 * @note  由 beep_handle() 驱动; 再次调用会覆盖正在演奏的模式.
 */
void beep_play_pattern(int count, int on_ms, int off_ms);

/**
 * @brief 检查蜂鸣器是否繁忙 (模式演奏中或延时关闭中)
 * @return true=繁忙, false=空闲
 */
bool beep_is_busy(void);

#ifdef __cplusplus
}
#endif
