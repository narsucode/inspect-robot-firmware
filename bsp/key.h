/**
 * @file    key.h
 * @brief   按键驱动 — GPIO 输入 + 软件防抖
 *
 * 三步拆解术:
 *   1. 直观理解: 按键就像一扇弹簧门——按下时触点闭合（GPIO 读到低电平），
 *      松手时弹开（读到高电平）。但弹簧门会"弹跳"，触点瞬间通断好几次，
 *      软件防抖就是告诉程序"别急，等门稳住再说"。
 *   2. 追根溯源: 没有防抖，按一次键可能被识别为 3~5 次——
 *      电机启停、模式切换等逻辑会乱套，完全不可用。
 *   3. 正规解释: 使用 GPIO 内部上拉 + 输入模式。软件防抖采用"连续确认"策略：
 *      连续 2 次检测到按下才确认为有效按键，松开后立即复位计数。
 *
 * 使用示例:
 *   @code
 *   key_init();
 *   if (key0_read_state() == KEY_STATE_PRESS) {
 *       // 按键被按下一次 (自带防抖，不会重复触发)
 *   }
 *   @endcode
 */

#pragma once
#ifdef __cplusplus
extern "C" {
#endif
#include "stdint.h"

/* ──── 引脚定义 ──── */
#define KEY_GPIO_BOOT0        0              /* KEY0: BOOT 按键, 上拉 */
#define KEY_GPIO_USER1        42             /* KEY1: 用户自定义按键, 上拉 */

/* ──── 状态返回值 ──── */
#define KEY_STATE_PRESS       1              /* 按键按下 */
#define KEY_STATE_RELEASE     0              /* 按键松开 */

/**
 * @brief 初始化按键 GPIO
 *
 * 配置 KEY0 (GPIO0) 和 KEY1 (GPIO42) 为输入模式 + 内部上拉。
 * 按键未按下时引脚为高电平，按下后拉低。
 */
void key_init(void);

/**
 * @brief 读取 KEY0 状态 (非阻塞，内置软件防抖)
 * @return KEY_STATE_PRESS(1) = 本次检测到按下, KEY_STATE_RELEASE(0) = 松开
 * @note   需要主循环每 10~20ms 轮询一次才能正常防抖
 */
uint8_t key0_read_state(void);

/**
 * @brief 读取 KEY1 状态 (非阻塞，内置软件防抖)
 * @return KEY_STATE_PRESS(1) = 按下, KEY_STATE_RELEASE(0) = 松开
 */
uint8_t key1_read_state(void);

/** @brief 内置测试循环 (阻塞运行) */
void key_test(void);

#ifdef __cplusplus
}
#endif
