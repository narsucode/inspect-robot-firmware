#ifndef __LED_H
#define __LED_H

#include <stdint.h>


// 配置 LED 对应的 GPIO 引脚（用户可根据硬件修改）
#define LED_GPIO     GPIO_NUM_14

// 配置 LED 有效电平（1=高电平点亮，0=低电平点亮）
#define LED_ACTIVE_LEVEL  0

// 函数声明
/**
 * @brief  LED 初始化（配置GPIO为输出，默认关灯）
 */
void led_init(void);

/**
 * @brief  非阻塞 LED 闪烁函数（需在定时中断/循环中周期性调用）
 * @param  interval: 闪烁间隔（单位：调用次数，通常=ms）
 */
void led_flash(uint16_t interval);

/**
 * @brief  设置 LED 亮灭状态
 * @param  state: 0=灭，1=亮
 */
void led_state(uint8_t state);

/**
 * @brief  关闭 LED
 */
void led_off(void);

/**
 * @brief  打开 LED
 */
void led_on(void);

/**
 * @brief  测试 LED 控制函数
 */
void test_led(void);

#endif