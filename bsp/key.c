/**
 * @file    key.c
 * @brief   按键驱动实现 — 软件防抖 + 非阻塞轮询
 *
 * 防抖原理:
 *   采用"连续确认"策略——不做延时等待，而是要求连续 2 次 (20ms) 都检测到
 *   按下才输出按下事件。这样既不影响主循环实时性，又能可靠过滤机械弹跳。
 *
 *   状态机:
 *     key_state=0  →  检测到按下 → key_state=1 (初步确认)
 *     key_state=1  →  仍检测到按下 → key_state=2 → 返回 PRESS (确认有效)
 *     任意时刻检测到松开 → key_state 立即归零 (防误触)
 */

#include "key.h"

#include "stdio.h"
#include "stdint.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "driver/gpio.h"

static const char *TAG = "KEY";

/* ================================================================
 * 底层读取 — 直接读 GPIO 电平 (无防抖)
 * ================================================================ */

/* KEY0 = GPIO0 (BOOT 键), 上拉, 按下为低电平 */
static uint8_t key0_is_pressed(void)
{
    uint8_t key_state = KEY_STATE_RELEASE;
    if (!gpio_get_level(KEY_GPIO_BOOT0)) {
        key_state = KEY_STATE_PRESS;
    }
    return key_state;
}

/* KEY1 = GPIO42, 上拉, 按下为低电平 */
static uint8_t key1_is_pressed(void)
{
    uint8_t key_state = KEY_STATE_RELEASE;
    if (!gpio_get_level(KEY_GPIO_USER1)) {
        key_state = KEY_STATE_PRESS;
    }
    return key_state;
}

/* ================================================================
 * 公开 API — 带软件防抖的按键读取
 * ================================================================ */

/* KEY0: 连续 2 次检测到按下 → 返回 PRESS
 *       key_state 计数: 0→1→2(触发) → 松开 → 0 */
uint8_t key0_read_state(void)
{
    static uint16_t key_state = 0;               /* 防抖计数器 */

    if (key0_is_pressed() == KEY_STATE_PRESS) {
        if (key_state < 3) {
            key_state++;                         /* 连续按下 → 计数递增 */
        }
    } else {
        key_state = 0;                           /* 松开 → 立即归零 (防误触) */
    }

    if (key_state == 2) {
        return KEY_STATE_PRESS;                  /* 连续 2 次确认 → 有效按下 */
    }
    return KEY_STATE_RELEASE;
}

/* KEY1: 同 KEY0 的防抖逻辑 */
uint8_t key1_read_state(void)
{
    static uint16_t key_state = 0;

    if (key1_is_pressed() == KEY_STATE_PRESS) {
        if (key_state < 3) {
            key_state++;
        }
    } else {
        key_state = 0;
    }

    if (key_state == 2) {
        return KEY_STATE_PRESS;
    }
    return KEY_STATE_RELEASE;
}

/* ================================================================
 * GPIO 初始化 — 配置 KEY0 + KEY1 为输入 + 内部上拉
 * ================================================================ */
void key_init(void)
{
    gpio_config_t io_conf = {};
    io_conf.intr_type    = GPIO_INTR_DISABLE;    /* 禁用中断 (轮询模式) */
    io_conf.mode         = GPIO_MODE_INPUT;      /* 输入模式 */
    io_conf.pin_bit_mask = ((1ULL << KEY_GPIO_BOOT0) | (1ULL << KEY_GPIO_USER1));
    io_conf.pull_down_en = 0;                    /* 禁用下拉 */
    io_conf.pull_up_en   = 1;                    /* 启用上拉 (未按=高, 按下=低) */
    gpio_config(&io_conf);
}

/* ================================================================
 * 内置测试 (阻塞运行，打印按键事件和内存信息)
 * ================================================================ */
void key_test(void)
{
    while (1) {
        if (key0_read_state() == KEY_STATE_PRESS) {
            ESP_LOGI(TAG, "按键 0 被按下");
            ESP_LOGI(TAG, "空闲内部堆大小 = %lu", (unsigned long)esp_get_free_internal_heap_size());
        }
        if (key1_read_state() == KEY_STATE_PRESS) {
            ESP_LOGI(TAG, "按键 1 被按下");
            printf("空闲堆大小 = %lu\r\n", (unsigned long)esp_get_free_heap_size());
        }

        vTaskDelay(pdMS_TO_TICKS(10));           /* 10ms 轮询周期 */
    }
}
