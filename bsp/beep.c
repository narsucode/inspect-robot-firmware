/**
 * @file    beep.c
 * @brief   蜂鸣器驱动实现 — 裸机模式 GPIO 高低电平控制
 *
 * 工作模式:
 *   驱动内置一个简单状态机，支持三种模式：
 *     OFF        → 关闭，GPIO 输出无效电平
 *     ON_ALWAYS  → 持续响，GPIO 输出有效电平
 *     ON_DELAY   → 延时关闭，beep_handle() 每 10ms 递减计数器
 *
 * 裸机注意事项:
 *   主循环中必须每 10ms 调用 beep_handle() 才能实现延时关闭功能。
 *   若不使用延时关闭 (只用 beep_on/beep_off)，可以不调 beep_handle。
 */

#include "beep.h"

#include <stdio.h>
#include <stdint.h>

#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"

#include "key.h"

static const char *TAG = "BEEP";

/* ──── 状态机变量 ──── */
static uint8_t  beep_state       = BEEP_STATE_OFF;  /* 当前状态 */
static uint16_t beep_delay_count = 0;               /* 延时剩余计数 (单位: 10ms) */

/* ──── 模式演奏变量 ──── */
static int  g_pat_remaining = 0;    /* 剩余响声次数 */
static int  g_pat_on_ms     = 0;    /* 每响声时长 (ms) */
static int  g_pat_off_ms    = 0;    /* 两声间隔 (ms) */
static bool g_pat_in_on     = true; /* true=响声中, false=间隔中 */
static int  g_pat_timer     = 0;    /* 当前阶段剩余计数 (单位: 10ms) */

/* ================================================================
 * GPIO 初始化 — 配置为推挽输出，默认关闭
 * ================================================================ */
static void beep_gpio_init(void)
{
    gpio_config_t io_conf = {};
    io_conf.intr_type    = GPIO_INTR_DISABLE;      /* 禁用中断 */
    io_conf.mode         = GPIO_MODE_OUTPUT;        /* 推挽输出 */
    io_conf.pin_bit_mask = (1ULL << BEEP_GPIO);     /* 选中引脚 */
    io_conf.pull_down_en = 0;                       /* 禁用下拉 */
    io_conf.pull_up_en   = 0;                       /* 禁用上拉 */
    gpio_config(&io_conf);
    beep_off();                                     /* 初始状态: 关闭 */
}

/* ================================================================
 * 公开 API
 * ================================================================ */

/* 初始化 (裸机模式: 不创建 FreeRTOS 任务) */
void beep_init(void)
{
    beep_gpio_init();
    ESP_LOGI(TAG, "蜂鸣器初始化完成（裸机模式，需在主循环轮询 beep_handle()）");
}

/* 持续响 */
void beep_on(void)
{
    beep_state       = BEEP_STATE_ON_ALWAYS;
    beep_delay_count = 0;
    gpio_set_level(BEEP_GPIO, BEEP_ACTIVE_LEVEL);
}

/* 立即关闭 */
void beep_off(void)
{
    beep_state       = BEEP_STATE_OFF;
    beep_delay_count = 0;
    gpio_set_level(BEEP_GPIO, !BEEP_ACTIVE_LEVEL);
}

/* 延时关闭 — time=0 关闭, time=1 常响, time>=10 延时 ms 后关闭 */
void beep_on_time(uint16_t time)
{
    if (time == 1) {
        /* 持续响 */
        beep_state       = BEEP_STATE_ON_ALWAYS;
        beep_delay_count = 0;
        beep_on();
    } else if (time == 0) {
        /* 立即关闭 */
        beep_state       = BEEP_STATE_OFF;
        beep_delay_count = 0;
        beep_off();
    } else {
        /* 延时关闭: 将 ms 换算为 10ms 计数单位 */
        if (time < 10)    time = 10;     /* 最小 10ms */
        if (time > 10000) time = 10000;  /* 最大 10 秒 */
        beep_state       = BEEP_STATE_ON_DELAY;
        beep_delay_count = (time / 10);  /* 每 tick = 10ms */
        gpio_set_level(BEEP_GPIO, BEEP_ACTIVE_LEVEL);
    }
}

/* 状态机轮询 — 每 10ms 调用一次，处理延时关闭倒计时 + 模式演奏 */
void beep_handle(void)
{
    if (beep_state == BEEP_STATE_ON_DELAY) {
        if (beep_delay_count > 0) {
            beep_delay_count--;
        } else {
            beep_off();
            beep_state = BEEP_STATE_OFF;
        }
    } else if (beep_state == BEEP_STATE_PATTERN) {
        if (g_pat_timer > 0) {
            g_pat_timer--;
        } else {
            /* 当前阶段结束, 切换 */
            if (g_pat_in_on) {
                /* 响声结束 → 进入间隔 (直接操作 GPIO, 不调 beep_off 避免覆盖状态) */
                gpio_set_level(BEEP_GPIO, !BEEP_ACTIVE_LEVEL);
                g_pat_in_on = false;
                g_pat_timer = g_pat_off_ms / 10;
            } else {
                /* 间隔结束 → 下一声或结束 */
                g_pat_remaining--;
                if (g_pat_remaining > 0) {
                    gpio_set_level(BEEP_GPIO, BEEP_ACTIVE_LEVEL);
                    g_pat_in_on = true;
                    g_pat_timer = g_pat_on_ms / 10;
                } else {
                    beep_state = BEEP_STATE_OFF;
                }
            }
        }
    }
}

void beep_play_pattern(int count, int on_ms, int off_ms)
{
    if (count <= 0 || count > 10) count = 1;
    if (on_ms  < 10)   on_ms  = 10;
    if (off_ms < 10)   off_ms = 10;
    g_pat_remaining = count;
    g_pat_on_ms     = on_ms;
    g_pat_off_ms    = off_ms;
    g_pat_in_on     = true;
    g_pat_timer     = on_ms / 10;
    beep_state      = BEEP_STATE_PATTERN;
    gpio_set_level(BEEP_GPIO, BEEP_ACTIVE_LEVEL);
}

bool beep_is_busy(void)
{
    return (beep_state == BEEP_STATE_PATTERN ||
            beep_state == BEEP_STATE_ON_DELAY);
}

/* ================================================================
 * 内置测试 (阻塞运行，使用 FreeRTOS 任务)
 * ================================================================ */
void beep_test(void)
{
    static uint8_t last_key_state = KEY_STATE_RELEASE;
    static uint8_t beep_enabled   = 0;

    key_init();
    beep_init();
    beep_off();

    while (1) {
        uint8_t key_state = key0_read_state();

        /* 按键下降沿触发: 翻转蜂鸣器开关 */
        if ((key_state == KEY_STATE_PRESS) && (last_key_state == KEY_STATE_RELEASE)) {
            beep_enabled = !beep_enabled;
            if (beep_enabled) {
                ESP_LOGI(TAG, "按键 0 被按下，蜂鸣器开启");
                beep_on();
            } else {
                ESP_LOGI(TAG, "按键 0 被按下，蜂鸣器关闭");
                beep_off();
            }
        }

        last_key_state = key_state;
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
