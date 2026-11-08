/**
 * @file    voice.c
 * @brief   语音播报模块驱动实现 — UART2 发送事件码
 *
 * 工作方式:
 *   只写不读。ESP32 通过 UART2 发 "EVENT\n"，天问模块按配置匹配后播报。
 *   不解析模块回传，避免引入额外状态机。
 *
 * 去重:
 *   voice_say() 记录上一条事件码与时间戳，相同事件码在 VOICE_MIN_GAP_MS
 *   内重复触发时直接丢弃。WiFi 抖动、Agent 反复重连时不会连续播报多遍。
 *
 * 线程安全:
 *   uart_write_bytes 内部有互斥，多个任务同时调用不会写乱。
 *   本模块的状态（s_last_code / s_last_us）只在主循环与初始化路径使用，不加锁。
 */

#include "voice.h"

#include <string.h>
#include <stdio.h>

#include "driver/uart.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"

#include "car_config.h"

static const char *TAG = "VOICE";

/** @brief 串口是否已初始化（未初始化时 voice_say 直接返回） */
static bool s_inited = false;

/** @brief 上一条成功发送的事件码，用于去重 */
static char s_last_code[24] = {0};

/** @brief 上一条事件码的发送时刻(us) */
static int64_t s_last_us = 0;

void voice_init(void)
{
    if (s_inited) {
        return;
    }

    const uart_config_t cfg = {
        .baud_rate  = VOICE_BAUD_RATE,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    /* 安装驱动：接收缓冲留 256 字节备用（当前不读，但保留以免模块回传撑爆 FIFO） */
    ESP_ERROR_CHECK(uart_driver_install(VOICE_UART_NUM, 256, VOICE_TX_BUF_SIZE, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(VOICE_UART_NUM, &cfg));
    ESP_ERROR_CHECK(uart_set_pin(VOICE_UART_NUM,
                                 PIN_VOICE_TX,      /* TX */
                                 PIN_VOICE_RX,      /* RX */
                                 UART_PIN_NO_CHANGE,
                                 UART_PIN_NO_CHANGE));

    s_inited = true;
    ESP_LOGI(TAG, "语音串口就绪: UART%d TX=GPIO%d RX=GPIO%d %d 8N1",
             (int)VOICE_UART_NUM, PIN_VOICE_TX, PIN_VOICE_RX, VOICE_BAUD_RATE);

    /* 上电自检：发一条，确认接线与波特率。天问侧没配这条也不会出声，无害 */
    voice_say_now("VOICE_READY");
}

void voice_say_now(const char *code)
{
    if (!s_inited || code == NULL || code[0] == '\0') {
        return;
    }

    /* 只发事件码本身，不带换行：天问的「串口接收触发事件」是严格字符串比对，
     * 多一个 \n 就对不上，一条都不会播 */
    char buf[40];
    int n = snprintf(buf, sizeof(buf), "%s", code);
    if (n <= 0) {
        return;
    }
    if (n > (int)sizeof(buf)) {
        n = (int)sizeof(buf);
    }

    int written = uart_write_bytes(VOICE_UART_NUM, buf, (size_t)n);
    if (written != n) {
        ESP_LOGW(TAG, "发送不完整: %s (%d/%d)", code, written, n);
        return;
    }

    ESP_LOGI(TAG, "播报: %s", code);

    /* 记录用于去重 */
    strncpy(s_last_code, code, sizeof(s_last_code) - 1);
    s_last_code[sizeof(s_last_code) - 1] = '\0';
    s_last_us = esp_timer_get_time();
}

void voice_say(const char *code)
{
    if (!s_inited || code == NULL || code[0] == '\0') {
        return;
    }

    int64_t now_us = esp_timer_get_time();
    bool same = (strncmp(s_last_code, code, sizeof(s_last_code)) == 0);
    if (same && (now_us - s_last_us) < (int64_t)VOICE_MIN_GAP_MS * 1000LL) {
        return;   /* 同一条事件码间隔太短，丢弃 */
    }

    voice_say_now(code);
}
