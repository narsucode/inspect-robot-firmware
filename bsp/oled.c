/**
 * @file    oled.c
 * @brief   SSD1306 OLED 驱动实现 — I2C 主机写命令/数据
 *
 * 通信协议:
 *   每个 I2C 写操作发送 2 字节:
 *     第 1 字节 = 0x00 → 后续为命令 (寄存器地址/配置)
 *     第 1 字节 = 0x40 → 后续为显示数据 (像素点阵)
 *
 * 初始化流程:
 *   发送 27 字节 SSD1306 初始化序列 → 配置对比度/扫描方向/电荷泵等 →
 *   全屏清空 → 开启显示
 *
 * 依赖: i2c_bus_init() 自动初始化 I2C 总线 (SDA=GPIO8, SCL=GPIO9)
 */

#include "oled.h"
#include "oled_font.h"
#include "oled_picture.h"
#include "i2c_bus.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include <string.h>

/* I2C 配置：与 i2c_bus 组件共用 I2C_NUM_0 总线 */
#define OLED_I2C_ADDR   0x3C
#define OLED_I2C_NUM    I2C_NUM_0

static const char *TAG = "OLED";

/* ── 内存显存 + 影子缓冲 ──────────────────────────────────────────
 * SSD1306 显存为 8 页 × 128 列，每字节 8 个像素。
 * 绘制先写 s_fb（纯内存），oled_flush() 时与 s_shadow 比较，
 * 只把"内容变了"的页通过 I2C 下发。
 *
 * 为什么这么改：原实现每个字节发一次 I2C 事务（2 字节），
 * 整屏 1024 字节 ≈ 300ms，会把 10ms 控制循环堵死。
 * 改后只发改动的页，通常 1~2 页 ≈ 12~24ms。
 */
static uint8_t s_fb[8][128];
static uint8_t s_shadow[8][128];
static bool    s_shadow_valid = false;

/* OLED SSD1306 初始化指令序列 */
static const uint8_t OLED_INIT_CMD[] = {
    0xAE,       /* 关闭显示 */
    0x00, 0x10, /* 设置列地址低/高半字节 */
    0x40,       /* 设置显示起始行 */
    0xB0,       /* 设置页地址 */
    0x81, 0xFF, /* 设置对比度 */
    0xA1,       /* 段重映射（左右镜像） */
    0xA6,       /* 正常显示（非反色） */
    0xA8, 0x3F, /* 多路复用比 64 */
    0xC8,       /* COM 扫描方向（上下翻转） */
    0xD3, 0x00, /* 显示偏移 */
    0xD5, 0x80, /* 显示时钟分频 / 振荡器频率 */
    0xD8, 0x05, /* 设置区域颜色 / 低功耗模式 */
    0xD9, 0xF1, /* 预充电周期 */
    0xDA, 0x12, /* COM 引脚硬件配置 */
    0xDB, 0x30, /* VCOMH 反压级别 */
    0x8D, 0x14, /* 启用电荷泵 */
    0xAF        /* 开启显示 */
};

/* 发送控制命令 */
static esp_err_t oled_write_cmd(uint8_t cmd)
{
    uint8_t buf[2] = {0x00, cmd};
    return i2c_master_write_to_device(OLED_I2C_NUM, OLED_I2C_ADDR, buf, 2, pdMS_TO_TICKS(100));
}

/* 发送显示数据 */
static esp_err_t oled_write_data(uint8_t data)
{
    uint8_t buf[2] = {0x40, data};
    return i2c_master_write_to_device(OLED_I2C_NUM, OLED_I2C_ADDR, buf, 2, pdMS_TO_TICKS(100));
}

/* 初始化 OLED */
void oled_init(void)
{
    /* 先确保 I2C 总线已初始化 */
    ESP_ERROR_CHECK(i2c_bus_init());

    /* 发送初始化指令序列 */
    for (int i = 0; i < sizeof(OLED_INIT_CMD); i++) {
        oled_write_cmd(OLED_INIT_CMD[i]);
    }

    oled_clear();
    ESP_LOGI(TAG, "OLED 初始化完成");
}

/**
 * @brief 把内存显存按页差分刷到屏幕
 *
 * @details 每页一次比较、两次事务（设页命令 + 129 字节数据）。
 *          未变化的页完全跳过，所以频繁刷新也不会长期占用总线。
 */
void oled_flush(void)
{
    if (!s_shadow_valid) {
        memset(s_shadow, 0xFF, sizeof(s_shadow));
        s_shadow_valid = true;
    }

    for (uint8_t page = 0; page < 8; page++) {
        if (memcmp(s_fb[page], s_shadow[page], 128) == 0) {
            continue;   /* 这页没变，跳过 */
        }

        /* 设页地址 + 列地址归零 */
        uint8_t cmd[4] = { 0x00, (uint8_t)(0xB0 + page), 0x00, 0x10 };
        if (i2c_master_write_to_device(OLED_I2C_NUM, OLED_I2C_ADDR,
                                       cmd, sizeof(cmd), pdMS_TO_TICKS(100)) != ESP_OK) {
            return;
        }

        /* 0x40 + 整页 128 字节 */
        uint8_t data[129];
        data[0] = 0x40;
        memcpy(&data[1], s_fb[page], 128);
        if (i2c_master_write_to_device(OLED_I2C_NUM, OLED_I2C_ADDR,
                                       data, sizeof(data), pdMS_TO_TICKS(200)) != ESP_OK) {
            return;
        }

        memcpy(s_shadow[page], s_fb[page], 128);
    }
}

/* 全屏清屏 */
void oled_clear(void)
{
    memset(s_fb, 0x00, sizeof(s_fb));
    memset(s_shadow, 0xFF, sizeof(s_shadow));   /* 强制每页都下发 */
    s_shadow_valid = true;
    oled_flush();
}

/* 设置显示坐标 */
static void oled_set_pos(uint8_t x, uint8_t y)
{
    oled_write_cmd(0xB0 + y);
    oled_write_cmd(((x & 0xF0) >> 4) | 0x10);
    oled_write_cmd(x & 0x0F);
}

/* 显示单个英文字符 */
void oled_show_char(uint8_t x, uint8_t y, char ch, FONT_SIZE f_size)
{
    if (x > 127 || y > 7) return;
    if (ch < ' ' || ch > '~') return;      /* 越界字符直接跳过，防止查表溢出 */
    uint8_t index = (uint8_t)(ch - ' ');

    if (f_size == FONT_16_EN) {
        if (y + 1 > 7) return;
        for (int i = 0; i < 8; i++) {
            if ((uint8_t)(x + i) > 127) break;
            s_fb[y][x + i]     = ANSIC0816[index][i];
            s_fb[y + 1][x + i] = ANSIC0816[index][i + 8];
        }
    } else {
        for (int i = 0; i < 6; i++) {
            if ((uint8_t)(x + i) > 127) break;
            s_fb[y][x + i] = ANSIC0608[index][i];
        }
    }
}

/* 显示英文字符串 */
void oled_show_string(uint8_t x, uint8_t y, char* str, FONT_SIZE f_size)
{
    while (*str) {
        oled_show_char(x, y, *str++, f_size);
        x += f_size;
    }
    oled_flush();       /* 只下发本行改动到的页 */
}

/* 显示图片（目前仅支持欢迎 Logo） */
void oled_show_image(uint8_t xpos, uint8_t ypos, uint8_t x_len, uint8_t y_len, IMAGE_INDEX image_index)
{
    if (image_index != WELCOME_LOGO_ENUM) return;

    for (uint16_t i = 0; i < y_len; i++) {
        uint8_t page = (uint8_t)(ypos + i);
        if (page > 7) break;
        for (uint16_t j = 0; j < x_len; j++) {
            if ((uint8_t)(xpos + j) > 127) break;
            s_fb[page][xpos + j] = WELCOME_LOGO[i * x_len + j];
        }
    }
    oled_flush();
}
