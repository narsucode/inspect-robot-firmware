/**
 * @file    oled.h
 * @brief   SSD1306 OLED 驱动 — I2C 接口, 128×64 像素
 *
 * 三步拆解术:
 *   1. 直观理解: OLED 就是机器人的"小电视"——
 *      128×64 个像素点排成 8 行 (页) × 128 列，每个像素可以独立开关。
 *      显示文字就是把字库里的点阵图案"画"到对应像素上。
 *   2. 追根溯源: 没有屏幕，你只能靠串口打印和 LED 指示灯来判断状态——
 *      拔掉 USB 线就彻底瞎子。OLED 让你随时随地看到电池、IMU、雷达数据。
 *   3. 正规解释: SSD1306 是 128×64 单色 OLED 驱动芯片，I2C 地址 0x3C。
 *      屏幕分 8 页 (Page 0~7)，每页 8 像素高 × 128 像素宽。
 *      字符显示依赖字库 (oled_font.h) 将 ASCII 码映射为像素数据。
 *
 * 坐标系:
 *   ┌──────────────────────┐
 *   │ y=0  Page 0 (0~7px)  │  ← FONT_8_EN 可写满 8 行
 *   │ y=1  Page 1 (8~15px) │     每行最多 21 字符 (6px/字)
 *   │ y=2  Page 2          │  ← FONT_16_EN 可写 4 行 (y=0,2,4,6)
 *   │ y=3  Page 3          │     每行最多 16 字符 (8px/字)
 *   │ y=4  Page 4          │
 *   │ y=5  Page 5          │
 *   │ y=6  Page 6          │
 *   │ y=7  Page 7 (56~63px)│
 *   └──────────────────────┘
 *     x=0 → → → → → x=127
 */

#ifndef __OLED_H
#define __OLED_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** 字体大小枚举 — 值 = 字符宽度 (像素) */
typedef enum {
    FONT_8_EN  = 6,          /**< 6×8 像素 ASCII 字体 */
    FONT_16_EN = 8           /**< 8×16 像素 ASCII 字体 */
} FONT_SIZE;

/** 内置图片索引 */
typedef enum {
    WELCOME_LOGO_ENUM = 0    /**< 欢迎 Logo (128×64 全屏) */
} IMAGE_INDEX;

/**
 * @brief 初始化 OLED (自动初始化 I2C 总线)
 *
 * 发送 SSD1306 初始化指令序列 → 清屏 → 开启显示。
 * 会先调用 i2c_bus_init() 确保总线就绪。
 */
void oled_init(void);

/** @brief 全屏清空 (所有像素灭) */
void oled_clear(void);

/**
 * @brief 把内存显存按页差分刷到屏幕（未变化的页跳过）
 */
void oled_flush(void);

/**
 * @brief 在指定坐标显示单个 ASCII 字符
 * @param x      列 (0~127)
 * @param y      页 (0~7, 每页 8 像素高)
 * @param ch     ASCII 字符
 * @param f_size 字体大小 (FONT_8_EN 或 FONT_16_EN)
 * @note  超出边界 (x>127 或 y>7) 自动忽略
 */
void oled_show_char(uint8_t x, uint8_t y, char ch, FONT_SIZE f_size);

/**
 * @brief 在指定坐标显示 ASCII 字符串
 * @param x      起始列
 * @param y      起始页
 * @param str    字符串 (以 '\0' 结尾)
 * @param f_size 字体大小
 * @note  超出屏幕右边界自动截断 (oled_show_char 内部做边界检查)
 */
void oled_show_string(uint8_t x, uint8_t y, char* str, FONT_SIZE f_size);

/**
 * @brief 显示内置图片
 * @param xpos   起始列
 * @param ypos   起始页
 * @param x_len  图片宽度 (像素)
 * @param y_len  图片高度 (页)
 * @param image  图片索引 (目前仅 WELCOME_LOGO_ENUM)
 */
void oled_show_image(uint8_t xpos, uint8_t ypos, uint8_t x_len,
                     uint8_t y_len, IMAGE_INDEX image);

#ifdef __cplusplus
}
#endif

#endif
