/**
 * @file    app_test.h
 * @brief   裸机硬件自检应用 — 对外接口
 *
 * 将所有测试逻辑（硬件初始化、超级循环、OLED/串口输出）封装在此，
 * main.c 只需一行调用即可启动全部自检功能。
 */

#ifndef APP_TEST_H
#define APP_TEST_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 启动硬件自检
 *
 * 执行流程:
 *   1. 初始化全部外设 (LED / Key / Beep / Motor / Encoder / Battery / OLED / MPU6050 / Radar)
 *   2. 进入 10ms 超级循环轮询 (不返回)
 *   3. 循环中按不同周期执行: 按键检测 / 蜂鸣器 / 编码器 / 电池 / IMU / 雷达 / OLED / 串口
 *
 * @note  此函数永不返回，内部使用 vTaskDelay 实现裸机风格轮询。
 */
void board_test_run(void);

#ifdef __cplusplus
}
#endif

#endif /* APP_TEST_H */
