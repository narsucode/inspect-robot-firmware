/**
 * @file    i2c_bus.h
 * @brief   I2C 总线驱动 — 主机模式，引脚见 car_config.h / 100KHz
 *
 * 三步拆解术:
 *   1. 直观理解: I2C 总线就像一条"对讲机专用频道"——
 *      两根线 (SDA 数据 + SCL 时钟)，总线上可以挂多个设备，
 *      每个设备有唯一地址，主机点名谁谁就回话。
 *   2. 追根溯源: 没有 I2C，OLED 屏幕、MPU6050 等设备每个都要
 *      占用好几根 GPIO 线，ESP32 引脚根本不够用。
 *   3. 正规解释: ESP32 硬件 I2C 控制器支持主机/从机模式，
 *      标准速率 100KHz / 快速 400KHz。本驱动使用 I2C_NUM_0，
 *      内部上拉电阻使能，无需外接上拉。
 *      为兼容幻尔 ESP32S3-Cam 视觉模块的 100KHz 从机固件，
 *      本总线统一跑 100KHz (OLED/MPU6050 均兼容)。
 *
 * 使用示例:
 *   @code
 *   i2c_bus_init();  // 初始化总线 (重复调用安全)
 *
 *   // 直接使用 ESP-IDF I2C API 读写设备:
 *   uint8_t buf[2] = {0x00, 0x00};
 *   i2c_master_write_to_device(I2C_NUM_0, 0x3C, buf, 2, 100);
 *   @endcode
 */

#ifndef __I2C_BUS_H__
#define __I2C_BUS_H__

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 初始化 I2C 主机总线 (重复调用安全)
 *
 * 配置: I2C_NUM_0 / SDA / SCL (引脚见 car_config.h) / 100KHz / 内部上拉
 * 若已初始化则打印警告并直接返回 ESP_OK。
 */
esp_err_t i2c_bus_init(void);

/**
 * @brief 获取已初始化的 I2C 端口号
 * @return 固定返回 I2C_NUM_0
 */
int i2c_bus_get_port(void);

#ifdef __cplusplus
}
#endif

#endif
