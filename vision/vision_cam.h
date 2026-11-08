/**
 * @file    vision_cam.h
 * @brief   幻尔 ESP32S3-Cam 视觉模块驱动 (I2C 从机 0x52)
 *
 * @details
 *   1. 直观理解: 视觉模块是一个"用 I2C 回答问题的摄像头"——
 *      主机问"人脸在哪"(寄存器 0x01)、"红色块在哪"(寄存器 0x00)，
 *      它回 4 个字节: 中心 x、中心 y、宽、高。
 *   2. 追根溯源: 图像本身走 WiFi(HTTP/MJPEG)，不走 I2C；
 *      I2C 只传"识别结果的坐标"，一次 4 字节，代价极低。
 *   3. 正规解释: 模块内部跑识别固件(color_detection_web 等)，
 *      以 I2C 从机身份暴露寄存器；主机先写寄存器地址，再重复起始位读 4 字节。
 *
 * 接线: 复用板载 I2C 总线 (SDA=GPIO8 / SCL=GPIO9, 见 car_config.h)，
 *       模块 IO47→SDA, IO48→SCL, GND 共地, 5V/2A 独立供电。
 *
 * @note  模块从机固件按 100kHz 实现，因此 i2c_bus 的 400kHz 需要降到 100kHz。
 *
 * @usage
 *   vision_cam_init();                       // 探测 0x52
 *   vision_block_t b;
 *   if (vision_cam_read_color(1, &b) == ESP_OK && b.w) { ... }   // 蓝色块
 */

#ifndef __VISION_CAM_H__
#define __VISION_CAM_H__

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ── 模块参数 ── */
#define VISION_I2C_ADDR         0x52    /**< 模块 I2C 从机地址 */
#define VISION_REG_FACE         0x01    /**< 人脸: 4 字节 x,y,w,h */
#define VISION_REG_COLOR_BASE   0x00    /**< 颜色基数: 0x00~0x04 每个 4 字节 */
#define VISION_REG_LIGHT        0x11    /**< 补光灯亮度 (写) */
#define VISION_FRAME_CX         120     /**< 识别固件画面中心 (240x240) */
#define VISION_FRAME_CY         120
#define VISION_READ_TIMEOUT_MS  100

/** @brief 一块识别结果: 中心坐标 + 宽高 (单位: 像素) */
typedef struct {
    uint8_t x;
    uint8_t y;
    uint8_t w;
    uint8_t h;
} vision_block_t;

/**
 * @brief 初始化: 复用板载 I2C 总线并探测模块
 * @return ESP_OK=找到 0x52; ESP_ERR_NOT_FOUND=没找到(检查接线/固件/供电)
 */
esp_err_t vision_cam_init(void);

/** @brief 探测 0x52 是否存在 (只发地址看 ACK) */
esp_err_t vision_cam_probe(void);

/** @brief 读任意寄存器 len 字节 */
esp_err_t vision_cam_read_reg(uint8_t reg, uint8_t *buf, size_t len);

/** @brief 读人脸 (寄存器 0x01); res=ESP_OK 且 w/h 非 0 表示识别到 */
esp_err_t vision_cam_read_face(vision_block_t *out);

/** @brief 读颜色通道 (idx 0~4); res=ESP_OK 且 w/h 非 0 表示识别到 */
esp_err_t vision_cam_read_color(uint8_t idx, vision_block_t *out);

/** @brief 写补光灯亮度 (寄存器 0x11, 0~255) */
esp_err_t vision_cam_set_light(uint8_t level);

#ifdef __cplusplus
}
#endif

#endif /* __VISION_CAM_H__ */
