/**
 * @file    vision_cam.c
 * @brief   幻尔 ESP32S3-Cam 视觉模块驱动实现 (I2C 从机 0x52)
 *
 * 复用 bsp/i2c_bus.c 已初始化的总线 (I2C_NUM_0)
 * 使用 legacy I2C API (driver/i2c.h)
 */

#include "vision_cam.h"
#include "i2c_bus.h"
#include "driver/i2c.h"
#include "esp_log.h"

static const char *TAG = "vision_cam";
static bool s_inited = false;  // 视觉模块初始化标记

/*
 * @brief   I2C 读取寄存器函数：先写寄存器地址，再用重复起始位读取 len 字节数据
 * @param   reg     要读取的模块内部寄存器地址
 * @param   buf     接收读到数据的缓冲区指针
 * @param   len     需要读取的字节数量
 * @return  esp_err_t ESP_OK成功，其他为错误码
 */
esp_err_t vision_cam_read_reg(uint8_t reg, uint8_t *buf, size_t len)
{
    if (!s_inited || buf == NULL || len == 0) {
        return ESP_ERR_INVALID_STATE;
    }

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();       // 创建I2C命令链表
    i2c_master_start(cmd);                              // 发送I2C起始信号
    i2c_master_write_byte(cmd, (VISION_I2C_ADDR << 1) | I2C_MASTER_WRITE, true); // 发送从机地址+写方向
    i2c_master_write_byte(cmd, reg, true);              // 发送要访问的寄存器编号
    i2c_master_start(cmd);                              // 重复起始位，不释放总线，切换为读
    i2c_master_write_byte(cmd, (VISION_I2C_ADDR << 1) | I2C_MASTER_READ, true);  // 从机地址+读方向
    if (len > 1) {
        i2c_master_read(cmd, buf, len - 1, I2C_MASTER_ACK); // 前面N-1个字节读完回复ACK，继续读
    }
    i2c_master_read_byte(cmd, buf + len - 1, I2C_MASTER_NACK); // 最后一个字节回复NACK，通知从机结束传输
    i2c_master_stop(cmd);                               // I2C停止信号

    esp_err_t ret = i2c_master_cmd_begin(i2c_bus_get_port(), cmd,
                                         pdMS_TO_TICKS(VISION_READ_TIMEOUT_MS)); // 执行I2C命令，带超时
    i2c_cmd_link_delete(cmd);                           // 释放命令链表资源

    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "读寄存器 0x%02X 失败: %s", reg, esp_err_to_name(ret));
    }
    return ret;
}

/*
 * @brief   I2C设备探测，检查视觉模块是否在线
 * @return  esp_err_t ESP_OK代表模块应答，设备存在
 */
esp_err_t vision_cam_probe(void)
{
    if (!s_inited) {
        return ESP_ERR_INVALID_STATE;
    }
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (VISION_I2C_ADDR << 1) | I2C_MASTER_WRITE, true); // 发送从机写地址，等待ACK
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(i2c_bus_get_port(), cmd,
                                         pdMS_TO_TICKS(VISION_READ_TIMEOUT_MS));
    i2c_cmd_link_delete(cmd);
    return ret;
}

/*
 * @brief   视觉模块驱动初始化入口
 * @return  esp_err_t ESP_OK探测到模块；非OK代表总线正常但模块未应答
 */
esp_err_t vision_cam_init(void)
{
    /* 复用板载I2C总线（i2c_bus内部有防重复初始化逻辑） */
    esp_err_t ret = i2c_bus_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C 总线初始化失败: %s", esp_err_to_name(ret));
        return ret;
    }

    s_inited = true; // 标记驱动已就绪

    ret = vision_cam_probe(); // 探测视觉模块
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "视觉模块在线 (0x%02X)", VISION_I2C_ADDR);
    } else {
        ESP_LOGW(TAG, "未找到视觉模块 (0x%02X): 检查 SDA/SCL 接线、共地、5V/2A 供电、以及模块是否烧了识别固件",
                 VISION_I2C_ADDR);
    }
    return ret;
}

/*
 * @brief   内部封装函数：读取4字节包围盒数据，解析为vision_block_t
 * @param   reg     目标寄存器地址
 * @param   out     输出包围盒结构体指针
 * @return  esp_err_t
 */
static esp_err_t read_block(uint8_t reg, vision_block_t *out)
{
    uint8_t buf[4] = {0};
    esp_err_t ret = vision_cam_read_reg(reg, buf, sizeof(buf));
    if (ret != ESP_OK) {
        return ret;
    }
    out->x = buf[0]; // 识别框左上角X坐标
    out->y = buf[1]; // 识别框左上角Y坐标
    out->w = buf[2]; // 识别框宽度
    out->h = buf[3]; // 识别框高度
    return ESP_OK;
}

/*
 * @brief   读取人脸检测包围盒
 * @param   out     输出人脸框结构体指针
 * @return  esp_err_t
 */
esp_err_t vision_cam_read_face(vision_block_t *out)
{
    if (out == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    return read_block(VISION_REG_FACE, out);
}

/*
 * @brief   读取指定编号色块识别包围盒
 * @param   idx     色块序号，范围0~4
 * @param   out     输出色块框结构体指针
 * @return  esp_err_t
 */
esp_err_t vision_cam_read_color(uint8_t idx, vision_block_t *out)
{
    if (out == NULL || idx > 4) {
        return ESP_ERR_INVALID_ARG;
    }
    return read_block(VISION_REG_COLOR_BASE + idx, out);
}

/*
 * @brief   设置视觉模块板载补光灯亮度
 * @param   level   亮度档位值
 * @return  esp_err_t
 */
esp_err_t vision_cam_set_light(uint8_t level)
{
    if (!s_inited) {
        return ESP_ERR_INVALID_STATE;
    }
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (VISION_I2C_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, VISION_REG_LIGHT, true); // 选择灯光控制寄存器
    i2c_master_write_byte(cmd, level, true);            // 写入亮度值
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(i2c_bus_get_port(), cmd,
                                         pdMS_TO_TICKS(VISION_READ_TIMEOUT_MS));
    i2c_cmd_link_delete(cmd);
    return ret;
}