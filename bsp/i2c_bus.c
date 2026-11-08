/**
 * @file    i2c_bus.c
 * @brief   I2C 总线驱动实现 — 主机模式, 100KHz, 自动防重复初始化
 *
 * 配置: I2C_NUM_0 / SDA=GPIO8 / SCL=GPIO9 / 内部上拉
 *
 * 防重复初始化: 静态标志 s_initialized 确保多次调用 i2c_bus_init()
 * 不会重复配置硬件。多个外设 (OLED, MPU6050, 视觉模块) 共享同一条 I2C 总线。
 *
 */

#include "i2c_bus.h"
#include "car_config.h"     /**< PIN_I2C_SDA / PIN_I2C_SCL 引脚定义 */
#include "driver/i2c.h"
#include "esp_log.h"

static const char *TAG = "i2c_bus";

#define I2C_MASTER_NUM      I2C_NUM_0        /**< 使用ESP32-S3硬件I2C0外设端口 */
#define I2C_MASTER_SDA_IO   PIN_I2C_SDA      /**< I2C数据线SDA引脚，取自全局硬件配置 */
#define I2C_MASTER_SCL_IO   PIN_I2C_SCL      /**< I2C时钟线SCL引脚，取自全局硬件配置 */
#define I2C_MASTER_FREQ_HZ  100000           /**< I2C通信时钟100KHz，兼容视觉模块从机 */

static bool s_initialized = false; /**< I2C总线初始化标记，防止重复初始化 */

/* =============================================================================
 * I2C总线初始化
 * ========================================================================== */
/**
 * @brief 初始化 I2C 主机总线，并配置 SDA/SCL 引脚与时钟频率。
 * @retval ESP_OK        初始化成功 / 已经初始化
 * @retval 其他esp_err   硬件配置失败
 */
esp_err_t i2c_bus_init(void)
{
    /* 判断是否已经初始化，避免重复调用导致异常 */
    if (s_initialized) {
        ESP_LOGW(TAG, "I2C 总线已经初始化，跳过重复配置");
        return ESP_OK;
    }

    /**< I2C主机配置结构体 */
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,                /**< 主机模式 */
        .sda_io_num = I2C_MASTER_SDA_IO,        /**< SDA引脚号 */
        .scl_io_num = I2C_MASTER_SCL_IO,        /**< SCL引脚号 */
        .sda_pullup_en = GPIO_PULLUP_ENABLE,    /**< SDA启用内部上拉 */
        .scl_pullup_en = GPIO_PULLUP_ENABLE,    /**< SCL启用内部上拉 */
        .master.clk_speed = I2C_MASTER_FREQ_HZ, /**< I2C时钟频率 */
    };

    /**< 设置I2C参数，不安装驱动 */
    esp_err_t ret = i2c_param_config(I2C_MASTER_NUM, &conf);
    if (ret != ESP_OK) {
        return ret;
    }

    /**< 安装I2C驱动，不使用RX/TX缓冲区 */
    ret = i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
    if (ret == ESP_OK) {
        s_initialized = true;
        ESP_LOGI(TAG, "I2C 总线初始化成功，SDA=GPIO%d，SCL=GPIO%d，%dHz",
                 I2C_MASTER_SDA_IO, I2C_MASTER_SCL_IO, I2C_MASTER_FREQ_HZ);
    }

    return ret;
}

/* =============================================================================
 * 获取I2C端口号
 * ========================================================================== */
/**
 * @brief 返回当前使用的 I2C 端口编号。
 * @return I2C端口号(I2C_NUM_0)
 */
int i2c_bus_get_port(void)
{
    return I2C_MASTER_NUM;
}