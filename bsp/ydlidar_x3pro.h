/**
 * @file    ydlidar_x3pro.h
 * @brief   YDLIDAR X3 Pro 激光雷达 ESP-IDF 驱动 —— 头文件
 *
 * 适用芯片:  ESP32-S3
 * RTOS:      ESP-IDF v5.3
 * 波特率:    115200 bps (8 数据位 / 1 停止位 / 无校验)
 * 测距频率:  4000 Hz
 * 扫描频率:  默认 6 Hz (PWM 50% @ 10KHz)
 * 每圈点数:  ~445 (原始) → 规整为 360 点 (ROS 标准格式)
 * 协议兼容:  YDLIDAR X2 / X3 / X4 系列 (0x55AA 帧头 + 三角测距)
 *
 * ──────────── 硬件接线 (ESP32-S3) ────────────
 *   雷达 VCC   → ESP32 5V
 *   雷达 GND   → ESP32 GND
 *   雷达 Tx    → ESP32 IO18 (UART1 RX, 第 2 脚)
 *   雷达 M_CTR → ESP32 IO17 (PWM 输出, 第 4 脚)
 *
 * ⚠️ 注意事项:
 *   1. M_CTR 必须使用 PWM (10KHz)，不能用 GPIO 恒高电平，否则数据乱码
 *   2. X3 Pro 仅有 Tx 单向输出, 无 Rx 引脚, 无需也无法向雷达发送指令
 *   3. 雷达上电 + M_CTR PWM → 自动开始测距并持续输出数据
 *
 * ──────────── 修改引脚或波特率 ────────────
 *   只改下面三个宏即可, 无需修改 .c 文件:
 */

#ifndef YDLIDAR_X3PRO_H
#define YDLIDAR_X3PRO_H

#include <stdint.h>
#include <stdbool.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "driver/uart.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ═════════════════════ 硬件配置 ═════════════════════ */

#define X3PRO_BAUDRATE          115200       /* 波特率 */
#define X3PRO_UART_NUM          UART_NUM_1   /* 使用的串口号 */
#define X3PRO_UART_RX           18           /* 接雷达 Tx (第 2 脚) */
#define X3PRO_MCTR_GPIO         17           /* 接雷达 M_CTR (第 4 脚) */

/* ═════════════════════ 协议常量 ═════════════════════ */

#define X3PRO_PH                   0x55AA   /* 数据包包头 (线上顺序: AA 55) */
#define X3PRO_POINT_SIZE           2        /* 每个采样点 2 字节 */
#define X3PRO_MAX_POINTS_PER_PKT   200      /* 单包最大采样点数 (安全上限) */
#define X3PRO_MAX_SCAN_POINTS      800      /* 每圈最大原始点数 (安全上限) */
#define X3PRO_PWM_FREQ             10000    /* M_CTR PWM 频率 (Hz) */

/* ═════════════════════ 数据类型 ═════════════════════ */

/**
 * @brief 一圈完整的 360° 扫描数据
 *
 * 包含两组数据:
 *   1. 原始采样点 (distances / angles)  —— 约 445 个，间隔 ~0.81°
 *   2. ROS 标准格式 (ranges)             —— 360 个，固定 1° 间隔
 *
 * 用法示例:
 * @code
 *   const ydlidar_x3pro_scan_t *s = ydlidar_x3pro_wait_scan(2000);
 *   if (s) {
 *       // 发布 ROS 消息时直接用 ranges 数组:
 *       memcpy(msg.ranges.data, s->ranges, 360 * sizeof(float));
 *
 *       // 需要原始角度时遍历:
 *       for (int i = 0; i < s->point_count; i++)
 *           printf("%.1f° %.3fm\n", s->angles[i], s->distances[i]);
 *   }
 * @endcode
 */
typedef struct {
    /* 原始数据 (约 445 个点, 间隔 0.81°) */
    float   distances[X3PRO_MAX_SCAN_POINTS];   /* 距离, 单位: 米 */
    float   angles[X3PRO_MAX_SCAN_POINTS];      /* 角度, 单位: °, 范围 [0, 360) */
    int     point_count;                        /* 本圈原始采样点数 */

    /* ROS 标准 360 点格式 (1° 间隔, 多源合并取最近值) */
    float   ranges[360];                        /* 距离, 单位: 米, 0°=正前方 */
    uint8_t ranges_filled[360];                 /* ranges[i] 是否被填充过 */

    /* 信号强度 (X3 Pro 不支持，保留为 0) */
    float   intensities[X3PRO_MAX_SCAN_POINTS];

    /* 内部控制字段 (用户不直接修改) */
    bool    ready;                              /* 本圈数据是否完整就绪 */
    int64_t timestamp_us;                       /* 采集完成时刻 (微秒) */
} ydlidar_x3pro_scan_t;

/* ═════════════════════ 公开 API ═════════════════════ */

/**
 * @brief 初始化雷达驱动
 *
 * 配置 UART1 (115200 bps / 8N1) 和 LEDC PWM (GPIO, 10KHz),
 * 启动后台解析协程 (Core 1, 优先级 20).
 *
 * @param rx_gpio   接雷达 Tx 引脚的 GPIO 编号
 * @param mctr_gpio 接雷达 M_CTR 引脚的 GPIO 编号
 */
void ydlidar_x3pro_init(int rx_gpio, int mctr_gpio);

/**
 * @brief 启动雷达扫描
 *
 * 输出 PWM 50% 占空比 → 电机以 ~6 Hz 旋转 → 雷达自动开始测距输出数据.
 * 阻塞约 3 秒等待电机达到稳定转速.
 */
void ydlidar_x3pro_start(void);

/**
 * @brief 停止雷达扫描
 *
 * 发送停止指令 + PWM 占空比归零 → 电机停止.
 */
void ydlidar_x3pro_stop(void);

/**
 * @brief 非阻塞获取最新一圈完整数据
 *
 * @return 最新扫描数据指针, 无新数据时返回 NULL.
 * @note   返回的指针在下一圈数据到来前有效.
 */
const ydlidar_x3pro_scan_t *ydlidar_x3pro_get_scan(void);

/**
 * @brief 阻塞等待下一圈完整数据
 *
 * @param timeout_ms 超时时间 (毫秒), 0 表示永久等待.
 * @return 扫描数据指针, 超时返回 NULL.
 */
const ydlidar_x3pro_scan_t *ydlidar_x3pro_wait_scan(uint32_t timeout_ms);

/**
 * @brief 设置电机转速 (PWM 占空比)
 *
 * @param pct 占空比百分比 0~100.  0=停止, 50≈6Hz, 80≈10Hz.
 * @note  转速与占空比非线性, 实际扫描频率可通过雷达输出数据包中的频
 *        率字段读取.
 */
void ydlidar_x3pro_set_speed(uint8_t pct);

/**
 * @brief 雷达调试测试: 初始化 + 启动 + 每 5 秒打印全景
 *
 * 阻塞运行, 不会返回. 用于快速验证硬件.
 *
 * @param rx_gpio   接雷达 Tx 引脚的 GPIO 编号
 * @param mctr_gpio 接雷达 M_CTR 引脚的 GPIO 编号
 */
void ydlidar_x3pro_test(int rx_gpio, int mctr_gpio);

#ifdef __cplusplus
}
#endif

#endif /* YDLIDAR_X3PRO_H */
