/**
 * @file    app_test.c
 * @brief   裸机硬件自检 — 全部测试逻辑封装于此
 *
 * 功能:
 *   1. 按键 KEY0 切换：电机正反转 + 蜂鸣器间歇响 / 全部停止
 *   2. OLED 显示(4行 x 16px)：电池 / 雷达 / 编码器 / 加速度
 *   3. 串口输出：编码器速度 + 12方向雷达障碍物 + IMU 六轴数据
 *   4. LED 故障指示：正常灭，异常闪（雷达断/IMU 断/编码器无数据）
 *
 * 裸机架构:
 *   - 所有逻辑在 board_test_run() 的超级循环中轮询，无新建 FreeRTOS 任务
 *   - 唯一例外：雷达内部的 x3pro_rx 解析任务（保留不动）
 *   - 主循环基准 10ms，不同模块按不同周期分时执行
 */

#include "app_test.h"

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#include "led.h"
#include "key.h"
#include "beep.h"
#include "motor.h"
#include "encoder.h"
#include "battery.h"
#include "mpu6050.h"
#include "oled.h"
#include "ydlidar_x3pro.h"

static const char *TAG = "TEST";

/* ================================================================
 * 模块级全局状态 — 所有函数直接读写，无需层层传参
 * ================================================================ */
static motor_handle_t   motor_l, motor_r;   /* 左右电机句柄 */
static encoder_handle_t enc_l, enc_r;       /* 左右编码器句柄 */
static bool             motor_running;      /* 电机是否在运行 */
static bool             g_imu_ok;           /* IMU 是否正常初始化 */
static imu_data_t       g_imu;              /* IMU 最新数据 (每 1s 更新) */

/* ================================================================
 * OLED 显示 — 4 行 FONT_16_EN (8×16), 每行最多 16 字符
 *
 * 参数仅传入实时值，IMU/电机等全局状态直接读取。
 * ================================================================ */
static void oled_update(float bat_v,
                         int32_t tick_l, int32_t tick_r,
                         int radar_pts, bool radar_ok)
{
    char buf[32];

    oled_clear();

    /* 第1行: 电池 + 温度 */
    snprintf(buf, sizeof(buf), "BAT:%.1fV T:%.0fC",
             bat_v, g_imu_ok ? g_imu.temp : 0.0f);
    oled_show_string(0, 0, buf, FONT_16_EN);

    /* 第2行: 雷达 + IMU 状态 */
    if (radar_ok) {
        snprintf(buf, sizeof(buf), "RAD:%dpts IMU:%s",
                 radar_pts, g_imu_ok ? "OK" : "NO");
    } else {
        snprintf(buf, sizeof(buf), "RAD:wait IMU:%s",
                 g_imu_ok ? "OK" : "NO");
    }
    oled_show_string(0, 2, buf, FONT_16_EN);

    /* 第3行: 编码器 */
    snprintf(buf, sizeof(buf), "L:%+05" PRId32 " R:%+05" PRId32, tick_l, tick_r);
    oled_show_string(0, 4, buf, FONT_16_EN);

    /* 第4行: 3 轴加速度 (g) */
    snprintf(buf, sizeof(buf), "A:%+.2f%+.2f%+.2f",
             g_imu.ax, g_imu.ay, g_imu.az);
    oled_show_string(0, 6, buf, FONT_16_EN);
}

/* ================================================================
 * 串口输出 — 编码器
 * ================================================================ */
static void serial_encoder(void)
{
    float   speed_l = encoder_get_speed(enc_l);
    float   rpm_l   = encoder_get_rpm(enc_l);
    int32_t tick_l  = encoder_get_ticks(enc_l);

    float   speed_r = encoder_get_speed(enc_r);
    float   rpm_r   = encoder_get_rpm(enc_r);
    int32_t tick_r  = encoder_get_ticks(enc_r);

    printf("[ENC] L 脉冲=%+" PRId32 "  速度=%.3f m/s  RPM=%.1f\n",
           tick_l, speed_l, rpm_l);
    printf("[ENC] R 脉冲=%+" PRId32 "  速度=%.3f m/s  RPM=%.1f\n",
           tick_r, speed_r, rpm_r);
}

/* ================================================================
 * 串口输出 — 雷达 12 方向 (每 30°)
 * ================================================================ */
static void serial_radar(const ydlidar_x3pro_scan_t *scan)
{
    if (!scan || !scan->ready) {
        printf("[RADAR] 暂无数据，等待中...\n");
        return;
    }

    static const int    dirs[] = {0, 30, 60, 90, 120, 150, 180, 210, 240, 270, 300, 330};
    static const char *names[] = {"前","前右","右前","右","右后","后右","后","后左","左后","左","左前","前左"};

    printf("[RADAR] %d 个原始点  12方向障碍:\n", scan->point_count);

    printf("  ");
    for (int i = 0; i < 6; i++) {
        float dist = scan->ranges[dirs[i]];
        if (dist > 0.01f && dist < 8.0f) {
            printf("  %s:%5.1fm", names[i], dist);
        } else {
            printf("  %s:  ---", names[i]);
        }
    }
    printf("\n  ");
    for (int i = 6; i < 12; i++) {
        float dist = scan->ranges[dirs[i]];
        if (dist > 0.01f && dist < 8.0f) {
            printf("  %s:%5.1fm", names[i], dist);
        } else {
            printf("  %s:  ---", names[i]);
        }
    }
    printf("\n");
}

/* ================================================================
 * 串口输出 — 状态汇总
 * ================================================================ */
static void serial_summary(float bat_v,
                            int32_t tick_l, int32_t tick_r,
                            float rpm_l, float rpm_r,
                            bool radar_ok, bool fault)
{
    printf("\n========== 硬件自检 ==========\n");
    printf("  电池: %.2f V    温度: %.1f C\n",
           bat_v, g_imu_ok ? g_imu.temp : 0.0f);
    printf("  IMU 加速度: ax=%.3f ay=%.3f az=%.3f (g)\n",
           g_imu.ax, g_imu.ay, g_imu.az);
    printf("  IMU 角速度: gx=%.2f gy=%.2f gz=%.2f (°/s)\n",
           g_imu.gx, g_imu.gy, g_imu.gz);
    printf("  左轮: %+" PRId32 " ticks  RPM:%.1f\n", tick_l, rpm_l);
    printf("  右轮: %+" PRId32 " ticks  RPM:%.1f\n", tick_r, rpm_r);
    printf("  雷达: %s    电机: %s\n",
           radar_ok ? "正常" : "无数据",
           motor_running ? "运行" : "停止");
    printf("  状态: %s\n", fault ? "⚠ 有故障" : "✓ 一切正常");
    printf("==============================\n\n");
}

/* ================================================================
 * 硬件初始化 — 一次性初始化全部外设
 * ================================================================ */
static void hw_init(void)
{
    /* GPIO 外设 */
    led_init();
    led_off();
    key_init();
    beep_init();

    /* 电机 */
    motor_init(&motor_l, MOTOR_L_A, MOTOR_L_B, 0);
    motor_init(&motor_r, MOTOR_R_A, MOTOR_R_B, 1);

    /* 编码器 */
    encoder_init(&enc_l, 11, 12);
    encoder_init(&enc_r, 48, 47);
    encoder_set_pulses_per_rev(enc_l, 11);
    encoder_set_pulses_per_rev(enc_r, 11);
    encoder_set_distance_per_tick(enc_l, 0.00010336f);
    encoder_set_distance_per_tick(enc_r, 0.00010336f);

    /* 电池 ADC */
    battery_init();

    /* OLED */
    oled_init();
    oled_clear();
    oled_show_string(0, 2, "Board Test", FONT_16_EN);
    oled_show_string(0, 4, "Init...", FONT_16_EN);

    /* MPU6050 */
    g_imu_ok = imu_init();
    if (!g_imu_ok) {
        ESP_LOGW(TAG, "MPU6050 未检测到！");
    }

    /* 激光雷达（内部保留 x3pro_rx 解析任务） */
    ydlidar_x3pro_init(X3PRO_UART_RX, X3PRO_MCTR_GPIO);
    ydlidar_x3pro_start();
    ESP_LOGI(TAG, "雷达已启动，等待首帧...");
}

/* ================================================================
 * 超级循环 — 裸机轮询核心
 * ================================================================ */
static void hw_loop(void)
{
    uint32_t tick  = 0;
    bool     fault = true;           /* 初始算故障 (雷达数据未就绪) */
    float    bat_v = 0.0f;           /* 电池电压缓存 */
    const ydlidar_x3pro_scan_t *scan = NULL;  /* 雷达数据指针 */

    hw_init();

    ESP_LOGI(TAG, "===== 初始化完成，进入超级循环 =====\n");
    vTaskDelay(pdMS_TO_TICKS(2000));  /* 给雷达 2 秒启动 */

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(10));
        tick += 10;

        /* ── 每圈：蜂鸣器 + LED ── */
        beep_handle();
        if (fault) led_flash(10);
        else       led_off();

        /* ── 按键 ── */
        if (key0_read_state() == KEY_STATE_PRESS) {
            motor_running = !motor_running;
            if (motor_running) {
                ESP_LOGI(TAG, "▶ 电机启动");
                motor_set_speed(motor_l,  400);
                motor_set_speed(motor_r, -400);
            } else {
                ESP_LOGI(TAG, "■ 电机停止");
                motor_stop(motor_l);
                motor_stop(motor_r);
                beep_off();
            }
        }

        /* 蜂鸣器间歇 (200ms 交替) */
        if (motor_running) {
            if (((tick / 200) % 2) == 0) beep_on();
            else                         beep_off();
        }

        /* ── 100ms：编码器速度 ── */
        if (tick % 100 == 0) {
            encoder_get_speed(enc_l);
            encoder_get_speed(enc_r);
        }

        /* ── 1000ms：电池 + IMU + 雷达 ── */
        if (tick % 1000 == 0) {
            battery_update();
            bat_v = battery_get_voltage();

            if (g_imu_ok) {
                imu_read(&g_imu);
            }

            scan = ydlidar_x3pro_get_scan();
        }

        /* ── 2000ms：OLED + 串口 + 故障检测 ── */
        if (tick % 2000 == 0) {
            bool    radar_ok  = (scan && scan->ready);
            int     radar_pts = radar_ok ? scan->point_count : 0;
            int32_t tick_l    = encoder_get_ticks(enc_l);
            int32_t tick_r    = encoder_get_ticks(enc_r);
            float   rpm_l     = encoder_get_rpm(enc_l);
            float   rpm_r     = encoder_get_rpm(enc_r);

            /* 故障判定 */
            fault = false;
            if (!radar_ok)  fault = true;
            if (!g_imu_ok)  fault = true;
            static uint32_t motor_t0 = 0;
            if (motor_running && motor_t0 == 0)      motor_t0 = tick;
            else if (!motor_running)                 motor_t0 = 0;
            if (motor_running && (tick - motor_t0 > 3000)
                && rpm_l == 0.0f && rpm_r == 0.0f)   fault = true;

            /* OLED */
            oled_update(bat_v, tick_l, tick_r, radar_pts, radar_ok);

            /* 串口 */
            serial_encoder();
            serial_radar(scan);
            serial_summary(bat_v, tick_l, tick_r, rpm_l, rpm_r, radar_ok, fault);
        }
    }
}

/* ================================================================
 * 公开入口 — 唯一对外 API
 * ================================================================ */
void board_test_run(void)
{
    ESP_LOGI(TAG, "╔══════════════════════════════╗");
    ESP_LOGI(TAG, "║  裸机硬件自检固件 v1.3       ║");
    ESP_LOGI(TAG, "║  LED: 正常灭 / 异常闪        ║");
    ESP_LOGI(TAG, "╚══════════════════════════════╝");

    hw_loop();  /* 永不返回 */
}
