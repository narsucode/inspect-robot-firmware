/**
 * @file    vision_task.c
 * @brief   视觉云台任务实现
 *
 * @details
 * 任务周期：VISION_TASK_PERIOD_MS（默认50ms，20Hz）
 * 执行流程：读取I2C从机0x52目标坐标 → 云台PD追踪控制 → 判断目标是否对准画面中心
 *
 * 设计要点：
 * 1. 独立FreeRTOS任务，不阻塞main.c 10ms主循环；
 * 2. 容错机制：视觉模块离线仅打印一次警告，云台保持中位待机，整车基础功能不受影响；
 * 3. 上电自检：遍历读取5个颜色通道+人脸检测数据并输出日志，快速确认目标颜色索引。
 */

#include "vision_task.h"
#include "vision_cam.h"
#include "vision_gimbal.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

static const char *TAG = "vision_task";

/* ================================================================
 * 任务基础配置
 * ================================================================ */
#define VISION_TASK_NAME        "vision_loop"
#define VISION_TASK_STACK       4096
#define VISION_TASK_PRIO        3
#define VISION_TASK_CORE        0       /* Core1分配给雷达解析任务，避免抢占冲突 */
#define VISION_TASK_PERIOD_MS   GIMBAL_CTRL_PERIOD_MS

/* ================================================================
 * 视觉目标通道配置
 * ================================================================ */
/* 目标颜色通道索引 0~4
 * 上电日志会打印全部5通道数据，根据日志修改此索引锁定对应颜色 */
#define VISION_COLOR_IDX        0

/* 上电扫描全部通道调试开关，用于确认通道索引，不影响正常运行逻辑 */
#define VISION_DEBUG_SCAN       1

static bool s_started = false;
static TaskHandle_t s_task = NULL;

/**
 * @brief vision_debug_scan — 上电自检，遍历读取5个颜色通道与人脸检测信息
 * @note 用于调试，快速确认哪个通道识别到目标色块
 */
static void vision_debug_scan(void)
{
    vision_block_t b;
    for (uint8_t i = 0; i < 5; i++) {
        if (vision_cam_read_color(i, &b) == ESP_OK) {
            ESP_LOGI(TAG, "颜色通道 %u: x=%u y=%u w=%u h=%u", i, b.x, b.y, b.w, b.h);
        } else {
            ESP_LOGW(TAG, "颜色通道 %u: 读取失败", i);
        }
        vTaskDelay(pdMS_TO_TICKS(20));
    }
    if (vision_cam_read_face(&b) == ESP_OK) {
        ESP_LOGI(TAG, "人脸: x=%u y=%u w=%u h=%u", b.x, b.y, b.w, b.h);
    } else {
        ESP_LOGW(TAG, "人脸: 读取失败");
    }
}

/**
 * @brief vision_loop — 视觉云台任务主循环
 * @param arg 任务传入参数，未使用
 * @note 周期轮询I2C视觉模块，自动切换视觉追踪/遥控模式
 */
static void vision_loop(void *arg)
{
    (void)arg;
    bool aimed_reported = false;

    /* 1.初始化云台外设，即使视觉模块离线，舵机也保持中位待机 */
    vision_gimbal_init();

    /* 2.初始化视觉摄像头模块；初始化失败仅告警，不中断整车运行 */
    bool cam_ok = (vision_cam_init() == ESP_OK);
    if (!cam_ok) {
        ESP_LOGW(TAG, "视觉模块不在线 → 云台待机; 主功能不受影响");
    }

    /* 3.上电调试扫描，打印全部颜色通道与人脸数据 */
#if VISION_DEBUG_SCAN
    if (cam_ok) {
        vision_debug_scan();
    }
#endif

    uint32_t ok_cnt = 0, lost_cnt = 0;

    for (;;) {
        /* 遥控模式判断：云台交由microROS /cmd_ptz_angle控制，暂停视觉追踪 */
        if (vision_gimbal_is_teleop()) {
            aimed_reported = false;
            vTaskDelay(pdMS_TO_TICKS(VISION_TASK_PERIOD_MS));
            continue;
        }

        vision_block_t b;
        esp_err_t ret = cam_ok ? vision_cam_read_color(VISION_COLOR_IDX, &b) : ESP_FAIL;

        if (ret == ESP_OK && b.w > 0 && b.h > 0) {
            /* 成功读取目标色块，送入云台PD追踪 */
            vision_gimbal_track(b.x, b.y);
            ok_cnt++;
            lost_cnt = 0;

            /* 目标稳定对准画面中心，打印一次对准提示 */
            if (vision_gimbal_is_aimed()) {
                if (!aimed_reported) {
                    ESP_LOGI(TAG, "已对准 (目标 %u,%u 尺寸 %ux%u)", b.x, b.y, b.w, b.h);
                    aimed_reported = true;
                }
            } else {
                aimed_reported = false;
            }

            /* 每约3.2s打印一次目标位置调试日志 */
            if ((ok_cnt & 0x3F) == 1) {
                ESP_LOGD(TAG, "目标 %u,%u 尺寸 %ux%u", b.x, b.y, b.w, b.h);
            }
        } else {
            /* 目标丢失：云台保持当前角度不动，不自动扫描，防止乱转 */
            vision_gimbal_lost(true);
            aimed_reported = false;
            lost_cnt++;
            /* 连续丢失20帧（约1s）打印一次丢失提示 */
            if (lost_cnt == 20) {
                ESP_LOGI(TAG, "目标丢失, 云台保持不动");
            }
        }

        vTaskDelay(pdMS_TO_TICKS(VISION_TASK_PERIOD_MS));
    }
}

/**
 * @brief vision_task_start — 创建并启动视觉云台任务
 * @return esp_err_t ESP_OK成功；ESP_FAIL任务创建失败
 */
esp_err_t vision_task_start(void)
{
    if (s_started) {
        ESP_LOGW(TAG, "视觉任务已在运行");
        return ESP_OK;
    }

    BaseType_t ok = xTaskCreatePinnedToCore(vision_loop, VISION_TASK_NAME,
                                            VISION_TASK_STACK, NULL,
                                            VISION_TASK_PRIO, &s_task,
                                            VISION_TASK_CORE);
    if (ok != pdPASS) {
        ESP_LOGE(TAG, "创建视觉任务失败");
        return ESP_FAIL;
    }

    s_started = true;
    ESP_LOGI(TAG, "视觉任务已启动 (T=%dms, Core%d)", VISION_TASK_PERIOD_MS, VISION_TASK_CORE);
    return ESP_OK;
}