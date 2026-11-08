/**
 * @file    vision_task.h
 * @brief   视觉云台任务 — 把"读坐标 → 控云台"做成独立 FreeRTOS 任务
 *
 * 设计原则: 不侵入老师 main.c 的 10ms 周期主循环, 只在 app_main() 里
 *           调用一次 vision_task_start(); 视觉缺失/失败不影响整车运行
 *           (/odom /scan /cmd_vel 等主功能照常)。
 */

#ifndef __VISION_TASK_H__
#define __VISION_TASK_H__

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/** @brief 启动视觉云台任务 (重复调用安全, 已在运行则直接返回 ESP_OK) */
esp_err_t vision_task_start(void);

#ifdef __cplusplus
}
#endif

#endif /* __VISION_TASK_H__ */
