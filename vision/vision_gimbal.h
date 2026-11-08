/**
 * @file    vision_gimbal.h
 * @brief   二自由度云台控制 (LEDC 硬件 PWM + 纯 PD + 三重低通滤波 + 角度限位)
 *
 * @details
 *   1. 直观理解: 把"目标在画面里偏了多少像素"变成"舵机转多少度、多少微秒"，
 *      目标是让识别到的色块/人脸始终待在画面中心 (120,120)。
 *   2. 追根溯源: 闭环必须在本 MCU 完成——走 WiFi 到上位机再回来，
 *      抖动和延迟会让云台抖成筛子；本地 20Hz 闭环最稳。
 *   3. 正规解释: 控制律照搬亚博 MicroROS-V2 的 color_tracker_ptz:
 *      目标中心低通 → 误差(死区) → 归一化 → PD(无积分) → 单步限幅
 *      → 步长低通 → 累加角度(限位) → 角度低通 → 角度换算脉宽 → LEDC。
 *      三重低通是"不抖"的关键；俯仰限位不对称是为了不撞机械底座。
 *
 * 引脚: PIN_GIMBAL_PAN / PIN_GIMBAL_TILT 在 bsp/car_config.h 中定义。
 * 供电: 舵机必须独立 5V/3A，且与主板共地（从主板取电会拉复位！）。
 *
 * @usage
 *   vision_gimbal_init();
 *   每 50ms 一次 (识别到目标时): vision_gimbal_track(b.x, b.y);
 *   已对准后可触发抓拍/上报: vision_gimbal_is_aimed() 为 true 时
 */

#ifndef __VISION_GIMBAL_H__
#define __VISION_GIMBAL_H__

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ── 舵机电气参数 (SG90/MG90S 通用) ── */
#define GIMBAL_PWM_FREQ_HZ      50      /**< 舵机标准 50Hz (周期 20ms) */
#define GIMBAL_LEDC_BITS        14      /**< LEDC 分辨率位数; ESP32-S3 上限 14 位 → 脉宽步进约 1.2us */
#define GIMBAL_US_MIN           500     /**< 脉宽下限 (us) */
#define GIMBAL_US_MAX           2500    /**< 脉宽上限 (us) */
#define GIMBAL_PAN_CENTER       1500    /**< 水平中位 (us) */
#define GIMBAL_TILT_CENTER      1500    /**< 俯仰中位 (us) */

/* ── 角度 ↔ 脉宽换算 ──
 * 180° 行程舵机 (SG90/MG90S 常见): (2500-500)/180 ≈ 11.1 us/度  ← 默认
 * 270° 行程舵机:                   (2500-500)/270 ≈  7.4 us/度
 * 说明: 本工程云台限位是 ±90°, 乘 11.1 正好铺满 500~2500us, 所以默认取 11.1;
 *       若实测"给 2500us 明显不到 90°"则说明是 270° 舵机, 改成 7.4。 */
#define GIMBAL_US_PER_DEG       11.1f

/* ── 控制参数 (数值照亚博 color_tracker_ptz.yaml) ──
 * 注意: KP/KD 作用在"归一化误差"(误差 ÷ 画面半宽)上, 不是直接乘像素,
 *       所以换分辨率/换画面尺寸都不用重调参。 */
#define GIMBAL_KP               0.7f    /**< 比例增益 (亚博 pd_kp) */
#define GIMBAL_KD               0.25f   /**< 微分增益 (亚博 pd_kd, 按每秒变化率) */
#define GIMBAL_MAX_STEP_DEG     1.2f    /**< 单步最大角度增量, 度 (亚博 max_ptz_step) */

/* ── 三重低通滤波系数 (0~1, 越小越平滑、越迟钝) ── */
#define GIMBAL_TARGET_SMOOTH_ALPHA  0.35f   /**< 目标中心滤波 (亚博 target_smooth_alpha) */
#define GIMBAL_STEP_SMOOTH_ALPHA    0.35f   /**< 单步增量滤波 (亚博 ptz_step_smooth_alpha) */
#define GIMBAL_ANGLE_SMOOTH_ALPHA   0.25f   /**< 角度输出滤波 (亚博 ptz_smooth_alpha) */

/* ── 角度限位 (度) ──
 * 俯仰不对称: 防止低头撞底座 / 抬头拉断线。
 * 亚博为 水平 ±90, 俯仰 -90~+20; 硬件装好后按实际机械行程改。 */
#define GIMBAL_PAN_MIN_DEG      (-90.f)
#define GIMBAL_PAN_MAX_DEG      ( 90.f)
#define GIMBAL_TILT_MIN_DEG     (-90.f)
#define GIMBAL_TILT_MAX_DEG     ( 90.f)

#define GIMBAL_DEADZONE_PX      6       /**< 误差死区(像素): 在此范围内不动, 防抖 */
#define GIMBAL_AIM_FRAMES       5       /**< 连续 N 帧在死区内 → 判定"已对准" */
#define GIMBAL_CTRL_PERIOD_MS   50      /**< 控制周期 (ms): 与视觉任务一致, 20Hz */

/* ── 画面中心 (识别固件 240x240) ── */
#define GIMBAL_FRAME_CX         120
#define GIMBAL_FRAME_CY         120

/* ── 方向适配: 装好后发现转反了就把对应项置 1 ──
 * PAN_INVERT : 目标偏右时云台却往左转 → 置 1
 * TILT_INVERT: 目标偏下时云台却往上抬 → 置 1 */
#define GIMBAL_PAN_INVERT       0
#define GIMBAL_TILT_INVERT      0

/* ── 上位机遥控模式 ──
 * 1 = 上电即"遥控模式": 暂停本地视觉追踪, 云台只受上位机 /cmd_ptz_angle 控制
 *     (视觉追踪未调通 / 想纯手动玩时用)
 * 0 = 上电即"视觉追踪模式": 由 vision_task 闭环驱动 */
#define GIMBAL_TELEOP_DEFAULT   1

/** @brief 初始化 LEDC 双通道并回中位 */
esp_err_t vision_gimbal_init(void);

/** @brief 用识别到的目标中心坐标驱动云台 (未识别到目标时不要调用) */
void vision_gimbal_track(uint8_t cx, uint8_t cy);

/** @brief 回到中位并清空控制状态 */
void vision_gimbal_center(void);

/** @brief 是否已对准 (连续 GIMBAL_AIM_FRAMES 帧在死区内) */
bool vision_gimbal_is_aimed(void);

/** @brief 取当前脉宽 (us), 用于上报/记录 */
void vision_gimbal_get_us(int *pan_us, int *tilt_us);

/** @brief 取当前角度 (度), 用于上报 */
void vision_gimbal_get_angle(float *pan_deg, float *tilt_deg);

/** @brief 直接指定云台角度 (度) — 供上位机 /cmd_ptz_angle 遥控使用
 *  @note  调用后自动进入遥控模式 (暂停视觉追踪) */
void vision_gimbal_set_angle(float pan_deg, float tilt_deg);

/** @brief 切换遥控模式: on=true 暂停视觉追踪, 只等 /cmd_ptz_angle; on=false 交回视觉追踪 */
void vision_gimbal_set_teleop(bool on);

/** @brief 当前是否处于遥控模式 */
bool vision_gimbal_is_teleop(void);

/** @brief 目标丢失时的处理: 保持不动或回中 (keep=true 保持) */
void vision_gimbal_lost(bool keep);

#ifdef __cplusplus
}
#endif

#endif /* __VISION_GIMBAL_H__ */
