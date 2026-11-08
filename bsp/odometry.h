/**
 * @file    odometry.h
 * @brief   差速轮式里程计 — 基于左右编码器的航位推算
 *
 * 三步拆解术:
 *   1. 直观理解: 里程计像车载导航一样，不看 GPS 不看窗外，
 *      只靠数轮子转了多少圈来推算车子走了多远、拐了多少度。
 *      左轮比右轮转得快 → 车子在右转；两轮一样快 → 直走。
 *   2. 追根溯源: 编码器只告诉 CPU "转了 N 个脉冲"，
 *      没有里程计模块，你只能在串口看一堆数字，
 *      无法把脉冲换算成坐标 (x, y, θ) 发给导航算法。
 *   3. 正规解释: 差速驱动机器人的运动学模型:
 *        v = (vL + vR) / 2              → 线速度
 *        ω = (vR - vL) / wheel_separation → 角速度
 *        dx = v * cos(θ) * dt
 *        dy = v * sin(θ) * dt
 *        dθ = ω * dt
 *
 * 硬件参数 (轮径/轮距/减速比/编码器线数) 全部来自 bsp/car_config.h，
 * 当前值: 轮径 65mm, 轮距 167mm, 11 PPR, 减速比 44.9:1,
 *         1975.6 ticks/轮圈, 0.00010336 m/tick。
 *
 * 使用示例 (高层封装, 推荐):
 * @code
 *   odom_hw_init();          // 初始化编码器 + 里程计状态
 *   while (1) {
 *       odom_poll();         // 读编码器 + 更新位姿
 *       float x, y, theta, vx, vth;
 *       odom_get_state(&x, &y, &theta, &vx, &vth);
 *       vTaskDelay(50);
 *   }
 * @endcode
 */

#ifndef ODOMETRY_H
#define ODOMETRY_H

#include <stdbool.h>
#include <stdint.h>
#include "encoder.h"
#include "car_config.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ================================================================
 * 数据结构
 * ================================================================ */

/** @brief 里程计状态 */
typedef struct {
    float x;                     /**< 全局 X 坐标 (m) */
    float y;                     /**< 全局 Y 坐标 (m) */
    float theta;                 /**< 全局朝向角 (rad), 0=正前方 */
    float vx;                    /**< 当前线速度 (m/s) */
    float vth;                   /**< 当前角速度 (rad/s) */

    /* ── 内部状态, 外部只读不写 ── */
    int32_t last_tick_l;         /**< 左轮上次脉冲 */
    int32_t last_tick_r;         /**< 右轮上次脉冲 */
    int64_t last_time_us;        /**< 上次更新时间戳 */

    /* ── 参数副本 ── */
    float dist_per_tick;         /**< 单 tick 距离 (m) */
    float dist_per_tick_r;   /* 右轮脉冲当量(含修正) */
    float wheel_separation;      /**< 轮距 (m) */
    float ticks_per_wheel_rev;   /**< 每轮圈 tick 数 */
} odom_t;

/* ================================================================
 * API — 高层封装 (通信层使用)
 * ================================================================ */

/**
 * @brief 初始化里程计硬件 (编码器 + 里程计状态)
 *
 * 内部调用 encoder_init + encoder_set_pulses_per_rev +
 * encoder_set_distance_per_tick + odom_init，
 * 引脚与机械参数取自 car_config.h，通信层无需关心。
 *
 * @return true=双路编码器就绪, false=初始化失败
 */
bool odom_hw_init(void);

/**
 * @brief 用最新编码器读数更新里程计 (读取硬件脉冲)
 *
 * 内部读取左右编码器 tick → odom_update()，
 * 通信层只需定时调用此函数即可。
 */
void odom_poll(void);

/**
 * @brief 获取里程计当前位姿和速度
 *
 * @param[out] x      X 坐标 (m)
 * @param[out] y      Y 坐标 (m)
 * @param[out] theta  朝向角 (rad)
 * @param[out] vx     线速度 (m/s)
 * @param[out] vth    角速度 (rad/s)
 */
void odom_get_state(float *x, float *y, float *theta,
                     float *vx, float *vth);

/**
 * @brief 获取编码器原始累积 tick 数 (用于手动验证)
 *
 * @param[out] tick_l  左轮编码器累积脉冲
 * @param[out] tick_r  右轮编码器累积脉冲
 *
 * @note  手动转轮子 1 圈 → tick 增量应 ≈ 1976
 *        公式: ticks_per_rev = PPR × 4 × 减速比 = 11×4×44.9
 */
void odom_get_ticks(int32_t *tick_l, int32_t *tick_r);

/**
 * @brief 获取编码器句柄 (供 PID 控制模块使用)
 *
 * @param[out] enc_l  左编码器句柄
 * @param[out] enc_r  右编码器句柄
 *
 * @note  必须在 odom_hw_init() 成功之后调用
 */
void odom_get_encoder_handles(encoder_handle_t *enc_l,
                               encoder_handle_t *enc_r);

/* ================================================================
 * API — 底层算法 (可脱离硬件独立使用/测试)
 * ================================================================ */

/**
 * @brief 初始化里程计算法状态
 * @param odom                 里程计句柄
 * @param dist_per_tick        单 tick 距离 (m)
 * @param wheel_separation     轮距 (m)
 * @param ticks_per_wheel_rev  每轮圈 tick 数
 */
void odom_init(odom_t *odom, float dist_per_tick,
               float wheel_separation, float ticks_per_wheel_rev);

/**
 * @brief 用最新编码器读数更新里程计位姿
 *
 * 核心算法 (中点法, 比欧拉法精度高一个数量级):
 *   delta_L = (tickL - last_tickL) × dist_per_tick
 *   delta_R = (tickR - last_tickR) × dist_per_tick
 *   delta_dist = (delta_R + delta_L) / 2
 *   delta_theta = (delta_R - delta_L) / wheel_separation
 *   x += delta_dist × cos(theta + delta_theta/2)
 *   y += delta_dist × sin(theta + delta_theta/2)
 *   theta += delta_theta
 *   vx = delta_dist / dt
 *   vth = delta_theta / dt
 *
 * @param odom      里程计句柄
 * @param tick_l    左轮当前累计脉冲
 * @param tick_r    右轮当前累计脉冲
 *
 * @note  首次调用自动记录基线，不产生位移
 * @note  调用频率不宜过高 (<200Hz)，否则 dt 精度下降
 */
void odom_update(odom_t *odom, int32_t tick_l, int32_t tick_r);

/**
 * @brief 重置里程计位姿为原点
 * @param odom  里程计句柄
 */
void odom_reset(odom_t *odom);

/**
 * @brief 获取当前轮子转速 (RPM, 输出轴)
 *
 * 基于最近一次 odom_update() 的速度反推:
 *   rpm = vx / (π × D) × 60
 *
 * @param odom  里程计句柄
 * @return 轮子转速 (RPM)
 */
float odom_get_wheel_rpm(const odom_t *odom);

#ifdef __cplusplus
}
#endif

#endif /* ODOMETRY_H */
