/**
 * @file    pid.h
 * @brief   通用 PID 控制器 — 位置式 + 积分抗饱和 + 微分先行
 *
 * 三步拆解术:
 *   1. 直观理解: PID 就像一个"有记忆的老司机"——
 *      P (比例): 现在偏了多少，立刻纠正（现在）
 *      I (积分): 偏了很久的总账，慢慢补回来（过去）
 *      D (微分): 纠正太快了，踩一脚刹车（未来趋势）
 *   2. 追根溯源: 没有 PID，你只能"开环"控制——
 *      发 500 PWM 出去，转多快全看电池电压和路面摩擦力，
 *      上坡变慢、下坡变快、电池没电了就趴窝。
 *   3. 正规解释: u(t) = Kp×e(t) + Ki×∫e(t)dt + Kd×de(t)/dt
 *      三项各司其职: P 快速响应, I 消除稳态误差, D 抑制超调。
 *
 * 本实现的特点:
 *   - 微分先行 (Derivative on Measurement): 对测量值求导而非对误差求导，
 *     避免设定值突变时的"微分冲击"(derivative kick)
 *   - 积分抗饱和: |积分| ≤ integral_limit, 防止长时间卡死导致积分爆炸
 *   - 输出限幅: output ∈ [out_min, out_max]
 *
 * 使用示例:
 * @code
 *   pid_t speed_pid;
 *   pid_init(&speed_pid, 200.0f, 10.0f, 0.0f, 500.0f, -1000, 1000);
 *
 *   while (1) {
 *       float actual = encoder_get_speed(enc);
 *       float pwm = pid_update(&speed_pid, 0.3f, actual, 0.01f);
 *       motor_set_speed(motor, (int)pwm);
 *       vTaskDelay(10);
 *   }
 * @endcode
 */

#ifndef PID_H
#define PID_H

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/** @brief PID 控制器状态 */
typedef struct {
    float kp;              /**< 比例增益 */
    float ki;              /**< 积分增益 */
    float kd;              /**< 微分增益 */

    float integral;        /**< 积分累加值 */
    float prev_measurement; /**< 上次测量值 (微分先行) */
    float integral_limit;  /**< 积分饱和上限 (绝对值) */
    float out_min;         /**< 输出下限 */
    float out_max;         /**< 输出上限 */

    bool  first_run;       /**< 首次运行标志 */
} pid_t;

/**
 * @brief 初始化 PID 控制器
 *
 * @param pid              PID 句柄
 * @param kp               比例增益
 * @param ki               积分增益
 * @param kd               微分增益 (0 = 关闭微分)
 * @param integral_limit   积分限幅 (绝对值), 如 500
 * @param out_min          输出下限
 * @param out_max          输出上限
 */
void pid_init(pid_t *pid, float kp, float ki, float kd,
              float integral_limit, float out_min, float out_max);

/**
 * @brief 执行一次 PID 计算
 *
 * 算法: u = Kp×e + Ki×∫e + Kd×(-d_measurement)
 *       微分先行: 对测量值求导而非对误差求导
 *
 * @param pid          PID 句柄
 * @param setpoint     目标值 (设定点)
 * @param measurement  当前实测值
 * @param dt           距上次调用时间间隔 (秒)
 * @return             控制器输出 (已限幅)
 */
float pid_update(pid_t *pid, float setpoint, float measurement, float dt);

/**
 * @brief 重置 PID — 清空积分 + 上次测量值
 * @param pid  PID 句柄
 */
void pid_reset(pid_t *pid);

#ifdef __cplusplus
}
#endif

#endif /* PID_H */
