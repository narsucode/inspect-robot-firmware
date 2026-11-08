/**
 * @file    encoder.h
 * @brief   电机编码器驱动 — 基于 ESP32 PCNT 硬件正交解码
 *
 * 三步拆解术:
 *   1. 直观理解: 编码器就像电机的"里程表+速度表"——
 *      A/B 两相输出方波，PCNT 硬件自动识别正反转并计数，
 *      不需要 CPU 一个脉冲一个脉冲地去数。
 *   2. 追根溯源: 没有编码器，你只能"盲开"电机——
 *      不知道实际转了多少圈、转多快、有没有堵转，
 *      PID 闭环控制也无从谈起。
 *   3. 正规解释: ESP32-S3 内置 PCNT(脉冲计数器)，
 *      支持正交解码(4倍频)，硬件自动处理方向识别和计数，
 *      配合 80μs 硬件毛刺滤波 + 软件突变滤波双重抗干扰
 *      (滤波阈值在 car_config.h 中配置)。
 *
 * 硬件要求:
 *   - 编码器输出 A/B 两相方波（常见于带霍尔传感器的减速电机）
 *   - 每路占用 1 个 GPIO
 *   - 需启用内部上拉（代码自动配置）
 *
 * 使用示例:
 *   @code
 *   encoder_handle_t enc;
 *   encoder_init(&enc, GPIO_A, GPIO_B);           // 初始化
 *   encoder_set_pulses_per_rev(enc, 11);          // 设置编码器线数
 *   encoder_set_distance_per_tick(enc, 0.0001f);  // 设置每脉冲距离
 *
 *   int32_t ticks = encoder_get_ticks(enc);       // 读取累计脉冲
 *   float speed    = encoder_get_speed(enc);      // 读取速度 m/s
 *   float rpm      = encoder_get_rpm(enc);        // 读取转速 RPM
 *   int   dir      = encoder_get_direction(enc);  // 读取方向: 1正/-1反/0停
 *   @endcode
 */

#ifndef ENCODER_H
#define ENCODER_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** 编码器句柄 — 不透明指针，外部只持有不访问内部 */
typedef struct encoder_t *encoder_handle_t;

/**
 * @brief 初始化编码器
 *
 * 配置 PCNT 正交解码 + 硬件毛刺滤波 + 软件抗干扰。
 * A/B 相自动配置内部上拉。
 *
 * @param out_handle  输出：编码器句柄
 * @param gpio_a      A 相信号引脚
 * @param gpio_b      B 相信号引脚
 * @return true=成功，false=失败（引脚冲突/内存不足）
 */
bool encoder_init(encoder_handle_t *out_handle, int gpio_a, int gpio_b);

/**
 * @brief 释放编码器硬件资源
 * @param handle  编码器句柄
 * @return true=成功
 */
bool encoder_deinit(encoder_handle_t handle);

/**
 * @brief 清零累计脉冲计数（硬件 + 软件缓存全清）
 *
 * 通常在以下场景调用：
 *   - 里程计归零
 *   - 运动开始前校准
 *   - PID 控制复位
 *
 * @param handle  编码器句柄
 * @return true=成功
 */
bool encoder_reset(encoder_handle_t handle);

/**
 * @brief 读取累计脉冲数（带软件抗干扰过滤）
 *
 * 软件突变滤波: 单次读值突变超过 ENCODER_MAX_DELTA_PER_READ (500)
 * 判定为电机 PWM 引起的 EMI 假脉冲，返回上次有效值，
 * 同时用硬件真实值更新基准，防止误差累积。
 *
 * @param handle  编码器句柄
 * @return 有效 32 位脉冲计数（正=正转累积，负=反转累积）
 */
int32_t encoder_get_ticks(encoder_handle_t handle);

/**
 * @brief 读取实时线速度
 *
 * 基于两次调用间的时间差和脉冲差计算。
 * 首次调用或调用间隔过短（<1μs）返回 0。
 *
 * @param handle  编码器句柄
 * @return 当前速度（m/s），正=正转方向，负=反转方向
 */
float encoder_get_speed(encoder_handle_t handle);

/**
 * @brief 读取实时转速 (电机轴 RPM)
 *
 * 使用 get_speed 缓存的最新速度值换算，不重新读取硬件。
 * 调用顺序：先 encoder_get_speed()，再 encoder_get_rpm()。
 *
 * 注意: 本函数返回的是**电机轴转速**，不是轮子输出轴转速。
 * 要获取轮子转速，请使用 odom_get_wheel_rpm()。
 *
 * 电机 RPM = speed / dist_per_tick / ticks_per_rev × 60
 *
 * @param handle  编码器句柄
 * @return 当前电机转速（RPM），正=正转，负=反转
 */
float encoder_get_rpm(encoder_handle_t handle);

/**
 * @brief 获取转动方向
 *
 * 基于最近一次速度读数判断，带微小死区（±0.001 m/s 视为停止）。
 *
 * @param handle  编码器句柄
 * @return  1 = 正转（前进），-1 = 反转（后退），0 = 停止
 */
int encoder_get_direction(encoder_handle_t handle);

/**
 * @brief 设置单个脉冲对应的行驶距离
 *
 * 计算公式：
 *   distance = 轮子周长 / (编码器线数 × 4 × 减速比)
 *   - 轮子周长 = π × 直径（单位：米）
 *   - 编码器线数 = 每电机轴圈输出的脉冲数（如 11 PPR）
 *   - ×4 是因为 PCNT 正交解码做了 4 倍频
 *   - 减速比 = 电机轴转速 / 输出轴转速
 *
 * 示例：轮径 65mm，编码器 11 PPR，减速比 44.9:1
 *   distance = (π × 0.065) / (11 × 4 × 44.9) ≈ 0.00010336 m
 *
 * @param handle      编码器句柄
 * @param distance_m  单脉冲距离（米）
 * @return true=成功
 */
bool encoder_set_distance_per_tick(encoder_handle_t handle, float distance_m);

/**
 * @brief 设置编码器每圈脉冲数（用于 RPM 计算）
 *
 * 注意：这里填的是编码器物理线数，不是 4 倍频后的值。
 * 常见值：11 PPR（市面常见减速电机霍尔编码器）
 *
 * @param handle         编码器句柄
 * @param pulses_per_rev 每圈脉冲数
 * @return true=成功
 */
bool encoder_set_pulses_per_rev(encoder_handle_t handle, int pulses_per_rev);

#ifdef __cplusplus
}
#endif

#endif /* ENCODER_H */
