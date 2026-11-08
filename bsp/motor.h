/**
 * @file    motor.h
 * @brief   直流电机驱动 — 基于 ESP32 MCPWM，支持正反转 + 调速 + 急停
 *
 * 三步拆解术:
 *   1. 直观理解: 把 MCPWM 想象成两个"水龙头"（A 和 B），电机夹在中间：
 *      - 正转：打开 A 水龙头，关死 B → 电流从 A 流向 B
 *      - 反转：关死 A，打开 B 水龙头 → 电流从 B 流向 A
 *      - 停止：两个水龙头都关死 → 没有电流
 *   2. 追根溯源: 没有 PWM 调速，电机只有"全速"和"停止"两种状态——
 *      小车永远在弹射起步和急刹之间切换，控制精度为零。
 *   3. 正规解释: ESP32-S3 内置两路 MCPWM (Motor Control PWM)，
 *      每路可生成互补/独立 PWM 信号，支持硬件死区插入。
 *      本驱动使用独立模式：A 相输出 PWM 占空比控制转速，
 *      B 相拉低作电流回路，实现单极性 H 桥驱动。
 *
 * 硬件要求:
 *   - 电机驱动板（如 TB6612 / L298N / DRV8833）
 *   - 每个电机占用 2 个 GPIO（A/B 两相）
 *   - 左右电机使用不同 MCPWM Group (0/1) 实现完全独立控制
 *
 * 速度说明:
 *   - 范围 -1000 ~ 1000
 *   - 正值 → 正转 (A 输出 PWM, B 拉低)
 *   - 负值 → 反转 (B 输出 PWM, A 拉低)
 *   - 0    → 停止 (AB 全拉低)
 *   - 绝对值越大转速越快, 1000 为满占空比
 *   - PWM 频率 20KHz, 人耳听不到啸叫
 *
 * 使用示例:
 *   @code
 *   #include "motor.h"
 *   #include "car_config.h"
 *
 *   motor_handle_t motor;
 *   motor_init(&motor, PIN_MOTOR_L_A, PIN_MOTOR_L_B, 0);  // 左电机
 *
 *   motor_set_speed(motor, 500);    // 正转 50% 速度
 *   vTaskDelay(pdMS_TO_TICKS(2000));
 *   motor_set_speed(motor, -300);   // 反转 30% 速度
 *   vTaskDelay(pdMS_TO_TICKS(2000));
 *   motor_stop(motor);              // 急停
 *   motor_deinit(motor);            // 释放资源
 *   @endcode
 */

#ifndef MOTOR_H
#define MOTOR_H

#include <stdbool.h>
#include <stdint.h>

#include "car_config.h"     /* 电机引脚 PIN_MOTOR_* 在此统一配置 */

#ifdef __cplusplus
extern "C" {
#endif

/** 电机句柄 — 不透明指针, 外部只持有不访问内部 */
typedef struct motor_t *motor_handle_t;

/**
 * @brief 初始化电机
 * @param out_handle  输出: 电机句柄
 * @param gpio_a      A 相控制引脚
 * @param gpio_b      B 相控制引脚
 * @param group_id    MCPWM 组号 (0 或 1)
 *                    - 左右电机用不同组可实现完全独立控制
 *                    - 同组电机共享定时器, 频率需一致
 * @return true=成功, false=失败
 */
bool motor_init(motor_handle_t *out_handle, int gpio_a, int gpio_b, int group_id);

/**
 * @brief 释放电机硬件资源
 * @param handle  电机句柄
 */
bool motor_deinit(motor_handle_t handle);

/**
 * @brief 设置电机速度与方向
 * @param handle  电机句柄
 * @param speed   速度值: -1000~1000, 正=正转, 负=反转, 0=停止
 *                绝对值越大转速越快, 1000 为满占空比
 */
void motor_set_speed(motor_handle_t handle, int speed);

/**
 * @brief 急停 (等价于 motor_set_speed(handle, 0))
 * @param handle  电机句柄
 */
void motor_stop(motor_handle_t handle);

/**
 * @brief 内置电机测试循环 (阻塞运行)
 *
 * 初始化左右电机 → 正转 2 秒 → 循环，用于快速验证硬件接线。
 */
void motor_test_run(void);

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_H */
