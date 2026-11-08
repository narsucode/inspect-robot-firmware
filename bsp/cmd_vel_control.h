/**
 * @file    cmd_vel_control.h
 * @brief   /cmd_vel 速度命令 → 差速驱动 PWM 转换器
 *
 * 三步拆解术:
 *   1. 直观理解: 你发一条 "前进 0.5m/s, 左转 0.2rad/s" 的命令，
 *      这个模块把它翻译成 "左轮 PWM=600, 右轮 PWM=400" 发给电机。
 *   2. 追根溯源: 没有这个模块，每个控制程序都要自己算差速运动学，
 *      容易把左右轮方向搞反、忘记单位换算，小车原地转圈。
 *   3. 正规解释: 差速驱动逆运动学:
 *        v_left  = v_x - ω_z × (L / 2)    [m/s]
 *        v_right = v_x + ω_z × (L / 2)    [m/s]
 *      再经前馈 + PID 闭环换算为 PWM (-1000~1000)。
 *
 * 安全特性:
 *   - 看门狗: 超时未收到新命令 → 自动停车 (超时值见 car_config.h)
 *   - 死区:   |v| 小于阈值 → PWM=0，防止微动爬行
 *   - 双 ramp: 目标速度限加速度 (m/s²)，输出 PWM 限变化率 (每tick)，
 *     加速柔和防打滑，减速/换向瞬时生效保安全
 *
 * 所有可调参数 (轮距/PID/看门狗/限幅等) 集中在 bsp/car_config.h。
 *
 * 使用示例:
 * @code
 *   cmd_vel_init(PIN_MOTOR_L_A, PIN_MOTOR_L_B, PIN_MOTOR_R_A, PIN_MOTOR_R_B);
 *   while (1) {
 *       cmd_vel_tick();  // 每 10ms 调用，看门狗 + ramp + PID
 *       vTaskDelay(10);
 *   }
 * @endcode
 */

#ifndef CMD_VEL_CONTROL_H
#define CMD_VEL_CONTROL_H

#include <stdbool.h>
#include <stdint.h>

#include "car_config.h"     /* 全车参数: 轮距/PID/限幅/引脚 */
#include "encoder.h"        /* encoder_handle_t */

#ifdef __cplusplus
extern "C" {
#endif

/* ================================================================
 * 公开 API
 * ================================================================ */

/**
 * @brief 初始化双路电机驱动
 *
 * 内部调用 motor_init() 创建左右电机句柄，
 * 左电机用 MCPWM Group 0, 右电机用 Group 1，完全独立控制。
 *
 * @param gpio_l_a  左电机 A 相
 * @param gpio_l_b  左电机 B 相
 * @param gpio_r_a  右电机 A 相
 * @param gpio_r_b  右电机 B 相
 * @return true=双路电机就绪, false=初始化失败
 */
bool cmd_vel_init(int gpio_l_a, int gpio_l_b,
                   int gpio_r_a, int gpio_r_b);

/**
 * @brief 绑定编码器句柄 (启用速度闭环 PID)
 *
 * 必须在 odom_hw_init() 之后调用。
 * 传入左右编码器句柄后，cmd_vel_tick() 将使用 PID 闭环控制，
 * 而非简单的开环 PWM 映射。
 *
 * @param enc_l  左编码器句柄 (来自 encoder_init)
 * @param enc_r  右编码器句柄
 *
 * @note  若不调用此函数，cmd_vel_tick() 退化为开环线性映射
 */
void cmd_vel_set_encoders(encoder_handle_t enc_l,
                           encoder_handle_t enc_r);

/**
 * @brief 接收 /cmd_vel 命令 (由 micro-ROS 回调触发)
 *
 * 将 ROS2 Twist 消息转为左右轮目标速度 (m/s)，
 * 软加速由 cmd_vel_tick() 每 10ms 逐步执行。
 *
 * @param linear_x   线速度 (m/s), 正=前进
 * @param angular_z  角速度 (rad/s), 正=逆时针 (左转)
 *
 * @note  每次收到命令自动喂狗 (重置看门狗计时)
 */
void cmd_vel_set_command(float linear_x, float angular_z);

/**
 * @brief 速度控制 tick — 每 10ms 在主循环中调用
 *
 * 职责:
 *   1. 看门狗: 超时 → 目标速度归零
 *   2. 目标速度 ramp: 限加速度，防弹射起步
 *   3. PID 闭环: 前馈 + 编码器反馈修正 (若已绑定编码器)
 *   4. PWM 变化率限制: 慢加速 / 快刹车
 *   5. 输出到电机
 *
 * @note  必须在主循环固定频率调用 (10ms)
 */
void cmd_vel_tick(void);

/**
 * @brief 紧急停止 — 立即将目标速度和当前 PWM 都归零
 */
void cmd_vel_stop(void);

/* ================================================================
 * 状态查询 — OLED 显示用
 * ================================================================ */

/**
 * @brief 获取最近一次收到的 /cmd_vel 命令
 * @param[out] vx  线速度 (m/s)
 * @param[out] wz  角速度 (rad/s)
 */
void cmd_vel_get_last_command(float *vx, float *wz);

/**
 * @brief 获取四轮状态: 目标速度 + 实际速度
 * @param[out] target_l  左轮目标 (m/s), ramp 平滑后的值
 * @param[out] target_r  右轮目标 (m/s)
 * @param[out] actual_l  左轮实际 (m/s), 编码器实测
 * @param[out] actual_r  右轮实际 (m/s)
 */
void cmd_vel_get_wheel_state(float *target_l, float *target_r,
                              float *actual_l, float *actual_r);

#ifdef __cplusplus
}
#endif

#endif /* CMD_VEL_CONTROL_H */
