/**
 * @file    pid.c
 * @brief   通用 PID 控制器实现
 *
 * 核心算法:
 *   error = setpoint - measurement
 *   P_out = Kp × error
 *   I_out = Ki × integral          (积分累加 + 抗饱和)
 *   D_out = Kd × (prev_meas - measurement) / dt   (微分先行, 防冲击)
 *   output = P_out + I_out + D_out  (限幅到 [out_min, out_max])
 *
 * 微分先行 (Derivative on Measurement) 的好处:
 *   - 设定值突变时, de/dt 会很大 → 输出剧烈跳变 (derivative kick)
 *   - 改对测量值求导: d_measurement/dt 变化平缓 → 输出平滑
 *   - 等价于对误差求导 (符号相反), 但不响应设定值跳变
 */

#include "pid.h"
#include <math.h>
#include <stdio.h>

void pid_init(pid_t *pid, float kp, float ki, float kd,
              float integral_limit, float out_min, float out_max)
{
    if (!pid) return;

    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->integral = 0.0f;
    pid->prev_measurement = 0.0f;
    pid->integral_limit = integral_limit;
    pid->out_min = out_min;
    pid->out_max = out_max;
    pid->first_run = true;
}

float pid_update(pid_t *pid, float setpoint, float measurement, float dt)
{
    if (!pid || dt <= 0.0f) return 0.0f;

    /* 首次调用: 初始化测量值, 不计算 */
    if (pid->first_run) {
        pid->prev_measurement = measurement;
        pid->first_run = false;
        return 0.0f;
    }

    /* ── 1. 比例项 ── */
    float error = setpoint - measurement;
    float p_out = pid->kp * error;

    /* 2. 积分项 (条件积分: 刹车目标=0时不累, 防回弹) */
    if (fabsf(setpoint) > 0.001f) {
        pid->integral += error * dt;

        /* 抗饱和: 钳位积分值 */
        if (pid->integral >  pid->integral_limit)
            pid->integral =  pid->integral_limit;
        if (pid->integral < -pid->integral_limit)
            pid->integral = -pid->integral_limit;
    }

    float i_out = pid->ki * pid->integral;

    /* ── 3. 微分项 (微分先行: 对测量值求导) ── */
    float d_measurement = pid->prev_measurement - measurement;  /* 注意符号! */
    float d_out = pid->kd * (d_measurement / dt);
    pid->prev_measurement = measurement;

    /* ── 4. 合成 + 限幅 ── */
    float output = p_out + i_out + d_out;
    if (output >  pid->out_max) output =  pid->out_max;
    if (output <  pid->out_min) output =  pid->out_min;

    return output;
}

void pid_reset(pid_t *pid)
{
    if (!pid) return;
    pid->integral = 0.0f;
    pid->prev_measurement = 0.0f;
    pid->first_run = true;
}
