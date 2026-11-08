/**
 * @file    cmd_vel_control.c
 * @brief   /cmd_vel → 差速 PWM 转换实现 — 支持开环 & PID 闭环双模式
 *
 * 模式切换:
 *   - 未调用 cmd_vel_set_encoders(): 开环模式 (线性映射, 兜底用)
 *   - 已调用 cmd_vel_set_encoders(): PID 闭环模式 (编码器反馈调 PWM)
 *
 * PID 闭环数据流 (每 10ms 一拍):
 *   /cmd_vel → 逆运动学(vL,vR) → 限幅/trim → ramp 平滑
 *     → 前馈 FF(开环估算) + PID(编码器反馈修正) → PWM 变化率限制
 *     → motor_set_speed()
 *
 * 安全机制 (两种模式共用):
 *   1. 看门狗: 超时未收到命令 → 目标归零, 纯 P 项主动刹车
 *   2. 死区:   |v| < CMD_VEL_DEAD_ZONE → 目标归零
 *   3. 限幅:   等比例缩放保差速比 + 独立钳位兜底
 *
 * 所有可调参数集中在 bsp/car_config.h, 本文件不含魔法数字。
 */

#include "cmd_vel_control.h"
#include "motor.h"
#include "encoder.h"
#include "pid.h"
#include "debug_log.h"
#include "display_status.h"
#include "esp_timer.h"
#include <stdio.h>
#include <math.h>
#include <stdlib.h>

/* ================================================================
 * 调试打印参数 (文件内私有, 非小车硬件参数)
 * ================================================================ */

/** @brief 收到命令后的密集调试窗口 (μs): 2s 内高频打印 */
#define DEBUG_WINDOW_US         2000000LL
/** @brief 追目标期打印间隔 (μs): 200ms */
#define DEBUG_RUN_INTERVAL_US   200000LL
/** @brief 刹车期打印间隔 (μs): 500ms */
#define DEBUG_BRK_INTERVAL_US   500000LL
/** @brief 空闲期打印间隔 (μs): 3s, 且有有效运动才打印 */
#define DEBUG_IDLE_INTERVAL_US  3000000LL

/* ================================================================
 * 模块级状态
 * ================================================================ */

static motor_handle_t   s_motor_l, s_motor_r;    /* 左右电机句柄 */
static encoder_handle_t s_enc_l, s_enc_r;        /* 左右编码器句柄 (闭环模式) */
static pid_t            s_pid_l, s_pid_r;        /* 左右轮 PID 控制器 */
static float s_target_vl  = 0.0f;                /* 左轮目标线速度 (m/s) */
static float s_target_vr  = 0.0f;                /* 右轮目标线速度 (m/s) */
static float s_target_wz  = 0.0f;                /* 目标角速度 (rad/s), 偏航校正用 */
static float s_yaw_integral = 0.0f;              /* 偏航 PI 积分 (m/s) */
static float s_yaw_correction = 0.0f;            /* 最近一拍的偏航校正值 (调试用) */
static int   s_curr_pwm_l = 0;                   /* 左电机当前 PWM (经变化率限制) */
static int   s_curr_pwm_r = 0;                   /* 右电机当前 PWM (经变化率限制) */
static float   s_last_vx       = 0.0f;           /* 最近一次命令的 vx (OLED 显示用) */
static float   s_last_wz       = 0.0f;           /* 最近一次命令的 wz (OLED 显示用) */
static float   s_disp_target_l = 0.0f;           /* 左轮目标速度 (OLED 显示用) */
static float   s_disp_target_r = 0.0f;           /* 右轮目标速度 (OLED 显示用) */
static float   s_disp_actual_l = 0.0f;           /* 左轮实际速度 (OLED 显示用) */
static float   s_disp_actual_r = 0.0f;           /* 右轮实际速度 (OLED 显示用) */
static int64_t s_last_cmd_us    = 0;             /* 上次收到命令的时间戳 */
static int64_t s_prev_cmd_us   = 0;             /* 上一条命令的绝对时间 (算 gap) */
static int    s_active_wdog_ms = CMD_VEL_WATCHDOG_FIRST_MS;  /* 动态看门狗 */
static int64_t s_debug_until_us = 0;             /* 密集调试截止时间 */
static bool  s_motor_ok   = false;               /* 电机初始化标志 */
static bool  s_encoder_ok = false;               /* 编码器绑定标志 */
static bool  s_pid_mode   = false;               /* PID 闭环模式标志 */

/* ================================================================
 * 初始化
 * ================================================================ */

bool cmd_vel_init(int gpio_l_a, int gpio_l_b,
                   int gpio_r_a, int gpio_r_b)
{
    bool ok = true;

    if (!motor_init(&s_motor_l, gpio_l_a, gpio_l_b, 0)) {
        debug_log_printf("[CMD_VEL] ERROR: 左电机初始化失败\n");
        ok = false;
    }

    if (!motor_init(&s_motor_r, gpio_r_a, gpio_r_b, 1)) {
        debug_log_printf("[CMD_VEL] ERROR: 右电机初始化失败\n");
        ok = false;
    }

    s_target_vl   = 0.0f;
    s_target_vr   = 0.0f;
    s_last_cmd_us = 0;
    s_motor_ok    = ok;
    s_encoder_ok  = false;
    s_pid_mode    = false;

    if (ok) {
        debug_log_printf("[CMD_VEL] 双路电机就绪 (开环模式)  "
                         "max=%.2f m/s  watchdog=自适应(%d/%d ms)\n",
                         (double)CAR_MAX_SPEED_MPS,
                         CMD_VEL_WATCHDOG_FIRST_MS,
                         CMD_VEL_WATCHDOG_STREAM_MS);
    }
    return ok;
}

void cmd_vel_set_encoders(encoder_handle_t enc_l,
                           encoder_handle_t enc_r)
{
    s_enc_l = enc_l;
    s_enc_r = enc_r;
    s_encoder_ok = (enc_l != NULL && enc_r != NULL);

    if (!s_encoder_ok) {
        debug_log_printf("[CMD_VEL] WARN: 编码器句柄无效, 保持开环模式\n");
        return;
    }

    /* 初始化双轮 PID (左右轮参数一致, 不对称由各自积分独立补偿) */
    pid_init(&s_pid_l,
             SPEED_PID_KP, SPEED_PID_KI, SPEED_PID_KD,
             SPEED_PID_INTEGRAL_LIMIT,
             SPEED_PID_OUT_MIN, SPEED_PID_OUT_MAX);

    pid_init(&s_pid_r,
             SPEED_PID_KP, SPEED_PID_KI, SPEED_PID_KD,
             SPEED_PID_INTEGRAL_LIMIT,
             SPEED_PID_OUT_MIN, SPEED_PID_OUT_MAX);

    s_pid_mode = true;

    debug_log_printf("[CMD_VEL] PID 闭环模式已启用  "
                     "Kp=%.0f Ki=%.0f Kd=%.0f  int_limit=%.0f\n",
                     (double)SPEED_PID_KP, (double)SPEED_PID_KI,
                     (double)SPEED_PID_KD, (double)SPEED_PID_INTEGRAL_LIMIT);
}

/* ================================================================
 * 命令接收 — 由 micro-ROS /cmd_vel 回调触发
 * 输入：ROS下发目标线速度linear_x(m/s)、目标角速度angular_z(rad/s)
 * 功能：接收 ROS 的整车速度指令，算出左右轮目标线速度，保存目标值（只写目标，不直接驱动电机）
 * ================================================================ */
void cmd_vel_set_command(float linear_x, float angular_z)
{
    // 如果电机状态异常，直接退出，不执行运动控制
    if (!s_motor_ok) return;

    /* ── 运动学逆解: Twist → 左右轮线速度 ── */
    // WHEEL_SEPARATION_M：两轮轮距，half_sep = 轮距 / 2
    float half_sep = WHEEL_SEPARATION_M * 0.5f;
    // 差速轮公式：左轮速度vl，右轮速度vr
    float vl = linear_x - angular_z * half_sep;
    float vr = linear_x + angular_z * half_sep;

    /* ── 等比例限幅: 保留差速比 (转弯不被削平) ── */
    float vmax = CAR_MAX_SPEED_MPS;  // 小车单轮最大允许速度
    float abs_vl = fabsf(vl), abs_vr = fabsf(vr);
    // 找出左右轮里面速度绝对值更大的那一个
    float max_wheel = abs_vl > abs_vr ? abs_vl : abs_vr;
    if (max_wheel > vmax) {
        // 超过最大速度，整体同比例缩放，保证转弯的曲率不变！
        float scale = vmax / max_wheel;
        vl *= scale;
        vr *= scale;
    }

    /* ── 方向相关 trim: 补偿电机正反向不对称 (当前全 1.0) ── */
    // 电机正向、反向转速特性不一样，乘上修正系数做硬件补偿
    vl *= (vl > 0.0f) ? TRIM_LEFT_FWD : TRIM_LEFT_REV;
    vr *= (vr > 0.0f) ? TRIM_RIGHT_FWD : TRIM_RIGHT_REV;

    /* ── 独立钳位兜底: 单轮溢出不连累另一轮 ── */
    // 二次限制，防止速度超出范围，兜底保护
    if (vl >  vmax) vl =  vmax;
    if (vl < -vmax) vl = -vmax;
    if (vr >  vmax) vr =  vmax;
    if (vr < -vmax) vr = -vmax;

    // 把解算出来的左右轮目标速度保存到静态全局变量，给电机PID循环读取
    s_target_vl   = vl;
    s_target_vr   = vr;
    s_target_wz   = angular_z;                          /* 偏航校正用 */
    s_last_vx     = linear_x;                           /* OLED 显示用 */
    s_last_wz     = angular_z;                          /* OLED 显示用 */
    s_last_cmd_us = esp_timer_get_time();               /* 记录收到指令的时间，看门狗喂狗 */
    s_debug_until_us = s_last_cmd_us + DEBUG_WINDOW_US; /* 开启一段时间调试日志窗口 */

    /* 自适应看门狗: 计算命令间隔, 首帧长/连发短 */
    // 计算两次cmd_vel指令的时间间隔
    int64_t gap_ms = (s_last_cmd_us - s_prev_cmd_us) / 1000;
    s_prev_cmd_us = s_last_cmd_us;
    // 如果很久没来指令，看门狗超时时间变长；连续收到指令，看门狗变快
    s_active_wdog_ms = (gap_ms > CMD_VEL_WATCHDOG_GAP_THRESHOLD || gap_ms == 0)
        ? CMD_VEL_WATCHDOG_FIRST_MS : CMD_VEL_WATCHDOG_STREAM_MS;

    /* 请求 OLED 刷新 (不立即写 I2C, 仅置脏标记) */
    // 标记屏幕需要更新，主循环统一刷新OLED，不在这里阻塞I2C
    display_status_request_update();
}

/* ================================================================
 * 速度控制 tick —— 每 10ms 在主循环调用一次
 *
 * 把上层下发的目标速度，经过平滑、闭环修正、变化率限制后，转成左右轮实际执行的 PWM。
 * 
 * 流程：
 *   ① 看门狗：命令超时 → 停车
 *   ② 目标速度 ramp：同向加速限幅，减速/换向瞬时
 *   ③ 计算 PWM：PID（前馈 + PID）或开环兜底
 *   ④ PWM 变化率限制：慢加速 / 快刹车
 *   ⑤ 输出电机 + 保存显示状态
 *   ⑥ 调试打印（仅 PID 模式）
 * ================================================================ */

void cmd_vel_tick(void)
{
    if (!s_motor_ok) return;

    /* 本拍时间间隔 dt（秒），PID 积分/微分要用 */
    int64_t now_us = esp_timer_get_time();
    static int64_t last_tick_us = 0;
    float dt = (float)(now_us - last_tick_us) * 1.0e-6f;
    if (last_tick_us == 0) dt = 0.01f;  /* 首次调用默认 10ms */
    last_tick_us = now_us;

    /* ── ① 看门狗：超时无命令 → 目标清零、复位 PID 停车 ──
     * 触发后 s_last_cmd_us 置 0，只报警一次，避免刷屏。 */
    if (s_last_cmd_us != 0) {
        int64_t elapsed_ms = (now_us - s_last_cmd_us) / 1000;
        if (elapsed_ms > s_active_wdog_ms) {
            s_target_vl    = 0.0f;
            s_target_vr    = 0.0f;
            s_target_wz    = 0.0f;
            s_yaw_integral = 0.0f;
            s_last_cmd_us  = 0;
            pid_reset(&s_pid_l);
            pid_reset(&s_pid_r);
            debug_log_printf("[CMD_VEL]  看门狗触发 (%.1fs 无命令) → 停车\n",
                             (double)elapsed_ms / 1000.0);
        }
    }

    /* ── ② 目标速度 ramp：让速度逐步逼近目标，避免突变导致打滑 ──
     * 只对“同向且正在加速”限幅，每拍最多变化 accel_step；
     * 减速、过零、换向一律瞬时到位，保证刹车和换向响应。 */
    static float s_ramp_vl = 0.0f, s_ramp_vr = 0.0f;
    float accel_step = CMD_VEL_ACCEL_LIMIT * dt;  /* 本拍允许速度增量 (m/s) */

#define RAMP_STEP(r, tgt) do {                                                \
    bool _same = ((r) >= 0.0f) == ((tgt) >= 0.0f);  /* 是否同方向 */          \
    bool _away = fabsf(tgt) > fabsf(r);             /* 是否在加速 */          \
    if (_same && _away) {                                                     \
        if ((tgt) > (r)) { (r) += accel_step; if ((r) > (tgt)) (r) = (tgt); } \
        else             { (r) -= accel_step; if ((r) < (tgt)) (r) = (tgt); } \
    } else {                                                                  \
        (r) = (tgt);  /* 减速/过零/换向：瞬时 */                              \
    }                                                                         \
} while(0)
    RAMP_STEP(s_ramp_vl, s_target_vl);
    RAMP_STEP(s_ramp_vr, s_target_vr);
#undef RAMP_STEP

    /* 死区过滤：目标过小时电机可能嗡嗡响不转，还白发热，直接当 0。
     * 对平滑后的值过滤，避免平滑过程中反复穿越死区。 */
    float tvl = s_ramp_vl;
    float tvr = s_ramp_vr;
    if (fabsf(tvl) < CMD_VEL_DEAD_ZONE) tvl = 0.0f;
    if (fabsf(tvr) < CMD_VEL_DEAD_ZONE) tvr = 0.0f;

    /* ── ③ 计算 PWM ── */
    float pwm_l, pwm_r;
    float ff_l = 0.0f, ff_r = 0.0f;            /* 前馈分量 (调试用) */
    float pid_out_l = 0.0f, pid_out_r = 0.0f;  /* PID 分量 (调试用) */
    float actual_l = 0.0f, actual_r = 0.0f;    /* 实际速度 (调试用) */

    if (s_pid_mode && s_encoder_ok) {
        /* ═══════════════════════════════════════════
         * PID 闭环：前馈打底 + PID 修边
         *
         * 前馈：低速纯 PID 输出不够，电机可能不转，
         *       先用目标速度估个基础 PWM 打底，再让 PID 修残差。
         * 刹车：目标刚归零那一帧清积分，之后靠 P 项输出反向 PWM
         *       主动刹车；刹车期间积分不累积，防止残留积分把车
         *       往后拽（回弹倒溜）。
         * ═══════════════════════════════════════════ */
        static float s_prev_tvl = 0.0f, s_prev_tvr = 0.0f;
        actual_l = encoder_get_speed(s_enc_l);
        actual_r = encoder_get_speed(s_enc_r);

        /* ── 直行速度对齐 ──
         * 两电机天生有差异，同 PWM 下转速不同，直行会跑偏。
         * 传统“让慢的追快的”遇到弱电机只会白费 PID 出力，
         * 这里改为：直行时以较慢编码器速度为两轮共同目标，
         * 让快的等慢的，保证走直（代价：最高速由弱电机决定）。
         * 转弯不做此对齐，逆运动学目标已正确，加校正反而易振荡。 */
#if CAR_STRAIGHT_SYNC
        {
            if (fabsf(s_target_wz) < 0.005f) {
                /* ═══ 直行：速度同步 ═══ */
                float sync_speed = (fabsf(actual_l) < fabsf(actual_r))
                    ? actual_l : actual_r;  /* 取较慢轮速 */

                if (fabsf(sync_speed) > 0.005f &&
                    (fabsf(tvl) > 0.01f || fabsf(tvr) > 0.01f)) {
                    /* 一阶低通逐步对齐，alpha=0.4 约 25ms 收敛 */
                    float alpha = 0.4f;
                    float blend = tvl * (1.0f - alpha) + sync_speed * alpha;
                    /* 保持方向符号，防止对齐时反向 */
                    if (tvl > 0.0f && blend < 0.0f) blend = 0.0f;
                    if (tvl < 0.0f && blend > 0.0f) blend = 0.0f;
                    tvl = blend;
                    tvr = blend;
                    s_yaw_correction = sync_speed;  /* 调试用：记录对齐速度 */
                } else {
                    s_yaw_correction = 0.0f;
                }
            } else {
                /* ═══ 转弯：不做偏航校正 ═══
                 * P 校正易在转向时振荡（追不上→猛踩→超调→猛刹），
                 * 转弯本就需要差速，逆运动学目标已是对的。 */
                s_yaw_correction = 0.0f;
            }
        }
#else
        /* 关闭直行同步：两轮各自追自己的目标速度。
         * 同步逻辑会把两轮目标都拉到较慢轮的实际值，
         * 两轮有效周长不同时会直接制造固定偏差。 */
        s_yaw_correction = 0.0f;
#endif

        /* 目标刚切到零那一帧复位 PID：清积分 + 启动 P 刹车 */
        if (tvl == 0.0f && s_prev_tvl != 0.0f) pid_reset(&s_pid_l);
        if (tvr == 0.0f && s_prev_tvr != 0.0f) pid_reset(&s_pid_r);
        s_prev_tvl = tvl;
        s_prev_tvr = tvr;

        /* 前馈：目标速度 → 开环预估 PWM（scale 是满 PWM 对应满速的系数） */
        float scale = (float)MOTOR_PWM_MAX / CAR_MAX_SPEED_MPS;
        ff_l = tvl * scale * SPEED_PID_FF_GAIN * SPEED_FF_TRIM_L;
        ff_r = tvr * scale * SPEED_PID_FF_GAIN * SPEED_FF_TRIM_R;

        /* PID：修正“目标 - 实际”的残差 */
        pid_out_l = pid_update(&s_pid_l, tvl, actual_l, dt);
        pid_out_r = pid_update(&s_pid_r, tvr, actual_r, dt);

        pwm_l = ff_l + pid_out_l;
        pwm_r = ff_r + pid_out_r;

        /* ????????????????????????????? PWM?
         * ????????0.05 m/s ????? ~85?????? 300 ???
         * ?????????????????????? */
        if (tvl > 0.0f && pwm_l < (float)CMD_VEL_MIN_PWM) pwm_l = (float)CMD_VEL_MIN_PWM;
        if (tvl < 0.0f && pwm_l > -(float)CMD_VEL_MIN_PWM) pwm_l = -(float)CMD_VEL_MIN_PWM;
        if (tvr > 0.0f && pwm_r < (float)CMD_VEL_MIN_PWM) pwm_r = (float)CMD_VEL_MIN_PWM;
        if (tvr < 0.0f && pwm_r > -(float)CMD_VEL_MIN_PWM) pwm_r = -(float)CMD_VEL_MIN_PWM;

        /* 零速锁定：目标零 + 实际近零连续 3 拍 → 清积分、PWM 归零，
         * 彻底消灭刹车末段因积分残留导致的回弹倒溜。 */
        static int zero_speed_cnt = 0;
        if (tvl == 0.0f && tvr == 0.0f &&
            fabsf(actual_l) < CMD_VEL_DEAD_ZONE &&
            fabsf(actual_r) < CMD_VEL_DEAD_ZONE) {
            zero_speed_cnt++;
            if (zero_speed_cnt >= 3) {
                pid_reset(&s_pid_l);
                pid_reset(&s_pid_r);
                pwm_l = 0.0f;
                pwm_r = 0.0f;
            }
        } else {
            zero_speed_cnt = 0;
        }
    } else {
        /* ═══════════════════════════════════════════
         * 开环兜底：无编码器反馈时，目标速度按
         * “满 PWM 对应满速”线性映射成 PWM，再限幅。
         * 精度差，但保证编码器失效时车还能动。
         * ═══════════════════════════════════════════ */
        float scale = (float)MOTOR_PWM_MAX / CAR_MAX_SPEED_MPS;
        pwm_l = tvl * scale;
        pwm_r = tvr * scale;

        if (pwm_l >  (float)MOTOR_PWM_MAX) pwm_l =  (float)MOTOR_PWM_MAX;
        if (pwm_l < -(float)MOTOR_PWM_MAX) pwm_l = -(float)MOTOR_PWM_MAX;
        if (pwm_r >  (float)MOTOR_PWM_MAX) pwm_r =  (float)MOTOR_PWM_MAX;
        if (pwm_r < -(float)MOTOR_PWM_MAX) pwm_r = -(float)MOTOR_PWM_MAX;
    }

    /* ── ④ PWM 变化率限制（不对称：慢加速 / 快刹车）──
     * 避免从 0 直接跳到满 PWM 造成电流冲击、打滑、齿轮受冲击。
     * |目标| < |当前| 视为刹车/减速，用 DOWN 速率；否则用 UP 速率。 */
    int target_l = (int)pwm_l;
    int target_r = (int)pwm_r;

    int slew_l = (abs(target_l) < abs(s_curr_pwm_l))
        ? PWM_SLEW_RATE_DOWN : PWM_SLEW_RATE_UP;
    int slew_r = (abs(target_r) < abs(s_curr_pwm_r))
        ? PWM_SLEW_RATE_DOWN : PWM_SLEW_RATE_UP;

    if (target_l > s_curr_pwm_l + slew_l) s_curr_pwm_l += slew_l;
    else if (target_l < s_curr_pwm_l - slew_l) s_curr_pwm_l -= slew_l;
    else s_curr_pwm_l = target_l;

    if (target_r > s_curr_pwm_r + slew_r) s_curr_pwm_r += slew_r;
    else if (target_r < s_curr_pwm_r - slew_r) s_curr_pwm_r -= slew_r;
    else s_curr_pwm_r = target_r;

    /* ── ⑤ 输出到电机 ── */
    motor_set_speed(s_motor_l, s_curr_pwm_l);
    motor_set_speed(s_motor_r, s_curr_pwm_r);

    /* 保存状态给 OLED：开环模式无反馈，实际值用目标值凑合显示 */
    s_disp_target_l = tvl;
    s_disp_target_r = tvr;
    if (s_pid_mode && s_encoder_ok) {
        s_disp_actual_l = actual_l;
        s_disp_actual_r = actual_r;
    } else {
        s_disp_actual_l = tvl;
        s_disp_actual_r = tvr;
    }

    /* ── ⑥ 调试打印（仅 PID 模式）──
     * 间隔动态：追目标 200ms / 刹车 500ms / 空闲 3s（有运动才打）。
     * 放在输出之后，打印的 PWM 是本拍真实下发值，方便对账。 */
    if (s_pid_mode && s_encoder_ok) {
        static int64_t last_pid_print_us = 0;
        bool is_active = (now_us < s_debug_until_us);
        int64_t interval_us;
        if (is_active) {
            interval_us = (fabsf(tvl) > 0.001f || fabsf(tvr) > 0.001f)
                ? DEBUG_RUN_INTERVAL_US : DEBUG_BRK_INTERVAL_US;
        } else {
            interval_us = DEBUG_IDLE_INTERVAL_US;
        }

        bool should_print = (now_us - last_pid_print_us > interval_us);
        if (should_print && (is_active ||
            fabsf(tvl) > 0.001f || fabsf(tvr) > 0.001f ||
            fabsf(actual_l) > 0.001f || fabsf(actual_r) > 0.001f)) {
            /* RUN=有目标速度；BRK=刹车中；SYNC=直行；TURN=转弯 */
            const char *tag = (fabsf(tvl) > 0.001f || fabsf(tvr) > 0.001f)
                ? "RUN" : "BRK";
            const char *mode = (fabsf(s_target_wz) < 0.005f) ? "SYNC" : "TURN";
            debug_log_printf("[%s|%s] T(%.2f,%.2f) A(%.2f,%.2f) "
                             "S=%.3f "
                             "FF(%+.0f,%+.0f) PI(%+.0f,%+.0f) "
                             "-> PWM(%+d,%+d)\n",
                             tag, mode, (double)tvl, (double)tvr,
                             (double)actual_l, (double)actual_r,
                             (double)s_yaw_correction,
                             (double)ff_l, (double)ff_r,
                             (double)pid_out_l, (double)pid_out_r,
                             s_curr_pwm_l, s_curr_pwm_r);
            last_pid_print_us = now_us;
        }
    }
}

/* ================================================================
 * 紧急停止
 * ================================================================ */

void cmd_vel_stop(void)
{
    s_target_vl  = 0.0f;
    s_target_vr  = 0.0f;
    pid_reset(&s_pid_l);
    pid_reset(&s_pid_r);

    if (s_motor_ok) {
        s_curr_pwm_l = 0;
        s_curr_pwm_r = 0;
        motor_stop(s_motor_l);
        motor_stop(s_motor_r);
    }

    debug_log_printf("[CMD_VEL] ⏹ 紧急停止\n");
}

/* ================================================================
 * 状态查询 — OLED 显示用
 * ================================================================ */

void cmd_vel_get_last_command(float *vx, float *wz)
{
    if (vx) *vx = s_last_vx;
    if (wz) *wz = s_last_wz;
}

void cmd_vel_get_wheel_state(float *target_l, float *target_r,
                              float *actual_l, float *actual_r)
{
    if (target_l) *target_l = s_disp_target_l;
    if (target_r) *target_r = s_disp_target_r;
    if (actual_l) *actual_l = s_disp_actual_l;
    if (actual_r) *actual_r = s_disp_actual_r;
}
