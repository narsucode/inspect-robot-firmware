/**
 * @file    odometry.c
 * @brief   差速轮式里程计实现 — 基于左右编码器脉冲的航位推算
 *
 * 数学模型 (中点法 / RK2):
 *   delta_L = (tickL_cur - tickL_prev) × dist_per_tick
 *   delta_R = (tickR_cur - tickR_prev) × dist_per_tick
 *   delta_S = (delta_R + delta_L) / 2           ← 弧长
 *   delta_θ = (delta_R - delta_L) / L           ← L = 轮距
 *
 *   位姿更新 (中点积分, 比欧拉法准):
 *     θ += delta_θ
 *     x += delta_S × cos(θ - delta_θ/2)
 *     y += delta_S × sin(θ - delta_θ/2)
 *
 *   速度:
 *     vx = delta_S / dt
 *     vth = delta_θ / dt
 *
 * 为什么用中点法而不是欧拉:
 *   - 欧拉法用起点朝向算位移 → 转弯时路径偏短
 *   - 中点法用"转弯中途"的朝向 → 精度高一个数量级
 */

#include "odometry.h"
#include "encoder.h"
#include "esp_timer.h"
#include <math.h>

/* ================================================================
 * 模块内里程计状态 — 通信层无需直接访问
 * static 私有变量，仅本.c文件可见
 * ================================================================ */
static encoder_handle_t s_enc_l;   // 左编码器句柄
static encoder_handle_t s_enc_r;   // 右编码器句柄
static odom_t           s_odom;    // 全局里程计状态结构体

/* ================================================================
 * 初始化 & 重置
 * ================================================================ */

/**
 * @brief 初始化里程计结构体参数
 * @param odom 里程结构体指针
 * @param dist_per_tick 单个编码器脉冲对应的轮子行走距离(m)
 * @param wheel_separation 左右两轮轮距(m)
 * @param ticks_per_wheel_rev 轮子转一圈总脉冲数
 */
void odom_init(odom_t *odom, float dist_per_tick,
               float wheel_separation, float ticks_per_wheel_rev)
{
    if (!odom) return;  // 空指针保护

    odom->x = 0.0f;                     // 全局坐标x
    odom->y = 0.0f;                     // 全局坐标y
    odom->theta = 0.0f;                 // 小车航向角(弧度)
    odom->vx = 0.0f;                    // 小车中心线速度 m/s
    odom->vth = 0.0f;                   // 小车角速度 rad/s
    odom->last_tick_l = 0;              // 上一次采样左编码器脉冲
    odom->last_tick_r = 0;              // 上一次采样右编码器脉冲
    odom->last_time_us = 0;             // 上一次采样时间(微秒)
    odom->dist_per_tick = dist_per_tick;// 1脉冲走多少米(左轮基准)
    odom->dist_per_tick_r = dist_per_tick * DIST_PER_TICK_R_TRIM;
    odom->wheel_separation = wheel_separation; // 轮距
    odom->ticks_per_wheel_rev = ticks_per_wheel_rev; // 每圈总脉冲
}

/**
 * @brief 重置里程计，坐标、角度、速度全部清零
 * @param odom 里程结构体指针
 */
void odom_reset(odom_t *odom)
{
    if (!odom) return; // 空指针保护

    odom->x = 0.0f;
    odom->y = 0.0f;
    odom->theta = 0.0f;
    odom->vx = 0.0f;
    odom->vth = 0.0f;
    odom->last_tick_l = 0;
    odom->last_tick_r = 0;
    odom->last_time_us = 0;
}

/* ================================================================
 * 更新 — 中点法航位推算【核心函数】
 * ================================================================ */

/**
 * @brief 根据最新编码器脉冲，更新小车位姿与速度
 * @param odom 里程计结构体指针
 * @param tick_l 当前左编码器总脉冲计数
 * @param tick_r 当前右编码器总脉冲计数
 */
void odom_update(odom_t *odom, int32_t tick_l, int32_t tick_r)
{
    if (!odom) return; // 空指针保护

    int64_t now_us = esp_timer_get_time(); // 获取当前系统微秒时间戳

    /* 首次调用: 记录基线, 无位移计算，只保存初始状态 */
    if (odom->last_time_us == 0) {
        odom->last_tick_l = tick_l;
        odom->last_tick_r = tick_r;
        odom->last_time_us = now_us;
        return;
    }

    /* ── 计算本次采样间隔内，左右轮新增脉冲 ── */
    int32_t d_tick_l = tick_l - odom->last_tick_l;
    int32_t d_tick_r = tick_r - odom->last_tick_r;

    /* 更新保存本次脉冲，作为下一轮的"上一次脉冲" */
    odom->last_tick_l = tick_l;
    odom->last_tick_r = tick_r;

    /* ── 脉冲计数 → 轮子实际行走距离 (单位m) ── */
    float d_left  = (float)d_tick_l * odom->dist_per_tick;
    float d_right = (float)d_tick_r * odom->dist_per_tick_r;

    /* ── 计算两次采样的时间间隔dt，转成秒 ── */
    float dt = (float)(now_us - odom->last_time_us) * 1.0e-6f;
    odom->last_time_us = now_us; // 更新上一次时间戳

    if (dt <= 0.0f) return; // 时间异常，直接退出，不计算

    /* ── 差速小车运动模型 ── */
    float d_dist  = (d_right + d_left) * 0.5f;          // 小车中心前进弧长
    float d_theta = (d_right - d_left) / odom->wheel_separation; // 角度变化量(rad)

    /* 中点法：取运动过程中间时刻的航向角做坐标积分，提升转弯精度 */
    float half_dtheta = d_theta * 0.5f;
    float mid_theta   = odom->theta + half_dtheta;

    /* 更新全局x,y坐标 */
    odom->x += d_dist * cosf(mid_theta);
    odom->y += d_dist * sinf(mid_theta);
    /* 更新航向角 */
    odom->theta += d_theta;

    /* 航向角归一化：限制角度范围 [-π , π] (-180° ~ +180°)，防止三角函数溢出 */
    while (odom->theta > M_PI)  odom->theta -= 2.0f * (float)M_PI;
    while (odom->theta < -M_PI) odom->theta += 2.0f * (float)M_PI;

    /* ── 计算小车中心速度 ── */
    odom->vx  = d_dist  / dt;    // 线速度 m/s
    odom->vth = d_theta / dt;    // 角速度 rad/s
}

/* ================================================================
 * 轮子转速 RPM (输出轴)
 * ================================================================ */

/**
 * @brief 计算轮子转速 RPM 转/分钟
 * @param odom 里程结构体指针
 * @return 轮子转速 rpm
 */
float odom_get_wheel_rpm(const odom_t *odom)
{
    if (!odom || odom->dist_per_tick <= 0.0f) return 0.0f;

    /* vx(m/s) → rps(圈/秒) → RPM(圈/分钟) */
    float m_per_rev = WHEEL_CIRCUMFERENCE_M; // 轮子一圈周长(m)
    if (m_per_rev <= 0.0f) return 0.0f;

    float rps = odom->vx / m_per_rev; // 每秒多少圈
    return rps * 60.0f;               // 乘以60得到每分钟圈数RPM
}

/* ================================================================
 * 高层封装 — 通信层用 (内部管理编码器 + 里程计状态)
 * ================================================================ */

/**
 * @brief 里程计硬件初始化：初始化左右编码器 + 初始化里程参数
 * @return true:硬件初始化成功 false:失败
 */
bool odom_hw_init(void)
{
    bool ok = true;

    /* 初始化左编码器 */
    if (!encoder_init(&s_enc_l, PIN_ENCODER_L_A, PIN_ENCODER_L_B)) {
        printf("[ODOM] ERROR: 左编码器初始化失败\n");
        ok = false;
    } else {
        encoder_set_pulses_per_rev(s_enc_l, ENCODER_PPR);
        encoder_set_distance_per_tick(s_enc_l, DIST_PER_TICK_M);
    }

    /* 初始化右编码器 */
    if (!encoder_init(&s_enc_r, PIN_ENCODER_R_A, PIN_ENCODER_R_B)) {
        printf("[ODOM] ERROR: 右编码器初始化失败\n");
        ok = false;
    } else {
        encoder_set_pulses_per_rev(s_enc_r, ENCODER_PPR);
        encoder_set_distance_per_tick(s_enc_r, DIST_PER_TICK_M * DIST_PER_TICK_R_TRIM);
    }

    /* 编码器全部初始化成功，再初始化里程计参数 */
    if (ok) {
        odom_init(&s_odom, DIST_PER_TICK_M, WHEEL_SEPARATION_M,
                  (float)TICKS_PER_WHEEL_REV);
        printf("[ODOM] 硬件就绪  wheel=%.0fmm  sep=%.0fmm  "
               "dist/tick=%.6fm  ticks/rev=%.1f\n",
               WHEEL_DIAMETER_M * 1000.0f, WHEEL_SEPARATION_M * 1000.0f,
               (double)DIST_PER_TICK_M, (double)TICKS_PER_WHEEL_REV);
    }
    return ok;
}

/**
 * @brief 里程计轮询更新函数，定时调用(10ms左右)
 *        读取编码器脉冲，调用odom_update计算位姿
 */
void odom_poll(void)
{
    int32_t tick_l = encoder_get_ticks(s_enc_l); // 获取左轮编码器总脉冲
    int32_t tick_r = encoder_get_ticks(s_enc_r); // 获取右轮编码器总脉冲
    odom_update(&s_odom, tick_l, tick_r);        // 更新里程计位姿
}

/**
 * @brief 获取小车当前里程状态 x,y,theta,vx,vth
 * @param x 输出x坐标指针，可以传NULL不需要就不读取
 * @param y 输出y坐标指针，可以传NULL
 * @param theta 输出航向角指针，可以传NULL
 * @param vx 输出线速度指针，可以传NULL
 * @param vth 输出角速度指针，可以传NULL
 */
void odom_get_state(float *x, float *y, float *theta,
                     float *vx, float *vth)
{
    if (x)     *x     = s_odom.x;
    if (y)     *y     = s_odom.y;
    if (theta) *theta = s_odom.theta;
    if (vx)    *vx    = s_odom.vx;
    if (vth)   *vth   = s_odom.vth;
}

/**
 * @brief 获取最近一次采样的编码器脉冲
 * @param tick_l 左轮脉冲输出指针
 * @param tick_r 右轮脉冲输出指针
 */
void odom_get_ticks(int32_t *tick_l, int32_t *tick_r)
{
    if (tick_l) *tick_l = s_odom.last_tick_l;
    if (tick_r) *tick_r = s_odom.last_tick_r;
}

/**
 * @brief 获取左右编码器句柄，外部模块可操作编码器
 * @param enc_l 左编码器句柄输出指针
 * @param enc_r 右编码器句柄输出指针
 */
void odom_get_encoder_handles(encoder_handle_t *enc_l,
                               encoder_handle_t *enc_r)
{
    if (enc_l) *enc_l = s_enc_l;
    if (enc_r) *enc_r = s_enc_r;
}