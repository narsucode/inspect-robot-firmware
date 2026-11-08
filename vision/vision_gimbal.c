/**
 * @file    vision_gimbal.c
 * @brief   二自由度云台控制实现
 *          (LEDC 50Hz + 纯 PD + 三重低通滤波 + 死区 + 单步限幅 + 角度限位)
 *
 * 控制链路 (与亚博 MicroROS-V2 color_tracker_ptz 一致):
 *   目标中心 --低通--> 误差(死区) --归一化--> PD(无积分) --单步限幅--> 步长低通
 *        --累加--> 角度限位 --角度低通--> 角度→脉宽 --> LEDC --> 舵机
 *
 * 为什么不用积分项: 积分靠"稳态误差"起作用, 而舵机是"给角度就到角度"的
 * 位置伺服, 目标静止时积分只会在中心附近来回蹭 (亚博就是纯 PD, KI=0)。
 */

#include "vision_gimbal.h"
#include "car_config.h"
#include "driver/ledc.h"
#include "esp_log.h"
#include <math.h>

static const char *TAG = "vision_gimbal";

/* ⚠ LEDC 通道 0 被雷达电机 (ydlidar_x3pro.c) 占用, 云台必须避开! */
#define PAN_CH      LEDC_CHANNEL_2
#define TILT_CH     LEDC_CHANNEL_3
#define GIMBAL_TMR  LEDC_TIMER_1       /* TIMER_0 已被雷达占用 */

/* ── 单轴控制状态 ── */
typedef struct {
    float center_f;       /**< 滤波后的目标坐标 (像素) */
    bool  has_target;     /**< 之前是否已锁定过目标 (决定滤波器是否要初始化) */
    float angle_deg;      /**< 当前角度 (度, 舵机坐标系) */
    float last_step_deg;  /**< 上一次步长 (步长低通用) */
    float last_err_norm;  /**< 上一次归一化误差 (微分用) */
    float min_deg;        /**< 角度下限 */
    float max_deg;        /**< 角度上限 */
} gimbal_axis_t;

static gimbal_axis_t s_pan  = { .min_deg = GIMBAL_PAN_MIN_DEG,  .max_deg = GIMBAL_PAN_MAX_DEG  };
static gimbal_axis_t s_tilt = { .min_deg = GIMBAL_TILT_MIN_DEG, .max_deg = GIMBAL_TILT_MAX_DEG };

static int  s_aim_count = 0;
static bool s_inited = false;
static bool s_teleop = GIMBAL_TELEOP_DEFAULT;   /* 遥控模式: 暂停视觉追踪, 只受 /cmd_ptz_angle 控制 */

/* ── 小工具 ── */
static inline float clampf(float v, float lo, float hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

/* 一阶低通: prev*(1-a) + now*a */
static inline float lpf(float prev, float now, float alpha)
{
    return (1.f - alpha) * prev + alpha * now;
}

/* 脉宽(us) → LEDC 占空比 (周期 20ms) */
static inline uint32_t us_to_duty(int us)
{
    if (us < GIMBAL_US_MIN) us = GIMBAL_US_MIN;
    if (us > GIMBAL_US_MAX) us = GIMBAL_US_MAX;
    return (uint32_t)(((uint64_t)us * ((1u << GIMBAL_LEDC_BITS) - 1)) / 20000);
}

/* 直接写两路 LEDC */
static void gimbal_write(int pan_us, int tilt_us)
{
    ledc_set_duty(LEDC_LOW_SPEED_MODE, PAN_CH,  us_to_duty(pan_us));
    ledc_update_duty(LEDC_LOW_SPEED_MODE, PAN_CH);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, TILT_CH, us_to_duty(tilt_us));
    ledc_update_duty(LEDC_LOW_SPEED_MODE, TILT_CH);
}

/* 把两个轴的当前角度输出到舵机 (角度→脉宽在这里发生) */
static void gimbal_apply_angle(void)
{
    int pan_us  = GIMBAL_PAN_CENTER  + (int)lroundf(s_pan.angle_deg  * GIMBAL_US_PER_DEG);
    int tilt_us = GIMBAL_TILT_CENTER + (int)lroundf(s_tilt.angle_deg * GIMBAL_US_PER_DEG);
    gimbal_write(pan_us, tilt_us);
}

esp_err_t vision_gimbal_init(void)
{
    ledc_timer_config_t tcfg = {
        .speed_mode      = LEDC_LOW_SPEED_MODE,
        .duty_resolution = GIMBAL_LEDC_BITS,
        .timer_num       = GIMBAL_TMR,
        .freq_hz         = GIMBAL_PWM_FREQ_HZ,
        .clk_cfg         = LEDC_AUTO_CLK,
    };
    esp_err_t ret = ledc_timer_config(&tcfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "ledc_timer_config 失败: %s", esp_err_to_name(ret));
        return ret;
    }

    ledc_channel_config_t ch = {
        .gpio_num   = PIN_GIMBAL_PAN,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel    = PAN_CH,
        .timer_sel  = GIMBAL_TMR,
        .duty       = us_to_duty(GIMBAL_PAN_CENTER),
        .hpoint     = 0,
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ch));

    ch.gpio_num = PIN_GIMBAL_TILT;
    ch.channel  = TILT_CH;
    ch.duty     = us_to_duty(GIMBAL_TILT_CENTER);
    ESP_ERROR_CHECK(ledc_channel_config(&ch));

    s_inited = true;
    vision_gimbal_center();
    ESP_LOGI(TAG, "云台就绪: PAN=GPIO%d TILT=GPIO%d (%.1f us/度)",
             PIN_GIMBAL_PAN, PIN_GIMBAL_TILT, (double)GIMBAL_US_PER_DEG);
    return ESP_OK;
}

/**
 * @brief 单轴推进一步
 * @param ax       轴状态
 * @param err_px   像素误差 (已过死区)
 * @param half_px  画面半宽/半高 (归一化用)
 * @param dt       控制周期 (s)
 * @param dir_sign +1/-1: 决定"误差减小的方向"对应舵机角度增大还是减小
 */
static void axis_update(gimbal_axis_t *ax, float err_px, float half_px,
                        float dt, float dir_sign)
{
    /* 1) 归一化误差: 1.0 表示目标贴在画面边缘 (与分辨率无关) */
    float e = err_px / half_px;

    /* 2) 微分 (每秒变化率), 抑制超调 */
    float dt_s = (dt > 1e-3f) ? dt : ((float)GIMBAL_CTRL_PERIOD_MS / 1000.f);
    float de = (e - ax->last_err_norm) / dt_s;
    ax->last_err_norm = e;

    /* 3) 纯 PD, 输出单位是"度"; 亚博把 max_step 同时当增益尺度用 */
    float raw = (GIMBAL_KP * e + GIMBAL_KD * de) * GIMBAL_MAX_STEP_DEG;
    raw = clampf(raw, -GIMBAL_MAX_STEP_DEG, GIMBAL_MAX_STEP_DEG);

    /* 4) 步长低通 (抑制单帧跳变, 防甩头) */
    float step = lpf(ax->last_step_deg, raw, GIMBAL_STEP_SMOOTH_ALPHA);
    ax->last_step_deg = step;

    /* 5) 累加到角度, 并按机械行程限位 (防撞限位) */
    float desired = clampf(ax->angle_deg + dir_sign * step, ax->min_deg, ax->max_deg);

    /* 6) 角度低通 (输出平滑, 不抖) */
    ax->angle_deg = clampf(lpf(ax->angle_deg, desired, GIMBAL_ANGLE_SMOOTH_ALPHA),
                           ax->min_deg, ax->max_deg);
}

void vision_gimbal_track(uint8_t cx, uint8_t cy)
{
    if (!s_inited) {
        return;
    }

    /* 目标中心低通: 首次锁定直接用测量值, 避免从 0 慢慢爬 */
    if (!s_pan.has_target) {
        s_pan.center_f   = (float)cx;
        s_tilt.center_f  = (float)cy;
        s_pan.has_target = s_tilt.has_target = true;
    } else {
        s_pan.center_f  = lpf(s_pan.center_f,  (float)cx, GIMBAL_TARGET_SMOOTH_ALPHA);
        s_tilt.center_f = lpf(s_tilt.center_f, (float)cy, GIMBAL_TARGET_SMOOTH_ALPHA);
    }

    float err_x = s_pan.center_f  - (float)GIMBAL_FRAME_CX;
    float err_y = s_tilt.center_f - (float)GIMBAL_FRAME_CY;

    /* 死区: 误差小于阈值视为 0, 避免在中心附近抖 */
    if (fabsf(err_x) < (float)GIMBAL_DEADZONE_PX) err_x = 0.f;
    if (fabsf(err_y) < (float)GIMBAL_DEADZONE_PX) err_y = 0.f;

    if (err_x == 0.f && err_y == 0.f) {
        if (s_aim_count < GIMBAL_AIM_FRAMES) s_aim_count++;
        return;                                  /* 已居中: 保持当前角度不动 */
    }
    s_aim_count = 0;

    float dt = (float)GIMBAL_CTRL_PERIOD_MS / 1000.f;

    /* 水平: 目标偏右 (err_x>0) → 角度朝 "+" 方向追; 反向则置 GIMBAL_PAN_INVERT */
    float sign_x = GIMBAL_PAN_INVERT ? -1.f : 1.f;
    /* 俯仰: 画面 y 向下增大, 目标偏下 (err_y>0) → 角度朝 "-" 方向追;
     *       实机方向相反则置 GIMBAL_TILT_INVERT */
    float sign_y = GIMBAL_TILT_INVERT ? 1.f : -1.f;

    axis_update(&s_pan,  err_x, (float)GIMBAL_FRAME_CX, dt, sign_x);
    axis_update(&s_tilt, err_y, (float)GIMBAL_FRAME_CY, dt, sign_y);

    gimbal_apply_angle();
}

void vision_gimbal_center(void)
{
    s_pan.has_target = s_tilt.has_target = false;
    s_pan.last_step_deg = s_tilt.last_step_deg = 0.f;
    s_pan.last_err_norm = s_tilt.last_err_norm = 0.f;
    s_pan.angle_deg = s_tilt.angle_deg = 0.f;
    s_aim_count = 0;
    gimbal_apply_angle();
}

bool vision_gimbal_is_aimed(void)
{
    return s_aim_count >= GIMBAL_AIM_FRAMES;
}

void vision_gimbal_get_us(int *pan_us, int *tilt_us)
{
    if (pan_us)  *pan_us  = GIMBAL_PAN_CENTER  + (int)lroundf(s_pan.angle_deg  * GIMBAL_US_PER_DEG);
    if (tilt_us) *tilt_us = GIMBAL_TILT_CENTER + (int)lroundf(s_tilt.angle_deg * GIMBAL_US_PER_DEG);
}

void vision_gimbal_get_angle(float *pan_deg, float *tilt_deg)
{
    if (pan_deg)  *pan_deg  = s_pan.angle_deg;
    if (tilt_deg) *tilt_deg = s_tilt.angle_deg;
}

void vision_gimbal_set_angle(float pan_deg, float tilt_deg)
{
    if (!s_inited) return;              /* 初始化前忽略, 防止写未配置的 LEDC */
    s_teleop = true;                    /* 收到遥控指令 → 进入遥控模式, 视觉追踪让位 */
    s_pan.angle_deg  = clampf(pan_deg,  s_pan.min_deg,  s_pan.max_deg);
    s_tilt.angle_deg = clampf(tilt_deg, s_tilt.min_deg, s_tilt.max_deg);
    /* 遥控模式下清掉闭环残留状态, 避免切回追踪时突然一抖 */
    s_pan.last_step_deg = s_tilt.last_step_deg = 0.f;
    s_pan.last_err_norm = s_tilt.last_err_norm = 0.f;
    gimbal_apply_angle();
}

void vision_gimbal_set_teleop(bool on)
{
    s_teleop = on;
}

bool vision_gimbal_is_teleop(void)
{
    return s_teleop;
}

void vision_gimbal_lost(bool keep)
{
    s_pan.has_target = s_tilt.has_target = false;
    s_pan.last_step_deg = s_tilt.last_step_deg = 0.f;
    s_pan.last_err_norm = s_tilt.last_err_norm = 0.f;
    s_aim_count = 0;
    if (!keep) {
        vision_gimbal_center();                  /* 回中位 */
    }
}
