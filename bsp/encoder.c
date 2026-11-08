/**
 * @file    encoder.c
 * @brief   电机编码器驱动实现 — PCNT 硬件正交解码 + 双层软件滤波
 *
 * 工作原理（比喻版）:
 *   把 PCNT 想象成一个"带方向感的自动计数器"：
 *   - A 相先来脉冲 → 计数器 +1（正转）
 *   - B 相先来脉冲 → 计数器 -1（反转）
 *   - 两相配合可以实现 4 倍频：每个物理脉冲周期计数 4 次，精度翻 4 倍
 *   硬件自动完成这一切，CPU 只管"看一眼计数器读数"就行。
 *
 * 滤波策略 (阈值在 car_config.h 中配置):
 *   - 硬件层: PCNT 毛刺滤波器 (80μs)，滤掉电机换向火花
 *   - 软件层: 单次读值突变 >500 → 判定 EMI 干扰，返回旧值
 *
 * 技术细节:
 *   - 计数范围: ±32767，硬件自动 32 位累加（overflow 由 accum_count 处理）
 *   - 内部上拉: 编码器通常为开漏输出，需上拉才能读到信号
 *   - 测速精度: 取决于调用频率，建议 10~50ms 周期
 */

#include "encoder.h"
#include "car_config.h"
#include "driver/pulse_cnt.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "esp_log.h"
#include <stdlib.h>
#include <inttypes.h>

#define TAG "encoder"

/* ===== 硬件参数 ===== */
#define PCNT_HIGH_LIMIT      32767     // PCNT 16位计数上限
#define PCNT_LOW_LIMIT      -32768     // PCNT 16位计数下限
// 毛刺滤波与突变阈值统一由 car_config.h 提供:
//   ENCODER_GLITCH_FILTER_NS / ENCODER_MAX_DELTA_PER_READ

/* ===== 编码器内部数据结构 ===== */
struct encoder_t {
    /* --- PCNT 硬件句柄 --- */
    pcnt_unit_handle_t    unit;        // PCNT 单元
    pcnt_channel_handle_t chan_a;      // A 相通道
    pcnt_channel_handle_t chan_b;      // B 相通道

    /* --- 引脚信息 --- */
    int gpio_a;
    int gpio_b;

    /* --- 测速缓存（用于 encoder_get_speed） --- */
    int32_t  last_tick;                // 上次读数
    int64_t  last_time_us;             // 上次时间戳
    float    last_speed;               // 最近一次速度值（m/s）

    /* --- 软件突变滤波状态 --- */
    int32_t  prev_tick;                // 上一次有效脉冲值
    bool     first_read;               // 是否首次读取

    /* --- 速度换算参数 --- */
    float    dist_per_tick;            // 单脉冲距离（m）
    int      pulses_per_rev;           // 每圈物理脉冲数
    int      ticks_per_rev;            // 每圈计数脉冲数（= pulses_per_rev × 4）

    /* --- 方向判断缓存 --- */
    int      last_direction;           // 上次方向 (1正转/-1反转/0停止)
};

/* ================================================================
 * 初始化
 * ================================================================ */

bool encoder_init(encoder_handle_t *out_handle, int gpio_a, int gpio_b)
{
    if (!out_handle) return false;

    struct encoder_t *enc = calloc(1, sizeof(struct encoder_t));
    if (!enc) return false;

    enc->gpio_a = gpio_a;
    enc->gpio_b = gpio_b;
    enc->dist_per_tick = 0.0001f;       // 默认 0.1mm/脉冲
    enc->pulses_per_rev = 11;           // 默认 11 PPR（常见减速电机）
    enc->ticks_per_rev = 11 * 4;        // 4 倍频 = 44
    enc->first_read = true;
    enc->last_direction = 0;

    /* ---- 第1步: 配置 GPIO 为输入 + 内部上拉 ---- */
    gpio_set_direction(gpio_a, GPIO_MODE_INPUT);
    gpio_pullup_en(gpio_a);
    gpio_set_direction(gpio_b, GPIO_MODE_INPUT);
    gpio_pullup_en(gpio_b);

    /* ---- 第2步: 创建 PCNT 单元 ---- */
    pcnt_unit_config_t unit_cfg = {
        .high_limit = PCNT_HIGH_LIMIT,
        .low_limit  = PCNT_LOW_LIMIT,
        .flags.accum_count = true,          // 硬件 32 位累加，自动处理溢出
    };
    if (pcnt_new_unit(&unit_cfg, &enc->unit) != ESP_OK) {
        free(enc);
        return false;
    }

    /* ---- 第3步: 配置硬件毛刺滤波 (80μs, 滤电机 PWM 噪声) ---- */
    pcnt_glitch_filter_config_t filter_cfg = {
        .max_glitch_ns = ENCODER_GLITCH_FILTER_NS
    };
    pcnt_unit_set_glitch_filter(enc->unit, &filter_cfg);

    /* ---- 第4步: 创建 A 相正交通道 ---- */
    pcnt_chan_config_t chan_cfg = {
        .edge_gpio_num  = gpio_a,
        .level_gpio_num = gpio_b,
    };
    if (pcnt_new_channel(enc->unit, &chan_cfg, &enc->chan_a) != ESP_OK) {
        pcnt_del_unit(enc->unit);
        free(enc);
        return false;
    }
    // A 相：上升沿时根据 B 相电平判断方向
    pcnt_channel_set_edge_action(enc->chan_a,
        PCNT_CHANNEL_EDGE_ACTION_DECREASE,   // B 低时 A 上升 → 反转
        PCNT_CHANNEL_EDGE_ACTION_INCREASE);  // B 高时 A 上升 → 正转
    pcnt_channel_set_level_action(enc->chan_a,
        PCNT_CHANNEL_LEVEL_ACTION_KEEP,      // B 低时不计
        PCNT_CHANNEL_LEVEL_ACTION_INVERSE);  // B 高时翻转

    /* ---- 第5步: 创建 B 相正交通道（实现 4 倍频）---- */
    chan_cfg.edge_gpio_num  = gpio_b;
    chan_cfg.level_gpio_num = gpio_a;
    if (pcnt_new_channel(enc->unit, &chan_cfg, &enc->chan_b) != ESP_OK) {
        pcnt_del_channel(enc->chan_a);
        pcnt_del_unit(enc->unit);
        free(enc);
        return false;
    }
    // B 相：上升沿时根据 A 相电平判断方向
    pcnt_channel_set_edge_action(enc->chan_b,
        PCNT_CHANNEL_EDGE_ACTION_INCREASE,   // A 高时 B 上升 → 正转
        PCNT_CHANNEL_EDGE_ACTION_DECREASE);  // A 低时 B 上升 → 反转
    pcnt_channel_set_level_action(enc->chan_b,
        PCNT_CHANNEL_LEVEL_ACTION_KEEP,
        PCNT_CHANNEL_LEVEL_ACTION_INVERSE);

    /* ---- 第6步: 注册溢出观测点 ---- */
    pcnt_unit_add_watch_point(enc->unit, PCNT_HIGH_LIMIT);
    pcnt_unit_add_watch_point(enc->unit, PCNT_LOW_LIMIT);

    /* ---- 第7步: 启动计数 ---- */
    pcnt_unit_enable(enc->unit);
    pcnt_unit_clear_count(enc->unit);
    pcnt_unit_start(enc->unit);

    enc->last_time_us = esp_timer_get_time();
    *out_handle = enc;

    ESP_LOGI(TAG, "✓ 编码器就绪  GPIO:%d/%d  PPR:%d  滤波:%dμs",
             gpio_a, gpio_b, enc->pulses_per_rev,
             ENCODER_GLITCH_FILTER_NS / 1000);
    return true;
}

/* ================================================================
 * 反初始化
 * ================================================================ */

bool encoder_deinit(encoder_handle_t handle)
{
    struct encoder_t *enc = (struct encoder_t *)handle;
    if (!enc) return false;

    pcnt_unit_stop(enc->unit);
    pcnt_unit_disable(enc->unit);
    pcnt_del_channel(enc->chan_a);
    pcnt_del_channel(enc->chan_b);
    pcnt_del_unit(enc->unit);
    free(enc);

    ESP_LOGI(TAG, "✓ 编码器已释放");
    return true;
}

/* ================================================================
 * 清零
 * ================================================================ */

bool encoder_reset(encoder_handle_t handle)
{
    struct encoder_t *enc = (struct encoder_t *)handle;
    if (!enc) return false;

    pcnt_unit_clear_count(enc->unit);
    enc->last_tick = 0;
    enc->last_time_us = esp_timer_get_time();
    enc->prev_tick = 0;
    enc->first_read = true;
    enc->last_direction = 0;

    return true;
}

/* ================================================================
 * 读取脉冲数（带第一层软件抗干扰）
 * ================================================================ */

int32_t encoder_get_ticks(encoder_handle_t handle)
{
    struct encoder_t *enc = (struct encoder_t *)handle;
    if (!enc) return 0;

    /* 从硬件读取 32 位累加值 */
    int raw32 = 0;
    pcnt_unit_get_count(enc->unit, &raw32);
    int32_t cur = raw32;

    /* 首次读取，初始化缓存 */
    if (enc->first_read) {
        enc->prev_tick = cur;
        enc->first_read = false;
        return cur;
    }

    /* 软件滤波：脉冲变化超合理范围 → 判定 EMI 干扰，返回旧值 */
    int32_t delta = cur - enc->prev_tick;
    if (delta < -ENCODER_MAX_DELTA_PER_READ || delta > ENCODER_MAX_DELTA_PER_READ) {
        static int64_t last_warn_us = 0;
        int64_t now = esp_timer_get_time();
        if (now - last_warn_us > 2000000) {
            ESP_LOGW(TAG, "GPIO%d/%d  ⚡ EMI 干扰  delta=%" PRId32 " (已过滤)",
                     enc->gpio_a, enc->gpio_b, delta);
            last_warn_us = now;
        }
        int32_t old = enc->prev_tick;
        enc->prev_tick = cur;  // 更新为当前值（防止持续异常）
        return old;
    }

    enc->prev_tick = cur;
    return cur;
}

/* ================================================================
 * 读取速度 m/s
 * ================================================================ */

float encoder_get_speed(encoder_handle_t handle)
{
    struct encoder_t *enc = (struct encoder_t *)handle;
    if (!enc) return 0.0f;

    int32_t cur_tick = encoder_get_ticks(handle);
    int64_t cur_time = esp_timer_get_time();

    /* ──── 速度计算：基于过滤后的 tick（防 EMI 假脉冲）──── */
    int32_t delta_tick = cur_tick - enc->last_tick;
    int64_t delta_us   = cur_time - enc->last_time_us;

    /*
     * ──── 关键修复：用真实硬件计数器更新 last_tick ────
     *
     * 当 EMI 过滤触发时，encoder_get_ticks() 返回的是旧值（prev_tick），
     * 但硬件计数器已经增长到 cur（含噪声）。如果 last_tick 用旧值更新，
     * 下一次正常读取时 delta_tick 会包含 EMI 期间累积的真实脉冲，造成
     * 虚假的 RPM 剧烈跳变（如 5045 RPM）。
     *
     * 解决方案：直接读取底层 PCNT 硬件计数器，无论是否有 EMI，
     * last_tick 始终反映硬件真实值。速度计算仍用过滤值（cur_tick），
     * 确保单次 EMI 脉冲不会污染速度输出。
     */
    int raw32 = 0;
    pcnt_unit_get_count(enc->unit, &raw32);
    enc->last_tick = (int32_t)raw32;
    enc->last_time_us = cur_time;

    if (delta_us <= 0) return enc->last_speed;

    float dist  = (float)delta_tick * enc->dist_per_tick;
    float speed = dist / ((float)delta_us / 1000000.0f);

    /* 缓存本次速度（供 get_rpm / get_direction 使用，避免重复读取） */
    enc->last_speed = speed;

    /* 更新方向缓存（基于速度符号） */
    if (speed > 0.001f) {
        enc->last_direction = 1;
    } else if (speed < -0.001f) {
        enc->last_direction = -1;
    } else {
        enc->last_direction = 0;
    }

    return speed;
}

/* ================================================================
 * 读取转速 RPM
 * ================================================================ */

float encoder_get_rpm(encoder_handle_t handle)
{
    struct encoder_t *enc = (struct encoder_t *)handle;
    if (!enc) return 0.0f;

    /*
     * 注意: 本函数使用 get_speed 缓存的最新速度值，不会重新读取硬件。
     * 因此调用顺序应为: encoder_get_speed() → encoder_get_rpm()
     * 如果直接调用 get_rpm 而从未调过 get_speed，返回 0。
     */
    if (enc->dist_per_tick <= 0.0f || enc->ticks_per_rev <= 0) {
        return 0.0f;
    }

    float ticks_per_sec = enc->last_speed / enc->dist_per_tick;       // 每秒脉冲数
    float rps = ticks_per_sec / (float)enc->ticks_per_rev;             // 每秒转数
    float rpm = rps * 60.0f;                                           // 每分钟转数

    return rpm;
}

/* ================================================================
 * 获取方向
 * ================================================================ */

int encoder_get_direction(encoder_handle_t handle)
{
    struct encoder_t *enc = (struct encoder_t *)handle;
    if (!enc) return 0;

    /* 速度已缓存方向，直接返回 */
    return enc->last_direction;
}

/* ================================================================
 * 参数设置
 * ================================================================ */

bool encoder_set_distance_per_tick(encoder_handle_t handle, float distance_m)
{
    struct encoder_t *enc = (struct encoder_t *)handle;
    if (!enc) return false;
    enc->dist_per_tick = distance_m;
    ESP_LOGI(TAG, "  脉冲当量 = %.6f mm (%.3f m/rev)",
             distance_m * 1000.0f, distance_m * enc->ticks_per_rev);
    return true;
}

bool encoder_set_pulses_per_rev(encoder_handle_t handle, int pulses_per_rev)
{
    struct encoder_t *enc = (struct encoder_t *)handle;
    if (!enc || pulses_per_rev <= 0) return false;
    enc->pulses_per_rev = pulses_per_rev;
    enc->ticks_per_rev  = pulses_per_rev * 4;  // 4 倍频
    ESP_LOGI(TAG, "  编码器线数 = %d PPR → 4倍频 = %d ticks/rev",
             pulses_per_rev, enc->ticks_per_rev);
    return true;
}
