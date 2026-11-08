/**
 * @file    ydlidar_x3pro.c
 * @brief   YDLIDAR X3 Pro 激光雷达 ESP-IDF 驱动 —— 实现文件
 *
 * ════════════════════ 通信协议 (YDLIDAR 三角测距系列) ════════════════════
 *
 * 雷达上电后，Tx 引脚持续输出二进制数据流，由两层结构组成:
 *
 * 【外层报文头】(上电后出现一次)
 *   A5 5A 05 00 00 40 81
 *    │  │  │        │  └── 类型码 0x81 = 扫描数据
 *    │  │  └────────── 长度 5 字节
 *    └──└── 固定标识 0xA55A
 *
 * 【内层数据包】(持续重复, 每个包 10+N*2 字节)
 *
 *   字节:   0    1    2    3    4    5    6    7    8    9    10...  10+N*2-1
 *         ┌────┬────┬────┬────┬────┬────┬────┬────┬────┬────┬─────────────┐
 *         │ PH │ PH │ CT │LSN │FSA │FSA │LSA │LSA │ CS │ CS │ S1 ... S_N  │
 *         │ 低 │ 高 │    │     │ 低 │ 高 │ 低  │ 高 │ 低 │ 高  │  (每点2字节) │
 *         └────┴────┴────┴────┴────┴────┴────┴────┴────┴────┴─────────────┘
 *           └─ 固定 0x55AA ─┘  ←────── 包头 10 字节 ──────→
 *
 * 字段说明:
 *   PH  (2B): 数据包包头, 固定 0x55AA (小端: AA 55)
 *   CT  (1B): 包类型. bit0=1 → 零位包(新一圈起点); bit0=0 → 普通点云包
 *   LSN (1B): 本包采样点数量 (范围 1~40, 零位包中固定为 1)
 *   FSA (2B): 起始角度.  解码: angle = (FSA >> 1) / 64.0 (°)
 *   LSA (2B): 结束角度.  解码同上
 *   CS  (2B): 异或校验和. 算法见 verify_checksum()
 *   Si  (2B): 采样点.    低 2 bit 质量信息 → 掩掉后得距离: dist = (Si & 0xFFFC) / 4000 (m)
 *
 * 角度插值:
 *   step = (LSA_deg - FSA_deg) / (LSN - 1)    (跨零时加 360°)
 *   angle_i = FSA_deg + step * i + angle_correction(dist_i)
 *
 * ════════════════════ 架构设计 ════════════════════
 *
 *   ┌─────────────┐     ┌────────────────┐     ┌──────────────┐
 *   │ UART1 ISR   │ ──→ │ rx_parse_task  │ ──→ │ 双缓冲       │
 *   │ (硬件 FIFO) │     │ Core1 / 优先20  │     │ s_buf[0..1]  │
 *   └─────────────┘     └────────────────┘     └──────┬───────┘
 *                                                      │
 *                            ┌─────────────────────────┘
 *                            ▼
 *              ┌──────────────────────┐
 *              │ wait_scan() 信号量    │ ← 主任务阻塞等待
 *              │ get_scan()  非阻塞    │ ← 主任务轮询获取
 *              └──────────────────────┘
 *
 * 零位包到达 → 上一圈数据完整 → 双缓冲交换 → 通知等待者
 */

#include "ydlidar_x3pro.h"

#include <inttypes.h>
#include <math.h>
#include <string.h>
#include "driver/ledc.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/task.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

static const char *TAG = "X3PRO";

/* ═════════════════════ 内部类型 ═════════════════════ */

/**
 * @brief 串口解析状态机
 *
 * 每收到一个字节就驱动一步状态转换, 直到收满一个完整数据包.
 * 状态机设计为线性流转, 任何异常字节都回到 ST_WAIT_PH1 重新同步.
 */
typedef enum {
    STATE_WAIT_PH1,   /* 等待包头第 1 字节 0xAA */
    STATE_WAIT_PH2,   /* 等待包头第 2 字节 0x55 */
    STATE_CT,         /* 读取包类型 */
    STATE_LSN,        /* 读取采样点数 */
    STATE_FSA_L,      /* 读取起始角度低字节 */
    STATE_FSA_H,      /* 读取起始角度高字节 */
    STATE_LSA_L,      /* 读取结束角度低字节 */
    STATE_LSA_H,      /* 读取结束角度高字节 */
    STATE_CS_L,       /* 读取校验和低字节 */
    STATE_CS_H,       /* 读取校验和高字节 */
    STATE_DATA,       /* 读取采样点数据 */
} ydlidar_state_t;

/**
 * @brief 单包解析上下文
 *
 * 状态机逐步填充此结构, 收满后调用 process_packet() 完成解析.
 */
typedef struct {
    uint8_t  ct;           /* 包类型, 仅 bit0 有效 */
    uint8_t  lsn;          /* 本包采样点数 */
    uint16_t fsa_raw;      /* 起始角度 (含校验位, 未右移) */
    uint16_t lsa_raw;      /* 结束角度 (含校验位, 未右移) */
    uint16_t cs;           /* 校验和 (从包中读取) */
    uint8_t  data[X3PRO_MAX_POINTS_PER_PKT * X3PRO_POINT_SIZE]; /* 采样数据缓冲区 */
    int      data_len;     /* 期望的采样数据总字节数 = lsn * 2 */
    int      data_idx;     /* 当前已读取的数据字节数 */
} ydlidar_packet_t;

/* ═════════════════════ 全局变量 ═════════════════════ */

/* 双缓冲: 写入缓冲 (接收数据) / 就绪缓冲 (供上层读取), 零位包到达时交换 */
static ydlidar_x3pro_scan_t s_scan_buffer[2];
static ydlidar_x3pro_scan_t *s_write_buf  = &s_scan_buffer[0]; /* 当前写入缓冲 */
static ydlidar_x3pro_scan_t *s_ready_buf  = NULL;              /* 已就绪缓冲 */

/* 同步原语 */
static SemaphoreHandle_t s_sem_scan  = NULL;  /* 计数信号量, 通知有新圈数据 */
static SemaphoreHandle_t s_mutex_buf = NULL;  /* 互斥锁, 保护缓冲区交换 */

/* 角度插值上下文 (数据包跨越多个包, 需要记住上一包的间隔) */
static float s_angle_step      = 0.0f;  /* 当前包内每个采样点的角度间隔 (°) */
static float s_angle_step_prev = 0.0f;  /* 上一包的角度间隔 (异常时回退) */

/* 通信质量统计 (仅用于日志, 不参与控制逻辑) */
static uint32_t s_stat_ph    = 0;  /* 累计找到的 PH 帧头次数 */
static uint32_t s_stat_ok    = 0;  /* 累计校验通过的包数 */
static uint32_t s_stat_fail  = 0;  /* 累计校验失败的包数 */
static uint32_t s_stat_ring  = 0;  /* 累计零位包数 (约等于圈数) */
static uint32_t s_stat_bytes = 0;  /* 累计接收字节数 */

/* ═════════════════════ 底层换算 (内联函数) ═════════════════════ */

/**
 * @brief 将原始采样值转换为距离 (单位: 米)
 *
 * @param raw  16 位采样值, 低 2 bit 为质量信息需掩掉
 * @return 距离 (米).  公式: distance = (raw & 0xFFFC) / 4000
 */
static inline float raw_to_distance(uint16_t raw) {
    return (float)(raw & 0xFFFC) / 4000.0f;
}

/**
 * @brief 将原始角度值转换为角度 (单位: 度)
 *
 * @param raw  16 位角度值, bit0 为固定校验位 1
 * @return 角度 (°).  公式: angle = (raw >> 1) / 64.0
 */
static inline float raw_to_angle(uint16_t raw) {
    return (float)(raw >> 1) / 64.0f;
}

/**
 * @brief 三角测距角度修正 (二级解析)
 *
 * 补偿因几何视差导致的角度偏差. 距离越近修正量越大.
 *
 * @param dist_m 测量距离 (米)
 * @return 角度修正量 (°), 距离为 0 时返回 0
 */
static inline float angle_correction(float dist_m) {
    if (dist_m <= 0.0f) return 0.0f;
    float dist_mm = dist_m * 1000.0f;
    if (dist_mm <= 0.0f) return 0.0f;
    return atanf((21.8f * (155.3f - dist_mm)) / (155.3f * dist_mm))
           * (180.0f / (float)M_PI);
}

/**
 * @brief 角度归一化到 [0, 360) 区间
 */
static inline float normalize_angle(float deg) {
    while (deg >= 360.0f) deg -= 360.0f;
    while (deg < 0.0f)   deg += 360.0f;
    return deg;
}

/* ═════════════════════ 数据包校验 ═════════════════════ */

/**
 * @brief 异或校验和验证
 *
 * 算法 (与 YDLIDAR SDK 一致):
 *   checksum = 0x55AA
 *            ^ ((LSN << 8) | CT)
 *            ^ FSA_raw
 *            ^ LSA_raw
 *            ^ S1 ^ S2 ^ ... ^ S_N
 *
 * 校验和本身不参与异或运算; 异或满足交换律, 顺序不严格.
 *
 * @return true 校验通过, false 数据损坏, 整包丢弃
 */
static bool verify_checksum(const ydlidar_packet_t *pkt) {
    uint16_t calc = 0x55AA;
    calc ^= ((uint16_t)pkt->lsn << 8) | pkt->ct;
    calc ^= pkt->fsa_raw;
    calc ^= pkt->lsa_raw;
    for (int i = 0; i < pkt->lsn; i++) {
        int offset = i * X3PRO_POINT_SIZE;
        calc ^= ((uint16_t)pkt->data[offset + 1] << 8) | pkt->data[offset];
    }
    return calc == pkt->cs;
}

/* ═════════════════════ 角度插值计算 ═════════════════════ */

/**
 * @brief 计算当前数据包内每个采样点的角度间隔
 *
 * 特殊情况:
 *   - LSA < FSA 且跨越 0°/360° 边界 → 正常跨零, 补充 360°
 *   - LSA < FSA 但不跨零        → 异常, 沿用上一包间隔 (避免跳变)
 */
static void calc_angle_step(const ydlidar_packet_t *pkt) {
    if (pkt->lsn <= 1) {
        s_angle_step = 0.0f;
        return;
    }

    float fsa_q6 = (float)(pkt->fsa_raw >> 1);  /* 去校验位后 = 角度 × 64 */
    float lsa_q6 = (float)(pkt->lsa_raw >> 1);

    if (lsa_q6 < fsa_q6) {
        /* LSA 数值上小于 FSA → 可能跨 0° */
        if (fsa_q6 > 270.0f * 64.0f && lsa_q6 < 90.0f * 64.0f) {
            /* 正常跨零: 从 340° 扫到 20° */
            float diff = (360.0f * 64.0f + lsa_q6 - fsa_q6) / 64.0f;
            s_angle_step = diff / (float)(pkt->lsn - 1);
        } else {
            /* 异常: 可能是数据错位, 沿用上一包间隔避免跳变 */
            s_angle_step = s_angle_step_prev;
        }
    } else {
        /* 正常情况: LSA > FSA */
        float diff = (lsa_q6 - fsa_q6) / 64.0f;
        s_angle_step = diff / (float)(pkt->lsn - 1);
    }
    s_angle_step_prev = s_angle_step;
}

/* ═════════════════════ 数据包解析 ═════════════════════ */

/**
 * @brief 解析一个完整数据包, 提取所有采样点的距离和角度
 *
 * 这是驱动最核心的解析函数. 流程:
 *   1. 异或校验 → 失败则丢弃
 *   2. 计算角度间隔
 *   3. 如果 CT[bit0]=1 → 零位包: 交换双缓冲, 通知上层有新圈数据
 *   4. 遍历每个采样点: 解码距离 → 插值角度 → 修正角度 → 存入缓冲
 *   5. 同时填充 ROS 标准 360 格点数组 (ranges[360])
 */
static void process_packet(const ydlidar_packet_t *pkt) {
    /* ── 1. 校验 ── */
    if (!verify_checksum(pkt)) {
        s_stat_fail++;
        return;
    }
    s_stat_ok++;

    /* ── 2. 角度间隔 ── */
    calc_angle_step(pkt);
    float fsa_deg = raw_to_angle(pkt->fsa_raw);

    /* ── 3. 零位包检测 ── */
    bool is_ring_start = (pkt->ct & 0x01) != 0;
    if (is_ring_start) {
        s_stat_ring++;
    }

    if (is_ring_start && s_write_buf->point_count > 0) {
        /* 上一圈数据完整 → 发布就绪缓冲, 切换写入缓冲 */
        xSemaphoreTake(s_mutex_buf, portMAX_DELAY);

        s_write_buf->ready = true;
        s_write_buf->timestamp_us = esp_timer_get_time();
        s_ready_buf = s_write_buf;

        /* 双缓冲交换: 当前写入缓冲变只读, 另一个变新写入区 */
        s_write_buf = &s_scan_buffer[
            (s_write_buf == &s_scan_buffer[0]) ? 1 : 0
        ];
        s_write_buf->point_count = 0;
        s_write_buf->ready = false;
        memset(s_write_buf->ranges_filled, 0,
               sizeof(s_write_buf->ranges_filled));

        xSemaphoreGive(s_mutex_buf);
        xSemaphoreGive(s_sem_scan);  /* 通知 wait_scan() 的调用者 */
    }

    /* ── 4. 遍历本包所有采样点 ── */
    for (int i = 0; i < pkt->lsn; i++) {
        if (s_write_buf->point_count >= X3PRO_MAX_SCAN_POINTS) break;

        /* 4.1 解码距离 */
        int offset = i * X3PRO_POINT_SIZE;
        uint16_t si = ((uint16_t)pkt->data[offset + 1] << 8)
                    | pkt->data[offset];
        float distance = raw_to_distance(si);

        /* 4.2 解码角度 (线性插值 + 二级修正) */
        float angle = fsa_deg + s_angle_step * (float)i;
        if (distance >= 0.12f) {  /* 盲区 12cm 以内不做修正 */
            angle += angle_correction(distance);
        }
        angle = normalize_angle(angle);

        /* 4.3 写入原始数据 */
        s_write_buf->distances[s_write_buf->point_count]   = distance;
        s_write_buf->angles[s_write_buf->point_count]      = angle;
        s_write_buf->intensities[s_write_buf->point_count] = 0.0f;

        /* 4.4 写入 ROS 标准 360 格点数组
         *     将角度映射为 0~359 的整数索引 (每格 1°)
         *     同一格有多个点时保留最近距离 (适合避障场景)
         *     约定: ranges[0] 对应 0°(正前方), ranges[180] 对应 180°(后方)
         *     注意: 跳过零距离点 (无回波), 不污染 ranges_filled 计数 */
        if (distance > 0.0f) {
            int idx = (int)(angle + 0.5f);
            if (idx >= 360) idx -= 360;
            if (idx >= 0 && idx < 360) {
                if (!s_write_buf->ranges_filled[idx] ||
                    distance < s_write_buf->ranges[idx]) {
                    s_write_buf->ranges[idx] = distance;
                    s_write_buf->ranges_filled[idx] = 1;
                }
            }
        }

        s_write_buf->point_count++;
    }
}

/* ═════════════════════ 串口接收 & 解析协程 ═════════════════════ */

/**
 * @brief 后台协程: 从 UART1 接收字节流, 逐字节驱动状态机, 解析数据包
 *
 * 运行在 Core 1, 优先级 20, 栈 8KB.
 * 设计要点:
 *   - 每次读取最多 512 字节 (1 tick 超时), 避免单次阻塞过长
 *   - 状态机在收到 0xAA 后等 0x55, 其他字节直接跳过 —— 自动从乱流中同步
 *   - 每 3 秒输出一次通信质量统计
 */
static void rx_parse_task(void *arg) {
    (void)arg;
    uint8_t rx_buf[512];
    ydlidar_state_t state = STATE_WAIT_PH1;
    ydlidar_packet_t packet;
    uint32_t last_log_tick = 0;

    ESP_LOGI(TAG, "[协程启动] Core%d  波特率=%d",
             xPortGetCoreID(), X3PRO_BAUDRATE);

    while (1) {
        /* 从 UART 读取数据, 1 tick 超时避免饿死低优先级任务 */
        int len = uart_read_bytes(X3PRO_UART_NUM, rx_buf,
                                   sizeof(rx_buf), 1);

        /* ── 周期性健康统计 (每 3 秒) ── */
        uint32_t now = xTaskGetTickCount();
        if (now - last_log_tick >= pdMS_TO_TICKS(3000)) {
            last_log_tick = now;
            ESP_LOGI(TAG,
                "[统计] 总字节=%" PRIu32 "  PH发现=%" PRIu32 "  OK=%" PRIu32 "  FAIL=%" PRIu32 "  累计圈数=%" PRIu32,
                s_stat_bytes, s_stat_ph, s_stat_ok,
                s_stat_fail, s_stat_ring);
        }

        if (len <= 0) {
            vTaskDelay(1);
            continue;
        }

        /* ── 逐字节驱动状态机 ── */
        for (int i = 0; i < len; i++) {
            uint8_t byte = rx_buf[i];
            s_stat_bytes++;

            switch (state) {
            /* ─ 等 0xAA ─ */
            case STATE_WAIT_PH1:
                if (byte == 0xAA) state = STATE_WAIT_PH2;
                break;

            /* ─ 等 0x55 → 帧头确认 ─ */
            case STATE_WAIT_PH2:
                if (byte == 0x55) {
                    s_stat_ph++;
                    state = STATE_CT;
                    memset(&packet, 0, sizeof(packet));
                } else if (byte != 0xAA) {
                    /* 连 0xAA 都不是 → 前面那个 0xAA 是误判, 从头找 */
                    state = STATE_WAIT_PH1;
                }
                /* else byte == 0xAA: 连续两个 0xAA, 停在 STATE_WAIT_PH2 等 0x55 */
                break;

            /* ─ 包类型 ─ */
            case STATE_CT:
                packet.ct = byte;
                state = STATE_LSN;
                break;

            /* ─ 采样点数 + 合法性检查 ─ */
            case STATE_LSN:
                packet.lsn = byte;
                if (byte == 0 || byte > X3PRO_MAX_POINTS_PER_PKT) {
                    /* LSN 越界 → 可能误同步, 重新开始找帧头 */
                    state = STATE_WAIT_PH1;
                    break;
                }
                packet.data_len = byte * X3PRO_POINT_SIZE;
                packet.data_idx = 0;
                state = STATE_FSA_L;
                break;

            /* ─ 起始角度 (2 字节, 小端) ─ */
            case STATE_FSA_L:
                packet.fsa_raw = byte;
                state = STATE_FSA_H;
                break;
            case STATE_FSA_H:
                packet.fsa_raw |= ((uint16_t)byte << 8);
                state = STATE_LSA_L;
                break;

            /* ─ 结束角度 (2 字节, 小端) ─ */
            case STATE_LSA_L:
                packet.lsa_raw = byte;
                state = STATE_LSA_H;
                break;
            case STATE_LSA_H:
                packet.lsa_raw |= ((uint16_t)byte << 8);
                state = STATE_CS_L;
                break;

            /* ─ 校验和 (2 字节, 小端) ─ */
            case STATE_CS_L:
                packet.cs = byte;
                state = STATE_CS_H;
                break;
            case STATE_CS_H:
                packet.cs |= ((uint16_t)byte << 8);
                state = (packet.data_len > 0) ? STATE_DATA : STATE_WAIT_PH1;
                break;

            /* ─ 采样数据 ─ */
            case STATE_DATA:
                if (packet.data_idx < (int)sizeof(packet.data)) {
                    packet.data[packet.data_idx++] = byte;
                }
                if (packet.data_idx >= packet.data_len) {
                    /* 收满一包 → 立即解析 → 回到帧同步状态 */
                    process_packet(&packet);
                    state = STATE_WAIT_PH1;
                }
                break;
            }
        }
    }
}

/* ═════════════════════ PWM 电机控制 ═════════════════════ */

/**
 * @brief 初始化 LEDC PWM 通道
 *
 * 配置: 定时器 0, 通道 0, 10KHz, 10 位分辨率 (0~1023)
 * 初始占空比为 0 (电机停止).
 *
 * ⚠️ 必须使用 PWM 而非 GPIO 恒高电平.
 *    恒高虽能让电机转, 但雷达内部数据时钟会失步, 导致输出乱码.
 */
static void pwm_init(int gpio) {
    ledc_timer_config_t timer_cfg = {
        .speed_mode      = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_10_BIT,
        .timer_num       = LEDC_TIMER_0,
        .freq_hz         = X3PRO_PWM_FREQ,
        .clk_cfg         = LEDC_AUTO_CLK,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&timer_cfg));

    ledc_channel_config_t ch_cfg = {
        .gpio_num   = gpio,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel    = LEDC_CHANNEL_0,
        .timer_sel  = LEDC_TIMER_0,
        .duty       = 0,      /* 初始停止 */
        .hpoint     = 0,
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ch_cfg));

    ESP_LOGI(TAG, "[PWM ] GPIO%d  %dHz  初始占空比=0", gpio, X3PRO_PWM_FREQ);
}

/**
 * @brief 设置 PWM 占空比
 * @param duty 占空比原始值 (0~1023)
 */
static void pwm_set_duty(uint32_t duty) {
    if (duty > 1023) duty = 1023;
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
}

/* ═════════════════════ 公开 API 实现 ═════════════════════ */

/**
 * @brief 初始化雷达驱动
 *
 * 步骤:
 *   1. 重置双缓冲
 *   2. 配置 UART1 (115200-8N1, RX=rx_gpio, TX 不用)
 *   3. 配置 LEDC PWM (GPIO=mctr_gpio, 10KHz, 初始停止)
 *   4. 创建信号量 (计数 8 用于缓冲, 互斥锁用于交换)
 *   5. 创建后台解析协程 (Core 1, 优先级 20, 栈 8KB)
 */
void ydlidar_x3pro_init(int rx_gpio, int mctr_gpio) {
    /* 重置状态 */
    memset(s_scan_buffer, 0, sizeof(s_scan_buffer));
    s_write_buf = &s_scan_buffer[0];
    s_ready_buf = NULL;
    s_angle_step = s_angle_step_prev = 0.0f;

    /* UART1 配置 */
    uart_config_t uart_cfg = {
        .baud_rate  = X3PRO_BAUDRATE,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    ESP_ERROR_CHECK(uart_param_config(X3PRO_UART_NUM, &uart_cfg));
    ESP_ERROR_CHECK(uart_set_pin(X3PRO_UART_NUM,
                                  UART_PIN_NO_CHANGE,  /* TX 不用 */
                                  rx_gpio,              /* RX 接雷达 Tx */
                                  UART_PIN_NO_CHANGE,   /* RTS 不用 */
                                  UART_PIN_NO_CHANGE)); /* CTS 不用 */
    ESP_ERROR_CHECK(uart_driver_install(X3PRO_UART_NUM,
                                         8192,           /* RX 缓冲区 8KB */
                                         0,              /* TX 缓冲区不用 */
                                         0, NULL, 0));

    /* PWM 配置 (电机控制) */
    pwm_init(mctr_gpio);
    pwm_set_duty(0);  /* 初始停止 */

    /* 同步原语 */
    s_sem_scan  = xSemaphoreCreateCounting(8, 0);
    s_mutex_buf = xSemaphoreCreateMutex();

    /* 后台解析协程 */
    xTaskCreatePinnedToCore(rx_parse_task, "x3pro_rx",
                            8192,   /* 栈 8KB */
                            NULL,
                            20,     /* 高优先级, 确保不丢数据 */
                            NULL,
                            1);     /* Core 1, 避免干扰 WiFi */

    ESP_LOGI(TAG, "[初始化] UART%d IO%d(RX) IO%d(MCTR) %dbps",
             X3PRO_UART_NUM, rx_gpio, mctr_gpio, X3PRO_BAUDRATE);
}

void ydlidar_x3pro_start(void) {
    /* 启动电机: PWM 50% ≈ 6Hz 扫描频率.
     * X3 Pro 没有 Rx 引脚, 上电 + M_CTR PWM 后自动开始输出数据,
     * 无需发送停止/启动指令. */
    pwm_set_duty(850);   /* 约 10Hz，减少转弯时的扫描拖影 */
    ESP_LOGI(TAG, "[启动] 电机 PWM=512/1023 (50%%), 等待稳定...");
    vTaskDelay(pdMS_TO_TICKS(2500));

    ESP_LOGI(TAG, "[就绪] 雷达数据流已启动");
}

void ydlidar_x3pro_stop(void) {
    /* X3 Pro 无 Rx 引脚, 直接停 PWM 即可 */
    pwm_set_duty(0);  /* 占空比归零 → 电机停止 */
    ESP_LOGI(TAG, "[停止] 电机已停转");
}

const ydlidar_x3pro_scan_t *ydlidar_x3pro_get_scan(void) {
    xSemaphoreTake(s_mutex_buf, portMAX_DELAY);
    const ydlidar_x3pro_scan_t *scan = s_ready_buf;
    xSemaphoreGive(s_mutex_buf);
    return scan;
}

const ydlidar_x3pro_scan_t *ydlidar_x3pro_wait_scan(uint32_t timeout_ms) {
    TickType_t ticks = (timeout_ms == 0)
                       ? portMAX_DELAY
                       : pdMS_TO_TICKS(timeout_ms);

    if (xSemaphoreTake(s_sem_scan, ticks) != pdTRUE) {
        return NULL;  /* 超时, 雷达可能无数据 */
    }

    xSemaphoreTake(s_mutex_buf, portMAX_DELAY);
    const ydlidar_x3pro_scan_t *scan = s_ready_buf;
    xSemaphoreGive(s_mutex_buf);
    return scan;
}

void ydlidar_x3pro_set_speed(uint8_t pct) {
    if (pct > 100) pct = 100;
    uint32_t duty = ((uint32_t)pct * 1023) / 100;
    pwm_set_duty(duty);
}

/* ═════════════════════ 调试测试函数 ═════════════════════ */

/**
 * @brief 雷达调试测试: 初始化 + 启动 + 每 5 秒打印全景
 *
 * 用于快速验证雷达硬件是否正常工作. 阻塞运行, 不会返回.
 * 每 30 圈 (约 5 秒) 打印一次 12 方向速览 + 360° 有效点统计.
 *
 * @param rx_gpio   接雷达 Tx 引脚的 GPIO 编号
 * @param mctr_gpio 接雷达 M_CTR 引脚的 GPIO 编号
 */
void ydlidar_x3pro_test(int rx_gpio, int mctr_gpio) {
    static const char *TAG_TEST = "X3TEST";
    float dirs[12] = {0,30,60,90,120,150,180,210,240,270,300,330};
    int frame = 0;

    ydlidar_x3pro_init(rx_gpio, mctr_gpio);
    ydlidar_x3pro_start();

    while (1) {
        const ydlidar_x3pro_scan_t *s = ydlidar_x3pro_wait_scan(2000);
        if (!s) {
            ESP_LOGW(TAG_TEST, "雷达超时, 检查接线/供电");
            continue;
        }
        frame++;

        /* 每 30 圈打印一次 (约 5 秒) */
        if (frame % 30 != 0) continue;

        ESP_LOGI(TAG_TEST, "=== 第%d圈: %d原始点 ===", frame, s->point_count);

        /* 12 方向速览 (ranges[idx] 中 idx 直接对应角度) */
        for (int d = 0; d < 12; d++) {
            int idx = (int)(dirs[d] + 0.5f);
            if (idx >= 360) idx -= 360;
            ESP_LOGI(TAG_TEST, "  %3.0f°  %5.3fm", dirs[d], s->ranges[idx]);
        }

        /* 360° 有效方向统计 */
        int valid = 0;
        float min_d = 999.0f, max_d = 0.0f;
        for (int i = 0; i < 360; i++) {
            if (s->ranges_filled[i]) {
                valid++;
                if (s->ranges[i] < min_d) min_d = s->ranges[i];
                if (s->ranges[i] > max_d) max_d = s->ranges[i];
            }
        }
        ESP_LOGI(TAG_TEST, "  [有效%d/360方向, 最近%.3fm, 最远%.3fm]",
                 valid, min_d, max_d);
    }
}
