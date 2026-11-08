/**
 * @file    voice.h
 * @brief   语音播报模块驱动 — 天问模块 UART2 文本触发
 *
 * 设计思路:
 *   1. 直观理解: ESP32 只管"发事件码"，具体念什么由天问模块里配的规则决定。
 *      固件发 "WIFI_OK\n"，天问侧配 "收到 WIFI_OK → 播 WiFi 已连接"。
 *   2. 为什么用字符串而不是单字节: 串口助手能直接看懂、上电噪声不容易误触发、
 *      新增事件只是加一个字符串，不会和旧码冲突。
 *   3. 为什么只发码不发中文: 中文走 GBK，固件要塞编码表；改措辞还要重烧固件。
 *      发码则换话术只改天问配置，固件不动。
 *
 * 接线 (ESP32-S3):
 *   GPIO40 (TX) → 天问模块 RX
 *   GPIO41 (RX) → 天问模块 TX
 *   必须共地；天问模块建议独立 5V 供电（播报瞬间电流大）
 *
 * 注意:
 *   GPIO40/41 是 ESP32-S3 的 JTAG 脚 (MTDO/MTDI)，日常运行无影响，
 *   但接 OpenOCD 在线调试时会冲突。
 */

#pragma once
#ifdef __cplusplus
extern "C" {
#endif

/* ================================================================
 * 事件码 — 与天问模块配置里的触发字符串必须一字不差
 * ================================================================ */

#define VOICE_EVT_WIFI_OK           "WIFI_OK"        /**< WiFi 已连接 */
#define VOICE_EVT_WIFI_FAIL         "WIFI_FAIL"      /**< WiFi 连接超时 */
#define VOICE_EVT_AGENT_OK          "AGENT_OK"       /**< micro-ROS Agent 已连接 */
#define VOICE_EVT_AGENT_LOST        "AGENT_LOST"     /**< micro-ROS Agent 断开 */
#define VOICE_EVT_LOW_BAT           "LOW_BAT"        /**< 电量严重不足 */

/* 以下留给 ROS 侧 /speak 透传使用，固件暂不主动发 */
#define VOICE_EVT_PATROL_START      "PATROL_START"   /**< 开始巡检 */
#define VOICE_EVT_PATROL_DONE       "PATROL_DONE"    /**< 巡检完成 */
#define VOICE_EVT_ALARM             "ALARM"          /**< 发现异常 */

/* ================================================================
 * 公开 API
 * ================================================================ */

/**
 * @brief 初始化语音串口 (UART2, 9600 8N1, TX=40 RX=41)
 * @note  必须在 app_main 中、任何 voice_say 之前调用一次
 */
void voice_init(void);

/**
 * @brief 发送一个事件码（自动补换行，带 500ms 去重）
 *
 * @param code 事件码字符串，例如 VOICE_EVT_WIFI_OK
 *
 * @note 去重规则: 与上一条完全相同且间隔 < VOICE_MIN_GAP_MS 时丢弃。
 *       这是为了防止 WiFi 抖动导致的连续重复播报。
 */
void voice_say(const char *code);

/**
 * @brief 发送事件码，跳过去重（需要立刻重复播报时使用）
 */
void voice_say_now(const char *code);

#ifdef __cplusplus
}
#endif
