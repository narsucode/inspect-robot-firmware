/**
 * @file    debug_log.h
 * @brief   无线调试日志缓冲区 — 把原本 printf 到串口的运行数据先缓存,
 *         再由 micro-ROS 周期发布到 /debug 主题.
 *
 * 三步拆解术:
 *   1. 直观理解: 想象 debug_log 是一辆小车的"黑匣子录音笔"——
 *      cmd_vel、odom、雷达等模块把想唠叨的话写进一个环形本子,
 *      ROS 这边每隔一段时间翻开本子, 把新内容通过 UDP 广播到 PC.
 *   2. 追根溯源: 没有它, 调试只能拖着 USB 线看串口; 小车跑起来后
 *      想远程看速度环数据几乎不可能.
 *   3. 正规解释: 本模块提供一个与 ROS 解耦的字符串日志环,
 *      bsp 层调用 debug_log_printf() 写入, ros 层调用
 *      debug_log_read_line() 读出并发布, 避免 bsp 直接依赖 micro-ROS.
 */

#ifndef DEBUG_LOG_H
#define DEBUG_LOG_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ================================================================
 * 配置宏
 * ================================================================ */

/** @brief 环形缓冲区总容量 (字节) */
#define DEBUG_LOG_BUF_SIZE      2048

/** @brief 单条日志最大长度 (字节, 含 '\0') */
#define DEBUG_LOG_MAX_LINE      256

/* ================================================================
 * 公开 API
 * ================================================================ */

/**
 * @brief 初始化日志缓冲区
 *
 * 清空环形缓冲区和行尾状态. 在调用 debug_log_printf() 前必须执行一次.
 */
void debug_log_init(void);

/**
 * @brief 格式化写入一条调试日志
 *
 * 行为类似 printf, 但输出被写入内部环形缓冲区而不是 UART.
 * 若缓冲区已满, 最旧的数据会被覆盖 (环形特性).
 *
 * @param[in] fmt  printf 风格格式字符串
 * @param[in] ...  可变参数
 *
 * @note  本函数在中断/任务中均可用 (当前实现无动态内存分配)
 * @note  超过 DEBUG_LOG_MAX_LINE 的日志会被截断
 */
void debug_log_printf(const char *fmt, ...);

/**
 * @brief 从缓冲区读取一行完整日志
 *
 * 按 FIFO 顺序读取一条以 '\n' 结尾的日志. 若缓冲区中没有完整行,
 * 返回 false 且 out 不被修改.
 *
 * @param[out] out      输出缓冲区, 用于存放读取到的字符串 (含 '\0')
 * @param[in]  max_len  out 缓冲区容量, 必须 >= DEBUG_LOG_MAX_LINE
 *
 * @return true=读取到一行完整日志, false=暂无完整行
 */
bool debug_log_read_line(char *out, size_t max_len);

/**
 * @brief 检查缓冲区中是否有可读数据
 *
 * @return true=缓冲区非空, false=空
 */
bool debug_log_available(void);

#ifdef __cplusplus
}
#endif

#endif /* DEBUG_LOG_H */
