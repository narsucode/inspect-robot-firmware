/**
 * @file    debug_log.c
 * @brief   无线调试日志缓冲区实现 — 环形 FIFO, 无动态内存分配
 *
 * @details
 * 实现要点:
 *   - 单生产者/单消费者模型: 所有写入发生在周期任务 (periodic_loop)
 *     或由其直接调用的回调中; 读取也发生在同一任务.
 *   - 因此无需互斥锁/临界区, 避免 micro-ROS 任务与日志任务竞争.
 *   - 写入时自动追加 '\n'; 读取时以 '\n' 作为行结束符,
 *     保证 /debug 发布的是完整字符串.
 */

#include "debug_log.h"
#include "car_config.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

/* ================================================================
 * 模块级状态
 * ================================================================ */

/** @brief 环形数据缓冲区 */
static char s_buf[DEBUG_LOG_BUF_SIZE];

/** @brief 写指针 (下一个要写入的位置) */
static volatile uint16_t s_head;

/** @brief 读指针 (下一个要读出的位置) */
static volatile uint16_t s_tail;

/** @brief 临时行缓冲, 用于 vsnprintf 格式化 */
static char s_line[DEBUG_LOG_MAX_LINE];

/* ================================================================
 * 内部辅助函数
 * ================================================================ */

/**
 * @brief 返回下一个位置 (带环形回绕)
 *
 * @param[in] pos  当前位置
 * @return 下一个位置, 到达末尾自动回到 0
 */
static inline uint16_t debug_log_next(uint16_t pos)
{
    return (uint16_t)((pos + 1) % DEBUG_LOG_BUF_SIZE);
}

/**
 * @brief 向环形缓冲区写入一个字节
 *
 * 若缓冲区已满, 覆盖最旧数据 (移动 s_tail).
 *
 * @param[in] ch  要写入的字符
 */
static void debug_log_put_char(char ch)
{
    uint16_t next = debug_log_next(s_head);

    /* 环形满: 覆盖最旧数据, 读指针前进 */
    if (next == s_tail) {
        s_tail = debug_log_next(s_tail);
    }

    s_buf[s_head] = ch;
    s_head = next;
}

/* ================================================================
 * 公开 API 实现
 * ================================================================ */

void debug_log_init(void)
{
    memset(s_buf, 0, sizeof(s_buf));
    s_head = 0;
    s_tail = 0;
    s_line[0] = '\0';
}

void debug_log_printf(const char *fmt, ...)
{
    if (!fmt) {
        return;
    }

    /* 1. 格式化到临时缓冲区
     *    预留 1 字节给末尾自动追加的 '\n', 避免单行撑满缓冲区
     *    导致 read_line 无法一次取走整行 */
    va_list args;
    va_start(args, fmt);
    int len = vsnprintf(s_line, sizeof(s_line) - 1, fmt, args);
    va_end(args);

    if (len <= 0) {
        return;
    }

    /* 2. 截断保护: 确保以 '\0' 结尾 */
    s_line[sizeof(s_line) - 1] = '\0';

#ifdef SERIAL_DEBUG
    /* 串口镜像: 定义 SERIAL_DEBUG 后, 调试信息同时输出到 UART */
    printf("%s", s_line);
    if (s_line[len - 1] != '\n') {
        printf("\n");
    }
#endif

    /* 3. 写入环形缓冲区, 末尾自动补 '\n' */
    for (int i = 0; i < len; i++) {
        debug_log_put_char(s_line[i]);
    }

    /* 若调用者没加换行, 自动补一个, 方便 read_line 按行切割 */
    if (len == 0 || s_line[len - 1] != '\n') {
        debug_log_put_char('\n');
    }
}

bool debug_log_read_line(char *out, size_t max_len)
{
    if (!out || max_len == 0) {
        return false;
    }

    /* 检查缓冲区中是否存在以 '\n' 结尾的完整行 */
    uint16_t scan = s_tail;
    bool found_newline = false;

    while (scan != s_head) {
        if (s_buf[scan] == '\n') {
            found_newline = true;
            break;
        }
        scan = debug_log_next(scan);
    }

    if (!found_newline) {
        return false;
    }

    /* 复制一行到 out, 包含 '\n' 并在最后补 '\0' */
    size_t copied = 0;
    while (s_tail != scan && copied < max_len - 1) {
        out[copied++] = s_buf[s_tail];
        s_tail = debug_log_next(s_tail);
    }

    /* 跳过换行符本身 (不复制到 out) */
    if (s_tail == scan) {
        s_tail = debug_log_next(s_tail);
    }

    out[copied] = '\0';
    return true;
}

bool debug_log_available(void)
{
    return (s_head != s_tail);
}
