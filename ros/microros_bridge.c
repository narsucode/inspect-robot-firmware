/**
 * @file    microros_bridge.c
 * @brief   micro-ROS 通信桥实现 — 订阅 /beep /cmd_vel + 定时发布电池/IMU/里程计/雷达/调试日志
 *
 * @details
 * 职责:
 *   - 订阅 /beep 话题 → beep_on_time() 控制蜂鸣器
 *   - 订阅 /cmd_vel 话题 → cmd_vel_set_command() 驱动差速电机
 *   - 订阅 /cmd_ptz_angle 话题 → vision_gimbal_set_angle() 云台角度遥控 (x=pan,y=tilt)
 *   - 定时发布 /battery 话题 (0.2Hz) → ADC 采样 → UInt16 发布
 *   - 定时发布 /imu 话题 (25Hz) → MPU6050 + EMA + Madgwick → sensor_msgs/Imu
 *   - 定时发布 /odom 话题 (20Hz) → 编码器脉冲 → 中点法航位推算 → nav_msgs/Odometry
 *   - 定时发布 /scan 话题 (10Hz) → YDLIDAR X3 Pro → sensor_msgs/LaserScan
 *   - 定时发布 /debug 话题 (5Hz) → 从 debug_log 环形缓冲区读取字符串 → std_msgs/String
 *   - 提供 microros_bridge_spin() 给主循环驱动执行器
 *
 * 单节点多角色:
 *   同一个 ROS2 节点 "esp32_car_sensors" 同时充当
 *   /beep、/cmd_vel、/cmd_ptz_angle 的订阅者和 /battery、/imu、/odom、/scan、/debug 的发布者，
 *   由执行器统一调度。
 *
 * 依赖:
 *   - micro_ros_espidf_component (rcl, rclc, rmw_microxrcedds)
 *   - bsp/beep (蜂鸣器驱动)
 *   - bsp/battery (ADC 电压检测驱动)
 *   - bsp/debug_log (调试日志环形缓冲)
 *   - bsp/mpu6050 (IMU 驱动 + 滤波)
 *   - bsp/encoder (编码器驱动)
 *   - bsp/odometry (里程计驱动)
 *   - bsp/ydlidar_x3pro (激光雷达驱动)
 */

#include "microros_bridge.h"

#include "voice.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <inttypes.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/u_int16.h>
#include <std_msgs/msg/string.h>
#include <sensor_msgs/msg/imu.h>
#include <sensor_msgs/msg/laser_scan.h>
#include <nav_msgs/msg/odometry.h>
#include <geometry_msgs/msg/twist.h>
#include <geometry_msgs/msg/vector3.h>
#include <rmw_microros/rmw_microros.h>
#include <rmw_microros/time_sync.h>

#include "beep.h"
#include "battery.h"
#include "debug_log.h"
#include "mpu6050.h"
#include "odometry.h"
#include "cmd_vel_control.h"
#include "ydlidar_x3pro.h"
#include "vision_gimbal.h"

/* ================================================================
 * 错误处理宏 (参考 micro-ROS 官方示例)
 * ================================================================ */

#define RCCHECK(fn)                                       \
    {                                                     \
        rcl_ret_t _rc = (fn);                             \
        if (_rc != RCL_RET_OK) {                          \
            printf("FATAL at line %d: %d\n",              \
                   __LINE__, (int)_rc);                    \
            vTaskDelete(NULL);                            \
        }                                                 \
    }

#define RCSOFTCHECK(fn)                                   \
    {                                                     \
        rcl_ret_t _rc = (fn);                             \
        if (_rc != RCL_RET_OK) {                          \
            printf("WARN at line %d: %d\n",               \
                   __LINE__, (int)_rc);                    \
        }                                                 \
    }

#define RCIGNORE(fn)                                      \
    do {                                                  \
        rcl_ret_t _rc __attribute__((unused)) = (fn);     \
        (void)_rc;                                        \
    } while (0)

/* ================================================================
 * 模块级全局变量
 * ================================================================ */

static rcl_subscription_t     g_beep_subscriber;
static std_msgs__msg__UInt16  g_beep_msg;

/* ── /speak 订阅 (语音播报) ── */
static rcl_subscription_t     g_speak_subscriber;
static std_msgs__msg__String  g_speak_msg;
/* std_msgs/String 是变长类型，接收缓冲必须自己提供，否则反序列化写 NULL 指针 */
static char                   g_speak_buf[64];
static rcl_publisher_t        g_battery_publisher;
static std_msgs__msg__UInt16  g_battery_msg;
static rcl_timer_t            g_timer;
static rclc_executor_t        g_executor;
static rclc_support_t         g_support;
static rcl_node_t             g_node;
static rcl_allocator_t        g_allocator;

/* ── IMU ── */
static rcl_publisher_t        g_imu_publisher;
static sensor_msgs__msg__Imu  g_imu_msg;
static rcl_timer_t            g_imu_timer;
static bool                   g_imu_ready = false;

/* ── 里程计 ── */
static rcl_publisher_t        g_odom_publisher;
static nav_msgs__msg__Odometry g_odom_msg;
static rcl_timer_t            g_odom_timer;

/* ── /cmd_vel 订阅 ── */
static rcl_subscription_t        g_cmd_vel_subscriber;
static geometry_msgs__msg__Twist  g_cmd_vel_msg;

/* ── /cmd_ptz_angle 订阅 (云台角度遥控) ── */
static rcl_subscription_t           g_cmd_ptz_subscriber;

/* 最近一次收到 /cmd_ptz_angle 的时间，供 OLED 刷新策略判断"云台是否在活跃控制中" */
static volatile int64_t              s_last_ptz_us = 0;
static geometry_msgs__msg__Vector3  g_cmd_ptz_msg;

/* ── 激光雷达 ── */
static rcl_publisher_t              g_scan_publisher;
static sensor_msgs__msg__LaserScan  g_scan_msg;
static rcl_timer_t                  g_scan_timer;
static int64_t                      g_scan_last_stamp_us = 0;

/* ── /debug 调试日志 ── */
static rcl_publisher_t         g_debug_publisher;
static std_msgs__msg__String   g_debug_msg;
static char                    g_debug_line[DEBUG_LOG_MAX_LINE];

/* ── Agent 连接状态 (OLED 显示用) ── */
static bool g_agent_connected = false;

/* ── 初始化完成标志 (防止 spin 在 executor 未初始化时崩溃) ── */
static bool g_init_done = false;

/* ================================================================
 * beep_callback — 订阅 /beep 话题
 * ================================================================ */

static void beep_callback(const void *msgin)
{
    const std_msgs__msg__UInt16 *msg = (const std_msgs__msg__UInt16 *)msgin;
    debug_log_printf("[ROS] %s msg: data=%u\n", MICROROS_BEEP_TOPIC, msg->data);
    beep_on_time(msg->data);
}

/* ================================================================
 * speak_callback — 订阅 /speak 话题 → 转发给语音模块
 * ================================================================ */

static void speak_callback(const void *msgin)
{
    const std_msgs__msg__String *msg = (const std_msgs__msg__String *)msgin;

    if (msg->data.data == NULL || msg->data.size == 0) {
        return;
    }

    /* 拷到本地缓冲并补结束符: 消息里的 data 不一定保证以 '\0' 结尾 */
    char code[40];
    size_t n = msg->data.size;
    if (n >= sizeof(code)) {
        n = sizeof(code) - 1;
    }
    memcpy(code, msg->data.data, n);
    code[n] = '\0';
    /* 去掉可能带的换行/空格, 避免和天问侧的匹配串对不上 */
    while (n > 0 && (code[n - 1] == '\n' || code[n - 1] == '\r' || code[n - 1] == ' ')) {
        code[--n] = '\0';
    }
    if (n == 0) {
        return;
    }

    debug_log_printf("[ROS] %s msg: %s\n", MICROROS_SPEAK_TOPIC, code);
    voice_say_now(code);   /* 上位机主动要求播报, 跳过去重 */
}

/* ================================================================
 * cmd_vel_callback — 订阅 /cmd_vel 话题 → 差速驱动回调函数
 * ================================================================ */
static void cmd_vel_callback(const void *msgin)
{
    // msgin是通用void指针，强制转为Twist结构体指针，才能访问消息成员
    const geometry_msgs__msg__Twist *msg = (const geometry_msgs__msg__Twist *)msgin;

    float linear_x  = (float)msg->linear.x;
    float angular_z = (float)msg->angular.z;

    // 将速度指令传入电机处理函数，做差速解算、控制轮子
    cmd_vel_set_command(linear_x, angular_z);
}

/* ================================================================
 * cmd_ptz_callback — 订阅 /cmd_ptz_angle 话题 → 云台角度遥控
 * ================================================================ */
static void cmd_ptz_callback(const void *msgin)
{
    // msgin是通用void指针，强制转为Vector3结构体指针，才能访问消息成员
    const geometry_msgs__msg__Vector3 *msg =
        (const geometry_msgs__msg__Vector3 *)msgin;

    // 提取云台水平、俯仰角度
    float pan_angle  = (float)msg->x;
    float tilt_angle = (float)msg->y;

    /* 记录最近一次云台指令时间（OLED 刷新策略要用） */
    s_last_ptz_us = esp_timer_get_time();

    /* 调试输出节流到 1Hz：云台跟随是 10Hz，每条都打会白占 WiFi 带宽 */
    {
        static int64_t s_last_log_us = 0;
        int64_t now_us = esp_timer_get_time();
        if (now_us - s_last_log_us >= 1000000LL) {
            debug_log_printf("[ROS] /cmd_ptz_angle 水平角度=%.1f°, 俯仰角度=%.1f°\n",
                             pan_angle, tilt_angle);
            s_last_log_us = now_us;
        }
    }

    // 将角度指令传入云台驱动函数，内部做角度限位并切换遥控模式
    vision_gimbal_set_angle(pan_angle, tilt_angle);
}

/**
 * @brief 云台是否处于活跃遥控状态（最近 1 秒内收到过 /cmd_ptz_angle）
 *
 * @details 用于 OLED 刷新策略：云台跟随期间不刷屏，避免 I2C 阻塞主循环，
 *          否则云台指令会被推迟几十毫秒，表现为跟随"一顿一顿"。
 */
bool microros_bridge_is_ptz_active(void)
{
    if (s_last_ptz_us == 0) {
        return false;
    }
    return (esp_timer_get_time() - s_last_ptz_us) < 1000000LL;
}

/* ================================================================
 * battery_timer_callback — 定时发布 /battery 话题
 * ================================================================ */

static void battery_timer_callback(rcl_timer_t *timer,
                                    int64_t last_call_time)
{
    (void)timer;
    (void)last_call_time;

    battery_update();
    float voltage = battery_get_voltage();
    g_battery_msg.data = (uint16_t)(voltage * 10.0f);
    RCSOFTCHECK(rcl_publish(&g_battery_publisher, &g_battery_msg, NULL));

    debug_log_printf("[ROS] %s published: data=%u (%.1fV)\n",
                     MICROROS_BATTERY_TOPIC, g_battery_msg.data, voltage);
}

/* ================================================================
 * imu_msg_init_covariance — 一次性填充 IMU 协方差矩阵
 * ================================================================ */

static void imu_msg_init_covariance(sensor_msgs__msg__Imu *msg)
{
    /* roll / pitch: Madgwick 可修正, yaw: 无磁力计标记不可靠 */
    msg->orientation_covariance[0]  = 0.01f;
    msg->orientation_covariance[4]  = 0.01f;
    msg->orientation_covariance[8]  = 999.0f;

    msg->angular_velocity_covariance[0] = 0.001f;
    msg->angular_velocity_covariance[4] = 0.001f;
    msg->angular_velocity_covariance[8] = 0.001f;

    msg->linear_acceleration_covariance[0] = 0.01f;
    msg->linear_acceleration_covariance[4] = 0.01f;
    msg->linear_acceleration_covariance[8] = 0.01f;
}

/* ================================================================
 * imu_timer_callback — 定时发布 /imu 话题 (25Hz)
 * ================================================================ */

static void imu_timer_callback(rcl_timer_t *timer,
                                int64_t last_call_time)
{
    (void)timer;
    (void)last_call_time;

    if (!g_imu_ready) return;

    imu_ros_data_t imu;
    if (!imu_read_ros(&imu)) {
        debug_log_printf("[ROS] IMU read failed\n");
        return;
    }

    int64_t time_ms = rmw_uros_epoch_millis();
    g_imu_msg.header.stamp.sec     = (int32_t)(time_ms / 1000);
    g_imu_msg.header.stamp.nanosec = (uint32_t)((time_ms % 1000) * 1000000);
    g_imu_msg.header.frame_id.data = (char *)MICROROS_IMU_FRAME_ID;
    g_imu_msg.header.frame_id.size = strlen(MICROROS_IMU_FRAME_ID);
    g_imu_msg.header.frame_id.capacity = strlen(MICROROS_IMU_FRAME_ID) + 1;

    g_imu_msg.orientation.w = imu.qw;
    g_imu_msg.orientation.x = imu.qx;
    g_imu_msg.orientation.y = imu.qy;
    g_imu_msg.orientation.z = imu.qz;
    g_imu_msg.angular_velocity.x = imu.gx;
    g_imu_msg.angular_velocity.y = imu.gy;
    g_imu_msg.angular_velocity.z = imu.gz;
    g_imu_msg.linear_acceleration.x = imu.ax;
    g_imu_msg.linear_acceleration.y = imu.ay;
    g_imu_msg.linear_acceleration.z = imu.az;

    RCSOFTCHECK(rcl_publish(&g_imu_publisher, &g_imu_msg, NULL));
}

/* ================================================================
 * odom_timer_callback — 定时发布 /odom 话题 (20Hz)
 * ================================================================ */

/**
 * @brief  20Hz 定时器回调 — 读编码器 → 航位推算 → 发布 nav_msgs/Odometry
 *
 * @details
 * 完整数据流:
 *   encoder_get_ticks() × 2 → odom_update() 中点法积分
 *   → 填充 nav_msgs/Odometry (位姿 + 速度 + 协方差)
 *   → rcl_publish() → UDP 广播
 *
 * odom 坐标系 (frame_id="odom") 的原点在里程计首次初始化处。
 * child_frame_id="base_link" 表示机器人本体。
 *
 * 协方差说明:
 *   - pose 协方差: 对角阵, 随行驶距离线性增长 (编码器累积误差)
 *   - twist 协方差: 对角阵, 编码器瞬时噪声
 *
 * @param[in] timer           触发此回调的定时器句柄 (未使用)
 * @param[in] last_call_time  上次调用的时间戳 (未使用)
 * @param[in] arg             用户自定义参数 (未使用)
 */
static void odom_timer_callback(rcl_timer_t *timer,
                                 int64_t last_call_time)
{
    (void)timer;
    (void)last_call_time;

    /* 1. 模块内部读取编码器 + 更新位姿 */
    odom_poll();

    /* 2. 读取里程计状态 */
    float x, y, theta, vx, vth;
    odom_get_state(&x, &y, &theta, &vx, &vth);

    /* 3. 时间戳 */
    int64_t time_ms = rmw_uros_epoch_millis();
    g_odom_msg.header.stamp.sec     = (int32_t)(time_ms / 1000);
    g_odom_msg.header.stamp.nanosec = (uint32_t)((time_ms % 1000) * 1000000);
    g_odom_msg.header.frame_id.data = (char *)MICROROS_ODOM_FRAME_ID;
    g_odom_msg.header.frame_id.size = strlen(MICROROS_ODOM_FRAME_ID);
    g_odom_msg.header.frame_id.capacity = strlen(MICROROS_ODOM_FRAME_ID) + 1;

    g_odom_msg.child_frame_id.data = (char *)MICROROS_ODOM_CHILD_FRAME_ID;
    g_odom_msg.child_frame_id.size = strlen(MICROROS_ODOM_CHILD_FRAME_ID);
    g_odom_msg.child_frame_id.capacity = strlen(MICROROS_ODOM_CHILD_FRAME_ID) + 1;

    /* 4. 位姿 (x, y, theta → 四元数) */
    g_odom_msg.pose.pose.position.x = x;
    g_odom_msg.pose.pose.position.y = y;
    g_odom_msg.pose.pose.position.z = 0.0f;

    /* theta → 四元数 (只绕 Z 轴旋转) */
    float half_theta = theta * 0.5f;
    g_odom_msg.pose.pose.orientation.x = 0.0f;
    g_odom_msg.pose.pose.orientation.y = 0.0f;
    g_odom_msg.pose.pose.orientation.z = sinf(half_theta);
    g_odom_msg.pose.pose.orientation.w = cosf(half_theta);

    /* 5. 线速度 (base_link 坐标系, 只有 x 方向) */
    g_odom_msg.twist.twist.linear.x  = vx;
    g_odom_msg.twist.twist.linear.y  = 0.0f;
    g_odom_msg.twist.twist.linear.z  = 0.0f;
    g_odom_msg.twist.twist.angular.x = 0.0f;
    g_odom_msg.twist.twist.angular.y = 0.0f;
    g_odom_msg.twist.twist.angular.z = vth;

    /* 6. 协方差矩阵 (pose: 随位移增大; twist: 固定噪声)
       — 36 个元素的数组, 按行主序 (row-major)
       — 索引: [row*6 + col]
       — 这里只设对角元, 非对角默认 0 (static 全局变量) */

    /* 6a. pose 协方差: (x, y, theta) */
    /*     位姿不确定性粗略建模为行驶距离的 2% */
    float dist_travelled = sqrtf(x * x + y * y);
    float pose_var = 0.0001f + dist_travelled * 0.02f;  /* min 1cm² + 2% drift */
    pose_var = pose_var > 0.5f ? 0.5f : pose_var;         /* cap at 0.5m² */

    g_odom_msg.pose.covariance[0]  = pose_var;   /* x 方差 */
    g_odom_msg.pose.covariance[7]  = pose_var;   /* y 方差 */
    g_odom_msg.pose.covariance[14] = 0.0f;      /* z  = 0 */
    g_odom_msg.pose.covariance[21] = 0.0f;      /* rx = 0 */
    g_odom_msg.pose.covariance[28] = 0.0f;      /* ry = 0 */
    g_odom_msg.pose.covariance[35] = 0.05f;     /* rz ≈ 0.22 rad (12.6°) */

    /* 6b. twist 协方差: (vx, vy, vz, ωx, ωy, ωz) */
    g_odom_msg.twist.covariance[0]  = 0.001f;   /* vx 方差 */
    g_odom_msg.twist.covariance[7]  = 0.0f;     /* vy = 0 */
    g_odom_msg.twist.covariance[14] = 0.0f;     /* vz = 0 */
    g_odom_msg.twist.covariance[21] = 0.0f;     /* ωx = 0 */
    g_odom_msg.twist.covariance[28] = 0.0f;     /* ωy = 0 */
    g_odom_msg.twist.covariance[35] = 0.001f;   /* ωz 方差 */

    /* 7. 发布 */
    RCSOFTCHECK(rcl_publish(&g_odom_publisher, &g_odom_msg, NULL));

    /* 8. 调试输出 (每 40 帧 ≈ 2s, 仅在有速度时打印) */
    static int odom_cnt = 0;
    if (++odom_cnt % 40 == 0 && (fabsf(vx) > 0.005f || fabsf(vth) > 0.01f)) {
        int32_t tick_l, tick_r;
        odom_get_ticks(&tick_l, &tick_r);

        /* ticks → 圈数: 1975.6 ticks/rev */
        float rev_l = (float)tick_l / TICKS_PER_WHEEL_REV;
        float rev_r = (float)tick_r / TICKS_PER_WHEEL_REV;

        debug_log_printf("[ROS] /odom  tick=(%+" PRId32 ",%+" PRId32 ")  "
                         "rev=(%.2f,%.2f)  "
                         "x=%.3f y=%.3f θ=%.1f°  vx=%.3f vth=%.2f\n",
                         tick_l, tick_r,
                         rev_l, rev_r,
                         x, y, theta * 57.2958f, vx, vth);
    }
}

/* ================================================================
 * scan_msg_init — 一次性填充 LaserScan 固定字段 + 分配 ranges 内存
 * ================================================================ */

/**
 * @brief  初始化 sensor_msgs/LaserScan 消息骨架
 *
 * 为 ranges 数组分配 360 个 float 的内存;
 * 固定字段 (角度范围、扫描时间、距离限幅) 一次性填充。
 */
static void scan_msg_init(sensor_msgs__msg__LaserScan *msg)
{
    msg->header.frame_id.data = (char *)MICROROS_SCAN_FRAME_ID;
    msg->header.frame_id.size = strlen(MICROROS_SCAN_FRAME_ID);
    msg->header.frame_id.capacity = strlen(MICROROS_SCAN_FRAME_ID) + 1;

    /* X3 Pro 扫描范围 0°~360°, 3° 分辨率
     *
     * 降采样原因: 360 方向 → CDR 序列化约 1496 字节,
     * XRCE-DDS 默认 UDP MTU=512 → 需 3 个分片 → 丢 1 片即反序列化失败.
     * 降到 120 方向 (3° 分辨率) → 约 520 字节 → 单包发送, 杜绝分片错误. */
    msg->angle_min = 0.0f;
    msg->angle_max = 2.0f * (float)M_PI;
    msg->angle_increment = 1.0f * (float)M_PI / 180.0f;

    /* 扫描频率约 6Hz → 一圈 166.7ms; 每个点耗时 ≈ 463μs */
    msg->scan_time = 1.0f / 6.0f;
    msg->time_increment = msg->scan_time / 360.0f;

    /* 盲区 0.12m, 最大量程 8m */
    msg->range_min = 0.12f;
    msg->range_max = 8.0f;

    /* ranges: 120 个 float (3° 分辨率, 单 UDP 包可完整传输) */
    msg->ranges.data = (float *)malloc(360 * sizeof(float));
    msg->ranges.size = 360;
    msg->ranges.capacity = 360;

    /* X3 Pro 不支持强度信息, intensities 留空 */
    msg->intensities.data = NULL;
    msg->intensities.size = 0;
    msg->intensities.capacity = 0;
}

/* ================================================================
 * scan_timer_callback — 定时发布 /scan 话题
 * ================================================================ */

/**
 * @brief  10Hz 定时器回调 — 读雷达 → 填充 LaserScan → 发布
 *
 * @details
 *   - 使用 ydlidar_x3pro_get_scan() 非阻塞获取最新一圈数据
 *   - 仅当有新圈数据 (时间戳变化) 时才发布, 避免重复发送旧帧
 *   - 未填充的方向置为 range_max+1, 表示无效
 */
static void scan_timer_callback(rcl_timer_t *timer,
                                 int64_t last_call_time)
{
    (void)timer;
    (void)last_call_time;

    const ydlidar_x3pro_scan_t *scan = ydlidar_x3pro_get_scan();
    if (!scan) {
        return;  /* 雷达尚未产出完整一圈 */
    }

    /* 去重: 与上一帧时间戳相同则不发布 */
    if (scan->timestamp_us == g_scan_last_stamp_us) {
        return;
    }
    g_scan_last_stamp_us = scan->timestamp_us;

    /* 时间戳：取「这一圈扫描的中心时刻」，不是发布时刻。
     * 一圈约 167ms，用发布时刻会让点云与 TF 差半圈，
     * 转弯时尤其明显，SLAM 会匹配错。 */
    int64_t now_us      = esp_timer_get_time();
    int64_t now_ms      = rmw_uros_epoch_millis();
    int64_t age_ms      = (now_us - scan->timestamp_us) / 1000;
    if (age_ms < 0)   age_ms = 0;
    if (age_ms > 500) age_ms = 500;
    int64_t stamp_ms    = now_ms - age_ms - 83;   /* 减半圈(83ms)居中 */
    if (stamp_ms < 0) stamp_ms = 0;
    g_scan_msg.header.stamp.sec     = (int32_t)(stamp_ms / 1000);
    g_scan_msg.header.stamp.nanosec = (uint32_t)((stamp_ms % 1000) * 1000000);

    /* 先把 120 个方向全部标为无效
     * 3° 分辨率降采样: lidar 360×1° → ROS 120×3°, 取每 3° 的第一个值 */
    const float invalid_range = g_scan_msg.range_max + 1.0f;
    for (int i = 0; i < 360; i++) {
        g_scan_msg.ranges.data[i] = invalid_range;
    }

    for (int i = 0; i < 360; i++) {
        /* 实测校验（盒子放右前方，扫描显示在左侧）：
         * 本车雷达的原始角度方向已与 ROS 一致，直接按索引取，不做镜像。
         * 之前做的 (360 - i) 反而造成左右翻转。 */
        int lidar_idx = (180 - i + 360) % 360;   /* 实测：右侧盒子在 -88°，方向正确 */
        if (scan->ranges_filled[lidar_idx] &&
            scan->ranges[lidar_idx] >= g_scan_msg.range_min &&
            scan->ranges[lidar_idx] <= g_scan_msg.range_max) {
            g_scan_msg.ranges.data[i] = scan->ranges[lidar_idx];
        }
    }

    RCSOFTCHECK(rcl_publish(&g_scan_publisher, &g_scan_msg, NULL));

    /* 调试输出: 每 30 帧 (约 3s) 打印一次有效方向数 */
    static int scan_cnt = 0;
    if (++scan_cnt % 30 == 0) {
        int valid = 0;
        for (int i = 0; i < 360; i++) {
            if (g_scan_msg.ranges.data[i] < invalid_range) valid++;
        }
        debug_log_printf("[ROS] /scan published: stamp=%lldus  valid=%d/360  points=%d\n",
                         (long long)scan->timestamp_us, valid, scan->point_count);
    }
}

/* ================================================================
 * microros_bridge_publish_debug — 发布 /debug 话题
 * ================================================================ */

/**
 * @brief 从日志缓冲区读取字符串并发布到 /debug
 *
 * @details
 * 每次最多发布 3 行日志, 防止单次日志爆发导致网络拥塞.
 * 由 microros_bridge_spin() 按 5Hz 节拍调用, 不占用 rclc 定时器资源.
 * 若缓冲区为空则跳过发布.
 */
static void microros_bridge_publish_debug(void)
{
    /* 一次最多发布 3 行, 避免拥塞 */
    for (int i = 0; i < 3; i++) {
        if (!debug_log_read_line(g_debug_line, sizeof(g_debug_line))) {
            break;
        }

        g_debug_msg.data.data   = g_debug_line;
        g_debug_msg.data.size   = strlen(g_debug_line);
        g_debug_msg.data.capacity = sizeof(g_debug_line);

        RCSOFTCHECK(rcl_publish(&g_debug_publisher, &g_debug_msg, NULL));
    }
}

/* ================================================================
 * microros_bridge_init — micro-ROS桥初始化 18步骨架
 * 注意：代码执行顺序必须严格1~18，不可调换；
 * ================================================================ */
void microros_bridge_init(void)
{
    // ======================【A ROS底层环境准备】======================
    // Step 1: 获取默认内存分配器
    g_allocator = rcl_get_default_allocator();

    // Step 2: 初始化ROS基础选项
    rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
    RCCHECK(rcl_init_options_init(&init_options, g_allocator));

#ifdef CONFIG_MICRO_ROS_ESP_XRCE_DDS_MIDDLEWARE
    // 配置XRCE-DDS，填写Agent的IP和端口
    rmw_init_options_t *rmw_opts = rcl_init_options_get_rmw_init_options(&init_options);
    RCCHECK(rmw_uros_options_set_udp_address(
        CONFIG_MICRO_ROS_AGENT_IP,
        CONFIG_MICRO_ROS_AGENT_PORT,
        rmw_opts));

    // Step 2.5: Ping Agent检测连接，连不上直接退出初始化
    if (!rmw_uros_ping_agent(1000, 1)) {
        printf("[ROS] Agent ping failed, skipping init\n");
        debug_log_printf("[ROS] Agent ping failed, skipping init\n");
        RCIGNORE(rcl_init_options_fini(&init_options));
        return;
    }
#endif

    // ======================【B ROS节点基础创建】======================
    // Step 3: 初始化rclc支持层
    RCCHECK(rclc_support_init_with_options(
        &g_support, 0, NULL, &init_options, &g_allocator));

    // 与 agent 校时：不调用的话 header.stamp 是开机时长（几十秒），
    // TF 与 slam_toolbox 会报时间戳过旧，建图直接失败
    if (!rmw_uros_sync_session(1000)) {
        printf("[ROS] time sync failed, stamps will be uptime\n");
        debug_log_printf("[ROS] time sync failed\n");
        for (int i = 0; i < 3; i++) {
            vTaskDelay(pdMS_TO_TICKS(500));
            if (rmw_uros_sync_session(1000)) {
                printf("[ROS] time sync ok (retry %d)\n", i + 1);
                break;
            }
        }
    }
    printf("[ROS] synced time ms=%lld\n", (long long)rmw_uros_epoch_millis());

    // Step 4: 创建ROS节点
    g_node = rcl_get_zero_initialized_node();
    RCCHECK(rclc_node_init_default(
        &g_node, MICROROS_NODE_NAME, "", &g_support));

    // ======================【C 话题对象创建】======================
    // Step 5: 订阅 /beep 话题 std_msgs/UInt16，控制蜂鸣器
    RCCHECK(rclc_subscription_init_default(
        &g_beep_subscriber, &g_node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt16),
        MICROROS_BEEP_TOPIC));

    // Step 5.1: 订阅 /speak 话题 std_msgs/String，转发给语音模块
    RCCHECK(rclc_subscription_init_default(
        &g_speak_subscriber, &g_node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
        MICROROS_SPEAK_TOPIC));
    g_speak_msg.data.data     = g_speak_buf;
    g_speak_msg.data.size     = 0;
    g_speak_msg.data.capacity = sizeof(g_speak_buf);

    // Step 6: 发布 /battery 话题 std_msgs/UInt16，电池电压
    RCCHECK(rclc_publisher_init_default(
        &g_battery_publisher, &g_node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt16),
        MICROROS_BATTERY_TOPIC));

    // Step 7: 发布 /imu 话题 sensor_msgs/Imu，IMU姿态数据
    RCCHECK(rclc_publisher_init_best_effort(
        &g_imu_publisher, &g_node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
        MICROROS_IMU_TOPIC));
    imu_msg_init_covariance(&g_imu_msg);

    // Step 8: 发布 /odom 话题 nav_msgs/Odometry，里程计
    RCCHECK(rclc_publisher_init_default(
        &g_odom_publisher, &g_node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
        MICROROS_ODOM_TOPIC));

    // Step 9: 发布 /scan 话题 sensor_msgs/LaserScan，激光雷达
    RCCHECK(rclc_publisher_init_default(
        &g_scan_publisher, &g_node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, LaserScan),
        MICROROS_SCAN_TOPIC));
    scan_msg_init(&g_scan_msg);

    // Step10: 发布 /debug 话题 std_msgs/String，调试日志
    // 说明：不单独开定时器，在spin循环里5Hz轮询发布，节省rclc定时器资源
    RCCHECK(rclc_publisher_init_default(
        &g_debug_publisher, &g_node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
        MICROROS_DEBUG_TOPIC));
    g_debug_msg.data.data = g_debug_line;
    g_debug_msg.data.size = 0;
    g_debug_msg.data.capacity = sizeof(g_debug_line);

    // Step11: 订阅 /cmd_vel 话题 geometry_msgs/Twist，速度指令
    RCCHECK(rclc_subscription_init_default(
        &g_cmd_vel_subscriber, &g_node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        MICROROS_CMD_VEL_TOPIC));

    // Step11b: 订阅 /cmd_ptz_angle 话题 geometry_msgs/Vector3，云台角度遥控
    //           (x=pan, y=tilt, 单位度; 让上位机键盘可直接控云台)
    RCCHECK(rclc_subscription_init_default(
        &g_cmd_ptz_subscriber, &g_node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Vector3),
        MICROROS_CMD_PTZ_TOPIC));

    // ======================【D 外设+定时器创建】======================
    // Step12: 初始化YDLIDAR X3PRO雷达硬件（串口+PWM）并启动
    ydlidar_x3pro_init(X3PRO_UART_RX, X3PRO_MCTR_GPIO);
    ydlidar_x3pro_start();

    // Step13: 初始化编码器硬件、里程计状态
    bool enc_ok = odom_hw_init();
    // 编码器绑定到速度PID闭环，必须在编码器初始化完成后执行
    if (enc_ok) {
        encoder_handle_t enc_l, enc_r;
        odom_get_encoder_handles(&enc_l, &enc_r);
        cmd_vel_set_encoders(enc_l, enc_r);
    }

    // Step14: 电池采样定时器
    g_timer = rcl_get_zero_initialized_timer();
    RCCHECK(rclc_timer_init_default(
        &g_timer, &g_support,
        RCL_MS_TO_NS(MICROROS_BATTERY_INTERVAL_MS),
        battery_timer_callback));

    // Step15: IMU定时器 25Hz
    g_imu_timer = rcl_get_zero_initialized_timer();
    RCCHECK(rclc_timer_init_default(
        &g_imu_timer, &g_support,
        RCL_MS_TO_NS(MICROROS_IMU_INTERVAL_MS),
        imu_timer_callback));

    // Step16: 里程计定时器 20Hz
    g_odom_timer = rcl_get_zero_initialized_timer();
    RCCHECK(rclc_timer_init_default(
        &g_odom_timer, &g_support,
        RCL_MS_TO_NS(MICROROS_ODOM_INTERVAL_MS),
        odom_timer_callback));

    // Step17: 激光雷达定时器 10Hz
    g_scan_timer = rcl_get_zero_initialized_timer();
    RCCHECK(rclc_timer_init_default(
        &g_scan_timer, &g_support,
        RCL_MS_TO_NS(MICROROS_SCAN_INTERVAL_MS),
        scan_timer_callback));

    // ======================【E 执行器与收尾】======================
    // Step18: 创建执行器，注册订阅者与定时器任务
    RCCHECK(rclc_executor_init(
        &g_executor, &g_support.context,
        MICROROS_EXECUTOR_HANDLES, &g_allocator));

    // 订阅任务挂载到执行器，ON_NEW_DATA：收到消息才触发回调
    RCCHECK(rclc_executor_add_subscription(
        &g_executor, &g_beep_subscriber, &g_beep_msg,
        &beep_callback, ON_NEW_DATA));
    RCCHECK(rclc_executor_add_subscription(
        &g_executor, &g_speak_subscriber, &g_speak_msg,
        &speak_callback, ON_NEW_DATA));
    RCCHECK(rclc_executor_add_subscription(
        &g_executor, &g_cmd_vel_subscriber, &g_cmd_vel_msg,
        &cmd_vel_callback, ON_NEW_DATA));
    RCCHECK(rclc_executor_add_subscription(
        &g_executor, &g_cmd_ptz_subscriber, &g_cmd_ptz_msg,
        &cmd_ptz_callback, ON_NEW_DATA));

    // 定时任务挂载到执行器
    RCCHECK(rclc_executor_add_timer(&g_executor, &g_timer));
    RCCHECK(rclc_executor_add_timer(&g_executor, &g_imu_timer));
    RCCHECK(rclc_executor_add_timer(&g_executor, &g_odom_timer));
    RCCHECK(rclc_executor_add_timer(&g_executor, &g_scan_timer));

    // IMU硬件初始化 + Madgwick姿态滤波器
    g_imu_ready = imu_init();
    if (g_imu_ready) {
        // Madgwick增益0.05，加速度/陀螺仪使用EMA平滑
        imu_filter_init(0.05f,
                        IMU_EMA_DEFAULT_ACCEL_ALPHA,
                        IMU_EMA_DEFAULT_GYRO_ALPHA);
    }

    // 打印初始化结果日志
    debug_log_printf("[ROS] microros_bridge ready\n");
    debug_log_printf("[ROS]   node     : %s\n", MICROROS_NODE_NAME);
    debug_log_printf("[ROS]   subscribe: %s (std_msgs/UInt16)\n", MICROROS_BEEP_TOPIC);
    debug_log_printf("[ROS]   subscribe: %s (geometry_msgs/Twist)\n", MICROROS_CMD_VEL_TOPIC);
    debug_log_printf("[ROS]   subscribe: %s (geometry_msgs/Vector3)\n", MICROROS_CMD_PTZ_TOPIC);
    debug_log_printf("[ROS]   publish  : %s (std_msgs/UInt16, %d ms)\n",
                     MICROROS_BATTERY_TOPIC, MICROROS_BATTERY_INTERVAL_MS);
    debug_log_printf("[ROS]   publish  : %s (sensor_msgs/Imu, %d ms)  IMU:%s\n",
                     MICROROS_IMU_TOPIC, MICROROS_IMU_INTERVAL_MS,
                     g_imu_ready ? "OK" : "FAIL");
    debug_log_printf("[ROS]   publish  : %s (nav_msgs/Odometry, %d ms)  ENC:%s\n",
                     MICROROS_ODOM_TOPIC, MICROROS_ODOM_INTERVAL_MS,
                     enc_ok ? "OK" : "FAIL");
    debug_log_printf("[ROS]   publish  : %s (sensor_msgs/LaserScan, %d ms)\n",
                     MICROROS_SCAN_TOPIC, MICROROS_SCAN_INTERVAL_MS);
    debug_log_printf("[ROS]   publish  : %s (std_msgs/String, %d ms)\n",
                     MICROROS_DEBUG_TOPIC, MICROROS_DEBUG_INTERVAL_MS);

    g_agent_connected = true;
    g_init_done = true;
}

/* ================================================================
 * microros_bridge_spin — 驱动执行器
 * ================================================================ */

void microros_bridge_spin(void)
{
    /* init 未完成 (Agent 不通跳过了) → 直接返回, 防止空 executor 崩溃 */
    if (!g_init_done) {
        return;
    }

    rcl_ret_t rc = rclc_executor_spin_some(
        &g_executor, RCL_MS_TO_NS(MICROROS_SPIN_TIMEOUT_MS));

    /* 断线检测: 连续 3 次非超时错误 → 标记 Agent 断开
     * (RCL_RET_TIMEOUT 是正常的空闲状态, 不算错误) */
    static int g_spin_error_count = 0;
    if (rc != RCL_RET_OK && rc != RCL_RET_TIMEOUT) {
        g_spin_error_count++;
        if (g_spin_error_count >= 3) {
            g_agent_connected = false;
            debug_log_printf("[ROS] Agent disconnected (spin error: %d)\n", (int)rc);
        }
    } else {
        g_spin_error_count = 0;
    }

    /* 5Hz 发布 /debug (不用定时器, 节省 rclc timer 资源) */
    static int64_t g_last_debug_pub_us = 0;
    int64_t now_us = esp_timer_get_time();
    if ((now_us - g_last_debug_pub_us) >=
            (int64_t)MICROROS_DEBUG_INTERVAL_MS * 1000LL) {
        g_last_debug_pub_us = now_us;
        microros_bridge_publish_debug();
    }
}

bool microros_bridge_is_agent_connected(void)
{
    return g_agent_connected;
}
