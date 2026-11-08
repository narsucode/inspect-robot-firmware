/**
 * @file    main.c
 * @brief   micro-ROS 智能小车入口程序
 * @author  智能小车项目
 * @version CAR_FIRMWARE_VERSION
 *
 * 系统概述
 * --------------------------
 * 硬件：ESP32-S3
 * 系统：FreeRTOS + micro-ROS
 * 功能：键盘遥控 + 轮速PID闭环 + IMU + 里程计 + 电池监测 + OLED状态显示 + 视觉云台
 *
 * 任务核分配：
 * - Core0：本周期主任务（micro-ROS通信、速度PID、蜂鸣器、OLED刷新）+ WiFi协议栈
 * - Core1：激光雷达解析任务（雷达驱动内部创建，不受主循环抖动干扰）
 *
 * 任务划分
 * --------------------------
 * 1. periodic_loop 主周期任务(10ms周期，Core0)
 *    负责：micro-ROS消息订阅/定时发布、蜂鸣器状态机、小车速度PID控制、电池状态检测、OLED屏幕刷新
 * 2. vision_task 视觉云台独立任务(内部20Hz闭环)
 *    负责：视觉坐标读取 + 舵机云台PD控制；模块缺失不会让整车瘫痪，云台自动回中位待机
 * 3. 激光雷达任务：ydlidar_x3pro_init内部创建，绑定Core1
 *
 * ROS2 话题列表
 * --------------------------
 * 订阅话题：
 *   /beep        std_msgs/UInt16      蜂鸣器控制
 *   /cmd_vel     geometry_msgs/Twist  小车速度指令
 *
 * 发布话题：
 *   /battery     std_msgs/UInt16      电池电压，0.2Hz
 *   /imu         sensor_msgs/Imu       IMU姿态，25Hz
 *   /odom        nav_msgs/Odometry     里程计，20Hz
 *   /scan        sensor_msgs/LaserScan 激光雷达数据，10Hz
 *   /debug       std_msgs/String       调试日志，5Hz
 *
 * 硬件配置
 * --------------------------
 * 轮径、轮距、PID参数、外设引脚统一放在 bsp/car_config.h
 * OLED刷新率自适应：收到cmd指令快速刷新(50ms)；空闲降速(500ms)节省资源
 */

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_timer.h"

#include "car_config.h"
#include "beep.h"
#include "battery.h"
#include "debug_log.h"
#include "cmd_vel_control.h"
#include "oled.h"
#include "led.h"
#include "display_status.h"
#include "voice.h"
#include "sys_monitor.h"
#include "vision_task.h"
#include "microros_bridge.h"

#if defined(CONFIG_MICRO_ROS_ESP_NETIF_WLAN) || \
    defined(CONFIG_MICRO_ROS_ESP_NETIF_ENET)
#include <uros_network_interfaces.h>
#endif

/* ===================== 可配置宏定义 ===================== */
/**
 * @brief 周期任务名称
 */
#define PERIODIC_TASK_NAME          "periodic_loop"

/**
 * @brief 主周期任务执行间隔(单位ms)
 * @note 同时是蜂鸣器状态机tick、cmd_vel速度控制的PID计算周期
 */
#define PERIODIC_TASK_INTERVAL_MS   10

/* ===================== 日志标签 ===================== */
static const char *TAG = "main";

/* ===================== 主周期任务 periodic_loop ===================== */
/**
 * @brief 主周期任务入口函数，死循环永不返回
 * @param arg 任务入参，未使用
 *
 * 每10ms执行一轮，执行顺序：
 * 1. microros_bridge_spin()
 *    - 处理订阅消息：/beep 蜂鸣指令、/cmd_vel 小车速度指令
 *    - 执行定时器回调：定时采样并发布电池、IMU、里程计、雷达、debug话题
 * 2. beep_handle() 蜂鸣器状态机，处理延时关闭逻辑
 * 3. cmd_vel_tick() 速度控制逻辑：指令看门狗、目标速度ramp缓变、PID闭环、输出电机PWM
 *
 * 上电阶段：
 * 调用microros_bridge_init初始化micro-ROS桥，同时内部初始化编码器、里程计、IMU
 * 执行sys_monitor_boot_check整机自检，自检结果用蜂鸣器提示，日志打印自检信息
 *
 * 分支逻辑：
 * - 每100个tick(1s)：读取电池状态，根据电池等级控制LED闪烁模式
 * - OLED刷新：收到新指令立即刷新；指令刚下发2s内50ms刷新一次；空闲状态500ms刷新一次
 */
static void periodic_loop(void *arg)
{
    (void)arg;

    ESP_LOGI(TAG, "正在初始化 micro-ROS 桥接组件...");
    microros_bridge_init();
    ESP_LOGI(TAG, "准备完成。");

    /* 上电整机自检：WiFi/Agent/雷达/IMU/电池，自检结果蜂鸣器提示 */
    sys_error_t boot_err = sys_monitor_boot_check();
    debug_log_printf("[MAIN] 上电自检: %s\n",
                     sys_error_string(boot_err));

    uint32_t tick = 0;
    while (1) {
        microros_bridge_spin();                        /* micro-ROS消息处理 */
        beep_handle();                                 /* 蜂鸣器状态机tick */
        cmd_vel_tick();                                /* 速度闭环：看门狗、ramp、PID、PWM输出 */

        /* 每1秒(100 tick)执行电池采样滤波 + 电池告警，控制LED指示灯 */
        if (tick % 100 == 0) {
            battery_state_t bat = sys_monitor_battery_tick();

            /* 语音播报: 只在状态"跳变"时播, 不跳变就闭嘴 */
            {
                static bool s_agent_prev = false;
                bool agent_now = microros_bridge_is_agent_connected();
                if (agent_now != s_agent_prev) {
                    voice_say(agent_now ? VOICE_EVT_AGENT_OK : VOICE_EVT_AGENT_LOST);
                    s_agent_prev = agent_now;
                }
            }
            {
                static bool s_bat_crit_prev = false;
                bool bat_crit = (bat == BAT_CRITICAL);
                if (bat_crit && !s_bat_crit_prev) {
                    voice_say(VOICE_EVT_LOW_BAT);
                }
                s_bat_crit_prev = bat_crit;
            }

            /* LED指示灯规则
             * BAT_CRITICAL 严重低电量：快闪，5个tick周期
             * BAT_WARNING  低电量告警：慢闪，50个tick周期
             * BAT_NORMAL   正常：Agent在线LED常亮；Agent离线LED慢闪
             */
            if (bat == BAT_CRITICAL) {
                led_flash(5);
            } else if (bat == BAT_WARNING) {
                led_flash(50);
            } else {
                if (microros_bridge_is_agent_connected()) {
                    led_on();
                } else {
                    led_flash(50);
                }
            }
        }

        /* OLED屏幕刷新逻辑
         * 触发条件：收到新cmd_vel指令标记 / 快速窗口内5tick刷新 / 空闲50tick刷新
         *
         * 重要：oled.c 是"一个字节一次 I2C 事务"，100kHz 下整屏重画要几十~上百毫秒。
         * 若每条 cmd_vel 都触发重画，10Hz 遥控就会把 10ms 主循环堵死，表现为"按键后延迟"。
         * 所以这里加最小间隔 OLED_MIN_REFRESH_US，保证控制循环优先。
         */
        {
            static int64_t s_last_oled_us = 0;
            int64_t now_us = esp_timer_get_time();
            bool want = display_status_consume_request() ||
                        (display_status_in_fast_window() && tick % 5 == 0) ||
                        tick % 50 == 0;
            /* 遥控/运动期间不刷新：此时屏幕数值每帧都在变，整屏重画要几十~上百毫秒，
             * 会把 10ms 控制循环和 /odom 发布一起堵住（实测运动时 /odom 掉到 640ms）。
             * 改为"空闲后再刷"，画面延后一点，但响应优先。 */
            /* 底盘遥控和云台遥控都算"运动中"：两者期间刷屏都会阻塞主循环 */
            bool idle = !display_status_in_fast_window() &&
                        !microros_bridge_is_ptz_active();
            if (want && idle && (now_us - s_last_oled_us) >= OLED_MIN_REFRESH_US) {
                display_status_update();
                s_last_oled_us = now_us;
            }
        }

        tick++;
        vTaskDelay(pdMS_TO_TICKS(PERIODIC_TASK_INTERVAL_MS));
    }
}

/* ===================== ESP-IDF 程序入口 app_main ===================== */
/**
 * @brief ESP-IDF 应用入口函数
 *
 * 硬件&驱动初始化顺序：
 * 1. debug_log_init     初始化调试日志缓存，必须最先执行
 * 2. battery_init       电池ADC采样初始化
 * 3. oled_init          OLED屏幕初始化，绘制上电首屏
 * 4. beep_init          蜂鸣器硬件初始化
 * 5. led_init           LED指示灯初始化
 * 6. cmd_vel_init       电机驱动、速度控制模块初始化
 * 7. uros_network_interface_initialize micro-ROS WiFi网络初始化
 *    等待WiFi连接，最长等待10s，连接成功标记状态并刷新OLED
 * 8. vision_task_start  启动独立视觉云台任务，云台模块异常不影响小车基础功能
 * 9. xTaskCreatePinnedToCore 创建主周期任务，绑定到Core0
 *
 * 配置说明：
 * WiFi、micro-ROS Agent参数、任务栈大小、任务优先级，全部在idf.py menuconfig配置
 */
void app_main(void)
{
    /* 1. 初始化调试日志缓冲 (必须在任何 debug_log_printf 之前调用) */
    debug_log_init();
    debug_log_printf("[MAIN] 启动: %s %s\n", CAR_HARDWARE_NAME, CAR_FIRMWARE_VERSION);

    /* 2. 初始化电池 ADC (OLED 显示需要电池数据) */
    battery_init();
    ESP_LOGI(TAG, "电池ADC就绪 (引脚%d)", BATTERY_GPIO);

    /* 3. 初始化 OLED 并显示首屏 */
    oled_init();
    oled_clear();
    display_status_update();
    ESP_LOGI(TAG, "OLED屏幕就绪");

    /* 4. 初始化蜂鸣器硬件 */
    beep_init();
    ESP_LOGI(TAG, "蜂鸣器驱动就绪 (引脚%d)", BEEP_GPIO);

    /* 4.5 初始化语音播报模块 (UART2: TX=GPIO40, RX=GPIO41)
     * 只发出事件码, 具体念什么由天问模块配置决定, 改话术不用重烧固件 */
    voice_init();

    /* 5. 初始化 LED */
    led_init();
    led_off();
    ESP_LOGI(TAG, "LED指示灯驱动就绪");

    /* 6. 初始化电机 + 速度控制 */
    cmd_vel_init(PIN_MOTOR_L_A, PIN_MOTOR_L_B,
                 PIN_MOTOR_R_A, PIN_MOTOR_R_B);
    ESP_LOGI(TAG, "电机与速度控制模块就绪");

    /* 7. 初始化 micro-ROS 网络 */
#if defined(CONFIG_MICRO_ROS_ESP_NETIF_WLAN) || \
    defined(CONFIG_MICRO_ROS_ESP_NETIF_ENET)
    ESP_ERROR_CHECK(uros_network_interface_initialize());
    ESP_LOGI(TAG, "网络(WiFi)初始化完成");
    esp_wifi_set_ps(WIFI_PS_NONE);

    /* 等待WiFi连接确认，最长等待10s，连接成功标记状态并刷新OLED */
    {
        uint32_t wifi_wait_ms = 0;
        while (wifi_wait_ms < 10000) {
            wifi_ap_record_t ap_info = {0};
            if (esp_wifi_sta_get_ap_info(&ap_info) == ESP_OK) {
                ESP_LOGI(TAG, "WiFi已连接到 %s", ap_info.ssid);
                wifi_status_set_connected(true);
                display_status_update();  /* 立即刷新OLED显示 W:+ */
                voice_say(VOICE_EVT_WIFI_OK);   /* 语音: WiFi 已连接 */
                break;
            }
            vTaskDelay(pdMS_TO_TICKS(500));
            wifi_wait_ms += 500;
        }
        if (wifi_wait_ms >= 10000) {
            ESP_LOGW(TAG, "WiFi连接超时");
            voice_say(VOICE_EVT_WIFI_FAIL);   /* 语音: WiFi 连接失败 */
        }
    }
#endif

    /* 8. 启动视觉云台独立任务
     * 独立任务20Hz闭环，读取视觉坐标控制舵机云台
     * 视觉模块接线异常不会导致整车无法运行，云台停在中位待机
     */
    vision_task_start();

    /* 9. 创建主周期任务，固定运行在Core0
     * Core0：WiFi、micro-ROS、主控制循环
     * Core1：激光雷达解析任务，雷达独占核心，减少控制抖动
     */
    xTaskCreatePinnedToCore(periodic_loop,                  // 任务入口函数：主周期循环函数
                            PERIODIC_TASK_NAME,             // 任务名称：仅调试日志识别使用
                            CONFIG_MICRO_ROS_APP_STACK,     // 任务栈大小，单位word(1word=4字节)，menuconfig配置
                            NULL,                           // 传入任务函数的参数，此处无参数传入
                            CONFIG_MICRO_ROS_APP_TASK_PRIO, // 任务优先级，数值越大优先级越高，menuconfig配置
                            NULL,                           // 任务句柄输出地址，不需要操作任务故填NULL
                            0);  /* Core 0 */               // 绑定CPU核心编号，固定运行在Core0



    ESP_LOGI(TAG, "周期任务 [%s] 绑定至核心0，周期=%d毫秒。",
             PERIODIC_TASK_NAME, PERIODIC_TASK_INTERVAL_MS);
}