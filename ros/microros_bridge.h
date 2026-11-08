/**
 * @file    microros_bridge.h
 * @brief   micro-ROS 通信桥 — 订阅 /beep /cmd_vel + 发布 /battery /imu /odom /scan /debug
 *
 * @details
 * 三步拆解术:
 *   1. 直观理解: 这个文件就像一个双向门童——
 *      PC 发来的 /beep、/cmd_vel 命令转给蜂鸣器和电机（订阅），
 *      ESP32 的电池电压、IMU 姿态、里程计定时广播给 PC（发布）。
 *   2. 追根溯源: 没有统一桥接层，订阅和发布分散在各处，
 *      初始化顺序、错误处理、宏配置各管各的，维护噩梦。
 *   3. 正规解释: 基于 micro-ROS rclc，一个节点同时拥有
 *      两个订阅者（/beep, /cmd_vel）和五个发布者（/battery, /imu, /odom, /scan, /debug），
 *      外加四个定时器驱动发布节奏，全部由一个执行器统一调度。
 *
 * 对应 ROS2 话题:
 *   订阅 /beep     (std_msgs/UInt16)      — 控制蜂鸣器
 *   订阅 /cmd_vel  (geometry_msgs/Twist)  — 速度控制
 *   订阅 /cmd_ptz_angle (geometry_msgs/Vector3) — 云台角度遥控 (x=pan, y=tilt, 度)
 *   发布 /battery  (std_msgs/UInt16)      — 0.2Hz 电池电压
 *   发布 /imu      (sensor_msgs/Imu)      — 25Hz IMU 姿态数据
 *   发布 /odom     (nav_msgs/Odometry)    — 20Hz 里程计
 *   发布 /scan     (sensor_msgs/LaserScan) — 10Hz 激光雷达
 *   发布 /debug    (std_msgs/String)      — 5Hz 调试日志
 *
 * @usage
 * @code
 *   beep_init(); battery_init();
 *   microros_bridge_init();
 *   while (1) {
 *       microros_bridge_spin();  // 驱动执行器
 *       beep_handle();           // 蜂鸣器状态机
 *       cmd_vel_tick();          // 速度控制
 *       vTaskDelay(10);
 *   }
 * @endcode
 */

#pragma once
#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

/* ================================================================
 * 可配置宏 — 一句话定位所有话题/频率/资源
 * (小车硬件参数在 bsp/car_config.h, 本文件只管 ROS 通信)
 * ================================================================ */

/* ── 节点身份 ── */

/** @brief ROS2 节点名称 */
#define MICROROS_NODE_NAME              "esp32_car_sensors"

/* ── 订阅话题 ── */

/** @brief 蜂鸣器控制话题 (本节点订阅)
 *  @note  协议: data=0 关, data=1 常响, data>=10 响 N ms 后自动关 */
#define MICROROS_BEEP_TOPIC             "/beep"

/** @brief 速度控制话题 (本节点订阅)
 *  @note  类型: geometry_msgs/Twist, 协议同 ROS2 标准
 *          linear.x  = 线速度 (m/s), 正=前进
 *          angular.z = 角速度 (rad/s), 正=逆时针(左转) */
#define MICROROS_CMD_VEL_TOPIC          "/cmd_vel"

/** @brief 云台角度遥控话题 (本节点订阅)
 *  @note  类型: geometry_msgs/Vector3
 *          x = 水平(pan) 角度(度), y = 俯仰(tilt) 角度(度), z 未用
 *          直接喂给 vision_gimbal_set_angle(), 并自动进入遥控模式 */
#define MICROROS_CMD_PTZ_TOPIC          "/cmd_ptz_angle"

/** @brief 语音播报话题 (本节点订阅)
 *  @note  类型: std_msgs/String
 *          收到字符串 → 原样转发给语音模块 (UART2), 由天问模块匹配后播报
 *          例: ros2 topic pub --once /speak std_msgs/String "{data: 'PATROL_DONE'}"
 *          约定的事件码见 bsp/voice.h (WIFI_OK / AGENT_OK / PATROL_DONE / ALARM ...) */
#define MICROROS_SPEAK_TOPIC            "/speak"

/* ── 发布话题 ── */

/** @brief 电池电压话题 (本节点发布)
 *  @note  协议: data = 电压(V) × 10, 例如 12.3V → data=123 */
#define MICROROS_BATTERY_TOPIC          "/battery"

/** @brief IMU 话题 (本节点发布)
 *  @note  类型: sensor_msgs/Imu, SI 单位, 含 Madgwick 姿态四元数 */
#define MICROROS_IMU_TOPIC              "/imu"

/** @brief IMU 坐标系 (frame_id) */
#define MICROROS_IMU_FRAME_ID           "imu_link"

/** @brief 里程计话题 (本节点发布)
 *  @note  类型: nav_msgs/Odometry, 差速驱动模型 */
#define MICROROS_ODOM_TOPIC             "/odom"

/** @brief 里程计父坐标系 (odom frame) */
#define MICROROS_ODOM_FRAME_ID          "odom"

/** @brief 里程计子坐标系 (base_link) */
#define MICROROS_ODOM_CHILD_FRAME_ID    "base_link"

/** @brief 激光雷达话题 (本节点发布)
 *  @note  类型: sensor_msgs/LaserScan, 360° 扫描数据 */
#define MICROROS_SCAN_TOPIC             "/scan"

/** @brief 激光雷达坐标系 (frame_id) */
#define MICROROS_SCAN_FRAME_ID          "laser"

/** @brief 调试日志话题 (本节点发布)
 *  @note  类型: std_msgs/String, 收集 cmd_vel/odom/雷达等调试输出 */
#define MICROROS_DEBUG_TOPIC            "/debug"

/* ── 发布频率 ── */

/** @brief 电池电压发布间隔 (ms)
 *  @note  5000ms = 0.2Hz, 电压变化慢, 0.2Hz 足够 */
#define MICROROS_BATTERY_INTERVAL_MS    5000

/** @brief IMU 发布间隔 (ms)
 *  @note  40ms = 25Hz, 兼顾姿态跟踪与网络负载 */
#define MICROROS_IMU_INTERVAL_MS        40

/** @brief 里程计发布间隔 (ms)
 *  @note  50ms = 20Hz, 差速轮里程计不需要过高频率 */
#define MICROROS_ODOM_INTERVAL_MS       50

/** @brief 激光雷达发布间隔 (ms)
 *  @note  100ms = 10Hz, 雷达扫描频率约 6Hz, 10Hz 足够跟新 */
#define MICROROS_SCAN_INTERVAL_MS       100

/** @brief 调试日志发布间隔 (ms)
 *  @note  200ms = 5Hz, 既不会拥塞网络, 又能及时输出调试信息 */
#define MICROROS_DEBUG_INTERVAL_MS      200

/* ── 执行器配置 ── */

/** @brief 执行器 spin 超时 (ms) */
#define MICROROS_SPIN_TIMEOUT_MS        1

/** @brief 执行器句柄数 (3 订阅者 + 4 定时器 = 7, 加余量取 12)
 *  @note  添加新话题时增大此值. /debug 不用定时器, 省一个 rclc timer 资源 */
#define MICROROS_EXECUTOR_HANDLES       12

/* ================================================================
 * 公开 API
 * ================================================================ */

/**
 * @brief  初始化 micro-ROS 通信桥 (订阅者 + 发布者 + 定时器)
 *
 * @details
 * 执行流程 (共 17 步):
 *   1. 获取默认内存分配器
 *   2. 创建初始化选项 (含 Agent IP/端口, 来自 menuconfig)
 *   3. 初始化 micro-ROS 支持层
 *   4. 创建节点 (名称: MICROROS_NODE_NAME)
 *   5. 创建 /beep 订阅者 (std_msgs/UInt16)
 *   6. 创建 /battery 发布者 (std_msgs/UInt16)
 *   7. 创建 /imu 发布者 (sensor_msgs/Imu)
 *   8. 创建 /odom 发布者 (nav_msgs/Odometry)
 *   9. 创建 /cmd_vel 订阅者 (geometry_msgs/Twist)
 *  10. 初始化里程计硬件, 绑定编码器到 PID 闭环
 *  11~13. 创建电池/IMU/里程计三个定时器
 *  14. 创建执行器并挂载全部订阅者与定时器
 *
 * @note   必须在 WiFi 连接成功、beep_init()、battery_init() 之后调用
 * @note   内部使用 RCCHECK 宏，失败时 vTaskDelete(NULL)
 * @warning 本函数不是线程安全的，应在任务入口一次性调用
 */
void microros_bridge_init(void);

/**
 * @brief  轮询 micro-ROS 执行器（非阻塞）
 *
 * @details
 * 驱动内部调度器:
 *   - 有新 /beep 消息 → 触发 beep_callback → beep_on_time()
 *   - 有新 /cmd_vel 消息 → 触发 cmd_vel_callback → cmd_vel_set_command()
 *   - 定时器到期 → 触发对应发布回调 → 采样+发布
 *   - 无事可做 → MICROROS_SPIN_TIMEOUT_MS 毫秒后返回
 *
 * @note  在主循环中每 10ms 调用，与 beep_handle()、cmd_vel_tick() 配对
 * @see   MICROROS_SPIN_TIMEOUT_MS
 */
void microros_bridge_spin(void);

/**
 * @brief  检测当前是否与 micro-ROS Agent 保持连接
 * @return true=Agent 可达, false=Agent 断开或尚未连接
 */
bool microros_bridge_is_agent_connected(void);

/**
 * @brief 云台是否处于活跃遥控状态（最近 1 秒内收到过 /cmd_ptz_angle）
 * @return true=活跃，false=空闲
 */
bool microros_bridge_is_ptz_active(void);

#ifdef __cplusplus
}
#endif
