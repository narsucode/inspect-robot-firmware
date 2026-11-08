/**
 * @file    car_config.h
 * @brief   小车参数总配置 — 全车唯一调参面板
 *
 * 设计思路：
 *   1. 直观理解：本文件是小车硬件与控制参数总表。轮子尺寸、轮距、电机参数、PID参数全部集中在此。
 *   2. 历史问题：旧代码轮距、PID、引脚分散在多个头文件。更换硬件需要多处修改，容易出现参数不一致，小车运动异常。
 *   3. 设计原则：单一数据源(Single Source of Truth)。所有硬件常量在此统一定义，其他模块通过#include引用，避免重复定义带来参数漂移。
 *
 * 使用规则：
 *   - 更换轮子/电机/引脚、调试PID，仅修改本文件，重新编译即可生效。
 *   - 本文件仅存放硬件常量与控制参数。ROS话题名、发布频率放在 ros/microros_bridge.h。
 */

#ifndef CAR_CONFIG_H
#define CAR_CONFIG_H

/** @brief OLED 状态屏最小刷新间隔(us)。
 *  oled.c 是逐字节 I2C 事务，整屏重画要几十~上百毫秒；
 *  不限频会把 10ms 控制循环堵死，表现为遥控延迟。 */
#define OLED_MIN_REFRESH_US     1000000


#ifdef __cplusplus
extern "C" {
#endif

/* =============================================================================
 * 0. 调试开关
 * ========================================================================== */

/**
 * @brief 串口调试开关
 *
 * 开启：debug_log_printf 同时输出到串口与 /debug 话题。
 * 关闭：仅通过无线发送 /debug 话题，降低CPU占用。
 *
 * @note  正常运行建议注释；仅现场串口调试时打开。
 */
// #define SERIAL_DEBUG

/* =============================================================================
 * 1. 底盘机械参数 | 更换轮子/电机修改此节
 * ========================================================================== */

/** @brief 固件版本号 */
#define CAR_FIRMWARE_VERSION    "v1.0"

/** @brief 硬件代号 */
#define CAR_HARDWARE_NAME       "YouDaoBot"

/** @brief 轮子直径 (m) */
#define WHEEL_DIAMETER_M        0.065f

/** @brief 轮子周长 (m) = π × 直径 */
#define WHEEL_CIRCUMFERENCE_M   0.2042f


/** @brief 轮距 (m)，两轮中心实测间距 */
#define WHEEL_SEPARATION_M      0.178f

/** @brief 电机减速比，电机轴转速 : 轮子输出轴转速 */
#define MOTOR_GEAR_RATIO        44.9f

/** @brief 编码器PPR，电机轴每圈原始脉冲数（未倍频） */
#define ENCODER_PPR             11

/** @brief 轮子每圈编码器总脉冲数 = PPR × 4正交倍频 × 减速比 */
#define TICKS_PER_WHEEL_REV     ((ENCODER_PPR) * 4 * (MOTOR_GEAR_RATIO))

/** @brief 单个脉冲对应的行驶距离 (m) */
/** @brief 里程计标定系数：实测里程计偏小 5.9%，用它补偿。
 *  标定方法：跑固定距离，k = 实际距离 / 里程计距离。 */
#define ODOM_DIST_SCALE         1.062f

/** @brief 单个脉冲对应的行驶距离 (m)，含标定系数 */
#define DIST_PER_TICK_M         (WHEEL_CIRCUMFERENCE_M / TICKS_PER_WHEEL_REV * ODOM_DIST_SCALE)
/** @brief 右轮脉冲当量修正：实测右轮每圈多走约 1.25%%，
 *  用于补偿两轮有效周长差（左右分别标定）。 */
#define DIST_PER_TICK_R_TRIM        1.0f

/** @brief 满PWM对应的最大线速度(m/s)，前馈开环基准 */
#define CAR_MAX_SPEED_MPS       0.19f

/** @brief 电机PWM满量程，motor_set_speed()范围 ±此值 */
#define MOTOR_PWM_MAX           1000

/* =============================================================================
 * 2. 引脚分配 | 硬件接线变更修改此节
 * ========================================================================== */

/* 电机 MCPWM */
#define PIN_MOTOR_L_A           15      /**< 左电机A相 MCPWM Group0 */
#define PIN_MOTOR_L_B           16      /**< 左电机B相 */
#define PIN_MOTOR_R_A           21      /**< 右电机A相 MCPWM Group1 */
#define PIN_MOTOR_R_B           19      /**< 右电机B相 */

/* 编码器 PCNT正交解码 */
#define PIN_ENCODER_L_A         11      /**< 左编码器A相，远离电机线降低EMI干扰 */
#define PIN_ENCODER_L_B         12      /**< 左编码器B相 */
#define PIN_ENCODER_R_A         48      /**< 右编码器A相 */
#define PIN_ENCODER_R_B         47      /**< 右编码器B相 */

/* 其他外设 */
#define PIN_BEEP                3       /**< 蜂鸣器，高电平触发 */
#define PIN_BATTERY_ADC         2       /**< 电池分压采样 ADC1_CH1 (GPIO2) */
#define PIN_I2C_SDA             8       /**< I2C数据，OLED/MPU6050共用 */
#define PIN_I2C_SCL             9       /**< I2C时钟 */

/* =============================================================================
 * 2.1 语音播报模块 (天问, UART2)
 *
 * 接线: ESP32 GPIO40(TX) → 模块 RX, GPIO41(RX) → 模块 TX, 必须共地
 * 注意: GPIO40/41 是 ESP32-S3 的 JTAG 脚, 接 OpenOCD 调试时会冲突
 * ========================================================================== */

/** @brief 语音模块串口 TX (ESP32 发 → 模块收) */
#define PIN_VOICE_TX            40

/** @brief 语音模块串口 RX (模块发 → ESP32 收, 当前不解析) */
#define PIN_VOICE_RX            41

/** @brief 语音模块串口号 (UART_NUM_2, 雷达占 UART1, 日志占 UART0) */
#define VOICE_UART_NUM          2

/** @brief 语音模块波特率, 必须与天问配置一致 */
#ifndef VOICE_BAUD_RATE
#define VOICE_BAUD_RATE         115200
#endif

/** @brief 串口发送缓冲字节数 */
#define VOICE_TX_BUF_SIZE       512

/** @brief 相同事件码的最小重复间隔(ms), 防止 WiFi 抖动时连续播报 */
#define VOICE_MIN_GAP_MS        500

/* =============================================================================
 * 3. 电池电压检测
 * ========================================================================== */

/** @brief 分压比 = (40.2K + 10K) / 10K */
#define BATTERY_DIVIDER_RATIO   5.02f

/** @brief 低压预警阈值(V)，3S锂电 3.5V/单节 */
#define BATTERY_WARN_V          10.5f

/** @brief 低压告警阈值(V)，3S锂电 3.33V/单节 */
#define BATTERY_CRITICAL_V      10.0f

/** @brief 电压EMA滤波系数(0~1)，抑制电机负载瞬态压降 */
#define BATTERY_EMA_ALPHA       0.3f

/** @brief 电压合理性下限(V)：低于此值视为采样无效(未接电池/分压异常)，不告警 */
#define BATTERY_INVALID_BELOW_V  5.0f

/** @brief 电池告警蜂鸣开关：1=响，0=静音（调试期可关） */
#define BATTERY_ALARM_BEEP_ENABLE   0

/** @brief 告警冷却时间(s)，避免蜂鸣器连续鸣叫 */
#define BATTERY_ALARM_COOLDOWN_S 30

/* =============================================================================
 * 4. 编码器抗干扰 EMI
 * ========================================================================== */

/** @brief PCNT硬件毛刺滤波时间(ns) */
#define ENCODER_GLITCH_FILTER_NS    12000

/** @brief 软件脉冲突变阈值，超过判定EMI干扰 */
#define ENCODER_MAX_DELTA_PER_READ  500

/* =============================================================================
 * 5. /cmd_vel 速度指令 | 安全限制与运动手感
 * ========================================================================== */

/** @brief 看门狗：首次遥控超时(ms) */
#define CMD_VEL_WATCHDOG_FIRST_MS     1000
/** @brief 看门狗：持续遥控超时(ms) */
#define CMD_VEL_WATCHDOG_STREAM_MS     300
/** @brief 看门狗模式切换阈值(ms) */
#define CMD_VEL_WATCHDOG_GAP_THRESHOLD 200

/** @brief 速度死区(m/s)，小于该值判定停止，防止低速爬行 */
#define CMD_VEL_DEAD_ZONE           0.01f

/* ???????????????PWM ????????
 * ???????????PID ???????? */
#define CMD_VEL_MIN_PWM             180

/** @brief 目标速度加速度限制(m/s²)，限制起步冲击 */
#define CMD_VEL_ACCEL_LIMIT         2.0f

/** @brief PWM上升斜率限制，每10ms最大PWM增量 */
#define PWM_SLEW_RATE_UP            80
/** @brief PWM下降斜率限制，每10ms最大PWM减量 */
#define PWM_SLEW_RATE_DOWN          200

/** @brief 左电机正向校准系数 */
/* 直行速度同步开关
 * 1 = 把两轮目标都拉到"较慢轮"的实际速度（旧逻辑，轮径有差异时会制造偏差）
 * 0 = 两轮各自追自己的目标速度（旧固件做法，推荐） */
#define CAR_STRAIGHT_SYNC           1

#define TRIM_LEFT_FWD               1.0f
/** @brief 左电机反向校准系数 */
#define TRIM_LEFT_REV               1.0f
/** @brief 右电机正向校准系数 */
#define TRIM_RIGHT_FWD              1.0f
/** @brief 右电机反向校准系数 */
#define TRIM_RIGHT_REV              1.0f

/* =============================================================================
 * 6. 电机速度环 PID | 前馈+PID闭环
 * ========================================================================== */

/** @brief PID比例增益 */
#define SPEED_PID_KP                2000.0f
/** @brief PID积分增益 */
#define SPEED_PID_KI                1000.0f
/** @brief PID微分增益 */
#define SPEED_PID_KD                0.0f

/** @brief 左轮前馈系数：补偿两电机效率差异（实测标定） */
#define SPEED_FF_TRIM_L             1.00f

/** @brief 右轮前馈系数（实测 0.56，右电机明显更有力） */
#define SPEED_FF_TRIM_R             1.00f

/** @brief 前馈增益，目标速度直接换算基础PWM */
#define SPEED_PID_FF_GAIN           0.90f

/** @brief PID积分限幅，防止堵转积分饱和 */
#define SPEED_PID_INTEGRAL_LIMIT    100.0f

/** @brief PID输出下限 */
#define SPEED_PID_OUT_MIN           (-(float)MOTOR_PWM_MAX)
/** @brief PID输出上限 */
#define SPEED_PID_OUT_MAX           ( (float)MOTOR_PWM_MAX)

/* =============================================================================
 * 7. 偏航角速度PI校正 | 改善直行跑偏
 *
 * 原理：左右轮独立速度闭环会因机械不对称导致航向偏移。
 *      通过编码器计算真实偏航角速度，使用PI控制器动态修正两轮速度差，保持目标航向。
 * ========================================================================== */

/** @brief 偏航PI比例增益 */
#define YAW_PID_KP                  0.5f
/** @brief 偏航PI积分增益 */
#define YAW_PID_KI                  0.2f

/** @brief 偏航校正积分限幅(m/s) */
#define YAW_PID_INTEGRAL_LIMIT      0.25f

/** @brief 最大轮速差校正量(m/s) */
#define YAW_MAX_CORRECTION          0.25f

/* =============================================================================
 * 8. 云台舵机配置 | LEDC硬件PWM
 *
 * @note 舵机必须独立5V电源供电，与主板共地。直接使用主板5V会拉低电压造成ESP复位。
 * ========================================================================== */

/** @brief 云台水平舵机PWM引脚 */
#define PIN_GIMBAL_PAN          1

/** @brief 云台俯仰舵机PWM引脚 */
#define PIN_GIMBAL_TILT         13

#ifdef __cplusplus
}
#endif

#endif /* CAR_CONFIG_H */
