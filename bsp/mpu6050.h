/**
 * @file    mpu6050.h
 * @brief   MPU6050 六轴姿态传感器驱动 — I2C 接口
 *
 * 三步拆解术:
 *   1. 直观理解: MPU6050 是机器人的"内耳"——
 *      同时测量三轴加速度 (感受重力/运动) 和三轴角速度 (感受旋转)，
 *      就像你闭着眼也能知道自己有没有歪、有没有被推。
 *   2. 追根溯源: 没有 IMU，你只能靠轮子编码器推算位置——
 *      但轮子打滑一次，整个定位就飘了。IMU 提供绝对姿态参考，
 *      编码器 + IMU 融合 (卡尔曼滤波) 才能做精确导航。
 *   3. 正规解释: MPU6050 集成 3 轴陀螺仪 (MPU3050) 和 3 轴加速度计，
 *      I2C 地址 0x68。加速度量程 ±2g (16384 LSB/g)，
 *      陀螺仪量程 ±250°/s (131 LSB/°/s)，内置温度传感器。
 *
 * 滤波器链:
 *   硬件 DLPF (CONFIG 寄存器, 44Hz)
 *    → 软件 EMA 低通 (可选, 默认 alpha=0.3)
 *    → Madgwick AHRS 姿态融合 (加速度计修正陀螺仪漂移)
 *    → 输出四元数 + SI 单位 (m/s², rad/s)
 */

#ifndef __MPU6050_H__
#define __MPU6050_H__

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** IMU 数据包 — 一次读取返回全部传感器数据 */
typedef struct {
    float ax, ay, az;        /**< 加速度 (g), 静止时 az ≈ +1.00 */
    float gx, gy, gz;        /**< 角速度 (°/s) */
    float temp;              /**< 芯片温度 (°C) */
} imu_data_t;

/** Madgwick AHRS 滤波器 — 加速度计 + 陀螺仪 → 姿态四元数 */
typedef struct {
    float beta;              /**< 滤波器增益 (默认 0.1f, 越大越信任陀螺仪) */
    float q0, q1, q2, q3;   /**< 当前姿态四元数 (w, x, y, z) */
    int64_t last_time_us;    /**< 上次更新时间 (微秒), 用于计算 dt */
} madgwick_t;

/** 软件 EMA 低通滤波状态 */
typedef struct {
    float ax_f, ay_f, az_f;  /**< 滤波后的加速度 (g) */
    float gx_f, gy_f, gz_f;  /**< 滤波后的角速度 (°/s) */
    bool  enabled;            /**< 是否启用 (默认 true) */
    float accel_alpha;        /**< 加速度平滑系数 0~1, 越小滤波越强 */
    float gyro_alpha;         /**< 陀螺仪平滑系数 0~1, 越小滤波越强 */
} imu_ema_t;

/** ROS 2 标准格式的 IMU 数据包 */
typedef struct {
    float ax, ay, az;        /**< 线加速度 (m/s²), 静止时 az ≈ +9.81 */
    float gx, gy, gz;        /**< 角速度 (rad/s) */
    float qw, qx, qy, qz;   /**< 姿态四元数 (w,x,y,z), Madgwick 融合输出 */
    float temp;              /**< 芯片温度 (°C) */
} imu_ros_data_t;

/** @brief 默认 EMA 平滑系数 (加速度 / 陀螺仪) */
#define IMU_EMA_DEFAULT_ACCEL_ALPHA  0.3f
#define IMU_EMA_DEFAULT_GYRO_ALPHA   0.7f

/* ──── MPU6050 寄存器地址 ──── */
#define MPU6050_ADDR          0x68    /**< I2C 设备地址 */
#define MPU6050_WHO_AM_I      0x75    /**< 设备 ID 寄存器 (应返回 0x68) */
#define MPU6050_PWR_MGMT_1    0x6B    /**< 电源管理 (写 0x00 唤醒) */
#define MPU6050_SMPLRT_DIV    0x19    /**< 采样率分频 */
#define MPU6050_CONFIG        0x1A    /**< 低通滤波配置 */
#define MPU6050_GYRO_CONFIG   0x1B    /**< 陀螺仪量程 */
#define MPU6050_ACCEL_CONFIG  0x1C    /**< 加速度计量程 */
#define MPU6050_ACCEL_XOUT_H  0x3B    /**< 加速度数据起始地址 */

/**
 * @brief 初始化并校准 MPU6050
 *
 * 执行: 唤醒 → 配置量程 → 检查 WHO_AM_I → 采样 200 次校准零偏
 * @return true=成功, false=未检测到设备
 * @note   校准期间需保持传感器静止（正面朝上）
 */
bool imu_init(void);

/**
 * @brief 检查 MPU6050 是否正确连接
 * @return true=WHO_AM_I 返回 0x68
 */
bool imu_check(void);

/**
 * @brief 执行零偏校准 (200 次采样取平均)
 *
 * 陀螺仪: 取静止时平均值作为零偏，后续读数减零偏
 * 加速度计: ax/ay 零偏 ≈ 0, az 零偏 = 平均值 - 1g
 */
void imu_calibrate(void);

/**
 * @brief 读取六轴数据 + 温度
 * @param out  输出数据包
 * @return true=读数成功
 */
bool imu_read(imu_data_t *out);

/** @brief 内置测试循环 (阻塞运行，每秒打印一份数据) */
void imu_test(void);

/* ================================================================
 * 高级 API — ROS 2 标准格式输出
 * ================================================================ */

/**
 * @brief 初始化 Madgwick AHRS 滤波器
 * @param filter  滤波器句柄
 * @param beta    滤波器增益 (推荐 0.03~0.1)
 *
 * beta 越小越信任加速度计 (姿态更稳但响应慢),
 * beta 越大越信任陀螺仪 (响应快但容易漂)。
 * MPU6050 陀螺仪噪声偏大, 25Hz 更新时建议 beta=0.05
 * (与 50Hz 下的 beta=0.1 时间域效果接近)。
 */
void madgwick_init(madgwick_t *filter, float beta);

/**
 * @brief 用最新 IMU 数据更新 Madgwick 滤波器
 *
 * @param filter  滤波器句柄
 * @param gx,gy,gz  陀螺仪数据 (rad/s)
 * @param ax,ay,az  加速度计数据 (g, 会被内部归一化)
 *
 * dt 由内部时间戳自动计算, 无需外部传入。
 * 调用后会更新 filter->q0~q3。
 */
void madgwick_update(madgwick_t *filter,
                     float gx, float gy, float gz,
                     float ax, float ay, float az);

/**
 * @brief 初始化 EMA 低通滤波器
 * @param ema          滤波器句柄
 * @param accel_alpha  加速度平滑系数 (0~1, 默认 0.3)
 * @param gyro_alpha   陀螺仪平滑系数 (0~1, 默认 0.7)
 *
 * 加速度建议较强滤波以抑制机体振动；
 * 陀螺仪建议较弱滤波以保持旋转响应速度。
 */
void imu_ema_init(imu_ema_t *ema, float accel_alpha, float gyro_alpha);

/**
 * @brief 对原始 IMU 数据应用 EMA 低通滤波
 * @param ema  滤波器句柄
 * @param raw  原始数据 (输入输出合一, 滤波后写回 raw)
 */
void imu_ema_apply(imu_ema_t *ema, imu_data_t *raw);

/**
 * @brief 初始化 IMU 滤波器链 (Madgwick + EMA)
 *
 * 将滤波器状态管理收敛到驱动模块内部，调用者只需
 * 传入参数一次，随后 imu_read_ros() 自动使用内部状态。
 *
 * @param beta          Madgwick 增益 (25Hz 下推荐 0.05)
 * @param accel_alpha   加速度 EMA 平滑系数 (默认 0.3)
 * @param gyro_alpha    陀螺仪 EMA 平滑系数 (默认 0.7)
 */
void imu_filter_init(float beta, float accel_alpha, float gyro_alpha);

/**
 * @brief 读取 IMU 数据并输出 ROS 2 标准格式 (SI 单位 + 姿态四元数)
 *
 * 完整数据流:
 *   1. imu_read() 读取原始数据 (g, °/s)
 *   2. EMA 低通滤波 (内部状态)
 *   3. Madgwick AHRS 姿态融合 (内部状态)
 *   4. 单位转换: g → m/s², °/s → rad/s
 *
 * @param out   ROS 格式输出 (SI 单位 + 四元数)
 * @return true=读数成功, false=I2C 通信失败
 *
 * @note  需先调用 imu_filter_init() 初始化滤波器
 * @note  调用频率建议 25~200Hz
 * @note  静止时 out->az ≈ +9.81 m/s² (重力)
 */
bool imu_read_ros(imu_ros_data_t *out);

/**
 * @brief 调整硬件 DLPF 截止频率
 * @param dlpf_cfg  DLPF_CFG 寄存器值 (0~6)
 *
 * MPU6050 DLPF 配置表:
 *   0: 加速度 260Hz, 陀螺仪 256Hz (无滤波)
 *   1: 加速度 184Hz, 陀螺仪 188Hz
 *   2: 加速度  94Hz, 陀螺仪  98Hz
 *   3: 加速度  44Hz, 陀螺仪  42Hz  ← 默认 (推荐)
 *   4: 加速度  21Hz, 陀螺仪  20Hz
 *   5: 加速度  10Hz, 陀螺仪  10Hz
 *   6: 加速度   5Hz, 陀螺仪   5Hz
 */
void imu_set_dlpf(uint8_t dlpf_cfg);

#ifdef __cplusplus
}
#endif

#endif
