#include "mpu6050.h"
#include "i2c_bus.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include <math.h>

static const char *TAG = "IMU_DRV";

static float s_gx_offset = 0.0f;
static float s_gy_offset = 0.0f;
static float s_gz_offset = 0.0f;

/* ========== 加速度计校准参数 ========== */
static float s_ax_offset = 0.0f;
static float s_ay_offset = 0.0f;
static float s_az_offset = 0.0f;


/* 从 MPU6050 指定寄存器读取一段数据。 */
static esp_err_t mpu6050_read_bytes(uint8_t reg, uint8_t *buf, size_t len)
{
    return i2c_master_write_read_device(I2C_NUM_0, MPU6050_ADDR,
                                        &reg, 1, buf, len, pdMS_TO_TICKS(100));
}

/* 向 MPU6050 指定寄存器写入一个字节。 */
static esp_err_t mpu6050_write_byte(uint8_t reg, uint8_t data)
{
    uint8_t buf[2] = {reg, data};
    return i2c_master_write_to_device(I2C_NUM_0, MPU6050_ADDR, buf, 2, pdMS_TO_TICKS(100));
}

/* 检查 MPU6050 是否正确响应，并返回设备识别结果。 */
bool imu_check(void)
{
    uint8_t who_am_i = 0;
    esp_err_t ret = mpu6050_read_bytes(MPU6050_WHO_AM_I, &who_am_i, 1);
    if (ret != ESP_OK || who_am_i != 0x68) {
        ESP_LOGE(TAG, "WHO_AM_I 读取失败，ret=%d，val=0x%02X", ret, who_am_i);
        return false;
    }

    ESP_LOGI(TAG, "MPU6050 检测成功，WHO_AM_I=0x%02X", who_am_i);
    return true;
}

/* 初始化 MPU6050，并完成基本配置与零偏校准。 */
bool imu_init(void)
{
    /* 先初始化公共 I2C 总线。 */
    ESP_ERROR_CHECK(i2c_bus_init());

    /* 唤醒 MPU6050。 */
    ESP_ERROR_CHECK(mpu6050_write_byte(MPU6050_PWR_MGMT_1, 0x00));
    vTaskDelay(pdMS_TO_TICKS(10));

    /* 配置采样率与滤波。
       SMPLRT_DIV=0 → 采样率 1kHz (配合 DLPF 使用);
       CONFIG=3     → DLPF 截止 44Hz/42Hz, 25Hz 输出仍满足奈奎斯特。 */
    ESP_ERROR_CHECK(mpu6050_write_byte(MPU6050_SMPLRT_DIV, 0x00));
    ESP_ERROR_CHECK(mpu6050_write_byte(MPU6050_CONFIG, 0x03));
    ESP_ERROR_CHECK(mpu6050_write_byte(MPU6050_GYRO_CONFIG, 0x00));
    ESP_ERROR_CHECK(mpu6050_write_byte(MPU6050_ACCEL_CONFIG, 0x00));

    vTaskDelay(pdMS_TO_TICKS(10));

    if (!imu_check()) {
        return false;
    }

    imu_calibrate();
    return true;
}
void imu_calibrate(void)
{
    imu_data_t imu;
    float sum_gx = 0.0f, sum_gy = 0.0f, sum_gz = 0.0f;
    float sum_ax = 0.0f, sum_ay = 0.0f, sum_az = 0.0f;
    int samples = 200;

    /* 先清零旧偏移，避免重复校准时被旧值污染 */
    s_gx_offset = s_gy_offset = s_gz_offset = 0.0f;
    s_ax_offset = s_ay_offset = s_az_offset = 0.0f;

    ESP_LOGI(TAG, "正在校准 IMU，请保持静止（正面朝上）...");

    for (int i = 0; i < samples; i++) {
        imu_read(&imu);
        sum_gx += imu.gx;
        sum_gy += imu.gy;
        sum_gz += imu.gz;
        sum_ax += imu.ax;
        sum_ay += imu.ay;
        sum_az += imu.az;
        vTaskDelay(pdMS_TO_TICKS(5));
    }

    /* 陀螺仪零偏 */
    s_gx_offset = sum_gx / samples;
    s_gy_offset = sum_gy / samples;
    s_gz_offset = sum_gz / samples;

    /* 加速度计零偏：静止时 ax/ay 应接近 0，az 应接近 1 */
    s_ax_offset = sum_ax / samples;          // 约 0
    s_ay_offset = sum_ay / samples;          // 约 0
    s_az_offset = sum_az / samples - 1.0f;   // 减去标准重力 1g

    ESP_LOGI(TAG, "校准完成:");
    ESP_LOGI(TAG, "  陀螺零偏: gx=%.2f gy=%.2f gz=%.2f", s_gx_offset, s_gy_offset, s_gz_offset);
    ESP_LOGI(TAG, "  加计零偏: ax=%.3f ay=%.3f az=%.3f", s_ax_offset, s_ay_offset, s_az_offset);
}

/* 从 MPU6050 读取加速度、陀螺仪和温度数据。 */
bool imu_read(imu_data_t *out)
{
    uint8_t buf[14];
    if (mpu6050_read_bytes(MPU6050_ACCEL_XOUT_H, buf, 14) != ESP_OK) {
        return false;
    }

    int16_t raw_ax = (buf[0] << 8) | buf[1];
    int16_t raw_ay = (buf[2] << 8) | buf[3];
    int16_t raw_az = (buf[4] << 8) | buf[5];
    int16_t raw_temp = (buf[6] << 8) | buf[7];
    int16_t raw_gx = (buf[8] << 8) | buf[9];
    int16_t raw_gy = (buf[10] << 8) | buf[11];
    int16_t raw_gz = (buf[12] << 8) | buf[13];

    out->ax = raw_ax / 16384.0f - s_ax_offset;
    out->ay = raw_ay / 16384.0f - s_ay_offset;
    out->az = raw_az / 16384.0f - s_az_offset;
    out->temp = raw_temp / 340.0f + 36.53f;
    out->gx = raw_gx / 131.0f - s_gx_offset;
    out->gy = raw_gy / 131.0f - s_gy_offset;
    out->gz = raw_gz / 131.0f - s_gz_offset;

    return true;
}

/* 执行 IMU 读取测试，并周期打印姿态与温度数据。 */
void imu_test(void)
{
    imu_data_t imu;

    while (1) {
        if (imu_read(&imu)) {
            ESP_LOGI(TAG, "加速度: ax=%.3fg  ay=%.3fg  az=%.3fg", imu.ax, imu.ay, imu.az);
            ESP_LOGI(TAG, "角速度: gx=%.2f°/s gy=%.2f°/s gz=%.2f°/s", imu.gx, imu.gy, imu.gz);
            ESP_LOGI(TAG, "温度: %.2f°C", imu.temp);
            ESP_LOGI(TAG, "-----------------------------------");
        } else {
            ESP_LOGW(TAG, "IMU 数据读取失败");
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

/* ================================================================
 * EMA 低通滤波器 — 抑制传感器高频噪声
 * ================================================================ */

void imu_ema_init(imu_ema_t *ema, float accel_alpha, float gyro_alpha)
{
    ema->enabled      = true;
    ema->accel_alpha  = accel_alpha;
    ema->gyro_alpha   = gyro_alpha;
    ema->ax_f = ema->ay_f = ema->az_f = 0.0f;
    ema->gx_f = ema->gy_f = ema->gz_f = 0.0f;
}

void imu_ema_apply(imu_ema_t *ema, imu_data_t *raw)
{
    if (!ema || !ema->enabled) return;

    float aa = ema->accel_alpha;
    float ga = ema->gyro_alpha;

    /* 加速度 EMA: 较强滤波，抑制机体振动 */
    ema->ax_f = aa * raw->ax + (1.0f - aa) * ema->ax_f;
    ema->ay_f = aa * raw->ay + (1.0f - aa) * ema->ay_f;
    ema->az_f = aa * raw->az + (1.0f - aa) * ema->az_f;

    /* 陀螺仪 EMA: 较弱滤波，保持旋转响应 */
    ema->gx_f = ga * raw->gx + (1.0f - ga) * ema->gx_f;
    ema->gy_f = ga * raw->gy + (1.0f - ga) * ema->gy_f;
    ema->gz_f = ga * raw->gz + (1.0f - ga) * ema->gz_f;

    raw->ax = ema->ax_f;
    raw->ay = ema->ay_f;
    raw->az = ema->az_f;
    raw->gx = ema->gx_f;
    raw->gy = ema->gy_f;
    raw->gz = ema->gz_f;
}

/* ================================================================
 * Madgwick AHRS — 加速度计 + 陀螺仪 → 姿态四元数
 *
 * 基于 Sebastian Madgwick 2011 年的经典算法。
 * 核心思想: 陀螺仪积分得到姿态 → 加速度计测重力方向来修正漂移
 *          → 梯度下降法最小化误差 → 输出无漂移姿态四元数
 *
 * 为什么选 Madgwick 而不是 Mahony:
 *   - Madgwick: 梯度下降, 收敛快, 对低频漂移修正强
 *   - Mahony:   比例-积分, 需要调 PI 参数, 对传感器噪声敏感
 *   - 互补滤波: 最简单但精度最差
 *
 * MPU6050 用 Madgwick 最合适: 陀螺仪噪声大, 需要加速度计强力修正。
 * ================================================================ */

void madgwick_init(madgwick_t *filter, float beta)
{
    filter->beta          = beta;
    filter->q0            = 1.0f;  /* w = 1: 朝向起始姿态 */
    filter->q1            = 0.0f;
    filter->q2            = 0.0f;
    filter->q3            = 0.0f;
    filter->last_time_us  = 0;
}

void madgwick_update(madgwick_t *filter,
                     float gx, float gy, float gz,
                     float ax, float ay, float az)
{
    float q0 = filter->q0, q1 = filter->q1, q2 = filter->q2, q3 = filter->q3;
    float beta = filter->beta;
    float recipNorm;
    float s0, s1, s2, s3;
    float qDot1, qDot2, qDot3, qDot4;
    float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2, _8q1, _8q2, q0q0, q1q1, q2q2, q3q3;

    /* ── 计算时间步长 dt ── */
    int64_t now = esp_timer_get_time();
    float dt = (filter->last_time_us > 0)
               ? (float)(now - filter->last_time_us) * 1.0e-6f
               : 0.04f;  /* 首次调用按 25Hz 估算 40ms */

    /* 安全限幅: dt 异常大 (长时间停更) 时限制为 50ms, 防积分爆炸 */
    if (dt <= 0.0f || dt > 0.05f) {
        dt = 0.04f;
    }
    filter->last_time_us = now;

    /* ── Step 1: 陀螺仪 → 四元数变化率 ── */
    qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
    qDot2 = 0.5f * ( q0 * gx + q2 * gz - q3 * gy);
    qDot3 = 0.5f * ( q0 * gy - q1 * gz + q3 * gx);
    qDot4 = 0.5f * ( q0 * gz + q1 * gy - q2 * gx);

    /* ── Step 2: 加速度计 → 梯度下降修正量 ──
     * 仅当加速度数据非零时计算 (避免除零) */
    if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

        /* 归一化加速度向量 → 单位重力方向 */
        recipNorm = 1.0f / sqrtf(ax * ax + ay * ay + az * az);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;

        /* 预计算辅助变量 (减少重复运算) */
        _2q0 = 2.0f * q0;
        _2q1 = 2.0f * q1;
        _2q2 = 2.0f * q2;
        _2q3 = 2.0f * q3;
        _4q0 = 4.0f * q0;
        _4q1 = 4.0f * q1;
        _4q2 = 4.0f * q2;
        _8q1 = 8.0f * q1;
        _8q2 = 8.0f * q2;
        q0q0 = q0 * q0;
        q1q1 = q1 * q1;
        q2q2 = q2 * q2;
        q3q3 = q3 * q3;

        /* 目标函数 f(四元数, 重力) 的雅可比转置 × f 的梯度 */
        s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
        s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1
             - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
        s2 = 4.0f * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3
             - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
        s3 = 4.0f * q1q1 * q3 - _2q1 * ax + 4.0f * q2q2 * q3 - _2q2 * ay;

        /* 归一化梯度方向 */
        recipNorm = 1.0f / sqrtf(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);
        s0 *= recipNorm;
        s1 *= recipNorm;
        s2 *= recipNorm;
        s3 *= recipNorm;

        /* 反馈修正: 陀螺仪预测 - beta × 梯度 */
        qDot1 -= beta * s0;
        qDot2 -= beta * s1;
        qDot3 -= beta * s2;
        qDot4 -= beta * s3;
    }

    /* ── Step 3: 欧拉积分 ── */
    q0 += qDot1 * dt;
    q1 += qDot2 * dt;
    q2 += qDot3 * dt;
    q3 += qDot4 * dt;

    /* ── Step 4: 归一化四元数 (防止数值漂移) ── */
    recipNorm = 1.0f / sqrtf(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    filter->q0 = q0 * recipNorm;
    filter->q1 = q1 * recipNorm;
    filter->q2 = q2 * recipNorm;
    filter->q3 = q3 * recipNorm;
}

/* ================================================================
 * 模块内滤波器状态 — 通信层无需关心
 * ================================================================ */
static madgwick_t  s_madgwick;       /**< Madgwick AHRS (内部) */
static imu_ema_t   s_imu_ema;        /**< EMA 低通 (内部) */

/* ================================================================
 * imu_read_ros — 一站式获取 ROS 2 标准 IMU 数据
 * ================================================================ */

/** 重力加速度常量 (m/s²) */
#define GRAVITY_MS2  9.80665f

/** 度 → 弧度 */
#define DEG2RAD      (M_PI / 180.0f)

/* ================================================================
 * imu_filter_init — 一次性初始化滤波器链
 * ================================================================ */

void imu_filter_init(float beta, float accel_alpha, float gyro_alpha)
{
    madgwick_init(&s_madgwick, beta);
    imu_ema_init(&s_imu_ema, accel_alpha, gyro_alpha);
}

bool imu_read_ros(imu_ros_data_t *out)
{
    imu_data_t raw;

    /* 1. 读取原始数据 */
    if (!imu_read(&raw)) {
        return false;
    }

    /* 2. 软件 EMA 低通滤波 (内部状态) */
    imu_ema_apply(&s_imu_ema, &raw);

    /* 3. 单位转换: °/s → rad/s (陀螺仪) */
    float gx_rad = raw.gx * DEG2RAD;
    float gy_rad = raw.gy * DEG2RAD;
    float gz_rad = raw.gz * DEG2RAD;

    /* 4. Madgwick 姿态融合 (内部状态) */
    madgwick_update(&s_madgwick, gx_rad, gy_rad, gz_rad,
                    raw.ax, raw.ay, raw.az);

    /* 5. 填充输出 — 全部 SI 单位 */
    out->ax   = raw.ax * GRAVITY_MS2;   /* g → m/s² */
    out->ay   = raw.ay * GRAVITY_MS2;
    out->az   = raw.az * GRAVITY_MS2;
    out->gx   = gx_rad;                 /* rad/s */
    out->gy   = gy_rad;
    out->gz   = gz_rad;
    out->qw   = s_madgwick.q0;           /* 四元数 w */
    out->qx   = s_madgwick.q1;           /* 四元数 x */
    out->qy   = s_madgwick.q2;           /* 四元数 y */
    out->qz   = s_madgwick.q3;           /* 四元数 z */
    out->temp = raw.temp;

    return true;
}

/* ================================================================
 * imu_set_dlpf — 运行时调整硬件低通滤波
 * ================================================================ */

void imu_set_dlpf(uint8_t dlpf_cfg)
{
    if (dlpf_cfg > 6) dlpf_cfg = 3;
    mpu6050_write_byte(MPU6050_CONFIG, dlpf_cfg);
    ESP_LOGI(TAG, "DLPF → cfg=%d (带宽请查手册)", dlpf_cfg);
}