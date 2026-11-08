/**
 * @file    selftest.c
 * @brief   电机 + 编码器自检实现
 *
 * 目的：更换控制板后，先确认电机能转、编码器能计数、方向一致，
 *       再跑完整的 micro-ROS 固件。
 *
 * 使用：car_config.h 里把 SELFTEST_MOTOR_ENCODER 设为 1，编译烧录即可；
 *       测完改回 0 重新烧录，恢复正常固件。
 */
#include "selftest.h"
#include "car_config.h"
#include "motor.h"
#include "encoder.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

static const char *TAG = "SELFTEST";

/**
 * @brief 执行一个动作并打印两侧编码器脉冲增量
 *
 * @param name  动作名
 * @param pwm_l 左轮 PWM (-1000 ~ 1000)
 * @param pwm_r 右轮 PWM (-1000 ~ 1000)
 * @param ms    持续时间
 */
static void run_phase(motor_handle_t ml, motor_handle_t mr,
                      encoder_handle_t el, encoder_handle_t er,
                      const char *name, int pwm_l, int pwm_r, int ms)
{
    /* 先清零并等一拍，保证计数从 0 开始 */
    encoder_reset(el);
    encoder_reset(er);
    vTaskDelay(pdMS_TO_TICKS(100));

    ESP_LOGI(TAG, "── %s   左PWM=%d 右PWM=%d  %dms", name, pwm_l, pwm_r, ms);
    motor_set_speed(ml, pwm_l);
    motor_set_speed(mr, pwm_r);
    vTaskDelay(pdMS_TO_TICKS(ms));

    motor_stop(ml);
    motor_stop(mr);
    vTaskDelay(pdMS_TO_TICKS(300));

    int32_t tl = encoder_get_ticks(el);
    int32_t tr = encoder_get_ticks(er);

    ESP_LOGI(TAG, "   结果: 左 ticks=%ld   右 ticks=%ld", (long)tl, (long)tr);
}

void selftest_motor_encoder(void)
{
    motor_handle_t   ml, mr;
    encoder_handle_t el, er;

    ESP_LOGW(TAG, "==========================================");
    ESP_LOGW(TAG, "   电机 + 编码器 自检");
    ESP_LOGW(TAG, "   左电机 GPIO%d/%d    左编码器 GPIO%d/%d",
             PIN_MOTOR_L_A, PIN_MOTOR_L_B, PIN_ENCODER_L_A, PIN_ENCODER_L_B);
    ESP_LOGW(TAG, "   右电机 GPIO%d/%d    右编码器 GPIO%d/%d",
             PIN_MOTOR_R_A, PIN_MOTOR_R_B, PIN_ENCODER_R_A, PIN_ENCODER_R_B);
    ESP_LOGW(TAG, "==========================================");

    if (!motor_init(&ml, PIN_MOTOR_L_A, PIN_MOTOR_L_B, 0)) {
        ESP_LOGE(TAG, "左电机初始化失败，检查 GPIO%d/%d",
                 PIN_MOTOR_L_A, PIN_MOTOR_L_B);
        return;
    }
    if (!motor_init(&mr, PIN_MOTOR_R_A, PIN_MOTOR_R_B, 1)) {
        ESP_LOGE(TAG, "右电机初始化失败，检查 GPIO%d/%d",
                 PIN_MOTOR_R_A, PIN_MOTOR_R_B);
        return;
    }
    if (!encoder_init(&el, PIN_ENCODER_L_A, PIN_ENCODER_L_B)) {
        ESP_LOGE(TAG, "左编码器初始化失败，检查 GPIO%d/%d",
                 PIN_ENCODER_L_A, PIN_ENCODER_L_B);
        return;
    }
    if (!encoder_init(&er, PIN_ENCODER_R_A, PIN_ENCODER_R_B)) {
        ESP_LOGE(TAG, "右编码器初始化失败，检查 GPIO%d/%d",
                 PIN_ENCODER_R_A, PIN_ENCODER_R_B);
        return;
    }

    ESP_LOGW(TAG, "初始化完成，2 秒后开始。请把车轮悬空！");
    vTaskDelay(pdMS_TO_TICKS(2000));

    while (1) {
        run_phase(ml, mr, el, er, "[1/5] 左轮正转",   400,    0, 1500);
        run_phase(ml, mr, el, er, "[2/5] 右轮正转",     0,  400, 1500);
        run_phase(ml, mr, el, er, "[3/5] 双轮正转",   400,  400, 1500);
        run_phase(ml, mr, el, er, "[4/5] 双轮反转",  -400, -400, 1500);
        run_phase(ml, mr, el, er, "[5/5] 原地右转",   400, -400, 1500);

        ESP_LOGW(TAG, "==== 一轮结束，停 3 秒 ====");
        motor_stop(ml);
        motor_stop(mr);
        vTaskDelay(pdMS_TO_TICKS(3000));
    }
}
