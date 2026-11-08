/**
 * @file    motor.c
 * @brief   直流电机驱动实现 — MCPWM 全桥控制
 *
 * 工作原理（比喻版）:
 *   把 MCPWM 想象成两个"水龙头"（A 和 B），电机夹在中间：
 *   - 正转：打开 A 水龙头，关死 B → 电流从 A 流向 B
 *   - 反转：关死 A，打开 B 水龙头 → 电流从 B 流向 A
 *   - 停止：两个水龙头都关死 → 没有电流
 *   水龙头开多大 = PWM 占空比 = 转速快慢
 *
 * 技术细节:
 *   - MCPWM 时钟 20MHz，周期 1000 ticks → PWM 频率 = 20KHz
 *   - 比较器在定时器归零时更新（影子寄存器），避免 PWM 毛刺
 *   - 使用 force_level 切换高低电平，比 GPIO 切换更干净
 */

#include "motor.h"
#include "driver/mcpwm_prelude.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdlib.h>

#define TAG "motor"

/* MCPWM 时钟配置 */
#define PWM_CLOCK_HZ    20000000    // 20MHz
#define PWM_PERIOD      1000        // 20MHz/1000 = 20KHz，超出人耳范围

/* 电机内部数据结构 */
struct motor_t {
    mcpwm_timer_handle_t  timer;
    mcpwm_oper_handle_t   oper;
    mcpwm_cmpr_handle_t   cmpr_a;
    mcpwm_cmpr_handle_t   cmpr_b;
    mcpwm_gen_handle_t    gen_a;
    mcpwm_gen_handle_t    gen_b;
    int gpio_a;
    int gpio_b;
    int group_id;
};

bool motor_init(motor_handle_t *out_handle, int gpio_a, int gpio_b, int group_id)
{
    if (!out_handle || group_id < 0 || group_id > 1) return false;

    struct motor_t *m = calloc(1, sizeof(struct motor_t));
    if (!m) return false;

    m->gpio_a = gpio_a;
    m->gpio_b = gpio_b;
    m->group_id = group_id;

    /* ===== 第1步: 创建定时器 ===== */
    mcpwm_timer_config_t timer_cfg = {
        .group_id       = group_id,
        .clk_src        = MCPWM_TIMER_CLK_SRC_DEFAULT,
        .resolution_hz  = PWM_CLOCK_HZ,
        .period_ticks   = PWM_PERIOD,
        .count_mode     = MCPWM_TIMER_COUNT_MODE_UP,
    };
    ESP_ERROR_CHECK(mcpwm_new_timer(&timer_cfg, &m->timer));

    /* ===== 第2步: 创建操作器 ===== */
    mcpwm_operator_config_t oper_cfg = { .group_id = group_id };
    ESP_ERROR_CHECK(mcpwm_new_operator(&oper_cfg, &m->oper));
    mcpwm_operator_connect_timer(m->oper, m->timer);

    /* ===== 第3步: 创建两个比较器 (A/B) ===== */
    mcpwm_comparator_config_t cmp_cfg = {
        .flags.update_cmp_on_tez = true  // 定时器归零时更新（防毛刺）
    };
    ESP_ERROR_CHECK(mcpwm_new_comparator(m->oper, &cmp_cfg, &m->cmpr_a));
    ESP_ERROR_CHECK(mcpwm_new_comparator(m->oper, &cmp_cfg, &m->cmpr_b));

    /* ===== 第4步: 创建两个发生器 (A/B) ===== */
    mcpwm_generator_config_t gen_cfg_a = { .gen_gpio_num = gpio_a };
    mcpwm_generator_config_t gen_cfg_b = { .gen_gpio_num = gpio_b };
    ESP_ERROR_CHECK(mcpwm_new_generator(m->oper, &gen_cfg_a, &m->gen_a));
    ESP_ERROR_CHECK(mcpwm_new_generator(m->oper, &gen_cfg_b, &m->gen_b));

    /* ===== 第5步: 配置 PWM 行为 ===== */
    // 发生器A: 周期开始时拉高，等于比较器A时拉低
    mcpwm_generator_set_actions_on_timer_event(m->gen_a,
        MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP,
                                     MCPWM_TIMER_EVENT_EMPTY,
                                     MCPWM_GEN_ACTION_HIGH),
        MCPWM_GEN_TIMER_EVENT_ACTION_END());
    mcpwm_generator_set_actions_on_compare_event(m->gen_a,
        MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP,
                                       m->cmpr_a,
                                       MCPWM_GEN_ACTION_LOW),
        MCPWM_GEN_COMPARE_EVENT_ACTION_END());

    // 发生器B: 同上
    mcpwm_generator_set_actions_on_timer_event(m->gen_b,
        MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP,
                                     MCPWM_TIMER_EVENT_EMPTY,
                                     MCPWM_GEN_ACTION_HIGH),
        MCPWM_GEN_TIMER_EVENT_ACTION_END());
    mcpwm_generator_set_actions_on_compare_event(m->gen_b,
        MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP,
                                       m->cmpr_b,
                                       MCPWM_GEN_ACTION_LOW),
        MCPWM_GEN_COMPARE_EVENT_ACTION_END());

    /* ===== 第6步: 初始占空比 0（电机停转）===== */
    mcpwm_comparator_set_compare_value(m->cmpr_a, 0);
    mcpwm_comparator_set_compare_value(m->cmpr_b, 0);

    /* ===== 第7步: 启动定时器 ===== */
    mcpwm_timer_enable(m->timer);
    mcpwm_timer_start_stop(m->timer, MCPWM_TIMER_START_NO_STOP);

    *out_handle = m;
    ESP_LOGI(TAG, "✓ 电机就绪  GPIO:%d/%d  Group:%d  PWM:20KHz", gpio_a, gpio_b, group_id);
    return true;
}

bool motor_deinit(motor_handle_t handle)
{
    struct motor_t *m = (struct motor_t *)handle;
    if (!m) return false;

    mcpwm_timer_start_stop(m->timer, MCPWM_TIMER_STOP_EMPTY);
    mcpwm_timer_disable(m->timer);
    mcpwm_del_generator(m->gen_b);
    mcpwm_del_generator(m->gen_a);
    mcpwm_del_comparator(m->cmpr_b);
    mcpwm_del_comparator(m->cmpr_a);
    mcpwm_del_operator(m->oper);
    mcpwm_del_timer(m->timer);
    free(m);

    ESP_LOGI(TAG, "✓ 电机已释放");
    return true;
}

void motor_set_speed(motor_handle_t handle, int speed)
{
    struct motor_t *m = (struct motor_t *)handle;
    if (!m) return;

    /* 限幅 */
    if (speed > 1000)  speed = 1000;
    if (speed < -1000) speed = -1000;

    if (speed > 0) {
        /* 正转: A 出 PWM, B 拉低 */
        mcpwm_generator_set_force_level(m->gen_b, 0, true);
        mcpwm_comparator_set_compare_value(m->cmpr_a, speed);
        mcpwm_generator_set_force_level(m->gen_a, -1, true);  // -1 = 解除强制,恢复PWM
    } else if (speed < 0) {
        /* 反转: B 出 PWM, A 拉低 */
        mcpwm_generator_set_force_level(m->gen_a, 0, true);
        mcpwm_comparator_set_compare_value(m->cmpr_b, -speed);
        mcpwm_generator_set_force_level(m->gen_b, -1, true);
    } else {
        /* 停止: AB 都拉低 */
        mcpwm_generator_set_force_level(m->gen_a, 0, true);
        mcpwm_generator_set_force_level(m->gen_b, 0, true);
    }
}

void motor_stop(motor_handle_t handle)
{
    motor_set_speed(handle, 0);
}

void motor_test_run(void)
{
    motor_handle_t motor_l, motor_r;

    ESP_LOGI(TAG, "╔═══════════════════════════╗");
    ESP_LOGI(TAG, "║  电机驱动测试 v1.0          ║");
    ESP_LOGI(TAG, "║  左: GPIO%d/%d            ║", PIN_MOTOR_L_A, PIN_MOTOR_L_B);
    ESP_LOGI(TAG, "║  右: GPIO%d/%d            ║", PIN_MOTOR_R_A, PIN_MOTOR_R_B);
    ESP_LOGI(TAG, "╚═══════════════════════════╝");

    /* 初始化左右电机 */
    if (!motor_init(&motor_l, PIN_MOTOR_L_A, PIN_MOTOR_L_B, 0)) {
        ESP_LOGE(TAG, "左电机初始化失败！");
        return;
    }
    if (!motor_init(&motor_r, PIN_MOTOR_R_A, PIN_MOTOR_R_B, 1)) {
        ESP_LOGE(TAG, "右电机初始化失败！");
        motor_deinit(motor_l);
        return;
    }

    ESP_LOGI(TAG, "开始循环测试 (Ctrl+] 退出)...\n");

    int round = 0;
    while (1) {
        round++;

        ESP_LOGI(TAG, "─ 第 %d 轮: 双轮正转 50%% (2秒)", round);
        motor_set_speed(motor_l, 500);
        motor_set_speed(motor_r, 500);
        vTaskDelay(pdMS_TO_TICKS(2000));

        ESP_LOGI(TAG, "─ 第 %d 轮: 停止 (0.5秒)", round);
        motor_stop(motor_l);
        motor_stop(motor_r);
        vTaskDelay(pdMS_TO_TICKS(500));

        ESP_LOGI(TAG, "─ 第 %d 轮: 双轮反转 50%% (2秒)", round);
        motor_set_speed(motor_l, -500);
        motor_set_speed(motor_r, -500);
        vTaskDelay(pdMS_TO_TICKS(2000));

        ESP_LOGI(TAG, "─ 第 %d 轮: 停止 (0.5秒)", round);
        motor_stop(motor_l);
        motor_stop(motor_r);
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}
