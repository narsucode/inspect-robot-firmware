#include "led.h"          // LED 驱动接口定义
#include <stdint.h>       // 标准整数类型
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"

// LED 闪烁函数：非阻塞式定时翻转 LED 状态
// 参数：interval 闪烁间隔
void led_flash(uint16_t interval)
{
    // 间隔为 0 时直接返回，防止无效执行
    if (interval == 0) return;
    
    static uint8_t state = 0;   // 保存 LED 当前亮灭状态（0=灭/1=亮）
    static uint16_t count = 0;  // 计数器，记录函数被调用的次数
    
    count++;
    // 计数器达到设定间隔值时执行状态翻转
    if (count >= interval)
    {
        count = 0;               // 计数器清零
        state = !state;          // 状态取反
        led_state(state);        // 更新 LED 状态
    }
}

// LED 初始化函数：配置 GPIO 为推挽输出模式
void led_init(void)
{
    // 定义 GPIO 配置结构体并初始化为 0，符合 C 语言标准
    gpio_config_t io_conf = {0};
    
    // 禁用 GPIO 中断
    io_conf.intr_type = GPIO_INTR_DISABLE;
    // 设置 GPIO 为输出模式
    io_conf.mode = GPIO_MODE_OUTPUT;
    // 设置要控制的 LED 引脚（位运算选中对应引脚）
    io_conf.pin_bit_mask = (1ULL << LED_GPIO);
    // 禁用内部下拉电阻
    io_conf.pull_down_en = 0;
    // 禁用内部上拉电阻
    io_conf.pull_up_en = 0;
    
    // 将配置写入 GPIO 寄存器
    gpio_config(&io_conf);
    
    // 初始化完成后默认关闭 LED
    led_off();
}

// 打开LED灯 turn on led
void led_on(void)
{
    gpio_set_level(LED_GPIO, LED_ACTIVE_LEVEL);
}

// 关闭LED灯 turn off led
void led_off(void)
{
    gpio_set_level(LED_GPIO, !LED_ACTIVE_LEVEL);
}

// 翻转LED灯的状态
void led_state(uint8_t state)
{
    if (state == 0)
    {
        gpio_set_level(LED_GPIO, !LED_ACTIVE_LEVEL);
    }
    else
    {
        gpio_set_level(LED_GPIO, LED_ACTIVE_LEVEL);
    }
}

/**
 * @brief  测试 LED 控制函数
 */
void test_led(void)
{
    printf("测试LED\n");
    led_init();  // 初始化 LED
    while (1)
    {
        led_flash(50);  // 设置闪烁间隔为 50 次调用（约 50ms）
        vTaskDelay(pdMS_TO_TICKS(10));  // 延时 10ms，确保循环不会过快
    }

    
}

