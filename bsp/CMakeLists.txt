idf_component_register(
    SRCS "ydlidar_x3pro.c" "encoder.c" "i2c_bus.c" "mpu6050.c" "battery.c" "beep.c" "led.c" "key.c" "motor.c" "oled.c" "odometry.c" "cmd_vel_control.c" "pid.c" "debug_log.c" "display_status.c" "voice.c" "sys_monitor.c"
    INCLUDE_DIRS "." "../ros"
    REQUIRES driver esp_adc esp_timer
)