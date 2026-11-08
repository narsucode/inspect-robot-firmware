# 巡检机器人下位机固件

> ESP32-S3 + FreeRTOS + micro-ROS
> 差速底盘控制、激光雷达、IMU、云台、电池监测

配套上位机：[inspect-robot-ros2](https://github.com/narsucode/inspect-robot-ros2)
\nDemo video: https://www.bilibili.com/video/BV1LUWJ6yEtp

---

## 功能

```text
订阅  /cmd_vel            geometry_msgs/Twist     差速运动学 → PID → 电机 PWM
订阅  /cmd_ptz_angle      geometry_msgs/Vector3   云台 pan/tilt（度）

发布  /scan               sensor_msgs/LaserScan   10 Hz，360 点，1° 间隔
发布  /odom               nav_msgs/Odometry       20 Hz，编码器航位推算
发布  /imu                sensor_msgs/Imu         25 Hz，MPU6050 + Madgwick
发布  /battery            std_msgs/UInt16         0.2 Hz，ADC 采样
发布  /debug              std_msgs/String         5 Hz，环形缓冲日志
```

---

## 硬件

| 部件 | 型号 | 接口 |
|---|---|---|
| 主控 | ESP32-S3 | — |
| 激光雷达 | YDLidar X3 Pro | UART1 115200，RX=GPIO18，M_CTR=GPIO17 |
| IMU | MPU6050 | I2C |
| 电机 | 直流减速电机 + 编码器 | 双路 PWM + 正交编码器 |
| 云台 | 二自由度舵机 | LEDC PWM 50 Hz，pan=GPIO1，tilt=GPIO13 |
| 电池 | 3S 锂电 | ADC 分压采样 |

---

## 任务划分

```text
Core 0   micro-ROS 通信、速度 PID、电池采样、OLED 刷新（10 ms 周期）
Core 1   激光雷达解析（独占核心，避免控制抖动影响数据完整性）
```

雷达解析放 Core 1 是关键：YDLidar 数据帧连续，被抢占会导致丢包、点云出现缺口。

---

## 关键设计

### 1. 云台指令补发

下位机是「收到指令就动」的绝对值范式，UDP 丢一包云台就卡住。

上位机侧在 `hold` 秒内每 0.4 秒补发一次，等效软重传。

### 2. 雷达方向补偿

YDLidar 顺时针输出，且雷达在车上装反了 180°，所以索引要映射：

```c
int lidar_idx = (180 - i + 360) % 360;
```

改这一行之前，四方向验证过：车头障碍物在图上的 0°。

### 3. LEDC 通道分配

雷达电机占用了 `LEDC_CHANNEL_0` 和 `TIMER_0`，云台必须避开：

```c
#define PAN_CH      LEDC_CHANNEL_2
#define TILT_CH     LEDC_CHANNEL_3
#define GIMBAL_TMR  LEDC_TIMER_1
```

冲突的表现是雷达转不动或云台乱抖。

### 4. 低速转向力矩

`CMD_VEL_MIN_PWM` 从 220 降到 180。原来的值在小角速度指令下电机起不来，表现为「按键了车不转」。

---

## 编译烧录

```bash
source ~/idf.sh
cd patrol_car
idf.py menuconfig      # WiFi SSID / 密码 / agent IP / 端口
idf.py build
idf.py -p /dev/ttyUSB0 flash monitor
```

### menuconfig 必填项

```text
Example Configuration  →  WiFi SSID / WiFi Password
Micro-ROS Settings     →  Agent IP（虚拟机 IP）/ Agent Port（8888）
```

### 烧完验证

```bash
docker restart mrosagent        # 重启 agent
# 按板子 RST
ros2 daemon stop; sleep 8
ros2 node list | grep -c esp32              # 要 1
ros2 topic info /scan | grep "Publisher count"   # 要 1
```

出现 2 说明板子连了两次，把 agent 重启后再给板子断电重上电。

---

## 踩过的坑

1. **LEDC 通道冲突**
   雷达用 `CHANNEL_0` + `TIMER_0`，云台必须用 `CHANNEL_2/3` + `TIMER_1`，否则互相干扰。

2. **雷达索引方向**
   硬件装反 180°，且 YDLidar 顺时针输出，索引要 `(180 - i + 360) % 360`。

3. **低速转不动**
   `CMD_VEL_MIN_PWM` 太高时小角速度起不来，降到 180 解决。

4. **tilt 一直仰头**
   舵机中位不是 1500 µs，机械牙盘错齿导致。要么改 `GIMBAL_TILT_CENTER`，要么在 ROS 侧加偏移。

5. **agent 重启后板子连不上**
   顺序必须是「先重启 agent，再给板子上电」。反过来会留下两个会话，话题重复。

6. **WiFi 省电导致 /scan 周期性延迟**
   加 `esp_wifi_set_ps(WIFI_PS_NONE)` 关掉 modem sleep。

---

## 目录结构

```text
patrol_car/
├── main/main.c              app_main + 周期任务
├── bsp/                     硬件驱动
│   ├── ydlidar_x3pro.c/.h   雷达解析（Core 1）
│   ├── cmd_vel_control.c    差速运动学 + PID
│   ├── odometry.c           编码器航位推算
│   ├── mpu6050.c            IMU 驱动 + Madgwick
│   └── car_config.h         引脚、参数集中配置
├── ros/microros_bridge.c    micro-ROS 桥（订阅 + 定时发布）
├── vision/                  云台控制 + 视觉追踪
└── sdkconfig                WiFi 与 agent 配置（不入库）
```

---

## 许可

MIT
