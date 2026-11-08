/**
 * @file    selftest.h
 * @brief   电机 + 编码器自检（不依赖 WiFi / micro-ROS）
 */
#ifndef __SELFTEST_H__
#define __SELFTEST_H__

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 电机 + 编码器自检主函数（死循环，永不返回）
 *
 * 依次执行 5 个动作，每个动作前后读编码器脉冲数并打印：
 *   1) 左轮正转   2) 右轮正转   3) 双轮正转
 *   4) 双轮反转   5) 原地右转（左正右反）
 *
 * 判断标准：
 *   - 轮子该转的转、该停的停
 *   - 正转时对应编码器脉冲为正，反转时为负
 *   - 单轮动作时另一侧脉冲接近 0
 */
void selftest_motor_encoder(void);

#ifdef __cplusplus
}
#endif

#endif /* __SELFTEST_H__ */
