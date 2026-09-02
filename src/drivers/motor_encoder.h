/**
 * @file motor_encoder.h
 * @brief 双轮正交编码器累计计数与ISR入口。
 *
 * 本接口只提供AB边沿累计值。轮周长、每圈计数、RPM和车体方向换算由上层负责；
 * 每个读取接口返回单轮快照，两次调用并不保证左右轮严格同时采样。
 */

#ifndef MOTOR_ENCODER_H
#define MOTOR_ENCODER_H

#include <stdint.h>
#include "motor_driver.h"

/**
 * @brief 上电初始化四个AB输入、双边沿EXINT和当前AB状态。
 * 依赖板级外部上拉；左轴计数因子+1，右轴-1，NVIC在状态初始化后使能。
 */
void motor_encoder_init(void);
/**
 * @brief 读取单轮累计边沿数，供速度环或里程计求增量。
 * @param motor MOTOR_LEFT或MOTOR_RIGHT。
 * @return 有符号累计计数；对齐32位读取是单值快照，单位不是RPM或mm。
 */
int32_t motor_encoder_get_count(motor_id_t motor);
/**
 * @brief 读取单轮AB两位同时跳变的诊断计数，不清除它。
 * @param motor MOTOR_LEFT或MOTOR_RIGHT。
 * @return 累计非法转换次数，不等于已确定的丢失脉冲数量。
 */
uint32_t motor_encoder_get_invalid_transition_count(motor_id_t motor);
/**
 * @brief 停车时清零两轴计数及诊断统计，内部短暂屏蔽中断后恢复原状态。
 * 调用后还须同步速度环上一计数和里程计原点，防止出现虚假的位移/速度跳变。
 */
void motor_encoder_reset_all(void);
/**
 * @brief 由EXINT ISR在确认并清除左轮挂起标志后调用。
 * 读取当前AB组合并累计一次状态变化，不等待、不进行速度计算。
 */
void motor_encoder_left_edge_handler(void);
/**
 * @brief 由EXINT ISR在确认并清除右轮挂起标志后调用。
 * 复用同一Gray码解码表，并应用右轴-1方向系数。
 */
void motor_encoder_right_edge_handler(void);

#endif
