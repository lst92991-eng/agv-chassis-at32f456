/**
 * @file motor_driver.h
 * @brief 双AT8236的PWM占空比和停车电平接口。
 *
 * 输入单位是PWM千分比。车体速度到轮速、PID和符号标定由上层负责；此层将
 * 右桥输入再反相。普通零PWM选择Coast，主动制动需显式指定Brake。
 */

#ifndef MOTOR_DRIVER_H
#define MOTOR_DRIVER_H

#include <stdint.h>

/** 左右电机的硬件编号，也用作双轴状态数组索引。 */
typedef enum { MOTOR_LEFT = 0, MOTOR_RIGHT } motor_id_t;
/** Coast为IN1=IN2=0；Brake为IN1=IN2=1，短接电机端实现主动制动。 */
typedef enum { MOTOR_STOP_COAST = 0, MOTOR_STOP_BRAKE } motor_stop_mode_t;

/**
 * @brief 上电初始化PC6～PC9和TMR8四通道20kHz PWM。
 * 初始化末尾使能输出，初值为Coast；上层随后按启动策略切换到Brake。
 */
void motor_driver_init(void);
/**
 * @brief 设置单轮内部有符号PWM，调用前必须完成初始化。
 * @param motor MOTOR_LEFT或MOTOR_RIGHT。
 * @param speed_permille 限幅到-1000～1000；符号为内部轴方向，0选择Coast。
 */
void motor_driver_set_speed(motor_id_t motor, int16_t speed_permille);
/**
 * @brief 顺序设置双轮PWM；正常运动时应由速度环产生这些值。
 * @param left_speed_permille 左轴内部PWM千分比，范围-1000～1000。
 * @param right_speed_permille 右轴内部PWM千分比，范围-1000～1000。
 */
void motor_driver_set_speeds(int16_t left_speed_permille,
                             int16_t right_speed_permille);
/**
 * @brief 指定单轮滑行或主动制动，安全停车使用Brake。
 * @param motor MOTOR_LEFT或MOTOR_RIGHT。
 * @param mode MOTOR_STOP_COAST或MOTOR_STOP_BRAKE。
 */
void motor_driver_stop(motor_id_t motor, motor_stop_mode_t mode);
/**
 * @brief 为两路H桥设置相同停车电平，不执行等待或闭环计算。
 * @param mode MOTOR_STOP_COAST或MOTOR_STOP_BRAKE；要求TMR8已初始化。
 */
void motor_driver_stop_all(motor_stop_mode_t mode);

#endif
