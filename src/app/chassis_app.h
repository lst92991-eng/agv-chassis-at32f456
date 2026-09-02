/**
 * @file chassis_app.h
 * @brief 唯一底盘状态机，由main初始化、FreeRTOS控制任务周期调用。
 *
 * 正常PWM输出由本模块组合drive_control执行；中断不运行本状态机。
 */
#ifndef CHASSIS_APP_H
#define CHASSIS_APP_H

#include <stdint.h>

/**
 * @brief 初始化底盘、输入、CAN和WDT，默认禁用运动。
 * @return 1=WDT配置成功，可以启动控制任务；传感器失败由FAULT状态另外表达。
 *         0=WDT配置失败，调用方应持续制动。
 * @note 调用前须完成系统时钟、板级延时以及board_port_time_init。
 */
uint8_t chassis_app_init(void);
/**
 * @brief 执行一次完整底盘循环；FreeRTOS任务按约10ms调度，内部使用真实经过时间。
 * @note 只能由单一控制任务调用，不可重入；即使DISABLED/HOLD/FAULT仍须持续调用。
 */
void chassis_app_step(void);

#endif
