/** @file board_port.h @brief 板级毫秒时基和独立硬件看门狗；外设中断入口位于at32f45x_int.c。 */
#ifndef BOARD_PORT_H
#define BOARD_PORT_H

#include <stdint.h>

/** @brief 系统时钟配置完成后启动1ms SysTick；仅启动时调用，清零毫秒计数。 */
void board_port_time_init(void);
/** @brief 仅供SysTick_Handler每1ms调用一次，更新板级毫秒计数。 */
void board_port_tick_1ms(void);
/** @brief 读取uint32毫秒计数；调用方以无符号差值计算实际周期和超时。 */
uint32_t board_port_now_ms(void);
/** @brief 耗时启动步骤完成后开启约100ms硬件WDT；成功返回1，配置超时返回0。 */
uint8_t board_port_watchdog_init(void);
/**
 * @brief 完整主循环结束时喂狗；FAULT状态下仍可调用以维持诊断和恢复流程。
 * @note 主循环卡死或CPU异常无法调用时，由硬件WDT触发复位。
 */
void board_port_watchdog_feed(void);

#endif
