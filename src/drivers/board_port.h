/** @file board_port.h @brief 板级毫秒时基；外设中断位于at32f45x_int.c。 */
#ifndef BOARD_PORT_H
#define BOARD_PORT_H
#include <stdint.h>
void board_port_time_init(void);
void board_port_tick_1ms(void);
uint32_t board_port_now_ms(void);
#endif
