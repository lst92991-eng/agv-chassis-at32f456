/** @file board_port.c @brief 板级1ms时基，不启动硬件看门狗。 */
#include "board_port.h"
#include "at32f45x.h"
static volatile uint32_t board_milliseconds;
void board_port_tick_1ms(void) { board_milliseconds++; }
void board_port_time_init(void)
{
  board_milliseconds = 0U;
  (void)SysTick_Config(system_core_clock / 1000U);
}
uint32_t board_port_now_ms(void) { return board_milliseconds; }
