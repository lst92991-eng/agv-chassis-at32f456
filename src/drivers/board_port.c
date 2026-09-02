/**
 * @file board_port.c
 * @brief 板级1ms时基和约100ms独立看门狗。
 *
 * 中断入口保留在官方命名的at32f45x_int.c，本模块维护启动前后连续的毫秒计数和WDT。
 * 独立看门狗监督主循环是否继续完成，传感器故障本身不要求停止喂狗。
 */
#include "board_port.h"

#include "at32f45x.h"
#include "at32f45x_wdt.h"

static volatile uint32_t board_milliseconds; /**< SysTick ISR唯一写者，主循环只读。 */

/** @brief 仅由SysTick中断每1ms调用，递增同一个毫秒计数，不执行控制逻辑。 */
void board_port_tick_1ms(void) { board_milliseconds++; }
/**
 * @brief 在系统时钟和板级延时初始化后配置启动阶段1ms SysTick中断。
 * @note 仅启动时清零；运行中再次调用会破坏协议截止时间和控制周期基准。
 *       FreeRTOS启动时以同一1ms周期接管硬件配置，不清除此处的毫秒计数。
 */
void board_port_time_init(void)
{
  board_milliseconds = 0U;
  (void)SysTick_Config(system_core_clock / 1000U);
}
/**
 * @brief 返回启动以来的uint32毫秒计数，供主循环计算实际经过时间。
 * @note 约49.7天自然回绕；短时间间隔使用uint32差值计算，不直接比较绝对大小。
 */
uint32_t board_port_now_ms(void) { return board_milliseconds; }

/**
 * @brief 配置并启动约100ms独立看门狗。
 * @return 1表示分频/重载寄存器更新完成并已启动，0表示配置等待超时。
 * @note /64分频、重载62给出名义约100ms，实际超时受内部低速时钟精度影响。
 *       更新等待有界且不下溢；必须在耗时较长的传感器启动校准之后调用。
 */
uint8_t board_port_watchdog_init(void)
{
  uint32_t timeout = 100000UL;
  wdt_register_write_enable(TRUE);
  wdt_divider_set(WDT_CLK_DIV_64);
  wdt_reload_value_set(62U);
  while(((wdt_flag_get(WDT_DIVF_UPDATE_FLAG) != RESET) ||
         (wdt_flag_get(WDT_RLDF_UPDATE_FLAG) != RESET)) && (timeout != 0U)) { timeout--; }
  wdt_register_write_enable(FALSE);
  if(timeout == 0U) return 0U;
  wdt_counter_reload();
  wdt_enable();
  return 1U;
}

/**
 * @brief 重载WDT，由应用层在每次完整主循环处理结束后调用。
 * @note 允许处于FAULT但主循环仍健康运行时喂狗，保留通信诊断与显式复位能力；
 *       不是仅当全部传感器健康才喂狗，也不应在中断内无条件喂狗。
 */
void board_port_watchdog_feed(void) { wdt_counter_reload(); }
