/**
 * @file bumper.c
 * @brief 四路低有效防撞条的EXINT采样、事件锁存和释放确认。
 *
 * 引脚由板上外部电路确定空闲电平，内部不启用上下拉。ISR记录采到的按下位，
 * 主循环合并实时电平和ISR事件，因此在两次控制周期之间已释放的按下事件也
 * 可以保留。ISR必须在低电平持续期间采到输入，不能保证捕获任意宽度短脉冲。
 * 本模块不操作电机；应用层读取锁存位后制动并锁存故障，释放不会自动恢复运动。
 */
#include "bumper.h"

#include "at32f45x.h"
#include "at32f45x_crm.h"
#include "at32f45x_exint.h"
#include "at32f45x_gpio.h"
#include "at32f45x_misc.h"
#include "at32f45x_scfg.h"

/* ISR置位、主循环临界区取走的按下事件；volatile用于异步可见性。 */
static volatile uint8_t bumper_irq_latch;
/* 主循环保存的历史碰撞位；只有显式bumper_clear()才会清零。 */
static uint8_t bumper_latch;
/* 连续无按下事件的更新次数，饱和到10；正常10ms更新时约为100ms。 */
static uint8_t bumper_release_ticks;

/**
 * @brief 读取四个低有效GPIO，转换成统一的防撞方向位掩码。
 * @return BUMPER_FRONT/REAR/LEFT/RIGHT的按位或，0表示当前全部释放。
 * ISR和主循环均可调用；仅采样当前电平，不清除历史事件，也不等待消抖。
 */
static uint8_t bumper_read(void)
{
  uint8_t mask = 0U;
  if(gpio_input_data_bit_read(GPIOC, GPIO_PINS_15) == RESET) mask |= BUMPER_FRONT;
  if(gpio_input_data_bit_read(GPIOC, GPIO_PINS_13) == RESET) mask |= BUMPER_REAR;
  if(gpio_input_data_bit_read(GPIOC, GPIO_PINS_14) == RESET) mask |= BUMPER_LEFT;
  if(gpio_input_data_bit_read(GPIOB, GPIO_PINS_9) == RESET) mask |= BUMPER_RIGHT;
  return mask;
}

/**
 * @brief 初始化一路防撞EXINT为双边沿触发，并清除原有挂起标志。
 * @param line 已完成端口映射的EXINT线掩码。
 * 下降沿可捕获按下，释放边沿也会进入ISR，但只有采到的低电平会被锁存。
 */
static void bumper_line(uint32_t line)
{
  exint_init_type exti;
  exint_default_para_init(&exti);
  exti.line_enable = TRUE;
  exti.line_mode = EXINT_LINE_INTERRUPT;
  exti.line_select = line;
  exti.line_polarity = EXINT_TRIGGER_BOTH_EDGE;
  exint_init(&exti);
  exint_flag_clear(line);
}

/**
 * @brief 上电配置四路防撞输入、EXINT和初始锁存状态。
 *
 * PC15/PC13/PC14/PB9分别对应前/后/左/右。初始化时采样输入，如果上电
 * 已按下则立即保留对应位。如果上电全部释放，则释放计数直接置为10，首次
 * 使能无需再等待100ms；发生按下事件后的释放才重新累计十个更新周期。
 */
void bumper_init(void)
{
  gpio_init_type gpio;
  crm_periph_clock_enable(CRM_GPIOB_PERIPH_CLOCK, TRUE);
  crm_periph_clock_enable(CRM_GPIOC_PERIPH_CLOCK, TRUE);
  crm_periph_clock_enable(CRM_SCFG_PERIPH_CLOCK, TRUE);
  gpio_default_para_init(&gpio);
  gpio.gpio_mode = GPIO_MODE_INPUT;
  gpio.gpio_pull = GPIO_PULL_NONE;
  gpio.gpio_pins = GPIO_PINS_9;
  gpio_init(GPIOB, &gpio);
  gpio.gpio_pins = GPIO_PINS_13 | GPIO_PINS_14 | GPIO_PINS_15;
  gpio_init(GPIOC, &gpio);
  scfg_exint_line_config(SCFG_PORT_SOURCE_GPIOB, SCFG_PINS_SOURCE9);
  scfg_exint_line_config(SCFG_PORT_SOURCE_GPIOC, SCFG_PINS_SOURCE13);
  scfg_exint_line_config(SCFG_PORT_SOURCE_GPIOC, SCFG_PINS_SOURCE14);
  scfg_exint_line_config(SCFG_PORT_SOURCE_GPIOC, SCFG_PINS_SOURCE15);
  bumper_line(EXINT_LINE_9);
  bumper_line(EXINT_LINE_13);
  bumper_line(EXINT_LINE_14);
  bumper_line(EXINT_LINE_15);
  bumper_irq_latch = bumper_latch = bumper_read();
  bumper_release_ticks = (bumper_latch == 0U) ? 10U : 0U;
  nvic_irq_enable(EXINT9_5_IRQn, 2U, 1U);
  nvic_irq_enable(EXINT15_10_IRQn, 2U, 2U);
}

/**
 * @brief 由EXINT ISR记录本次变化线路中仍处于按下状态的方向位。
 * @param changed_mask 外层ISR确认的线路集合，使用BUMPER_*位掩码。
 * 本函数只累积事件，不清EXINT标志、不执行制动；释放边沿不会清除按下历史。
 */
void bumper_edge_irq(uint8_t changed_mask)
{
  bumper_irq_latch |= bumper_read() & changed_mask;
}

/**
 * @brief 在正常10ms主循环中合并ISR事件和当前输入，并累计释放时间。
 *
 * 用短关中断区取走并清空ISR邮箱，避免事件在这个取走过程被覆盖。遇到任何
 * 当前按下或已采到的按下事件，便将其加入历史锁存并重新开始释放计数。
 * 释放门以十次调用计数，主循环变慢时确认时间也延长；本函数不清历史锁存。
 */
void bumper_update(void)
{
  uint32_t irq = __get_PRIMASK();
  uint8_t edge;
  uint8_t active = bumper_read();
  __disable_irq();
  edge = bumper_irq_latch;
  bumper_irq_latch = 0U;
  if(irq == 0U) __enable_irq();
  if((active | edge) != 0U)
  {
    bumper_latch |= active | edge;
    bumper_release_ticks = 0U;
  }
  else if(bumper_release_ticks < 10U) bumper_release_ticks++;
}

/**
 * @brief 供主循环读取自上次显式清除以来的碰撞方向。
 * @return 历史按下位掩码；全部释放后仍保留，直到bumper_clear()成功。
 */
uint8_t bumper_latched(void) { return bumper_latch; }
/**
 * @brief 检查当前输入已释放，且释放计数达到门限。
 * @return 1表示当前采样与计数满足清除条件，否则0。
 * 这是主循环的一次条件检查，不是对随后新中断的屏蔽或保证。
 */
uint8_t bumper_can_clear(void) { return (bumper_read() == 0U) && (bumper_release_ticks >= 10U); }
/**
 * @brief 在应用显式复位故障时尝试清除历史碰撞位。
 * @return 1表示通过释放检查并清除bumper_latch，0表示仍不允许清除。
 * 不清ISR邮箱、不直接清应用故障或恢复电机。检查和赋值未用临界区包围，
 * 它不是与新边沿互斥的原子复位；后续bumper_update()仍负责接收新事件。
 */
uint8_t bumper_clear(void)
{
  if(bumper_can_clear() == 0U) return 0U;
  bumper_latch = 0U;
  return 1U;
}
