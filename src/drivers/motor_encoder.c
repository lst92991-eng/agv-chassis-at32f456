/**
 * @file motor_encoder.c
 * @brief 左右AB编码器双边沿正交解码与异常跳变统计。
 *
 * 左轮PA6/PA7、右轮PC4/PC5依赖板上外部10kΩ上拉到5V，MCU内部不上下拉。
 * A/B两相的上升沿、下降沿均参与解码，一次合法跳变对应一个计数。引脚必须
 * 满足板级5V接口条件，不能将这份配置直接套用到不耐5V的其他GPIO。
 * 边沿在EXINT ISR中解码；主循环读取累计计数，速度/RPM和毫米换算由上层完成。
 * 右轴先在此层反相，上层仍有反馈及车体坐标符号标定，计数不直接代表前进距离。
 */

#include "motor_encoder.h"

#include <string.h>

#include "at32f45x.h"
#include "at32f45x_crm.h"
#include "at32f45x_exint.h"
#include "at32f45x_gpio.h"
#include "at32f45x_misc.h"
#include "at32f45x_scfg.h"

/** 一个物理编码器的ISR状态；count/invalid允许主循环异步读取。 */
typedef struct
{
  volatile int32_t count;    /**< 应用方向标定前的累计有符号边沿数，不是圈数。 */
  volatile uint32_t invalid; /**< A/B两位同时跳变的累计次数，不推测丢失脉冲数。 */
  uint8_t state;             /**< 上一次采样的AB状态：A放bit1，B放bit0。 */
  int8_t direction;          /**< 本层轴向因子：左轴+1，右轴-1。 */
} motor_encoder_axis_t;

/* 索引=(旧AB<<2)|新AB；合法Gray码相邻状态返回+1/-1，其余返回0。 */
static const int8_t motor_encoder_step[16] =
{
   0,  1, -1,  0, -1,  0,  0,  1,
   1,  0,  0, -1,  0, -1,  1,  0
};
static motor_encoder_axis_t motor_encoder_axis[2];

/**
 * @brief 采样指定编码器的A/B电平，形成正交解码的二位状态。
 * @param motor 内部调用保证为MOTOR_LEFT或MOTOR_RIGHT。
 * @return bit1为A相、bit0为B相，范围0～3。
 * 上电读取用于建立初值；ISR读取用于与上一状态比较，不对电平做延时消抖。
 */
static uint8_t motor_encoder_read(motor_id_t motor)
{
  gpio_type *port = (motor == MOTOR_LEFT) ? GPIOA : GPIOC;
  uint16_t a = (motor == MOTOR_LEFT) ? GPIO_PINS_6 : GPIO_PINS_4;
  uint16_t b = (motor == MOTOR_LEFT) ? GPIO_PINS_7 : GPIO_PINS_5;
  return (uint8_t)(((gpio_input_data_bit_read(port, a) != RESET) ? 2U : 0U) |
                   ((gpio_input_data_bit_read(port, b) != RESET) ? 1U : 0U));
}

/**
 * @brief 将一个编码器相位接入EXINT双边沿中断。
 * @param line 已完成GPIO端口映射的EXINT线掩码。
 * 仅初始化时调用；配置后清除挂起标志，NVIC在所有轴状态就绪后再使能。
 */
static void motor_encoder_line(uint32_t line)
{
  exint_init_type config;
  exint_default_para_init(&config);
  config.line_enable = TRUE;
  config.line_mode = EXINT_LINE_INTERRUPT;
  config.line_select = line;
  config.line_polarity = EXINT_TRIGGER_BOTH_EDGE;
  exint_init(&config);
  exint_flag_clear(line);
}

/**
 * @brief 上电建立双编码器输入、电气方向约定和ISR初始状态。
 *
 * 输入采用外部上拉，四相均使用双边沿EXINT；右轴方向系数为-1。使能NVIC前
 * 先清空累计值并采样当前AB状态，避免把上电时的初始电平当作一次有效移动。
 * 函数只在控制初始化阶段调用，没有硬件错误返回值。
 */
void motor_encoder_init(void)
{
  gpio_init_type gpio;

  crm_periph_clock_enable(CRM_GPIOA_PERIPH_CLOCK, TRUE);
  crm_periph_clock_enable(CRM_GPIOC_PERIPH_CLOCK, TRUE);
  crm_periph_clock_enable(CRM_SCFG_PERIPH_CLOCK, TRUE);
  gpio_default_para_init(&gpio);
  gpio.gpio_mode = GPIO_MODE_INPUT;
  /* 外部上拉已定义电平，内部上下拉会改变5V接口的边沿特性。 */
  gpio.gpio_pull = GPIO_PULL_NONE;
  gpio.gpio_pins = GPIO_PINS_4 | GPIO_PINS_5;
  gpio_init(GPIOC, &gpio);
  gpio.gpio_pins = GPIO_PINS_6 | GPIO_PINS_7;
  gpio_init(GPIOA, &gpio);
  scfg_exint_line_config(SCFG_PORT_SOURCE_GPIOC, SCFG_PINS_SOURCE4);
  scfg_exint_line_config(SCFG_PORT_SOURCE_GPIOC, SCFG_PINS_SOURCE5);
  scfg_exint_line_config(SCFG_PORT_SOURCE_GPIOA, SCFG_PINS_SOURCE6);
  scfg_exint_line_config(SCFG_PORT_SOURCE_GPIOA, SCFG_PINS_SOURCE7);
  motor_encoder_line(EXINT_LINE_4);
  motor_encoder_line(EXINT_LINE_5);
  motor_encoder_line(EXINT_LINE_6);
  motor_encoder_line(EXINT_LINE_7);
  memset(motor_encoder_axis, 0, sizeof(motor_encoder_axis));
  motor_encoder_axis[MOTOR_LEFT].state = motor_encoder_read(MOTOR_LEFT);
  motor_encoder_axis[MOTOR_RIGHT].state = motor_encoder_read(MOTOR_RIGHT);
  motor_encoder_axis[MOTOR_LEFT].direction = 1;
  motor_encoder_axis[MOTOR_RIGHT].direction = -1;
  nvic_irq_enable(EXINT4_IRQn, 2U, 0U);
  nvic_irq_enable(EXINT9_5_IRQn, 2U, 1U);
}

/**
 * @brief 在编码器ISR内按AB状态转换表累计单轮计数。
 * @param motor 内部已确定的左轮或右轮编号。
 *
 * 合法相邻转换累计+1/-1并应用轴向因子；两位同时变化只记invalid，不猜测
 * 丢失方向。即使异常也更新state，使下一次边沿从实际读到的电平继续解码。
 * 本函数不清EXINT标志，外层at32f45x_int.c中断入口负责确认和清除相应标志。
 */
static void motor_encoder_edge(motor_id_t motor)
{
  motor_encoder_axis_t *axis = &motor_encoder_axis[motor];
  uint8_t state = motor_encoder_read(motor);
  int8_t step = motor_encoder_step[(axis->state << 2) | state];
  /* 两位同时变化不是合法Gray码相邻状态，计数保持但记录诊断。 */
  if((step == 0) && (state != axis->state)) axis->invalid++;
  axis->count += step * axis->direction;
  axis->state = state;
}

/**
 * @brief 左轮A相或B相EXINT确认后调用的统一解码入口。
 * 仅供ISR调用；同一中断中两个相位都挂起时，外层仍只按当前AB状态解码一次。
 */
void motor_encoder_left_edge_handler(void) { motor_encoder_edge(MOTOR_LEFT); }
/**
 * @brief 右轮A相或B相EXINT确认后调用的统一解码入口。
 * 仅供ISR调用；计数方向由右轴direction=-1统一翻转。
 */
void motor_encoder_right_edge_handler(void) { motor_encoder_edge(MOTOR_RIGHT); }

/**
 * @brief 读取一个轴的累计有符号边沿数，供主循环求差分。
 * @param motor 调用者应传MOTOR_LEFT或MOTOR_RIGHT。
 * @return 当前计数快照；大于MOTOR_RIGHT的编号返回0。
 * 对齐32位读不会拆成多个半字，但分开读取左右轮不构成同一时刻的成对快照。
 */
int32_t motor_encoder_get_count(motor_id_t motor)
{
  return (motor <= MOTOR_RIGHT) ? motor_encoder_axis[motor].count : 0;
}

/**
 * @brief 读取单轴非法AB转换累计数，用于诊断电气毛刺或采样漏边沿。
 * @param motor 调用者应传MOTOR_LEFT或MOTOR_RIGHT。
 * @return 累计次数；大于MOTOR_RIGHT的编号返回0。本接口不会清除统计值。
 */
uint32_t motor_encoder_get_invalid_transition_count(motor_id_t motor)
{
  return (motor <= MOTOR_RIGHT) ? motor_encoder_axis[motor].invalid : 0U;
}

/**
 * @brief 在短临界区内清零两轮计数和非法跳变统计。
 *
 * 只在停车/里程计复位路径调用；保留AB历史状态和轴向因子。先保存PRIMASK，
 * 结束时仅恢复原有的开中断状态，不会擅自打开调用者已关闭的中断。
 * 调用者还需同步速度环的上一计数和里程计原点，否则清零会被误解为反向位移。
 */
void motor_encoder_reset_all(void)
{
  uint32_t irq = __get_PRIMASK();
  __disable_irq();
  /* count与invalid由ISR写入，成组清零期间必须屏蔽对应边沿。 */
  motor_encoder_axis[MOTOR_LEFT].count = 0;
  motor_encoder_axis[MOTOR_RIGHT].count = 0;
  motor_encoder_axis[MOTOR_LEFT].invalid = 0U;
  motor_encoder_axis[MOTOR_RIGHT].invalid = 0U;
  if(irq == 0U) __enable_irq();
}
