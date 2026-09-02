/**
 * @file motor_driver.c
 * @brief AT8236双路H桥的20kHz双PWM底层驱动。
 *
 * 输入是速度环给出的有符号PWM千分比，不是mm/s或RPM。上层先按左右轴的
 * 标定符号换算目标，本层还对右桥反相，因此不能将这里的正PWM直接解释为
 * 两个车轮都向车体前方转动。实际方向由整条标定和接线链共同决定。
 * PC6/PC7连接左桥IN1/IN2，PC8/PC9连接右桥IN1/IN2；TMR8提供共同PWM周期。
 * 正常运动由速度环调用本层；本层也供应用停车和CPU异常制动使用。
 */

#include "motor_driver.h"

#include "at32f45x_crm.h"
#include "at32f45x_gpio.h"
#include "at32f45x_tmr.h"

#define MOTOR_PWM_HZ         20000UL
#define MOTOR_PWM_MAX            1000

/* TMR8自动重装值ARR；一个PWM周期含ARR+1个计数，初始化后保持不变。 */
static uint32_t motor_pwm_period;

/**
 * @brief 将电机编号和H桥输入脚映射到TMR8比较通道。
 * @param motor 已由调用者检查的MOTOR_LEFT或MOTOR_RIGHT。
 * @param in2 0选择IN1，非0选择IN2。
 * @return 左桥使用CH1/CH2，右桥使用CH3/CH4。
 */
static tmr_channel_select_type motor_channel(motor_id_t motor, uint8_t in2)
{
  if(motor == MOTOR_LEFT)
    return (in2 != 0U) ? TMR_SELECT_CHANNEL_2 : TMR_SELECT_CHANNEL_1;
  return (in2 != 0U) ? TMR_SELECT_CHANNEL_4 : TMR_SELECT_CHANNEL_3;
}

/**
 * @brief 将单个H桥输入的PWM占空比写入TMR8比较寄存器。
 *
 * @param channel 由motor_channel()获得的比较通道。
 * @param duty 千分比；0为持续低，1000为持续高，超过1000按1000处理。
 * 采用整数乘除量化占空比；通道预装载已开启，比较值按定时器更新机制生效。
 */
static void motor_pwm(tmr_channel_select_type channel, uint16_t duty)
{
  if(duty > MOTOR_PWM_MAX) duty = MOTOR_PWM_MAX;
  tmr_channel_value_set(TMR8, channel,
      ((motor_pwm_period + 1UL) * duty) / MOTOR_PWM_MAX);
}

/**
 * @brief 上电时配置PC6～PC9复用输出和TMR8四路20kHz PWM。
 *
 * 在开始正常控制前调用一次。PWM周期按当前APB2时钟计算，四通道共用计数器，
 * 比较值和周期均开启预装载。使能TMR8主输出前先写入Coast；上层完成本函数
 * 后会显式切换到Brake。本函数没有硬件初始化错误返回值。
 */
void motor_driver_init(void)
{
  crm_clocks_freq_type clocks = {0};
  gpio_init_type gpio;
  tmr_output_config_type output;
  static const tmr_channel_select_type channels[4] =
  {
    TMR_SELECT_CHANNEL_1, TMR_SELECT_CHANNEL_2,
    TMR_SELECT_CHANNEL_3, TMR_SELECT_CHANNEL_4
  };
  uint8_t channel;

  crm_periph_clock_enable(CRM_GPIOC_PERIPH_CLOCK, TRUE);
  crm_periph_clock_enable(CRM_TMR8_PERIPH_CLOCK, TRUE);
  gpio_default_para_init(&gpio);
  gpio.gpio_pins = GPIO_PINS_6 | GPIO_PINS_7 | GPIO_PINS_8 | GPIO_PINS_9;
  gpio.gpio_mode = GPIO_MODE_MUX;
  gpio.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
  gpio.gpio_pull = GPIO_PULL_DOWN;
  gpio.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
  gpio_init(GPIOC, &gpio);
  gpio_pin_mux_config(GPIOC, GPIO_PINS_SOURCE6, GPIO_MUX_3);
  gpio_pin_mux_config(GPIOC, GPIO_PINS_SOURCE7, GPIO_MUX_3);
  gpio_pin_mux_config(GPIOC, GPIO_PINS_SOURCE8, GPIO_MUX_3);
  gpio_pin_mux_config(GPIOC, GPIO_PINS_SOURCE9, GPIO_MUX_3);

  crm_clocks_freq_get(&clocks);
  /* 周期从APB2时钟派生，避免系统时钟调整后PWM频率静默漂移。 */
  motor_pwm_period = clocks.apb2_freq / MOTOR_PWM_HZ - 1UL;
  tmr_base_init(TMR8, motor_pwm_period, 0U);
  tmr_cnt_dir_set(TMR8, TMR_COUNT_UP);
  tmr_clock_source_div_set(TMR8, TMR_CLOCK_DIV1);
  tmr_output_default_para_init(&output);
  output.oc_mode = TMR_OUTPUT_CONTROL_PWM_MODE_A;
  output.oc_output_state = TRUE;
  output.oc_polarity = TMR_OUTPUT_ACTIVE_HIGH;
  output.oc_idle_state = FALSE;
  for(channel = 0U; channel < 4U; channel++)
  {
    tmr_output_channel_config(TMR8, channels[channel], &output);
    tmr_output_channel_buffer_enable(TMR8, channels[channel], TRUE);
  }
  tmr_period_buffer_enable(TMR8, TRUE);
  /* 使能定时器主输出前先写两个输入均为低的Coast值。 */
  motor_driver_stop_all(MOTOR_STOP_COAST);
  tmr_output_enable(TMR8, TRUE);
  tmr_counter_enable(TMR8, TRUE);
}

/**
 * @brief 把单轴速度环输出换算成AT8236的两路输入PWM。
 *
 * @param motor MOTOR_LEFT或MOTOR_RIGHT；其他编号不产生输出。
 * @param speed 内部轴方向下的有符号PWM千分比，限幅到-1000～1000。
 * 右桥在本函数内反相，左桥保持输入符号。速度和电流闭环不在本层执行。
 * speed为0时选择Coast；需要安全停车时，调用者必须显式调用Brake接口。
 */
void motor_driver_set_speed(motor_id_t motor, int16_t speed)
{
  uint16_t duty;

  if((motor != MOTOR_LEFT) && (motor != MOTOR_RIGHT)) return;
  if(speed > MOTOR_PWM_MAX) speed = MOTOR_PWM_MAX;
  if(speed < -MOTOR_PWM_MAX) speed = -MOTOR_PWM_MAX;
  if(motor == MOTOR_RIGHT) speed = (int16_t)-speed;
  if(speed == 0)
  {
    motor_driver_stop(motor, MOTOR_STOP_COAST);
    return;
  }
  duty = (uint16_t)((speed > 0) ? speed : -speed);
  /*
   * 慢衰减双PWM：一侧常高，另一侧占空比为1-duty。两输入不同的时段驱动
   * 电机；两输入均高的时段短接制动/续流。交换常高端即改变驱动极性。
   */
  motor_pwm(motor_channel(motor, 0U),
      (speed > 0) ? MOTOR_PWM_MAX : (uint16_t)(MOTOR_PWM_MAX - duty));
  motor_pwm(motor_channel(motor, 1U),
      (speed > 0) ? (uint16_t)(MOTOR_PWM_MAX - duty) : MOTOR_PWM_MAX);
}

/**
 * @brief 依次设置左右H桥的有符号PWM，复用单轮限幅和方向映射。
 * @param left 左轴内部有符号PWM千分比，范围-1000～1000。
 * @param right 右轴内部有符号PWM千分比，范围-1000～1000。
 * 两次寄存器设置顺序执行，不在此接口内形成软件原子更新。
 */
void motor_driver_set_speeds(int16_t left, int16_t right)
{
  motor_driver_set_speed(MOTOR_LEFT, left);
  motor_driver_set_speed(MOTOR_RIGHT, right);
}

/**
 * @brief 用固定输入电平使一侧H桥滑行或主动制动。
 * @param motor MOTOR_LEFT或MOTOR_RIGHT；其他编号直接返回。
 * @param mode MOTOR_STOP_BRAKE使IN1/IN2均高；其他值使两输入均低。
 * Brake用于本地故障/正常停车路径，Coast释放驱动；二者电气效果不同。
 */
void motor_driver_stop(motor_id_t motor, motor_stop_mode_t mode)
{
  uint16_t duty;
  if((motor != MOTOR_LEFT) && (motor != MOTOR_RIGHT)) return;
  /* 两输入同时高为主动制动，同时低为滑行。 */
  duty = (mode == MOTOR_STOP_BRAKE) ? MOTOR_PWM_MAX : 0U;
  motor_pwm(motor_channel(motor, 0U), duty);
  motor_pwm(motor_channel(motor, 1U), duty);
}

/**
 * @brief 按左、右顺序将两侧H桥设置为同一种停车模式。
 * @param mode MOTOR_STOP_BRAKE为主动制动，MOTOR_STOP_COAST为滑行。
 * 应用停车及CPU异常路径可调用；无等待和闭环计算，前提是TMR8已初始化。
 */
void motor_driver_stop_all(motor_stop_mode_t mode)
{
  motor_driver_stop(MOTOR_LEFT, mode);
  motor_driver_stop(MOTOR_RIGHT, mode);
}
