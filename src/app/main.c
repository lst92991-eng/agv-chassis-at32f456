/**
  **************************************************************************
  * @file     main.c
  * @brief    主程序入口
  *
 * main.c只负责正式底盘固件的硬件初始化和FreeRTOS调度启动。
  **************************************************************************
  */

#include "at32f45x_board.h"
#include "at32f45x_clock.h"
#include "chassis_app.h"
#include "app_freertos.h"
#include "motor_driver.h"
#include "board_port.h"

/**
 * @brief 完成板级初始化，创建唯一底盘任务并启动FreeRTOS。
 *
 * 毫秒时基在传感器启动前已工作；初始化顺序和默认制动状态不变。
 * 后续10ms调度由app_freertos承担，控制算法仍使用同一板级真实毫秒时间。
 */
int main(void)
{
  /* 配置系统时钟。当前工程按板载 12MHz 外部晶振配置。 */
  system_clock_config();

  /* 所有外设中断使用同一组抢占优先级配置。 */
  nvic_priority_group_config(NVIC_PRIORITY_GROUP_4);

  /* 初始化板级外设，例如延时函数和基础 GPIO。 */
  at32_board_init();
  /* 板级DWT用于外设微秒延时；独立SysTick用于跨模块毫秒计时。 */
  board_port_time_init();

  /* 先完成原有底盘初始化，再创建静态任务；创建后立即启动调度，不插入耗时等待。 */
  if((chassis_app_init() == 0U) || (app_freertos_init() == 0U))
  {
    motor_driver_stop_all(MOTOR_STOP_BRAKE);
    for(;;) {}
  }

  app_freertos_start();
}
