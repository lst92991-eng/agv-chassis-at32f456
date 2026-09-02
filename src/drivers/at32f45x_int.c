/**
  **************************************************************************
  * @file     at32f45x_int.c
  * @brief    main interrupt service routines.
  **************************************************************************
  *
  * Copyright (c) 2025, Artery Technology, All rights reserved.
  *
  * The software Board Support Package (BSP) that is made available to
  * download from Artery official website is the copyrighted work of Artery.
  * Artery authorizes customers to use, copy, and distribute the BSP
  * software and its related documentation for the purpose of design and
  * development in conjunction with Artery microcontrollers. Use of the
  * software is governed by this copyright notice and the following disclaimer.
  *
  * THIS SOFTWARE IS PROVIDED ON "AS IS" BASIS WITHOUT WARRANTIES,
  * GUARANTEES OR REPRESENTATIONS OF ANY KIND. ARTERY EXPRESSLY DISCLAIMS,
  * TO THE FULLEST EXTENT PERMITTED BY LAW, ALL EXPRESS, IMPLIED OR
  * STATUTORY OR OTHER WARRANTIES, GUARANTEES OR REPRESENTATIONS,
  * INCLUDING BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY,
  * FITNESS FOR A PARTICULAR PURPOSE, OR NON-INFRINGEMENT.
  *
  **************************************************************************
  */

/*
 * 保留官方中断文件名与版权头。以下项目ISR从board_port.c原样迁回：
 * 普通外设中断只采样/投递，CPU异常保持原主动制动路径。
 * SVC/PendSV由FreeRTOS官方移植层提供强入口，DebugMon继续使用启动文件弱默认。
 */
#include "at32f45x_int.h"

#include "at32f45x_exint.h"
#include "board_port.h"
#include "FreeRTOS.h"
#include "task.h"
#include "adc_dma.h"
#include "bumper.h"
#include "canfd_driver.h"
#include "motor_driver.h"
#include "motor_encoder.h"

/* 不将此名字映射为SysTick_Handler，避免覆盖官方中断文件中的共享时基入口。 */
void xPortSysTickHandler(void);

/** @brief 先推进板级毫秒时基；调度器启动后再推进OS节拍，启动前不得访问内核延时链表。 */
void SysTick_Handler(void)
{
  board_port_tick_1ms();
  if(xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED) xPortSysTickHandler();
}

/**
 * @brief CPU异常共用收尾：屏蔽中断、双电机主动制动，然后永久停留。
 * @note 异常上下文不走协议或业务状态机；屏障保证制动寄存器写入后再停留。
 *       已启用的硬件看门狗会因无法继续喂狗而复位；若尚未启用则保持停留。
 */
static void board_safe_halt(void)
{
  __disable_irq();
  motor_driver_stop_all(MOTOR_STOP_BRAKE);
  __DSB();
  __ISB();
  for(;;) {}
}

/** @brief 不可屏蔽异常入口，直接转入制动停留。 */
void NMI_Handler(void) { board_safe_halt(); }
/** @brief 硬故障入口，直接转入制动停留。 */
void HardFault_Handler(void) { board_safe_halt(); }
/** @brief 内存访问管理异常入口，直接转入制动停留。 */
void MemManage_Handler(void) { board_safe_halt(); }
/** @brief 总线访问异常入口，直接转入制动停留。 */
void BusFault_Handler(void) { board_safe_halt(); }
/** @brief 非法指令等用法异常入口，直接转入制动停留。 */
void UsageFault_Handler(void) { board_safe_halt(); }

/** @brief CAN3接收ISR，把硬件帧交给驱动缓冲；此处不执行运动指令。 */
void CAN3_RX_IRQHandler(void) { canfd_driver_rx_irq(); }
/** @brief ADC搬运完成/错误ISR，由ADC驱动发布三通道快照状态。 */
void DMA1_Channel1_IRQHandler(void) { adc_dma_irq(); }

/** @brief 右编码器PC4的EXINT4入口，清事件标志后采样AB相位。 */
void EXINT4_IRQHandler(void)
{
  if(exint_interrupt_flag_get(EXINT_LINE_4) != RESET)
  {
    exint_flag_clear(EXINT_LINE_4);
    motor_encoder_right_edge_handler();
  }
}

/**
 * @brief 分发共享EXINT5～9：右编码器PC5、左编码器PA6/PA7和右防撞条。
 * @note 同次ISR内同一编码器只采样一次AB相位；防撞条只记录输入事件，
 *       后续制动由主循环消费锁存状态。
 */
void EXINT9_5_IRQHandler(void)
{
  uint8_t left = 0U;
  uint8_t right = 0U;
  if(exint_interrupt_flag_get(EXINT_LINE_5) != RESET)
  {
    exint_flag_clear(EXINT_LINE_5);
    right = 1U;
  }
  if(exint_interrupt_flag_get(EXINT_LINE_6) != RESET)
  {
    exint_flag_clear(EXINT_LINE_6);
    left = 1U;
  }
  if(exint_interrupt_flag_get(EXINT_LINE_7) != RESET)
  {
    exint_flag_clear(EXINT_LINE_7);
    left = 1U;
  }
  if(exint_interrupt_flag_get(EXINT_LINE_9) != RESET)
  {
    exint_flag_clear(EXINT_LINE_9);
    bumper_edge_irq(BUMPER_RIGHT);
  }
  if(left != 0U) motor_encoder_left_edge_handler();
  if(right != 0U) motor_encoder_right_edge_handler();
}

/**
 * @brief 收集后/左/前三路防撞条边沿，合并位掩码后一次交给输入驱动。
 * @note 共享中断中逐路清标志，避免漏掉同一时刻触发的多路输入。
 */
void EXINT15_10_IRQHandler(void)
{
  uint8_t changed = 0U;
  if(exint_interrupt_flag_get(EXINT_LINE_13) != RESET)
  {
    exint_flag_clear(EXINT_LINE_13);
    changed |= BUMPER_REAR;
  }
  if(exint_interrupt_flag_get(EXINT_LINE_14) != RESET)
  {
    exint_flag_clear(EXINT_LINE_14);
    changed |= BUMPER_LEFT;
  }
  if(exint_interrupt_flag_get(EXINT_LINE_15) != RESET)
  {
    exint_flag_clear(EXINT_LINE_15);
    changed |= BUMPER_FRONT;
  }
  if(changed != 0U) bumper_edge_irq(changed);
}
