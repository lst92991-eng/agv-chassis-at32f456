/**
  **************************************************************************
  * @file     at32f45x_clock.c
  * @brief    system clock config program
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

/* includes ------------------------------------------------------------------*/
#include "at32f45x_clock.h"

/**
  * @brief  启动阶段配置12MHz外部晶振和PLL，得到192MHz系统时钟。
  * @note   必须在外设、SysTick和延时函数初始化前调用。原厂启动流程等待晶振、
  *         PLL和时钟切换就绪，这些等待没有软件超时，不能当作运行期恢复接口。
  *         此函数保留原厂时钟配置；主频改变时须同时核对CAN、PWM和SPI分频。
  *         时钟关系如下：
  *         - system clock        = (hext * pll_ns)/(pll_ms * pll_fp)
  *         - system clock source = pll (hext)
  *         - hext                = 12000000
  *         - sclk                = 192000000
  *         - ahbdiv              = 1
  *         - ahbclk              = 192000000
  *         - apb2div             = 1
  *         - apb2clk             = 192000000
  *         - apb1div             = 1
  *         - apb1clk             = 192000000
  *         - apb3div             = 4
  *         - apb3clk             = 48000000
  *         - pll_ns              = 64
  *         - pll_ms              = 1
  *         - pll_fp              = 4
  * @param  无。
  * @retval 无；完成后更新system_core_clock，供SysTick等外设计算周期。
  */
void system_clock_config(void)
{
  /* 启动时恢复CRM默认状态，避免继承调试环境留下的时钟配置。 */
  crm_reset();

  /* 升频前先设置Flash等待周期和内核供电，满足192MHz运行条件。 */
  flash_psr_set(FLASH_WAIT_CYCLE_5);

    /* 开启PWC访问时钟，以便配置内部LDO输出。 */
  crm_periph_clock_enable(CRM_PWC_PERIPH_CLOCK, TRUE);
 
  /* 内核LDO设为1.3V；修改主频时需按芯片手册联合核对。 */
  pwc_ldo_output_voltage_set(PWC_LDO_OUTPUT_1V3);

  crm_clock_source_enable(CRM_CLOCK_SOURCE_HEXT, TRUE);

  /* 等待板载12MHz晶振稳定；此启动等待无软件超时。 */
  while(crm_hext_stable_wait() == ERROR)
  {
  }

  /* PLL系统输出：12MHz × 64 / 1 / 4 = 192MHz。 */
  crm_pll_config(CRM_PLL_SOURCE_HEXT, 64, 1, CRM_PLL_FP_4);

  /* PLLU独立分频为/16，对应48MHz输出。 */
  crm_pllu_div_set(CRM_PLL_FU_16);

  /* 开启PLLU输出，不代表此处启用USB业务。 */
  crm_pllu_output_set(TRUE);

  /* 参数稳定后启动PLL，再等待锁定。 */
  crm_clock_source_enable(CRM_CLOCK_SOURCE_PLL, TRUE);

  /* PLL未锁定前不可切为系统主时钟；此启动等待无软件超时。 */
  while(crm_flag_get(CRM_PLL_STABLE_FLAG) != SET)
  {
  }

  /* AHB保持192MHz，供CPU和相关高速外设使用。 */
  crm_ahb_div_set(CRM_AHB_DIV_1);

  /* APB3四分频到48MHz，低于其90MHz上限。 */
  crm_apb3_div_set(CRM_APB3_DIV_4);

  /* APB2保持192MHz；定时器/PWM频率需按实际外设时钟换算。 */
  crm_apb2_div_set(CRM_APB2_DIV_1);

  /* APB1保持192MHz；SPI等驱动在此基础上使用各自分频。 */
  crm_apb1_div_set(CRM_APB1_DIV_1);

  /* 升频切换期间开启自动步进，减小频率突变带来的供电冲击。 */
  crm_auto_step_mode_enable(TRUE);

  /* 选择已锁定的PLL作为系统时钟。 */
  crm_sysclk_switch(CRM_SCLK_PLL);

  /* 以硬件状态确认切换完成，再关闭自动步进模式。 */
  while(crm_sysclk_switch_status_get() != CRM_SCLK_PLL)
  {
  }

  /* 主频稳定后恢复普通运行模式。 */
  crm_auto_step_mode_enable(FALSE);

  /* 同步软件记录的主频，保证后续SysTick和延时初始化使用192MHz。 */
  system_core_clock_update();
}
