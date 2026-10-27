/**
 * @file adc_dma.c
 * @brief ADC1扫描PA3/PA4/PA5，并由DMA一次搬运完整三通道样本。
 *
 * 普通通道序列由主循环约每20ms软件触发一次，DMA搬运期间CPU无需等待。
 * DMA环形模式复用三元素缓冲区，并不意味着ADC连续扫描或由硬件定时器触发。
 * 驱动输出12位原始值和批次序号；电压/电流换算、零点和保护由safety服务完成。
 */
#include "adc_dma.h"

#include "at32f45x.h"
#include "at32f45x_adc.h"
#include "at32f45x_crm.h"
#include "at32f45x_dma.h"
#include "at32f45x_gpio.h"
#include "at32f45x_misc.h"

static volatile uint16_t adc_dma_raw[3];       /**< DMA依次写入母线、左电流、右电流。 */
static volatile uint32_t adc_dma_sequence;     /**< 仅DMA完整批次结束中断递增。 */
static volatile uint8_t adc_dma_busy;          /**< 扫描开始置1，完成或出错中断清0。 */
static volatile uint8_t adc_dma_ready;         /**< 校准通过为1；DMA错误使其回到0。 */

/**
 * @brief 启动阶段等待ADC就绪，并完成硬件校准初始化和校准。
 * @return 1表示三个等待阶段均完成，0表示任一阶段耗尽轮询次数。
 * @note 仅初始化时阻塞；100000是轮询上限而非时间单位，运行采样不调用此函数。
 */
static uint8_t adc_dma_calibrate(void)
{
  uint32_t limit = 100000UL;
  while((adc_flag_get(ADC1, ADC_RDY_FLAG) == RESET) && (limit != 0U)) { limit--; }
  if(limit == 0U) return 0U;
  adc_calibration_init(ADC1);
  limit = 100000UL;
  while(adc_calibration_init_status_get(ADC1) && (limit != 0U)) { limit--; }
  if(limit == 0U) return 0U;
  adc_calibration_start(ADC1);
  limit = 100000UL;
  while(adc_calibration_status_get(ADC1) && (limit != 0U)) { limit--; }
  return (limit > 0U) ? 1U : 0U;
}

/**
 * @brief 配置ADC1三通道顺序扫描、DMA1通道1搬运及完成/错误中断。
 * @return 1表示ADC硬件校准成功，0表示启动失败，后续start将保持无操作。
 * @note 先关DMA中断并清软件状态，配置完成后才允许中断。此函数也可由
 *       应用层显式故障复位路径重试，重试后仍需新采样满足上层安全检查。
 */
uint8_t adc_dma_init(void)
{
  gpio_init_type gpio;
  adc_common_config_type common;
  adc_base_config_type base;
  dma_init_type dma;

  nvic_irq_disable(DMA1_Channel1_IRQn);
  adc_dma_ready = 0U;
  adc_dma_busy = 0U;
  adc_dma_sequence = 0U;
  crm_periph_clock_enable(CRM_GPIOA_PERIPH_CLOCK, TRUE);
  crm_periph_clock_enable(CRM_ADC1_PERIPH_CLOCK, TRUE);
  crm_periph_clock_enable(CRM_DMA1_PERIPH_CLOCK, TRUE);
  gpio_default_para_init(&gpio);
  gpio.gpio_mode = GPIO_MODE_ANALOG;
  gpio.gpio_pins = GPIO_PINS_3 | GPIO_PINS_4 | GPIO_PINS_5;
  gpio_init(GPIOA, &gpio);
  adc_reset();
  adc_common_default_para_init(&common);
  common.combine_mode = ADC_INDEPENDENT_MODE;
  common.div = ADC_HCLK_DIV_6;
  common.common_dma_mode = ADC_COMMON_DMAMODE_DISABLE;
  adc_common_config(&common);
  adc_base_default_para_init(&base);
  base.sequence_mode = TRUE;
  base.repeat_mode = FALSE;
  base.data_align = ADC_RIGHT_ALIGNMENT;
  base.ordinary_channel_length = 3U;
  adc_base_config(ADC1, &base);
  adc_resolution_set(ADC1, ADC_RESOLUTION_12B);
  adc_ordinary_channel_set(ADC1, ADC_CHANNEL_3, 1U, ADC_SAMPLETIME_92_5);
  adc_ordinary_channel_set(ADC1, ADC_CHANNEL_4, 2U, ADC_SAMPLETIME_92_5);
  adc_ordinary_channel_set(ADC1, ADC_CHANNEL_5, 3U, ADC_SAMPLETIME_92_5);
  /* EDGE_NONE关闭外部触发边沿；下方TMR1CH1枚举并没有启用定时器触发。 */
  adc_ordinary_conversion_trigger_set(ADC1, ADC_ORDINARY_TRIG_TMR1CH1,
                                      ADC_ORDINARY_TRIG_EDGE_NONE);
  dma_reset(DMA1_CHANNEL1);
  dma_default_para_init(&dma);
  dma.peripheral_base_addr = (uint32_t)&ADC1->odt;
  dma.memory_base_addr = (uint32_t)adc_dma_raw;
  dma.direction = DMA_DIR_PERIPHERAL_TO_MEMORY;
  dma.buffer_size = 3U;
  dma.peripheral_inc_enable = FALSE;
  dma.memory_inc_enable = TRUE;
  dma.peripheral_data_width = DMA_PERIPHERAL_DATA_WIDTH_HALFWORD;
  dma.memory_data_width = DMA_MEMORY_DATA_WIDTH_HALFWORD;
  dma.loop_mode_enable = TRUE;
  dma.priority = DMA_PRIORITY_HIGH;
  dma_init(DMA1_CHANNEL1, &dma);
  dmamux_enable(DMA1, TRUE);
  dmamux_init(DMA1MUX_CHANNEL1, DMAMUX_DMAREQ_ID_ADC1);
  dma_flag_clear(DMA1_GL1_FLAG);
  dma_interrupt_enable(DMA1_CHANNEL1, DMA_FDT_INT | DMA_DTERR_INT, TRUE);
  dma_channel_enable(DMA1_CHANNEL1, TRUE);
  adc_dma_mode_enable(ADC1, TRUE);
  adc_dma_request_repeat_enable(ADC1, TRUE);
  adc_enable(ADC1, TRUE);
  if(adc_dma_calibrate() == 0U) return 0U;
  adc_dma_ready = 1U;
  nvic_irq_enable(DMA1_Channel1_IRQn, 2U, 3U);
  return 1U;
}

/**
 * @brief 主循环启动一次三通道扫描，立即返回，由DMA异步填充缓冲区。
 * @note 尚未就绪或上一轮仍忙时忽略本次请求；绝不重启尚未完成的扫描。
 *       主循环必须先take上一批完整样本，再start下一批，以维持快照一致性。
 */
void adc_dma_start(void)
{
  if((adc_dma_ready == 0U) || (adc_dma_busy != 0U)) return;
  adc_dma_busy = 1U;
  adc_ordinary_software_trigger_enable(ADC1, TRUE);
}

/**
 * @brief DMA1通道1 ISR入口：发布整批完成或锁定传输错误状态。
 * @note 仅清硬件标志并更新共享状态，不做单位换算或保护决策。错误优先于
 *       完成处理；错误后ready清0，等待上层显式重新初始化。
 */
void adc_dma_irq(void)
{
  if(dma_flag_get(DMA1_DTERR1_FLAG) != RESET)
  {
    dma_flag_clear(DMA1_GL1_FLAG);
    adc_dma_busy = 0U;
    adc_dma_ready = 0U;
    return;
  }
  if(dma_flag_get(DMA1_FDT1_FLAG) != RESET)
  {
    dma_flag_clear(DMA1_GL1_FLAG);
    adc_dma_sequence++;
    adc_dma_busy = 0U;
  }
}

/**
 * @brief 复制最近一组完成的三通道快照，主循环调用且不等待转换。
 * @param sample 非空输出地址；失败时保持调用方原数据。
 * @return 1表示读到完整快照，0表示未就绪、未完成首批、正在采样或参数为空。
 * @note take不消费批次，连续调用可返回相同sequence；power_telemetry据序号判断新样本。
 *       保存/恢复原PRIMASK，避免误开启调用方已关闭的中断；busy==0才可复制，
 *       因为屏蔽CPU中断并不能阻止DMA继续改写内存。
 */
uint8_t adc_dma_take(adc_dma_sample_t *sample)
{
  uint32_t irq;
  if(sample == 0) return 0U;
  irq = __get_PRIMASK();
  __disable_irq();
  /* 关CPU中断不会停止DMA；只有扫描完成且未启动下一轮时才能取整组样本。 */
  if((adc_dma_ready == 0U) || (adc_dma_sequence == 0U) || (adc_dma_busy != 0U))
  {
    if(irq == 0U) __enable_irq();
    return 0U;
  }
  sample->bus_raw = adc_dma_raw[0];
  sample->left_raw = adc_dma_raw[1];
  sample->right_raw = adc_dma_raw[2];
  sample->sequence = adc_dma_sequence;
  if(irq == 0U) __enable_irq();
  return 1U;
}
