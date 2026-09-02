/** @file adc_dma.h @brief ADC1三通道扫描DMA最新样本。 */
#ifndef ADC_DMA_H
#define ADC_DMA_H

#include <stdint.h>

/** @brief 单批ADC扫描快照；原始值为12位右对齐读数，sequence用于判断更新。 */
typedef struct
{
  uint16_t bus_raw;       /**< PA3/ADC通道3：分压后的母线采样。 */
  uint16_t left_raw;      /**< PA4/ADC通道4：左电机电流采样。 */
  uint16_t right_raw;     /**< PA5/ADC通道5：右电机电流采样。 */
  uint32_t sequence;      /**< 整批扫描完成计数；同序号表示同一批数据。 */
} adc_dma_sample_t;

/** @brief 配置ADC1扫描DMA并执行启动校准；成功返回1，失败返回0。 */
uint8_t adc_dma_init(void);
/** @brief 非阻塞软件触发一批三通道采样；未就绪或忙时忽略请求。 */
void adc_dma_start(void);
/** @brief 仅供DMA1通道1 ISR调用，发布批次完成或传输故障。 */
void adc_dma_irq(void);
/**
 * @brief 将最近完整批次复制到sample；成功返回1，否则返回0。
 * @note 本函数不等待、不消费样本；主循环应先take后start，用sequence识别新批次。
 */
uint8_t adc_dma_take(adc_dma_sample_t *sample);

#endif
