/** @file safety.h @brief 车辆本地故障、ADC样本和ENABLE准入。 */
#ifndef SAFETY_H
#define SAFETY_H

#include <stdint.h>

#include "adc_dma.h"

/** @brief 故障位图，多个原因可同时存在；只有显式复位才能清除。 */
enum
{
  SAFETY_FAULT_NONE = 0U,          /**< 没有已锁存的故障。 */
  SAFETY_FAULT_BUMPER = 0x01U,    /**< 防撞条触发，由应用层上报。 */
  SAFETY_FAULT_ESTOP = 0x02U,     /**< 收到显式急停命令。 */
  SAFETY_FAULT_TIMEOUT = 0x04U,   /**< Reference失效或主循环超时，由应用层上报。 */
  SAFETY_FAULT_CAN = 0x08U,       /**< CAN总线/接收异常，由应用层上报。 */
  SAFETY_FAULT_IMU = 0x10U,       /**< IMU不可用或里程计时间步异常，由应用层上报。 */
  SAFETY_FAULT_DRIVE = 0x20U,     /**< 轮控输入、测速或堵转异常，由应用层上报。 */
  SAFETY_FAULT_POWER = 0x40U      /**< ADC陈旧、欠压/过压或持续峰值电流。 */
};

/** @brief 电气量及统一故障快照，电压/电流为经标定换算后的整数物理量。 */
typedef struct
{
  uint16_t bus_mv;                /**< 母线电压，mV。 */
  uint16_t left_current_ma;       /**< 左侧电流，mA，扣除静止零点后最小为0。 */
  uint16_t right_current_ma;      /**< 右侧电流，mA，扣除静止零点后最小为0。 */
  uint8_t fault;                 /**< SAFETY_FAULT_*按位组合，直到显式复位。 */
  uint8_t ready;                 /**< 校准完成且无故障；不包含应用额外的ENABLE条件。 */
} safety_status_t;

/** @brief 用当前单调毫秒now_ms初始化电气监督；启动后等待连续静止样本建立零点。 */
void safety_init(uint32_t now_ms);
/**
 * @brief 主循环更新ADC监督；即使没有新样本也应调用，以检测采样断流。
 * @param now_ms 当前真实毫秒数。
 * @param sample 完整DMA快照，可为空；重复序号不刷新新鲜度。
 * @param zero_allowed 仅在应用确认静止、禁止驱动时置1，允许零点校准。
 */
void safety_update(uint32_t now_ms, const adc_dma_sample_t *sample,
                   uint8_t zero_allowed);
/** @brief 将fault中的故障位锁存并撤销准入；实际制动由调用方执行。 */
void safety_raise(uint8_t fault);
/** @brief 返回电气校准完成且无故障的准入位；ENABLE还需应用检查IMU、防撞及CAN。 */
uint8_t safety_can_enable(void);
/** @brief 返回统一故障位图，读取不清除锁存。 */
uint8_t safety_fault(void);
/**
 * @brief 显式清故障：要求ADC新鲜、电压在恢复区间、左右电流低于峰值门限。
 * @param now_ms 当前单调毫秒数。
 * @return 1清除成功、0电气条件未恢复；成功后ready仍取决于零点校准是否完成。
 * 调用方须先确认非电气故障原因已解除，此接口自身不检查防撞、IMU或CAN。
 */
uint8_t safety_clear(uint32_t now_ms);
/** @brief 复制最新状态到status，空指针忽略；与更新函数在同一主循环上下文使用。 */
void safety_get(safety_status_t *status);

#endif
