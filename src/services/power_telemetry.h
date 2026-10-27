/** @file power_telemetry.h @brief ADC电压/电流换算与有效性上报，不控制运动。 */
#ifndef POWER_TELEMETRY_H
#define POWER_TELEMETRY_H

#include <stdint.h>
#include "adc_dma.h"

typedef struct
{
  uint16_t bus_mv;
  uint16_t left_current_ma;
  uint16_t right_current_ma;
  uint8_t valid; /* 只表示采样新鲜度，供上位机判断。 */
} power_telemetry_t;

void power_telemetry_init(void);
/* stationary仅用于建立电流零点，不是运动准入条件。 */
void power_telemetry_update(uint32_t now_ms, const adc_dma_sample_t *sample,
                            uint8_t stationary);
void power_telemetry_get(power_telemetry_t *value);

#endif
