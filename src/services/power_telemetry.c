/** @file power_telemetry.c @brief 只采集、换算与上报；电气保护由上位机执行。 */
#include "power_telemetry.h"

#include <string.h>
#include "board_config.h"

static power_telemetry_t power_value;
static uint16_t power_zero[2];
static uint32_t power_sum[2];
static uint32_t power_sequence, power_sample_ms;
static uint8_t power_zero_count;

static uint16_t power_u16(uint32_t value)
{
  return value > 65535UL ? 65535U : (uint16_t)value;
}

static uint16_t power_current(uint16_t raw, uint16_t zero)
{
  return raw > zero ? power_u16((uint32_t)(raw - zero) *
      BOARD_CURRENT_UA_PER_COUNT / 1000UL) : 0U;
}

void power_telemetry_init(void)
{
  memset(&power_value, 0, sizeof(power_value));
  memset(power_zero, 0, sizeof(power_zero));
  memset(power_sum, 0, sizeof(power_sum));
  power_sequence = power_sample_ms = 0U;
  power_zero_count = 0U;
}

void power_telemetry_update(uint32_t now_ms, const adc_dma_sample_t *sample,
                            uint8_t stationary)
{
  if((sample != 0) && (sample->sequence != 0U) &&
     (sample->sequence != power_sequence))
  {
    power_sequence = sample->sequence;
    power_sample_ms = now_ms;
    /* 未完成零点时可直接运动，使用零偏0；停车后继续完成标定。 */
    if(power_zero_count < BOARD_ADC_ZERO_SAMPLES)
    {
      if((stationary != 0U) && (sample->left_raw <= BOARD_ADC_ZERO_MAX_RAW) &&
         (sample->right_raw <= BOARD_ADC_ZERO_MAX_RAW))
      {
        power_sum[0] += sample->left_raw;
        power_sum[1] += sample->right_raw;
        if(++power_zero_count == BOARD_ADC_ZERO_SAMPLES)
        {
          power_zero[0] = (uint16_t)(power_sum[0] / BOARD_ADC_ZERO_SAMPLES);
          power_zero[1] = (uint16_t)(power_sum[1] / BOARD_ADC_ZERO_SAMPLES);
        }
      }
      else
      {
        power_sum[0] = power_sum[1] = 0U;
        power_zero_count = 0U;
      }
    }
    power_value.bus_mv = power_u16((uint32_t)sample->bus_raw *
        BOARD_BUS_UV_PER_COUNT / 1000UL + BOARD_BUS_OFFSET_MV);
    power_value.left_current_ma = power_current(sample->left_raw, power_zero[0]);
    power_value.right_current_ma = power_current(sample->right_raw, power_zero[1]);
  }
  power_value.valid = ((power_sequence != 0U) &&
      ((now_ms - power_sample_ms) < BOARD_ADC_REPORT_STALE_MS)) ? 1U : 0U;
}

void power_telemetry_get(power_telemetry_t *value)
{
  if(value != 0) *value = power_value;
}
