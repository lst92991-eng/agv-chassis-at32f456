#include <stdio.h>
#include <stdlib.h>

#include "safety.h"

static void expect_true(int value, const char *text)
{
  if(!value) { fprintf(stderr, "%s\n", text); exit(1); }
}

int main(void)
{
  adc_dma_sample_t sample = {2260U, 0U, 0U, 0U};
  safety_status_t status;
  unsigned int index;
  safety_init(0U);
  for(index = 1U; index <= 20U; index++)
  {
    sample.sequence = index;
    safety_update(index * 20U, &sample, 1U);
  }
  expect_true(safety_can_enable() != 0U, "zero calibration did not enable");
  sample.sequence++;
  sample.bus_raw = 100U;
  safety_update(420U, &sample, 0U);
  expect_true((safety_fault() & SAFETY_FAULT_POWER) != 0U, "undervoltage not latched");
  sample.sequence++;
  sample.bus_raw = 2260U;
  safety_update(440U, &sample, 0U);
  expect_true(safety_clear(440U) != 0U, "recovered power fault did not clear");
  safety_get(&status);
  expect_true(status.ready != 0U, "ready not restored");
  safety_update(500U, &sample, 0U);
  expect_true(safety_clear(500U) == 0U, "stale ADC cleared power fault");
  safety_init(1000U);
  safety_update(1010U, 0, 0U);
  expect_true(safety_fault() == 0U, "boot calibration time consumed ADC grace");
  safety_update(1060U, 0, 0U);
  expect_true(safety_fault() != 0U, "missing first ADC sample did not expire");
  /* 实车抓到的345/582个净ADC码应为278/469mA，不能放大10倍误报过流。 */
  safety_init(0U);
  sample.left_raw = 345U;
  sample.right_raw = 582U;
  for(index = 1U; index <= 6U; index++)
  {
    sample.sequence++;
    safety_update(index * 20U, &sample, 0U);
  }
  safety_get(&status);
  expect_true(status.left_current_ma == 278U && status.right_current_ma == 469U,
              "INA180A3/10mohm current conversion has wrong scale");
  expect_true(safety_fault() == 0U, "normal measured startup current falsely tripped");
  safety_init(0U);
  sample.left_raw = 1000U;
  sample.right_raw = 0U;
  for(index = 1U; index <= 3U; index++)
  {
    sample.sequence++;
    safety_update(index * 25U, &sample, 0U);
  }
  expect_true(safety_fault() == 0U, "overcurrent tripped before 100ms");
  sample.sequence++;
  safety_update(100U, &sample, 0U);
  expect_true(safety_fault() != 0U, "100ms overcurrent depended on five function calls");
  puts("SAFETY_TEST_PASS");
  return 0;
}
