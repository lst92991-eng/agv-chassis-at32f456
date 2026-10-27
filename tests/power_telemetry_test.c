#include <assert.h>
#include <stdio.h>
#include "power_telemetry.h"
int main(void)
{
  adc_dma_sample_t s={2450U,5U,10U,0U};
  power_telemetry_t value;
  unsigned i;
  power_telemetry_init();
  for(i=1;i<=20;i++){s.sequence++;power_telemetry_update(i*20,&s,1);}
  s.sequence++;s.left_raw=15;s.right_raw=30;
  power_telemetry_update(420,&s,0);power_telemetry_get(&value);
  assert(value.valid && value.left_current_ma==8 && value.right_current_ma==16);
  s.sequence++;s.bus_raw=100;s.left_raw=2000;s.right_raw=3000;
  power_telemetry_update(440,&s,0);power_telemetry_get(&value);
  assert(value.valid && value.bus_mv==885 && value.left_current_ma>800);
  s.sequence++;s.bus_raw=4000;
  power_telemetry_update(460,&s,0);power_telemetry_get(&value);
  assert(value.valid && value.bus_mv==35400);
  power_telemetry_update(520,&s,0);power_telemetry_get(&value);assert(!value.valid);
  s.sequence++;power_telemetry_update(530,&s,0);power_telemetry_get(&value);assert(value.valid);
  power_telemetry_init();s.sequence++;s.left_raw=345;s.right_raw=582;s.bus_raw=2260;
  power_telemetry_update(20,&s,0);power_telemetry_get(&value);
  assert(value.valid && value.left_current_ma==278 && value.right_current_ma==469);
  puts("POWER_TELEMETRY_TEST_PASS");return 0;
}
