#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include "../src/app/chassis_app.c"

/* 只替换外设边界；执行生产状态机和真实safety模块。 */
static uint32_t test_now;
static uint8_t adc_init_ok, imu_init_ok, imu_fresh, watchdog_ok;
static unsigned int watchdog_calls, imu_calls, adc_calls, drive_calls, resync_calls;
static float drive_dt, pose_dt;
static int32_t encoder_counts[2];
static adc_dma_sample_t sample;
static drive_reference_t incoming_reference, applied_reference;
static chassis_command_t incoming_command;
static uint8_t reference_valid;

uint32_t board_port_now_ms(void) { return test_now; }
uint8_t board_port_watchdog_init(void) { watchdog_calls++; return watchdog_ok; }
void board_port_watchdog_feed(void) {}
uint8_t adc_dma_init(void) { adc_calls++; return adc_init_ok; }
void adc_dma_start(void) {}
uint8_t adc_dma_take(adc_dma_sample_t *value) { *value = sample; return sample.sequence != 0U; }
uint8_t imu_bmi270_init(void) { imu_calls++; imu_fresh = imu_init_ok; return imu_init_ok; }
uint8_t imu_bmi270_update(float dt) { (void)dt; return imu_fresh; }
uint8_t imu_bmi270_fresh(void) { return imu_fresh; }
float imu_bmi270_gyro_z_dps(void) { return 0.0f; }
void bumper_init(void) {}
void bumper_update(void) {}
uint8_t bumper_latched(void) { return 0U; }
uint8_t bumper_can_clear(void) { return 1U; }
uint8_t bumper_clear(void) { return 1U; }
void chassis_protocol_init(void) { reference_valid = 0U; incoming_command.type = CHASSIS_CMD_NONE; }
void chassis_protocol_poll(uint32_t now) { (void)now; }
uint8_t chassis_protocol_bus_healthy(void) { return 1U; }
void canfd_driver_recover(void) {}
void chassis_protocol_clear_reference(void) { reference_valid = 0U; }
uint8_t chassis_protocol_take_command(chassis_command_t *value)
{
  *value = incoming_command;
  incoming_command.type = CHASSIS_CMD_NONE;
  return value->type != CHASSIS_CMD_NONE;
}
uint8_t chassis_protocol_get_reference(uint32_t now, drive_reference_t *value)
{ (void)now; if(reference_valid) *value = incoming_reference; return reference_valid; }
void chassis_protocol_send_status(const chassis_status_t *value, uint32_t now)
{ (void)value; (void)now; }
void chassis_protocol_send_odometry(const odometry_t *value, int16_t v, int16_t w, uint32_t now)
{ (void)value; (void)v; (void)w; (void)now; }
void drive_control_init(void) { memset(encoder_counts, 0, sizeof(encoder_counts)); }
void drive_control_stop(void) { memset(&applied_reference, 0, sizeof(applied_reference)); }
void drive_control_clear_fault(void) { drive_control_stop(); }
void drive_control_resync_feedback(void) { resync_calls++; }
uint8_t drive_control_update(const drive_reference_t *value, float yaw, float dt)
{ (void)yaw; applied_reference = *value; drive_dt = dt; drive_calls++; return 1U; }
float drive_control_measured_rpm(motor_id_t motor) { (void)motor; return 0.0f; }
int16_t drive_control_pwm(motor_id_t motor) { (void)motor; return 0; }
int32_t motor_encoder_get_count(motor_id_t motor) { return encoder_counts[motor]; }
void motor_encoder_reset_all(void) { memset(encoder_counts, 0, sizeof(encoder_counts)); }
void odometry_init(void) {}
void odometry_reset(void) {}
void odometry_update(int32_t left, int32_t right, float yaw, float dt)
{ (void)left; (void)right; (void)yaw; pose_dt = dt; }
void odometry_get(odometry_t *value) { memset(value, 0, sizeof(*value)); value->valid = 1U; }

static void check(int ok, const char *message)
{ if(!ok) { fprintf(stderr, "%s\n", message); exit(1); } }

static void reset_fixture(void)
{
  test_now = 1000U;
  adc_init_ok = imu_init_ok = watchdog_ok = 1U;
  watchdog_calls = imu_calls = adc_calls = drive_calls = resync_calls = 0U;
  memset(&sample, 0, sizeof(sample));
  memset(&incoming_reference, 0, sizeof(incoming_reference));
}

static void step(uint32_t elapsed_ms)
{
  test_now += elapsed_ms;
  sample.bus_raw = 2260U;
  sample.sequence++;
  chassis_app_step();
}

int main(void)
{
  unsigned int index;
  reset_fixture();
  adc_init_ok = 0U;
  check(chassis_app_init() != 0U && imu_calls == 1U && watchdog_calls == 1U,
        "ADC failure skipped IMU/WDT initialization");
  check((safety_fault() & SAFETY_FAULT_POWER) != 0U, "ADC failure not reported");
  adc_init_ok = 1U;
  incoming_command.type = CHASSIS_CMD_FAULT_RESET;
  chassis_handle_command();
  check(adc_calls == 2U && chassis_state == CHASSIS_FAULT,
        "ADC recovery must reinit and wait for fresh samples before clearing");
  reset_fixture();
  imu_init_ok = 0U;
  check(chassis_app_init() != 0U && watchdog_calls == 1U, "IMU failure skipped WDT");
  reset_fixture();
  watchdog_ok = 0U;
  check(chassis_app_init() == 0U, "WDT init failure was reported as success");

  reset_fixture();
  check(chassis_app_init() != 0U, "init failed");
  step(21U);
  check(drive_calls == 1U && fabsf(drive_dt - 0.021f) < 0.00001f,
        "disabled chassis did not sample using actual elapsed time");
  step(25U);
  check(fabsf(pose_dt - 0.025f) < 0.00001f && fabsf(drive_dt - 0.025f) < 0.00001f,
        "odometry or wheel feedback still uses fixed dt");
  for(index = 0U; index < 20U; index++) step(20U);
  check(safety_can_enable(), "stationary ADC calibration failed");

  incoming_reference.linear_mm_s = 100;
  incoming_reference.accel_mm_s2 = 500U;
  incoming_reference.decel_mm_s2 = 1000U;
  reference_valid = 1U;
  incoming_command.type = CHASSIS_CMD_ENABLE;
  step(20U);
  check(chassis_state == CHASSIS_ENABLED && !reference_valid,
        "ENABLE executed a reference received before enable");
  reference_valid = 1U;
  step(20U);
  check(chassis_state == CHASSIS_RUNNING && applied_reference.linear_mm_s == 100,
        "fresh post-enable reference did not execute");
  incoming_command.type = CHASSIS_CMD_STOP;
  step(20U);
  check(chassis_state == CHASSIS_DISABLED && applied_reference.linear_mm_s == 0,
        "STOP failed to brake");
  encoder_counts[0] = encoder_counts[1] = -500;
  incoming_command.type = CHASSIS_CMD_RESET_ODOMETRY;
  step(20U);
  check(resync_calls == 1U && encoder_counts[0] == 0, "odometry reset omitted feedback resync");

  step(70U);
  check((safety_fault() & SAFETY_FAULT_TIMEOUT) != 0U && applied_reference.linear_mm_s == 0,
        "control overrun did not fault and brake");
  puts("CHASSIS_APP_TEST_PASS");
  return 0;
}
