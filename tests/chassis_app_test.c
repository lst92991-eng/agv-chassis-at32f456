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
static uint8_t test_bumper, test_bus_healthy;
static chassis_status_t observed_status;

uint32_t board_port_now_ms(void) { return test_now; }
uint8_t board_port_watchdog_init(void) { watchdog_calls++; return watchdog_ok; }
void board_port_watchdog_feed(void) {}
uint8_t adc_dma_init(void) { adc_calls++; return adc_init_ok; }
void adc_dma_start(void) {}
uint8_t adc_dma_take(adc_dma_sample_t *value) { *value = sample; return adc_init_ok && sample.sequence != 0U; }
uint8_t imu_bmi270_init(void) { imu_calls++; imu_fresh = imu_init_ok; return imu_init_ok; }
uint8_t imu_bmi270_update(float dt) { (void)dt; return imu_fresh; }
uint8_t imu_bmi270_fresh(void) { return imu_fresh; }
float imu_bmi270_gyro_z_dps(void) { return 0.0f; }
void bumper_init(void) {}
void bumper_update(void) {}
uint8_t bumper_latched(void) { return test_bumper; }
uint8_t bumper_can_clear(void) { return test_bumper == 0U; }
uint8_t bumper_clear(void) { return 1U; }
void chassis_protocol_init(void) { reference_valid = 0U; incoming_command.type = CHASSIS_CMD_NONE; }
void chassis_protocol_poll(uint32_t now) { (void)now; }
uint8_t chassis_protocol_bus_healthy(void) { return test_bus_healthy; }
void canfd_driver_recover(void) { test_bus_healthy = 1U; }
void chassis_protocol_clear_reference(void) { reference_valid = 0U; }
uint8_t chassis_protocol_take_command(chassis_command_t *value)
{
  *value = incoming_command;
  incoming_command.type = CHASSIS_CMD_NONE;
  return value->type != CHASSIS_CMD_NONE;
}
uint8_t chassis_protocol_get_reference(drive_reference_t *value)
{ if(reference_valid) *value = incoming_reference; return reference_valid; }
void chassis_protocol_send_status(const chassis_status_t *value, uint32_t now)
{ observed_status = *value; (void)now; }
void chassis_protocol_send_odometry(const odometry_t *value, int16_t v, int16_t w, uint32_t now)
{ (void)value; (void)v; (void)w; (void)now; }
void drive_control_init(void) { memset(encoder_counts, 0, sizeof(encoder_counts)); }
void drive_control_stop(void) { memset(&applied_reference, 0, sizeof(applied_reference)); }
void drive_control_resync_feedback(void) { resync_calls++; }
void drive_control_update(const drive_reference_t *value, float dt)
{ applied_reference = *value; drive_dt = dt; drive_calls++; }
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
static void step(uint32_t elapsed)
{
  test_now += elapsed; sample.bus_raw=1000U; sample.left_raw=3000U;
  sample.sequence++; chassis_app_step();
}
static void command(chassis_command_type_t code)
{
  incoming_command.type=code; incoming_command.sequence++; step(20U);
}
int main(void)
{
  test_now=1000U; adc_init_ok=imu_init_ok=0U; watchdog_ok=1U;
  test_bumper=15U;test_bus_healthy=0U;
  chassis_app_init();
  check(watchdog_calls==0U && chassis_state==CHASSIS_IDLE, "startup gate/WDT remains");
  incoming_reference.linear_mm_s=100;incoming_reference.accel_mm_s2=500;
  incoming_reference.decel_mm_s2=1000;reference_valid=1;
  step(20U);
  check(chassis_state==CHASSIS_RUNNING && applied_reference.linear_mm_s==100,
        "single motion frame was blocked by sensor/enable gate");
  step(5000U);
  check(chassis_state==CHASSIS_RUNNING && applied_reference.linear_mm_s==100,
        "elapsed time caused automatic stop");
  check(observed_status.state==2U && (observed_status.fault & 0x59U)==0x59U,
        "sensor/bumper/CAN diagnostics were not reported independently");
  command(CHASSIS_CMD_STOP);
  check(chassis_state==CHASSIS_IDLE && applied_reference.linear_mm_s==0 && !reference_valid,
        "STOP did not discard prior target");
  step(20U);check(chassis_state==CHASSIS_IDLE, "old target resumed after STOP");
  reference_valid=1;step(20U);check(chassis_state==CHASSIS_RUNNING, "new target after STOP rejected");
  command(CHASSIS_CMD_DISABLE);reference_valid=1;step(20U);
  check(chassis_state==CHASSIS_IDLE && applied_reference.linear_mm_s==0, "explicit DISABLE ignored");
  command(CHASSIS_CMD_ENABLE);
  check(chassis_state==CHASSIS_ENABLED && !reference_valid, "ENABLE retained pre-command target");
  reference_valid=1;step(20U);
  command(CHASSIS_CMD_ESTOP);reference_valid=1;step(20U);
  check(chassis_state==CHASSIS_ESTOP && applied_reference.linear_mm_s==0, "ESTOP did not hold");
  command(CHASSIS_CMD_ENABLE);
  check(chassis_state==CHASSIS_ESTOP && chassis_last_command_result==1, "ENABLE released ESTOP");
  command(CHASSIS_CMD_FAULT_RESET);
  check(chassis_state==CHASSIS_IDLE && chassis_last_command_result==0, "RESET retained sensor admission gate");
  reference_valid=1;step(20U);encoder_counts[0]=encoder_counts[1]=-500;
  command(CHASSIS_CMD_RESET_ODOMETRY);
  check(chassis_state==CHASSIS_RUNNING && resync_calls==1U && encoder_counts[0]==0,
        "odometry reset changed motion or omitted feedback resync");
  memset(&incoming_reference,0,sizeof(incoming_reference));reference_valid=1;step(20U);
  check(chassis_state==CHASSIS_ENABLED && applied_reference.linear_mm_s==0, "HOLD failed");
  check(watchdog_calls==0U, "watchdog reintroduced");
  puts("CHASSIS_APP_TEST_PASS");return 0;
}
