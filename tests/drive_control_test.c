#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "drive_control.h"
#include "motor_encoder.h"
#include "board_config.h"

static int32_t encoder_count[2];
static int16_t applied_left;
static int16_t applied_right;
static uint32_t brake_count;

void motor_encoder_init(void) {}
int32_t motor_encoder_get_count(motor_id_t motor) { return encoder_count[motor]; }
void motor_encoder_reset_all(void) { encoder_count[0] = encoder_count[1] = 0; }
uint32_t motor_encoder_get_invalid_transition_count(motor_id_t motor) { (void)motor; return 0U; }
void motor_driver_init(void) {}
void motor_driver_set_speed(motor_id_t motor, int16_t pwm)
{ if(motor == MOTOR_LEFT) applied_left = pwm; else applied_right = pwm; }
void motor_driver_set_speeds(int16_t left, int16_t right) { applied_left = left; applied_right = right; }
void motor_driver_stop(motor_id_t motor, motor_stop_mode_t mode)
{
  motor_driver_set_speed(motor, 0);
  if(mode == MOTOR_STOP_BRAKE) brake_count++;
}
void motor_driver_stop_all(motor_stop_mode_t mode)
{ applied_left = applied_right = 0; if(mode == MOTOR_STOP_BRAKE) brake_count++; }

static drive_reference_t reference(void)
{
  drive_reference_t value = {100, 0, 500U, 1000U};
  return value;
}

static void expect_true(uint8_t value, const char *text)
{
  if(value == 0U) { fprintf(stderr, "%s\n", text); exit(1); }
}

static void test_healthy_drive(void)
{
  drive_reference_t value = reference();
  uint8_t index;
  encoder_count[0] = encoder_count[1] = 0;
  drive_control_init();
  for(index = 0U; index < 12U; index++)
  {
    encoder_count[MOTOR_LEFT] -= 9;
    encoder_count[MOTOR_RIGHT] -= 9;
    expect_true(drive_control_update(&value, 0.0f, 0.02f), "healthy update");
  }
  expect_true(drive_control_has_fault() == 0U, "healthy feedback fault");
  expect_true((applied_left != 0) || (applied_right != 0), "healthy output missing");
}

static void test_bad_feedback_brakes(void)
{
  drive_reference_t value = reference();
  encoder_count[0] = encoder_count[1] = 0;
  drive_control_init();
  encoder_count[MOTOR_LEFT] = 200;
  expect_true(drive_control_update(&value, 0.0f, 0.02f) == 0U, "raw feedback accepted");
  expect_true((applied_left == 0) && (applied_right == 0) && (brake_count != 0U),
              "fault did not brake both motors");
}

static void test_hold_measures_and_reset_resyncs(void)
{
  drive_reference_t hold = {0};
  drive_reference_t value = reference();
  uint8_t index;
  drive_control_init();
  for(index = 0U; index < 25U; index++)
  {
    encoder_count[MOTOR_LEFT] -= 20;
    encoder_count[MOTOR_RIGHT] -= 20;
    expect_true(drive_control_update(&hold, 500.0f, 0.02f), "HOLD rejected");
    expect_true(applied_left == 0 && applied_right == 0, "HOLD applied yaw PWM");
  }
  expect_true(drive_control_measured_rpm(MOTOR_LEFT) < -20.0f,
              "HOLD failed to measure residual wheel motion");
  expect_true(drive_control_update(&value, 0.0f, 0.02f), "restart counted parked pulses");
  drive_control_stop();
  motor_encoder_reset_all();
  drive_control_resync_feedback();
  expect_true(drive_control_measured_rpm(MOTOR_LEFT) == 0.0f, "reset retained filter");
  expect_true(drive_control_update(&value, 0.0f, 0.02f), "odom reset caused false overspeed");
}

static void test_fault_requires_explicit_clear(void)
{
  drive_reference_t hold = {0};
  drive_control_init();
  encoder_count[MOTOR_LEFT] = 200;
  expect_true(drive_control_update(&hold, 0.0f, 0.02f) == 0U, "HOLD hid overspeed");
  drive_control_stop();
  motor_encoder_reset_all();
  drive_control_resync_feedback();
  expect_true(drive_control_update(&hold, 0.0f, 0.02f) == 0U, "stop/resync cleared fault");
  drive_control_clear_fault();
  expect_true(drive_control_update(&hold, 0.0f, 0.02f), "explicit fault reset failed");
}

static void test_elapsed_time_and_stall(void)
{
  drive_reference_t hold = {0};
  drive_reference_t value = reference();
  drive_control_init();
  encoder_count[MOTOR_LEFT] = 44;
  expect_true(drive_control_update(&hold, 0.0f, 0.025f), "variable period rejected");
  expect_true(fabsf(drive_control_measured_rpm(MOTOR_LEFT) - 8.0f) < 0.01f,
              "RPM did not use actual 25ms elapsed time");

  drive_control_init();
  expect_true(drive_control_update(&value, 0.0f, 0.1f), "stall before 100ms");
  expect_true(drive_control_update(&hold, 0.0f, 0.02f), "HOLD failed after partial stall");
  expect_true(drive_control_update(&value, 0.0f, 0.1f), "HOLD retained stall history");
  expect_true(drive_control_update(&value, 0.0f, 0.099f), "stall before 200ms");
  expect_true(drive_control_update(&value, 0.0f, 0.001f) == 0U, "200ms stall missed");
  expect_true(applied_left == 0 && applied_right == 0, "stall did not brake");
}

static void test_arc_common_ramp_and_reverse_gate(void)
{
  drive_reference_t arc = {100, 200, 500U, 1000U};
  uint8_t index;
  drive_control_init();
  expect_true(drive_control_update(&arc, 200.0f, 0.02f), "arc startup failed");
  /* 起步反馈为零，去掉静摩擦前馈后PWM比例应接近轮速比1:3。 */
  expect_true(abs((applied_right - (int)BOARD_FF_STATIC_PWM) -
                 3 * (-applied_left - (int)BOARD_FF_STATIC_PWM)) <= 3,
              "independent acceleration destroyed 1:3 arc ratio");
  for(index = 0U; index < 16U; index++)
  {
    encoder_count[MOTOR_LEFT] -= 4;
    encoder_count[MOTOR_RIGHT] -= 12;
    expect_true(drive_control_update(&arc, 200.0f, 0.02f), "arc feedback failed");
  }
  arc.angular_mrad_s = 800; /* 只有左轮要求换向，两轮仍应共同减到零。 */
  for(index = 0U; index < 12U; index++)
  {
    encoder_count[MOTOR_LEFT] -= 4;
    encoder_count[MOTOR_RIGHT] -= 12;
    expect_true(drive_control_update(&arc, 800.0f, 0.02f), "reverse deceleration failed");
    expect_true((applied_left == 0) == (applied_right == 0), "wheels did not reach zero together");
    expect_true(applied_left <= 0 && applied_right >= 0, "reversed before zero feedback");
  }
  expect_true(applied_left == 0 && applied_right == 0, "reverse did not hold brake");
  for(index = 0U; index < 2U; index++)
  {
    expect_true(drive_control_update(&arc, 800.0f, 0.02f), "zero gate update failed");
    expect_true(applied_left == 0 && applied_right == 0, "reverse used fewer than 3 zero samples");
  }
  expect_true(drive_control_update(&arc, 800.0f, 0.02f), "reverse release failed");
  expect_true(applied_left > 0 && applied_right > 0, "reverse not released after 3 zero samples");
}

static void test_init_resets_direction_history(void)
{
  drive_reference_t value = reference();
  drive_control_init();
  expect_true(drive_control_update(&value, 0.0f, 0.02f), "initial forward failed");
  drive_control_init();
  value.linear_mm_s = -100;
  expect_true(drive_control_update(&value, 0.0f, 0.02f), "fresh reverse failed");
  expect_true(applied_left > 0 && applied_right < 0, "init retained old direction history");
}

int main(void)
{
  test_healthy_drive();
  test_bad_feedback_brakes();
  test_hold_measures_and_reset_resyncs();
  test_fault_requires_explicit_clear();
  test_elapsed_time_and_stall();
  test_arc_common_ramp_and_reverse_gate();
  test_init_resets_direction_history();
  puts("DRIVE_CONTROL_TEST_PASS");
  return 0;
}
