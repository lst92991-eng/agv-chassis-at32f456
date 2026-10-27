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

static void check(int ok, const char *message)
{ if(!ok) { fprintf(stderr, "%s\n", message); exit(1); } }

int main(void)
{
  drive_reference_t forward = {100, 0, 0U, 0U};
  drive_reference_t hold = {0, 0, 0U, 0U};
  unsigned i;
  uint32_t before;
  drive_control_init();
  for(i=0;i<200;i++) drive_control_update(&forward, 0.02f);
  check(applied_left < 0 && applied_right > 0, "command stalled without encoder movement");
  forward.linear_mm_s = -100;
  drive_control_update(&forward, 0.02f);
  check(applied_left > 0 && applied_right < 0, "local reversal gate remains");
  forward.linear_mm_s = 0; forward.angular_mrad_s = 400;
  drive_control_update(&forward, 0.02f);
  check(applied_left > 0 && applied_right > 0, "differential turn signs wrong");
  forward.linear_mm_s = 1000; forward.angular_mrad_s = 0;
  drive_control_update(&forward, 0.5f);
  check(applied_left == -1000 && applied_right == 1000, "range/period fault gate remains");
  before=brake_count;
  encoder_count[0] = encoder_count[1] = 10000;
  drive_control_update(&forward, 0.02f);
  check(brake_count==before && fabsf(drive_control_measured_rpm(MOTOR_LEFT))>120.0f,
        "encoder threshold still causes fault stop");
  drive_control_update(&hold, 0.02f);
  check(applied_left==0 && applied_right==0 && brake_count>before, "HOLD did not brake");
  motor_encoder_reset_all(); drive_control_resync_feedback();
  drive_control_update(&hold, 0.02f);
  check(drive_control_measured_rpm(MOTOR_LEFT)==0.0f, "feedback reset created count jump");
  forward.linear_mm_s=100; forward.accel_mm_s2=100; forward.decel_mm_s2=500;
  drive_control_init(); drive_control_update(&forward,0.02f);
  check(applied_left<0 && applied_right>0 && applied_right<200, "ramp did not apply");
  drive_control_stop(); check(applied_left==0 && applied_right==0, "explicit stop failed");
  puts("DRIVE_CONTROL_TEST_PASS"); return 0;
}
