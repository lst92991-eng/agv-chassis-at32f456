/** @file drive_control.c @brief 车体v/omega差速换算、共同斜率与双轮速度PID。 */
#include "drive_control.h"
#include "board_config.h"
#include "motor_encoder.h"
#include <string.h>

typedef struct
{
  float target_rpm, measured_rpm, integral, previous_error;
  int32_t last_count;
  int16_t pwm;
} drive_axis_t;

static drive_axis_t drive_axis[2];
static float drive_left_ramp, drive_right_ramp;

static float drive_abs(float value) { return value < 0.0f ? -value : value; }
static float drive_limit(float value, float limit)
{
  if(value > limit) return limit;
  if(value < -limit) return -limit;
  return value;
}

/* 两轮共用较小比例，保持差速目标比例；斜率参数0表示直接到达目标。 */
static float drive_fraction(float current, float target, float up, float down)
{
  float delta = drive_abs(target - current);
  float limit = current * target >= 0.0f && drive_abs(target) > drive_abs(current) ? up : down;
  return limit > 0.0f && delta > limit ? limit / delta : 1.0f;
}

static void drive_measure(drive_axis_t *axis, motor_id_t motor, float dt_s)
{
  int32_t count = motor_encoder_get_count(motor);
  int32_t delta = (int32_t)((uint32_t)count - (uint32_t)axis->last_count);
  float sign = motor == MOTOR_LEFT ? BOARD_LEFT_FEEDBACK_SIGN : BOARD_RIGHT_FEEDBACK_SIGN;
  float rpm = (float)delta * 60.0f / (BOARD_ENCODER_COUNTS_PER_REV * dt_s) * sign;
  axis->last_count = count;
  axis->measured_rpm += BOARD_RPM_FILTER_ALPHA * (rpm - axis->measured_rpm);
}

/* PID限幅是执行器数值范围与抗积分饱和，不产生故障或运动门禁。 */
static int16_t drive_pid(drive_axis_t *axis, float dt_s)
{
  float error, derivative, candidate, correction, output;
  if(axis->target_rpm == 0.0f)
  {
    axis->integral = axis->previous_error = 0.0f;
    return 0;
  }
  error = axis->target_rpm - axis->measured_rpm;
  derivative = (error - axis->previous_error) / dt_s;
  candidate = drive_limit(axis->integral + error * dt_s, BOARD_PID_LIMIT / BOARD_PID_KI);
  correction = BOARD_PID_KP * error + BOARD_PID_KI * candidate + BOARD_PID_KD * derivative;
  if(!((correction > BOARD_PID_LIMIT && error > 0.0f) ||
       (correction < -BOARD_PID_LIMIT && error < 0.0f))) axis->integral = candidate;
  axis->previous_error = error;
  output = (axis->target_rpm > 0.0f ? BOARD_FF_STATIC_PWM : -BOARD_FF_STATIC_PWM) +
      axis->target_rpm * BOARD_FF_PER_RPM +
      drive_limit(BOARD_PID_KP * error + BOARD_PID_KI * axis->integral +
                  BOARD_PID_KD * derivative, BOARD_PID_LIMIT);
  if(axis->target_rpm > 0.0f && output < 0.0f) output = 0.0f;
  if(axis->target_rpm < 0.0f && output > 0.0f) output = 0.0f;
  return (int16_t)drive_limit(output, 1000.0f);
}

void drive_control_init(void)
{
  motor_encoder_init();
  motor_driver_init();
  motor_encoder_reset_all();
  memset(drive_axis, 0, sizeof(drive_axis));
  drive_control_resync_feedback();
  drive_control_stop();
}

void drive_control_resync_feedback(void)
{
  uint8_t index;
  for(index = 0U; index < 2U; index++)
  {
    drive_axis[index].last_count = motor_encoder_get_count((motor_id_t)index);
    drive_axis[index].measured_rpm = 0.0f;
  }
}

void drive_control_stop(void)
{
  uint8_t index;
  for(index = 0U; index < 2U; index++)
  {
    drive_axis[index].target_rpm = 0.0f;
    drive_axis[index].integral = drive_axis[index].previous_error = 0.0f;
    drive_axis[index].pwm = 0;
  }
  drive_left_ramp = drive_right_ramp = 0.0f;
  motor_driver_stop_all(MOTOR_STOP_BRAKE);
}

void drive_control_update(const drive_reference_t *reference, float dt_s)
{
  float turn, left, right, up, down, fraction, right_fraction;
  uint8_t index;
  /* 仅排除无法参与数学运算的调用，不锁存故障、不修改已有输出。 */
  if(reference == 0 || !(dt_s > 0.0f)) return;
  drive_measure(&drive_axis[MOTOR_LEFT], MOTOR_LEFT, dt_s);
  drive_measure(&drive_axis[MOTOR_RIGHT], MOTOR_RIGHT, dt_s);
  if(!reference->linear_mm_s && !reference->angular_mrad_s &&
     !reference->accel_mm_s2 && !reference->decel_mm_s2)
  {
    drive_control_stop();
    return;
  }
  turn = BOARD_TRACK_WIDTH_MM * reference->angular_mrad_s / 2000.0f;
  left = (reference->linear_mm_s - turn) * 60.0f / BOARD_WHEEL_CIRCUMFERENCE_MM;
  right = (reference->linear_mm_s + turn) * 60.0f / BOARD_WHEEL_CIRCUMFERENCE_MM;
  up = reference->accel_mm_s2 * dt_s * 60.0f / BOARD_WHEEL_CIRCUMFERENCE_MM;
  down = reference->decel_mm_s2 * dt_s * 60.0f / BOARD_WHEEL_CIRCUMFERENCE_MM;
  fraction = drive_fraction(drive_left_ramp, left, up, down);
  right_fraction = drive_fraction(drive_right_ramp, right, up, down);
  if(right_fraction < fraction) fraction = right_fraction;
  drive_left_ramp += (left - drive_left_ramp) * fraction;
  drive_right_ramp += (right - drive_right_ramp) * fraction;
  drive_axis[MOTOR_LEFT].target_rpm = drive_left_ramp * BOARD_LEFT_TARGET_SIGN;
  drive_axis[MOTOR_RIGHT].target_rpm = drive_right_ramp * BOARD_RIGHT_TARGET_SIGN;
  for(index = 0U; index < 2U; index++)
  {
    drive_axis[index].pwm = drive_pid(&drive_axis[index], dt_s);
    if(drive_axis[index].target_rpm == 0.0f)
      motor_driver_stop((motor_id_t)index, MOTOR_STOP_BRAKE);
    else motor_driver_set_speed((motor_id_t)index, drive_axis[index].pwm);
  }
}

float drive_control_measured_rpm(motor_id_t motor)
{
  return motor <= MOTOR_RIGHT ? drive_axis[motor].measured_rpm : 0.0f;
}
int16_t drive_control_pwm(motor_id_t motor)
{
  return motor <= MOTOR_RIGHT ? drive_axis[motor].pwm : 0;
}
