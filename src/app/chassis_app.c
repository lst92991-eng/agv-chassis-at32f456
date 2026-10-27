/** @file chassis_app.c @brief 上位机命令执行与遥测组合，不承担自动保护。 */
#include "chassis_app.h"
#include <string.h>
#include "adc_dma.h"
#include "board_config.h"
#include "board_port.h"
#include "bumper.h"
#include "canfd_driver.h"
#include "chassis_protocol.h"
#include "drive_control.h"
#include "imu_bmi270.h"
#include "motor_encoder.h"
#include "odometry.h"
#include "power_telemetry.h"

typedef enum
{
  CHASSIS_IDLE = 0, CHASSIS_ENABLED = 1,
  CHASSIS_RUNNING = 2, CHASSIS_ESTOP = 3
} chassis_state_t;

static chassis_state_t chassis_state;
/* 只有上位机明确DISABLE会设置此位；上电不要求额外ENABLE。 */
static uint8_t chassis_disabled;
static uint32_t chassis_last_step_ms, chassis_last_speed_ms, chassis_last_adc_ms;
static uint32_t chassis_last_status_ms, chassis_last_odom_ms, chassis_uptime_ms;
static int32_t chassis_zero_left, chassis_zero_right;
static uint16_t chassis_last_command_sequence;
static uint8_t chassis_last_command_result;

static int16_t chassis_i16(float value)
{
  if(value > 32767.0f) return 32767;
  if(value < -32768.0f) return -32768;
  return (int16_t)value;
}

/* 仅由显式控制命令调用。清除旧Reference，阻止停止前的报文重新起步。 */
static void chassis_stop(chassis_state_t state)
{
  drive_control_stop();
  chassis_protocol_clear_reference();
  chassis_state = state;
}

static void chassis_handle_command(void)
{
  chassis_command_t command;
  if(!chassis_protocol_take_command(&command)) return;
  chassis_last_command_sequence = command.sequence;
  chassis_last_command_result = 0U;
  switch(command.type)
  {
    case CHASSIS_CMD_ENABLE:
      if(chassis_state == CHASSIS_ESTOP) chassis_last_command_result = 1U;
      else
      {
        chassis_disabled = 0U;
        chassis_stop(CHASSIS_ENABLED);
      }
      break;
    case CHASSIS_CMD_DISABLE:
      chassis_disabled = 1U;
      chassis_stop(chassis_state == CHASSIS_ESTOP ? CHASSIS_ESTOP : CHASSIS_IDLE);
      break;
    case CHASSIS_CMD_STOP:
      chassis_stop(chassis_state == CHASSIS_ESTOP ? CHASSIS_ESTOP : CHASSIS_IDLE);
      break;
    case CHASSIS_CMD_ESTOP:
      chassis_stop(CHASSIS_ESTOP);
      break;
    case CHASSIS_CMD_FAULT_RESET:
      /* 清除上位机ESTOP锁存和历史输入；传感器状态不参与命令准入。 */
      chassis_disabled = 0U;
      chassis_stop(CHASSIS_IDLE);
      (void)bumper_clear();
      canfd_driver_recover();
      break;
    case CHASSIS_CMD_RESET_ODOMETRY:
      motor_encoder_reset_all();
      drive_control_resync_feedback();
      chassis_last_speed_ms = chassis_uptime_ms;
      odometry_reset();
      break;
    default:
      chassis_last_command_result = 1U;
      break;
  }
}

static void chassis_send_status(void)
{
  chassis_status_t status;
  power_telemetry_t power;
  odometry_t odom;
  uint8_t imu_valid = imu_bmi270_fresh();
  power_telemetry_get(&power);
  odometry_get(&odom);
  memset(&status, 0, sizeof(status));
  status.state = (uint8_t)chassis_state;
  status.bumper_mask = bumper_latched();
  /* 沿用原故障位作为输入/采集诊断，只上报，不自动改变运动状态。 */
  status.fault = (status.bumper_mask ? 0x01U : 0U) |
      (chassis_state == CHASSIS_ESTOP ? 0x02U : 0U) |
      (!chassis_protocol_bus_healthy() ? 0x08U : 0U) |
      (!imu_valid ? 0x10U : 0U) | (!power.valid ? 0x40U : 0U);
  status.flags = (imu_valid ? 0x01U : 0U) |
      ((odom.valid && imu_valid) ? 0x02U : 0U);
  status.command_sequence = chassis_last_command_sequence;
  status.command_result = chassis_last_command_result;
  status.left_pwm = drive_control_pwm(MOTOR_LEFT);
  status.right_pwm = drive_control_pwm(MOTOR_RIGHT);
  status.bus_mv = power.bus_mv;
  status.left_current_ma = power.left_current_ma;
  status.right_current_ma = power.right_current_ma;
  chassis_protocol_send_status(&status, chassis_uptime_ms);
}

static void chassis_send_odometry(void)
{
  odometry_t odom;
  float left = drive_control_measured_rpm(MOTOR_LEFT) * BOARD_LEFT_TARGET_SIGN;
  float right = drive_control_measured_rpm(MOTOR_RIGHT) * BOARD_RIGHT_TARGET_SIGN;
  odometry_get(&odom);
  chassis_protocol_send_odometry(&odom,
      chassis_i16((left + right) * BOARD_WHEEL_CIRCUMFERENCE_MM / 120.0f),
      chassis_i16(imu_bmi270_gyro_z_dps() * 100.0f), chassis_uptime_ms);
}

void chassis_app_step(void)
{
  drive_reference_t reference = {0};
  adc_dma_sample_t adc;
  int32_t left = motor_encoder_get_count(MOTOR_LEFT);
  int32_t right = motor_encoder_get_count(MOTOR_RIGHT);
  float dt_s;
  chassis_uptime_ms = board_port_now_ms();
  dt_s = (chassis_uptime_ms - chassis_last_step_ms) / 1000.0f;
  if(!(dt_s > 0.0f)) return;
  chassis_last_step_ms = chassis_uptime_ms;
  chassis_protocol_poll(chassis_uptime_ms);
  bumper_update();
  power_telemetry_update(chassis_uptime_ms,
      adc_dma_take(&adc) ? &adc : 0,
      (chassis_state != CHASSIS_RUNNING && left == chassis_zero_left &&
       right == chassis_zero_right && drive_control_pwm(MOTOR_LEFT) == 0 &&
       drive_control_pwm(MOTOR_RIGHT) == 0) ? 1U : 0U);
  chassis_zero_left = left;
  chassis_zero_right = right;
  if(chassis_uptime_ms - chassis_last_adc_ms >= BOARD_ADC_PERIOD_MS)
  {
    chassis_last_adc_ms = chassis_uptime_ms;
    adc_dma_start();
  }
  (void)imu_bmi270_update(dt_s);
  chassis_handle_command();

  /* 新运动帧直接执行；断流保持最新目标，停止由上位机发命令。 */
  if(!chassis_disabled && chassis_state != CHASSIS_ESTOP &&
     chassis_protocol_get_reference(&reference))
  {
    chassis_state = (reference.linear_mm_s || reference.angular_mrad_s ||
        reference.accel_mm_s2 || reference.decel_mm_s2) ?
        CHASSIS_RUNNING : CHASSIS_ENABLED;
  }
  if(chassis_uptime_ms - chassis_last_speed_ms >= BOARD_SPEED_PERIOD_MS)
  {
    float speed_dt = (chassis_uptime_ms - chassis_last_speed_ms) / 1000.0f;
    chassis_last_speed_ms = chassis_uptime_ms;
    drive_control_update(&reference, speed_dt);
  }
  odometry_update(-motor_encoder_get_count(MOTOR_LEFT),
      -motor_encoder_get_count(MOTOR_RIGHT), imu_bmi270_gyro_z_dps(), dt_s);
  if(chassis_uptime_ms - chassis_last_status_ms >= BOARD_STATUS_PERIOD_MS)
  {
    chassis_last_status_ms = chassis_uptime_ms;
    chassis_send_status();
  }
  else if(chassis_uptime_ms - chassis_last_odom_ms >= BOARD_SPEED_PERIOD_MS)
  {
    chassis_last_odom_ms = chassis_uptime_ms;
    chassis_send_odometry();
  }
}

void chassis_app_init(void)
{
  drive_control_init();
  bumper_init();
  odometry_init();
  (void)adc_dma_init();
  (void)imu_bmi270_init();
  chassis_protocol_init();
  power_telemetry_init();
  chassis_state = CHASSIS_IDLE;
  chassis_disabled = 0U;
  chassis_last_command_sequence = 0U;
  chassis_last_command_result = 0U;
  chassis_zero_left = chassis_zero_right = 0;
  chassis_uptime_ms = board_port_now_ms();
  chassis_last_step_ms = chassis_last_speed_ms = chassis_last_adc_ms = chassis_uptime_ms;
  chassis_last_status_ms = chassis_last_odom_ms = chassis_uptime_ms;
  adc_dma_start();
}
