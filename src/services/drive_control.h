/** @file drive_control.h @brief 差速、双轮速度PID与显式制动，无故障锁存。 */
#ifndef DRIVE_CONTROL_H
#define DRIVE_CONTROL_H
#include <stdint.h>
#include "motor_driver.h"
typedef struct
{
  int16_t linear_mm_s;       /* 车体线速度，前进为正。 */
  int16_t angular_mrad_s;    /* 车体角速度，左转为正。 */
  uint16_t accel_mm_s2;      /* 加速斜率，0表示直接更新。 */
  uint16_t decel_mm_s2;      /* 减速斜率，0表示直接更新。 */
} drive_reference_t;
void drive_control_init(void);
void drive_control_stop(void);
void drive_control_resync_feedback(void);
/* 按实际dt执行；速度范围与运动策略由上位机负责，PWM保持硬件千分比范围。 */
void drive_control_update(const drive_reference_t *reference, float dt_s);
float drive_control_measured_rpm(motor_id_t motor);
int16_t drive_control_pwm(motor_id_t motor);
#endif
