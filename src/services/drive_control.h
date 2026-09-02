/** @file drive_control.h @brief 差速换算、双轮速度PID和主动制动。 */
#ifndef DRIVE_CONTROL_H
#define DRIVE_CONTROL_H

#include <stdint.h>

#include "motor_driver.h"

/** @brief 车体坐标下的瞬时运动目标；有效期和序号由协议层另行管理。 */
typedef struct
{
  int16_t linear_mm_s;       /**< 中心线速度，mm/s，前进为正。 */
  int16_t angular_mrad_s;    /**< 车体角速度，mrad/s，左转为正。 */
  uint16_t accel_mm_s2;     /**< 轮线速度加速上限，mm/s²；普通目标允许10～500。 */
  uint16_t decel_mm_s2;     /**< 轮线速度减速上限，mm/s²；普通目标允许10～1000。 */
} drive_reference_t;

/** @brief 上电初始化编码器与PWM、清编码器并保持制动；应用启动时调用一次。 */
void drive_control_init(void);
/** @brief 双轮立即主动制动并清PID/斜率；保留故障和测速基准，后续仍须update。 */
void drive_control_stop(void);
/** @brief 应用确认复位条件后调用：清轮控故障、主动制动、重新同步测速。 */
void drive_control_clear_fault(void);
/** @brief 编码器清零后同步测量基准/滤波；保留故障与换向历史，调用方同步控制时间。 */
void drive_control_resync_feedback(void);
/**
 * @brief 执行一次差速与双轮速度闭环，名义每20ms调用，停车时也必须持续调用。
 * @param reference 四字段全零表示HOLD主动制动；普通目标已通过序号/TTL及应用ENABLE准入。
 * @param yaw_rate_mrad_s 有效的IMU实测角速度，mrad/s，左转为正。
 * @param dt_s 距上次调用的实际秒数，范围(0,0.1]，不能用名义周期代替。
 * @return 1：目标处理成功，包含HOLD；0：输入/轮控故障，双轮已主动制动。
 */
uint8_t drive_control_update(const drive_reference_t *reference,
                             float yaw_rate_mrad_s, float dt_s);
/** @brief 返回任一轮是否锁存故障；只有clear_fault会显式清除。 */
uint8_t drive_control_has_fault(void);
/** @brief 返回最近滤波RPM，正负为电机控制坐标；转换成车体坐标需乘对应TARGET_SIGN。 */
float drive_control_measured_rpm(motor_id_t motor);
/** @brief 返回最近一次有符号PWM千分比[-1000,1000]；零目标由驱动另行设置主动制动。 */
int16_t drive_control_pwm(motor_id_t motor);

#endif
