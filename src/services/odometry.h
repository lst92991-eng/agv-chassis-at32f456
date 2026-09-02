/** @file odometry.h @brief 编码器与BMI270组成的平面相对里程计。 */
#ifndef ODOMETRY_H
#define ODOMETRY_H

#include <stdint.h>

/** @brief 从最近一次reset原点累计的二维相对位姿；不包含Roll/Pitch或绝对定位。 */
typedef struct
{
  float x_mm;                 /**< 起始坐标系X方向位移，mm，初始前进方向为正。 */
  float y_mm;                 /**< 起始坐标系Y方向位移，mm，初始左侧方向为正。 */
  float yaw_deg;              /**< 相对航向，度，左转为正，对外范围[-180,180]。 */
  uint8_t valid;              /**< 最近一次时间步更新有效；不代替外部IMU新鲜度检查。 */
  uint8_t encoder_rejected;   /**< 本次reset后曾拒绝过编码器角速度，锁存到下次reset。 */
} odometry_t;

/** @brief 初始化相对原点，初始valid=0；应用同时使输入编码器计数归零。 */
void odometry_init(void);
/** @brief 清位置、航向和异常历史；应用必须同步清编码器并重置轮控测速基准。 */
void odometry_reset(void);
/**
 * @brief 主循环输入累计编码器及IMU数据，更新相对位姿。
 * @param left_count 左轮累计计数，前进增加，复位后从0计数。
 * @param right_count 右轮累计计数，前进增加，复位后从0计数。
 * @param gyro_z_dps 有效Z轴角速度，度/s，左转为正。
 * @param dt_s 真实测量间隔，s；允许0.001～0.100，越界会令valid=0。
 * 编码器转角异常仅取消本次航向修正；未提供平移打滑修正或绝对航向约束。
 */
void odometry_update(int32_t left_count, int32_t right_count,
                     float gyro_z_dps, float dt_s);
/** @brief 将最新状态复制到odom；空指针忽略，调用上下文应与update一致。 */
void odometry_get(odometry_t *odom);

#endif
