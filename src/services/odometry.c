/**
 * @file odometry.c
 * @brief 陀螺仪预测航向、编码器低频修正并积分x/y。
 *
 * 此模块只维护本次复位起点下的平面相对位姿：X向前、Y向左、左转Yaw为正。
 * 编码器给出中心轮程与差分转角，陀螺仪给出快速转角预测；融合不是绝对定位。
 * 不读硬件、不控制电机，主循环负责提供方向已统一且时间对齐的计数与IMU数据。
 */
#include "odometry.h"

#include <math.h>

#include "board_config.h"

/* 内部累计航向使用度；车轮差分几何及sin/cos使用弧度，边界显式转换。 */
#define ODOMETRY_RAD_TO_DEG 57.2957795f
#define ODOMETRY_DEG_TO_RAD 0.01745329252f
/* 编码器累计航向的低频纠偏时间常数；不是陀螺仪硬件滤波频率。 */
#define ODOMETRY_CORRECTION_TAU_S 2.0f
/* 只拒绝明显异常的编码器角速度，不等同于完整的轮胎打滑检测。 */
#define ODOMETRY_MAX_ENCODER_RATE_DPS 120.0f

/* 所有状态由主循环单一上下文访问；get提供值拷贝，没有ISR同步锁。 */
static odometry_t odometry_state;
/* 累计计数的上一测量点，reset将它们置零，需与外部计数清零配套。 */
static int32_t odometry_last_left;
static int32_t odometry_last_right;
/* 连续累计角不做±180度折返，避免跨界时融合误差突然跳360度。 */
static float odometry_yaw_unwrapped;
static float odometry_encoder_yaw;

/** @brief 返回角速度幅值，供编码器异常门使用。 */
static float odometry_abs(float value) { return (value < 0.0f) ? -value : value; }
/** @brief 将对外发布的Yaw归一到[-180,180]度；内部积分仍保留连续累计角。 */
static float odometry_wrap(float yaw)
{
  while(yaw > 180.0f) yaw -= 360.0f;
  while(yaw < -180.0f) yaw += 360.0f;
  return yaw;
}

/** @brief 初始化相对里程计，效果等同reset；外部编码器应处于配套的零点。 */
void odometry_init(void)
{
  odometry_reset();
}

/**
 * @brief 将当前位置定义为新原点，清相对航向与异常历史，等待新样本建立有效输出。
 * 同时把上次累计计数设为0；应用必须同步清零/归零输入计数，不能只重置此模块。
 * 本函数不接触编码器硬件，也不更改轮控测速基准。
 */
void odometry_reset(void)
{
  odometry_state.x_mm = 0.0f;
  odometry_state.y_mm = 0.0f;
  odometry_state.yaw_deg = 0.0f;
  odometry_state.valid = 0U;
  odometry_state.encoder_rejected = 0U;
  odometry_last_left = odometry_last_right = 0;
  odometry_yaw_unwrapped = odometry_encoder_yaw = 0.0f;
}

/**
 * @brief 将一次轮程和陀螺仪角增量融合成相对x/y/yaw。
 * @param left_count 左轮累计计数，已统一为车体前进时增加。
 * @param right_count 右轮累计计数，已统一为车体前进时增加。
 * @param gyro_z_dps 本周期Z轴角速度，度/s，左转为正，调用方保证样本有效。
 * @param dt_s 本次测量间隔，单位s；小于1ms或大于100ms时只置valid=0并返回。
 * 无效时间不更新计数基准。编码器角速度异常时只跳过该次Yaw修正，轮程仍参与XY。
 */
void odometry_update(int32_t left_count, int32_t right_count,
                     float gyro_z_dps, float dt_s)
{
  int32_t left_delta;
  int32_t right_delta;
  float distance;
  float encoder_delta_yaw;
  float predicted_yaw;
  float encoder_rate;
  float yaw_before;
  float yaw_delta;
  float midpoint;

  if((dt_s < 0.001f) || (dt_s > 0.100f))
  {
    odometry_state.valid = 0U;
    return;
  }
  left_delta = left_count - odometry_last_left;
  right_delta = right_count - odometry_last_right;
  odometry_last_left = left_count;
  odometry_last_right = right_count;
  yaw_before = odometry_yaw_unwrapped;
  /* 短时航向变化首先相信陀螺仪；轮差分给出低频约束，抑制陀螺仪零偏积分。 */
  predicted_yaw = yaw_before + gyro_z_dps * dt_s;
  encoder_delta_yaw = ((float)(right_delta - left_delta) *
      BOARD_WHEEL_CIRCUMFERENCE_MM / (BOARD_ENCODER_COUNTS_PER_REV *
      BOARD_TRACK_WIDTH_MM)) * ODOMETRY_RAD_TO_DEG;
  encoder_rate = encoder_delta_yaw / dt_s;
  if(odometry_abs(encoder_rate) <= ODOMETRY_MAX_ENCODER_RATE_DPS)
  {
    /* 与实际dt匹配的一阶互补修正：陀螺仪负责预测，编码器累计角缓慢拉回。 */
    float gain = dt_s / (ODOMETRY_CORRECTION_TAU_S + dt_s);
    odometry_encoder_yaw += encoder_delta_yaw;
    odometry_yaw_unwrapped = predicted_yaw + gain *
        (odometry_encoder_yaw - predicted_yaw);
  }
  else
  {
    odometry_yaw_unwrapped = predicted_yaw;
    /* 历史标志会保持到reset；下一次正常数据不会自动清除它。 */
    odometry_state.encoder_rejected = 1U;
  }
  yaw_delta = odometry_yaw_unwrapped - yaw_before;
  /* 用周期中点航向分配本周期中心位移，降低圆弧运动中的离散积分误差。 */
  midpoint = (yaw_before + yaw_delta * 0.5f) * ODOMETRY_DEG_TO_RAD;
  distance = ((float)(left_delta + right_delta) * 0.5f) *
      BOARD_WHEEL_CIRCUMFERENCE_MM / BOARD_ENCODER_COUNTS_PER_REV;
  odometry_state.x_mm += distance * cosf(midpoint);
  odometry_state.y_mm += distance * sinf(midpoint);
  odometry_state.yaw_deg = odometry_wrap(odometry_yaw_unwrapped);
  /* valid表示本周期数学更新完成，不保证没有打滑，也不代表绝对位置可信。 */
  odometry_state.valid = 1U;
}

/** @brief 拷贝最新位姿和质量标志；输出指针为空时不操作，在主循环上下文调用。 */
void odometry_get(odometry_t *odom)
{
  if(odom != 0) *odom = odometry_state;
}
