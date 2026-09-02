/**
 * @file drive_control.c
 * @brief 20ms车体速度到双AT8236 PWM的唯一闭环执行器。
 *
 * 应用主循环按实际经过时间调用：编码器测速→角速度补偿→差速换算→共同限幅/
 * 斜率→左右轮独立前馈+PID→PWM。停车期间仍调用，保持编码器差分基准连续。
 * 本模块锁存轮控故障并制动；通信TTL、IMU有效性和ENABLE准入由协议/应用层保证。
 */
#include "drive_control.h"

#include "board_config.h"
#include "motor_encoder.h"
#include <string.h>

/** @brief 一个电机的速度环状态；所有RPM已换算到该电机控制方向坐标。 */
typedef struct
{
  float target_rpm;         /**< 共同斜率之后、送入PID的有符号输出轴转速。 */
  float requested_rpm;      /**< 限速后但尚未过换向门/斜率的目标，用于判断换向。 */
  float measured_rpm;       /**< 对外发布的低通后RPM；正负与target_rpm保持一致。 */
  float filtered_rpm;       /**< 一阶低通的上一时刻状态，单位RPM。 */
  float integral;           /**< 速度误差的时间积分，单位RPM·s，未乘Ki。 */
  float previous_error;     /**< 上次速度误差，单位RPM，用于计算D项。 */
  int32_t last_count;       /**< 上次测速的累计编码器计数，不能在停车时停止更新。 */
  int16_t pwm;              /**< 最近一次输出的有符号PWM千分比，范围[-1000,1000]。 */
  uint16_t stall_ms;        /**< 目标有效但未按目标方向转动的连续时长，单位ms。 */
  uint8_t zero_ticks;       /**< 原始测速连续不超过1RPM的次数，最多记3次。 */
  uint8_t fault;            /**< 超速、无效输入或堵转锁存；停止命令不会清除此标志。 */
  int8_t last_sign;         /**< 最近非零目标方向；停车后保留，用于防止带速反转。 */
} drive_axis_t;

/* 下标与MOTOR_LEFT/MOTOR_RIGHT一致；仅由应用控制上下文更新。 */
static drive_axis_t drive_axis[2];
/* 斜率状态仍使用车体前进为正的轮速坐标，之后才施加安装方向符号。 */
static float drive_left_ramp;
static float drive_right_ramp;

/** @brief 返回浮点幅值，用于速度、限幅和斜率计算。 */
static float drive_abs(float value) { return (value < 0.0f) ? -value : value; }
/** @brief 将value对称限制到[-limit,+limit]；调用方提供非负limit。 */
static float drive_limit(float value, float limit)
{
  if(value > limit) return limit;
  if(value < -limit) return -limit;
  return value;
}

/**
 * @brief 求单轮本周期最多可以完成多少比例的目标变化。
 * @param current 当前斜率输出，单位RPM。
 * @param target 本周期希望达到的轮速，单位RPM。
 * @param up 允许的单周期加速量，单位RPM，已乘实际dt。
 * @param down 允许的单周期减速量，单位RPM，已乘实际dt。
 * @return [0,1]内的插值比例；两轮最终共用其中较小值。
 * 同向且幅值增大才使用up，其余变化使用down。目标已达到时返回1，不除以零。
 */
static float drive_fraction(float current, float target, float up, float down)
{
  float delta = drive_abs(target - current);
  float limit = (((current * target) >= 0.0f) &&
                 (drive_abs(target) > drive_abs(current))) ? up : down;
  return (delta > limit) ? limit / delta : 1.0f;
}

/** @brief 请求方向与上次非零方向相反、且尚无3次连续零速测量时，返回1阻止换向。 */
static uint8_t drive_reverse_blocked(const drive_axis_t *axis)
{
  int8_t requested_sign = (axis->requested_rpm > 0.0f) ? 1 :
      ((axis->requested_rpm < 0.0f) ? -1 : 0);
  return ((requested_sign != 0) && (axis->last_sign != 0) &&
          (requested_sign != axis->last_sign) && (axis->zero_ticks < 3U)) ? 1U : 0U;
}

/**
 * @brief 用累计编码器差分计算RPM，并更新低通测速与换向零速证据。
 * @param axis 本轮速度环状态，不可为空。
 * @param motor 左/右电机，用于读取计数并应用反馈安装符号。
 * @param dt_s 距本轮上次测速的真实秒数；调用方已保证0<dt_s<=0.1。
 * 原始RPM过大时锁存故障并保留上一次滤波值；零速门使用原始值，避免滤波延迟。
 */
static void drive_measure(drive_axis_t *axis, motor_id_t motor, float dt_s)
{
  int32_t count = motor_encoder_get_count(motor);
  int32_t delta = count - axis->last_count;
  float sign = (motor == MOTOR_LEFT) ? BOARD_LEFT_FEEDBACK_SIGN :
                                       BOARD_RIGHT_FEEDBACK_SIGN;
  float rpm = (float)delta * 60.0f /
      (BOARD_ENCODER_COUNTS_PER_REV * dt_s) * sign;

  axis->last_count = count;
  if(drive_abs(rpm) > BOARD_RAW_RPM_LIMIT)
  {
    axis->fault = 1U;
    return;
  }
  /* 固定系数一阶低通抑制计数离散噪声；系数不是姿态融合权重。 */
  axis->filtered_rpm += BOARD_RPM_FILTER_ALPHA * (rpm - axis->filtered_rpm);
  axis->measured_rpm = axis->filtered_rpm;
  if(drive_abs(rpm) <= 1.0f)
  {
    if(axis->zero_ticks < 3U) axis->zero_ticks++;
  }
  else axis->zero_ticks = 0U;
}

/**
 * @brief 对一个电机执行前馈+PID速度环，并检测持续堵转/反向反馈。
 * @param axis 包含本周期目标RPM、实测RPM和历史积分的状态，不可为空。
 * @param dt_s 实际测速/控制间隔，单位s，已由外层检查。
 * @return 有符号PWM千分比；故障或零目标返回0，实际制动由外层统一执行。
 * P补当前误差，I补持续负载，D使用误差差分；Kd当前为0但计算路径保留。
 * 此处只产生速度控制输出，ADC电流不参与本PID，即当前没有电流内环。
 */
static int16_t drive_pid(drive_axis_t *axis, float dt_s)
{
  float error;
  float derivative;
  float candidate;
  float correction;
  float feed_forward;
  float output;
  float forward;

  if((axis->fault != 0U) || (axis->target_rpm == 0.0f))
  {
    axis->integral = 0.0f;
    axis->previous_error = 0.0f;
    axis->stall_ms = 0U;
    return 0;
  }
  /* 将反馈投影到目标方向：持续反向转动也属于“没有按要求前进”。 */
  forward = (axis->target_rpm > 0.0f) ? axis->measured_rpm : -axis->measured_rpm;
  if((drive_abs(axis->target_rpm) >= BOARD_STALL_TARGET_RPM) &&
     (forward <= BOARD_STALL_MEASURED_RPM))
  {
    /* 累积真实毫秒，门限仍由名义周期×次数表示，避免调度抖动改变堵转时长。 */
    axis->stall_ms += (uint16_t)(dt_s * 1000.0f + 0.5f);
    if(axis->stall_ms >= BOARD_STALL_TICKS * BOARD_SPEED_PERIOD_MS)
    {
      axis->fault = 1U;
      return 0;
    }
  }
  else axis->stall_ms = 0U;

  error = axis->target_rpm - axis->measured_rpm;
  derivative = (error - axis->previous_error) / dt_s;
  /* Ki需大于0：积分状态按允许的PID修正量换算限幅，防止长时间积累。 */
  candidate = drive_limit(axis->integral + error * dt_s,
                          BOARD_PID_LIMIT / BOARD_PID_KI);
  correction = BOARD_PID_KP * error + BOARD_PID_KI * candidate +
      BOARD_PID_KD * derivative;
  /* 候选PID修正已饱和且误差还会加深饱和时，不接受本次积分。
   * 该抗饱和针对PID修正限幅；前馈相加后的总PWM还有独立的1000限幅。 */
  if(!(((correction > BOARD_PID_LIMIT) && (error > 0.0f)) ||
       ((correction < -BOARD_PID_LIMIT) && (error < 0.0f))))
    axis->integral = candidate;
  axis->previous_error = error;
  /* 静摩擦补偿+RPM线性前馈承担基础输出，PID只修正负载/电机差异。 */
  feed_forward = (axis->target_rpm > 0.0f) ? BOARD_FF_STATIC_PWM : -BOARD_FF_STATIC_PWM;
  output = feed_forward + axis->target_rpm * BOARD_FF_PER_RPM +
      drive_limit(BOARD_PID_KP * error + BOARD_PID_KI * axis->integral +
                  BOARD_PID_KD * derivative, BOARD_PID_LIMIT);
  /* 不允许PID修正把输出翻到目标反方向；反向行驶必须经过上层共同换向门。 */
  if(axis->target_rpm > 0.0f)
  {
    if(output < 0.0f) output = 0.0f;
    return (int16_t)drive_limit(output, 1000.0f);
  }
  if(output > 0.0f) output = 0.0f;
  return (int16_t)drive_limit(output, 1000.0f);
}

/**
 * @brief 检查车体速度和加减速度是否在本板支持范围内；有效返回1。
 * 四字段全零是立即制动的HOLD特例。普通零速度若带加减速度，仍走斜率控制。
 * 此函数只校验运动数值；Reference序号、TTL和ENABLE由协议/应用层检查。
 */
static uint8_t drive_reference_ok(const drive_reference_t *reference)
{
  if(reference == 0) return 0U;
  if((drive_abs((float)reference->linear_mm_s) > BOARD_MAX_LINEAR_MM_S) ||
     (drive_abs((float)reference->angular_mrad_s) > BOARD_MAX_ANGULAR_MRAD_S)) return 0U;
  if((reference->linear_mm_s == 0) && (reference->angular_mrad_s == 0) &&
     (reference->accel_mm_s2 == 0U) && (reference->decel_mm_s2 == 0U)) return 1U;
  return ((reference->accel_mm_s2 >= 10U) && (reference->accel_mm_s2 <= 500U) &&
          (reference->decel_mm_s2 >= 10U) && (reference->decel_mm_s2 <= 1000U)) ? 1U : 0U;
}

/** @brief 上电初始化编码器、PWM和双轮状态，清零计数后保持主动制动；启动阶段调用一次。 */
void drive_control_init(void)
{
  motor_encoder_init();
  motor_driver_init();
  motor_driver_stop_all(MOTOR_STOP_BRAKE);
  motor_encoder_reset_all();
  memset(drive_axis, 0, sizeof(drive_axis));
  drive_control_resync_feedback();
  drive_left_ramp = drive_right_ramp = 0.0f;
}

/**
 * @brief 将测速基准同步到当前编码器值，防止计数清零被当成一次巨大位移。
 * 应用在停止状态清编码器后调用，并同步自己的速度调度时间。
 * 清除测量滤波/零速证据，保留PID故障、目标及历史方向；本函数不输出PWM。
 */
void drive_control_resync_feedback(void)
{
  uint8_t index;
  for(index = 0U; index < 2U; index++)
  {
    drive_axis[index].last_count = motor_encoder_get_count((motor_id_t)index);
    drive_axis[index].filtered_rpm = 0.0f;
    drive_axis[index].measured_rpm = 0.0f;
    drive_axis[index].zero_ticks = 0U;
  }
}

/**
 * @brief 立即清除双轮目标、斜率和PID积累，并直接命令两个H桥主动制动。
 * 保留故障、编码器测速基准、实测速度和历史方向；停车后仍需周期调用update测速。
 */
void drive_control_stop(void)
{
  uint8_t index;
  for(index = 0U; index < 2U; index++)
  {
    drive_axis[index].target_rpm = 0.0f;
    drive_axis[index].requested_rpm = 0.0f;
    drive_axis[index].integral = 0.0f;
    drive_axis[index].previous_error = 0.0f;
    drive_axis[index].stall_ms = 0U;
    drive_axis[index].pwm = 0;
  }
  drive_left_ramp = drive_right_ramp = 0.0f;
  motor_driver_stop_all(MOTOR_STOP_BRAKE);
}

/** @brief 显式清除双轮故障后制动并重置测速基准；只能在应用确认复位条件满足后调用。 */
void drive_control_clear_fault(void)
{
  drive_axis[MOTOR_LEFT].fault = drive_axis[MOTOR_RIGHT].fault = 0U;
  drive_control_stop();
  drive_control_resync_feedback();
}

/**
 * @brief 用最新车体Reference完成一次双轮速度闭环；任一轮故障都会制动两轮。
 * @param reference 已通过通信时效和应用ENABLE准入检查的目标；四字段全零表示HOLD。
 * @param yaw_rate_mrad_s IMU实测车体角速度，单位mrad/s，左转为正，调用方保证有效。
 * @param dt_s 自上次调用的真实间隔，单位s，必须在(0,0.1]内。
 * @return 1表示本次有效（含HOLD）；0表示输入或轮控异常，已锁存故障并制动。
 * 名义每20ms调用，禁止只在RUNNING调用，否则停车期间编码器位移会堆到下一次。
 */
uint8_t drive_control_update(const drive_reference_t *reference,
                             float yaw_rate_mrad_s, float dt_s)
{
  float angular;
  float turn;
  float left;
  float right;
  float scale;
  float acceleration;
  float deceleration;
  float right_fraction;
  uint8_t index;

  if((drive_reference_ok(reference) == 0U) || !(dt_s > 0.0f && dt_s <= 0.1f))
  {
    drive_axis[MOTOR_LEFT].fault = 1U;
    drive_axis[MOTOR_RIGHT].fault = 1U;
  }
  if((dt_s > 0.0f) && (dt_s <= 0.1f))
  {
    drive_measure(&drive_axis[MOTOR_LEFT], MOTOR_LEFT, dt_s);
    drive_measure(&drive_axis[MOTOR_RIGHT], MOTOR_RIGHT, dt_s);
  }
  if(drive_control_has_fault() != 0U)
  {
    drive_control_stop();
    return 0U;
  }
  /* HOLD也持续测速；制动后不积分IMU偏差、不清除已锁存故障。 */
  if((reference->linear_mm_s == 0) && (reference->angular_mrad_s == 0) &&
     (reference->accel_mm_s2 == 0U) && (reference->decel_mm_s2 == 0U))
  {
    drive_control_stop();
    return 1U;
  }
  /* 角速度比例反馈修正的是转速误差，不是“保持某个绝对航向”的位置环。 */
  angular = (float)reference->angular_mrad_s + BOARD_YAW_RATE_GAIN *
      ((float)reference->angular_mrad_s - yaw_rate_mrad_s);
  angular = drive_limit(angular, BOARD_MAX_ANGULAR_MRAD_S);
  /* vL=v-L*ω/2，vR=v+L*ω/2；2000同时完成mrad→rad及二分轮距。 */
  turn = BOARD_TRACK_WIDTH_MM * angular / 2000.0f;
  left = ((float)reference->linear_mm_s - turn) * 60.0f / BOARD_WHEEL_CIRCUMFERENCE_MM;
  right = ((float)reference->linear_mm_s + turn) * 60.0f / BOARD_WHEEL_CIRCUMFERENCE_MM;
  scale = drive_abs(left);
  if(drive_abs(right) > scale) scale = drive_abs(right);
  /* 超过轮速上限时共同缩放左右目标，保持目标轮速比例而非单轮截断。 */
  if(scale > BOARD_MAX_WHEEL_RPM)
  {
    left *= BOARD_MAX_WHEEL_RPM / scale;
    right *= BOARD_MAX_WHEEL_RPM / scale;
  }
  drive_axis[MOTOR_LEFT].requested_rpm = left * BOARD_LEFT_TARGET_SIGN;
  drive_axis[MOTOR_RIGHT].requested_rpm = right * BOARD_RIGHT_TARGET_SIGN;
  /* 任一轮换向时两轮一起减到零，并等待实测连续三个零速样本。 */
  if(((left * drive_left_ramp) < 0.0f) || ((right * drive_right_ramp) < 0.0f) ||
     drive_reverse_blocked(&drive_axis[MOTOR_LEFT]) ||
     drive_reverse_blocked(&drive_axis[MOTOR_RIGHT]))
    left = right = 0.0f;
  /* 将mm/s²转换为本周期RPM增量，再用同一比例沿双轮目标矢量推进。 */
  acceleration = reference->accel_mm_s2 * dt_s * 60.0f / BOARD_WHEEL_CIRCUMFERENCE_MM;
  deceleration = reference->decel_mm_s2 * dt_s * 60.0f / BOARD_WHEEL_CIRCUMFERENCE_MM;
  scale = drive_fraction(drive_left_ramp, left, acceleration, deceleration);
  right_fraction = drive_fraction(drive_right_ramp, right, acceleration, deceleration);
  if(right_fraction < scale) scale = right_fraction;
  drive_left_ramp += (left - drive_left_ramp) * scale;
  drive_right_ramp += (right - drive_right_ramp) * scale;

  /* 安装符号仅在车体坐标→电机控制坐标的边界应用。 */
  drive_axis[MOTOR_LEFT].target_rpm = drive_left_ramp * BOARD_LEFT_TARGET_SIGN;
  drive_axis[MOTOR_RIGHT].target_rpm = drive_right_ramp * BOARD_RIGHT_TARGET_SIGN;
  if(drive_axis[MOTOR_LEFT].target_rpm > 0.0f) drive_axis[MOTOR_LEFT].last_sign = 1;
  if(drive_axis[MOTOR_LEFT].target_rpm < 0.0f) drive_axis[MOTOR_LEFT].last_sign = -1;
  if(drive_axis[MOTOR_RIGHT].target_rpm > 0.0f) drive_axis[MOTOR_RIGHT].last_sign = 1;
  if(drive_axis[MOTOR_RIGHT].target_rpm < 0.0f) drive_axis[MOTOR_RIGHT].last_sign = -1;
  drive_axis[MOTOR_LEFT].pwm = drive_pid(&drive_axis[MOTOR_LEFT], dt_s);
  drive_axis[MOTOR_RIGHT].pwm = drive_pid(&drive_axis[MOTOR_RIGHT], dt_s);
  if(drive_control_has_fault() != 0U)
  {
    drive_control_stop();
    return 0U;
  }
  for(index = 0U; index < 2U; index++)
  {
    if(drive_axis[index].target_rpm == 0.0f)
      motor_driver_stop((motor_id_t)index, MOTOR_STOP_BRAKE);
    else motor_driver_set_speed((motor_id_t)index, drive_axis[index].pwm);
  }
  return 1U;
}

/** @brief 任一轮锁存故障时返回1；不清除故障，也不区分具体故障来源。 */
uint8_t drive_control_has_fault(void)
{
  return (drive_axis[MOTOR_LEFT].fault | drive_axis[MOTOR_RIGHT].fault) != 0U;
}

/** @brief 返回指定电机最近的滤波RPM（电机控制坐标）；越过右轮枚举的编号返回0。 */
float drive_control_measured_rpm(motor_id_t motor)
{
  return (motor <= MOTOR_RIGHT) ? drive_axis[motor].measured_rpm : 0.0f;
}

/** @brief 返回指定电机最近一次有符号PWM千分比，供状态上报；越过右轮编号返回0。 */
int16_t drive_control_pwm(motor_id_t motor)
{
  return (motor <= MOTOR_RIGHT) ? drive_axis[motor].pwm : 0;
}
