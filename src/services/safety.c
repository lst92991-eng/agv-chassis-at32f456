/**
 * @file safety.c
 * @brief 简洁的电压、电流和本地故障锁存；故障只能显式清除。
 *
 * 消费ADC DMA快照，进行静止电流零点校准、电压/电流换算、超时与电气保护。
 * 其他故障由应用调用safety_raise汇总到同一位图，应用负责据此制动与状态迁移。
 * 全部接口由主循环调用；本模块不在ISR内操作PWM，也不负责读取防撞条/IMU。
 */
#include "safety.h"

#include "board_config.h"

static safety_status_t safety_status;
/* 电流零点为ADC原始计数，左右INA180分开校准。 */
static uint16_t safety_left_zero;
static uint16_t safety_right_zero;
/* 连续合格静止样本的累加器；未完成校准时遇到不合格样本会重新开始。 */
static uint32_t safety_left_sum;
static uint32_t safety_right_sum;
/* 最近一次接受新序号的时间来自主循环单调毫秒钟，不是DMA硬件采样时间戳。 */
static uint32_t safety_last_sample_ms;
static uint32_t safety_last_sequence;
static uint8_t safety_zero_count;
/* 任一侧超过峰值电流门限的连续时长；两侧都低于门限后清零，单位ms。 */
static uint32_t safety_peak_ms;

/** @brief 将换算量饱和到16位无符号范围，避免状态上报窄化时回绕。 */
static uint16_t safety_u16(uint32_t value)
{
  return (value > 65535UL) ? 65535U : (uint16_t)value;
}

/**
 * @brief 将INA180原始ADC值扣除静止零点后转换成mA，并做16位饱和。
 * @param raw 本次12位ADC原始计数。
 * @param zero 本侧静止零点计数。
 * @return 非负电流mA；低于零点按0处理，本通道不输出有符号回生电流。
 */
static uint16_t safety_current(uint16_t raw, uint16_t zero)
{
  return (raw > zero) ? safety_u16(((uint32_t)(raw - zero) *
      BOARD_CURRENT_UA_PER_COUNT) / 1000UL) : 0U;
}

/** @brief 上电清状态并开始等待ADC零点校准；now_ms为单调毫秒钟，未就绪时不能ENABLE。 */
void safety_init(uint32_t now_ms)
{
  safety_status.bus_mv = 0U;
  safety_status.left_current_ma = 0U;
  safety_status.right_current_ma = 0U;
  safety_status.fault = SAFETY_FAULT_NONE;
  safety_status.ready = 0U;
  safety_left_zero = safety_right_zero = 0U;
  safety_left_sum = safety_right_sum = 0UL;
  safety_last_sequence = 0UL;
  safety_last_sample_ms = now_ms;
  safety_zero_count = 0U;
  safety_peak_ms = 0U;
}

/**
 * @brief 处理一个ADC快照并维护采样超时、静止零点和电气故障锁存。
 * @param now_ms 当前真实单调毫秒数；使用无符号差值处理正常计时器回绕。
 * @param sample 最新完整DMA快照；为空、序号为0或重复时仅检查样本陈旧超时。
 * @param zero_allowed 应用确认禁止驱动且编码器静止时置1，允许累计电流零点。
 * 相同序号不能重复喂新鲜度或校准计数。校准完成后零点保持，不随运行电流漂移。
 * 仅维护故障和准入标志；发现故障后由应用统一主动制动。
 */
void safety_update(uint32_t now_ms, const adc_dma_sample_t *sample,
                   uint8_t zero_allowed)
{
  uint8_t zero_sample;
  uint32_t sample_ms;
  const uint32_t peak_limit_ms = BOARD_PEAK_CURRENT_TICKS * BOARD_ADC_PERIOD_MS;

  if((sample == 0) || (sample->sequence == 0U))
  {
    if((now_ms - safety_last_sample_ms) >= BOARD_ADC_STALE_MS)
      safety_raise(SAFETY_FAULT_POWER);
    return;
  }
  if(sample->sequence == safety_last_sequence)
  {
    if((now_ms - safety_last_sample_ms) >= BOARD_ADC_STALE_MS)
      safety_raise(SAFETY_FAULT_POWER);
    return;
  }
  /* 峰值电流使用实际新样本间隔累积；不能因为主循环重复读取同一值而重复计时。 */
  sample_ms = now_ms - safety_last_sample_ms;
  safety_last_sequence = sample->sequence;
  safety_last_sample_ms = now_ms;
  safety_status.bus_mv = safety_u16(((uint32_t)sample->bus_raw *
      BOARD_BUS_UV_PER_COUNT) / 1000UL + BOARD_BUS_OFFSET_MV);
  /* 只有静止、两侧接近零电流且母线电压正常的样本才能用于校准。
   * 避免把真实电机电流学成零点，掩盖随后出现的过流。 */
  zero_sample = ((zero_allowed != 0U) &&
      (sample->left_raw <= BOARD_ADC_ZERO_MAX_RAW) &&
      (sample->right_raw <= BOARD_ADC_ZERO_MAX_RAW) &&
      (safety_status.bus_mv >= BOARD_BUS_UV_TRIP_MV) &&
      (safety_status.bus_mv <= BOARD_BUS_OV_TRIP_MV)) ? 1U : 0U;
  if((safety_zero_count < BOARD_ADC_ZERO_SAMPLES) && (zero_sample != 0U))
  {
    safety_left_sum += sample->left_raw;
    safety_right_sum += sample->right_raw;
    if(++safety_zero_count == BOARD_ADC_ZERO_SAMPLES)
    {
      safety_left_zero = (uint16_t)(safety_left_sum / BOARD_ADC_ZERO_SAMPLES);
      safety_right_zero = (uint16_t)(safety_right_sum / BOARD_ADC_ZERO_SAMPLES);
    }
  }
  else if(safety_zero_count < BOARD_ADC_ZERO_SAMPLES)
  {
    safety_left_sum = safety_right_sum = 0UL;
    safety_zero_count = 0U;
  }
  /* 校准完成前仍计算并检查电流；完成后使用固定零点扣除测量偏置。 */
  safety_status.left_current_ma = safety_current(sample->left_raw, safety_left_zero);
  safety_status.right_current_ma = safety_current(sample->right_raw, safety_right_zero);
  if((safety_status.bus_mv < BOARD_BUS_UV_TRIP_MV) ||
     (safety_status.bus_mv > BOARD_BUS_OV_TRIP_MV)) safety_raise(SAFETY_FAULT_POWER);
  if((safety_status.left_current_ma >= BOARD_PEAK_CURRENT_MA) ||
     (safety_status.right_current_ma >= BOARD_PEAK_CURRENT_MA))
  {
    /* 任一侧持续超限达到名义5×20ms门限才锁存；单次间隔最多加一个门限。 */
    if(safety_peak_ms < peak_limit_ms)
      safety_peak_ms += (sample_ms < peak_limit_ms) ? sample_ms : peak_limit_ms;
    if(safety_peak_ms >= peak_limit_ms) safety_raise(SAFETY_FAULT_POWER);
  }
  else safety_peak_ms = 0U;
  safety_status.ready = ((safety_zero_count == BOARD_ADC_ZERO_SAMPLES) &&
                         (safety_status.fault == SAFETY_FAULT_NONE)) ? 1U : 0U;
}

/** @brief 按位锁存一个或多个本地故障并禁止ENABLE；不会覆盖之前已记录的故障。 */
void safety_raise(uint8_t fault)
{
  safety_status.fault |= fault;
  safety_status.ready = 0U;
}

/** @brief 返回电流零点完成且当前没有故障的准入标志；外部IMU/防撞/总线仍由应用检查。 */
uint8_t safety_can_enable(void)
{
  return safety_status.ready;
}

/** @brief 返回已锁存故障位图，读取不会清除任何位。 */
uint8_t safety_fault(void)
{
  return safety_status.fault;
}

/**
 * @brief 在ADC新鲜、电压进入恢复区间且两侧电流低于门限后显式清除故障。
 * @param now_ms 当前单调毫秒数，用于检查最近样本是否陈旧。
 * @return 1表示已清锁存位，0表示电气恢复条件不满足；1不等于校准已完成。
 * 本函数会清全部故障位，因此调用前应用必须另外确认IMU、防撞、CAN均已恢复。
 * 欠压/过压使用比触发区间更窄的恢复区间，避免阈值附近反复允许复位。
 */
uint8_t safety_clear(uint32_t now_ms)
{
  if((safety_last_sequence == 0U) ||
     ((now_ms - safety_last_sample_ms) >= BOARD_ADC_STALE_MS) ||
     (safety_status.bus_mv < BOARD_BUS_UV_RECOVER_MV) ||
     (safety_status.bus_mv > BOARD_BUS_OV_RECOVER_MV) ||
     (safety_status.left_current_ma >= BOARD_PEAK_CURRENT_MA) ||
     (safety_status.right_current_ma >= BOARD_PEAK_CURRENT_MA)) return 0U;
  safety_status.fault = SAFETY_FAULT_NONE;
  safety_status.ready = (safety_zero_count == BOARD_ADC_ZERO_SAMPLES) ? 1U : 0U;
  return 1U;
}

/** @brief 将电压、电流、故障和准入状态复制给应用；空指针忽略，与update同上下文调用。 */
void safety_get(safety_status_t *status)
{
  if(status != 0) *status = safety_status;
}
