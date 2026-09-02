/**
 * @file imu_bmi270.c
 * @brief SPI3传输、BMI270初始化、静止零偏和Z轴角速度。
 *
 * Bosch SensorAPI负责寄存器、配置文件装载和数据解码；本文件实现板级SPI回调、
 * 上电零偏和角速度单位换算。正常由主循环调用，不在中断内读传感器。
 * 输出是车体Z轴相对角速度，不是姿态角；航向和XY积分由里程计服务完成。
 */
#include "imu_bmi270.h"

#include <string.h>

#include "at32f45x.h"
#include "at32f45x_board.h"
#include "at32f45x_crm.h"
#include "at32f45x_gpio.h"
#include "at32f45x_spi.h"
#include "bmi270.h"
#include "motor_encoder.h"

#define IMU_SPI_TIMEOUT 10000UL  /**< 每个SPI状态的最大轮询次数，不是微秒数。 */
#define IMU_BIAS_SAMPLES 64U     /**< 启动校准要求收齐的有效静止样本数。 */

/* 单主循环持有这些状态；imu_ok记录初始化成功，age记录最近有效陀螺样本的新鲜度。 */
static struct bmi2_dev imu_device;
static float imu_bias_dps;
static float imu_rate_dps;
static uint16_t imu_age_ms;
static uint8_t imu_ok;

/**
 * @brief 控制PA15软件片选，拉低时开始一次SPI事务。
 * @param high 0表示选中BMI270，非0表示释放片选。
 * @note 每条失败路径也必须释放片选，以便下一周期重新读取。
 */
static void imu_cs(uint8_t high)
{
  /* 上次传输超时后可能迟到一个字节，新事务开始前丢弃，避免读帧错位。 */
  if((high == 0U) && (spi_i2s_flag_get(SPI3, SPI_I2S_RDBF_FLAG) != RESET))
    (void)spi_i2s_data_receive(SPI3);
  gpio_bits_write(GPIOA, GPIO_PINS_15, (high != 0U) ? TRUE : FALSE);
}

/**
 * @brief 有界等待SPI3发送/接收状态，交换一个8位字节。
 * @param tx 要发送的字节；读数据阶段发送0产生时钟。
 * @param rx 接收字节输出，内部调用保证非空。
 * @return 1表示字节交换完成，0表示发送或接收等待超时。
 * @note 同步等待只覆盖单字节硬件传输，超时不允许无符号计数器下溢。
 */
static uint8_t imu_transfer(uint8_t tx, uint8_t *rx)
{
  uint32_t timeout = IMU_SPI_TIMEOUT;
  while((spi_i2s_flag_get(SPI3, SPI_I2S_TDBE_FLAG) == RESET) && (timeout != 0U)) { timeout--; }
  if(timeout == 0U) return 0U;
  spi_i2s_data_transmit(SPI3, tx);
  timeout = IMU_SPI_TIMEOUT;
  while((spi_i2s_flag_get(SPI3, SPI_I2S_RDBF_FLAG) == RESET) && (timeout != 0U)) { timeout--; }
  if(timeout == 0U) return 0U;
  *rx = (uint8_t)spi_i2s_data_receive(SPI3);
  return 1U;
}

/**
 * @brief Bosch读回调：发送寄存器地址后，连续接收指定长度的数据。
 * @param reg 官方库已加上SPI读标志的寄存器地址。
 * @param data 官方库提供的接收缓冲区。
 * @param length 官方库要求传输的字节数，普通寄存器读取已经包含dummy字节。
 * @param context 官方接口上下文；本板只有一个SPI3设备，因此不使用。
 * @return 官方接口成功码或BMI2_E_COM_FAIL。
 * @note dummy字节由SensorAPI在返回后剔除，此回调不能再额外跳过一个字节。
 */
static BMI2_INTF_RETURN_TYPE imu_read(uint8_t reg, uint8_t *data,
                                      uint32_t length, void *context)
{
  uint32_t index;
  uint8_t value;
  UNUSED(context);
  imu_cs(0U);
  if(imu_transfer(reg, &value) == 0U) { imu_cs(1U); return BMI2_E_COM_FAIL; }
  for(index = 0U; index < length; index++)
  {
    if(imu_transfer(0U, &data[index]) == 0U) { imu_cs(1U); return BMI2_E_COM_FAIL; }
  }
  imu_cs(1U);
  return BMI2_INTF_RET_SUCCESS;
}

/**
 * @brief Bosch写回调：片选保持有效，依次发送地址和完整数据块。
 * @param reg 官方库处理后的写寄存器地址。
 * @param data 待写数据，生命周期由调用方保证。
 * @param length 数据字节数，不包含地址字节。
 * @param context 本板未使用的官方接口上下文。
 * @return 任意字节超时即返回BMI2_E_COM_FAIL，否则返回成功码。
 */
static BMI2_INTF_RETURN_TYPE imu_write(uint8_t reg, const uint8_t *data,
                                       uint32_t length, void *context)
{
  uint32_t index;
  uint8_t value;
  UNUSED(context);
  imu_cs(0U);
  if(imu_transfer(reg, &value) == 0U) { imu_cs(1U); return BMI2_E_COM_FAIL; }
  for(index = 0U; index < length; index++)
  {
    if(imu_transfer(data[index], &value) == 0U) { imu_cs(1U); return BMI2_E_COM_FAIL; }
  }
  imu_cs(1U);
  return BMI2_INTF_RET_SUCCESS;
}

/**
 * @brief 将SensorAPI所需的微秒延时转交板级延时函数。
 * @param period_us 最短等待时间，单位us；达到1ms后向上取整到毫秒。
 * @param context 本板未使用的官方接口上下文。
 * @note 这是同步延时，主要服务启动/配置时序，不能在ISR中调用。
 */
static void imu_delay(uint32_t period_us, void *context)
{
  UNUSED(context);
  if(period_us >= 1000U) delay_ms((uint16_t)((period_us + 999U) / 1000U));
  else if(period_us != 0U) delay_us(period_us);
}

/**
 * @brief 初始化BMI270连接的SPI3和独立片选GPIO。
 *
 * PC10/PC11/PC12复用为SCK/MISO/MOSI，PA15软件片选；8位、MSB优先、
 * SPI mode 0、主机时钟32分频。片选初始为高，避免配置外设时选中传感器。
 */
static void imu_spi_init(void)
{
  gpio_init_type gpio;
  spi_init_type spi;
  crm_periph_clock_enable(CRM_GPIOA_PERIPH_CLOCK, TRUE);
  crm_periph_clock_enable(CRM_GPIOC_PERIPH_CLOCK, TRUE);
  crm_periph_clock_enable(CRM_SPI3_PERIPH_CLOCK, TRUE);
  gpio_default_para_init(&gpio);
  gpio.gpio_pins = GPIO_PINS_10 | GPIO_PINS_11 | GPIO_PINS_12;
  gpio.gpio_mode = GPIO_MODE_MUX;
  gpio.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
  gpio.gpio_pull = GPIO_PULL_NONE;
  gpio.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
  gpio_init(GPIOC, &gpio);
  gpio_pin_mux_config(GPIOC, GPIO_PINS_SOURCE10, GPIO_MUX_6);
  gpio_pin_mux_config(GPIOC, GPIO_PINS_SOURCE11, GPIO_MUX_6);
  gpio_pin_mux_config(GPIOC, GPIO_PINS_SOURCE12, GPIO_MUX_6);
  gpio.gpio_pins = GPIO_PINS_15;
  gpio.gpio_mode = GPIO_MODE_OUTPUT;
  gpio_init(GPIOA, &gpio);
  imu_cs(1U);
  spi_default_para_init(&spi);
  spi.transmission_mode = SPI_TRANSMIT_FULL_DUPLEX;
  spi.master_slave_mode = SPI_MODE_MASTER;
  spi.mclk_freq_division = SPI_MCLK_DIV_32;
  spi.first_bit_transmission = SPI_FIRST_BIT_MSB;
  spi.frame_bit_num = SPI_FRAME_8BIT;
  spi.clock_polarity = SPI_CLOCK_POLARITY_LOW;
  spi.clock_phase = SPI_CLOCK_PHASE_1EDGE;
  spi.cs_mode_selection = SPI_CS_SOFTWARE_MODE;
  spi_init(SPI3, &spi);
  spi_software_cs_internal_level_set(SPI3, SPI_SWCS_INTERNAL_LEVEL_HIGHT);
  spi_enable(SPI3, TRUE);
}

/**
 * @brief 通过官方库配置200Hz加速度计/陀螺仪并开启采样。
 * @return 1表示全部官方调用成功，0表示读配置、写配置或使能失败。
 * @note 必须先填写config[].type再读取配置。±2g和±2000°/s量程
 *       与下方零偏、角速度换算系数绑定；AVG4/Normal是芯片内部滤波。
 */
static uint8_t imu_configure(void)
{
  struct bmi2_sens_config config[2] = {0};
  uint8_t sensors[2] = {BMI2_ACCEL, BMI2_GYRO};
  config[0].type = BMI2_ACCEL;
  config[1].type = BMI2_GYRO;
  if(bmi2_get_sensor_config(config, 2U, &imu_device) != BMI2_OK) return 0U;
  config[0].cfg.acc.odr = BMI2_ACC_ODR_200HZ;
  config[0].cfg.acc.range = BMI2_ACC_RANGE_2G;
  config[0].cfg.acc.bwp = BMI2_ACC_NORMAL_AVG4;
  config[0].cfg.acc.filter_perf = BMI2_PERF_OPT_MODE;
  config[1].cfg.gyr.odr = BMI2_GYR_ODR_200HZ;
  config[1].cfg.gyr.range = BMI2_GYR_RANGE_2000;
  config[1].cfg.gyr.bwp = BMI2_GYR_NORMAL_MODE;
  config[1].cfg.gyr.filter_perf = BMI2_PERF_OPT_MODE;
  if(bmi2_set_sensor_config(config, 2U, &imu_device) != BMI2_OK) return 0U;
  return (bmi2_sensor_enable(sensors, 2U, &imu_device) == BMI2_OK) ? 1U : 0U;
}

/**
 * @brief 上电静止校准，取64个有效样本的Z轴角速度平均值作为零偏。
 *
 * 每5ms尝试一次，最多200次；只有加速度和陀螺仪同时data-ready才计入样本。
 * 编码器必须保持不变，重力模长须在0.7～1.3g，Z轴角速度标准差不得超过1°/s。
 * 这些是校准质量门，无法代替外部绝对姿态基准；调用时车辆必须实际静止。
 * @return 1表示样本收齐且质量门通过，0表示运动、振动、异常数据或样本不足。
 * @note 本函数仅在启动期间阻塞；64个样本至少需要约320ms加通信时间。
 */
static uint8_t imu_calibrate(void)
{
  struct bmi2_sens_data data;
  uint16_t count = 0U;
  uint16_t tries = 0U;
  float sum = 0.0f;
  float square_sum = 0.0f;
  float rate, gravity_squared;
  int32_t left = motor_encoder_get_count(MOTOR_LEFT);
  int32_t right = motor_encoder_get_count(MOTOR_RIGHT);
  while((count < IMU_BIAS_SAMPLES) && (tries++ < 200U))
  {
    delay_ms(5U);
    if((motor_encoder_get_count(MOTOR_LEFT) != left) ||
       (motor_encoder_get_count(MOTOR_RIGHT) != right)) return 0U;
    if((bmi2_get_sensor_data(&data, &imu_device) == BMI2_OK) &&
       ((data.status & (BMI2_DRDY_GYR | BMI2_DRDY_ACC)) == (BMI2_DRDY_GYR | BMI2_DRDY_ACC)))
    {
      /* 静止零偏只接收0.7～1.3g样本；单位g²，无需开方。 */
      gravity_squared = ((float)data.acc.x * data.acc.x + (float)data.acc.y * data.acc.y +
                         (float)data.acc.z * data.acc.z) / (16384.0f * 16384.0f);
      if((gravity_squared < 0.49f) || (gravity_squared > 1.69f)) return 0U;
      rate = (float)data.gyr.z * 2000.0f / 32768.0f;
      sum += rate;
      square_sum += rate * rate;
      count++;
    }
  }
  if(count != IMU_BIAS_SAMPLES) return 0U;
  imu_bias_dps = sum / (float)count;
  /* 与原标定一致：标准差大于1°/s说明校准期间不稳定。 */
  if(square_sum / (float)count - imu_bias_dps * imu_bias_dps > 1.0f) return 0U;
  imu_rate_dps = 0.0f;
  return 1U;
}

/**
 * @brief 绑定板级回调，调用Bosch初始化、配置和静止零偏校准。
 * @return 1表示IMU可用，0表示启动失败；上层据此锁存IMU故障。
 * @note 编码器应已初始化且车轮静止。初始化失败会使后续update直接失败；
 *       本驱动不会在运行中自动重新初始化或重新校准。
 */
uint8_t imu_bmi270_init(void)
{
  imu_ok = 0U;
  memset(&imu_device, 0, sizeof(imu_device));
  imu_spi_init();
  imu_device.intf = BMI2_SPI_INTF;
  imu_device.read = imu_read;
  imu_device.write = imu_write;
  imu_device.delay_us = imu_delay;
  imu_device.read_write_len = 32U;
  /* 告知官方层BMI270 SPI读含1个dummy字节；传输回调只忠实搬运。 */
  imu_device.dummy_byte = 1U;
  imu_ok = ((bmi270_init(&imu_device) == BMI2_OK) &&
            (imu_configure() != 0U) && (imu_calibrate() != 0U)) ? 1U : 0U;
  imu_age_ms = 0U;
  return imu_ok;
}

/**
 * @brief 读取最近陀螺仪样本，扣除零偏、低通并更新新鲜度。
 * @param dt_s 距上次主循环的实际秒数，由真实毫秒时钟计算，正常约0.010s。
 * @return 1表示初始化成功且有效陀螺样本未陈旧，0表示通信失败或已陈旧。
 * @note 芯片200Hz采样，但主循环约100Hz读取最新值，不读取FIFO补齐每个样本。
 *       运行中通信失败仍保留imu_ok，下一周期可重读；应用层故障不会自动清除。
 */
uint8_t imu_bmi270_update(float dt_s)
{
  struct bmi2_sens_data data;
  uint16_t elapsed = (uint16_t)(dt_s * 1000.0f + 0.5f);
  float raw;
  if(imu_ok == 0U) return 0U;
  if(bmi2_get_sensor_data(&data, &imu_device) != BMI2_OK)
  {
    /* 保留初始化结果；下一周期可以重读，但故障复位仍由应用层决定。 */
    imu_age_ms = 65535U;
    return 0U;
  }
  if((data.status & BMI2_DRDY_GYR) != 0U)
  {
    raw = (float)data.gyr.z * 2000.0f / 32768.0f - imu_bias_dps;
    /* 固定每个有效样本0.2的一阶低通；并非基于dt自适应的滤波或姿态解算。 */
    imu_rate_dps += 0.2f * (raw - imu_rate_dps);
    imu_age_ms = 0U;
  }
  else if((uint32_t)imu_age_ms + elapsed < 65535UL) imu_age_ms += elapsed;
  else imu_age_ms = 65535U;
  return imu_bmi270_fresh();
}

/** @brief 返回最近滤波Z轴角速度，单位°/s；使用者同时检查fresh。 */
float imu_bmi270_gyro_z_dps(void) { return imu_rate_dps; }
/** @brief 初始化成功且样本年龄小于50ms时返回1，否则返回0。 */
uint8_t imu_bmi270_fresh(void) { return (imu_ok != 0U) && (imu_age_ms < 50U); }
