/**
 * @file chassis_app.c
 * @brief 一条可讲清的控制链：CAN→安全→里程计→差速PID→PWM。
 *
 * FreeRTOS唯一控制任务约每10ms调用一次，本文件是底盘状态的唯一应用层写者。
 * 协议只提供管理命令和车体v/omega，轮控负责RPM/PWM计算；本层决定何时允许执行。
 * 传感、通信或运动故障在本地制动并锁存，上位机必须显式复位和重新ENABLE。
 */
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
#include "safety.h"

/**
 * @brief 内部状态，同时作为0x181的state字段。
 * ENABLED允许等待新Reference；RUNNING表示正在执行已接受的非HOLD目标。
 * STOP/DISABLE回到DISABLED；FAULT_RESET成功也回到DISABLED，不自动续跑。
 */
typedef enum
{
  CHASSIS_DISABLED = 0, /**< 禁止正常运动，仍持续采样、测速和发送状态。 */
  CHASSIS_ENABLED,      /**< 准入已通过，等待新Reference或保持HOLD。 */
  CHASSIS_RUNNING,      /**< 有效非HOLD Reference由轮控周期执行。 */
  CHASSIS_FAULT         /**< 故障锁存，主动制动，等待原因恢复和显式复位。 */
} chassis_state_t;

static chassis_state_t chassis_state;
static uint8_t chassis_reference_seen; /* 区分ENABLE后尚无目标与活动目标突然过期。 */
/* 调度均使用板端真实毫秒差值，不使用循环次数冒充时间。 */
static uint32_t chassis_last_step_ms, chassis_last_speed_ms, chassis_last_adc_ms;
static uint32_t chassis_last_status_ms, chassis_last_odom_ms;
static int32_t chassis_zero_left, chassis_zero_right; /* 前次循环计数，供电流零点静止门比较。 */
static uint16_t chassis_last_command_sequence;
static uint8_t chassis_last_command_result;
static uint32_t chassis_uptime_ms;

/** @brief 将遥测float饱和截断到i16，避免速度/角速度转换溢出。 */
static int16_t chassis_i16(float value)
{
  if(value > 32767.0f) return 32767;
  if(value < -32768.0f) return -32768;
  return (int16_t)value;
}

/**
 * @brief 锁存故障、立即请求双轮主动制动并撤销Reference。
 * @param fault safety.h中的故障位，可同时置多个位；本函数不会清除历史故障。
 *
 * 制动不等待CAN回执；Reference屏障同时阻止故障前已排队的速度重新激活车辆。
 */
static void chassis_stop_fault(uint8_t fault)
{
  safety_raise(fault);
  drive_control_stop();
  chassis_protocol_clear_reference();
  chassis_state = CHASSIS_FAULT;
  chassis_reference_seen = 0U;
}

/**
 * @brief 每周期取出一个已经经过协议优先级选择的管理命令并执行状态转换。
 *
 * ENABLE只检查准入并建立新接收边界，本身不设置轮速；STOP和DISABLE都撤销使能。
 * FAULT_RESET可以尝试恢复CAN/ADC，但必须由新鲜有效的传感状态证明故障可清除。
 * 结果保存到0x181快照中，0表示接受、1表示条件不满足。
 */
static void chassis_handle_command(void)
{
  chassis_command_t command;
  adc_dma_sample_t adc;
  if(chassis_protocol_take_command(&command) == 0U) return;
  chassis_last_command_sequence = command.sequence;
  chassis_last_command_result = 0U;
  switch(command.type)
  {
    case CHASSIS_CMD_ENABLE:
      /* 电流零点、母线/电流、IMU、防撞释放和CAN链路都通过，才能进入ENABLED。 */
      if((chassis_state == CHASSIS_DISABLED) && (safety_can_enable() != 0U) &&
         (imu_bmi270_fresh() != 0U) && (bumper_can_clear() != 0U) &&
         (chassis_protocol_bus_healthy() != 0U))
      {
        chassis_protocol_clear_reference(); /* ENABLE前已到达的速度不能启动电机。 */
        chassis_reference_seen = 0U;
        chassis_state = CHASSIS_ENABLED;
      }
      else chassis_last_command_result = 1U;
      break;
    case CHASSIS_CMD_DISABLE:
    case CHASSIS_CMD_STOP:
      /* 本V0.3实现的STOP会撤销ENABLE，下次运动必须先重新ENABLE。 */
      drive_control_stop();
      chassis_protocol_clear_reference();
      chassis_reference_seen = 0U;
      if(chassis_state != CHASSIS_FAULT) chassis_state = CHASSIS_DISABLED;
      break;
    case CHASSIS_CMD_ESTOP:
      chassis_stop_fault(SAFETY_FAULT_ESTOP);
      break;
    case CHASSIS_CMD_FAULT_RESET:
      /* 只发起恢复，不假定恢复已经完成；新ADC样本未就绪时本次RESET可以被拒绝。 */
      if(chassis_protocol_bus_healthy() == 0U) canfd_driver_recover();
      if(((safety_fault() & SAFETY_FAULT_POWER) != 0U) && (adc_dma_take(&adc) == 0U))
      {
        (void)adc_dma_init();
        adc_dma_start(); /* 新样本到来前safety_clear仍会拒绝陈旧数据。 */
      }
      if((bumper_clear() != 0U) && (imu_bmi270_fresh() != 0U) &&
         (chassis_protocol_bus_healthy() != 0U) && (safety_clear(chassis_uptime_ms) != 0U))
      {
        drive_control_clear_fault();
        chassis_protocol_clear_reference();
        chassis_reference_seen = 0U;
        chassis_state = CHASSIS_DISABLED;
      }
      else chassis_last_command_result = 1U;
      break;
    case CHASSIS_CMD_RESET_ODOMETRY:
      if(chassis_state == CHASSIS_DISABLED)
      {
        motor_encoder_reset_all();
        /* 编码器清零和测速基准必须一起更新，防止把原累计轮程算成一次超速。 */
        drive_control_resync_feedback();
        chassis_last_speed_ms = chassis_uptime_ms;
        odometry_reset();
      }
      else chassis_last_command_result = 1U;
      break;
    default:
      chassis_last_command_result = 1U;
      break;
  }
}

/**
 * @brief 汇集状态机、故障、传感有效性和PWM/电气快照，交给0x181编码器。
 *
 * flags的bit0为IMU有效，bit1为里程计和IMU同时有效；电流/电压是最近有效样本。
 * 本函数只提供诊断，不参与本周期是否停车的判断。
 */
static void chassis_send_status(void)
{
  chassis_status_t status;
  safety_status_t safety;
  odometry_t odometry;

  safety_get(&safety);
  odometry_get(&odometry);
  memset(&status, 0, sizeof(status));
  status.state = (uint8_t)chassis_state;
  status.fault = safety.fault;
  status.bumper_mask = bumper_latched();
  status.flags = ((imu_bmi270_fresh() != 0U) ? 0x01U : 0U) |
      ((odometry.valid != 0U) && (imu_bmi270_fresh() != 0U) ? 0x02U : 0U);
  status.command_sequence = chassis_last_command_sequence;
  status.command_result = chassis_last_command_result;
  status.left_pwm = drive_control_pwm(MOTOR_LEFT);
  status.right_pwm = drive_control_pwm(MOTOR_RIGHT);
  status.bus_mv = safety.bus_mv;
  status.left_current_ma = safety.left_current_ma;
  status.right_current_ma = safety.right_current_ma;
  chassis_protocol_send_status(&status, chassis_uptime_ms);
}

/**
 * @brief 用双轮滤波RPM计算车体线速度，连同相对位姿发送0x381。
 *
 * 轮控内部正负号先按安装参数还原成“车体前进为正”；平均RPM乘轮周长/60
 * 得到mm/s。陀螺仪deg/s乘100转换成报文要求的0.01°/s。
 */
static void chassis_send_odometry(void)
{
  odometry_t odometry;
  float left_body = drive_control_measured_rpm(MOTOR_LEFT) * BOARD_LEFT_TARGET_SIGN;
  float right_body = drive_control_measured_rpm(MOTOR_RIGHT) * BOARD_RIGHT_TARGET_SIGN;
  float linear = (left_body + right_body) * BOARD_WHEEL_CIRCUMFERENCE_MM / 120.0f;
  odometry_get(&odometry);
  chassis_protocol_send_odometry(&odometry, chassis_i16(linear),
      chassis_i16(imu_bmi270_gyro_z_dps() * 100.0f), chassis_uptime_ms);
}

/**
 * @brief 执行一次非重入底盘循环，由main按约10ms周期调用。
 *
 * 顺序为真实时间检查→协议/传感采样→本地故障处理→管理命令→Reference执行，
 * 随后更新里程计和遥测。速度环独立按实际经过的20ms周期运行，禁用/HOLD时也测速。
 * 本周期计算使用同一次读取的毫秒时钟；WDT只在完整循环结束后重装。
 */
void chassis_app_step(void)
{
  drive_reference_t reference = {0};
  adc_dma_sample_t adc;
  odometry_t odometry;
  int32_t left = motor_encoder_get_count(MOTOR_LEFT);
  int32_t right = motor_encoder_get_count(MOTOR_RIGHT);
  float dt_s;

  chassis_uptime_ms = board_port_now_ms();
  dt_s = (chassis_uptime_ms - chassis_last_step_ms) / 1000.0f;
  if(dt_s <= 0.0f) return;
  chassis_last_step_ms = chassis_uptime_ms;
  /* 控制循环本身停顿达到50ms也视为超时，即使上位机仍持续发送数据。 */
  if(dt_s >= BOARD_IMU_STALE_MS / 1000.0f) chassis_stop_fault(SAFETY_FAULT_TIMEOUT);
  chassis_protocol_poll(chassis_uptime_ms);
  bumper_update();
  /* 先消费上次完整DMA结果，再触发下一轮扫描，避免读取正在搬运的三个通道。 */
  if(adc_dma_take(&adc) != 0U)
    safety_update(chassis_uptime_ms, &adc,
        ((chassis_state == CHASSIS_DISABLED) &&
         (left == chassis_zero_left) && (right == chassis_zero_right) &&
         (drive_control_pwm(MOTOR_LEFT) == 0) &&
         (drive_control_pwm(MOTOR_RIGHT) == 0)) ? 1U : 0U);
  else safety_update(chassis_uptime_ms, 0, 0U);
  chassis_zero_left = left;
  chassis_zero_right = right;
  if((chassis_uptime_ms - chassis_last_adc_ms) >= BOARD_ADC_PERIOD_MS)
  {
    chassis_last_adc_ms = chassis_uptime_ms;
    adc_dma_start();
  }
  if(bumper_latched() != 0U) safety_raise(SAFETY_FAULT_BUMPER);
  if(chassis_protocol_bus_healthy() == 0U) safety_raise(SAFETY_FAULT_CAN);
  if(imu_bmi270_update(dt_s) == 0U) safety_raise(SAFETY_FAULT_IMU);
  if(safety_fault() != SAFETY_FAULT_NONE) chassis_stop_fault(safety_fault());
  chassis_handle_command();

  if((chassis_state == CHASSIS_ENABLED) || (chassis_state == CHASSIS_RUNNING))
  {
    if(chassis_protocol_get_reference(chassis_uptime_ms, &reference) != 0U)
    {
      chassis_reference_seen = 1U;
      if((reference.linear_mm_s == 0) && (reference.angular_mrad_s == 0) &&
         (reference.accel_mm_s2 == 0U) && (reference.decel_mm_s2 == 0U))
      {
        /* 四字段全零HOLD是明确的停车保持，不应继续应用陀螺仪误差补偿。 */
        drive_control_stop();
        chassis_state = CHASSIS_ENABLED;
      }
      else chassis_state = CHASSIS_RUNNING;
    }
    /* 曾经执行过Reference之后读取失败，才按TTL失效锁存故障；尚未收到目标可等待。 */
    else if(chassis_reference_seen != 0U) chassis_stop_fault(SAFETY_FAULT_TIMEOUT);
  }

  /* 测速独立于ENABLE/HOLD；停车期间也不能积攒编码器增量到下一次起步。 */
  if((chassis_uptime_ms - chassis_last_speed_ms) >= BOARD_SPEED_PERIOD_MS)
  {
    float speed_dt = (chassis_uptime_ms - chassis_last_speed_ms) / 1000.0f;
    chassis_last_speed_ms = chassis_uptime_ms;
    if(chassis_state != CHASSIS_RUNNING) memset(&reference, 0, sizeof(reference));
    if(drive_control_update(&reference, imu_bmi270_gyro_z_dps() * 17.45329252f, speed_dt) == 0U)
      chassis_stop_fault(SAFETY_FAULT_DRIVE);
  }

  /* 编码器驱动计数按既有安装标定统一取负，转换为车体前进方向的有符号轮程。 */
  odometry_update(-motor_encoder_get_count(MOTOR_LEFT),
                  -motor_encoder_get_count(MOTOR_RIGHT),
                  imu_bmi270_gyro_z_dps(), dt_s);
  odometry_get(&odometry);
  if(odometry.valid == 0U) chassis_stop_fault(SAFETY_FAULT_IMU);
  if((chassis_uptime_ms - chassis_last_status_ms) >= BOARD_STATUS_PERIOD_MS)
  {
    chassis_last_status_ms = chassis_uptime_ms;
    chassis_send_status();
  }
  /* 单次循环最多尝试发一帧；状态与里程计同时到期时，里程计延后到后续循环。 */
  else if((chassis_uptime_ms - chassis_last_odom_ms) >= BOARD_SPEED_PERIOD_MS)
  {
    chassis_last_odom_ms = chassis_uptime_ms;
    chassis_send_odometry();
  }
  /* 故障状态也继续喂狗：此处证明控制循环可运行，不代表所有传感器均健康。 */
  board_port_watchdog_feed();
}

/**
 * @brief 按制动→输入设备→协议→安全时基→WDT顺序启动底盘。
 *
 * IMU静止校准在启动阶段完成，随后才建立ADC初次采样宽限时刻，避免校准时间
 * 消耗运行期ADC超时。ADC/IMU初始化失败仍保持FAULT并提供通信诊断。
 * @return 1=WDT配置成功，可以运行诊断/控制循环；0=WDT配置失败，应由main保持制动。
 * @note 返回1不表示全部传感器已就绪，正常运动仍需通过ENABLE准入。
 */
uint8_t chassis_app_init(void)
{
  uint8_t adc_ok, imu_ok, watchdog_ok;
  drive_control_init();
  bumper_init();
  odometry_init();
  adc_ok = adc_dma_init();
  imu_ok = imu_bmi270_init();
  chassis_protocol_init();
  chassis_state = CHASSIS_DISABLED;
  chassis_reference_seen = 0U;
  chassis_uptime_ms = board_port_now_ms();
  chassis_last_step_ms = chassis_last_speed_ms = chassis_last_adc_ms = chassis_uptime_ms;
  chassis_last_status_ms = chassis_last_odom_ms = chassis_uptime_ms;
  safety_init(chassis_uptime_ms);
  if(adc_ok == 0U) chassis_stop_fault(SAFETY_FAULT_POWER);
  if(imu_ok == 0U) chassis_stop_fault(SAFETY_FAULT_IMU);
  watchdog_ok = board_port_watchdog_init(); /* 外设失败也必须完成WDT配置。 */
  if(adc_ok != 0U) adc_dma_start();
  return watchdog_ok; /* 传感故障保留通信诊断；WDT配置失败由main保持制动。 */
}
