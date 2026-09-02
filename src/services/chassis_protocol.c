/**
 * @file chassis_protocol.c
 * @brief 固定四报文教学协议：控制、Reference、状态和里程计。
 *
 * 本文件只在主循环中运行：将CAN字节转换成控制命令/车体参考量，再把状态编码发送。
 * TTL和序号是接收准入条件，电机制动由chassis_app执行。它不直接写PWM。
 * CAN接收时间和order由驱动ISR记录，软件排队不能延长Reference的有效期。
 */
#include "chassis_protocol.h"

#include <string.h>

#include "board_config.h"
#include "canfd_driver.h"

/* 每条0x401自行携带TTL；120ms是常用发送值，接收器允许50～300ms。 */
#define PROTOCOL_MIN_TTL_MS 50U
#define PROTOCOL_MAX_TTL_MS 300U

static drive_reference_t protocol_reference; /* 最近一条通过全部校验的车体速度。 */
static chassis_command_t protocol_command;   /* 尚未消费的最高优先级管理命令。 */
static uint16_t protocol_reference_sequence;
static uint16_t protocol_command_sequence;
static uint16_t protocol_ttl_ms;
static uint32_t protocol_reference_ms;       /* ISR记录的板端接收毫秒数，不是解析时刻。 */
static uint32_t protocol_reference_barrier;  /* 最近一次撤销目标时的接收order屏障。 */
static uint8_t protocol_reference_valid;     /* 当前目标是否允许被读取。 */
static uint8_t protocol_reference_seen;      /* 历史序号是否存在，STOP不会清除此标志。 */
static uint8_t protocol_command_seen;        /* 独立标志保证合法sequence=0仍然能够去重。 */

/** @brief 读取小端u16；调用者已确认data至少包含两个字节。 */
static uint16_t protocol_u16(const uint8_t *data)
{
  return (uint16_t)data[0] | ((uint16_t)data[1] << 8U);
}

/** @brief 将小端二进制补码解释成i16，供v/omega等有符号字段使用。 */
static int16_t protocol_i16(const uint8_t *data)
{
  return (int16_t)protocol_u16(data);
}

/** @brief 读取小端u32；逐字节组合，避免未对齐访问和结构体填充差异。 */
static uint32_t protocol_u32(const uint8_t *data)
{
  return (uint32_t)data[0] | ((uint32_t)data[1] << 8U) |
      ((uint32_t)data[2] << 16U) | ((uint32_t)data[3] << 24U);
}

/** @brief 写入两个小端字节；有符号值由调用者先转换为u16保留位模式。 */
static void protocol_put_u16(uint8_t *data, uint16_t value)
{
  data[0] = (uint8_t)value;
  data[1] = (uint8_t)(value >> 8U);
}

/** @brief 写入四个小端字节，接收端必须使用协议定义的有符号/无符号类型。 */
static void protocol_put_u32(uint8_t *data, uint32_t value)
{
  data[0] = (uint8_t)value;
  data[1] = (uint8_t)(value >> 8U);
  data[2] = (uint8_t)(value >> 16U);
  data[3] = (uint8_t)(value >> 24U);
}

/**
 * @brief 判断16位序号是否向前推进，65535→0属于合法更新。
 * @return 1=新序号，0=重复或落后；相隔半个序号空间及以上视为旧帧。
 */
static uint8_t protocol_after(uint16_t value, uint16_t previous)
{
  uint16_t delta = (uint16_t)(value - previous);
  return ((delta != 0U) && (delta < 0x8000U)) ? 1U : 0U;
}

/** @brief 管理命令优先级：ESTOP＞DISABLE＞STOP＞普通操作，NONE也为0。 */
static uint8_t protocol_priority(chassis_command_type_t command)
{
  if(command == CHASSIS_CMD_ESTOP) return 3U;
  if(command == CHASSIS_CMD_DISABLE) return 2U;
  return command == CHASSIS_CMD_STOP ? 1U : 0U;
}

/**
 * @brief 解析0x201并更新唯一待处理命令，不在此处改变底盘状态。
 *
 * 8字节布局：cmd:u8、reserved:u8、sequence:u16、reserved:u32，均为小端。
 * STOP类请求允许重复/旧序号；普通命令只能递增。低优先级命令即使序号更新，
 * 也不能覆盖本轮待执行的停止命令，其序号仍进入历史，防止之后重放。
 * @param frame 已由CAN驱动复制的完整帧，调用者保证非空。
 */
static void protocol_parse_control(const canfd_frame_t *frame)
{
  chassis_command_t command;
  uint8_t is_new;
  if((frame->size != 8U) || (frame->data[1] != 0U) ||
     (protocol_u32(&frame->data[4]) != 0U)) return;
  command.type = (chassis_command_type_t)frame->data[0];
  command.sequence = protocol_u16(&frame->data[2]);
  if((command.type <= CHASSIS_CMD_NONE) ||
     (command.type > CHASSIS_CMD_RESET_ODOMETRY)) return;
  is_new = (protocol_command_seen == 0U) ||
      protocol_after(command.sequence, protocol_command_sequence);
  /* 停止请求不受旧序号限制，但不能倒退普通命令历史。 */
  if((is_new == 0U) && (protocol_priority(command.type) == 0U)) return;
  if(is_new != 0U)
  {
    protocol_command_sequence = command.sequence;
    protocol_command_seen = 1U;
  }
  if(protocol_priority(command.type) >= protocol_priority(protocol_command.type))
    protocol_command = command;
}

/**
 * @brief 解析0x401，校验接收屏障、序号、TTL和运动上限后替换最新Reference。
 *
 * 16字节布局：seq:u16、TTL:u16、v:i16、omega:i16、accel:u16、decel:u16、reserved:u32。
 * 单位依次为ms、mm/s、mrad/s、mm/s²、mm/s²。四个运动字段全零代表HOLD。
 * 任一校验失败均保留原目标及其原截止时间，不用错误包给旧运动续命。
 * @param frame 完整的接收快照；receive_ms和order属于驱动元数据，不在线上传输。
 * @param now_ms 本轮协议处理的真实板端毫秒时刻，与receive_ms使用同一时钟。
 */
static void protocol_parse_reference(const canfd_frame_t *frame, uint32_t now_ms)
{
  uint16_t sequence;
  uint16_t ttl;
  uint32_t order_delta = frame->order - protocol_reference_barrier;
  drive_reference_t reference;

  /* 32位半量程判断排除ENABLE/STOP之前已经到达、尚未解析的帧。 */
  if((frame->size != BOARD_CAN_REFERENCE_SIZE) || (order_delta == 0U) ||
     (order_delta >= 0x80000000UL)) return;
  sequence = protocol_u16(&frame->data[0]);
  ttl = protocol_u16(&frame->data[2]);
  reference.linear_mm_s = protocol_i16(&frame->data[4]);
  reference.angular_mrad_s = protocol_i16(&frame->data[6]);
  reference.accel_mm_s2 = protocol_u16(&frame->data[8]);
  reference.decel_mm_s2 = protocol_u16(&frame->data[10]);
  if((protocol_u32(&frame->data[12]) != 0U) ||
     (ttl < PROTOCOL_MIN_TTL_MS) || (ttl > PROTOCOL_MAX_TTL_MS) ||
     ((now_ms - frame->receive_ms) >= ttl) ||
     ((protocol_reference_seen != 0U) &&
      (protocol_after(sequence, protocol_reference_sequence) == 0U))) return;
  if((reference.linear_mm_s > (int16_t)BOARD_MAX_LINEAR_MM_S) ||
     (reference.linear_mm_s < -(int16_t)BOARD_MAX_LINEAR_MM_S) ||
     (reference.angular_mrad_s > (int16_t)BOARD_MAX_ANGULAR_MRAD_S) ||
     (reference.angular_mrad_s < -(int16_t)BOARD_MAX_ANGULAR_MRAD_S)) return;
  /* 全零HOLD无需斜率；其余指令的加减速度必须保持平台范围。 */
  if(((reference.linear_mm_s != 0) || (reference.angular_mrad_s != 0) ||
      (reference.accel_mm_s2 != 0U) || (reference.decel_mm_s2 != 0U)) &&
     ((reference.accel_mm_s2 < 10U) || (reference.accel_mm_s2 > 500U) ||
      (reference.decel_mm_s2 < 10U) || (reference.decel_mm_s2 > 1000U))) return;
  protocol_reference = reference;
  protocol_reference_sequence = sequence;
  protocol_ttl_ms = ttl;
  protocol_reference_ms = frame->receive_ms;
  protocol_reference_valid = 1U;
  protocol_reference_seen = 1U;
}

/** @brief 上电时建立空命令、空Reference和新的序号历史，并初始化CAN3。 */
void chassis_protocol_init(void)
{
  memset(&protocol_reference, 0, sizeof(protocol_reference));
  memset(&protocol_command, 0, sizeof(protocol_command));
  protocol_reference_sequence = protocol_command_sequence = 0U;
  protocol_reference_seen = protocol_command_seen = 0U;
  canfd_driver_init();
  chassis_protocol_clear_reference();
}

/**
 * @brief 主循环推进非阻塞CAN发送并消费已接收报文，随后由应用层取管理命令。
 * @param now_ms 板端真实毫秒数；处理多个速度帧后，只保存最后一条通过校验的目标。
 */
void chassis_protocol_poll(uint32_t now_ms)
{
  canfd_frame_t frame;
  canfd_driver_poll(now_ms);
  while(canfd_driver_take(&frame) != 0U)
  {
    if(frame.id == BOARD_CAN_ID_CONTROL) protocol_parse_control(&frame);
    else if(frame.id == BOARD_CAN_ID_REFERENCE) protocol_parse_reference(&frame, now_ms);
  }
}

/**
 * @brief 取出本轮胜出的管理命令并清空待处理槽，不清除去重历史。
 * @param command 输出参数，非空时写入命令码及其序号。
 * @return 1=取到命令；0=输出指针无效或无待处理命令。
 */
uint8_t chassis_protocol_take_command(chassis_command_t *command)
{
  if((command == 0) || (protocol_command.type == CHASSIS_CMD_NONE)) return 0U;
  *command = protocol_command;
  protocol_command.type = CHASSIS_CMD_NONE;
  return 1U;
}

/**
 * @brief 读取仍在TTL内的最新Reference；同一目标可以在多个控制周期被读取。
 *
 * elapsed>=TTL即失效，采用无符号差值支持毫秒计数回绕。本函数只返回有效性，
 * 不直接停车；应用层在活动Reference消失后锁存TIMEOUT并调用主动制动。
 * @param now_ms 当前板端毫秒数。
 * @param reference 有效时输出v/omega和加减速限制；返回0时不修改输出。
 * @return 1=目标有效；0=没有目标、已撤销、已过期或输出指针为空。
 */
uint8_t chassis_protocol_get_reference(uint32_t now_ms, drive_reference_t *reference)
{
  if((reference == 0) || (protocol_reference_valid == 0U) ||
     ((now_ms - protocol_reference_ms) >= protocol_ttl_ms)) return 0U;
  *reference = protocol_reference;
  return 1U;
}

/**
 * @brief 撤销当前目标，并记录已接收帧的order边界。
 *
 * 成功ENABLE、STOP或故障转换调用。保留历史sequence以拒绝旧包重放；
 * 之后必须收到越过order屏障且序号更新的帧，才能再次形成有效目标。
 */
void chassis_protocol_clear_reference(void)
{
  /* 禁止ENABLE/STOP前已进入接收队列的速度帧再次激活车辆。 */
  protocol_reference_valid = 0U;
  protocol_reference_barrier = canfd_driver_rx_order();
}

/** @brief 转发CAN3链路健康状态；返回1不代表已收到上位机运动指令。 */
uint8_t chassis_protocol_bus_healthy(void)
{
  return canfd_driver_healthy();
}

/**
 * @brief 将应用快照逐字段编码为24字节0x181，尝试非阻塞发送。
 *
 * [0..3]状态/故障/防撞/有效性；[4..6]命令序号/结果；[8..11]毫秒时钟；
 * [12..15]左右PWM；[16..21]母线mV、左右电流mA。其余字节固定为0。
 * 硬件忙时本次状态可能丢弃，下一周期重新发送；函数返回不代表上位机已收到报文。
 * @param status 应用层刚生成的状态，不直接发送其内存布局。
 * @param uptime_ms 编码入报文的板端毫秒时刻。
 */
void chassis_protocol_send_status(const chassis_status_t *status, uint32_t uptime_ms)
{
  uint8_t data[24] = {0};
  if(status == 0) return;
  data[0] = status->state;
  data[1] = status->fault;
  data[2] = status->bumper_mask;
  data[3] = status->flags;
  protocol_put_u16(&data[4], status->command_sequence);
  data[6] = status->command_result;
  protocol_put_u32(&data[8], uptime_ms);
  protocol_put_u16(&data[12], (uint16_t)status->left_pwm);
  protocol_put_u16(&data[14], (uint16_t)status->right_pwm);
  protocol_put_u16(&data[16], status->bus_mv);
  protocol_put_u16(&data[18], status->left_current_ma);
  protocol_put_u16(&data[20], status->right_current_ma);
  (void)canfd_driver_send(BOARD_CAN_ID_STATUS, data, sizeof(data));
}

/**
 * @brief 将相对位姿与速度编码为20字节0x381，尝试非阻塞发送。
 *
 * 布局：x:i32、y:i32、yaw:i32、v:i16、yaw_rate:i16、time:u32。
 * 单位为mm、mm、0.01°、mm/s、0.01°/s、ms；float转整数按C转换规则截断。
 * @param odometry 编码器与IMU融合的相对位姿，绝对位置仍需上位机校正。
 * @param linear_mm_s 车体有符号线速度。
 * @param yaw_rate_cdeg_s 车体有符号偏航角速度，0.01°/s。
 * @param uptime_ms 板端毫秒时刻。
 */
void chassis_protocol_send_odometry(const odometry_t *odometry,
                                    int16_t linear_mm_s,
                                    int16_t yaw_rate_cdeg_s,
                                    uint32_t uptime_ms)
{
  uint8_t data[20] = {0};
  if(odometry == 0) return;
  protocol_put_u32(&data[0], (uint32_t)(int32_t)odometry->x_mm);
  protocol_put_u32(&data[4], (uint32_t)(int32_t)odometry->y_mm);
  protocol_put_u32(&data[8], (uint32_t)(int32_t)(odometry->yaw_deg * 100.0f));
  protocol_put_u16(&data[12], (uint16_t)linear_mm_s);
  protocol_put_u16(&data[14], (uint16_t)yaw_rate_cdeg_s);
  protocol_put_u32(&data[16], uptime_ms);
  (void)canfd_driver_send(BOARD_CAN_ID_ODOMETRY, data, sizeof(data));
}
