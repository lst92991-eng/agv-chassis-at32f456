/**
 * @file chassis_protocol.h
 * @brief 单上位机V0.3控制命令、短TTL车体Reference及状态发送接口。
 *
 * 所有接口由主循环调用。CAN ISR由驱动层复制帧，协议层不直接操作电机。
 * 0x201管理使能/停止/复位；0x401是唯一非安全运动输入，所有多字节字段为小端。
 */
#ifndef CHASSIS_PROTOCOL_H
#define CHASSIS_PROTOCOL_H

#include <stdint.h>

#include "drive_control.h"
#include "odometry.h"

/** @brief 0x201的命令码；真正的状态转换和准入检查由chassis_app执行。 */
typedef enum
{
  CHASSIS_CMD_NONE = 0,
  CHASSIS_CMD_ENABLE = 1,
  CHASSIS_CMD_DISABLE = 2,
  CHASSIS_CMD_STOP = 3,
  CHASSIS_CMD_ESTOP = 4,
  CHASSIS_CMD_FAULT_RESET = 5,
  CHASSIS_CMD_RESET_ODOMETRY = 6
} chassis_command_type_t;

/** @brief 解码后的管理命令；停止类不受旧序号限制，普通操作必须递增。 */
typedef struct
{
  chassis_command_type_t type; /**< 操作码，无运动速度或PWM字段。 */
  uint16_t sequence;           /**< 16位循环序号，允许65535→0。 */
} chassis_command_t;

/** @brief 0x181的内存快照；协议编码器逐字段发送，禁止直接发送此结构体内存。 */
typedef struct
{
  uint8_t state;                /**< 0禁用、1已使能、2执行Reference、3故障。 */
  uint8_t fault;                /**< safety.h定义的锁存故障位。 */
  uint8_t bumper_mask;          /**< 前/后/左/右防撞锁存位，定义见bumper.h。 */
  uint8_t flags;                /**< bit0:IMU有效；bit1:里程计与IMU均有效。 */
  uint16_t command_sequence;    /**< 最近被应用层处理的管理命令序号。 */
  uint8_t command_result;       /**< 0接受、1拒绝；上电尚无命令时默认0。 */
  int16_t left_pwm;             /**< 左轮内部有符号PWM千分比，不是RPM。 */
  int16_t right_pwm;            /**< 右轮内部有符号PWM千分比，不是RPM。 */
  uint16_t bus_mv;              /**< 最近有效ADC样本换算的母线电压，mV。 */
  uint16_t left_current_ma;     /**< 左路扣除零点后的非负电流，mA。 */
  uint16_t right_current_ma;    /**< 右路扣除零点后的非负电流，mA。 */
} chassis_status_t;

/** @brief 初始化CAN及空协议状态；仅上电时调用，运行中不能借此重置序号历史。 */
void chassis_protocol_init(void);
/** @brief 用真实板端毫秒数推进收发/解析；应先于本周期应用状态机调用。 */
void chassis_protocol_poll(uint32_t now_ms);
/** @brief 输出并消费最高优先级待处理命令；返回1表示有命令，0表示无命令或空指针。 */
uint8_t chassis_protocol_take_command(chassis_command_t *command);
/**
 * @brief 输出仍在50～300ms报文TTL内的最新Reference，成功返回1，否则返回0。
 * @param now_ms 与CAN接收时间同源的真实毫秒时钟。
 * @param reference 有效时写入车体v/omega与斜率限制；此调用不消费目标、不执行制动。
 */
uint8_t chassis_protocol_get_reference(uint32_t now_ms, drive_reference_t *reference);
/** @brief 撤销目标并屏蔽已接收帧，保留序号历史；ENABLE/STOP/故障切换时调用。 */
void chassis_protocol_clear_reference(void);
/** @brief 返回1表示CAN驱动当前无bus-off/error-passive/已锁存链路错误。 */
uint8_t chassis_protocol_bus_healthy(void);
/** @brief 非阻塞尝试发送24字节0x181；硬件忙时本次可丢弃，uptime_ms单位ms。 */
void chassis_protocol_send_status(const chassis_status_t *status, uint32_t uptime_ms);
/**
 * @brief 非阻塞尝试发送20字节0x381相对里程计；不提供绝对位置保证。
 * @param odometry 位姿x/y单位mm，yaw单位度，线上yaw转换为0.01°。
 * @param linear_mm_s 车体线速度，mm/s，前进为正。
 * @param yaw_rate_cdeg_s 偏航角速度，0.01°/s，左转为正。
 * @param uptime_ms 板端真实毫秒数。
 */
void chassis_protocol_send_odometry(const odometry_t *odometry,
                                    int16_t linear_mm_s,
                                    int16_t yaw_rate_cdeg_s,
                                    uint32_t uptime_ms);

#endif
