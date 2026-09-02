/**
 * @file canfd_driver.h
 * @brief CAN3 FD硬件收发、ISR接收快照和链路健康接口。
 *
 * 使用标准ID、ISO CAN FD、BRS，仲裁500kbit/s、数据4Mbit/s。
 * 除rx_irq()外均由主循环调用，协议/电机状态机位于上层。
 */
#ifndef CANFD_DRIVER_H
#define CANFD_DRIVER_H

#include <stdint.h>

/** @brief CAN接收帧的板内表示；receive_ms/order不是线上payload的一部分。 */
typedef struct
{
  uint32_t id;          /**< 标准CAN标识符，当前接收0x201/0x401。 */
  uint32_t receive_ms;  /**< ISR读入硬件帧时的板端毫秒时间，作为TTL起点。 */
  uint32_t order;       /**< 已入队帧的单调循环顺序号，区分使能/停止前后。 */
  uint8_t size;         /**< 真实payload字节数，已从DLC换算。 */
  uint8_t data[32];     /**< 有效内容为前size字节，不含时间元数据。 */
} canfd_frame_t;

/** @brief 上电初始化GPIO、CAN3时序和RX队列；运行恢复应调用recover。 */
void canfd_driver_init(void);
/** @brief 主循环更新发送完成/50ms发送超时，now_ms必须来自真实毫秒时钟。 */
void canfd_driver_poll(uint32_t now_ms);
/** @brief 仅供CAN3 RX中断分发调用，复制帧但不执行控制命令。 */
void canfd_driver_rx_irq(void);
/** @brief 原子取出一帧到frame；成功返回1，空队列/空指针返回0。 */
uint8_t canfd_driver_take(canfd_frame_t *frame);
/**
 * @brief 非阻塞复制并提交标准ID报文，返回1仅表示硬件请求提交成功。
 * @param id 标准CAN ID；本项目发送0x181/0x381。
 * @param data size字节的输入数据，调用期间必须有效。
 * @param size 当前业务使用20/24字节，驱动最大容量32字节。
 * @return 0=忙、异常或提交失败，由调用方选择下次周期重发。
 */
uint8_t canfd_driver_send(uint32_t id, const uint8_t *data, uint8_t size);
/** @brief 返回1表示无总线/驱动锁存错误；不代表已有有效Reference或已ENABLE。 */
uint8_t canfd_driver_healthy(void);
/** @brief 获取最近入队的order；CAN恢复不清零该计数，避免旧屏障失效。 */
uint32_t canfd_driver_rx_order(void);
/** @brief 屏蔽接收中断后重建CAN和队列；不清车辆故障、不恢复电机使能。 */
void canfd_driver_recover(void);

#endif
