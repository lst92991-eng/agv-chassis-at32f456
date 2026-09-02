/**
 * @file canfd_driver.c
 * @brief CAN3的500k/4M BRS配置、短RX队列和非阻塞发送。
 *
 * PB3/PB4使用MUX11接CAN3；硬件链路只搬运报文，不解释ENABLE或速度含义。
 * ISR写RX队列，主循环取出快照；TX只有一个正在发送的槽，忙时调用者稍后重试。
 * 上层通过healthy()判断bus-off、error-passive、RX溢出或TX超时等链路故障。
 */
#include "canfd_driver.h"

#include <string.h>

#include "at32f45x.h"
#include "at32f45x_can.h"
#include "at32f45x_crm.h"
#include "at32f45x_gpio.h"
#include "at32f45x_misc.h"
#include "board_config.h"
#include "board_port.h"

/* 环形队列保留一个空位区分满/空，数组4项对应最多3帧待处理。 */
#define CANFD_RX_COUNT 4U
#define CANFD_TX_TIMEOUT_MS 50U

static canfd_frame_t canfd_rx[CANFD_RX_COUNT];
static volatile uint8_t canfd_head;     /* ISR发布写入位置。 */
static volatile uint8_t canfd_tail;     /* 主循环发布消费位置。 */
static volatile uint8_t canfd_overflow; /* RX溢出或TX超时的锁存错误，恢复时清除。 */
static uint8_t canfd_tx_busy;           /* 主循环独占的硬件发送槽占用标志。 */
static uint8_t canfd_tx_handle;
static uint32_t canfd_tx_started_ms;
static uint32_t canfd_now_ms;
static volatile uint32_t canfd_order;   /* 已入队输入帧的顺序号；恢复CAN也不重置。 */

/** @brief 将CAN FD的DLC枚举换成真实长度；DLC大于8后不是字节数本身。 */
static uint8_t canfd_dlc_size(can_data_length_type dlc)
{
  static const uint8_t size[16] = {0U,1U,2U,3U,4U,5U,6U,7U,8U,12U,16U,20U,24U,32U,48U,64U};
  return ((uint8_t)dlc < 16U) ? size[(uint8_t)dlc] : 0U;
}

/**
 * @brief 为本驱动的最多32字节发送缓冲选择DLC。
 * @param size 调用者提供的长度；8以内及12/16/20/24直接映射，其他值按32字节发送。
 * @note 生产协议当前只发送20和24字节；额外填充来自清零的TX结构体。
 */
static can_data_length_type canfd_dlc(uint8_t size)
{
  if(size <= 8U) return (can_data_length_type)size;
  if(size == 12U) return CAN_DLC_BYTES_12;
  if(size == 16U) return CAN_DLC_BYTES_16;
  if(size == 20U) return CAN_DLC_BYTES_20;
  if(size == 24U) return CAN_DLC_BYTES_24;
  return CAN_DLC_BYTES_32;
}

/**
 * @brief 初始化CAN3引脚、500k/4M ISO FD时序和接收中断，并清软件队列错误。
 *
 * 时序参数依赖当前192MHz PCLK配置，调整系统时钟后必须重新计算。
 * 运行中应通过recover()先屏蔽RX中断再重建硬件；此函数本身不恢复车辆ENABLE。
 */
void canfd_driver_init(void)
{
  gpio_init_type gpio;
  can_bittime_type timing;

  crm_periph_clock_enable(CRM_GPIOB_PERIPH_CLOCK, TRUE);
  crm_periph_clock_enable(CRM_CAN3_PERIPH_CLOCK, TRUE);
  gpio_default_para_init(&gpio);
  gpio.gpio_pins = GPIO_PINS_3 | GPIO_PINS_4;
  gpio.gpio_mode = GPIO_MODE_MUX;
  gpio.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
  gpio.gpio_pull = GPIO_PULL_NONE;
  gpio.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
  gpio_init(GPIOB, &gpio);
  gpio_pin_mux_config(GPIOB, GPIO_PINS_SOURCE3, GPIO_MUX_11);
  gpio_pin_mux_config(GPIOB, GPIO_PINS_SOURCE4, GPIO_MUX_11);
  crm_can_clock_select(CRM_CAN3, CRM_CAN_CLOCK_SOURCE_PCLK);
  can_software_reset(CAN3, TRUE);
  can_bittime_default_para_init(&timing);
  timing.bittime_div = 2U;
  timing.ac_bts1_size = 144U;
  timing.ac_bts2_size = 48U;
  timing.ac_rsaw_size = 24U;
  timing.fd_bts1_size = 18U;
  timing.fd_bts2_size = 6U;
  timing.fd_rsaw_size = 6U;
  timing.fd_ssp_offset = 19U;
  can_bittime_set(CAN3, &timing);
  can_fd_iso_mode_enable(CAN3, TRUE);
  can_stb_transmit_mode_set(CAN3, CAN_STB_TRANSMIT_BY_FIFO);
  can_software_reset(CAN3, FALSE);
  /* FALSE表示不接收控制器判定出错的帧，并非关闭所有报文接收；ID在ISR中筛选。 */
  can_receive_all_enable(CAN3, FALSE);
  can_mode_set(CAN3, CAN_MODE_COMMUNICATE);
  can_interrupt_enable(CAN3, CAN_RIE_INT | CAN_ROIE_INT, TRUE);
  can_flag_clear(CAN3, CAN_ALL_FLAG);
  canfd_head = canfd_tail = canfd_overflow = 0U;
  canfd_tx_busy = 0U;
  nvic_irq_enable(CAN3_RX_IRQn, 3U, 0U);
}

/**
 * @brief 主循环维护TX占用状态，超过50ms未结束则中止发送并锁存链路错误。
 * @param now_ms 板端真实毫秒数，须在本周期send()之前调用。
 * @note healthy()反映该锁存状态，实际制动由应用层处理，不等待CAN上报完成。
 */
void canfd_driver_poll(uint32_t now_ms)
{
  canfd_now_ms = now_ms;
  if((canfd_tx_busy != 0U) && (can_flag_get(CAN3, CAN_TPIF_FLAG) != RESET))
  {
    can_flag_clear(CAN3, CAN_TPIF_FLAG);
    canfd_tx_busy = 0U;
  }
  if((canfd_tx_busy != 0U) && ((now_ms - canfd_tx_started_ms) >= CANFD_TX_TIMEOUT_MS))
  {
    can_transmit_abort(CAN3, CAN_ABORT_TRANSMIT_PTB);
    canfd_tx_busy = 0U;
    canfd_overflow = 1U;
  }
}

/**
 * @brief CAN3接收ISR：校验帧类型，复制0x201/0x401及其接收时间、顺序号。
 *
 * 一次最多读取8个硬件帧，避免ISR持续占用CPU。软件队列满时记录故障，
 * 不覆盖已经排队的控制命令；非本板业务ID不占用队列。ISR不执行PID或状态机。
 */
void canfd_driver_rx_irq(void)
{
  can_rxbuf_type rx;
  uint8_t next;
  uint8_t size;
  uint8_t budget = 8U;

  while((can_rxbuf_status_get(CAN3) != CAN_RXBUF_STATUS_EMPTY) && (budget != 0U))
  {
    budget--;
    memset(&rx, 0, sizeof(rx));
    if(can_rxbuf_read(CAN3, &rx) != SUCCESS) break;
    size = canfd_dlc_size(rx.data_length);
    if((rx.id_type != CAN_ID_STANDARD) || (rx.frame_type != CAN_FRAME_DATA) ||
       (rx.fd_format != CAN_FORMAT_FD) || (rx.fd_rate_switch != CAN_BRS_ON) ||
       (size == 0U) || (size > sizeof(canfd_rx[0].data))) continue;
    if((rx.id != BOARD_CAN_ID_CONTROL) && (rx.id != BOARD_CAN_ID_REFERENCE)) continue;
    next = (uint8_t)((canfd_head + 1U) % CANFD_RX_COUNT);
    if(next == canfd_tail)
    {
      canfd_overflow = 1U;
      continue;
    }
    canfd_rx[canfd_head].id = rx.id;
    canfd_rx[canfd_head].receive_ms = board_port_now_ms();
    canfd_rx[canfd_head].order = ++canfd_order;
    canfd_rx[canfd_head].size = size;
    memcpy(canfd_rx[canfd_head].data, rx.data, size);
    /* 数据及元信息先写完，再公布head，防止消费者看见尚未完成的帧。 */
    __DMB();
    canfd_head = next;
  }
  if(can_flag_get(CAN3, CAN_ROIF_FLAG) != RESET) canfd_overflow = 1U;
  can_flag_clear(CAN3, CAN_RIF_FLAG | CAN_RAFIF_FLAG | CAN_RFIF_FLAG | CAN_ROIF_FLAG);
}

/**
 * @brief 主循环取出一帧原始接收快照并推进消费位置。
 * @param frame 非空输出缓冲，成功后包含payload和ISR元数据。
 * @return 1=取到一帧；0=队列空或输出指针为空。
 * @note 短临界区只保护CPU ISR共享队列，返回后由协议层解释字节含义。
 */
uint8_t canfd_driver_take(canfd_frame_t *frame)
{
  uint32_t irq;
  if(frame == 0) return 0U;
  irq = __get_PRIMASK();
  __disable_irq();
  if(canfd_tail == canfd_head)
  {
    if(irq == 0U) __enable_irq();
    return 0U;
  }
  *frame = canfd_rx[canfd_tail];
  canfd_tail = (uint8_t)((canfd_tail + 1U) % CANFD_RX_COUNT);
  if(irq == 0U) __enable_irq();
  return 1U;
}

/**
 * @brief 主循环尝试把一帧复制到CAN3主发送缓冲并发起传输。
 * @param id 本板业务标准ID；调用者应使用0x181或0x381。
 * @param data 要发送的数据，函数内已复制，返回后无需保留该缓冲。
 * @param size 有效字节数1～32；当前协议使用20/24。
 * @return 1=已提交硬件发送请求；0=参数无效、硬件忙、链路异常或提交失败。
 * @note 返回1不代表对端已收到，更不代表对端业务已经处理；完成由poll()推进。
 */
uint8_t canfd_driver_send(uint32_t id, const uint8_t *data, uint8_t size)
{
  can_txbuf_type tx;
  if((data == 0) || (size == 0U) || (size > 32U) || (canfd_tx_busy != 0U) ||
     (canfd_driver_healthy() == 0U)) return 0U;
  memset(&tx, 0, sizeof(tx));
  tx.id = id;
  tx.id_type = CAN_ID_STANDARD;
  tx.frame_type = CAN_FRAME_DATA;
  tx.data_length = canfd_dlc(size);
  tx.fd_format = CAN_FORMAT_FD;
  tx.fd_rate_switch = CAN_BRS_ON;
  tx.handle = canfd_tx_handle++;
  memcpy(tx.data, data, size);
  if((can_txbuf_write(CAN3, CAN_TXBUF_PTB, &tx) != SUCCESS) ||
     (can_txbuf_transmit(CAN3, CAN_TRANSMIT_PTB) != SUCCESS)) return 0U;
  canfd_tx_busy = 1U;
  canfd_tx_started_ms = canfd_now_ms;
  return 1U;
}

/**
 * @brief 判断控制器没有bus-off/error-passive，且本驱动没有锁存溢出或TX超时。
 * @return 1=上述链路条件正常；0=上层应按CAN故障停止运动。
 */
uint8_t canfd_driver_healthy(void)
{
  return ((CAN3->ctrlstat_bit.busoff == 0U) &&
          (can_flag_get(CAN3, CAN_EPASS_FLAG) == RESET) &&
          (canfd_overflow == 0U)) ? 1U : 0U;
}

/**
 * @brief 显式恢复时屏蔽RX中断并重建CAN硬件和短队列。
 * @note 旧队列被丢弃，order保持连续；函数不清上层故障，也不重新使能电机。
 */
void canfd_driver_recover(void)
{
  nvic_irq_disable(CAN3_RX_IRQn);
  canfd_driver_init();
}

/** @brief 返回ISR最近发布的接收顺序号，供协议建立ENABLE/STOP接收屏障。 */
uint32_t canfd_driver_rx_order(void) { return canfd_order; }
