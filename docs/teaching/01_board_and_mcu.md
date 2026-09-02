# 01：板卡与主控资源

## 1. 这块板承担什么

它是差速底盘的实时执行器：接收上位机给出的车体速度 `v/ω`，完成左右轮速度控制、
相对里程计和本地停车。它不保存路线、RFID 站点或任务状态。

```text
上位机 v/ω
    ↓ CAN FD
协议与状态机 → 差速换算 → 左右速度 PID → AT8236 → 两个电机
       ↑              ↑             ↑
   TTL / STOP      编码器        IMU / 电流 / 防撞条
```

## 2. 主控与时钟

当前 Keil 工程 `mdk_v5/agv_motor.uvprojx` 的目标器件是 **AT32F456RCT7**：

| 项目 | 当前工程信息 | 课堂含义 |
| --- | --- | --- |
| 内核 | Cortex-M4 | 先学习启动、SysTick、NVIC 和中断，再进入外设 |
| Flash | `0x08000000` 起 256 KB | 固件和启动向量所在区域 |
| SRAM | `0x20000000` 起 108 KB | 全局状态、DMA 缓冲和栈所在区域 |
| 外部晶振 | 12 MHz | `system_clock_config()` 的输入 |
| 系统时钟 | 192 MHz | SysTick、PWM、CAN、SPI 分频都由它推导 |
| AHB / APB1 / APB2 | 192 / 192 / 192 MHz | 修改时钟必须重新核对外设时序 |
| APB3 | 48 MHz | 保持在当前工程规定范围内 |

时钟配置位于 `src/drivers/at32f45x_clock.c`。课程中不要先改 PLL；先理解
`12 MHz × 64 ÷ 1 ÷ 4 = 192 MHz`，再说明为什么 PWM、CAN 和 SPI 的分频不能只看主频。

## 3. 已核对的板载资源

下表来自当前源码和新版网表核对摘要 `hardware/motor_board_netlist_review.md`。

| 功能 | MCU 引脚 / 外设 | 正式模块 | 说明 |
| --- | --- | --- | --- |
| 左 H 桥 | PC6 / PC7，TMR8 CH1 / CH2 | `drivers/motor_driver.c` | AT8236 左桥双 PWM，20 kHz |
| 右 H 桥 | PC8 / PC9，TMR8 CH3 / CH4 | `drivers/motor_driver.c` | AT8236 右桥双 PWM，20 kHz |
| 左编码器 | PA6 / PA7，EXINT | `drivers/motor_encoder.c` | AB 相四边沿软件解码 |
| 右编码器 | PC4 / PC5，EXINT | `drivers/motor_encoder.c` | AB 相四边沿软件解码 |
| 母线电压 | PA3 / ADC1 IN3 | `drivers/adc_dma.c` | 24 V 分压采样 |
| 左 / 右电流 | PA4 / PA5，ADC1 IN4 / IN5 | `drivers/adc_dma.c` | INA180 输出，三通道 DMA 快照 |
| IMU | PC10 / PC11 / PC12，SPI3 | `drivers/imu_bmi270.c` | SCK / MISO / MOSI |
| IMU 片选 | PA15 | `drivers/imu_bmi270.c` | 软件 CS |
| CAN FD | PB3 / PB4，CAN3，MUX11 | `drivers/canfd_driver.c` | 500 kbit/s 仲裁、4 Mbit/s 数据、BRS |
| 前 / 后 / 左防撞 | PC15 / PC13 / PC14 | `drivers/bumper.c` | 低有效、EXINT 锁存 |
| 右防撞 | PB9 | `drivers/bumper.c` | 低有效、EXINT 锁存 |
| 调试下载 | ST-Link SWD | `tools/at32f456_stlink_program.ps1` | 已在实板烧录流程验证 |

## 4. 资源阅读方式

先画出四条线，再读代码：

```text
输出：TMR8 → AT8236 → 电机
反馈：编码器 / BMI270 / ADC DMA → services
通信：CAN3 RX ISR → protocol → app
安全：防撞条 / 电气 / TTL / WDT → active brake
```

这样读时，学生能区分“哪个引脚连到哪里”和“谁决定运动”。GPIO 驱动只做硬件读写；
`chassis_app` 才是唯一决定 ENABLE、RUNNING、FAULT 和停车的状态机。

## 5. 需要先确认、不能凭空补写的资源

当前正式源码没有启用调试 UART 或板载 LED。因此，课程的第一版 Hello World 不假定
PA9/PA10 UART 或某个 LED 可用。旧版 JSON 中虽有相关网络，但它不是当前板的原理图证据。
如果后续要把 Hello World 从调试器变量换成串口文本或 LED 闪烁，应先提供新版原理图，
确认连接器、引脚、电平和外设实例。
