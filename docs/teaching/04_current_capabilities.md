# 04：当前固件已经实现什么

## 1. 正式代码如何分层

```text
drivers
  GPIO / TMR8 / EXINT / ADC DMA / SPI3 / CAN3 / SysTick / WDT
      ↓
services
  差速与速度 PID / 里程计 / 电气安全 / CAN 报文
      ↓
app
  ENABLE、RUNNING、FAULT 状态机与唯一控制循环
```

`src/app/main.c` 只启动时钟、板级时间、`chassis_app_init()`，再每约 10 ms 调用
`chassis_app_step()`。没有 FreeRTOS；实时性来自 SysTick、外设中断和短主循环。

## 2. 已实现能力地图

| 能力 | 主要模块 | 当前行为 |
| --- | --- | --- |
| 1 ms 时间基 | `board_port.*` | SysTick 为 TTL、调度、测速提供共同毫秒钟 |
| PWM 与制动 | `motor_driver.*` | TMR8 四路 20 kHz PWM；Coast / Brake 明确区分 |
| 编码器测速 | `motor_encoder.*` | 左右 AB 相 EXINT 计数 |
| 差速运动学 | `drive_control.*` | `v/ω` 转左右轮 RPM，共同比例限幅与斜率限制 |
| 速度闭环 | `drive_control.*` | 前馈 + PID，编码器 RPM 低通，堵转检测 |
| 相对里程计 | `odometry.*` | 编码器位移与 BMI270 Z 轴角速度融合 |
| IMU | `imu_bmi270.*` | SPI3、静止零偏、样本新鲜度检查 |
| 电气监督 | `adc_dma.*`、`safety.*` | 母线、左右电流、DMA 新鲜度和锁存故障 |
| 防撞条 | `bumper.*` | 四方向低有效输入、边沿锁存、主循环制动 |
| CAN FD | `canfd_driver.*`、`chassis_protocol.*` | V0.3 的控制、Reference、状态和里程计四报文 |
| 顶层状态机 | `chassis_app.*` | DISABLED → ENABLED → RUNNING / FAULT |

## 3. 当前协议只承担什么

上位机发出 `ENABLE` 后，底盘确认状态，再接收连续的 `0x401` 速度 Reference：

```text
0x201 ENABLE
  → 0x181 ENABLED
  → 0x401 (v, ω, accel, decel, TTL, sequence)
  → 差速 + 速度 PID
  → STOP / TTL 到期 / 本地故障
```

`0x401` 的 TTL 是本地失联停车条件；STOP、ESTOP、防撞、电压电流、IMU 新鲜度和
看门狗仍由底盘板优先处理。

## 4. 已有实车证据与边界

已完成的动态验收包括通信、前进、后退、500 mm 距离、90° 转向、回正与 360° 转向。
近期修复了电流换算中误写的 10 倍系数；该修复后 300 mm/s 直行与停车正常。
完整记录见 `hardware/v03_stlink_dynamic_acceptance_2026-09-01.md`。

这不等于全部标定已经结束。轮周长、轮距、速度精度、制动距离、绝对航向误差和电气阈值
仍应由后续实验分别标定。教学中必须区分“功能链路已跑通”和“物理指标已被量化验收”。

## 5. 当前不属于电机板的能力

- RFID 读取、标签解释、任务和路线决策；
- TOF、IR、视觉和上位机定位策略；
- 地图、站点、转弯半径和业务状态机；
- 真正的 PWM 同步电流环或转矩控制。

这些能力应在各自硬件节点或上位机层实现，最后只向本板发送受约束的 `v/ω`。
