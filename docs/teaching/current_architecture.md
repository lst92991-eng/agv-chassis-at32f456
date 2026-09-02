# 教学架构与人工验收

## 一条控制链

```text
上位机 v/ω
  → CAN FD驱动
  → 协议解析
  → 10ms底盘状态机
  → 差速与20ms速度PID
  → AT8236 PWM
  → 电机与编码器
```

`chassis_app`是唯一状态机：

```text
DISABLED → ENABLED → RUNNING
     ↑                 ↓
     └──── FAULT ←─────┘
```

## CAN FD教学协议

| 方向 | ID | 内容 |
| --- | ---: | --- |
| 上位机→底盘 | `0x201` | 控制命令和递增sequence |
| 上位机→底盘 | `0x401` | sequence、TTL、`v`、`ω`、加速度、减速度 |
| 底盘→上位机 | `0x181` | 状态、故障、防撞、电压、电流和命令结果 |
| 底盘→上位机 | `0x381` | x、y、相对Yaw、实际速度 |

`0x401`持续刷新就是通信租约。Reference TTL到期后，本板主动制动并锁存FAULT。

`0x201`固定8字节：`[命令码, 0, sequence_u16, 0_u32]`。命令码依次为1=ENABLE、
2=DISABLE、3=STOP、4=ESTOP、5=FAULT_RESET、6=RESET_ODOMETRY。

`0x401`固定16字节小端布局：`sequence_u16, ttl_ms_u16, v_mm_s_i16,
omega_mrad_s_i16, accel_mm_s2_u16, decel_mm_s2_u16, reserved_u32`。TTL范围50～300ms；
本板限制`|v|≤300mm/s`、`|omega|≤1200mrad/s`。

## 模块阅读顺序

先核对`drivers/at32f45x_clock.c`的系统时钟。中断入口保留官方文件名
`drivers/at32f45x_int.c`及`inc/at32f45x_int.h`；`drivers/board_port.c`负责时基和WDT。
SysTick先调用一次`board_port_tick_1ms()`保持原1ms计数；调度器启动后才调用
`xPortSysTickHandler()`。SVC/PendSV直接映射到官方ARM_CM4F移植入口，不能用普通C函数包装。

1. `drivers/motor_driver.c`：20kHz PWM、方向、Coast和Brake。
2. `drivers/motor_encoder.c`：左右AB编码器计数。
3. `drivers/adc_dma.c`、`bumper.c`、`imu_bmi270.c`：硬件输入。
4. `services/drive_control.c`：差速、共同限幅、斜率、前馈和PID。
5. `services/odometry.c`：编码器轮程和IMU航向融合。
6. `services/safety.c`：母线、电流和故障锁存。
7. `services/chassis_protocol.c`：四种CAN FD报文。
8. `app/chassis_app.c`：按固定顺序组合所有模块。
9. `app/app_freertos.c`：单静态控制任务按10ms调用同一个状态机；再由`main.c`启动调度。

FreeRTOS内核及移植层位于`third_party/freertos/`，属于第三方源码，不纳入自研行数。
`inc/FreeRTOSConfig.h`及`app_freertos.c/h`是本项目的OS配置/接入代码，需单独说明并计入自研支持代码。
任务、TCB和栈采用静态分配；不引入堆，也不恢复原先分散的业务队列与四任务模型。

## 人工验收

1. 架空：ENABLE后前进、后退、左右转和原地旋转；STOP必须制动。
2. 通信：停止发送0x401，确认TTL到期后两电机主动制动并进入FAULT。
3. 防撞：逐个按下四路防撞条，确认立即制动且FAULT_RESET前不能重新ENABLE。
4. 电气：确认ADC DMA样本连续；母线越出18.0～25.8V不能ENABLE；0.8A持续100ms停车。
5. 里程计：直行、后退、左右360°旋转和方形闭环；据实车结果标定轮周长和有效轮距。

相对里程计用于两次RFID、磁条或视觉校正之间的连续估算，不能替代上位机绝对定位。

## 最小修补后的测试入口

本版沿用上述四报文V0.3布局。`agv-supervisor`已同步为该布局并通过软件回归；构建后的现有
`agv_supervisor_motor_acceptance`是动态验收入口，用法见上位机README。
也可用支持500k/4M BRS的CAN FD收发工具按下列格式手工测试。

1. 静止上电约1秒，接收0x181，确认fault=0且flags的bit0=1（IMU有效）。
2. 发送ENABLE：0x201，8字节`01 00 01 00 00 00 00 00`。等待0x181中state=1、
   command_sequence=1、command_result=0，然后再发送运动Reference。
3. 先以100mm/s直行：0x401，16字节`01 00 78 00 64 00 00 00 F4 01 E8 03 00 00 00 00`。
   每20～50ms重复发送时，必须递增前两个字节的sequence；TTL为120ms。
4. 停止刷新Reference：到期后的下一控制周期应制动并进入state=3（FAULT）。
5. 发FAULT_RESET（命令码5，递增命令sequence）后重新ENABLE；再测STOP、四路防撞、
   停车后清里程再起步、左右圆弧和300mm/s直行。

STOP/DISABLE/ESTOP覆盖同批普通命令；成功ENABLE后只执行之后新到达的Reference。
STOP或故障清除不重置sequence历史，发送器必须持续递增；若发送器重新从0编号，需
同步重启板子。合法STOP/ESTOP本身不受这一序号限制。

硬件初始化失败时先保持静止并查电源/SPI接线；IMU首次校准失败需要重新上电。
运行中的短暂IMU通信错误恢复后仍需FAULT_RESET和ENABLE，不会自行恢复运动。
ADC重启后的FAULT_RESET只有在新鲜电气样本通过后才成功。

软件回归入口仍为`tools/run_host_tests.ps1`，覆盖里程计、轮控、电气、协议和应用
状态机五组测试。它们证明对应代码路径通过；真实SPI3、ADC DMA、制动距离和里程计
精度仍由本节的人工上板测试验收。
