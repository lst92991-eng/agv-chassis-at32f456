# 底盘 P0 安全修复与四路防撞动态回归

日期：2026-08-17

对象：AT32F456 电机驱动板、整车 CAN FD 网络、右/左/后/前四路防撞条

测试条件：车辆架空，主动轮离地；CAN FD 为 500 kbit/s 仲裁段、4 Mbit/s
数据段、BRS 开启。

## 1. 验收结论

本轮 P0 安全补丁已完成编译、实板烧录和回归测试：

- STOP、ESTOP、通信超时和 CAN 链路故障现在构成普通命令队列屏障；屏障前
  已排队但尚未执行的命令不会在停车后重新启动运动。
- 防撞清故障同时检查实时低电平、按下沿待处理锁存、稳定输入和故障快照，
  短脉冲不会被清故障路径漏掉。
- CAN Bus-Off 故障只有在控制器退出 bus-off、error-passive 和
  error-warning 后才允许清除。
- CPU NMI、HardFault、MemManage、BusFault 和 UsageFault 都进入统一的关中断、
  主动制动安全停机路径。
- 停车状态继续刷新编码器反馈，停止后的 CAN 线速度不再冻结为运动前旧值。
- 右、左、后、前四路防撞动态测试均触发本地制动、取消活动任务、锁存
  `0x0008`；用户已逐路确认车轮物理停止。
- 扩容后的 CAN 紧急结果队列通过 4 条旧命令加 STOP 的同周期压力测试，
  13 个相关结果帧全部可观察，没有再次丢失终态。

这表示本轮涉及的软件安全链路和架空动态回归已经通过；它不等同于经认证的
功能安全，也不替代车辆落地后的停止距离、负载和硬件断能测试。

## 2. 修复内容

### 2.1 安全命令队列屏障

普通 FreeRTOS 命令队列项增加 `safety_epoch`。STOP、ESTOP、COMM_TIMEOUT 和
CAN_BUSOFF 第一次进入优先槽时推进 epoch；控制任务先处理优先命令，再拒绝
epoch 已过期的普通命令。优先命令等待处理期间，新普通命令也会直接返回忙。

该机制关闭了以下危险时序：

```text
旧普通命令入队 -> STOP 到达 -> STOP 先制动 -> 旧命令随后又启动电机
```

现在对应时序为：

```text
旧普通命令入队 -> STOP 推进 epoch -> STOP 先制动 -> 旧命令因 epoch 过期被拒绝
```

### 2.2 防撞与故障清除

- `bumper_input_has_pressed_latch()` 暴露 ISR 按下沿待处理锁存状态。
- `bumper_safety_can_clear()` 必须同时满足：实时 GPIO 无按下、ISR 待处理锁存为
  0、稳定活动位为 0、故障快照为 0、释放稳定时间满足要求。
- CAN 链路故障清除增加硬件健康检查，禁止在 bus-off/error-passive/
  error-warning 仍存在时只清软件故障位。

### 2.3 异常停机与停止反馈

- 五类 Cortex-M 异常统一调用 `motor_exception_safe_halt()`：关中断并执行
  `motor_driver_stop_all(MOTOR_STOP_BRAKE)`，随后保持停机。
- 非 DRIVE 状态在主动制动前调用 `motor_speed_control_resync_feedback()`，避免
  `0x381` 在线路已停止后仍保留旧线速度。

### 2.4 CAN 紧急结果队列

原紧急队列和普通遥测队列均为 8 槽环形队列，各只有 7 个可用位置。压力测试
曾出现 STOP 本身成功，但后两条旧命令的拒绝终态因紧急队列已满而不可观察。

修复后：

| 队列 | 槽数 | 可用帧数 | 设计目的 |
| --- | ---: | ---: | --- |
| 紧急结果 | 16 | 15 | 容纳命令结果、安全事件和同周期屏障回执 |
| 普通遥测 | 8 | 7 | 保持原上限，避免扩大陈旧周期状态的积压 |

队列结构改为显式帧缓冲区指针和容量，push/peek/pop 均按各自容量回卷，并对
空指针和非法容量做防御检查。

## 3. 构建与烧录

| 项目 | 结果 |
| --- | --- |
| 编译器 | Keil ARMCC 5.06 update 7 build 960 |
| 编译结果 | `0 Error(s), 0 Warning(s)` |
| 程序大小 | Code 44112 B，RO-data 9480 B，RW-data 492 B，ZI-data 15212 B |
| BIN 大小 | 53648 B |
| BIN SHA-256 | `7018EC453CF24C29D141F4F5D0C66AFC8557E11CB34417662E9807C3647397F0` |
| 调试器 | ST-Link，SWD 100 kHz |
| Flash 算法 | 官方 `AT32F456_256.FLM` |
| 校验 | `verify_image` 成功，53648 B 全部一致 |

首次 FLM 初始化仍发生一次目标 halt 超时；当时尚未开始扇区擦除。按既有恢复
流程执行 `reset halt` 后重新运行完整擦除、编程和校验，全部 FLM 调用返回 0。
烧录命令的外层等待窗口先结束，导致自动 `reset run` 未执行；随后明确执行
`reset halt -> resume`，复位向量为 `0x080002B8`，板端恢复周期上报。

队列扩容前一版 P0 安全固件的 BIN SHA-256 为
`5DD65DF42D3EA7D1FC4BEBB78D153BD09297E4C4EAB36D1F45B7935A1290C046`；
实板当前运行的是上表所列 `7018...97F0` 版本。

## 4. CAN 与命令屏障回归

### 4.1 P0 补丁烧录前后网络基线

P0 基础补丁烧录前、烧录后各执行 20 秒整网监听，结果均通过：

| CAN ID | 内容 | 频率 |
| --- | --- | ---: |
| `0x181` | 底盘状态 | 约 10 Hz |
| `0x182` | 磁导航状态 | 约 100 Hz |
| `0x183` | BMS 状态 | 约 1 Hz |
| `0x381` | 底盘里程计 | 约 50 Hz |
| `0x481` | 底盘安全状态 | 约 10 Hz |

两轮监听均未发现 CAN 错误帧、DLC/FD/BRS 异常或磁导航序号跳变。

### 4.2 普通 STOP 动态测试

架空状态下发送 50 mm/s 的 GOTO，再发送 STOP：

- STOP 收到 `RECEIVED -> ACCEPTED -> SUCCEEDED`。
- 活动 GOTO 收到中止终态。
- 后续轮速回到 0，未重新启动。
- 最终 `0x481` 为 `bumper=0`、`estop=0`、`safety=0`、`fault=0`、
  `warning=0`。

### 4.3 4 条旧命令加 STOP 压力测试

为确保 5 帧在同一 10 ms 控制周期前完成接收，测试在刚收到 `0x381` 后立即
发送 4 条 RESET_ODOM 和 1 条 STOP。最终通过样本：

| 命令 ID | 命令 | 结果 stage | 结果码/detail |
| ---: | --- | --- | --- |
| 700 | CLAIM_CONTROL | `1 -> 3` | `0/0` |
| 701 | RESET_ODOM | `0 -> 4` | `0/0 -> 1/5` |
| 702 | RESET_ODOM | `0 -> 4` | `0/0 -> 1/5` |
| 703 | RESET_ODOM | `0 -> 4` | `0/0 -> 1/5` |
| 704 | RESET_ODOM | `0 -> 4` | `0/0 -> 1/5` |
| 705 | STOP | `0 -> 1 -> 3` | 全部 `0/0` |

其中 stage `0/1/3/4` 分别为 RECEIVED/ACCEPTED/SUCCEEDED/REJECTED。
4 条 RESET_ODOM 都先入普通队列，STOP 推进安全 epoch 后优先成功，随后 4 条
旧命令全部因 epoch 过期被拒绝。结果帧无缺失。

同一 3 秒观察窗口还收到 `0x381` 150 帧、`0x481` 30 帧，最大线速度为
0 mm/s；最终安全状态为：

```text
bumper=0x00, estop=0x00, safety=0,
fault=0x00000000, warning=0x00000000
```

## 5. 四路防撞动态验收

每一路均按以下独立流程执行：清故障并确认全 0、以 50 mm/s 启动架空车轮、
只触发目标防撞条、确认任务终止和连续停止状态、释放并显式清故障。一路完成
且恢复全 0 后才进入下一路。

| 方向 | 防撞位 | 触发安全状态 | 活动任务 | 停止状态 | 清故障 | 物理观察 |
| --- | ---: | --- | --- | --- | --- | --- |
| 右 | `0x01` | state 2，fault `0x0008` | GOTO aborted | 连续 10 帧速度为 0 | 通过 | 用户确认停止 |
| 左 | `0x02` | state 2，fault `0x0008` | GOTO aborted | 连续 10 帧速度为 0 | 通过 | 用户确认停止 |
| 后 | `0x04` | state 2，fault `0x0008` | GOTO aborted | 连续 10 帧速度为 0 | 通过 | 用户确认停止 |
| 前 | `0x08` | state 2，fault `0x0008` | GOTO aborted | 连续 10 帧速度为 0 | 通过 | 用户确认停止 |

四次测试中 CAN 错误均为 0。每次防撞释放后，FAULT_RESET/CLEAR_FAULT 都收到
`RECEIVED -> ACCEPTED -> SUCCEEDED`，下一次运动不是自动恢复，而是由新的 GOTO
显式启动。最后一次前侧测试清除后的最终安全状态为全 0。

## 6. 当前整车安全状态

文档完成时：

- 四路防撞均已释放，防撞位 `0x00`。
- 急停来源 `0x00`。
- safety state 为 0。
- fault 和 warning 均为 0。
- 没有活动运动任务，架空轮速为 0。
- 最新队列版固件已经烧录并运行。

## 7. 尚未覆盖的安全项目

本轮通过项仍不能替代以下后续验收：

1. 车辆落地、额定载荷和不同地面附着条件下测量四路触发后的机械停止时间与
   停止距离。
2. 实测看门狗复位、编码器断线/反向、堵转、过流和驱动器硬件故障的停机路径。
3. 用可控故障注入验证 CAN error-warning、error-passive、bus-off、恢复和故障
   清除门槛。
4. 评估独立硬件急停/接触器断能链路；软件主动制动不能替代硬件最终断能。
5. 按 180° 修正后的物理位置重新配置四块超声波阵列的节点地址，并逐通道遮挡
   验证方位、盲区和串扰。

超声波阵列的正确位置映射继续以
[`vehicle_orientation_and_safety_validation_2026-08-15.md`](vehicle_orientation_and_safety_validation_2026-08-15.md)
第 2 节为唯一基准。

## 8. 代码与证据索引

- 命令 epoch、安全优先槽和控制任务：[`app_freertos.c`](../src/app_freertos.c)
- 防撞实时输入与按下锁存：[`bumper_input.c`](../src/bumper_input.c)、
  [`bumper_input.h`](../src/bumper_input.h)
- 防撞清除条件：[`bumper_safety.c`](../src/bumper_safety.c)
- CAN 队列、结果和链路健康：[`canfd_protocol.c`](../src/canfd_protocol.c)、
  [`canfd_protocol.h`](../src/canfd_protocol.h)
- 异常安全停机：[`at32f45x_int.c`](../src/at32f45x_int.c)
- 停止反馈刷新：[`chassis_motion_controller.c`](../src/chassis_motion_controller.c)
