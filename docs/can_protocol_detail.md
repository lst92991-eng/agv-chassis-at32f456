# CAN 协议与接入详细参考

适用版本：2026-10-27下位机执行/遥测重构。交付时以Git提交号及固件SHA-256绑定版本，不以可变的main分支名代替版本。

## 先确认职责和版本

下位机负责CAN解析、差速换算、双轮速度PID、传感器采集和遥测。上位机负责运动计划、速度边界、保护策略、通信监控和停止决策。

本次保持四种CAN报文的ID、长度、字段位置、小端编码和命令码，改变的是执行策略：

- 上电静止，新的0x401可以直接起步，无前置ENABLE要求。
- 目标持续保持到新目标或显式停止；TTL字段仍在，但下位机不执行TTL超时停车。
- 电气、传感器、防撞和CAN诊断不会自动撤销目标；fault非零不再等同于下位机已停车。
- STOP、DISABLE、ESTOP等由上位机明确发出。CAN发送成功不等于命令已执行，必须检查回执与反馈。
- CAN完全失联时，不能指望继续向同一失效链路发送STOP就能停车；上位机系统应自行定义这类情况的停止执行途径。

已烧录并完成架空验证的BIN为36180字节，SHA-256：

```text
E8E67EDAD75A1BAAFC74AD64903CF6CA505BB4A8D347F07F2A2F33989FC66C6E
```

现有四帧协议没有固件版本查询字段，不能仅凭收到24字节0x181就判断已升级。请保留交付包中的版本清单和烧录记录。

## 接线与Linux接口

CAN_H接CAN_H、CAN_L接CAN_L，按硬件设计连接参考地；总线两端各120Ω。中间节点不额外增加终端电阻。

链路必须使用11位标准ID、ISO CAN FD、BRS；仲裁500kbit/s、数据4Mbit/s，仲裁采样点75%。**8字节的0x201也必须是FD+BRS帧，不能按Classic CAN发送。**

```bash
sudo ip link set can0 down
sudo ip link set can0 type can bitrate 500000 sample-point 0.75 dbitrate 4000000 dsample-point 0.75 fd on
sudo ip link set can0 up
ip -details -statistics link show can0
```

本次WeActStudio/Candlelight适配器将数据段采样点量化为73.3%，实测可通信。默认仲裁采样点87.5%曾导致接收失败。

## 发送运动目标：0x401，16字节

所有多字节字段均为小端。v为车体中心线速度，前进为正；omega为车体角速度，左转为正。

| 偏移 | 类型 | 含义 |
| ---: | --- | --- |
| 0 | uint16 | Reference sequence |
| 2 | uint16 | TTL，保留字段，当前下位机不使用其计时 |
| 4 | int16 | v，mm/s |
| 6 | int16 | omega，mrad/s |
| 8 | uint16 | accel，mm/s²；0表示直接更新 |
| 10 | uint16 | decel，mm/s²；0表示直接更新 |
| 12 | uint32 | 保留，必须为0 |

Python编码示例，不依赖结构体ABI：

```python
payload = struct.pack("<HHhhHHI", reference_seq, 120, v_mm_s,
                      omega_mrad_s, accel_mm_s2, decel_mm_s2, 0)
```

新上电、sequence=1、v=50mm/s、omega=0、accel=100、decel=500：

```text
ID=0x401  FD=1  BRS=1  LEN=16
01 00 78 00 32 00 00 00 64 00 F4 01 00 00 00 00
```

只需发送一次即可保持该目标。改变速度时发送下一条更新sequence的目标，不要求不断刷新同一目标。
四个运动量v/omega/accel/decel全零表示HOLD制动；v/omega为零但带斜率时，按斜率减速到零。

下位机不再限制原300mm/s、1200mrad/s、75RPM边界，PWM仍受硬件[-1000,1000]表示范围约束。
本轮只验收了50～300mm/s的架空前进变速，不能据此扩大已验证范围。

## 管理命令：0x201，8字节

```python
payload = struct.pack("<BBHI", command_code, 0, command_seq, 0)
```

| 命令码 | 名称 | 行为 |
| ---: | --- | --- |
| 1 | ENABLE | 解除DISABLE、清旧目标并等待新目标；不能解除ESTOP |
| 2 | DISABLE | 停车并禁止运动帧，直到ENABLE或FAULT_RESET |
| 3 | STOP | 停车并清旧目标；之后的新运动帧可重新启动；已有DISABLE/ESTOP仍有效 |
| 4 | ESTOP | 上位机显式锁存停止，必须FAULT_RESET解除 |
| 5 | FAULT_RESET | 解除ESTOP/DISABLE并保持静止，清旧目标；尝试清碰撞历史及恢复CAN |
| 6 | RESET_ODOMETRY | 清相对里程和编码器、同步测速基准，不改变当前目标 |

首次管理命令为STOP、command_seq=1时：

```text
ID=0x201  FD=1  BRS=1  LEN=8
03 00 01 00 00 00 00 00
```

等待0x181的command_sequence匹配、command_result=0，并检查state/PWM；需要确认停稳时，再观察连续零速度里程计。
已有ESTOP时，STOP可保持state=3；不能要求所有停止都返回state=0。

## 两个序号要分别维护

- 管理命令sequence与Reference sequence是**两套独立的16位计数器**。
- 常规新报文满足`delta = uint16(new - previous)`且`0 < delta < 32768`，支持65535→0回绕。
- Reference重复/倒退序号被忽略；普通管理命令同样需要更新序号。
- STOP、DISABLE、ESTOP允许旧/重复序号，但不会倒退普通命令的历史。正常发送仍建议统一递增。
- 同一处理批次优先级为ESTOP > DISABLE > STOP > 普通命令。不要连续堆积互相矛盾的管理命令，应等待对应回执。
- STOP、ENABLE、FAULT_RESET等不会清除序号历史；它们会清除目标及已经进入接收队列的旧Reference。等命令回执后再发新目标。
- 上位机重启而板子未重启时，不要把序号直接归零。持久保存计数，或在首次接入时协调板端重启。当前协议没有读取最后Reference序号的字段。

## 接收状态：0x181，24字节

| 偏移 | 类型 | 含义 |
| ---: | --- | --- |
| 0 | uint8 | state：0静止、1ENABLE/HOLD、2执行目标、3上位机ESTOP锁存 |
| 1 | uint8 | fault/诊断位，见下表 |
| 2 | uint8 | 防撞历史：bit0前、bit1后、bit2左、bit3右 |
| 3 | uint8 | flags：bit0 IMU有效；bit1里程计和IMU同时有效 |
| 4 | uint16 | 最近被处理的管理命令sequence |
| 6 | uint8 | command_result：0接受，1拒绝 |
| 7 | uint8 | 保留0 |
| 8 | uint32 | 板端运行毫秒数 |
| 12 / 14 | int16 | 左/右内部PWM千分比，不是RPM，也不是车体方向符号 |
| 16 / 18 / 20 | uint16 | 母线mV、左电流mA、右电流mA |
| 22 | uint16 | 保留0 |

| 诊断位 | 当前含义 |
| --- | --- |
| 0x01 | 防撞历史位非零，仅报告 |
| 0x02 | 上位机ESTOP命令锁存，与state=3对应 |
| 0x08 | CAN链路/驱动诊断，仅报告 |
| 0x10 | IMU数据无效，仅报告 |
| 0x40 | ADC没有有效新样本，仅报告；不再表示下位机欠/过压或过流判决 |
| 0x04 / 0x20 | 当前固件不再产生自动超时/轮控锁存位 |

除上位机明确设置的ESTOP外，诊断位不会改变运动状态。ADC无效时保留最近测量值，必须结合诊断位解释，不能把旧值当成实时数据。
电流静止零点标定不阻挡起步；未完成前使用零偏0。电气采样准确性不在本轮11米运动验收范围内。

## 接收里程计：0x381，20字节

| 偏移 | 类型 | 含义 |
| ---: | --- | --- |
| 0 / 4 | int32 | x/y，mm，复位原点下的相对位姿 |
| 8 | int32 | yaw，0.01°，左转为正 |
| 12 | int16 | 车体线速度，mm/s |
| 14 | int16 | yaw_rate，0.01°/s，注意与发送omega的mrad/s不同 |
| 16 | uint32 | 板端运行毫秒数 |

0x181目标周期100ms，0x381以20ms为目标，但同周期优先发状态，硬件忙时也可能丢本次遥测。
上位机使用实际接收时间及板端时刻判断数据新鲜度，不把目标周期当作严格到达保证。

## 最小联调流程

1. 对齐交付版本，配置CAN并先运行`python3 tools/ubuntu_canfd_idle.py status`。
2. 新上电时发送上面的单条0x401，观察state=2、PWM变化和0x381速度。
3. 发送更新序号的目标改变速度；发送STOP并等待回执、双PWM为0及连续零速度反馈。
4. 分别验证DISABLE→ENABLE、ESTOP→FAULT_RESET后的新目标行为。
5. 上位机自行实现超时、保护、速度边界、诊断处理和停止策略；本次固件不替它做这些决策。

仓库已提供标准库SocketCAN示例与有限测试器，详见[Ubuntu最小操作](../tools/UBUNTU_IDLE_TEST.md)。
默认每次Reference从1开始，板子不重启时通过`--reference-start`接续计数。
测试器的3秒/180秒时限和反馈检查属于上位机工具，不属于下位机。

## 常见接入问题

- 无帧且ERROR-PASSIVE：核对500k/4M、FD+BRS、75%仲裁采样点、接线与两端终端。
- 只有RX增长、TX不增长：本次适配器曾出现`failed to xmit URB: -EPIPE`，普通can0 down/up未恢复；执行`usbreset`查看当前设备号，再用`sudo usbreset BBB/DDD`复位对应USB-CAN设备，重新配置can0。不要复用别人的USB设备号。
- 运动帧无效：核对LEN=16、保留位0、独立的Reference序号；检查是否仍处于DISABLE或ESTOP。
- 管理命令没有匹配回执：区分USB写入、总线发送计数和板端处理结果；不能仅凭send返回成功判定命令已执行。

## 交付清单与验收证据

交付给上位机开发者：固定Git提交、本文、当前架构/协议说明、SocketCAN示例、已测试BIN及SHA-256、11米实测记录。
仓库不提交构建二进制与原始日志；需要离线交付时使用本次本地交付包，包内包含固件、源码快照、原始日志和版本清单。

已完成Windows与Ubuntu六组主机测试、Keil 0错误0警告构建、实板烧录校验，以及架空11.0004米/93.7秒的50→300→50mm/s变速与STOP验证。
详见[11米测试记录](../hardware/execution_11m_raised_2026-10-27.md)。
