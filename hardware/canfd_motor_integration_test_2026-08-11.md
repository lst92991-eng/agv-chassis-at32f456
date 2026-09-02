# CAN FD 电机联调记录（2026-08-11）

> 这是历史单板测试记录，其中的 1 Mbit/s / 4 Mbit/s 参数和私有协议已被取代。当前整车基线为 500 kbit/s / 1 Mbit/s、BRS，见 [`canfd_vehicle_flash_test_2026-08-15.md`](canfd_vehicle_flash_test_2026-08-15.md)。
>
> 后续静态审查发现的命令仲裁、会话生命周期和上位机发送问题，统一记录在 [`canfd_code_design_review_2026-08-13.md`](canfd_code_design_review_2026-08-13.md)。本记录证明单次主链路测试通过，不代表整车安全准入。

## 1. 测试目的

验证新电机驱动板固件、USB-CAN FD 适配器和 Qt 上位机之间的完整链路，重点确认：

- CAN FD 参数与固件一致，持续通信不进入 Error Passive/Bus-Off。
- 心跳、清故障、指定距离运动和停车命令能够被板端接收并确认。
- `GOTO X=300 mm` 执行完成后，板端上报完成事件和 300 mm 里程计结果。
- 通信异常时保持本地停车和故障锁存，不绕过现有安全保护。

## 2. 测试环境

| 项目 | 配置 |
| --- | --- |
| 电机板 MCU | AT32F456RCT7 |
| 固件工程 | `agile-drive_-ch32` |
| 下载器 | ST-Link，下载后校验通过并复位运行 |
| USB-CAN FD | WeActStudio USB2CANFDV1，gs_usb/CandleLight FD |
| 上位机 | `MCU Debug Tool`，CandleLight CANFD + CAN Frame |
| 仲裁段 | 1 Mbit/s，采样点 75% |
| 数据段 | 4 Mbit/s，采样点 75%，BRS 开启 |
| 心跳周期 | 100 ms |
| 运动命令 | X=300 mm，Y=0，yaw=0，速度 50 mm/s |

采样点必须与板端 75% 配置一致。上位机原默认仲裁段/数据段采样点为 87.5%/80%，持续遥测时出现 CAN 错误累积；两段均调整为 75% 后链路测试和运动测试通过。

## 3. 测试步骤

1. 固件重新下载并校验，复位后进入正式底盘模式。
2. 上位机连接 CandleLight CANFD，确认 `RX frames` 持续增长。
3. 启动 `Heartbeat 100ms`，由上位机生成新的非零会话 ID，并同步更新各快捷命令 CRC。
4. 发送 `Stop`，再发送 `Clear Fault`；等待板端返回 RECEIVED/ACCEPTED。
5. 发送 `Move 300mm`，保持 100 ms 心跳直到完成。
6. 收到 COMPLETED 后再发送一次 `Stop`，关闭周期心跳。

命令行复核命令：

```powershell
python tools\canfd_agv_test.py goto --distance-mm 300 --speed-mm-s 50
```

## 4. 通过结果

| 检查项 | 结果 |
| --- | --- |
| 固件下载与镜像校验 | 通过，镜像 53,396 bytes |
| CAN FD 链路 | 通过，1 Mbit/s / 4 Mbit/s / 75% / BRS |
| STOP | 收到 RECEIVED 和 ACCEPTED |
| CLEAR_FAULT | 收到 RECEIVED 和 ACCEPTED，故障清零 |
| GOTO 300 mm | 收到 RECEIVED、ACCEPTED、COMPLETED |
| 里程计 | X 从 0 mm 连续增长到 300 mm |
| 协议解析 | `bad_crc=0` |
| 遥测 | `0x201/0x211/0x221/0x231` 均持续收到 |
| 最终停车 | STOP 收到 RECEIVED 和 ACCEPTED |
| 图形上位机操作 | 用户现场确认测试通过 |

命令行完整测试摘要：

```text
PASS rx=519, bad_crc=0, telemetry=201/211/221/231,
completed=1, odom_x_mm=300
```

## 5. 已知现象和安全结论

- 板端在 USB-CAN 尚未连接时发送遥测，可能先锁存历史 CAN 链路故障 `0x0020`；适配器连接稳定后，可在有效会话内执行 `CLEAR_FAULT`。
- 上位机曾出现 `Failed to write all command bytes` / WinUSB 写超时并伴随界面卡顿；重启上位机后本轮测试通过。该现象尚需长时间压力测试，出现时不得重复点击运动命令，应先停止周期发送并断开重连。
- 防撞条本轮未做实体触发测试。固件已实现本地立即停车、锁存故障和 CAN FD 安全事件上报，必须在车轮落地前单独完成四路输入验证。
- 当前 PID 为台架基线：20 ms，Kp=4.0，Ki=8.0，Kd=0。它不是整车负载最终参数，落地后需按左右轮分别记录阶跃、稳态误差、峰值电流和制动距离再调参。
- 当前 `GOTO` 只开放正向直行 300..3000 mm、50..100 mm/s；倒车、转向和速度流保持禁用。

## 6. 整车联调准入项

- 四路防撞输入方向、按下电平、锁存和事件上报全部通过。
- 急停、心跳超时、拔掉 CAN 和 Bus-Off 恢复路径分别验证。
- 轮径、减速比和编码器每转计数实车标定，300 mm 使用卷尺复测。
- 带载重新整定左右轮 PID，并确认 PWM、速度和电流限幅。
- 连续运行至少 30 分钟，不出现 WinUSB 写超时、CAN 错误累积或遥测丢失。
