# 电机板 CAN FD 4 Mbit/s 烧录与整车静态验证

> 历史验收记录：下文`printf.*`名称是当时构建证据。当前生产工程已改为
> `mdk_v5/agv_motor.uvprojx`，标准构建入口为`tools/build_motor_firmware.ps1`。

日期：2026-08-15

安全边界：本轮只接收周期 PDO，没有发送 `CLAIM_CONTROL`、
`ENABLE_MOTION`、`GOTO_REL` 或速度设定值，电机保持停车。

## 修改和位时序

- CAN3：PB3/RX、PB4/TX、MUX11，ISO CAN FD、11 位标准 ID、BRS、自动重传。
- PCLK 为 192 MHz，仲裁段 `192 MHz / (2 * (144+48)) = 500 kbit/s`。
- 数据段 `192 MHz / (2 * (18+6)) = 4 Mbit/s`，采样点 75%，SJW 为
  `2*6/192 MHz = 62.5 ns`，SSP 为 19。
- 周期发送保持不变：`0x181/32B/10Hz`、`0x381/24B/50Hz`、
  `0x481/16B/10Hz`。

## 可复现构建

```powershell
$project='D:\AGV\code_model\agile-drive_-ch32\mdk_v5\printf.uvprojx'
$log='D:\AGV\code_model\agile-drive_-ch32\mdk_v5\canfd_4m_build_20260815.log'
$uvArgs=@('-b', $project, '-j0', '-o', $log)
Start-Process -FilePath 'D:\App\Keil\UV4\UV4.exe' `
  -ArgumentList $uvArgs -Wait -WindowStyle Hidden
& 'D:\App\Keil\ARM\ARMCC\bin\fromelf.exe' --bin `
  --output 'D:\AGV\code_model\agile-drive_-ch32\mdk_v5\objects\printf.bin' `
  'D:\AGV\code_model\agile-drive_-ch32\mdk_v5\objects\printf.axf'
Select-String -Path $log -Pattern '0 Error|0 Warning|Error:|Warning:'
Get-FileHash -Algorithm SHA256 `
  D:\AGV\code_model\agile-drive_-ch32\mdk_v5\objects\printf.bin
```

ARMCC 5.06 update 7 build 960：0 Error、0 Warning。BIN 为 53,268 Byte，
SHA-256：`0EA116E70C54409E41DE32902D4671918041A02641021119EC163AC4F3E8B9F4`。

## ST-Link 烧录

探针为 ST-Link V2J40M27，序列号 `066EFF515471525067244941`，实测目标电压
约 2.75～2.81 V，DPIDR `0x2BA01477`：

```powershell
& D:\AGV\code_model\agile-drive_-ch32\tools\at32f456_stlink_program.ps1 `
  -Firmware D:\AGV\code_model\agile-drive_-ch32\mdk_v5\objects\printf.bin `
  -RunAfterProgram
```

脚本使用 100 kHz SWD 和官方 `AT32F456_256.FLM`，逐扇区擦除、逐页编程，
检查每个 FLM 调用返回 0，并用 `verify_image` 校验 53,268 Byte。成功标志必须为：

```text
FLASH_VERIFY_OK bytes=53268
RESET_RUN_OK
```

首次 FLM 初始化曾超时；使用相同 OpenOCD 参数执行一次
`init; reset halt; wait_halt 3000; shutdown` 后，重新运行完整脚本即成功。不得
在超时后跳过重新编程或 `verify_image`。本轮还修正了 `-RunAfterProgram` 的参数
传递和退出码保存，避免 PowerShell 将换行后的 `-f` 误解释为命令、并避免沿用
旧的 `$LASTEXITCODE` 误报复位成功。手工复位抽查 PC 为 `0x080061F8`，随后恢复
运行；COM22 持续输出 `POSE state=IDLE fault=0`。

## 整车监听结果

拓扑：`USB2CANFD(120 Ω) -> 电机板 -> BMS -> 磁导航(120 Ω)`；全部 GND
共地，断电测 CAN-H/CAN-L 约 60 Ω。三板统一烧录后执行只读监听：

```powershell
$py='C:\Users\lst\.cache\codex-runtimes\codex-primary-runtime\dependencies\python\python.exe'
$env:PYTHONPATH='D:\AGV\tmp\canfd_monitor_runtime'
& $py D:\AGV\code_model\mag-track_-ch32\tools\canfd_vehicle_monitor.py `
  --duration 60 --data-bitrate 4000000
```

| CAN-ID | 内容 | 60 秒帧数 | 频率 | 结果 |
| --- | --- | ---: | ---: | --- |
| `0x181` | 电机状态 | 600 | 10.000 Hz | 32B、FD+BRS |
| `0x381` | 电机里程计 | 2,999 | 49.983 Hz | 24B、FD+BRS |
| `0x481` | 电机安全状态 | 600 | 10.000 Hz | 16B、FD+BRS |

同一窗口还收到磁导航 `0x182` 6,005 帧、BMS `0x183` 60 帧。全网总计
10,264 帧，预期 ID 无缺失，长度和 FD/BRS 异常为 0，CAN 错误帧为 0，磁导航
序号跳变为 0。整机静态 CAN FD 判定通过。

磁导航第一次全网监听没有 `0x182`，明确重新断电上电后恢复；故障时未读取其
CAN 错误寄存器，因此不能直接判定为 Bus-Off。完整复现流程和后续诊断边界见
磁导航仓库 `docs/canfd_vehicle_4m_validation_2026-08-15.md`。

## 尚未覆盖

本记录不覆盖落地运动、负载变化、急停实车动作、PID 调整或磁条跟随。下一阶段
应先完成 RS-485 全网验收，再用上位机和磁条做动态磁导航，最后架空低速确认
方向和编码器极性后进入落地 PID 调试。500 kbit/s / 1 Mbit/s 记录保留为回退
基线。

## 后续进展

同日后续已完成右轮机械方向确认、右编码器反馈符号修正、重新编译烧录、架空
低速闭环复测、四路防撞输入方向测试和右侧防撞动态安全链路测试。四块超声波
阵列的旧位置还确认需要整体旋转 180° 解释。最新方位、数据和未完成项统一见
[`vehicle_orientation_and_safety_validation_2026-08-15.md`](vehicle_orientation_and_safety_validation_2026-08-15.md)。
