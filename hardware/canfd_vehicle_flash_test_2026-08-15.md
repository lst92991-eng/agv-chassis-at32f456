# 电机板整车 CAN FD 对齐、ST-Link 烧录与静态验收记录

> 历史验收记录：下文`printf.*`名称是2026-08-15当时的构建证据。当前生产工程已
> 收口为`mdk_v5/agv_motor.uvprojx`，应使用`tools/build_motor_firmware.ps1`生成
> `mdk_v5/objects/agv_motor.bin`，不要照抄下文旧产物路径。

日期：2026-08-15

对象：AT32F456RCT7 电机驱动板，正式底盘控制固件

安全边界：未发送 `CLAIM_CONTROL`、`ENABLE_MOTION`、`GOTO_REL` 或速度设定值；测试期间电机保持停车。

## 1. 本次固件变化

- CAN3 保持 PB3/RX、PB4/TX、MUX11，位时序从 1 Mbit/s / 4 Mbit/s 改为整车统一的 500 kbit/s / 1 Mbit/s。
- 使用 ISO CAN FD、11 位标准 ID、BRS 和控制器无限自动重传。
- 删除线上 PDO 的私有版本头和应用层 CRC，改为协议 V0.1 的逐字段小端布局。
- 接收 PDO：命令 `0x201/32B`、租约 `0x301/12B`、速度预留 `0x401/16B`。
- 发送 PDO：底盘状态 `0x181/32B`、命令结果 `0x281/16B`、里程计 `0x381/24B`、安全状态 `0x481/16B`。
- `0x181` 和 `0x481` 周期 100 ms，`0x381` 周期 20 ms；旧私有电机诊断 PDO 不再上总线。
- CAN 运动增加安全门：上电默认运动禁用；必须先取得非零会话并显式 `ENABLE_MOTION`；租约丢失、急停、bus-off 和故障复位后回到运动禁用。
- `STOP`、`ESTOP`、复位、使能/禁用命令返回 `RECEIVED -> ACCEPTED -> SUCCEEDED`；长任务进入 `EXECUTING` 后再返回终态。

## 2. 构建产物

Keil 工程：`mdk_v5/printf.uvprojx`

工具链：ARMCC 5.06 update 7 build 960

结果：0 errors，0 warnings

BIN：`mdk_v5/objects/printf.bin`

大小：53,268 bytes

SHA-256：`C181C58E7790E2BD04697C1235D71E4EA3384421406FD2FDDA559E16BB7548B1`

构建与导出命令：

```powershell
$uvArgs = @('-b', 'D:\AGV\code_model\agile-drive_-ch32\mdk_v5\printf.uvprojx', '-j0', '-o', 'D:\AGV\code_model\agile-drive_-ch32\mdk_v5\canfd_vehicle_build_20260815.log')
Start-Process -FilePath 'D:\App\Keil\UV4\UV4.exe' -ArgumentList $uvArgs -Wait -WindowStyle Hidden
& 'D:\App\Keil\ARM\ARMCC\bin\fromelf.exe' --bin --output 'D:\AGV\code_model\agile-drive_-ch32\mdk_v5\objects\printf.bin' 'D:\AGV\code_model\agile-drive_-ch32\mdk_v5\objects\printf.axf'
```

## 3. ST-Link 烧录

探针：ST-Link V2J40M27，USB 序列号 `066EFF515471525067244941`。

实测目标电压：约 2.82～2.85 V。

实测 SWD DPIDR：`0x2BA01477`，目标为 Cortex-M4 r0p1。

烧录命令：

```powershell
& 'D:\AGV\code_model\agile-drive_-ch32\tools\at32f456_stlink_program.ps1' `
  -Firmware 'D:\AGV\code_model\agile-drive_-ch32\mdk_v5\objects\printf.bin' `
  -RunAfterProgram
```

脚本使用官方 `AT32F456_256.FLM` 在 RAM 中完成扇区擦除和分页编程。最终记录：

```text
FLASH_VERIFY_OK bytes=53268
```

本板 SWD 需要显式指定 `CPUTAPID=0x2BA01477`。NRST 未可靠接入，因此运行阶段必须使用 Cortex-M `SYSRESETREQ`，不能只依赖 ST-Link 的硬件复位。软件复位后停核抽查 PC 为 `0x080083B6`，证明程序已从 Flash 执行。

若 FLM 首次调用偶发超时，先用脚本相同的 ST-Link/DAP 参数执行一次软件 `reset halt`，再重新执行完整烧录；不得跳过 `verify_image`。

## 4. 静态 CAN FD 验收

适配器：USB2CANFD，500 kbit/s 仲裁段、1 Mbit/s 数据段、BRS。

接收到的电机板帧：

| CAN-ID | DLC | 实测频率 | 结果 |
| --- | ---: | ---: | --- |
| `0x181` | 32 | 约 10 Hz | 布局和周期正确 |
| `0x381` | 24 | 约 50 Hz | 时间戳按 20 ms 增长，布局正确 |
| `0x481` | 16 | 约 10 Hz | 布局和周期正确 |

全量烧录并复位三块板后，只监听 60 秒，不发送控制、使能或运动命令：

| 节点 | CAN-ID | DLC | 60 秒帧数 | 板端周期检查 |
| --- | --- | ---: | ---: | --- |
| 电机板 | `0x181` | 32 | 600 | 约 10 Hz |
| 磁导航 | `0x182` | 48 | 6002 | 10 ms，序号跳变 0 |
| BMS | `0x183` | 32 | 60 | 1000 ms |
| 电机板 | `0x381` | 24 | 2999 | 20 ms |
| 电机板 | `0x481` | 16 | 600 | 100 ms |

- 总计 10,261 帧，全部为 CAN FD+BRS，原始 CAN 错误帧 0，USB 发送回显 0。
- 磁导航时间戳异常 0、序号跳变 0；首次监听无 `0x182`，磁导航单独断电 5 秒再上电后恢复。
- BMS 末帧示例：23,750 mV、SOC 84%、单体 3,977～3,980 mV、压差 3 mV、26.0 °C，无活动故障。
- 电机板安全状态帧故障位和警告位均为 0；本轮未验证任何会导致电机运动的命令。

## 5. 结论与下一轮边界

- 当前 `500 kbit/s / 1 Mbit/s` 是已烧录、已回读、整车三节点 60 秒零错误的回退基线。
- 下一轮 5 Mbit/s 仅改变数据段，必须先确认三种控制器均能得到合法位时序，再全量构建和烧录；不得只改 USB2CANFD。
- 5 Mbit/s 先做 USB2CANFD 与单节点短线 A/B，再逐个接回 BMS 和磁导航，最终重复 60 秒错误帧、周期和序号检查。
- RS-485 与 CAN FD 分开验收：先检查 A/B/GND、端接与偏置，再做单节点请求应答、全网轮询、超时/CRC/地址冲突测试。
- 电机落地运动、PID 和磁条动态算法不属于本次静态通信验收。
