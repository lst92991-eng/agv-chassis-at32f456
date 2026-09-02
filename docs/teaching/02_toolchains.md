# 02：Keil 与 OpenOCD 开发环境

## 1. 工具链定位

Keil 是本课程的主开发环境：编辑、编译、源码级调试和查看寄存器都以它为主。
OpenOCD 是补充工具：用于可脚本化烧录、校验和自动化复位。两者应使用同一个目标芯片、
同一个二进制文件，不能分别维护两份配置。

| 工具 | 当前用途 | 是否主路径 |
| --- | --- | --- |
| Keil uVision 5 | 编译 `agv_motor.uvprojx`、阅读分组、单步和 Watch | 是 |
| ARM Compiler 5.06 update 7 | 当前工程的 C 编译器 | 是 |
| ArteryTek AT32F45x DFP 2.0.1 | 器件包、SVD、Flash 算法 | 是 |
| ST-Link | SWD 下载与调试 | 是 |
| OpenOCD | 脚本化下载、逐字节校验、复位 | 辅助 |

## 2. Keil 配置

1. 安装 Keil MDK，并安装 `AT32F45x_DFP` 器件包。
2. 打开 `mdk_v5/agv_motor.uvprojx`。
3. 在 Target Options 中确认器件为 `AT32F456RCT7`，不要选相近型号替代。
4. 在 Project 窗口按 `app → services → drivers → config → bsp → third_party → firmware → cmsis` 阅读。
5. 首次只执行 **Build**，目标是 `0 Error(s), 0 Warning(s)`；构建产物为 `mdk_v5/objects/agv_motor.axf`。

命令行复现 Keil 构建：

```powershell
Set-Location D:\AGV\code_model\agile-drive_-ch32
.\tools\build_motor_firmware.ps1
```

该脚本调用当前 Keil 工程，并用 `fromelf` 导出 `mdk_v5/objects/agv_motor.bin`。它是课堂中
“构建产物如何产生”的标准入口，不建议学生手工拼接旧版 `printf.*` 工程。

## 3. ST-Link 与 OpenOCD 配置

当前脚本的已知配置是：ST-Link、SWD、100 kHz、AT32F456 Cortex-M4、
官方 `AT32F456_256.FLM` Flash 算法。板上 NRST 没有作为可靠前提，因此复位采用
`SYSRESETREQ`。

烧录命令：

```powershell
Set-Location D:\AGV\code_model\agile-drive_-ch32
.\tools\at32f456_stlink_program.ps1 `
  -Firmware .\mdk_v5\objects\agv_motor.bin `
  -RunAfterProgram
```

脚本会检查输入文件长度、擦写所需扇区、调用官方 Flash 算法，并执行 `verify_image`。
只有出现 `FLASH_VERIFY_OK` 与 `RESET_RUN_OK` 才可认为下载成功。

## 4. 课堂中的下载顺序

```text
Keil Build
  → 检查 0 error / 0 warning
  → 记录 bin 路径与 SHA-256
  → ST-Link / OpenOCD 烧录
  → verify_image
  → 重新上电或 SYSRESETREQ
  → 只读检查状态帧
```

第一次使用新 PC、换过发送端或丢失上位机 Reference 序号文件时，先重启电机板再测试。
这是当前协议的序号边界，不是 Keil 或 OpenOCD 的问题。

## 5. 调试纪律

- 下载或暂停正式电机固件前，先确认车辆不会运动；当前烧录脚本会先清 PWM 比较值。
- 正式电气/运动测试中，不要用断点长时间停住 `chassis_app_step()`；主循环、ADC 和看门狗时基会被破坏。
- 常规动态观察优先使用 CAN `0x181` 状态和 `0x381` 里程计。需要 SWD 读取时，只能在明确的采样实验中不停核读取。
- OpenOCD 不替代 Keil 的主工程配置。课堂中应先让学生在 Keil 看懂源码，再用脚本理解可重复烧录。
