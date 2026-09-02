# AGV电机驱动板教学固件

AT32F456RCT7底盘板的最小正式控制实现。上位机负责路线、RFID、目标距离和
`v/ω`计算；本板只负责实时差速执行、速度闭环、相对里程计和本地停车保护。

## 当前快照与文档使用边界（2026-09-02）

- 本仓库范围是完整的电机/底盘控制板工程，不包含上位机、BMS、磁导航或测距板项目。
- 正式实现以`src/`、`inc/`和`mdk_v5/agv_motor.uvprojx`为准；当前使用FreeRTOS单静态控制任务，而非裸机业务循环。
- `docs/teaching/01–05`为待校准的教学草稿，部分内容早于FreeRTOS恢复；其中“没有FreeRTOS”、裸机Hello World及WFI主循环描述不能代表当前正式固件，也不能直接照搬其删文件步骤。
- `hardware/`保存历史资料及测试记录；旧网表、旧协议、GOTO/FOLLOW/磁导航记录不代表本板当前功能。`third_party/st_vl53l0x/`是未加入正式构建的历史残留，不应据此恢复TOF。
- 当前正式构建路径为Keil ARM Compiler 5。SDK虽包含GCC启动/链接文件，但项目级GCC/CMake、GCC版FreeRTOS接入和VS Code双工具链尚未完成。
- 发布前检查：Keil全量构建0错误0告警，6组主机测试通过。此前实车运动记录属于FreeRTOS恢复前版本；当前RTOS版没有据此获得实车验收结论。
- 构建和烧录脚本使用`D:/App/...`本机工具路径，其他电脑需安装对应工具、DFP并调整路径；编译器、器件包和工具程序不随源码分发。
- 厂商SDK、CMSIS、FreeRTOS与Bosch源码保留各自版权和许可，不计入自研业务行数；CMSIS适用的Apache-2.0许可正文见[许可证](licenses/CMSIS-Apache-2.0.txt)。本仓库不以统一许可证覆盖第三方声明。

面向学生的从零学习路线见[教学课程目录](docs/teaching/README.md)：先看资源和环境，
再完成无电机输出的 Hello World，最后逐层读回当前正式底盘能力。

## 阅读主线

```text
CAN FD Reference → chassis_protocol → chassis_app
→ drive_control → motor_driver → AT8236
                 ↑ encoder / BMI270 / ADC DMA / bumper
```

目录只保留三层：

```text
src/app/       FreeRTOS静态控制任务＋唯一10ms底盘状态机
src/services/  协议、差速PID、里程计和安全判断
src/drivers/   CAN、PWM、编码器、BMI270、ADC DMA、防撞和WDT
src/config/    唯一板级标定源
```

官方中断模板名称保留为`src/drivers/at32f45x_int.c`，对应现有`inc/at32f45x_int.h`；
SysTick、CPU异常和外设中断入口集中在这里。SVC/PendSV由FreeRTOS官方移植层直接实现。
`board_port.c`仅维护毫秒时基和独立WDT，不再混放中断入口。启动汇编、官方时钟文件及SDK目录保持原有边界。

## FreeRTOS调度与统计口径

- `third_party/freertos/`原样恢复历史FreeRTOS 10.4.3内核及ARMCC Cortex-M4F移植层；不计入自研代码行数。
- `inc/FreeRTOSConfig.h`使用1kHz节拍、静态分配和栈溢出检查；项目配置单列为自研支持代码。
- `app_freertos.c`只创建一个静态`ChassisCtrl`任务（优先级5、1024个栈字），每10ms调用原有`chassis_app_step()`；Idle使用静态128个栈字，只休眠、不喂狗。
- 迟到时只执行一次最新控制，按实际唤醒时刻安排下一周期，不补跑旧周期；速度环、ADC、TTL等仍使用原板级毫秒计数和原参数。
- 硬件/传感器初始化仍在调度启动前完成。创建RTOS任务后立即启动调度，期间不能插入依赖SysTick的长等待。
- 未恢复旧四任务、旧业务队列、旧通信协议；断言、栈溢出或调度器意外返回均主动制动。OS切换后的上板调度/栈水位验收需单独完成。

2026-09-02恢复检查：24个OS文件与历史版本逐一一致；6组主机测试通过、Keil全量构建0错误0告警。
自研`src`的C/H有效行由2073变为2134（+61），项目`FreeRTOSConfig.h`另计50行，合计2184行；
此口径不包含注释、空行、测试、构建脚本、FreeRTOS、Bosch或厂商SDK。没有通过移除OS压低这个数字。
本次没有烧录或进行RTOS版实车测试；控制任务实际栈水位、启动喂狗延迟仍需上板验证。

## 最小CAN FD协议

| 方向 | ID | 用途 |
| --- | ---: | --- |
| 上位机→底盘 | `0x201` | ENABLE、DISABLE、STOP、ESTOP、FAULT_RESET、RESET_ODOMETRY |
| 上位机→底盘 | `0x401` | sequence、TTL、线速度、角速度、加速度、减速度 |
| 底盘→上位机 | `0x181` | 状态、故障、防撞、电气和命令结果 |
| 底盘→上位机 | `0x381` | x、y、相对Yaw和实际速度 |

`0x401`必须周期刷新。TTL过期、CAN bus-off、防撞、IMU陈旧、堵转或电气故障都会
由本板主动制动并锁存FAULT。

## 构建和测试

```powershell
& .\tools\run_host_tests.ps1
& .\tools\build_motor_firmware.ps1
```

构建要求0错误、0告警。烧录后的人工验收顺序见
[教学架构与验收](docs/teaching/current_architecture.md)。
