# AGV 下位机执行与遥测固件

AT32F456RCT7，192MHz，FreeRTOS单静态控制任务。按2026-10-27确认的职责边界，
本板只解析CAN命令、执行差速和双轮速度PID、采集并上报数据；运动保护由上位机负责。

交付给上位机开发者时，先读[接入与交付说明](docs/upper_controller_integration.md)。

## 当前行为

- 上电静止；一条合法的新0x401直接执行，不需要先发ENABLE，也不等待电气零点或传感器准入。
- 目标保持到新运动帧或停止命令。0x401的TTL字段位置保留，下位机不执行TTL超时停车。
- 欠/过压、过流、ADC/IMU陈旧、防撞、通信故障、轮控异常不再自动清目标或制动。
- 不启动独立看门狗，CPU/RTOS异常处理不再自动写PWM。
- 差速、方向标定、前馈+PID、共同加减速斜率保留；速度边界由上位机负责。
- 电压、电流、碰撞历史、IMU与里程计继续上报，采集诊断不参与运动决策。

## CAN 字节布局

ISO CAN FD，标准ID，BRS，500k/4M。仲裁采样点使用75%。

| 方向 | ID | 字节数 | 内容 |
| --- | --- | ---: | --- |
| 上位机→下位机 | 0x201 | 8 | 命令码、保留位、sequence |
| 上位机→下位机 | 0x401 | 16 | sequence、TTL、v、omega、accel、decel、保留位 |
| 下位机→上位机 | 0x181 | 24 | 状态、诊断、防撞、命令结果、PWM、电压、电流 |
| 下位机→上位机 | 0x381 | 20 | 相对x/y/yaw、速度、板端时刻 |

ID、帧长度、字段位置、字节序、命令码和遥测金样本保持不变；执行策略按上述新职责变化。
完整字段与命令语义见[当前架构与协议](docs/teaching/current_architecture.md)。

## 工程与验证

```powershell
.\tools\run_host_tests.ps1
.\tools\build_motor_firmware.ps1 -Rebuild
```

生产路径：`src/app/main.c → app_freertos → chassis_app → drive_control → motor_driver`。
`power_telemetry`替代原`safety`服务，只做ADC换算与有效性上报。

Ubuntu操作见[最小步骤](tools/UBUNTU_IDLE_TEST.md)。
软件构建和主机测试不等于新版本已烧录或通过实物运动验收。
`hardware/`是历史证据；教学01–05中尚未校准的旧保护、TTL和状态机描述，以当前架构文档为准。

2026-10-27实物验证：新固件已通过DAPLink烧录与校验，完成架空11.0004米、50→300→50mm/s变速运行，
93.7秒后上位机STOP成功，双PWM与连续三份速度反馈归零。见[本次记录](hardware/execution_11m_raised_2026-10-27.md)。
