# 上位机交付与使用说明

## 1. 仓库和版本

- 远端仓库：<https://github.com/lst92991-eng/agv-chassis-at32f456>
- 已烧录并完成11米架空测试的固件基准提交：`c2cbfdf90639400efd3e0b49ee33bc9afbe51824`。
- 测试固件：`agv_motor.bin`，36180字节；SHA-256：`E8E67EDAD75A1BAAFC74AD64903CF6CA505BB4A8D347F07F2A2F33989FC66C6E`。
- 仓库不存放BIN；可使用交付包内`firmware/agv_motor.bin`。协议没有版本查询字段，以烧录记录和Hash确认版本。

## 2. 下位机能力

下位机接收CAN运动命令，完成差速换算、双轮速度PID，并上报电压、电流、防撞输入、IMU和相对里程计。上位机负责路线、速度范围、保护和停止决策。

上电后，一条新的`0x401`即可运动，无需先发ENABLE；目标会保持到被更新或收到停止命令。TTL字段保留，但下位机不根据TTL自动停车。CAN断开时，上位机需要自己的独立停止途径。

## 3. CAN协议包含什么

11位标准ID，ISO CAN FD、BRS，仲裁500kbit/s，数据4Mbit/s；多字节字段为小端。**包括8字节命令在内，四种帧都使用FD+BRS。**

| 方向 | ID / 长度 | 内容 |
| --- | --- | --- |
| 上位机→下位机 | `0x401` / 16字节 | 运动序号、TTL、线速度v、角速度omega、加速/减速斜率 |
| 上位机→下位机 | `0x201` / 8字节 | ENABLE、DISABLE、STOP、ESTOP、FAULT_RESET、RESET_ODOMETRY |
| 下位机→上位机 | `0x181` / 24字节 | 状态、诊断、防撞、命令回执、左右PWM、电压、电流 |
| 下位机→上位机 | `0x381` / 20字节 | 相对x/y/yaw、线速度、角速度和时间 |

`0x401`依次为`sequence:u16, TTL:u16, v:i16, omega:i16, accel:u16, decel:u16, reserved:u32`；TTL单位ms，v单位mm/s，omega单位mrad/s，accel和decel单位mm/s²，reserved填0。`0x201`为`cmd:u8, reserved:u8, sequence:u16, reserved:u32`。

命令码：ENABLE=1、DISABLE=2、STOP=3、ESTOP=4、FAULT_RESET=5、RESET_ODOMETRY=6。

运动帧和管理命令使用**两套独立的16位序号**；STOP不清除序号。详细字段、命令行为及回执见[CAN协议详细参考](can_protocol_detail.md)。

## 4. Ubuntu通信和测试

准备：小车架空；USB-CAN FD已连接Ubuntu，接口为`can0`；CAN_H/CAN_L对应连接，总线两端各120Ω；电机板已烧录上述版本。首次测试前重新上电电机板，清除旧序号；测试时只用一个上位机发送命令。开发者需要仓库读取权限。

在Ubuntu终端执行：

```bash
git clone https://github.com/lst92991-eng/agv-chassis-at32f456.git
cd agv-chassis-at32f456
git checkout c2cbfdf90639400efd3e0b49ee33bc9afbe51824

sudo ip link set can0 down
sudo ip link set can0 type can bitrate 500000 sample-point 0.75 dbitrate 4000000 dsample-point 0.75 fd on
sudo ip link set can0 up

python3 tools/ubuntu_canfd_idle.py status
python3 tools/ubuntu_canfd_idle.py spin
```

`status`应收到`state=0`、`pwm=(0, 0)`。`spin`以50mm/s运行3秒后发送STOP；正常应看到`STOP acknowledged`及双PWM归零。此工具运行时限属于上位机，不是下位机保护。

如果上述`spin`成功、板子未重启且期间没有其他运动发送器，紧接着进行11米变速测试：

```bash
python3 tools/ubuntu_canfd_idle.py distance --reference-start 1
```

速度按50→100→150→200→250→300→250→200→150→100→50mm/s切换；成功输出`PASS ODOMETRY_DISTANCE_MM=...`和STOP回执。再次运行时要用上次最后的运动序号续接，或重新上电清历史。

## 5. 本次测试结果

已完成架空相对里程计**11.0004米**、**93.7秒**变速测试；STOP回执成功，双PWM归零，连续三帧速度为0。详见[测试记录](../hardware/execution_11m_raised_2026-10-27.md)。
