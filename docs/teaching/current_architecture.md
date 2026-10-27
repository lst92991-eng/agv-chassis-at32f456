# 当前架构与协议（2026-10-27）

用户确认：上位机负责保护和决策，下位机负责执行与上报。只有一个生产入口与一个10ms静态任务。

```text
0x201/0x401 → chassis_protocol → chassis_app → drive_control → motor_driver
                                               ↑ 编码器速度反馈
ADC / 防撞 / BMI270 / 里程计 → 数据与有效性 → 0x181 / 0x381
```

传感器诊断与运动执行分离。IMU只参与遥测/相对位姿，不再补偿轮控目标；差速使用上位机给定v/omega。

## 运动与命令

上电静止，新的有效0x401直接起步。目标没有本地有效期，持续到被更新或显式撤销。

| 0x201命令码 | 行为 |
| ---: | --- |
| 1 ENABLE | 解除上位机DISABLE，清除旧目标，进入state=1；不根据传感器拒绝使能。ESTOP锁存时仍需先RESET |
| 2 DISABLE | 制动并清目标，禁止后续运动帧，直到上位机ENABLE或FAULT_RESET |
| 3 STOP | 制动并清目标，进入静止；新运动帧可起步。已有DISABLE或ESTOP锁存仍有效 |
| 4 ESTOP | 上位机明确要求的锁存停止，state=3、fault含0x02 |
| 5 FAULT_RESET | 解除ESTOP/DISABLE、清目标，进入静止；尝试清防撞历史和恢复CAN，不设置传感准入条件 |
| 6 RESET_ODOMETRY | 清相对里程和编码器，同步测速基准；不改变运动目标 |

state沿用0=静止、1=ENABLE/HOLD、2=执行、3=上位机ESTOP锁存。无自动FAULT状态迁移。
序号规则、STOP/ESTOP优先级和停止前报文屏障保留；重复或倒退序号不当作新命令。

## 固定线上格式

全部多字节字段为小端；标准ID、ISO FD、BRS、500k/4M，仲裁采样点75%。

- 0x201 / 8字节：cmd:u8，reserved:u8，sequence:u16，reserved:u32。
- 0x401 / 16字节：sequence:u16，TTL:u16，v:i16，omega:i16，accel:u16，decel:u16，reserved:u32。
- 0x181 / 24字节：state/fault/bumper/flags各u8；offset4命令seq:u16；offset6结果:u8；offset8时刻:u32；offset12/14左右PWM:i16；offset16/18/20母线mV和左右mA:u16；其余保留字节为0。
- 0x381 / 20字节：x:i32，y:i32，yaw:i32，v:i16，yaw_rate:i16，time:u32。

v单位mm/s，omega单位mrad/s，accel/decel单位mm/s²；0斜率表示直接更新，全零运动量为HOLD。
TTL字节兼容旧发送器但不在下位机计时。速度与斜率不再被原300/1200/75或10–500/1000门限拒绝。
PWM按硬件[-1000,1000]范围饱和；这是输出表示范围，PID积分限幅也是控制器数值处理，不会锁存故障。

## 遥测诊断

原fault字节保留：0x01防撞历史、0x02上位机ESTOP、0x08CAN链路诊断、0x10 IMU无效、0x40 ADC无有效新样本。
除上位机明确设置的ESTOP锁存外，诊断位不再自动停止或禁止运动；0x04自动超时、0x20轮控锁存不再产生。
不在下位机判断欠压、过压或过流。ADC无效时保留最近值并报告0x40，恢复后诊断自动更新。
flags bit0为IMU有效，bit1为里程计与IMU同时有效；无效数据不能由上位机当成有效零值。

电流零点仍在静止时积累，但不阻挡起步。未完成时用零偏0换算。防撞历史只上报，由FAULT_RESET尝试清除。
CPU/RTOS异常保留异常入口和停留代码，不再额外写PWM；独立看门狗不初始化、不喂狗。

## 验证边界

六组主机测试覆盖：里程计、差速/PID、ADC遥测、协议金样本、单帧直接执行/目标保持/手动停止、FreeRTOS调度。
坏长度/保留位仍拒绝，16位序号回绕保持，停止前缓存目标不会复活。
新固件的烧录与实物运动结果必须单独记录；历史带保护版本的台架结果不替代本次验收。
