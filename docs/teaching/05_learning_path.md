# 05：从 Hello World 逐步重建到底盘

## 教学原则

每一步只增加一个可观察能力，并保留前一步的验收方法。学生不应一开始就运行整车固件；
先能说明“这一根线、这一段代码、这一项验收”之间的关系，再进入下一层。

## 建议阶段

| 阶段 | 本课只解决的问题 | 对应正式资源 | 通过条件 |
| --- | --- | --- | --- |
| 0 | 看懂板子和资源表 | 01 文档 | 能指出主控、SWD、PWM、编码器、ADC、CAN、IMU |
| 1 | Hello World | 时钟 + SysTick | Watch 中计数器每 500 ms 增加 |
| 2 | 主循环与 WFI | `main.c`、`board_port.*` | 能解释 1 ms 中断与 10 ms 控制周期 |
| 3 | 电机输出保持安全 | `motor_driver.*` | 初始化后 Coast/Brake，不让电机自行启动 |
| 4 | 编码器读数 | `motor_encoder.*` | 手转车轮，AB 计数方向可解释 |
| 5 | 防撞条 | `bumper.*` | 四路输入与锁存位正确，先不接运动任务 |
| 6 | ADC DMA 与电气量 | `adc_dma.*`、`safety.*` | 三通道批次、零点、单位换算和新鲜度可解释 |
| 7 | BMI270 | `imu_bmi270.*` | SPI3、零偏、Z 轴角速度和新鲜度可解释 |
| 8 | CAN FD | `canfd_driver.*`、`chassis_protocol.*` | 能发/收四种固定报文，理解 sequence 与 TTL |
| 9 | 差速与速度环 | `drive_control.*` | 能从 `v/ω` 算出左右目标并说明共同限幅 |
| 10 | 顶层安全状态机 | `chassis_app.*` | 能画出 DISABLED / ENABLED / RUNNING / FAULT 转移 |

## 每阶段的固定问题

每节课结束前，学生应回答四个问题：

1. 输入来自哪个引脚、外设或上位机报文？
2. 数据在哪个模块被转换成有单位的量？
3. 哪个模块拥有状态或最终输出权限？
4. 这一阶段失败时，系统如何保持停车或安全状态？

## 建议的目录快照

教学快照应放在单独的课程工作区或 Git 分支，而不是删改当前正式工程：

```text
lesson-01-board/
lesson-02-hello/
lesson-03-timebase/
lesson-04-encoder/
lesson-05-bumper/
lesson-06-adc/
lesson-07-imu/
lesson-08-can/
lesson-09-drive-control/
lesson-10-chassis-app/
```

每个快照只新增该课需要的文件；正式工程继续作为“完成态参考实现”。这样学生可以
从最小例程向上推导，而不会误把临时教学代码当作可直接上车的生产代码。

## 下一次工作如何开始

完成环境与 Hello World 后，下一步建议从阶段 2 开始：把正式 `main.c` 的时钟、
SysTick 和 `__WFI()` 调度单独讲清楚，再引入 `board_port`。随后依照本表逐步连接
真实驱动；任何触及电机输出的阶段，都要保留 Brake、故障锁存和独立断电手段。
