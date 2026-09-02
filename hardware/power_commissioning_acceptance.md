# 电机板电气监督首次标定与测试

> 当前生产Target按用户要求启用开发动态验收配置：6串母线18.0V跳闸/19.2V恢复、
> 25.2V恢复/25.8V跳闸，单电机0.8A持续100ms跳闸。它允许受监控的动态开发验收，
> 但不替代最终产品的电流、电压和热证据。

## 1. 准备

- 小车架空，左右轮无接触，周围无人；
- 准备独立断电手段、万用表和SWD调试器；
- 确认烧录BIN的SHA256与交付报告一致；
- 电机板母线、电机和编码器接线保持正式整车状态。

## 2. 静止采集

当前精简固件不保留调试UART和独立电气诊断PDO。常规动态记录优先读取CAN 0x181中的
电压、电流、故障和内部PWM。核对原始采样时，用与固件匹配的AXF/map定位以下真实符号，
静止采集至少10秒；运行中若需SWD采样，应不停核读取，不能用断点暂停控制循环：

```text
adc_dma_raw[0..2] = bus,left,right
adc_dma_sequence, adc_dma_ready, adc_dma_busy
safety_status = bus_mv,left_current_ma,right_current_ma,fault,ready
safety_left_zero, safety_right_zero, safety_zero_count
safety_last_sample_ms, safety_last_sequence, safety_peak_ms
```

`adc_dma_raw[0]`对应PA3母线，`adc_dma_raw[1]`对应PA4左电流，`adc_dma_raw[2]`对应PA5右电流。
上位机状态日志字段为`STATUS,host_ms,state,fault,bus_mv,left_ma,right_ma,left_pwm,right_pwm`。
若当前SWD工具不能导出连续记录，动态开发仍可依靠现有本地保护继续，但不得把本轮
结果表述为最终电气标定完成。

通过条件：

- ADC采样阶段需20组连续合格新样本，名义20ms/组，约400ms；此前还有IMU启动校准；
- 只使用DMA已就绪、序号连续推进且样本年龄小于60ms的记录；ADC超时周期不可用于拟合；
- 左右零点均不超过19 ADC码，静止后`current_ma`接近0；
- 万用表母线电压与`bus_mv`方向一致，记录两者用于拟合增益/偏置；
- 静止零点完成后`safety_status.fault=0`且`ready=1`；CAN 0x181应为DISABLED、fault=0、flags=3；
- 不得出现0x40电气故障。该位合并ADC超时、欠压、过压、持续过流，必要时结合原始采样区分来源。

当前电流系数为806µA/ADC码：10mΩ分流、INA180A3增益100、名义3.3V/12位ADC。
这恢复了历史正常实现；旧精简版8060的10倍错误已修复。每20ms单次扫描不是PWM同步
平均电流测量，独立仪表与实际VDDA校验仍属于后续标定内容。

若零点窗口无法完成、原始值抖动明显或母线换算错误，停止后续测试，先查模拟地、
分流器、INA180、VDDA和分压网络。

## 3. 批准输入

下面是最终电气标定应补齐的证据；不等同于当前固件新增的ENABLE配置开关：

- 10秒完整POWER日志；
- 同时刻万用表母线电压；
- 电机型号、额定/启动/堵转电流或独立台架测量；
- 允许的持续时间、线束/连接器和驱动器热条件。

由这些独立量计算电流增益、母线增益/偏置、UV/OV迟滞和峰值持续时间。批准配置
必须重新构建并记录Hash；不能直接沿用临时3A/33V硬件上限作为电机保护结论。

## 4. 批准后的架空顺序

1. STOP、ESTOP、Reference TTL到期和旧Reference接收屏障；
2. 左右轮正向、反向及编码器符号；
3. 最高速度制动后，连续3个原始零速样本前不得反向；
4. 单轴反馈断开/反向时，两轴同周期停车；
5. 单轮受阻、电流故障和显式FAULT_RESET；清除后保持DISABLED，不自动恢复运动；
6. 再按已定义的上位机场景验证落地运动；磁导航业务不属于电机板驱动层。

每一门失败都回到STOP并保留日志，不继续下一门。
