/**
 * @file board_config.h
 * @brief 底盘板唯一硬件、运动和保护标定源。
 *
 * 集中保存本板几何、安装方向、控制周期、PID和电气换算/保护参数。
 * 上位机Reference必须符合这里的速度上限；更换轮子、电机、采样电路或接线后，
 * 对应参数需重新实车标定。注释给出数值含义，不代表这些参数已通过当前硬件验收。
 */
#ifndef BOARD_CONFIG_H
#define BOARD_CONFIG_H

/* 名义调度节拍，单位ms。积分、测速和超时使用实际毫秒钟，不以这些常数代替dt。 */
#define BOARD_CONTROL_PERIOD_MS             10U
#define BOARD_SPEED_PERIOD_MS               20U
/* 总状态发送周期；里程计发送以速度周期为目标，由应用安排。 */
#define BOARD_STATUS_PERIOD_MS             100U
/* ADC由主循环按此周期触发一组三通道扫描，DMA负责搬运完成结果。 */
#define BOARD_ADC_PERIOD_MS                 20U
/* 建议的Reference TTL值；当前有效期实际来自每帧ttl_ms，协议允许50～300ms。
 * 此常量目前未参与TTL判定，不能把它理解成已强制执行的固定120ms超时。 */
#define BOARD_REFERENCE_TIMEOUT_MS         120U
/* 未获得新陀螺仪样本的允许时长；应用也用此值检测主循环长时间停顿。 */
#define BOARD_IMU_STALE_MS                  50U

/* 差速底盘几何和车辆坐标：X向前、Y向左、左转为正。 */
/* 轮周长用于计数/RPM与毫米互换；轮距用于左右轮速差与车体角速度互换。 */
#define BOARD_WHEEL_CIRCUMFERENCE_MM    267.035f
#define BOARD_TRACK_WIDTH_MM            500.0f
/* 输出轴每圈累计计数，包含AB四边沿解码及减速比，不能误当单相脉冲数。 */
#define BOARD_ENCODER_COUNTS_PER_REV   2640.0f
/* Reference车体速度硬上限：mm/s、mrad/s；轮速上限则作用于差速换算后的RPM。 */
#define BOARD_MAX_LINEAR_MM_S             300.0f
#define BOARD_MAX_ANGULAR_MRAD_S         1200.0f
#define BOARD_MAX_WHEEL_RPM                75.0f
/* 角速度补偿ω'=ω+gain*(ω-ω_imu)，gain无量纲；不是绝对航向角闭环。 */
#define BOARD_YAW_RATE_GAIN                 0.35f

/* 速度环：error为RPM、dt为秒、输出为PWM千分比。
 * Kp单位PWM/RPM，Ki为PWM/(RPM·s)，Kd为PWM·s/RPM；Ki必须大于0，代码用其换算积分限幅。
 * D项路径存在；当前Kd=0关闭其贡献，没有删除微分算法。 */
#define BOARD_PID_KP                         4.0f
#define BOARD_PID_KI                         8.0f
#define BOARD_PID_KD                         0.0f
/* 仅限制PID修正量，基础前馈另加；最终总PWM再限制到±1000。 */
#define BOARD_PID_LIMIT                    250.0f
/* 前馈=目标方向*STATIC_PWM+目标RPM*PER_RPM，前者补静摩擦，后者补转速对应输出。 */
#define BOARD_FF_STATIC_PWM                100.0f
#define BOARD_FF_PER_RPM                     9.4f
/* 编码器速度低通filtered+=alpha*(raw-filtered)，alpha无量纲且按名义周期调参。 */
#define BOARD_RPM_FILTER_ALPHA               0.20f
/* 原始RPM异常门，不是允许的目标轮速；超过此值会锁存轮控故障。 */
#define BOARD_RAW_RPM_LIMIT                120.0f
/* 目标幅值至少8RPM，但沿目标方向的实测不超过1RPM，持续10×20=200ms即报堵转。
 * 运行时累积实际dt；TICKS仅用于表达时间门限，不按调用次数计故障时长。 */
#define BOARD_STALL_TARGET_RPM               8.0f
#define BOARD_STALL_MEASURED_RPM             1.0f
#define BOARD_STALL_TICKS                   10U

/* TARGET_SIGN把车体前进为正的轮速转成电机驱动方向；FEEDBACK_SIGN把原始编码器
 * 转成同一个电机控制坐标。两组符号含义不同，改接线后需分别验收，不能一起盲目取反。 */
#define BOARD_LEFT_TARGET_SIGN              -1.0f
#define BOARD_RIGHT_TARGET_SIGN              1.0f
#define BOARD_LEFT_FEEDBACK_SIGN             1.0f
#define BOARD_RIGHT_FEEDBACK_SIGN           -1.0f

/* 电气监督：PA3母线，PA4/PA5分别为左右INA180。 */
/* 12位ADC满量程；当前换算使用下面已经折算到每计数的标定系数。 */
#define BOARD_ADC_FULL_SCALE               4095U
/* 母线电压=raw*UV_PER_COUNT/1000+OFFSET_MV，单位最终为mV。 */
#define BOARD_BUS_UV_PER_COUNT           8850UL
#define BOARD_BUS_OFFSET_MV                  0L
/* INA180A3增益100、分流10mΩ：3.3V/4095/(100*0.01Ω)≈806µA/码。
 * 电流=(raw-静止零点)*UA_PER_COUNT/1000，单位mA；与历史实车版本一致。 */
#define BOARD_CURRENT_UA_PER_COUNT        806UL
/* 欠压低于18V、过压高于25.8V即锁存；显式复位需回到19.2～25.2V，形成恢复滞回。 */
#define BOARD_BUS_UV_TRIP_MV            18000UL
#define BOARD_BUS_UV_RECOVER_MV         19200UL
#define BOARD_BUS_OV_TRIP_MV            25800UL
#define BOARD_BUS_OV_RECOVER_MV         25200UL
/* 任一侧达到800mA持续5×20=100ms锁存，实际累计新样本间隔；不是电流闭环。 */
#define BOARD_PEAK_CURRENT_MA             800U
#define BOARD_PEAK_CURRENT_TICKS            5U
/* 连续60ms没有接受新ADC序号即故障，重复读取缓存不算新数据。 */
#define BOARD_ADC_STALE_MS                  60U
/* 需连续20组禁止驱动且静止的合格样本建立零点；任一侧raw>19则重新累计。 */
#define BOARD_ADC_ZERO_SAMPLES              20U
#define BOARD_ADC_ZERO_MAX_RAW              19U

/* 四种11位标准CAN FD ID：接收控制命令/速度Reference，发送状态/里程计。
 * 具体小端字段在chassis_protocol中显式编解码，不能直接发送C结构体内存。 */
#define BOARD_CAN_ID_CONTROL             0x201U
#define BOARD_CAN_ID_STATUS              0x181U
#define BOARD_CAN_ID_ODOMETRY            0x381U
#define BOARD_CAN_ID_REFERENCE           0x401U
/* Reference固定16字节，包含sequence、TTL、v/ω、加减速度及保留字段。 */
#define BOARD_CAN_REFERENCE_SIZE           16U

#endif
