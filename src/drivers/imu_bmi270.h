/** @file imu_bmi270.h @brief Bosch BMI270官方库的AT32 SPI3薄适配。 */
#ifndef IMU_BMI270_H
#define IMU_BMI270_H

#include <stdint.h>

/**
 * @brief 启动SPI3、官方BMI270配置和64样本零偏校准；成功返回1。
 * @note 编码器须已启动、车辆须保持静止。初始化包含阻塞延时，仅启动时调用。
 */
uint8_t imu_bmi270_init(void);
/**
 * @brief 主循环读取最新样本；dt_s是本次实际经过秒数，正常约10ms。
 * @return 1表示样本可用，0表示通信或新鲜度失败；故障锁存由应用层负责。
 */
uint8_t imu_bmi270_update(float dt_s);
/** @brief 获取扣零偏并低通后的Z轴角速度(°/s)，不代表绝对航向。 */
float imu_bmi270_gyro_z_dps(void);
/** @brief 样本年龄小于50ms且初始化成功返回1；读取角速度前应检查此状态。 */
uint8_t imu_bmi270_fresh(void);

#endif
