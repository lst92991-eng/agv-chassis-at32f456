/** @file app_freertos.h @brief 唯一底盘任务的静态创建、调度启动及失败收尾。 */
#ifndef APP_FREERTOS_H
#define APP_FREERTOS_H

#include <stdint.h>

/** @brief 在chassis_app_init之后创建唯一控制任务；成功返回1，不创建旧业务队列。 */
uint8_t app_freertos_init(void);
/** @brief 启动调度器；正常不返回，意外返回时主动制动并等待复位。 */
void app_freertos_start(void);
/** @brief RTOS断言/栈溢出统一入口：关中断、双轮制动并停止喂狗。 */
void app_freertos_assert_failed(const char *file, int line);

#endif
