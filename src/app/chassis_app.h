/** @file chassis_app.h @brief 唯一命令执行与遥测组合，由FreeRTOS周期调用。 */
#ifndef CHASSIS_APP_H
#define CHASSIS_APP_H
void chassis_app_init(void);
void chassis_app_step(void);
#endif
