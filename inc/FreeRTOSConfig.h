/** @file FreeRTOSConfig.h @brief 项目RTOS配置；内核源码单独位于third_party/freertos。 */
#ifndef FREERTOS_CONFIG_H
#define FREERTOS_CONFIG_H

#include <stdint.h>
#include "system_at32f45x.h"

/* 1ms内核节拍；高优先级外设ISR只投递驱动快照，不调用FreeRTOS API。 */
#define configUSE_PREEMPTION                         1
#define configUSE_TIME_SLICING                       1
#define configUSE_IDLE_HOOK                          1
#define configUSE_TICK_HOOK                          0
#define configCPU_CLOCK_HZ                           ((unsigned long)system_core_clock)
#define configTICK_RATE_HZ                           ((TickType_t)1000U)
#define configMAX_PRIORITIES                         7
#define configMINIMAL_STACK_SIZE                     ((uint16_t)128U)
#define configMAX_TASK_NAME_LEN                      16
#define configUSE_16_BIT_TICKS                       0
#define configIDLE_SHOULD_YIELD                      1
#define configUSE_PORT_OPTIMISED_TASK_SELECTION      1

/* 控制任务、Idle任务及栈全部静态分配，不引入heap或运行时分配路径。 */
#define configSUPPORT_STATIC_ALLOCATION              1
#define configSUPPORT_DYNAMIC_ALLOCATION             0
#define configUSE_MUTEXES                            1
#define configUSE_RECURSIVE_MUTEXES                  0
#define configUSE_COUNTING_SEMAPHORES                0
#define configQUEUE_REGISTRY_SIZE                    0
#define configUSE_QUEUE_SETS                         0
#define configUSE_TIMERS                             0
#define configUSE_CO_ROUTINES                        0
#define configMAX_CO_ROUTINE_PRIORITIES              1
#define configUSE_TICKLESS_IDLE                      0

#define configCHECK_FOR_STACK_OVERFLOW               2
#define configUSE_MALLOC_FAILED_HOOK                 0
#define INCLUDE_vTaskPrioritySet                     0
#define INCLUDE_uxTaskPriorityGet                   0
#define INCLUDE_vTaskDelete                          0
#define INCLUDE_vTaskSuspend                         0
#define INCLUDE_vTaskDelayUntil                      1
#define INCLUDE_vTaskDelay                           1
#define INCLUDE_xTaskGetSchedulerState               1
#define INCLUDE_xTaskGetCurrentTaskHandle            1
#define configPRIO_BITS                              4
#define configLIBRARY_LOWEST_INTERRUPT_PRIORITY      15
#define configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY 5
#define configKERNEL_INTERRUPT_PRIORITY \
  (configLIBRARY_LOWEST_INTERRUPT_PRIORITY << (8 - configPRIO_BITS))
#define configMAX_SYSCALL_INTERRUPT_PRIORITY \
  (configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY << (8 - configPRIO_BITS))

void app_freertos_assert_failed(const char *file, int line);
#define configASSERT(expression) \
  do { if((expression) == 0) app_freertos_assert_failed(__FILE__, __LINE__); } while(0)

/* SVC/PendSV必须直接绑定官方汇编移植入口；不能套普通C函数包装。 */
#define vPortSVCHandler                              SVC_Handler
#define xPortPendSVHandler                           PendSV_Handler
/* SysTick由at32f45x_int.c维护板级毫秒时基，再有条件调用xPortSysTickHandler。 */

#endif
