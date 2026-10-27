/**
 * @file app_freertos.c
 * @brief FreeRTOS只提供调度载体，底盘业务仍由同一个chassis_app_step顺序执行。
 *
 * 一条静态控制任务加内核Idle，不拆分CAN/电气/轮控的状态所有权。
 * 任务只负责周期唤醒，底盘模块负责实际dt、命令执行和遥测。
 */
#include "app_freertos.h"

#include <stddef.h>
#include "FreeRTOS.h"
#include "task.h"
#include "at32f45x.h"
#include "board_config.h"
#include "chassis_app.h"

#define APP_CONTROL_STACK_WORDS 1024U
#define APP_CONTROL_PRIORITY      5U

static StaticTask_t app_control_storage;
static StackType_t app_control_stack[APP_CONTROL_STACK_WORDS];
static StaticTask_t app_idle_storage;
static StackType_t app_idle_stack[configMINIMAL_STACK_SIZE];

/** @brief 每次实际唤醒只执行一个最新周期；延误不补跑过时的控制周期。 */
static void app_control_task(void *parameter)
{
  TickType_t last_wake = xTaskGetTickCount();
  (void)parameter;
  for(;;)
  {
    vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(BOARD_CONTROL_PERIOD_MS));
    /* 以实际开始时刻重定位，保持原先“迟到执行一次、下次从本次开始计时”的语义。 */
    last_wake = xTaskGetTickCount();
    chassis_app_step();
  }
}

/** @brief 静态创建控制任务；外设已由main调用chassis_app_init完成。 */
uint8_t app_freertos_init(void)
{
  return xTaskCreateStatic(app_control_task, "ChassisCtrl", APP_CONTROL_STACK_WORDS,
      NULL, APP_CONTROL_PRIORITY, app_control_stack, &app_control_storage) != NULL ? 1U : 0U;
}

/** @brief 调度器启动后不应返回；若返回则进入内核异常停留。 */
void app_freertos_start(void)
{
  vTaskStartScheduler();
  app_freertos_assert_failed(__FILE__, __LINE__);
}

/** @brief 为内核Idle任务提供固定TCB及栈，不使用动态内存。 */
void vApplicationGetIdleTaskMemory(StaticTask_t **task_buffer,
                                  StackType_t **stack_buffer, uint32_t *stack_size)
{
  *task_buffer = &app_idle_storage;
  *stack_buffer = app_idle_stack;
  *stack_size = configMINIMAL_STACK_SIZE;
}

/** @brief 空闲时只等待中断，不执行底盘业务，不操作电机。 */
void vApplicationIdleHook(void) { __WFI(); }

/** @brief 栈溢出不能继续运行，复用RTOS断言停留，不改写PWM。 */
void vApplicationStackOverflowHook(TaskHandle_t task, char *task_name)
{
  (void)task;
  (void)task_name;
  app_freertos_assert_failed(NULL, 0);
}

/** @brief 内核异常只关中断并停留，不执行运动策略或改写PWM。 */
void app_freertos_assert_failed(const char *file, int line)
{
  (void)file;
  (void)line;
  __disable_irq();
  __DSB();
  __ISB();
  for(;;) {}
}
