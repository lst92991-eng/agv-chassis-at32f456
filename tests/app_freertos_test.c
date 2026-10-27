#include <setjmp.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "app_freertos.h"
#include "task.h"
#include "at32f45x.h"
#include "board_config.h"
#include "board_port.h"
#include "chassis_app.h"
#include "motor_driver.h"

/* 仅替换RTOS和CPU边界；直接链接生产包装层，不增加生产测试分支。 */
static jmp_buf test_exit;
static TaskFunction_t created_task;
static void *created_parameter;
static StackType_t *created_stack;
static StaticTask_t *created_storage;
static uint32_t created_stack_words;
static UBaseType_t created_priority;
static TickType_t test_tick, releases[4];
static unsigned int create_calls, step_calls, delay_calls, scheduler_calls;
static unsigned int idle_calls, watchdog_calls, irq_calls, brake_calls;
static uint8_t create_ok;

static void check(int ok, const char *message)
{ if(!ok) { fprintf(stderr, "%s\n", message); exit(1); } }

TaskHandle_t xTaskCreateStatic(TaskFunction_t entry, const char *name,
    uint32_t stack_words, void *parameter, UBaseType_t priority,
    StackType_t *stack, StaticTask_t *storage)
{
  check(name != NULL && name[0] != '\0', "control task has no name");
  create_calls++;
  created_task = entry;
  created_parameter = parameter;
  created_stack_words = stack_words;
  created_priority = priority;
  created_stack = stack;
  created_storage = storage;
  return create_ok != 0U ? storage : NULL;
}

TickType_t xTaskGetTickCount(void) { return test_tick; }
BaseType_t xTaskDelayUntil(TickType_t *last_wake, TickType_t period)
{
  TickType_t wake;
  BaseType_t should_delay;
  check(last_wake != NULL && period == pdMS_TO_TICKS(BOARD_CONTROL_PERIOD_MS),
        "control task did not request its configured 10ms period");
  delay_calls++;
  wake = *last_wake + period;
  /* 与FreeRTOS的无符号回绕判定相同；已错过的唤醒点不前移模拟时钟。 */
  should_delay = test_tick < *last_wake ?
      (wake < *last_wake && wake > test_tick) :
      (wake < *last_wake || wake > test_tick);
  *last_wake = wake;
  if(should_delay != pdFALSE) test_tick = wake;
  return should_delay;
}

void chassis_app_step(void)
{
  check(step_calls < 4U, "control task ran extra catch-up steps");
  releases[step_calls++] = test_tick;
  /* 第二次业务执行耗时25ms，第三次应只迟到执行一次，再从实际时间等待。 */
  if(step_calls == 2U) test_tick += 25U;
  if(step_calls == 4U) longjmp(test_exit, 1);
}

void vTaskStartScheduler(void) { scheduler_calls++; }
void board_port_watchdog_feed(void) { watchdog_calls++; }
void __WFI(void) { idle_calls++; }
void __disable_irq(void) { irq_calls++; }
void __DSB(void) {}
void __ISB(void) { longjmp(test_exit, 2); }
void motor_driver_stop_all(motor_stop_mode_t mode)
{ (void)mode; brake_calls++; }

static void reset_fixture(void)
{
  created_task = NULL;
  created_parameter = NULL;
  created_stack = NULL;
  created_storage = NULL;
  created_stack_words = created_priority = 0U;
  test_tick = 0U;
  memset(releases, 0, sizeof(releases));
  create_calls = step_calls = delay_calls = scheduler_calls = 0U;
  idle_calls = watchdog_calls = irq_calls = brake_calls = 0U;
  create_ok = 1U;
}

static void test_static_creation(void)
{
  StaticTask_t *idle_storage, *again_storage;
  StackType_t *idle_stack, *again_stack;
  uint32_t idle_words, again_words;
  reset_fixture();
  check(configSUPPORT_STATIC_ALLOCATION == 1 && configSUPPORT_DYNAMIC_ALLOCATION == 0,
        "RTOS configuration is not static-only");
  check(configTICK_RATE_HZ == 1000U && configCHECK_FOR_STACK_OVERFLOW == 2,
        "RTOS tick or stack-overflow protection changed");
  check(app_freertos_init() == 1U && create_calls == 1U, "expected one static control task");
  check(created_task != NULL && created_parameter == NULL &&
        created_stack != NULL && created_storage != NULL,
        "control task lacks entry or static storage");
  check(created_stack_words == 1024U && created_priority == 5U,
        "control task stack or priority changed");
  check(step_calls == 0U && scheduler_calls == 0U,
        "task creation executed business logic or started the scheduler");
  vApplicationGetIdleTaskMemory(&idle_storage, &idle_stack, &idle_words);
  vApplicationGetIdleTaskMemory(&again_storage, &again_stack, &again_words);
  check(idle_storage != NULL && idle_stack != NULL && idle_words == configMINIMAL_STACK_SIZE,
        "Idle static storage is missing or incorrectly sized");
  check(idle_storage == again_storage && idle_stack == again_stack && idle_words == again_words,
        "Idle hook did not retain static storage");
  check(idle_storage != created_storage && idle_stack != created_stack,
        "Idle and control tasks share stack or TCB storage");
  reset_fixture();
  create_ok = 0U;
  check(app_freertos_init() == 0U && create_calls == 1U,
        "static task creation failure was reported as success");
  check(step_calls == 0U && scheduler_calls == 0U,
        "failed task creation ran control or scheduler");
}

static void test_period_and_wrap(void)
{
  static const struct {
    TickType_t start;
    TickType_t expected[4];
  } cases[] = {
    {0U, {10U, 20U, 45U, 55U}},
    {UINT32_MAX - 14U, {UINT32_MAX - 4U, 5U, 30U, 40U}}
  };
  unsigned int index;
  for(index = 0U; index < sizeof(cases) / sizeof(cases[0]); index++)
  {
    reset_fixture();
    test_tick = cases[index].start;
    check(app_freertos_init() != 0U, "control task creation failed");
    if(setjmp(test_exit) == 0)
    {
      created_task(created_parameter);
      check(0, "control task unexpectedly returned");
    }
    check(step_calls == 4U && delay_calls == 4U &&
          memcmp(releases, cases[index].expected, sizeof(releases)) == 0,
          "control timing caught up after overrun or failed at tick wrap");
    check(watchdog_calls == 0U, "RTOS wrapper fed the watchdog outside chassis_app");
  }
}

static void test_idle_does_not_feed_watchdog(void)
{
  reset_fixture();
  vApplicationIdleHook();
  vApplicationIdleHook();
  check(idle_calls == 2U && watchdog_calls == 0U && step_calls == 0U,
        "Idle must only wait for interrupts, never run control or feed watchdog");
}

static void test_fault_hooks_only_halt(void)
{
  unsigned int scenario;
  for(scenario = 0U; scenario < 3U; scenario++)
  {
    reset_fixture();
    if(setjmp(test_exit) == 0)
    {
      if(scenario == 0U) app_freertos_assert_failed(__FILE__, __LINE__);
      else if(scenario == 1U) vApplicationStackOverflowHook(NULL, NULL);
      else app_freertos_start();
      check(0, "RTOS failure path unexpectedly returned");
    }
    check(irq_calls == 1U && brake_calls == 0U && watchdog_calls == 0U,
          "RTOS exception must not change PWM or start/reset a watchdog");
    check(scheduler_calls == (scenario == 2U ? 1U : 0U),
          "scheduler-return failure path was not exercised exactly once");
  }
}

int main(void)
{
  test_static_creation();
  test_period_and_wrap();
  test_idle_does_not_feed_watchdog();
  test_fault_hooks_only_halt();
  puts("APP_FREERTOS_TEST_PASS");
  return 0;
}
