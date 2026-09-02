#ifndef TEST_TASK_H
#define TEST_TASK_H

#include "FreeRTOS.h"

typedef void (*TaskFunction_t)(void *);
typedef StaticTask_t *TaskHandle_t;

TaskHandle_t xTaskCreateStatic(TaskFunction_t entry, const char *name,
    uint32_t stack_words, void *parameter, UBaseType_t priority,
    StackType_t *stack, StaticTask_t *storage);
TickType_t xTaskGetTickCount(void);
BaseType_t xTaskDelayUntil(TickType_t *last_wake, TickType_t period);
#define vTaskDelayUntil(last_wake, period) ((void)xTaskDelayUntil(last_wake, period))
void vTaskStartScheduler(void);

void vApplicationGetIdleTaskMemory(StaticTask_t **storage, StackType_t **stack,
                                  uint32_t *stack_words);
void vApplicationIdleHook(void);
void vApplicationStackOverflowHook(TaskHandle_t task, char *task_name);

#endif
