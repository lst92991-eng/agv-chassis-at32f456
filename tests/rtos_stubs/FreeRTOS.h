#ifndef TEST_FREERTOS_H
#define TEST_FREERTOS_H

#include <stdint.h>

typedef uint32_t TickType_t;
typedef uint32_t StackType_t;
typedef uint32_t UBaseType_t;
typedef int32_t BaseType_t;
typedef struct { uint32_t unused; } StaticTask_t;

/* 复用生产配置，桩不维护第二套周期、栈大小或内存分配开关。 */
#include "FreeRTOSConfig.h"

#define pdTRUE  1
#define pdFALSE 0
#define pdMS_TO_TICKS(ms) ((TickType_t)(((TickType_t)(ms) * configTICK_RATE_HZ) / 1000U))

#endif
