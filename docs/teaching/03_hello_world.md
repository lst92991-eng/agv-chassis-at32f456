# 03：最小 Hello World

## 1. 这一课的目标

在不初始化电机、CAN、ADC、编码器或看门狗的前提下，证明：

1. 时钟已从 12 MHz 外部晶振切到 192 MHz；
2. SysTick 每 1 ms 工作；
3. Keil 能下载、运行并观察变量。

这里的 “Hello World” 是 **调试器可观察的心跳计数器**。当前正式板没有经新版原理图
确认的教学 LED 或调试 UART 输出，因此不先假定某个 LED/串口可以使用。

## 2. Hello World 源码

在单独的教学工程中使用下面的 `hello_main.c`。不要直接替换正式工程的
`src/app/main.c`。

```c
#include <stdint.h>

#include "at32f45x.h"
#include "at32f45x_clock.h"

static volatile uint32_t hello_ms;
static volatile uint32_t hello_counter;

void SysTick_Handler(void)
{
    hello_ms++;
}

int main(void)
{
    uint32_t last_ms = 0U;

    system_clock_config();
    nvic_priority_group_config(NVIC_PRIORITY_GROUP_4);
    (void)SysTick_Config(system_core_clock / 1000U);

    for(;;)
    {
        if((uint32_t)(hello_ms - last_ms) >= 500U)
        {
            last_ms = hello_ms;
            hello_counter++;
        }
        __WFI();
    }
}
```

## 3. 最小工程组成

从当前 Keil 工程复制器件、启动文件、系统文件和厂商驱动配置；把正式 `main.c` 与
`chassis_app.c` 从教学 Target 排除，加入 `hello_main.c` 与
`src/drivers/at32f45x_clock.c`。需要保留的底层文件至少包括：

```text
startup_at32f45x.s
system_at32f45x.c
at32f45x_clock.c
at32f45x_crm.c / at32f45x_flash.c / at32f45x_pwc.c / at32f45x_misc.c
hello_main.c
```

这样做是为了让第一课只讲时钟、向量表、SysTick 和调试，不把电机安全状态机混进来。

## 4. Keil 验收

1. Build 后必须为 0 error、0 warning。
2. 连接 ST-Link，进入 Debug。
3. 在 Watch 中添加 `hello_ms`、`hello_counter`。
4. Run 后，`hello_ms` 连续增加，`hello_counter` 约每 500 ms 增加 1。
5. Stop 后查看 Call Stack，确认程序位于 `main` 的循环或 `SysTick_Handler`。

这比串口打印更适合第一课：它不依赖未确认的外设引脚，也不会阻塞控制程序。

## 5. 下一版 Hello World

当新版原理图确认 LED 或 UART 资源后，再增加二选一扩展：

- LED：在 500 ms 节拍中翻转一个已确认的 GPIO；
- UART：在有界发送接口中输出 `hello_counter`。

两种扩展都必须独立于正式电机 Target。学生完成该例程后，才进入正式工程的
`board_port_time_init()` 与 10 ms 底盘调度。
