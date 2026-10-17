/**
 * @file freertos_hooks.c
 * @brief FreeRTOS 失败诊断钩子
 *
 * @details
 *  FreeRTOS 提供了两个"出问题就调我"的钩子，链接器只在
 *  configCHECK_FOR_STACK_OVERFLOW / configUSE_MALLOC_FAILED_HOOK 打开
 *  之后才需要它们的实现。本文件把这三个钩子集中放：
 *
 *    1. vApplicationStackOverflowHook
 *       - FreeRTOS 每次切换 task 时（method 1）或在中断里（method 2）
 *         检查栈指针是否越界，越界就调这里。栈溢出说明该任务的栈
 *         给小了——调高 app_tasks.h 里的 STACK_SIZE_*，不要加打印或
 *         分配。
 *       - 参数 pxTask 指向受害任务（method 2 才有），pcTaskName 是
 *         它的名字。method 1 第二个参数是 signed portCHAR *，是栈顶
 *         当时被破坏时"猜测"的栈指针位置。
 *       - 这里**不打印、不分配、不调用 RTOS API**——溢出时栈本身就
 *         不可信，调用任何东西都可能再次爆栈。最稳的是把全局错误标志
 *         置位 + 关中断 + 死循环，让看门狗或 IDE 调试器看到。
 *
 *    2. vApplicationMallocFailedHook
 *       - pvPortMalloc() 返回 NULL 时调，常见于 heap 耗尽或碎片化。
 *       - 同样不调 RTOS API。
 *
 *    3. vApplicationIdleHook / vApplicationTickHook
 *       - 钩子，但当前 configUSE_IDLE_HOOK / configUSE_TICK_HOOK 都
 *         是 0，留空定义避免误开。
 */

#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "stm32h7xx_hal.h"
#include "opa2000.h"   /* g_freertos_fault 标志 */

/* 失败标志已经在 opa2000.c 全局定义。钩子只写、不定义。 */

void vApplicationStackOverflowHook(TaskHandle_t pxTask, char *pcTaskName)
{
    (void)pxTask;
    g_freertos_fault = 0xDEAD0001U;
    /* 记录哪个任务挂了——pcTaskName 是 task 创建时传入的字符串。 */
    /* 不调 snprintf：溢出时栈本身可能已经不可信。 */
    __disable_irq();
    for (;;) {
        /* 看门狗（如果配了 IWDG）会在这里复位 MCU。IDE 调试器停在
         * 这一行能直接看到是哪个任务溢出。 */
    }
}

void vApplicationMallocFailedHook(void)
{
    g_freertos_fault = 0xDEAD0002U;
    __disable_irq();
    for (;;) {
    }
}

/* 下面两个钩子保留空定义：万一有人打开 configUSE_IDLE_HOOK /
 * configUSE_TICK_HOOK 也不至于链接失败。 */
void vApplicationIdleHook(void)         { }
void vApplicationTickHook(void)         { }
