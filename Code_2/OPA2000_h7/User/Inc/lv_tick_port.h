#ifndef LV_TICK_PORT_H
#define LV_TICK_PORT_H

/*============================================================================
 *  LVGL 的毫秒时基来源（lv_conf.h: LV_TICK_CUSTOM = 1）
 *
 *  为什么不直接调 lv_tick_inc()
 *  --------------------------------
 *  LVGL 7 要求 lv_tick_inc() 由 1ms 的硬件定时器中断调用。放在 GUI 任务里
 *  调是错的：这个任务优先级 3，而 Monitor 是 5、ADC 是 4、Protect 是 6，
 *  任何一个抢占它，这段时间就从 LVGL 的时基里凭空消失了——lv_tick_inc(5)
 *  补不回来，因为丢的是真实时间而不是调用次数。结果是动画、定时器、控件
 *  状态机全部越跑越慢，而且慢多少取决于调度器当天的负载。
 *
 *  为什么用 xTaskGetTickCount()
 *  ---------------------------
 *  FreeRTOS 的 configTICK_RATE_HZ 是 1000，SysTick 每毫秒加一，所以 tick
 *  计数值本身就等于毫秒数，不需要任何换算，也不会累计误差。这个值由内核
 *  的 tick 中断维护，抢占再多次也只会让它更准，不会更差。
 *
 *  模拟器
 *  ------
 *  LVGL_SIM 构建（CMakeLists.txt）没有 FreeRTOS，改用 Windows 的
 *  GetTickCount()，同样是单调递增的毫秒计数。
 *==========================================================================*/

#include <stdint.h>

#ifdef LVGL_SIM

#include <windows.h>

static inline uint32_t lv_port_tick_ms(void)
{
    return (uint32_t)GetTickCount();
}

#else

#include "FreeRTOS.h"
#include "task.h"

static inline uint32_t lv_port_tick_ms(void)
{
    return (uint32_t)xTaskGetTickCount();
}

#endif

#endif /* LV_TICK_PORT_H */
