/**
 * @file app_tasks.h
 * @brief FreeRTOS Tasks Header
 */

#ifndef APP_TASKS_H
#define APP_TASKS_H

#ifdef __cplusplus
extern "C" {
#endif

#include "FreeRTOS.h"
#include "task.h"

/*============================================================================
 * Task Priorities (lower number = lower priority)
 *============================================================================*/

#define PRIORITY_GUI_TASK        3
#define PRIORITY_ADC_TASK        4
#define PRIORITY_COMM_TASK       3
#define PRIORITY_MONITOR_TASK    5
#define PRIORITY_PROTECT_TASK    6

/*============================================================================
 * Task Stack Sizes (in words)
 *============================================================================*/

#define STACK_SIZE_GUI_TASK      2048
#define STACK_SIZE_ADC_TASK      512
#define STACK_SIZE_COMM_TASK     1024
#define STACK_SIZE_MONITOR_TASK  512
#define STACK_SIZE_PROTECT_TASK  512

/*============================================================================
 * Function Prototypes
 *============================================================================*/

/**
 * @brief Create all FreeRTOS tasks
 */
void app_tasks_create(void);

/**
 * @brief GUI Task
 * @param argument Task argument
 */
void app_gui_task(void *argument);

/**
 * @brief ADC Processing Task
 * @param argument Task argument
 */
void app_adc_task(void *argument);

/**
 * @brief Communication Task
 * @param argument Task argument
 */
void app_comm_task(void *argument);

/**
 * @brief Monitor Task (fan control, etc)
 * @param argument Task argument
 */
void app_monitor_task(void *argument);

/**
 * @brief Protection Task
 * @param argument Task argument
 */
void app_protect_task(void *argument);

#ifdef __cplusplus
}
#endif

#endif /* APP_TASKS_H */