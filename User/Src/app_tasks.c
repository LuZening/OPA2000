/**
 * @file app_tasks.c
 * @brief FreeRTOS Tasks Implementation
 */

#include "app_tasks.h"
#include "gui.h"
#include "user_adc.h"
#include "comm.h"
#include "power_control.h"
#include "touch.h"
#include "protection.h"
#include "freq_detect.h"
#include "config.h"
#include "pca9539.h"

/*============================================================================
 * Private Variables
 *============================================================================*/

/* Task handles */
static TaskHandle_t gui_task_handle = NULL;
static TaskHandle_t adc_task_handle = NULL;
static TaskHandle_t comm_task_handle = NULL;
static TaskHandle_t monitor_task_handle = NULL;
static TaskHandle_t protect_task_handle = NULL;

/*============================================================================
 * Public Functions
 *============================================================================*/

/**
 * @brief Create all tasks
 */
void app_tasks_create(void)
{
    BaseType_t result;

    /* GUI Task */
    result = xTaskCreate(
        app_gui_task,
        "GUI Task",
        STACK_SIZE_GUI_TASK,
        NULL,
        PRIORITY_GUI_TASK,
        &gui_task_handle
    );
    if (result != pdPASS) {
        /* Error handling */
    }

    /* ADC Task */
    result = xTaskCreate(
        app_adc_task,
        "ADC Task",
        STACK_SIZE_ADC_TASK,
        NULL,
        PRIORITY_ADC_TASK,
        &adc_task_handle
    );
    if (result != pdPASS) {
        /* Error handling */
    }

    /* Communication Task */
    result = xTaskCreate(
        app_comm_task,
        "Comm Task",
        STACK_SIZE_COMM_TASK,
        NULL,
        PRIORITY_COMM_TASK,
        &comm_task_handle
    );
    if (result != pdPASS) {
        /* Error handling */
    }

    /* Monitor Task */
    result = xTaskCreate(
        app_monitor_task,
        "Monitor Task",
        STACK_SIZE_MONITOR_TASK,
        NULL,
        PRIORITY_MONITOR_TASK,
        &monitor_task_handle
    );
    if (result != pdPASS) {
        /* Error handling */
    }

    /* Protection Task */
    result = xTaskCreate(
        app_protect_task,
        "Protect Task",
        STACK_SIZE_PROTECT_TASK,
        NULL,
        PRIORITY_PROTECT_TASK,
        &protect_task_handle
    );
    if (result != pdPASS) {
        /* Error handling */
    }
}

/**
 * @brief GUI Task - handles LVGL and user input
 */
void app_gui_task(void *argument)
{
    TickType_t last_wake_time;
    TouchPointType touch_point;

    (void)argument;

    last_wake_time = xTaskGetTickCount();

    /* Initialize GUI */
    gui_init();

    /* Start GUI */
    gui_start();

    /* Initialize touch */
    touch_init();

    while (1) {
        /* Process LVGL tick */
        gui_tick();

        /* Process touch */
        touch_process();
        if (touch_get_position(&touch_point) && touch_point.pressed) {
            /* Input handled by LVGL indev */
        }

        /* Process calibration input during calibration */
        gui_process_calibration();
        gui_process_coupler_cal();
        gui_update_sensor_inspect();

        /* Update display values */
        gui_update_display();

        /* Check alert status - if cleared from GUI */
        if (!protection_is_active() && gui_is_alert_active()) {
            gui_dismiss_alert();
        }

        /* 10ms tick */
        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(10));
    }
}

/**
 * @brief ADC Task - processes ADC data
 */
void app_adc_task(void *argument)
{
    TickType_t last_wake_time;

    (void)argument;

    last_wake_time = xTaskGetTickCount();

    /* Initialize ADC */
    adc_init();
    adc_start();

    while (1) {
        /* Process ADC data */
        adc_process_data();

        /* 50ms - 20Hz update rate */
        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(50));
    }
}

/**
 * @brief Communication Task - handles USB and serial communication
 */
void app_comm_task(void *argument)
{
    TickType_t last_wake_time;

    (void)argument;

    last_wake_time = xTaskGetTickCount();

    /* Initialize communication */
    comm_init();
    comm_start();

    while (1) {
        /* Process received commands */
        comm_process_rx();

        /* Check for alerts from communication (e.g., "ALERT OFF") */
        if (g_alert_flag) {
            g_alert_flag = false;
            /* Alert was set, GUI will handle display */
        }

        /* 10ms tick */
        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(10));
    }
}

/**
 * @brief Monitor Task - fan control, temperature monitoring
 */
void app_monitor_task(void *argument)
{
    TickType_t last_wake_time;
    static bool ptt_last_handled = false;  /* Track last handled PTT state */

    (void)argument;

    last_wake_time = xTaskGetTickCount();

    /* Initialize power control */
    power_init();

    /* Initialize PCA9539 button expander */
    pca9539_init();

    /* Initialize PTT state tracking based on current GPIO state */
    ptt_last_handled = power_get_ptt_state();

    while (1) {
        /* Check for PTT edge from ISR */
        int ptt_isr_flag = power_get_ptt_flag();
        bool ptt_current;
        bool ptt_just_changed = false;

        if (ptt_isr_flag != -1) {
            /* ISR has set a new flag - use it */
            ptt_current = (ptt_isr_flag == 1);

            /* Check if this is a new edge (different from last handled state) */
            if (ptt_current != ptt_last_handled) {
                ptt_just_changed = true;
            }

            /* Clear the ISR flag after reading */
            power_clear_ptt_flag();
        } else {
            /* No ISR flag - check if we missed an edge by reading GPIO directly */
            /* This is the fallback for edge cases */
            ptt_current = power_get_ptt_state();

            /* If current state differs from last handled, we may have missed an edge */
            if (ptt_current != ptt_last_handled) {
                ptt_just_changed = true;
            }
        }

        /* Process PTT state change only on edge */
        if (ptt_just_changed) {
            if (ptt_current) {
                /* PTT pressed */
                if (g_power_state == POWER_STATE_STANDBY) {
                    power_tx_enable();
                }
            } else {
                /* PTT released */
                if (g_power_state == POWER_STATE_TX) {
                    power_tx_disable();
                }
            }

            /* Update last handled state to prevent duplicate triggers */
            ptt_last_handled = ptt_current;
        }

        /* Update fan based on temperature */
        power_update_fan(g_adc_data.ntc_temp);

        /* Update band from frequency detector */
        if (g_power_state == POWER_STATE_TX) {
            power_update_band(freq_detect_get_frequency());
        }

        /* Process PCA9539 buttons (debounced) */
        pca9539_process();

        /* Check for button presses */
        Pca9539ButtonState buttons;
        if (pca9539_get_buttons(&buttons)) {
            if (buttons.power_on) {
                /* Power on button pressed */
            }
            if (buttons.power_off) {
                /* Power off button pressed */
            }
            /* Add other button handling as needed */
        }

        /* 100ms tick */
        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(100));
    }
}

/**
 * @brief Protection Task - monitors protection conditions
 */
void app_protect_task(void *argument)
{
    TickType_t last_wake_time;

    (void)argument;

    last_wake_time = xTaskGetTickCount();

    /* Initialize protection */
    protection_init();

    /* Initialize frequency detector */
    freq_detect_init();

    /* Start frequency detection when in TX */
    freq_detect_start();

    while (1) {
        /* Check protection conditions */
        if (g_power_state == POWER_STATE_TX) {
            protection_check();
        }

        /* Check frequency if transmitting */
        if (g_power_state == POWER_STATE_TX) {
            freq_detect_check();
        }

        /* If alert triggered, update GUI alert if needed */
        if (g_alert_flag) {
            g_alert_flag = false;
            if (!gui_is_alert_active()) {
                gui_show_alert(g_alert_active);
            }
            /* Play alert sound */
            power_play_alert();
        }

        /* Check if alert was cleared (from GUI or comm) */
        if (!protection_is_active() && gui_is_alert_active()) {
            /* Protection cleared externally */
            gui_dismiss_alert();
            /* Stop alert sound */
            power_set_buzzer(false);
        }

        /* 20ms tick - 50Hz protection check */
        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(20));
    }
}