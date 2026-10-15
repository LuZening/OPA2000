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
#include "buzzer.h"
#include "fan_tach.h"
#include "radio_uart.h"
#include "config.h"

/*============================================================================
 * Private Variables
 *============================================================================*/

/* Task handles */
static TaskHandle_t gui_task_handle = NULL;
static TaskHandle_t adc_task_handle = NULL;
static TaskHandle_t comm_task_handle = NULL;
static TaskHandle_t monitor_task_handle = NULL;
static TaskHandle_t protect_task_handle = NULL;
static TaskHandle_t config_task_handle = NULL;

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

    /* Config Save Task - lowest priority, see app_config_task() */
    result = xTaskCreate(
        app_config_task,
        "Config Task",
        STACK_SIZE_CONFIG_TASK,
        NULL,
        PRIORITY_CONFIG_TASK,
        &config_task_handle
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

    /* touch_init() and the g_calibration_required flag were already set
     * by main() before osKernelStart().  We only need to land on the
     * right first screen.  Re-check the flag once here in case the
     * touch calibration menu (if displayed) toggles it. */
    if (g_calibration_required) {
        gui_show_calibration();
    }

    /* Note: touch driver is already running (probed in main() during
     * the single-threaded boot step), so we go straight into the
     * polling loop here. */

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
        /* Publish the instantaneous engineering values.  Note this also
           runs from the ADC1 DMA interrupt, so the task-side call is only
           a second, slower consumer of the same data - not the primary
           path. */
        adc_process();

        /* Advance the coupler-2 peak holds for the GUI bar graphs.  Task
           context on purpose: the release step uses expf(), which must not
           run in the priority-0 DMA interrupt. */
        adc_peak_hold_tick();

        /* Transmit gate (design spec step 6): the amplifier may only be
           keyed once BOTH halves are true -
             g_allow_rf    the IM zero-offset calibration finished, so the
                           main-current reading is trustworthy, and
             g_gui_ready   the GUI painted a frame, so the operator can
                           actually see what the amp is doing.
           Until then power_control's ptt_apply_level() refuses every
           request.  Re-evaluated every 50 ms; the flags are volatile so
           this is a cheap poll. */
        g_ptt_ready = (g_allow_rf && g_gui_ready);

        /* 50ms - 20Hz update rate */
        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(50));
    }
}

/**
 * @brief Communication Task - handles USB and serial communication
 */
void app_comm_task(void *argument)
{
    (void)argument;

    /* No vTaskDelayUntil() anchor here: the loop blocks for an
     * unpredictable time in radio_uart_wait_frame(), so there is no
     * fixed grid left to "catch up" on.  The CAT poll accumulator keeps
     * its own 100 ms cadence via radio_band_tick()'s internal counter,
     * which is unaffected by how often this loop runs. */

    /* Initialize communication */
    comm_init();
    comm_start();

    /* Start the radio CAT link: UART4 DMA + IDLE receive, with the
     * protocol layer attached.  After this the task only has to pump the
     * poll accumulator and drain received bytes. */
    (void)radio_uart_init();

    while (1) {
        /* Process received commands */
        comm_process_rx();

        /* Radio CAT poll accumulator: ticked every 10 ms, sends a query
         * every 100 ms.  Must run regardless of whether a CAT frame just
         * arrived, otherwise an unanswered radio would stop being polled
         * and we could never discover its protocol in the first place. */
        radio_band_tick();
        radio_band_poll();

        /* Block until the UART4 ISR hands over a frame, then let
         * radio_band slice it, identify the protocol and switch band.
         * This is what keeps frame parsing out of the ISR.
         *
         * Timeout is one poll interval rather than "forever" so the two
         * calls above keep their 10 ms cadence even when the radio is
         * silent - which is the normal state, since a CAT radio only
         * answers when asked. */
        (void)radio_uart_wait_frame(RADIO_BAND_POLL_PERIOD_MS);

        /* Re-anchor the delay: radio_uart_wait_frame() blocks for an
         * unpredictable time, so vTaskDelayUntil() can no longer be used
         * to hold the 10 ms grid (it would try to "catch up" by bursting
         * iterations).  A plain delay is correct here and the tick above
         * is what actually needs the grid. */
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

/**
 * @brief Monitor Task - fan control, temperature monitoring
 */
void app_monitor_task(void *argument)
{
    TickType_t last_wake_time;
    static bool ptt_last_handled = false;  /* Track last handled PTT state */
    /* Divider for the slow housekeeping below.  The loop itself must run at
     * 10 ms so power_rail_step() can honour the HV sequencing gap, but the
     * PTT edge check, fan curve and tach only need 100 ms. */
    static uint32_t slow_div = 0U;

    (void)argument;

    last_wake_time = xTaskGetTickCount();

    /* Initialize power control */
    power_init();
    buzzer_init();

    /* Initialize PTT state tracking based on current GPIO state */
    ptt_last_handled = power_get_ptt_state();

    while (1) {
        /* HV rail sequencing first, on every tick.  This is why the loop
         * runs at 10 ms: power_psu_on() / power_psu_off() only throw one of
         * the two switches, and power_rail_step() performs the second
         * HV_STAGE_DELAY_MS later.  It must be serviced even while keyed
         * or a fault is latched - dropping the bias late is worse than
         * dropping it early. */
        power_rail_step();

        if (++slow_div >= 10U) {
        slow_div = 0U;
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
                /* PTT pressed.  power_tx_enable() does its own full interlock
                 * check (g_hv_on, power_rail_busy(), g_alert_flag, ...), so
                 * there is no point pre-screening the flag here. */
                power_tx_enable();
            } else {
                /* PTT released */
                if (g_tx_active) {
                    power_tx_disable();
                }
            }

            /* Update last handled state to prevent duplicate triggers */
            ptt_last_handled = ptt_current;
        }

        /* Update fan based on temperature */
        power_update_fan(g_adc_data.ntc_core_tenth_c);

        /* Update band from frequency detector */
        if (g_tx_active) {
            power_update_band(freq_detect_get_frequency());
        }

        /* Fan / pump tachometer, refreshed on the same slow tick */
        fan_tach_calculate_rpm();
        }   /* slow_div */

        /* 10ms tick: sets the HV rail sequencing resolution. */
        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(10));
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
        /* Backstop for the conditions that have no hardware watchdog.  Runs
         * unconditionally: NTC over-temperature applies while idle too, and
         * the coupler limits inside protection_check() gate on g_tx_active
         * themselves. */
        protection_check();

        /* Check frequency if transmitting */
        if (g_tx_active) {
            freq_detect_check();
        }

        /* NOTE: the alert popup and the buzzer are NOT driven from here.
         * protection_trigger_locked() already latches g_alert_flag and
         * starts the 3-tone warning the instant the fault happens, and
         * protection_clear() silences it.  Re-touching either here would
         * race with the GUI task (which owns the popup) and with the CAT /
         * SPI6 command path.  See docs/AGENT_BRIEFING.md 5.4 / 5.5. */

        /* 20ms tick - 50Hz protection check */
        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(20));
    }
}

/**
 * @brief Config Save Task
 *
 * The only place in the firmware that writes the internal flash.
 *
 * Programming a flash word stalls instruction fetch for the duration, and
 * erasing a sector stalls it much longer - tens of milliseconds.  Nothing
 * in the GUI or the CAT link may be held up by that, so the actual write
 * happens here at the lowest priority in the system: every other task,
 * including the protection task, preempts it freely.
 *
 * The 1 s period is a coalescing window.  Dragging a protection-threshold
 * slider raises the dirty flag on every LVGL value-changed event; without
 * this delay a single gesture would append dozens of records and burn
 * through the ring for nothing.  One write per second is fast enough that
 * a power cut loses at most one second of tuning.
 */
void app_config_task(void *argument)
{
    TickType_t last_wake_time;

    (void)argument;

    last_wake_time = xTaskGetTickCount();

    while (1) {
        if (config_is_dirty()) {
            /* Clear first, then write.  If an edit lands between the clear
             * and the write, config_save() stores the newer values and the
             * flag is still set for the next pass - the edit is not lost.
             * Clearing afterwards instead would swallow it. */
            config_clear_dirty();
            (void)config_save();
        }

        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(1000));
    }
}