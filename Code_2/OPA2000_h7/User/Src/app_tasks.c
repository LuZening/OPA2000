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

        /* Kick off the next touch poll.  The samples themselves arrive via
         * the I2C2 DMA completion interrupt and are read by LVGL's indev
         * read_cb inside gui_tick() above - the GUI never has to look at a
         * touch sample itself. */
        touch_process();

        /* Process calibration input during calibration */
        gui_process_calibration();
        gui_process_coupler_cal();
        gui_update_sensor_inspect();

        /* Update display values */
        gui_update_display();

        /* ================= 保护告警弹窗 =================
         *
         * 故障路径（protection_trigger_locked()，中断上下文）不能碰 LVGL，
         * 它只置 g_alert_flag；把标志变成弹窗是这里的事。
         *
         * 两个变量都是 volatile 且由中断侧写：g_alert_flag 是 bool（1 字节，
         * 单字节访问在 M7 上原子），g_alert_active 是 AlertType（枚举，GCC
         * 下 int = 4 字节，对齐访问同样原子）。所以直接读是安全的——不依赖
         * 任何锁，也不依赖"读两次比对"这种技巧。
         *
         * 顺序很重要：先看"有没有弹窗"再看"要不要弹"，否则保护已解除而弹窗
         * 还在时会被下面那个分支抢先关掉，弹窗就再也不会出现。 */
        if (!gui_is_alert_active()) {
            if (g_alert_flag) {
                gui_show_alert(g_alert_active);
            }
        } else {
            /* 弹窗还开着，但保护侧已经解除（CAT 下过 ALERT OFF，或故障条件
             * 自行消失）——这时把残留的弹窗收掉。 */
            if (!protection_is_active()) {
                gui_dismiss_alert();
            }
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

        /* Drive the CC6920 IM zero-offset calibration.  This is the ONLY
           writer of g_allow_rf = true (see adc_im_zero_offset_tick()), and
           adc_init() deliberately clears both im_calibrated and g_allow_rf
           on every boot, so without this call the amplifier could never
           key at all - ptt_apply_level() refuses every request while
           g_allow_rf is false.

           It runs at the 50 ms task tick rather than the 10 ms the state
           machine was dimensioned for, which stretches the 50-sample
           window from ~500 ms to ~2.5 s.  That is fine: the window only has
           to average out ADC noise while the PA draws no current, and at
           boot the rail is still down.

           Self-limiting: the state machine returns immediately once
           g_config.im_calibrated is set, so after the first successful
           pass this is a single load-and-branch per tick.  On failure it
           retries up to 3 times and then raises ALERT_HARDWARE_FAULT by
           itself (user_adc.c) - no operator action required either way. */
        (void)adc_im_zero_offset_tick();

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
 * @brief Communication Task - handles USB / LPUART1 (PC CAT) and UART4 (radio CAT)
 */
void app_comm_task(void *argument)
{
    (void)argument;

    /* 持有"上一帧生效的 radio_baud"。每次循环开头比较：若 PC 端已
     * 改了 g_config.radio_baud_rate，就调 radio_uart_set_baud()
     * 让 UART4 立即生效。值 0 = 未生效；>0 = 第一次比较。 */
    uint32_t last_applied_radio_baud = 0U;

    /* No vTaskDelayUntil() anchor here: the loop blocks for an
     * unpredictable time in radio_uart_wait_frame(), so there is no
     * fixed grid left to "catch up" on.  The CAT poll accumulator keeps
     * its own 200 ms cadence via radio_band_tick()'s internal counter,
     * which is unaffected by how often this loop runs. */

    /* Initialize communication */
    comm_init();
    comm_start();

    /* Start the radio CAT link: UART4 DMA + IDLE receive, with the
     * protocol layer attached.  After this the task only has to pump the
     * poll accumulator and drain received bytes. */
    (void)radio_uart_init();
    last_applied_radio_baud = g_config.radio_baud_rate;

    while (1) {
        /* 0) GUI 改 radio_baud 后，g_config 已经被写但 UART4 还在用
         *    旧值。comm 任务拿到机会把它落实——这是用户要求"GUI 改
         *    立即生效"的具体实现。最多每轮执行一次。 */
        if (g_config.radio_baud_rate != 0U &&
            g_config.radio_baud_rate != last_applied_radio_baud) {
            (void)radio_uart_set_baud(g_config.radio_baud_rate);
            last_applied_radio_baud = g_config.radio_baud_rate;
        }

        /* 1) PC CAT 接收：从 s_rx_pending 取一段，按行解析与执行。
         *    LPUART1 的 IDLE 中断已 memcpy 到 s_rx_pending。 */
        comm_process_rx();

        /* 2) 100ms 主动上报一行 status 到 PC。comm_periodic_tick()
         *    内部 10ms 累加到 10 次后发一行。 */
        comm_periodic_tick();

        /* 2a) 事件推送：comm_alert_pump() 把 alert=ON/OFF 事件 ring
         *     里的内容写到 s_tx_ring。每次 comm 任务循环都做（10ms），
         *     让保护事件在 < 10ms 内送达 PC，不必等 100ms status 周期。 */
        comm_alert_pump();

        /* 3) TX pump：把 s_tx_ring 里的数据搬到 s_tx_dma_buf 并
         *    启动 DMA，DMA 完成后由 HAL_UART_TxCpltCallback 拉
         *    s_tx_dma_busy = false，下一次 pump 继续从 ring 取。 */
        comm_tx_pump();

        /* 3) Radio CAT poll accumulator: ticked every 10 ms, sends a
         *    query every 200 ms.  Must run regardless of whether a CAT
         *    frame just arrived, otherwise an unanswered radio would
         *    stop being polled and we could never discover its protocol. */
        radio_band_tick();
        radio_band_poll();

        /* 4) Block until the UART4 ISR hands over a frame, then let
         *    radio_band slice it, identify the protocol and switch band.
         *    This is what keeps frame parsing out of the ISR.
         *
         *    Timeout is one poll interval (200 ms in 10 ms tick) rather
         *    than "forever" so the three calls above keep their cadence
         *    even when the radio is silent. */
        (void)radio_uart_wait_frame(RADIO_BAND_POLL_PERIOD_MS);

        /* 5) Plain delay rather than vTaskDelayUntil(): radio_uart_wait_frame
         *    blocks for an unpredictable time, so the "absolute deadline"
         *    anchor no longer matches.  10 ms keeps the loop responsive. */
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