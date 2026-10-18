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
/* g_freertos_fault, latched by app_task_create_failed() and the FreeRTOS
 * failure hooks in freertos_hooks.c. */
#include "opa2000.h"
/* osDelay() for the external-latch reset pulse in the protection task. */
#include "cmsis_os2.h"
/* CLEAR_FAULTX_OC_LOW/HIGH and RF_BIAS_ENABLE/DISABLE pin macros. */
#include "hw_pinmap.h"
#include "lv_port_disp.h"

/*!< How long the protection task holds CLEAR_FAULTX_OC and RF_BIAS low when
 *   the operator dismisses an alert.  Long enough for the external latch to
 *   reset, short enough that the operator does not notice the PA blink. */
#define PROTECT_LATCH_CLEAR_MS    100U

/*============================================================================
 * Private Variables
 *============================================================================*/

/* Task handles */
static TaskHandle_t gui_handler_task_handle = NULL;
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

    /* A task that failed to create never runs, and the symptom appears
     * somewhere else entirely: a missing Protect task looks exactly like a
     * working one whose comparator interrupt never fires, and a missing
     * Monitor task looks like the power rail never coming up.  There is no
     * way to recover from it either - the scheduler is already running by
     * the time anyone notices.  So: name the task, then stop.  No printing
     * (the heap is exactly what just ran out) and no RTOS calls. */
#define OPA2000_REQUIRE_TASK(name, fn, stack, prio, handle)          \
    do {                                                             \
        result = xTaskCreate((fn), (name), (stack), NULL, (prio),    \
                             &(handle));                              \
        if (result != pdPASS) {                                      \
            g_freertos_fault = 0xDEAD0003U;                          \
            app_task_create_failed(name);                           \
        }                                                            \
    } while (0)

    /* GUI Task */
    OPA2000_REQUIRE_TASK("GUI Handler", app_gui_handler_task,
                         STACK_SIZE_GUI_TASK, PRIORITY_GUI_TASK,
                         gui_handler_task_handle);

    OPA2000_REQUIRE_TASK("GUI Task", app_gui_task,
                         STACK_SIZE_GUI_TASK, PRIORITY_GUI_TASK,
                         gui_task_handle);

    /* ADC Task */
    OPA2000_REQUIRE_TASK("ADC Task", app_adc_task,
                         STACK_SIZE_ADC_TASK, PRIORITY_ADC_TASK,
                         adc_task_handle);

    /* Communication Task */
    OPA2000_REQUIRE_TASK("Comm Task", app_comm_task,
                         STACK_SIZE_COMM_TASK, PRIORITY_COMM_TASK,
                         comm_task_handle);

    /* Monitor Task */
    OPA2000_REQUIRE_TASK("Monitor Task", app_monitor_task,
                         STACK_SIZE_MONITOR_TASK, PRIORITY_MONITOR_TASK,
                         monitor_task_handle);

    /* Protection Task */
    OPA2000_REQUIRE_TASK("Protect Task", app_protect_task,
                         STACK_SIZE_PROTECT_TASK, PRIORITY_PROTECT_TASK,
                         protect_task_handle);

    /* Config Save Task - lowest priority, see app_config_task() */
    OPA2000_REQUIRE_TASK("Config Task", app_config_task,
                         STACK_SIZE_CONFIG_TASK, PRIORITY_CONFIG_TASK,
                         config_task_handle);

#undef OPA2000_REQUIRE_TASK
}

void app_gui_handler_task(void* argument)
{
    (void)argument;


    /* Start GUI */
    osThreadFlagsWait(0x01U, osFlagsWaitAll, osWaitForever); // wait for start signal, after GUI init
    while (1)
    {
        gui_lock();
        /* Process LVGL handler */
        uint32_t time_till_next_ms = lv_task_handler();
        gui_unlock();
        osDelay(pdMS_TO_TICKS(time_till_next_ms));
    }
}

void app_task_create_failed(const char *task_name)
{
    /* g_freertos_fault is already latched by the caller.  Park here with
     * interrupts off: a half-built task set is not a state the protection
     * chain can safely run in, and continuing would let the PA be keyed
     * with no Protect task watching. */
    (void)task_name;
    __disable_irq();
    for (;;) {
    }
}

/**
 * @brief GUI Task - handles LVGL and user input
 */
void app_gui_task(void *argument)
{
    TickType_t last_wake_time;
    /* touch_init() and the g_calibration_required flag were already set
     * by main() before osKernelStart().  We only need to land on the
     * right first screen.  Re-check the flag once here in case the
     * touch calibration menu (if displayed) toggles it. */
//    for(int i = 0; i< LCD_WIDTH * LCD_HEIGHT; ++i)
//        lcd_framebuffer[i] = 0x1234;

    /* Initialize GUI */
    gui_init();
#if 1
    if (g_calibration_required) {
        gui_show_calibration();
    }
#endif
    /* Note: touch driver is already running (probed in main() during
     * the single-threaded boot step), so we go straight into the
     * polling loop here. */
    osThreadFlagsSet(gui_handler_task_handle, 0x01U); // allow LVGL handler routine to start
    last_wake_time = xTaskGetTickCount();
    while (1) {
        gui_lock();
        /* Process LVGL handler */
//        HAL_IWDG_Refresh(&hiwdg1);
        /* Kick off the next touch poll.  The samples themselves arrive via
         * the I2C2 DMA completion interrupt and are read by LVGL's indev
         * read_cb inside gui_tick() above - the GUI never has to look at a
         * touch sample itself. */
        touch_process();
        /* Process calibration input during calibration */
        gui_process_calibration();
        gui_process_coupler_cal();
        gui_update_sensor_inspect();
        /* Update display values, enable TFT display */
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

        gui_unlock();
        /* 50ms tick */
        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(50));
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
//        adc_process();

        /* 上电稳定等待 + CC6920 主电流零点校准（user_adc.c 里的状态机）。
           它一次性包办了整段上电流程，所以这里不可能漏掉某一步，也不
           可能有两个地方同时在跑：

             1. SETTLING     前 1 s，什么都不信。CLEAR_FAULTX_OC 持续
                             拉低，外部故障锁存器被按住不放；同时
                             boot_hold_thresholds_open() 在进入 RTOS 之前
                             就已把 AWD1/2/3 和两路 COMP 的 DAC 全部推到
                             满量程，所以这 1 s 里不会误报。
             2. CALIBRATING  跑 IM 零点校准（50 次采样 + 校验）。
             3. ARMED        校准通过，此刻才认为 ADC 值有效：
                             boot_apply_thresholds() 把真正的 AWD/COMP
                             门槛下发下去，CLEAR_FAULTX_OC 释放，GUI 才开始
                             显示这些读数。

           这是 g_allow_rf = true 的唯一写者，而 adc_init() 每次上电都会
           清掉 im_calibrated 和 g_allow_rf，所以没有这次调用放大器永远
           无法起振——ptt_apply_level() 在 g_allow_rf 为假时拒绝一切请求。

           校准窗口在 50 ms 的任务节拍上跑（状态机按 10 ms 设计），所以
           50 次采样从 ~500 ms 拉长到 ~2.5 s。无妨：它只要在 PA 不取流
           的情况下把 ADC 噪声平均掉，而上电时高压还没建立。

           自限：g_config.im_calibrated 一旦置位状态机立刻返回，所以成功
           之后每拍只剩一次取值+分支。失败则一直重试下去、不再告警——
           零点偏移不收敛多半是冷机/热机漂移而不是硬件坏了，对它报警只会
           训练操作员忽略告警。代价是失败时放大器一直无法起振
           （g_allow_rf 保持为假），但不会有误动作。 */
        (void)adc_stabilisation_tick();

        /* Advance the coupler-2 peak holds for the GUI bar graphs.  Task
           context on purpose: the release step uses expf(), which must not
           run in the priority-0 DMA interrupt. */
        adc_peak_hold_tick();

        /* Re-arm the ADC2 injected conversions (NTC core, NTC aux, die
           temp).  They are configured software-start with no trigger, so
           they run once and park - without this the NTC numbers freeze at
           whatever they read at boot.  Self-gated to 100 ms. */
        adc_injected_tick();

        /* Transmit gate (design spec step 6): the amplifier may only be
           keyed once ALL THREE halves are true -
             adc_stabilisation_done()  上电稳定等待期结束：模拟量稳定，
                           零点校准完成，真正的 AWD/COMP 门槛已下发，
                           CLEAR_FAULTX_OC 已释放。
             g_allow_rf    the IM zero-offset calibration finished, so the
                           main-current reading is trustworthy, and
             g_gui_ready   the GUI painted a frame, so the operator can
                           actually see what the amp is doing.
           Until then power_control's ptt_apply_level() refuses every
           request.  Re-evaluated every 50 ms; the flags are volatile so
           this is a cheap poll. */
        g_ptt_ready = (adc_stabilisation_done() && g_allow_rf && g_gui_ready);

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
        vTaskDelay(pdMS_TO_TICKS(20));
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

    /* TACH edge counters.  The ISR itself is armed by freq_detect_init()
     * (it owns TIM7); this only clears the counters and seeds the RPM
     * window timestamp so the first snapshot has a baseline. */
    fan_tach_init();

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
        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(100));
    }
}

/**
 * @brief External fault latch reset pulse - runs in the protection task
 *
 *  CLEAR_FAULTX_OC (PG1) is open-drain into C65-2 / D10-1 / R71-1.  Holding
 *  it low is what actually resets the external fault latch; left floating,
 *  the latch stays closed - so the operator taps "OK", the firmware's own
 *  latch clears, and the hardware fault survives.
 *
 *  RF_BIAS (PG2) goes down for the same window.  The latch reset is only
 *  meaningful while the PA is not being driven, and cutting the bias for
 *  100 ms stops an RF-side event from re-latching the fault in the instant
 *  the reset takes effect.
 *
 *  RF_BIAS is restored to whatever it was, not blindly enabled: a P0
 *  shutdown already dropped it (protection_trigger_locked() step 4) and a P1
 *  dismissal never raised it.  Enabling it unconditionally would key a PA
 *  the operator had deliberately left idle, or undo a shutdown that is
 *  still meant to hold.
 *
 *  Why this task does the delay: app_monitor_task (priority 5) runs
 *  power_rail_step() on a 10 ms tick, and power_rail_step() calls
 *  RF_BIAS_ENABLE().  The protection task sits at priority 6, so osDelay()
 *  here keeps the monitor out for the whole window and the level we restore
 *  is genuinely the level we sampled.  Nothing of consequence runs below
 *  priority 5, so the 100 ms costs only the monitor's fan curve.
 */
static void protect_task_latch_clear_pulse(void)
{
    /* Sample BEFORE pulling anything down - reading afterwards would of
     * course just return the level we ourselves wrote. */
    GPIO_PinState bias_was = HAL_GPIO_ReadPin(RF_BIAS_GPIO_Port, RF_BIAS_Pin);

    CLEAR_FAULTX_OC_LOW();
    RF_BIAS_DISABLE();

    osDelay(pdMS_TO_TICKS(PROTECT_LATCH_CLEAR_MS));

    CLEAR_FAULTX_OC_HIGH();
    if (bias_was == GPIO_PIN_SET) {
        RF_BIAS_ENABLE();
    }
    /* else: leave it low.  It was low going in, so either the PA is idle or
     * a P0 shutdown is still in force - both want it to stay down. */
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
        /* External latch reset requested by a dismiss.  Runs before
         * protection_check() so the hardware latch is back to a known state
         * as early as possible, and costs one 100 ms osDelay in which only
         * the monitor task (priority 5) gets to run. */
        if (protection_take_latch_clear_request()) {
            protect_task_latch_clear_pulse();
        }

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
