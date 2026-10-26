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
#include "overdrive.h"
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

    /* The config mutex has to exist before any task can take it.  Created
     * here rather than lazily inside config_lock(), because two tasks
     * calling config_lock() for the first time at the same moment would
     * otherwise each create their own mutex and end up with two
     * different locks - which is exactly the bug the lock was meant to
     * prevent. */
    config_mutex_init();

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

    /* Initialize ADC.  This resets the peak holds, the injected re-arm
     * timer and the stabilisation state machine - all of which must be zero
     * before the first sample arrives, or the GUI would show whatever the
     * DMA buffer happened to contain. */
    adc_init();

    /* Overdrive indicator: clear the 200 ms peak window and the beep latch
     * before the first sample, otherwise the first pass would judge a window
     * full of stale zeros.  While the DMA is stopped below, adc1_raw stays
     * zeroed, so the LED simply stays dark through the silence. */
    overdrive_init();

    /* Power-on silence.  adc_presettle() brings the regulators and the
     * reference buffers up but deliberately takes no sample; the DMA stays
     * stopped for ADC_PRESETTLE_MS so the cold-conversion transient never
     * reaches g_adc_data.  A cold ADC's first conversions are wrong in a
     * structured way - regulator charge, reference settling, the CC6920
     * bias and the coupler detectors' DC rest point all still moving - and
     * the peak holds (tau = 5 s) would stretch that into a visible spike
     * that reads like a real transmission.
     *
     * vTaskDelay, not HAL_Delay: the GUI task has to come up in parallel to
     * raise g_gui_ready, and g_ptt_ready cannot be satisfied until both that
     * and the IMAIN calibration are done. */
    adc_presettle();
    vTaskDelay(pdMS_TO_TICKS(ADC_PRESETTLE_MS));

    /* Sampling from here on.  last_wake_time is re-seeded because the
     * silence above consumed the original reference point, and
     * vTaskDelayUntil() would otherwise try to catch up in a burst. */
    adc_start();
    CLEAR_FAULTX_OC_LOW();
    RF_BIAS_DISABLE();
    osDelay(10);
    CLEAR_FAULTX_OC_HIGH();
    last_wake_time = xTaskGetTickCount();
    while (1) {
        /* Publish the instantaneous engineering values.  Note this also
           runs from the ADC1 DMA interrupt, so the task-side call is only
           a second, slower consumer of the same data - not the primary
           path. */
//        adc_process();

        /* 上电流程 + CC6920 主电流零点校准（user_adc.c 里的状态机）。
           它一次性包办了整段上电流程，所以这里不可能漏掉某一步，也不
           可能有两个地方同时在跑：

             1. 静默期 100 ms（adc_presettle + 上面的 vTaskDelay）。
                这段时间不取样：转换器已上电但 DMA 未启动，所以冷启动
                的瞬态不会进入 g_adc_data。同时
                boot_hold_thresholds_open() 在进入 RTOS 之前就已把
                AWD1/2/3 和两路 COMP 的 DAC 全部推到满量程，所以这
                100 ms 里不会误报。
             2. ARMING       静默期结束后的第一拍：真正的 AWD/COMP 门槛
                             下发，CLEAR_FAULTX_OC 释放，比较器启动，
                             g_allow_rf 置位 —— 从这一刻起操作员可以按键。
             3. 后台校准     IM 零点校准在后台跑（50 次采样 + 校验），
                             成功即写入 im_zero_offset_adc 并重新拟合
                             AWD2 门槛。它是一个"测量"步骤，不再是
                             "发射许可"的前提：传感器不收敛只是电流读数
                             不可信，不该让功放变成砖。

           校准窗口在 50 ms 的任务节拍上跑（状态机按 10 ms 设计），所以
           50 次采样从 ~500 ms 拉长到 ~2.5 s。无妨：它只要在 PA 不取流
           的情况下把 ADC 噪声平均掉，而上电时高压还没建立。

           自限：g_config.im_calibrated 一旦置位状态机立刻返回，所以成功
           之后每拍只剩一次取值+分支。失败则一直重试下去、不再告警——
           零点偏移不收敛多半是冷机/热机漂移而不是硬件坏了，对它报警只会
           训练操作员忽略告警。代价是电流读数一直是 0（未标定），但功放
           照样能正常发射，保护链也照样工作。 */
        (void)adc_stabilisation_tick();

        /* Advance the coupler-2 peak holds for the GUI bar graphs.  Task
           context on purpose: the release step uses expf(), which must not
           run in the priority-0 DMA interrupt. */
        adc_peak_hold_tick();

        /* Overdrive indicator: 200 ms peak of the RF-input detector (INP4,
         * the same node COMP1 watches).  One P0 buzzer cycle is started per
         * excursion - overdrive_take_beep() only hands out one request until
         * the level has fallen back below the trip point, so a complete
         * buzzer cycle always plays out before the next one can begin. */
//         NOTE: implement software overdrive protection later
//        overdrive_tick(HAL_GetTick(),
//                       (uint16_t)g_adc_data.adc1_raw[ADC1_RANK_RF_IN]);
//        if (overdrive_take_beep()) {
//            buzzer_start();
//        }

        /* Re-arm the ADC2 injected conversions (NTC core, NTC aux, die
           temp).  They are configured software-start with no trigger, so
           they run once and park - without this the NTC numbers freeze at
           whatever they read at boot.  Self-gated to 100 ms. */
        adc_injected_tick();

        /* Transmit gate (design spec step 6): the amplifier may only be
           keyed once ALL THREE halves are true -
             adc_stabilisation_done()  静默期已过、真正的 AWD/COMP 门槛
                           已下发、CLEAR_FAULTX_OC 已释放、比较器已启动。
                           注意这不再包含"零点校准完成"——校准是测量
                           步骤，不是发射许可的前提。
             g_allow_rf    no fault is latched.  上电路径在静默期结束时
                           置位，protection.c 在 P0 故障时清零、解除
                           故障时恢复。
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
        // Note: suppress Comm task at this moment.
#if 0
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
        comm_alert_pump(); // Note: Hard fault right after dismiss

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
#endif
    }
}

/**
 * @brief Monitor Task - fan control, temperature monitoring
 */
void app_monitor_task(void *argument)
{
    TickType_t last_wake_time;
    static bool ptt_last_handled = false;  /* Track last handled PTT state */
    /* Divider for the slow housekeeping below.  The loop itself runs at
     * 100 ms so power_rail_step() services the HV sequencing, and the PTT
     * edge check, fan curve and tach run every 10th pass of it. */
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
        /* Release the IM (main current) mute once its HV-inrush window has
         * elapsed.  power_hv_on() raises it; this task is the only place
         * that clears it, and the two handlers only test the flag. */
        protection_im_mask_tick();

        /* Timed clear of the external fault latch.  This task is the
         * heartbeat that drives the pin: protection_clear_fault_safe() only
         * raises a flag, and the service below pulls CLEAR_FAULTX_OC low
         * and puts it back when the requested time is up.  The callers are
         * interrupt handlers and command handlers, so neither of them can
         * afford to hold a pin down itself. */
        protection_fault_clear_service();

        /* HV rail sequencing first, on every tick.  power_hv_on() /
         * power_hv_off() only throw one of the two switches, and
         * power_rail_step() performs the second one tick later - i.e. the
         * bias-before-rail gap is one monitor tick here, not the nominal
         * HV_STAGE_DELAY_MS.  It must be serviced even while keyed or a
         * fault is latched - dropping the bias late is worse than dropping
         * it early. */
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

        /* 100 ms tick: sets the HV rail sequencing resolution AND the
         * granularity of the IM inrush-mute release, which is why
         * IM_MASK_HOLD_MS is 100 ms.  The PTT edge check, fan curve and
         * tach are divided further by slow_div. */
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
 *  power_rail_step() on a 100 ms tick, and power_rail_step() calls
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

    /* The clear line goes through protection.c's holder mask, so a timed
     * clear service running at the same time cannot have its pulse cut
     * short by this one (or the other way round). */
    protection_fault_clear_hold();
    RF_BIAS_DISABLE();

    osDelay(pdMS_TO_TICKS(PROTECT_LATCH_CLEAR_MS));

    protection_fault_clear_release();
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
         * unconditionally: NTC over-temperature, the coupler limits and
         * the PA bias all apply whether or not the amplifier is keyed,
         * because an idle PA can be struck by a stray carrier or have a
         * failing LPF - and with no legitimate signal in flight those
         * readings mean even more, not less. */
        protection_check();

        /* Frequency check: unconditional.  It drives the LPF band follow, and
         * a frequency that disagrees with the selected band is just as
         * wrong when the amplifier is idle as when it is keyed - so the
         * keying state must not decide whether this runs.  The
         * measurement itself is validated inside freq_detect.c (valid
         * frequency window, window-accepted, 3-sample agreement), which
         * is what keeps idle noise from being mistaken for a carrier. */
        freq_detect_check();

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
 * The task does NOT poll.  It blocks on a thread flag (CONFIG_SAVE_FLAG),
 * and whoever changes a config field calls config_mark_dirty(), which sets
 * the dirty flag and posts the flag.  That replaced a 1 s poll.
 *
 * The poll existed only to coalesce a burst of edits into a single write:
 * dragging a protection-threshold slider raises the dirty flag on every
 * LVGL value-changed event, and without a window one gesture would append
 * dozens of records and burn through the ring for nothing.  The GUI's own
 * staging does that job better - s_menu_dirty is raised by the sub-page
 * callbacks and committed when the session ends, so the write lands on a
 * boundary the operator perceives as "I finished editing" rather than on an
 * arbitrary timer edge.  A power cut mid-session now restores the previous
 * settings instead of a half-applied set.
 *
 * A lost wake-up cannot lose an edit: the thread flag is sticky, and the
 * task re-reads config_is_dirty() after every wake.
 */
void app_config_task(void *argument)
{
    (void)argument;

    /* Publish the task id so config_mark_dirty() can wake this task.  Done
     * before the first wait, so an edit made between here and the wait is
     * not lost - the flag is sticky and the dirty flag is checked first
     * thing below anyway.  config_set_save_task() also posts the flag if
     * an edit was already pending, which covers the boot window before
     * this task existed. */
    config_set_save_task((void *)osThreadGetId());

    while (1) {
        /* Block until someone changes something.  A thread flag rather than
         * a 1 s poll: the poll existed only to coalesce a burst of edits
         * into one write, and the GUI's staging (s_menu_dirty, committed
         * when the session ends) does that job better - the save now lands
         * on a boundary the operator perceives as "I finished editing",
         * not on an arbitrary timer edge.
         *
         * The wait returns immediately when the flag is already set, so a
         * wake-up that arrives while the previous write is still running is
         * simply remembered rather than lost. */
        (void)osThreadFlagsWait(CONFIG_SAVE_FLAG, osFlagsWaitAll, osWaitForever);

        /* config_flush_pending() does the clear-then-write under the
         * config lock, so an edit that lands while the write is running is
         * kept for the next pass rather than swallowed.
         *
         * It returns true only when a write actually happened AND
         * succeeded, so the toast is raised for real saves and not for
         * the no-op wake-ups or the retry case (where it re-raises the
         * dirty flag and comes straight back).
         *
         * The toast takes the GUI mutex itself and releases it as soon as
         * the widget exists, so the only thing this lowest-priority task
         * does to the GUI is a few microseconds of object creation.  The
         * three seconds it then stays on screen cost nothing here - LVGL
         * drives its fade from lv_task_handler() in the GUI handler task,
         * not from here. */
        if (config_flush_pending()) {
            gui_toast_config_saved();
        }
        osDelay(pdMS_TO_TICKS(2000)); // at most write once each 2seconds
    }
}
