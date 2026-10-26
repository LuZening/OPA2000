/**
 * @file protection.c
 * @brief Protection system implementation.
 *
 * @details Two families of source, one shutdown path.
 *
 * Digital - a discrete fault line on an EXTI pin, handled inside the EXTI
 * ISR.  HARDERR_PWR (PB12) comes from the power board and is ACTIVE HIGH.
 * The handler re-reads the pin before acting so a glitch edge cannot shut
 * the amplifier down.  protection_digital_isr() is the generic entry for
 * the RF deck lines still to be added.
 *
 * Analog - a conversion result against a limit.  Every limit lives in
 * g_config, never as a constant in this file (see "Protection limits"
 * below).  Where the analog watchdog owns the channel the comparison is
 * in hardware and the AWD interrupt drives the shutdown:
 *     AWD2 -> CH5  IMain   g_config.band_im_max_mA
 *     AWD3 -> CH6  REV1    g_config.band_rev1_max_w
 * The channels with no watchdog behind them are compared in interrupt
 * context right after the fresh results are published:
 *     ADC1 DMA complete   REV2 / SWR / FWD1 / FWD2 / VBIAS config limits
 *     ADC2 injected done  NTC   g_config.band_ntc_max_tenth_c
 *
 * None of the software comparisons is gated on whether the amplifier is
 * currently keyed.  The reasoning is in protection_check_adc1_dma(): an
 * idle amplifier can be struck by a stray carrier, can be connected to
 * the wrong antenna, or can have a failing LPF, and in every one of those
 * cases the readings that would give it away are valid measurements -
 * with no legitimate signal in flight to blame them on.  The single
 * switch that governs the software side is g_config.allow_protection;
 * the hardware watchdogs, the comparators and PB12 act on a fault
 * regardless of any software flag.
 *
 * Shutdown sequence (identical from every source, ISR-safe):
 *   1. RF_IN_PROTECT_RAISE  (tell the RF board to protect, PB13) - first, so
 *      the RF deck starts latching its hardware bus at the earliest moment
 *   2. RELAY1_OFF + RELAY2_OFF + PTT_OUT low + ptt_state = IDLE
 *   3. PWR_ON_DISABLE       (cut high-voltage supply to PA)
 *   4. RF_BIAS_DISABLE      (cut PA bias)
 *   5. buzzer_start()       (3-tone warning, loops until cleared)
 *   6. g_alert_active / g_alert_flag, g_allow_rf = false, g_tx_active = false
 *
 * The latch holds until protection_clear(), which the GUI agent calls on
 * popup dismiss and the Networking agent calls on a CAT / SPI6 command.
 *
 * protection_check() runs the same comparisons on the protect task's tick
 * as a backstop for anything the fast paths miss.  g_config.allow_protection
 * == 0 short-circuits all three software comparison functions before they do
 * any work; protection_trigger_locked() carries the same guard for the ISR
 * and EXTI sources.
 */

#include "protection.h"
#include "power_control.h"
#include "buzzer.h"
#include "adc_calibration.h"
#include "user_adc.h"    /* adc_stabilisation_done() - comparator bring-up gate */
#include "config.h"
#include "comm.h"        /* comm_post_alert_evt() */
#include "stm32h7xx_hal.h"

/*============================================================================
 *  Protection limits
 *
 *  NO HARD-CODED THRESHOLDS.  Every limit below is read from g_config,
 *  which is the single source of truth shared by the GUI (Settings
 *  menu -> Protection page), the CAT/serial link, the flash-backed
 *  config blob and this file.  Units:
 *
 *      power    g_config in W,   comparisons in mW  -> * 1000
 *      current  g_config in mA,  comparisons in mA  -> as-is
 *      SWR      g_config in x100                     -> as-is
 *      NTC      g_config in 0.1 C                    -> as-is
 *
 *  The two channels with a hardware watchdog behind them (IMain, REV1)
 *  additionally get their threshold pushed into the AWD registers by
 *  boot_apply_thresholds(); the software comparison below is the
 *  second line of defence for everything that has no AWD.
 *
 *  Read pattern: each field is read once into a local.  All of them are
 *  word-size, so the read is atomic on Cortex-M7 and does not need the
 *  config mutex (see the concurrency rules in config.h - a single
 *  field read is allowed unlocked).
 *==========================================================================*/

/*! Effective software limits for this evaluation, snapshotted from g_config. */
typedef struct {
    int32_t rev2_max_mW;
    int32_t rev2_swr_mW;
    int32_t swr_max_x100;
    int32_t fwd1_max_mW;
    int32_t fwd2_max_mW;
    int32_t ntc_max_tenth_c;
} protect_limits_t;

/**
 * @brief Snapshot the operator-configured limits out of g_config.
 *
 * Called at the top of every protection predicate.  Grouping the reads
 * into one helper keeps the unit conversions in a single place and
 * makes it obvious at a glance that nothing is hard-coded any more.
 */
static void protect_limits_get(protect_limits_t *lim)
{
    lim->rev2_max_mW    = (int32_t)g_config.band_rev2_max_w      * 1000L;
    lim->rev2_swr_mW    = (int32_t)g_config.band_rev2_swr_w      * 1000L;
    lim->swr_max_x100   = (int32_t)g_config.band_swr_max_x100;
    lim->fwd1_max_mW    = (int32_t)g_config.band_fwd1_max_w      * 1000L;
    lim->fwd2_max_mW    = (int32_t)g_config.band_fwd2_max_w      * 1000L;
    lim->ntc_max_tenth_c = (int32_t)g_config.band_ntc_max_tenth_c;
}

/*============================================================================
 *  State (volatile for ISR/task sharing)
 *============================================================================*/

static volatile bool     s_protection_active = false;
static volatile AlertType s_last_fault        = ALERT_NONE;
static volatile AlertSeverity s_last_severity = ALERT_SEV_NONE;

/*!< One bit per ProtectionSource, set the moment that source's comparison
 *   passes and never cleared until power-on.  Volatile because it is stamped
 *   from ISR context and read by the GUI task.
 *
 *   A single word OR is the whole update, and it is atomic on Cortex-M: an
 *   interrupt that fires halfway through cannot tear it, and the only
 *   concurrency here is "two sources at once" - where losing one stamp
 *   would be wrong but a lost UPDATE is not possible anyway, since the
 *   interrupted context re-executes its own store afterwards. */
static volatile uint32_t s_source_fired = 0U;

/*!< Set by protection_clear() (GUI / comm task), consumed by the protection
 *   task.  Volatile because the two live in different contexts. */
static volatile bool     s_latch_clear_requested = false;

/*!< True while the digital fault lines (PB12 HARDPWR_ERR and the other
 *   EXTI fault inputs) are masked off, i.e. between power-on and the end
 *   of the ADC settling window.
 *
 *   WHY THEY ARE MASKED AT ALL
 *   The power board's fault line comes up with the rest of the supply, so
 *   during the settling window it can still be asserting - or can pulse
 *   once as the rail comes up.  Neither is a real fault, but a P0 fault
 *   is expensive: it cuts the HV rail, drops the relays, starts the 3-tone
 *   buzzer and latches a popup the operator has to dismiss.  All of that
 *   during the first 100 ms of every cold boot, for a supply that is about
 *   to come up healthy.
 *
 *   The mask is per EXTI LINE, not per NVIC IRQ: PB12 (line 12) and PF15
 *   (line 15, the backup-rail monitor) share EXTI15_10_IRQn, so masking the
 *   whole IRQ would take the backup monitor down with it.  EXTI->IMR1 is
 *   the per-line interrupt mask, and that is what is used here.
 *
 *   The pending flag is cleared when the mask goes on, so an edge that
 *   arrives while masked does not fire the instant the mask comes off -
 *   by then the rail has settled and the line is either still asserted
 *   (a real fault, and the level check in the handler catches it) or it
 *   has gone back low (a startup transient, correctly forgotten). */
static volatile bool     s_digital_faults_masked = true;

/*!< True while the RF-deck fault latch (PB13) is driven high, i.e. a P0 has
 *   actually latched hardware protection and a dismiss still owes the deck a
 *   reset pulse.  P1 never drives that pin, so it must never schedule the
 *   pulse - otherwise dismissing a temperature warning would drop RF_BIAS
 *   for 100 ms on a perfectly healthy amplifier.
 *   Volatile for the same reason as the flag above: written from the ISR
 *   trigger path, read from the GUI/comm clear path. */
static volatile bool     s_hw_latch_engaged = false;

/*!< Dismiss is deliberately NOT instantaneous, and this is what makes it
 *   work at all.  The comparisons run in interrupt context - the ADC1
 *   DMA-completion interrupt fires several times per millisecond - and
 *   nothing about the clear path changes a reading.  So clearing the latch
 *   outright lets the very same fault re-latch microseconds later: the popup
 *   reappears and the buzzer restarts before the operator's finger has left
 *   the glass.  Tapping Dismiss would look like a frozen panel.
 *
 *   So the clear path arms this instead, and protection_trigger_locked()
 *   refuses to re-latch while it is set.  Wall clock in ms, so it is
 *   readable from the interrupt paths too.
 *
 *   Deliberately SHORT.  It only has to outlast one GUI round trip - long
 *   enough that the popup is really gone and a stray second tap cannot
 *   re-latch either.  It is not a "condition gone" detector, because it
 *   must not be: a long hold-off would also mask a DIFFERENT fault that
 *   arrives inside the window, and silently ignoring a real overdrive or
 *   over-current for seconds is not an acceptable trade for a UI that
 *   behaves.  Past this deadline the checks are live again, so a condition
 *   that genuinely persists latches a second time, visibly and with a fresh
 *   buzzer cycle - which is the correct outcome for something that has not
 *   gone away. */
#define PROTECTION_REARM_HOLD_MS   400U
static volatile uint32_t s_clear_holdoff_ms = 0U;

/*!< The fault the operator just dismissed.  The hold-off applies to THIS
 *   cause only: a different fault arriving inside the window is a different
 *   event and must latch immediately - a real overdrive or over-current is
 *   an emergency, and masking it because the operator acknowledged something
 *   else a moment ago would be indefensible.  ALERT_NONE means no hold-off
 *   is armed. */
static volatile AlertType s_clear_holdoff_cause = ALERT_NONE;

/*============================================================================
 *  CLEAR_FAULTX_OC - who owns the pin
 *
 *  The pin is open-drain into the external fault latch reset network, and
 *  three different mechanisms want it low:
 *
 *    1. the IM inrush mute (protection_mask_im_high) - the surge must not
 *       leave the RF deck's latch closed;
 *    2. the timed clear service (protection_clear_fault_safe) - the
 *       generic "clear the latch now" request;
 *    3. the blocking dismiss pulse (protection_fault_clear_hold) - the
 *       operator acknowledged a P0, and the reset has to take effect
 *       immediately.
 *
 *  One pin with three drivers is how a pulse ends early and leaves the
 *  latch half-cleared, so the pin has ONE owner: a bit mask of who is
 *  holding it low.  The line is driven low when the mask goes from 0 to
 *  non-zero, and back high only when the last holder lets go.  A timed
 *  pulse that ends while the dismiss pulse is still running therefore
 *  leaves the line low, which is exactly right.
 *
 *  The mask is a single byte, so a read-modify-write is atomic enough on
 *  Cortex-M7 for the two contexts that touch it (the monitor task and the
 *  protect task).  The IM mute is armed from power_hv_on() and released
 *  from the same monitor task, so the three never actually race.
 *==========================================================================*/

/*!< Bit set while the IM inrush mute holds the line low. */
#define CLEAR_HOLD_IM_MUTE   0x01U
/*!< Bit set while the monitor-task clear service holds the line low. */
#define CLEAR_HOLD_SERVICE   0x02U
/*!< Bit set while the blocking dismiss pulse holds the line low. */
#define CLEAR_HOLD_BLOCKING  0x04U

/*!< Who currently holds CLEAR_FAULTX_OC low. */
static volatile uint8_t  s_clear_holders = 0U;

/*! Drive the line low if this is the first holder. */
static void clear_pin_hold(uint8_t owner_bit)
{
    uint8_t before = s_clear_holders;
    s_clear_holders = (uint8_t)(before | owner_bit);
    if (before == 0U) {
#ifndef LVGL_SIM
        CLEAR_FAULTX_OC_LOW();
#endif
    }
}

/*! Let one holder go; the line goes high only when nobody holds it any
 *  more. */
static void clear_pin_release(uint8_t owner_bit)
{
    uint8_t after = (uint8_t)(s_clear_holders & (uint8_t)~owner_bit);
    s_clear_holders = after;
    if (after == 0U) {
#ifndef LVGL_SIM
        CLEAR_FAULTX_OC_HIGH();
#endif
    }
}

/*============================================================================
 *  IM (main current) mute - the HV inrush window
 *
 *  Raising the high-voltage rail charges the supply's bulk capacitors, and
 *  the current that flows while they charge looks exactly like a short
 *  circuit to the current protection: AWD2 and COMP2 watch that node and
 *  both would trip.  A trip on the way UP to standby is nonsense - the
 *  amplifier has not even been keyed yet.
 *
 *  So it is one flag, and nothing else:
 *
 *      power_hv_on()          -> protection_mask_im_high()  (before PWR_ON)
 *      app_monitor_task()     -> protection_im_mask_tick()  (every tick)
 *      AWD2 / COMP2 handlers  -> protection_im_masked()     (read only)
 *
 *  The window also holds CLEAR_FAULTX_OC low: the inrush can latch the RF
 *  deck's hardware fault bus, and a latch that outlives the transient would
 *  keep the amplifier cut for a fault that never existed.  The clear line is
 *  asserted with the mute and released when the window closes.
 *
 *  A volatile bool store is atomic on Cortex-M7, so arming and testing need
 *  no lock, and the ISR side costs exactly one read with no FreeRTOS call -
 *  which is what makes it legal at main priority 0.
 *
 *  The duration is measured from the tick recorded when the mute was armed
 *  rather than accumulated by the pump task, so a task that arrives late
 *  still releases on time instead of extending the window by its own delay.
 *==========================================================================*/

/*!< True while the main-current protection is being ignored. */
static volatile bool s_im_masked = false;

/*!< Tick at which the mute was armed; the pump task counts from here. */
static volatile uint32_t s_im_mask_tick = 0U;

/**
 * @brief Mute the main-current protection for the HV inrush window.
 *
 *  Called from power_hv_on() BEFORE PWR_ON is asserted: the comparator can
 *  see the surge on the very first millisecond, so the flag must not be a
 *  step behind the switch.
 *
 *  Also holds the external fault latch CLEAR_FAULTX_OC low for the whole
 *  window.  The inrush can latch the RF deck's hardware bus before or while
 *  the firmware is ignoring the trip, and a latch that survives the event
 *  would leave the amplifier cut for a fault that was never real - so the
 *  clear line is asserted here and released in protection_im_mask_tick()
 *  when the window closes.
 *
 *  The reference tick is published first and the flag last, so an interrupt
 *  that observes the mute always finds a valid start time.
 *
 *  Task context.  The stores are ISR-safe (single words), but nothing in
 *  the firmware calls this from an interrupt handler.
 */
void protection_mask_im_high(void)
{
    clear_pin_hold(CLEAR_HOLD_IM_MUTE);
    s_im_mask_tick = HAL_GetTick();
    s_im_masked    = true;
}

/**
 * @brief Is the main-current protection muted right now?
 *
 *  ISR-safe: one volatile read, no locks, no FreeRTOS calls.  Called from
 *  protection_awd2_cb() and protection_comp2_isr().
 */
bool protection_im_masked(void)
{
    return s_im_masked;
}

/**
 * @brief Release the mute once its hold window has elapsed.
 *
 *  Called by the monitor ("pump") task on every tick - that task is the
 *  only place that clears the flag.  No-op while no mute is armed, and a
 *  no-op until IM_MASK_HOLD_MS has passed.
 *
 *  Releasing also puts CLEAR_FAULTX_OC back high, undoing the hold that
 *  protection_mask_im_high() asserted: whatever the inrush latched has had
 *  the whole window to clear, and the latch bus goes back to normal.
 *
 *  Order matters: the software flag is cleared BEFORE the clear line is
 *  released, so the firmware is listening again by the time the external
 *  latch can set.  The other way round would leave a sliver in which the
 *  hardware can latch a trip that nothing responds to.
 *
 *  The window is measured from the tick recorded when the mute was armed
 *  rather than accumulated here, so a task that arrives late still releases
 *  on time instead of extending the mute by its own delay.  Unsigned
 *  subtraction is wrap-safe: the window is orders of magnitude shorter than
 *  half the tick counter's range.
 */
void protection_im_mask_tick(void)
{
    if (!s_im_masked) {
        return;
    }
    if ((uint32_t)(HAL_GetTick() - s_im_mask_tick) >= IM_MASK_HOLD_MS) {
        s_im_masked = false;
        clear_pin_release(CLEAR_HOLD_IM_MUTE);
    }
}

/*============================================================================
 *  Timed clear of the external fault latch
 *
 *  CLEAR_FAULTX_OC (PG1) is open-drain into the latch reset network.
 *  Holding it low is what actually clears the latch; left floating, the
 *  latch stays closed however much the firmware thinks it dismissed the
 *  fault.
 *
 *  The caller cannot do the pulse itself: it has to hold the pin low for a
 *  while, and the places that need it are interrupt handlers (a protection
 *  trip) and command handlers (CAT / SPI6).  Neither may block, and neither
 *  may call a FreeRTOS delay.
 *
 *  So the request is a FLAG, and the monitor ("pump") task is the
 *  HEARTBEAT that drives the pin:
 *
 *      caller (ISR or task) -> protection_clear_fault_safe(ms)
 *                                 |  flag + hold time, no blocking
 *                                 v
 *      monitor task           -> protection_fault_clear_service()
 *                                 |  pulls the line low, arms the deadline
 *                                 |  releases it when the time is up
 *
 *  ISR-safe: two volatile word stores, no lock, no FreeRTOS API.  The hold
 *  time is written BEFORE the request flag, so a request that lands while
 *  the service is running is never seen with a stale duration.
 *
 *  WHY THE MONITOR TASK AND NOT A TIMER
 *
 *  The monitor task is already the heartbeat of the slow half of the
 *  firmware, it owns no delay-critical work, and it is the task that
 *  already drives the HV rail sequencing.  A hardware timer would need an
 *  interrupt to do exactly the same thing that one flag and one existing
 *  task tick already do.
 *
 *  RF_BIAS IS NOT TOUCHED
 *
 *  The dismiss path (protect_task_latch_clear_pulse() in app_tasks.c) also
 *  drops the PA bias, because there the PA really did fault and an RF-side
 *  event must not re-latch the fault the instant the reset takes effect.
 *  This API is generic: whoever asks for it may be asking for a latch clear
 *  while the amplifier is keyed, and cutting the bias would unkey it.  The
 *  caller that needs the bias dropped asks for that separately.
 *
 *  ONE OWNER FOR THE PIN
 *
 *  The pin is shared with the IM inrush mute and with the blocking dismiss
 *  pulse, so all three go through the same holder mask - see "CLEAR_FAULTX_OC
 *  - who owns the pin" above.
 *==========================================================================*/

/*!< True when a clear has been requested and the service has not started
 *   it yet. */
static volatile bool     s_fault_clear_pending = false;
/*!< Requested hold time of the pending pulse, milliseconds. */
static volatile uint32_t s_fault_clear_ms      = 0U;
/*!< Tick at which the service's pulse ends. */
static volatile uint32_t s_fault_clear_deadline = 0U;

/**
 * @brief Request a timed clear of the external fault latch.
 *
 *  ISR-safe and task-safe.  Nothing is written to the pin here: the request
 *  is recorded and the monitor task drives the pin on its next pass, so the
 *  caller never blocks and never touches a GPIO.
 *
 *  @param milliseconds How long the clear line should be held low.  0 is
 *         treated as 1 ms: a zero-length pulse would not give the latch
 *         network any edge to react to.
 */
void protection_clear_fault_safe(uint32_t milliseconds)
{
    s_fault_clear_ms      = (milliseconds > 0U) ? milliseconds : 1U;
    s_fault_clear_pending = true;
}

/**
 * @brief Drive the fault-latch clear pulse.  MONITOR TASK ONLY.
 *
 *  One call does one step of the pulse:
 *    - a pending request pulls CLEAR_FAULTX_OC low and arms the deadline;
 *    - an expired pulse puts the line back high.
 *
 *  Task context: it writes a GPIO, which is fine from a task, and it must
 *  not be called from an interrupt handler (the caller there uses
 *  protection_clear_fault_safe() instead).
 */
void protection_fault_clear_service(void)
{
    if (s_fault_clear_pending) {
        /* Clear the flag BEFORE the pin work, so a request that arrives
         * during this call is picked up on the next pass instead of being
         * swallowed by this one's cleanup. */
        s_fault_clear_pending = false;
        s_fault_clear_deadline = HAL_GetTick() + s_fault_clear_ms;
        clear_pin_hold(CLEAR_HOLD_SERVICE);
        return;
    }

    if ((s_clear_holders & CLEAR_HOLD_SERVICE) != 0U) {
        /* Unsigned subtraction is wrap-safe: the deadline being behind us
         * gives a small difference, and 0x80000000 marks the halfway point
         * of the tick counter. */
        if ((uint32_t)(HAL_GetTick() - s_fault_clear_deadline) < 0x80000000U) {
            clear_pin_release(CLEAR_HOLD_SERVICE);
        }
    }
}

/**
 * @brief Hold the clear line low from the caller's own context.
 *
 *  The blocking dismiss path needs the line low NOW - it is dropping the
 *  PA bias for the same window, and a request flag would make it wait for
 *  the monitor task's next tick.  This is the counterpart of
 *  protection_clear_fault_safe() for callers that can afford to hold the
 *  line themselves.
 *
 *  Task context (it writes a GPIO).
 */
void protection_fault_clear_hold(void)
{
    clear_pin_hold(CLEAR_HOLD_BLOCKING);
}

/**
 * @brief Release a hold taken by protection_fault_clear_hold().
 *
 *  The line only goes back high when nobody holds it any more, so a timed
 *  pulse running at the same time cannot be cut short by this.
 *
 *  Task context.
 */
void protection_fault_clear_release(void)
{
    clear_pin_release(CLEAR_HOLD_BLOCKING);
}

/**
 * @brief How much longer the active timed pulse runs, in milliseconds.
 *
 *  The monitor task uses this to shorten its wait while a pulse is in
 *  flight, so the line is released when the caller asked rather than a
 *  whole task tick later.  0 means "nothing running".
 */
uint32_t protection_fault_clear_remaining_ms(void)
{
    if ((s_clear_holders & CLEAR_HOLD_SERVICE) == 0U) {
        return 0U;
    }
    uint32_t now = HAL_GetTick();
    /* Unsigned subtraction is wrap-safe: a deadline ahead of now gives a
     * large difference (>= 0x80000000), a deadline behind us a small one.
     * So the large difference is the "still running" case, and what is
     * left of the window is the deadline minus now. */
    if ((uint32_t)(now - s_fault_clear_deadline) >= 0x80000000U) {
        return s_fault_clear_deadline - now;
    }
    return 0U;   /* expired */
}

/*============================================================================
 *  Initialisation
 *==========================================================================*/

void protection_init(void)
{
    s_protection_active = false;
    s_last_fault        = ALERT_NONE;
    s_last_severity     = ALERT_SEV_NONE;
    s_source_fired      = 0U;
    g_alert_active      = ALERT_NONE;
    g_alert_flag        = false;
    g_alert_severity    = ALERT_SEV_NONE;
    /* Idle state: no fault latched, so the RF board must be told "no fault".
     * Doing this once here also means a protection_clear() before any fault
     * cannot leave PB13 in whatever state the pin came up in. */
    RF_IN_PROTECT_CLEAR();
    buzzer_stop();
    /* No dismiss is pending at boot.  Without this a request that somehow
     * survived a soft restart would fire a spurious 100 ms bias cut. */
    s_latch_clear_requested = false;
    s_hw_latch_engaged = false;
    s_clear_holdoff_ms   = 0U;
    s_clear_holdoff_cause = ALERT_NONE;
    /* Power-on state: no inrush mute armed.  The only path that raises it
     * is power_hv_on(), and this runs before the rail is ever touched. */
    s_im_masked    = false;
    s_im_mask_tick = 0U;
    /* NOTE: s_digital_faults_masked is deliberately NOT reset here.
     * protection_init() runs from the protect task, which starts after the
     * scheduler does - by then main() has already called
     * protection_hold_digital_faults(), and the lines must stay masked
     * until the ADC settling window ends.  The static initialiser puts it
     * in the masked state, and only protection_arm_digital_faults() clears
     * it. */
}

/*============================================================================
 *  Atomic shutdown (ISR-safe, no FreeRTOS API)
 *
 *  P0 (emergency): 7-step hard shutdown.  RF board notified, relays open,
 *      PWR_ON + RF_BIAS cut, buzzer 3-tone 10 s, g_allow_rf latched low.
 *      Dismiss re-arms g_allow_rf.
 *  P1 (mild, e.g. NTC overheat): nothing on the RF side except refusing
 *      PTT.  PWR_ON / RF_BIAS stay up, RELAYs stay where they were
 *      (IDLE), the RF board protection bus is not driven, and g_allow_rf
 *      is untouched - the IMain zero offset is unaffected by PA
 *      temperature, and the operator should be able to key again as soon
 *      as they dismiss.  Only the PTT gate (g_alert_flag) blocks keying.
 *==========================================================================*/

void protection_trigger_locked(AlertType cause)
{
    /* Master kill switch (g_config.allow_protection == 0).  Placed here, at
     * the single chokepoint every source funnels through, so one gate covers
     * the EXTI handlers, the AWD callbacks, the DMA/task comparisons and the
     * digital fault line alike - there is no second door to leave open.
     *
     * Returning here suppresses EVERYTHING the firmware would otherwise do
     * about the fault, which is what "protection off" has to mean:
     *   - the shutdown sequence (relays, PWR_ON, RF_BIAS, RF_IN_PROTECT)
     *   - the buzzer, so no tone announces a fault nothing is doing about
     *   - the latch and g_alert_flag, so gui_show_alert() is never reached
     *     and no popup appears
     *   - the alert= event, so the CAT link carries no protection traffic
     *     and the host sees the same silence the front panel does
     *
     * What this does NOT - and cannot - suppress: the hardware itself.  The
     * ADC analog watchdogs (AWD1 RF input, AWD2 main current, AWD3 coupler
     * reverse), the COMP1 / COMP2 comparators and the PB12 hardware-fault
     * input all act on a fault regardless of what this MCU decides, because
     * they exist precisely so that a hung or crashed controller cannot leave
     * the amplifier running.  Pushing those thresholds out of the way is a
     * separate, deliberate operation (boot_apply_thresholds), not something
     * this flag should do behind the operator's back.
     *
     * The re-entry guard sits below this one on purpose: a fault that was
     * already latched when the switch was thrown keeps its latch, so
     * protection_clear() still behaves normally and the operator can get
     * back to a known state instead of having a stale popup they cannot
     * dismiss. */
    if (g_config.allow_protection == 0U) {
        return;
    }

    /* Prevent re-entry: only first trigger wins, until cleared. */
    if (s_protection_active) {
        return;
    }

    /* A dismiss that has not been given time to take effect yet.  The fault
     * is real and still present - the operator is not wrong about the
     * hardware - but re-latching it here would defeat the dismiss entirely:
     * the comparisons run from the ADC interrupts, so the popup and the
     * buzzer would be back microseconds after the operator dismissed them,
     * with no way to tell the two states apart.
     *
     * Scoped to the dismissed cause on purpose.  A DIFFERENT fault arriving
     * inside the window is a different event and falls straight through:
     * masking a real overdrive or over-current because the operator
     * acknowledged something else a moment ago is not a trade worth making.
     * Once the deadline passes the hold-off lifts entirely, so a condition
     * that never went away latches a second time - visibly, with a fresh
     * buzzer cycle, which is right for something that is still there. */
    if (s_clear_holdoff_ms != 0U) {
        /* Deadline still in the future?  Unsigned subtraction is wrap-safe:
         * a deadline ahead of now gives a large difference (>= 0x80000000),
         * a deadline behind us a small one. */
        if ((uint32_t)(HAL_GetTick() - s_clear_holdoff_ms) >= 0x80000000U) {
            if (cause == s_clear_holdoff_cause) {
                return;                 /* same fault, still inside the hold-off */
            }
        } else {
            s_clear_holdoff_ms   = 0U;   /* deadline passed - fault is live */
            s_clear_holdoff_cause = ALERT_NONE;
        }
    }

    s_protection_active = true;
    s_last_fault        = cause;
    s_last_severity     = alert_severity(cause);

    if (s_last_severity == ALERT_SEV_P0) {
        /* 1. raise hardware fault latch FIRST (PB13 -> D7 ->
         *    RF_IN_PROTECT_MIXED) so the RF board starts its own
         *    protection and latches the hardware bus at the earliest
         *    possible moment.  Every other step below is local to this
         *    MCU and can afford to follow. */
        RF_IN_PROTECT_RAISE();
        s_hw_latch_engaged = true;
        /* 2. drop both RELAYs and the keying feedback immediately.
         *    RF may be live at the antenna port. */
        RELAY1_OFF();
        RELAY2_OFF();
        /* 3. cut high voltage. */
        PWR_ON_DISABLE();
        /* 4. cut PA bias. */
        RF_BIAS_DISABLE();
        /* 5a. Abandon any HV sequencing that was in flight.  This path
         *     drops PWR_ON / RF_BIAS directly with zero delay - deliberately
         *     not via power_hv_off(), whose 10 ms bias-before-rail gap
         *     would be the wrong thing to do to a fault.  The pending
         *     stage therefore has to be cleared, or power_rail_step()
         *     would later re-assert a rail that protection just cut. */
        power_rail_abort();
        /* 5. start the 3-tone warning.  Hard-capped at 10 s by buzzer.c;
         *    the operator is never forced to dismiss to silence the room. */
        buzzer_start();
        /* 7. block new PTT until the operator dismisses.  g_allow_rf is
         *    the one knob ptt_apply_level() / power_tx_enable() check. */
        g_allow_rf = false;
    } else {
        /* P1: thermal / fan / pump.  Refuse PTT, leave everything else
         * alone.  The PA is still biased and ready to go as soon as the
         * condition clears. */
    }
    /* PTT_OUT low + ptt_state IDLE applies to BOTH severities - it is the
     * "stop keying right now" half of the gate, even if P1 lets the
     * hardware stay armed.
     *
     * Routed through power_ptt_force_release_isr() rather than writing
     * ptt_state here: g_power_state is only published by ptt_leave_tx(), so
     * a direct assignment left the top banner saying "Transmit" after a P0
     * shutdown.  That is the single writer rule for g_power_state, and it
     * has to hold on the fault path too. */
    power_ptt_force_release_isr();

    /* 5b. start the P1 warning tone.  Hard-capped at 10 s. */
    if (s_last_severity == ALERT_SEV_P1) {
        buzzer_start_p1();
    }

    /* 6. publish to cross-task state (volatiles already declared
     *    volatile in opa2000.c; reads on task side are atomic on M7).
     *    g_alert_flag is what makes the GUI task call gui_show_alert(),
     *    so the popup is suppressed simply by never setting it. */
    g_alert_active   = cause;
    g_alert_severity = s_last_severity;
    g_alert_flag     = true;
    /* NOTE: g_allow_rf was already cleared above for P0; repeating it here
     * would say nothing new.  P1 leaves it alone so the same CC6920
     * zero-offset calibration continues to gate keying as normal. */
    g_tx_active = false;
    /* NOTE: g_hv_on and g_psu_on are deliberately left alone by both
     * severities - a fault never silently powers the rig off; the
     * operator has to do that explicitly.  P0 additionally cuts the
     * HV rail so the PA cannot produce more RF or heat while latched;
     * P1 leaves the rail up so the PA stays warm and ready.  What
     * actually blocks RF is g_allow_rf (P0) or g_alert_flag (both). */

    /* 8. Push the alert=ON event onto the LPUART1 comm ring so the host
     *    sees it on the next pump, without waiting for the 100 ms status
     *    cycle.  comm_post_alert_evt() is ISR-safe (no FreeRTOS API). */
    comm_post_alert_evt(cause, s_last_severity, true);
}

/*============================================================================
 *  Digital sources - EXTI interrupt context
 *==========================================================================*/

/* Lines to mask while the supply is still settling.  12 is PB12
 * (HARDPWR_ERR, the power board's fault line); 15 is PF15
 * (BACKUP_POWEREDX, the backup-rail monitor) and rides on the same
 * EXTI15_10_IRQn, so it is masked together with it. */
#define DIGITAL_FAULT_EXTI_LINES   ((1UL << 12) | (1UL << 15))

void protection_hold_digital_faults(void)
{
    /* EXTI->IMR1 is the per-line interrupt mask: clearing a bit stops that
     * line from reaching the NVIC, while the OTHER lines on the same
     * EXTI15_10_IRQn keep working.  Disabling the whole NVIC IRQ instead
     * would take the backup-rail monitor down with the fault line. */
    EXTI->IMR1 &= ~DIGITAL_FAULT_EXTI_LINES;

    /* Drop anything that latched while the line was still armed, so the
     * mask coming off later does not immediately fire a stale edge. */
    EXTI->PR1 &= ~DIGITAL_FAULT_EXTI_LINES;

    s_digital_faults_masked = true;
}

void protection_arm_digital_faults(void)
{
    /* Re-enable the lines.  The pending flag was cleared when the mask went
     * on, so nothing fires from here - but a fault that is STILL asserted
     * (the rail did not come up) would not produce a new edge, because the
     * line never fell.  So the level is checked explicitly below. */
    EXTI->IMR1 |= DIGITAL_FAULT_EXTI_LINES;
    EXTI->PR1 &= ~DIGITAL_FAULT_EXTI_LINES;

    s_digital_faults_masked = false;

    /* A line that is still high when the mask comes off is a real fault,
     * not a startup transient - the supply had its chance to settle and
     * it did not.  Check it now, because no new edge will ever arrive. */
    if (HAL_GPIO_ReadPin(HARDPWR_ERR_GPIO_Port, HARDPWR_ERR_Pin)
        == GPIO_PIN_SET) {
        protection_note_source(PROT_SRC_HARDERR);
        protection_trigger_locked(ALERT_HARDWARE_FAULT);
    }
}

void protection_digital_isr(AlertType cause)
{
    /* A masked line must not latch a fault.  The mask is per line, so this
     * check is the only thing standing between a supply that is still
     * coming up and a P0 shutdown with a 10 s buzzer on it. */
    if (s_digital_faults_masked) {
        return;
    }
    /* Stamped before the trigger, and outside it: a masked-out line is not
     * a fault, but an UNMASKED line that reaches here while another fault
     * holds the latch is exactly the case the history exists to expose. */
    protection_note_source(PROT_SRC_HARDERR);
    protection_trigger_locked(cause);
}

void protection_hardpwr_err_isr(void)
{
    /* Masked during the power-on settling window - see
     * protection_hold_digital_faults().  The supply is not up yet, so
     * whatever this line says about it is not information about a fault. */
    if (s_digital_faults_masked) {
        return;
    }

    /* HARDERR_PWR (PB12) is ACTIVE HIGH: only act while the line really is
     * asserted, so a spurious edge cannot shut the amplifier down. */
    if (HAL_GPIO_ReadPin(HARDPWR_ERR_GPIO_Port, HARDPWR_ERR_Pin)
        == GPIO_PIN_SET) {
        protection_note_source(PROT_SRC_HARDERR);
        protection_trigger_locked(ALERT_HARDWARE_FAULT);
    }
}

void protection_comp1_isr(void)
{
    /* Backstop for the comparator bring-up window.
     *
     * COMP1 fires straight from the comparator IRQ into
     * protection_trigger_locked() - there is no software gate between the
     * analog edge and the P0 shutdown.  adc_stabilisation_tick() keeps the
     * reference parked at the VDDA rail until the settling window closes, so
     * an edge in here can only be a real fault... unless the reference was
     * programmed early by something that did not know about the window.
     *
     * This guard is what makes that failure mode harmless: during the
     * window the comparators are the only protection still missing, and a
     * spurious edge there costs nothing worth keeping.  It is deliberately
     * NOT the mechanism that prevents the false alarm (the parked reference
     * is) - it is the belt to that braces, and it also covers the reverse
     * case where a comparator is somehow enabled before the whole power-on
     * sequence has run. */
    if (!adc_stabilisation_done()) {
        return;
    }

    /* COMP1 = RF input overdrive / over-power (PB2 / ADC1_INP4).
     *
     * Stamped after the bring-up guard, not before: an edge inside the
     * settling window is the reference still ramping, not a fault, and
     * recording it would point the operator at a comparator that never
     * actually saw an overdrive. */
    protection_note_source(PROT_SRC_COMP1);
    protection_trigger_locked(ALERT_OVERDRIVE);
}

void protection_comp2_isr(void)
{
    /* Same bring-up guard as COMP1 above - see the comment there. */
    if (!adc_stabilisation_done()) {
        return;
    }

    /* HV inrush window: the rail is charging its bulk capacitors and the
     * transient on this node is expected, not a fault.  The flag is raised
     * by power_hv_on() before PWR_ON closes and released by the pump task
     * IM_MASK_HOLD_MS later.
     *
     * Deliberately BEFORE protection_note_source(): a muted trip is not a
     * fault event, and recording it would make the "which source has fired"
     * history point at the current protection for what was a normal
     * power-up. */
    if (protection_im_masked()) {
        return;
    }

    /* COMP2 = main current over-current (PE9 / ADC1_INP5). */
#ifndef NO_CURRENT_HIGH
    protection_note_source(PROT_SRC_COMP2);
    protection_trigger_locked(ALERT_CURRENT_HIGH);
#else
    /* Reminder that belongs to the branch that actually removes the trip.
     * #warning is a GCC/Clang extension - MSVC (the host test build) has no
     * such directive and fails on it, so it is compiled out there. */
#if !defined(_MSC_VER)
#warning "NO CURRENT HIGH: the main-current trip is compiled out"
#endif
#endif
}

/*============================================================================
 *  Analog sources with a hardware watchdog - ADC IRQ context
 *==========================================================================*/

void protection_awd1_cb(void)
{
    /* AWD1 -> CH4 RF input power over band-dependent threshold.
     * ioc default AWD1HighThreshold = 32767 -> set to band threshold by
     * adc_awd_set_rf_in_mW() before TX starts.
     *
     * COMP1 watches the SAME node, so the two stamps are deliberately
     * separate: which of them fires is the difference between "the limit
     * is set too low" and "the analog front end is overdriving". */
    protection_note_source(PROT_SRC_AWD1);
    protection_trigger_locked(ALERT_OVERDRIVE);
}

void protection_awd2_cb(void)
{
    /* AWD2 -> CH5 main current, the hardware line behind
     * g_config.band_im_max_mA (programmed by boot_apply_thresholds()).
     * Shares its node with COMP2 - see the note above.
     *
     * HV inrush window: muted for the same reason as COMP2 above - the
     * surge that charges the bulk capacitors is not an over-current. */
    if (protection_im_masked()) {
        return;
    }
#ifndef NO_CURRENT_HIGH
    protection_note_source(PROT_SRC_AWD2);
    protection_trigger_locked(ALERT_CURRENT_HIGH);
#endif
}

void protection_awd3_cb(void)
{
    /* AWD3 -> CH6 coupler 1 reverse, the hardware last line behind
     * g_config.band_rev1_max_w (programmed by boot_apply_thresholds()).
     *
     * Its own bit, not the software coupler's: REV1 is the one coupler
     * channel with NO software comparison behind it (the DMA checks cover
     * REV2 / FWD1 / FWD2 / SWR only), so AWD3 firing is the only evidence
     * that reverse power ever went out of limits.  Sharing a bit with the
     * software checks would have made that unreportable. */
    protection_note_source(PROT_SRC_AWD3);
    protection_trigger_locked(ALERT_REFLECT_POWER_HIGH);
}

/*============================================================================
 *  Analog sources without a watchdog - compared in interrupt context
 *
 *  These channels have no analog watchdog behind them, so the comparison
 *  happens in the DMA / injected-completion interrupt, right after the
 *  fresh conversion results have been published to g_adc_data.
 *==========================================================================*/

void protection_check_adc1_dma(void)
{
    /* Already latched - nothing left to decide. */
    if (s_protection_active) {
        return;
    }

    /* Same master switch as protection_check().  This one runs in the ADC1
     * DMA completion interrupt, i.e. several times per millisecond, so the
     * snapshot copy below is worth skipping outright. */
    if (g_config.allow_protection == 0U) {
        return;
    }

    /* Snapshot the operator-configured limits once. */
    protect_limits_t lim;
    protect_limits_get(&lim);

    /* Coupler 2 reverse, the reading the operator actually sees. */
    if (g_adc_data.rev2_mW > lim.rev2_max_mW) {
        protection_note_source(PROT_SRC_COUPLER);
        protection_trigger_locked(ALERT_REFLECT_POWER_HIGH);
        return;
    }
    /* Reverse power above the configured floor AND an SWR above the
     * configured ceiling: a high SWR at low power is just coupler noise. */
    if (g_adc_data.rev2_mW > lim.rev2_swr_mW &&
        g_adc_data.swr_x100 > lim.swr_max_x100) {
        protection_note_source(PROT_SRC_COUPLER);
        protection_trigger_locked(ALERT_SWR_HIGH);
        return;
    }
    /* Coupler 1 forward. */
    if (g_adc_data.fwd1_mW > lim.fwd1_max_mW) {
        protection_note_source(PROT_SRC_COUPLER);
        protection_trigger_locked(ALERT_REFLECT_POWER_HIGH);
        return;
    }
    /* Coupler 2 forward. */
    if (g_adc_data.fwd2_mW > lim.fwd2_max_mW) {
        protection_note_source(PROT_SRC_COUPLER);
        protection_trigger_locked(ALERT_REFLECT_POWER_HIGH);
        return;
    }
    /* PA bias (INP14).  Lives here rather than in protection_check() so the
     * response lands within one ADC1 DMA completion instead of waiting for
     * the Control task tick - design spec section 7.5.1 requires every
     * channel without an analog watchdog to be compared in the DMA
     * interrupt.  VBIAS shares ADC1's regular group with the coupler
     * channels, so the fresh g_adc_data.vbias_mV is already published by
     * the time this runs.
     *
     * NOT gated on the keying state.  An amplifier that is not keyed can
     * still be reflected into by a stray carrier, by a wrongly connected
     * antenna, or by a fault in the LPF itself - and the coupler and VBIAS
     * readings that would reveal exactly that are just as meaningful (in
     * fact more meaningful, because there is no legitimate signal to
     * blame them on) while the amplifier sits idle.  The limits are the
     * operator's own settings either way; a reading above them means the
     * hardware is not behaving, and that is worth reporting whether the
     * PA happens to be on the air or not. */
    if (g_adc_data.vbias_mV > g_config.band_vbias_max_mV) {
        protection_note_source(PROT_SRC_VBIAS);
        protection_trigger_locked(ALERT_VBIAS_FAULT);
    }
}

void protection_check_ntc(void)
{
    if (s_protection_active) {
        return;
    }
    /* Same master switch as protection_check(). */
    if (g_config.allow_protection == 0U) {
        return;
    }
    /* Over-temperature latches whether or not we are keyed: the PA is still
     * dissipating and the condition will not clear on its own. */
    protect_limits_t lim;
    protect_limits_get(&lim);
    if (g_adc_data.ntc_core_tenth_c > lim.ntc_max_tenth_c) {
        protection_note_source(PROT_SRC_NTC);
        protection_trigger_locked(ALERT_NTC_OVERHEAT);
    }
}

/*============================================================================
 *  Task-side software protection (task tick) - backstop
 *==========================================================================*/

void protection_check(void)
{
    /* Don't re-trigger if already latched. */
    if (s_protection_active) {
        return;
    }

    /* Master switch off: nothing below could produce an effect, because
     * every branch ends in protection_trigger_locked() and that returns
     * immediately.  Bail out here rather than after the fact so the work
     * is genuinely skipped - protect_limits_get() copies a whole snapshot
     * struct out of g_config, and this runs on every tick of the protect
     * task forever.  Compare before the latch check rather than after: the
     * latch can still be set from a fault that arrived while protection was
     * armed, and that one must still be clearable. */
    if (g_config.allow_protection == 0U) {
        return;
    }

    /* Over-temperature applies whether or not we are keyed. */
    protect_limits_t lim;
    protect_limits_get(&lim);
    if (g_adc_data.ntc_core_tenth_c > lim.ntc_max_tenth_c) {
        protection_note_source(PROT_SRC_NTC);
        protection_trigger_locked(ALERT_NTC_OVERHEAT);
        return;
    }

    /* The coupler and bias limits are NOT gated on the keying state either.
     * An idle amplifier can be struck by a stray carrier, can be connected
     * to the wrong antenna, or can have a failing LPF - and in every one of
     * those cases the coupler and VBIAS readings that would give it away are
     * valid measurements.  With the amplifier on the air there is a
     * legitimate signal those readings could be blamed on; sitting idle
     * there is not, which makes an out-of-limit reading MORE meaningful,
     * not less.  So the same operator-configured limits apply in both
     * states and this backstop runs them unconditionally. */
    protection_check_adc1_dma();

    /* VBIAS out of range. */
    if (g_adc_data.vbias_mV > g_config.band_vbias_max_mV) {
        protection_note_source(PROT_SRC_VBIAS);
        protection_trigger_locked(ALERT_VBIAS_FAULT);
    }
}

/*============================================================================
 *  Ack / clear (called from GUI or PC command after user ack)
 *============================================================================*/

void protection_clear(void)
{
    /* Read before the latch-release below clears it: this is what decides
     * whether the RF deck owes us a reset pulse. */
    bool hw_latch_was_engaged = s_hw_latch_engaged;

    /* Hold the comparisons off for a moment.  This is what makes the
     * dismiss usable at all - see the s_clear_holdoff_ms comment above.
     * Armed only when there was really something latched: a spurious
     * clear (the CAT link, a stray key) must not silently disarm the
     * protection for the next five seconds. */
    if (s_protection_active) {
        s_clear_holdoff_ms    = HAL_GetTick() + PROTECTION_REARM_HOLD_MS;
        s_clear_holdoff_cause = s_last_fault;
    }

    s_protection_active = false;
    s_last_fault        = ALERT_NONE;
    s_last_severity     = ALERT_SEV_NONE;
    g_alert_active      = ALERT_NONE;
    g_alert_flag        = false;
    g_alert_severity    = ALERT_SEV_NONE;
    /* Release the RF deck's own latch.  Keyed off whether a P0 actually
     * engaged it, not off the recorded severity, so a P1 fault can never
     * release a latch it never raised - dismissing a temperature warning
     * must not drop the bias of a healthy amplifier. */
    if (s_hw_latch_engaged) {
        RF_IN_PROTECT_CLEAR();
        s_hw_latch_engaged = false;
    }
    buzzer_stop();

    /* 主动通知 LPUART1 告警已解除。cause / sev 在 OFF 事件里忽略，
     * 但要传入 s_last_fault 的值让 PC 端知道刚解的是哪个。 */
    comm_post_alert_evt(s_last_fault, ALERT_SEV_NONE, false);
    /* Re-arm the RF path.
     *
     * g_allow_rf now means "no fault is latched", not "the zero-offset
     * calibration finished" - the calibration is a measurement step and
     * deliberately has nothing to do with the transmit permission (see
     * user_adc.c).  So a dismissed fault restores it unconditionally:
     *   P0  -> the trigger cleared it; the fault was the reason it was
     *         low, and the operator has now acknowledged it.
     *   P1  -> the trigger never touched it.
     *   NONE-> nothing was latched, nothing to restore.
     * The power-on path in adc_stabilisation_tick() is the other writer:
     * it holds g_allow_rf low until the silence is over, so the two
     * never fight - a fault can only be latched after the protection
     * chain is live, which is after that point. */
    g_allow_rf = true;

    /* Ask the protection task to pulse the external latch reset.  NOT done
     * inline on purpose: this function is called from the GUI task (popup
     * dismiss) and from the comm task (CAT / SPI6 command), and the pulse
     * has to hold two pins down for 100 ms.  Doing that here would freeze
     * LVGL input handling, or stall the CAT parser, for that long.
     *
     * Only when a P0 actually engaged the external latch.  The pulse pulls
     * RF_BIAS down as well as CLEAR_FAULTX_OC, so raising it for a fault
     * that never touched the RF deck would drop the bias of an amplifier
     * that is simply too hot - a fresh fault, with a fresh buzzer cycle,
     * seconds after the operator dismissed a temperature warning. */
    if (hw_latch_was_engaged) {
        s_latch_clear_requested = true;
    }
}

bool protection_take_latch_clear_request(void)
{
    bool requested = s_latch_clear_requested;

    /* Clear before the caller acts on it, not after.  A dismiss that lands
     * while the 100 ms pulse is still running then sets the flag again and
     * the next tick pulses once more, instead of the request being silently
     * swallowed by this one's cleanup. */
    s_latch_clear_requested = false;
    return requested;
}

bool protection_is_active(void) { return s_protection_active; }

/*============================================================================
 *  Per-source trigger history
 *==========================================================================*/

void protection_note_source(ProtectionSource src)
{
    /* The enum is the bit index, so this is one shift and one OR.  The
     * bound check is not paranoia: the enum is an index into a 32-bit word,
     * and a src >= 32 would shift off the end and mark an unrelated source
     * (or silently vanish) rather than fail visibly. */
    if ((unsigned)src < (unsigned)PROT_SRC_COUNT) {
        s_source_fired |= (uint32_t)1U << (uint32_t)src;
    }
}

uint32_t protection_source_fired_mask(void)
{
    return s_source_fired;
}

void protection_clear_source_history(void)
{
    s_source_fired = 0U;
}

/*============================================================================
 *  GUI helpers
 *============================================================================*/

const char *protection_get_fault_string(AlertType a)
{
    switch (a) {
        case ALERT_SWR_HIGH:            return "SWR Too High";
        case ALERT_REFLECT_POWER_HIGH:  return "Reflected Power High";
        case ALERT_CURRENT_HIGH:         return "Main Current Too High";
        case ALERT_HARDWARE_FAULT:      return "Hardware Fault";
        case ALERT_OVERDRIVE:           return "RF Input Overdrive";
        case ALERT_FREQ_MISMATCH:       return "Frequency Mismatch";
        case ALERT_NTC_OVERHEAT:        return "Core Temperature High";
        case ALERT_VBIAS_FAULT:         return "PA Bias Fault";
        case ALERT_FAN_FAULT:           return "Fan Fault";
        default:                        return "Unknown Fault";
    }
}
