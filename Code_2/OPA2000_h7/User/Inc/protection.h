/**
 * @file protection.h
 * @brief Protection system header.
 *
 * @details Two layers:
 *   1. Hardware-triggered (ISR context, main prio 0, no FreeRTOS API):
 *        - ADC1 AWD1/AWD2/AWD3 on CH4/CH5/CH6
 *        - COMP1/COMP2
 *        - EXTI12 (HARDPWR_ERR on PB12)
 *      Each handler calls protection_trigger_locked() to atomically:
 *        - release RELAY1 + RELAY2
 *        - drop PWR_ON_SIG (PG7)
 *        - raise RF_IN_PROTECT_CPU_SIG (PB13) -> hardware latch
 *        - clear g_allow_rf, set g_alert_flag, set g_alert_active
 *   2. Software-triggered (Control task, low prio):
 *        - REV2 > 400 W  (always)
 *        - REV2 > 200 W and SWR > 2.0
 *        - FWD1 > 1100 W
 *        - FWD2 > 1050 W
 *        - NTC > 90 C
 *        - VBIAS > config threshold
 *        - ADC value sanity (e.g. IM out of range)
 *
 * The task-side checks run in protection_check() called from the Control
 * task on the 100 ms tick.
 *
 * Severity: each AlertType maps to a severity (alert_severity() in
 * opa2000.c).  P0 (emergency) does the full 7-step hard shutdown and
 * drives PB13; P1 (mild, e.g. thermal) cuts PWR_ON + RF_BIAS so the PA
 * stops self-heating but does NOT latch the RF board protection bus and
 * does NOT clobber g_allow_rf - dismissing a P1 alert returns the rig to
 * keyable immediately.
 */

#ifndef PROTECTION_H
#define PROTECTION_H

#ifdef __cplusplus
extern "C" {
#endif

#include "opa2000.h"
#include "hw_pinmap.h"

/*============================================================================
 *  ISR-side: lock-free direct GPIO shutdown
 *==========================================================================*/

/* Bring the protection subsystem to its power-on state: latch cleared, alert
 * flags down, RF_IN_PROTECT low, buzzer silent.  Call once from main() before
 * the scheduler starts (before any interrupt can reach a handler). */
void protection_init(void);

/* Atomic hardware shutdown.  Called from:
 *   - ADC1 AWD1/2/3 callbacks  (from ADC1_2_IRQHandler, main prio 0)
 *   - COMP1/2 callbacks         (from COMP_IRQHandler, main prio 0)
 *   - EXTI15_10 IRQHandler      (HARDPWR_ERR, main prio 0)
 *   - User ad-hoc triggers from protection.c
 * MUST NOT call any FreeRTOS API. */
void protection_trigger_locked(AlertType cause);

/*============================================================================
 *  Protection sources
 *
 *  Digital  : a discrete fault line, brought in on an EXTI pin, handled
 *             inside the EXTI ISR.  The line is ACTIVE HIGH - the handler
 *             re-reads the pin before acting so a spurious edge cannot
 *             shut the amplifier down.
 *               HARDERR_PWR  (PB12)  power board alarm
 *               (spare)             future RF deck fault lines
 *  Analog   : a conversion result compared against a limit.  Where the
 *             channel is wired to an analog watchdog the comparison is in
 *             hardware and fires the AWD interrupt; the remaining channels
 *             are compared in the DMA / injected-completion interrupt.
 *               AWD2  CH5   IMain   30 A   <- needs a valid IM zero offset
 *               AWD3  CH6   REV1    800 W
 *               DMA          REV2    400 W
 *               DMA          FWD1    1100 W
 *               DMA          FWD2    1050 W
 *               ADC2 inj     NTC     90 C
 *
 *  The MAIN-CURRENT protection (AWD2 on CH5 and COMP2 on the same node)
 *  is the one protection that depends on a measurement rather than on a
 *  fixed limit: both the watchdog threshold and the comparator reference
 *  are "the CC6920's zero offset plus the current limit".  Until the
 *  zero-offset calibration has produced a usable number, both are parked
 *  where they cannot trip - see adc_im_protection_valid() in
 *  adc_calibration.h.  A threshold computed from an unmeasured offset
 *  would sit at the wrong current, which is worse than no threshold.
 *
 *  Nothing else is affected by that calibration: the amplifier can be
 *  keyed and every other protection stays armed while it is pending.
 *============================================================================*/

/* Digital fault lines.  Generic so RF deck signals can be added without
 * touching the shutdown path.  Interrupt context, no FreeRTOS API. */
void protection_hardpwr_err_isr(void);   /* PB12, ACTIVE HIGH         */
void protection_digital_isr(AlertType cause);   /* future EXTI fault lines */

/*==============================================================================
 *  Holding the digital fault lines off until the supply has settled
 *
 *  The power board's fault line (PB12) comes up with the rest of the
 *  supply, so during the ADC's power-on settling window it can still be
 *  asserting, or can pulse once as the rail comes up.  A P0 fault is
 *  expensive - it cuts the HV rail, drops the relays, sounds the 3-tone
 *  buzzer for 10 s and latches a popup - and all of that at every cold
 *  boot, for a supply that is about to come up healthy.
 *
 *  So the lines are masked from power-on until the settling window ends.
 *  The mask is per EXTI LINE, not per NVIC IRQ: PB12 (line 12) and PF15
 *  (line 15, the backup-rail monitor) share EXTI15_10_IRQn, so masking
 *  the IRQ would take the backup monitor down with it.
 *
 *  When the mask comes off, a line that is STILL asserted is a real
 *  fault - the supply had its chance to settle and it did not - and is
 *  latched then, because a line that never fell produces no new edge.
 *
 *  Call order: protection_hold_digital_faults() before the scheduler
 *  starts, protection_arm_digital_faults() when the settling window ends
 *  (adc_stabilisation_tick(), from the ADC task).
 *==========================================================================*/
void protection_hold_digital_faults(void);
void protection_arm_digital_faults(void);
void protection_comp1_isr(void);          /* COMP1 RF input overdrive   */
void protection_comp2_isr(void);          /* COMP2 main over-current     */

/* AWD callbacks are wired into HAL_ADC_LevelOutOfWindowCallback. */
void protection_awd1_cb(void);   /* CH4 RF input  */
void protection_awd2_cb(void);   /* CH5 IMain     */
void protection_awd3_cb(void);   /* CH6 REV1      */

/*============================================================================
 *  Fast analog checks - interrupt context, no FreeRTOS API
 *
 *  Called from the ADC1 DMA-completion interrupt, right after the fresh
 *  conversion results have been published to g_adc_data.  Covers only the
 *  channels that have no analog watchdog behind them.
 *============================================================================*/
void protection_check_adc1_dma(void);

/* Called from the ADC2 injected-completion interrupt, where the core NTC
 * result lands. */
void protection_check_ntc(void);

/*============================================================================
 *  Task-side checks (Control 100 ms tick) - backstop and slow conditions
 *============================================================================*/

/* Run all software protection checks.  Each check may call
 * protection_trigger_locked() directly via the same code path as ISR. */
void protection_check(void);

/* Clear latched fault after the user acknowledges the alert and the
 * underlying condition has been resolved.
 *
 * Non-blocking: it only raises a request.  The 100 ms external-latch reset
 * pulse is executed by the protection task - see the latch-clear section in
 * protection.c for why it cannot run here. */
void protection_clear(void);

/* True when a dismiss has asked for the external latch reset pulse, and
 * clears the request.  Test-and-clear rather than a plain query so a request
 * arriving during the 100 ms pulse is not lost. */
bool protection_take_latch_clear_request(void);

/* Has any fault been latched?  Read-only accessor for GUI. */
bool protection_is_active(void);

/*==============================================================================
 *  Per-source trigger history
 *
 *  WHY THIS IS NOT THE SAME AS g_alert_active
 *  g_alert_active answers "is something latched right now", and one latch
 *  holds a single cause - the FIRST one to arrive.  That is the right
 *  behaviour for the popup, and the wrong thing to debug with: once a
 *  fault has latched, every later source is refused by the re-entry guard,
 *  so the display shows one cause and stays silent about the rest.  A rig
 *  that trips CURRENT_HIGH on every transmission looks identical whether
 *  the comparator found it or the analog watchdog did, and those two have
 *  completely different fixes.
 *
 *  So each source stamps its own bit the moment its comparison passes, and
 *  the stamp survives both the latch and the dismiss.  Nothing is cleared
 *  except by protection_init() at power-on, which makes it a "since boot"
 *  record rather than a live state - the question it answers is "has this
 *  source EVER fired", not "is it firing".
 *
 *  Deliberately stamped BEFORE protection_trigger_locked(), so a source that
 *  fires while another fault is latched, or while allow_protection is 0, is
 *  still recorded.  Those are exactly the cases where the firmware did
 *  nothing and the operator needs to know why.
 *==========================================================================*/
typedef enum {
    PROT_SRC_AWD1 = 0,   /*!< AWD1, RF input overdrive on CH4        */
    PROT_SRC_AWD2,       /*!< AWD2, main current on CH5              */
    PROT_SRC_AWD3,       /*!< AWD3, coupler-1 reverse on CH6         */
    PROT_SRC_COMP1,      /*!< COMP1 comparator, RF overdrive         */
    PROT_SRC_COMP2,      /*!< COMP2 comparator, main over-current    */
    PROT_SRC_HARDERR,    /*!< PB12 power-board fault line           */
    PROT_SRC_NTC,        /*!< core NTC over-temperature             */
    PROT_SRC_VBIAS,      /*!< PA bias out of range                  */
    PROT_SRC_COUPLER,    /*!< REV2 / SWR / FWD1 / FWD2 in software   */
    PROT_SRC_FREQ,       /*!< measured frequency vs band mismatch    */
    PROT_SRC_COUNT
} ProtectionSource;

/* Stamp one source.  ISR-safe: a single-word OR on a volatile, which is
 * atomic on Cortex-M.  Called from interrupt context, so it must stay this
 * cheap. */
void protection_note_source(ProtectionSource src);

/* Bit i set == source i has fired at least once since power-on.
 * PROT_SRC_COUNT <= 32 is a hard requirement - the history is one word. */
uint32_t protection_source_fired_mask(void);

/* Forget the history.  Only protection_init() calls this; the GUI exposes it
 * so a bench session can be cleared without a reboot. */
void protection_clear_source_history(void);

/* Fault description string for GUI display. */
const char *protection_get_fault_string(AlertType a);

/*============================================================================
 *  IM (main current) mute - the HV inrush window
 *
 *  Raising the high-voltage rail charges the supply's bulk capacitors, and
 *  the charging current looks like a short circuit to the current
 *  protection.  AWD2 and COMP2 both watch that node and both would trip -
 *  and a trip on the way UP to standby is nonsense, since the amplifier is
 *  not even keyed yet.
 *
 *  So one flag, tested by both main-current entry points:
 *
 *      power_hv_on()          -> protection_mask_im_high()  (before PWR_ON)
 *      app_monitor_task()     -> protection_im_mask_tick()  (every tick)
 *      AWD2 / COMP2 handlers  -> protection_im_masked()     (read only)
 *
 *  The mute covers ONLY the main-current paths.  RF input overdrive,
 *  coupler reverse, over-temperature and the bias checks stay armed
 *  throughout - they watch different nodes and have nothing to do with the
 *  supply's inrush.
 *==========================================================================*/

/*!< How long the main-current protection stays muted after PWR_ON is
 *   asserted: 100 ms, i.e. one monitor-task tick.
 *
 *   The window is measured from the tick recorded when the mute was armed
 *   and released by protection_im_mask_tick(), so the effective length is
 *   one tick of the monitor task (100 ms) plus whatever it takes that tick
 *   to see the elapsed time - never less than this value.  Long enough for
 *   the capacitor charging transient to have fully decayed, short enough
 *   that a genuine over-current is still caught almost immediately. */
#define IM_MASK_HOLD_MS   100U

/*! Raise the mute and record the tick the window counts from.  Called from
 *  power_hv_on() BEFORE the HV rail is switched on.
 *
 *  Also drives CLEAR_FAULTX_OC low for the duration, so a latch the inrush
 *  set is cleared instead of outliving the event.  It is released again by
 *  protection_im_mask_tick(). */
void protection_mask_im_high(void);

/*! Release the mute once IM_MASK_HOLD_MS has elapsed since it was raised,
 *  and put CLEAR_FAULTX_OC back high.  Called by the monitor ("pump") task
 *  on every tick; that task is the only place the flag is cleared.  No-op
 *  while no mute is armed. */
void protection_im_mask_tick(void);

/*! Is the main-current protection muted right now?  ISR-safe: one volatile
 *  read, no locks and no FreeRTOS calls. */
bool protection_im_masked(void);

/*==============================================================================
 *  Timed clear of the external fault latch
 *
 *  CLEAR_FAULTX_OC (PG1) is open-drain into the latch reset network.
 *  Holding it low is what actually clears the latch; left floating, the
 *  latch stays closed however much the firmware thinks it dismissed the
 *  fault.
 *
 *  The caller cannot run the pulse itself: it has to hold the pin low for a
 *  while, and the places that need it are interrupt handlers (a protection
 *  trip) and command handlers (CAT / SPI6).  Neither may block, and neither
 *  may call a FreeRTOS delay.
 *
 *  So the request is a FLAG and the monitor ("pump") task is the heartbeat
 *  that drives the pin:
 *
 *      caller (ISR or task) -> protection_clear_fault_safe(ms)
 *                                 |  flag + hold time, no blocking
 *                                 v
 *      monitor task           -> protection_fault_clear_service()
 *                                 |  pulls the line low, arms the deadline
 *                                 |  releases it when the time is up
 *
 *  RF_BIAS is NOT touched by this API.  The dismiss path drops the PA bias
 *  as well, because there the PA really did fault; this one is generic and
 *  may be called while the amplifier is keyed, so it only moves the clear
 *  line.  A caller that needs the bias dropped asks for that separately.
 *==========================================================================*/

/*! Request a timed clear of the external fault latch.
 *
 *  ISR-safe and task-safe: two volatile word stores, no lock, no GPIO write
 *  and no FreeRTOS API.  Nothing happens on the pin until the monitor task
 *  next runs, so the caller never blocks.
 *
 *  @param milliseconds How long the clear line should be held low.  0 is
 *         treated as 1 ms - a zero-length pulse would give the latch
 *         network no edge to react to. */
void protection_clear_fault_safe(uint32_t milliseconds);

/*! Drive the fault-latch clear pulse.  MONITOR TASK ONLY - it writes the
 *  GPIO, so it must not be called from an interrupt handler. */
void protection_fault_clear_service(void);

/*! Hold the clear line low from the caller's own context.  Task context.
 *
 *  The counterpart of protection_clear_fault_safe() for callers that can
 *  afford to hold the line themselves and need it low NOW - the dismiss
 *  path, which drops the PA bias for the same window.  The line is only
 *  released when the last holder lets go, so a timed pulse running at the
 *  same time is not cut short. */
void protection_fault_clear_hold(void);

/*! Release a hold taken by protection_fault_clear_hold().  Task context. */
void protection_fault_clear_release(void);

/*! How much longer the active timed pulse runs, in milliseconds.  0 means
 *  "nothing running".  The monitor task uses it to release the line when
 *  the caller asked rather than a whole task tick later. */
uint32_t protection_fault_clear_remaining_ms(void);

#ifdef __cplusplus
}
#endif

#endif /* PROTECTION_H */