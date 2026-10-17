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
 *     ADC1 DMA complete   REV2 / SWR / FWD1 / FWD2 config limits
 *     ADC2 injected done  NTC   g_config.band_ntc_max_tenth_c
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
 * protection_check() runs the same comparisons on the 100 ms task tick as
 * a backstop for anything the fast paths miss.
 */

#include "protection.h"
#include "power_control.h"
#include "buzzer.h"
#include "adc_calibration.h"
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

/*============================================================================
 *  Initialisation
 *==========================================================================*/

void protection_init(void)
{
    s_protection_active = false;
    s_last_fault        = ALERT_NONE;
    s_last_severity     = ALERT_SEV_NONE;
    g_alert_active      = ALERT_NONE;
    g_alert_flag        = false;
    g_alert_severity    = ALERT_SEV_NONE;
    /* Idle state: no fault latched, so the RF board must be told "no fault".
     * Doing this once here also means a protection_clear() before any fault
     * cannot leave PB13 in whatever state the pin came up in. */
    RF_IN_PROTECT_CLEAR();
    buzzer_stop();
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
    /* Prevent re-entry: only first trigger wins, until cleared. */
    if (s_protection_active) {
        return;
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
     * hardware stay armed. */
    power_ptt_out_set(false);
    ptt_state = PTT_STATE_IDLE;

    /* 5b. start the P1 warning tone.  Hard-capped at 10 s. */
    if (s_last_severity == ALERT_SEV_P1) {
        buzzer_start_p1();
    }

    /* 6. publish to cross-task state (volatiles already declared
     *    volatile in opa2000.c; reads on task side are atomic on M7). */
    g_alert_active   = cause;
    g_alert_severity = s_last_severity;
    g_alert_flag     = true;
    /* 7. P0 only: block new PTT until the operator dismisses.  P1 leaves
     *    g_allow_rf alone so the same CC6920 zero-offset calibration
     *    continues to gate keying as normal.  g_tx_active is cleared in
     *    both cases so the GUI's status banner immediately reflects
     *    "not transmitting". */
    if (s_last_severity == ALERT_SEV_P0) {
        g_allow_rf = false;
    }
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

void protection_digital_isr(AlertType cause)
{
    protection_trigger_locked(cause);
}

void protection_hardpwr_err_isr(void)
{
    /* HARDERR_PWR (PB12) is ACTIVE HIGH: only act while the line really is
     * asserted, so a spurious edge cannot shut the amplifier down. */
    if (HAL_GPIO_ReadPin(HARDPWR_ERR_GPIO_Port, HARDPWR_ERR_Pin)
        == GPIO_PIN_SET) {
        protection_trigger_locked(ALERT_HARDWARE_FAULT);
    }
}

void protection_comp1_isr(void)
{
    /* COMP1 = RF input overdrive / over-power (PB2 / ADC1_INP4). */
    protection_trigger_locked(ALERT_OVERDRIVE);
}

void protection_comp2_isr(void)
{
    /* COMP2 = main current over-current (PE9 / ADC1_INP5). */
    protection_trigger_locked(ALERT_CURRENT_HIGH);
}

/*============================================================================
 *  Analog sources with a hardware watchdog - ADC IRQ context
 *==========================================================================*/

void protection_awd1_cb(void)
{
    /* AWD1 -> CH4 RF input power over band-dependent threshold.
     * ioc default AWD1HighThreshold = 32767 -> set to band threshold by
     * adc_awd_set_rf_in_mW() before TX starts. */
    protection_trigger_locked(ALERT_OVERDRIVE);
}

void protection_awd2_cb(void)
{
    /* AWD2 -> CH5 main current, the hardware line behind
     * g_config.band_im_max_mA (programmed by boot_apply_thresholds()). */
    protection_trigger_locked(ALERT_CURRENT_HIGH);
}

void protection_awd3_cb(void)
{
    /* AWD3 -> CH6 coupler 1 reverse, the hardware last line behind
     * g_config.band_rev1_max_w (programmed by boot_apply_thresholds()). */
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
    /* Already latched, or nothing is keyed: no RF to protect. */
    if (s_protection_active || !g_tx_active) {
        return;
    }

    /* Snapshot the operator-configured limits once. */
    protect_limits_t lim;
    protect_limits_get(&lim);

    /* Coupler 2 reverse, the reading the operator actually sees. */
    if (g_adc_data.rev2_mW > lim.rev2_max_mW) {
        protection_trigger_locked(ALERT_REFLECT_POWER_HIGH);
        return;
    }
    /* Reverse power above the configured floor AND an SWR above the
     * configured ceiling: a high SWR at low power is just coupler noise. */
    if (g_adc_data.rev2_mW > lim.rev2_swr_mW &&
        g_adc_data.swr_x100 > lim.swr_max_x100) {
        protection_trigger_locked(ALERT_SWR_HIGH);
        return;
    }
    /* Coupler 1 forward. */
    if (g_adc_data.fwd1_mW > lim.fwd1_max_mW) {
        protection_trigger_locked(ALERT_REFLECT_POWER_HIGH);
        return;
    }
    /* Coupler 2 forward. */
    if (g_adc_data.fwd2_mW > lim.fwd2_max_mW) {
        protection_trigger_locked(ALERT_REFLECT_POWER_HIGH);
        return;
    }
    /* PA bias (INP14).  Lives here rather than in protection_check() so the
     * response lands within one ADC1 DMA completion instead of waiting for
     * the Control task tick - design spec section 7.5.1 requires every
     * channel without an analog watchdog to be compared in the DMA
     * interrupt.  VBIAS shares ADC1's regular group with the coupler
     * channels, so the fresh g_adc_data.vbias_mV is already published by
     * the time this runs.  The g_tx_active gate above covers "bias should
     * be up", which is exactly the condition under which a reading means
     * anything. */
    if (g_adc_data.vbias_mV > g_config.band_vbias_max_mV) {
        protection_trigger_locked(ALERT_VBIAS_FAULT);
    }
}

void protection_check_ntc(void)
{
    if (s_protection_active) {
        return;
    }
    /* Over-temperature latches whether or not we are keyed: the PA is still
     * dissipating and the condition will not clear on its own. */
    protect_limits_t lim;
    protect_limits_get(&lim);
    if (g_adc_data.ntc_core_tenth_c > lim.ntc_max_tenth_c) {
        protection_trigger_locked(ALERT_NTC_OVERHEAT);
    }
}

/*============================================================================
 *  Task-side software protection (100 ms tick) - backstop
 *==========================================================================*/

void protection_check(void)
{
    /* Don't re-trigger if already latched. */
    if (s_protection_active) {
        return;
    }

    /* Over-temperature applies whether or not we are keyed. */
    protect_limits_t lim;
    protect_limits_get(&lim);
    if (g_adc_data.ntc_core_tenth_c > lim.ntc_max_tenth_c) {
        protection_trigger_locked(ALERT_NTC_OVERHEAT);
        return;
    }

    /* The coupler limits only matter while the amplifier passes RF. */
    if (!g_tx_active) {
        return;
    }

    protection_check_adc1_dma();

    /* VBIAS out of range with the bias engaged. */
    if (g_adc_data.vbias_mV > g_config.band_vbias_max_mV) {
        protection_trigger_locked(ALERT_VBIAS_FAULT);
    }
}

/*============================================================================
 *  Ack / clear (called from GUI or PC command after user ack)
 *============================================================================*/

void protection_clear(void)
{
    /* Capture the severity BEFORE we clobber s_last_severity, so the
     * dismiss-time branch can decide whether to re-arm g_allow_rf. */
    AlertSeverity sev = s_last_severity;

    s_protection_active = false;
    s_last_fault        = ALERT_NONE;
    s_last_severity     = ALERT_SEV_NONE;
    g_alert_active      = ALERT_NONE;
    g_alert_flag        = false;
    g_alert_severity    = ALERT_SEV_NONE;
    /* Only P0 driven the hardware fault latch; for P1 this is a no-op
     * (the pin is still in its idle state). */
    if (sev == ALERT_SEV_P0) {
        RF_IN_PROTECT_CLEAR();
    }
    buzzer_stop();

    /* 主动通知 LPUART1 告警已解除。cause / sev 在 OFF 事件里忽略，
     * 但要传入 s_last_fault 的值让 PC 端知道刚解的是哪个。 */
    comm_post_alert_evt(s_last_fault, ALERT_SEV_NONE, false);
    /* Re-arm the RF path:
     *   P0  -> g_allow_rf had been cleared by the trigger; restore it.
     *         The CC6920 zero offset is unaffected by the fault, so the
     *         original calibration is still good.
     *   P1  -> g_allow_rf was never touched by the trigger.  Nothing to
     *         restore; the operator can key immediately on the next PTT
     *         press (g_ptt_ready is also already true).
     *   P0/NONE both also need g_allow_rf = true, because the CC6920
     *     zero offset is the only reason it can be low at this point
     *     and that was set at boot.  Conservative: always re-arm. */
    g_allow_rf = true;
}

bool protection_is_active(void) { return s_protection_active; }

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