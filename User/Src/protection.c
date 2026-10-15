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

/*============================================================================
 *  Initialisation
 *==========================================================================*/

void protection_init(void)
{
    s_protection_active = false;
    s_last_fault        = ALERT_NONE;
    g_alert_active      = ALERT_NONE;
    g_alert_flag        = false;
    /* Idle state: no fault latched, so the RF board must be told "no fault".
     * Doing this once here also means a protection_clear() before any fault
     * cannot leave PB13 in whatever state the pin came up in. */
    RF_IN_PROTECT_CLEAR();
    buzzer_stop();
}

/*============================================================================
 *  Atomic shutdown (ISR-safe, no FreeRTOS API)
 *==========================================================================*/

void protection_trigger_locked(AlertType cause)
{
    /* Prevent re-entry: only first trigger wins, until cleared. */
    if (s_protection_active) {
        return;
    }
    s_protection_active = true;
    s_last_fault        = cause;

    /* 1. raise hardware fault latch FIRST (PB13 -> D7 -> RF_IN_PROTECT_MIXED)
     *    so the RF board starts its own protection and latches the hardware
     *    bus at the earliest possible moment - every other step below is
     *    local to this MCU and can afford to follow. */
    RF_IN_PROTECT_RAISE();
    /* 2. drop both RELAYs and the keying feedback immediately */
    RELAY1_OFF();
    RELAY2_OFF();
    power_ptt_out_set(false);
    ptt_state = PTT_STATE_IDLE;
    /* 3. cut high voltage */
    PWR_ON_DISABLE();
    /* 4. cut PA bias */
    RF_BIAS_DISABLE();

    /* 5. start the 3-tone warning; it loops until protection_clear().
     *    Done here rather than in the task so the alarm is audible even if
     *    the fault happened while the scheduler was busy. */
    buzzer_start();

    /* 5a. Abandon any HV sequencing that was in flight.  This path drops
     *     PWR_ON / RF_BIAS directly with zero delay - deliberately not via
     *     power_hv_off(), whose 10 ms bias-before-rail gap would be the
     *     wrong thing to do to a fault.  The pending stage therefore has
     *     to be cleared, or power_rail_step() would later re-assert a rail
     *     that protection just cut.
     *
     *     g_hv_on is left alone for the same reason g_psu_on was: the front
     *     panel should keep reading "ready" so the operator is told to power
     *     cycle rather than silently powered off.  What actually blocks RF is
     *     g_allow_rf / g_alert_flag below. */
    power_rail_abort();

    /* 6. publish to cross-task state (volatiles already declared
     *    volatile in opa2000.c; reads on task side are atomic on M7). */
    g_alert_active = cause;
    g_alert_flag   = true;
    /* 7. block new PTT until IM is re-calibrated or fault cleared */
    g_allow_rf     = false;
    g_tx_active    = false;
    /* NOTE: g_hv_on and g_psu_on are deliberately left alone.  A fault
     * drops the relays, PWR_ON and the bias, but the front panel still reads
     * "ready" so the operator is told to power-cycle rather than silently
     * powered off.  g_allow_rf above is what actually blocks RF. */
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
    s_protection_active = false;
    s_last_fault        = ALERT_NONE;
    g_alert_active      = ALERT_NONE;
    g_alert_flag        = false;
    RF_IN_PROTECT_CLEAR();
    buzzer_stop();
    /* Note: g_allow_rf stays false until user re-runs the IM zero-offset
     * calibration (adc_im_zero_offset_tick()).  That is intentional -
     * the operator must explicitly re-arm the RF path. */
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
        default:                        return "Unknown Fault";
    }
}